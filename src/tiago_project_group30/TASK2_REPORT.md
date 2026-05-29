# Task 2 — Autonomous Mobile Robotics Exam, Group 30

## Obiettivo

Il Task 2 inizia con il robot spawnato in una posizione casuale all'interno di una mappa già nota (costruita nel Task 1 con explore_lite). Il robot non conosce la propria posizione iniziale e deve:

1. Portare il braccio in posizione sicura (HOME).
2. Localizzarsi sulla mappa salvata usando AMCL.
3. Esplorare la mappa con waypoint casuali finché non rileva entrambi i marker ArUco (PICK e PLACE).
4. Navigare verso la posa di approccio del marker PICK.
5. Navigare verso la posa di approccio del marker PLACE.

---

## Architettura software

Il nodo principale `Task2Manager` (in `task2_manager.py`) istanzia sei componenti, ognuno nel proprio file:

| Componente | File | Responsabilità |
|---|---|---|
| `ArmController` | `tiago_arm.py` | MoveIt2 arm + head tilt |
| `NavClient` | `task2_nav.py` | Client Nav2 `NavigateToPose`, polling |
| `AmclLocalizer` | `task2_amcl.py` | Localizzazione AMCL + spin + pulizia costmap |
| `ArucoTracker` | `task2_aruco.py` | Rilevamento ArUco + broadcast posa di approccio |
| `CostmapSampler` | `task2_costmap.py` | Campionamento waypoint dalla global costmap |
| `StateMachine` | `task2_state_machine.py` | Macchina a stati non bloccante |

La `StateMachine` gira su un thread dedicato (daemon). Il `MultiThreadedExecutor` (4 thread) gira sul thread principale e mantiene attivi subscription, service client, e action future. Le due strutture sono sincronizzate tramite un `threading.Event` (`executor_ready`): la macchina a stati aspetta che l'executor sia in spin prima di procedere, poi attende 10 secondi aggiuntivi per permettere a Nav2, AMCL e agli altri nodi del sistema di completare l'avvio.

Il pattern è quello del Lab 4: tutte le operazioni asincrone (MoveIt2, Nav2, spin) vengono avviate con chiamate non bloccanti e i risultati vengono letti ogni tick (a ~20 Hz) tramite metodi `update_flags()` dedicati, senza callback.

---

## Macchina a stati

### State 0 — Tuck arm to HOME

Il braccio di Tiago viene portato in una configurazione HOME predefinita tramite MoveIt2 (`arm_torso` group, planner `RRTConnectkConfigDefault`). Contemporaneamente la testa viene inclinata di −0.5 rad (~17° sotto l'orizzonte) tramite un `JointTrajectory` pubblicato direttamente su `/head_controller/joint_trajectory`. L'inclinazione è scelta per tenere i marker ArUco su pareti e cubi nel FOV della camera a distanze di 1–3 m senza perdere il campo orizzontale.

Il braccio esteso sarebbe pericoloso in prossimità di ostacoli e bloccherebbe il FOV del LiDAR e della camera RGB, impedendo sia la navigazione sia il rilevamento ArUco.

- Transizione → **State 1** immediatamente dopo aver inviato il goal MoveIt2.

### State 1 — Wait for arm motion

La macchina a stati legge ogni tick lo stato di MoveIt2 tramite `arm.query_state()`. Lo stato `EXECUTING` fa scattare il flag `motion_started`; quando MoveIt2 torna a `IDLE` scatta `motion_done`.

Se il movimento non è ancora iniziato dopo un timeout di 6 s (e.g. MoveIt2 non trova un piano), si ritorna allo **State 0** per un nuovo tentativo.

- Transizione → **State 2** quando `motion_done` è True.

### State 2 — AMCL global localization

Questo stato gestisce il problema dello spawn casuale: il robot non sa dove si trova nella mappa. È strutturato in quattro sotto-stati.

**Sub-state 0** — `request_global_localization()`
Viene chiamato il servizio `/reinitialize_global_localization` (tipo `std_srvs/Empty`). AMCL redistribuisce le sue particelle uniformemente sull'intera mappa, abbandonando qualsiasi stima precedente.

La stessa operazione si può fare manualmente da terminale con `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`, ma nel codice viene usato un service client Python ROS 2 (`node.create_client` + `call_async`). Il motivo è che il comando shell andrebbe eseguito come subprocess esterno, non avrebbe accesso agli stati interni del nodo, e non potrebbe essere integrato nel flusso della macchina a stati. Il client Python invece è asincrono, gira nell'executor del nodo, e la chiamata avviene nello stesso contesto del resto del codice (stesso nodo, stessa gestione degli errori, stesso logging).

**Sub-state 1** — Clear local costmap
Prima di far girare il robot si svuota la local costmap con il servizio `/local_costmap/clear_entirely_local_costmap`. I marker di ostacolo accumulati prima della reinizializzazione potrebbero far scattare il check "Collision Ahead" del behavior server di Nav2 e abortire lo spin appena iniziato.

**Sub-state 2** — Spin 2π
Viene inviato un goal al Nav2 `/spin` action server: `target_yaw = 2π` (rotazione completa su se stesso). Questo espone il LiDAR a tutte le direzioni e fornisce ad AMCL abbastanza scan per convergere.

**Sub-state 3** — Wait for convergence
Ogni tick viene letto `/amcl_pose` (sottoscrizione). La convergenza è definita come:

```
cov_xx  < 0.10 m²   (σ_x  ≈ 0.32 m)
cov_yy  < 0.10 m²   (σ_y  ≈ 0.32 m)
cov_yaw < 0.07 rad² (σ_θ  ≈ 0.26 rad ≈ 15°)
```

Se AMCL converge prima che lo spin termini, lo spin viene cancellato e si procede. Se lo spin termina senza convergenza, si torna al sub-state 1 (pulizia costmap + nuovo spin). Dopo 3 spin senza convergenza si procede comunque (AMCL è solitamente già abbastanza vicino alla realtà entro quei ~30 s).

Al momento della convergenza, `update_amcl_flags()` svuota automaticamente la **global costmap** (`/global_costmap/clear_entirely_global_costmap`). Questo rimuove i "phantom obstacle" che l'obstacle_layer ha accumulato mentre AMCL stava riportando una posa sbagliata. Lo strato statico (la mappa salvata) non è toccato.

- Transizione → **State 3**.

### State 3 — Random search

Il robot esplora la mappa navigando verso waypoint casuali. A ogni waypoint esegue una rotazione completa (spin 2π) per scandagliare l'ambiente con la camera. Lo stato esce quando entrambe le pose di approccio ArUco (`pick_approach_pose` e `place_approach_pose`) sono state latched.

Internamente lo stato ha tre fasi (`search_phase`):

- **`'nav'`** — un goal NavigateToPose è in volo. Quando il goal termina con successo si passa a `'spin'`; se fallisce si passa direttamente a `'sample'`. Un timeout di 25 s cancella il goal e fa passare a `'sample'`.
- **`'spin'`** — viene lanciato un `/spin 2π` per scansionare i 360°. Quando lo spin finisce (o va in timeout dopo 15 s) si passa a `'sample'`.
- **`'sample'`** — viene campionato un nuovo waypoint dalla global costmap (vedi sezione dedicata) e inviato un nuovo goal NavigateToPose.

Quando entrambi i marker sono stati visti, eventuali goal e spin in corso vengono cancellati, i flag di navigazione vengono azzerati, e si chiama `aruco.freeze()` che blocca l'aggiornamento delle pose di approccio: il Nav2 target non deve spostarsi durante l'avvicinamento finale.

- Transizione → **State 4**.

### State 4 — Navigate to PICK approach

Il robot naviga verso la posa di approccio del marker PICK. Il goal viene inviato con `nav.send_goal(x, y, yaw)` dove `(x, y, yaw)` è estratto dalla `pick_approach_pose` latched.

Se il goal fallisce, viene immediatamente ritentato (stessa posa). Se rimane in volo per più di 180 s viene cancellato e ritentato. Il timeout elevato (180 s, alzato da 60 s originali) è necessario perché il BT Nav2 `navigate_with_replanning_and_recovery` può impiegare 10–15 s per ciclo di recovery (ClearLocalCostmap + Spin + Wait + replan); a 60 s si annullavano solo 2–3 cicli, riportando il BT a zero e perdendo tutti i progressi.

- Transizione → **State 5** quando il goal ha status `SUCCEEDED`.

### State 5 — Navigate to PLACE approach

Identico a State 4, ma verso `place_approach_pose`. Stesso timeout di 180 s e stessa logica di retry.

- Transizione → **State 6**.

### State 6 — Done

Il flag `self.finished = True` segnala al loop principale (`task2_manager.py`) di uscire dallo spin e spegnere il nodo.

---

## Navigazione: campionamento dei waypoint

### Sottoscrizione alla global costmap

Il `CostmapSampler` si sottoscrive a `/global_costmap/costmap` (tipo `nav_msgs/OccupancyGrid`) con QoS `TRANSIENT_LOCAL`/`RELIABLE` per ricevere il messaggio latched che Nav2 pubblica all'avvio.

La costmap è una griglia dove ogni cella ha valore:
- **0** — libero
- **1–252** — occupato o inflated
- **253–254** — costmap obstacle (lethal/inscribed)
- **-1** — sconosciuto

### Maschera di raggiungibilità (BFS 4-connessa)

Prima di campionare, viene calcolata una maschera booleana delle celle raggiungibili dal robot tramite una **BFS (Breadth-First Search) 4-connessa** sulle celle libere (valore 0).

Una BFS è un algoritmo di visita a grafo che esplora i nodi per livelli: prima tutti i vicini diretti della cella di partenza, poi i vicini dei vicini, e così via — come cerchi che si espandono sull'acqua. "4-connessa" significa che ogni cella è considerata adiacente solo alle 4 celle che la toccano sui lati (su, giù, sinistra, destra), non in diagonale. L'algoritmo parte dalla cella del robot e visita solo celle con valore 0 (libere), segnando quelle raggiunte in una maschera booleana. Le celle libere non raggiungibili dalla cella del robot (es. stanze separate o artefatti isolati della mappa) non vengono mai visitate.

Questo evita di campionare "isole libere" disconnesse (artefatti SLAM, bordi della bounding box della mappa) che Nav2 non potrebbe raggiungere.

Il punto di partenza della BFS è la cella del robot. Se la cella esatta è inflated (valore > 0, come spesso accade vicino a muri), si cerca il free seed nel vicinato crescente (anelli 1, 2, ..., 5 celle).

Le celle entro `EDGE_MARGIN = 0.6 m` dal bordo della mappa vengono escluse dalla maschera. Il global NavFn planner espande un campo potenziale attorno al goal che si estende di qualche inflation radius; se quell'espansione raggiunge pixel fuori dalla mappa, Nav2 loga `"worldToMap failed"` decine di migliaia di volte per tentativo di piano. 0.6 m è maggiore del raggio di inflazione (0.45 m in `tiago_nav2.yaml`), quindi il planner non raggiunge mai pixel out-of-bounds.

### Campionamento pesato per distanza

Dalla maschera si ottiene la lista di celle libere raggiungibili. Su questi candidati si applica:

1. **Visited rejection**: le celle entro `VISITED_RADIUS = 1.5 m` da qualsiasi waypoint già tentato vengono scartate. Questo impedisce al robot di tornare ripetutamente nello stesso punto già esplorato. Il raggio di 1.5 m è scelto in relazione al range utile di rilevamento ArUco: dalla costante `MAX_DETECTION_DISTANCE = 3.5 m` sappiamo che il robot può rilevare marker fino a quella distanza durante lo spin; nella pratica, al di sopra di ~3 m l'errore PnP è già degradato, quindi il range affidabile è ~3 m. Con VISITED_RADIUS = 1.5 m (metà del range affidabile), si garantisce che ogni nuovo waypoint sia sufficientemente lontano dal precedente da aggiungere copertura visiva nuova, senza lasciare zone scoperte tra un waypoint e l'altro.

2. **Hard cap di distanza**: vengono escluse le celle a distanza > `SOFT_MAX_RANGE = 3.0 m` dal robot. Se non rimane nessun candidato entro 3 m, il cap viene rimosso e si accettano tutte le celle non visitate (comportamento di fallback per aree quasi saturate).

3. **Pesatura esponenziale**: i candidati rimasti vengono pesati con `w = exp(-d / PREFERRED_RANGE)` dove `PREFERRED_RANGE = 2.0 m`. La funzione esponenziale decresce rapidamente: una cella a 2 m (= PREFERRED_RANGE) ha peso `exp(-1) ≈ 0.37`, cioè è circa 37 volte meno probabile di una cella a distanza zero. Una cella a 4 m ha peso `exp(-2) ≈ 0.14`. In pratica i waypoint vicini al robot vengono scelti quasi sempre rispetto a quelli lontani, ma non con certezza assoluta (il campionamento rimane probabilistico).

   Il risultato è un'**esplorazione a cerchi espandenti**: all'inizio della ricerca tutte le celle vicine sono non visitate, quindi il robot tende a esplorare prima il vicinato immediato. Man mano che quelle celle vengono aggiunte a `visited_waypoints` e scartate dal filtro (punto 1), i candidati rimasti sono sempre più distanti e il campionamento si sposta progressivamente verso zone più lontane. Il robot non salta subito ai bordi della mappa, ma espande la ricerca con ordine. Allo stesso tempo, poiché il campionamento è probabilistico (non deterministico), si evitano i pattern fissi che potrebbero lasciare buchi.

4. Il campionamento avviene con `np.random.choice` sulla distribuzione normalizzata dei pesi.

Se `sample_random_xy()` restituisce `None` (tutte le celle visitate o BFS non disponibile), la macchina a stati azzera `visited_waypoints` e riprova. Questo copre sia la saturazione reale (tutti i punti dell'area già visitati) sia i fallimenti transienti di TF.

---

## Localizzazione e allineamento della mappa

### Problema: spawn casuale

Il robot viene spawnato in una posizione casuale della mappa. Il file URDF/launch imposta `use_sim_time=true` e Nav2 carica la mappa salvata nel Task 1 (server `/map`), ma AMCL non sa a priori dove il robot si trova nella mappa.

### Soluzione: /reinitialize_global_localization + spin

Il servizio `/reinitialize_global_localization` (interfaccia `std_srvs/Empty`) comanda ad AMCL di distribuire uniformemente le sue N particelle sull'intera mappa (invece di tenerle concentrate sull'ultima stima). Questa è la procedura standard Nav2 per la localizzazione globale (equivalente a `ros2 service call /reinitialize_global_localization std_srvs/srv/Empty`).

Dopo la ridistribuzione, il robot esegue uno spin completo (2π rad). Man mano che il LiDAR vede l'ambiente da ogni direzione, il filtro a particelle può trovare le zone della mappa che combaciano con i laser scan e concentrare le particelle nel posto giusto.

### Controllo di convergenza

AMCL usa un filtro a particelle: mantiene N copie ipotetiche della posa del robot (le "particelle"), ognuna con una propria posizione e orientazione nella mappa. All'inizio, dopo `/reinitialize_global_localization`, queste particelle sono distribuite uniformemente su tutta la mappa — il robot potrebbe trovarsi ovunque. Man mano che arrivano i laser scan durante lo spin, le particelle che si trovano in posizioni incompatibili con i dati LiDAR ricevono un peso basso e vengono eliminate; quelle compatibili sopravvivono e si moltiplicano. Il risultato è che gradualmente le particelle si concentrano intorno alla posa reale del robot.

La convergenza viene rilevata misurando la **covarianza** della distribuzione delle particelle, cioè quanto sono sparse. La matrice di covarianza 6×6 di `/amcl_pose` viene letta agli elementi `[0]` (varianza in X), `[7]` (varianza in Y) e `[35]` (varianza in yaw). Le soglie scelte sono:

```
cov_xx  < 0.10 m²   (σ_x  ≈ 0.32 m)
cov_yy  < 0.10 m²   (σ_y  ≈ 0.32 m)
cov_yaw < 0.07 rad² (σ_θ  ≈ 0.26 rad ≈ 15°)
```

Queste soglie sono volutamente **allentate**. Il motivo è che il filtro a particelle mantiene sempre un piccolo numero di particelle "random" sparse casualmente sulla mappa, come meccanismo di protezione contro la perdita di localizzazione (se il robot venisse spostato fisicamente, le particelle random permetterebbero di ritrovare la posa). Queste particelle sparse — la **coda** della distribuzione — fanno sì che la covarianza non scenda mai a zero, nemmeno quando la stima è già ottima. Se si scegliesse una soglia molto bassa (es. `< 0.001 m²`) si aspetterebbe indefinitamente perché le particelle random impediscono alla varianza di raggiungere quel valore. Con `< 0.10 m²` si dichiara convergenza non appena la massa principale delle particelle si è raggruppata attorno alla posa reale, ignorando la coda residua.

### Pulizia delle costmap

La localizzazione introduce un problema secondario: **phantom obstacles**. Mentre AMCL riportava la posa sbagliata, l'obstacle_layer della global costmap ha inserito marcatori di ostacolo in posizioni errate (le letture LiDAR erano proiettate nella mappa in base alla posa sbagliata). Questi marcatori non vengono rimossi automaticamente finché il robot non ci passa fisicamente sopra.

- La **local costmap** viene svuotata prima di ogni spin per evitare che i marker stale facciano scattare il check "Collision Ahead" del behavior server di Nav2 (~200 ms nel futuro) e abortiscano lo spin.
- La **global costmap** viene svuotata automaticamente nel momento esatto della convergenza (dentro `update_amcl_flags()`). Questo garantisce che la ricerca dei waypoint non eviti aree che sembrano ostruite solo per via della posa iniziale sbagliata. Lo strato statico (la mappa del Task 1) NON è toccato da questa operazione (è un plugin separato).

---

## Rilevamento ArUco e posa di approccio

### Configurazione aruco_ros

Due istanze del nodo `aruco_single` di aruco_ros vengono avviate con `reference_frame=''`. Questa impostazione fa sì che il nodo pubblichi la posa del marker nel frame della camera ottica (`head_front_camera_rgb_optical_frame`), senza applicare alcuna trasformazione TF. Le trasformazioni (da frame camera a frame mappa) sono eseguite manualmente nel codice nel momento del rilevamento.

Questo approccio è lo stesso adottato nel **Lab 3**: anche lì `aruco_single` era configurato con `reference_frame=''` e la composizione della posa nel frame di riferimento globale veniva fatta nel nodo ROS scritto durante il laboratorio, usando il TF buffer per ottenere la trasformazione `map ← camera` al momento della detections. Il vantaggio rispetto all'usare `reference_frame='map'` direttamente nel nodo aruco_ros è il controllo completo su quando e come avviene la composizione (gate di convergenza AMCL, keep-closest policy, timestamp preciso).

Le subscription sono:
- `/aruco_pick/aruco_single/transform` — marker ID 26 (cubo PICK)
- `/aruco_place/aruco_single/transform` — marker ID 238 (destinazione PLACE)

### Composizione nel frame MAP

Quando arriva un messaggio `TransformStamped` dal nodo ArUco:

1. **Gate di convergenza**: la composizione è abilitata solo dopo che AMCL ha convergito. Prima della convergenza il TF `map ← camera` è costruito su una posa AMCL sbagliata e la posizione risultante nella mappa sarebbe errata (abbiamo osservato sperimentalmente una posa di approccio che atterrava dentro un muro quando la detections arrivava 1.3 s dopo `/reinitialize_global_localization`).

2. **Filtro di distanza**: la distanza camera–marker si calcola direttamente dalla norma del vettore di traslazione del messaggio ArUco (che è già in frame camera). Rilevamenti oltre `MAX_DETECTION_DISTANCE = 3.5 m` vengono scartati. A quella distanza un marker di 25 cm occupa ~45 pixel su un'immagine 640×480 e l'errore di stima PnP è nell'ordine di 30 cm, insufficiente per posizionare accuratamente la posa di approccio.

3. **Keep-closest policy**: se il marker è già stato visto in precedenza, il nuovo rilevamento sovrascrive il precedente solo se la distanza camera–marker è **inferiore** a quella del rilevamento salvato. Questo garantisce che la stima finale usi l'osservazione più accurata (quella fatta più vicino al marker).

4. **Composizione temporale**: il TF `map ← camera` viene cercato al timestamp esatto del messaggio ArUco (`msg.header.stamp`). Questo congela il marker alla sua posizione reale nel mondo: ricomporre in seguito con un TF corrente moltiplicherebbe un TF recente (con il robot in una posizione diversa) per una posa vecchia del marker nella camera, producendo un risultato che si sposterebbe nella mappa ad ogni movimento del robot.

La composizione avviene con PyKDL: `marker_in_map = frame_map_cam * aruco_in_cam`.

### Calcolo della posa di approccio

Un timer a 5 Hz chiama `publish_approach_frames()`. Ogni volta che `pick_marker_in_map` (o `place_marker_in_map`) è disponibile, viene calcolata la posa di approccio:

1. Si estrae la **direzione Z del marker in frame map** (asse Z del marker che punta verso la camera, convenzione aruco_ros): `marker_z = marker_in_map.M * Vector(0, 0, 1)`.

2. Si proietta questa direzione sul piano orizzontale XY, normalizzando il componente 2D. Questo rimuove qualsiasi componente verticale dovuta all'inclinazione della camera o del marker.

3. La **posizione di approccio** è il punto nella mappa a `APPROACH_DISTANCE = 0.55 m` dal marker lungo questa direzione: `(marker.x + 0.55 * dx, marker.y + 0.55 * dy)`.

4. Il **yaw** è calcolato come `atan2(marker.y - approach.y, marker.x - approach.x)`, cioè il robot guarda verso il marker quando arriva alla posa di approccio.

5. L'orientazione viene costruita come `Rotation.RPY(0, 0, yaw)` — roll e pitch zero. Questo è necessario per Nav2 2D (che usa solo il yaw) e produce frame visivamente corretti in RViz.

La posa viene broadcasted come TF (`aruco_pick_approach` / `aruco_place_approach`) e latched come `PoseStamped` per il Nav2 goal.

### Freeze sulla transizione State 3 → State 4

Finché il robot è in State 3 (ricerca), la posa di approccio viene continuamente aggiornata con le nuove osservazioni (keep-closest). Al momento della transizione a State 4 viene chiamato `aruco.freeze()` che imposta `_refresh_approach_poses = False`. Da quel punto il goal Nav2 non "insegue" più una stima in movimento.

Questo meccanismo di freeze **non era presente nel Lab 3**. Nel Lab 3 c'era un singolo marker già noto e il robot si avvicinava direttamente; il goal rimaneva stabile perché non ci fosse ricerca attiva. Nel Task 2, invece, la ricerca continua finché entrambi i marker non sono visti, e senza il freeze la posa di approccio verrebbe aggiornata ad ogni nuova detections anche mentre Nav2 sta già navigando verso di essa: il goal cambierebbe continuamente, causando re-planning ad ogni aggiornamento. Il freeze blocca l'aggiornamento nel momento esatto della transizione, garantendo un target stabile per tutta la fase di avvicinamento.

---

## Parametri chiave

| Parametro | Valore | Motivazione |
|---|---|---|
| `APPROACH_DISTANCE` | 0.55 m | Distanza di stop davanti al marker |
| `MAX_DETECTION_DISTANCE` | 3.5 m | Soglia accuratezza PnP ArUco |
| `VISITED_RADIUS` | 1.5 m | Memoria waypoint: no ri-campionamento in zone già scansionate |
| `PREFERRED_RANGE` | 2.0 m | Scala del peso esponenziale |
| `SOFT_MAX_RANGE` | 3.0 m | Cap distanza per campionamento |
| `EDGE_MARGIN` | 0.6 m | Margine dai bordi mappa (> inflation radius 0.45 m) |
| `SEARCH_NAV_TIMEOUT` | 25 s | Timeout singolo waypoint di ricerca |
| `SEARCH_SPIN_TIMEOUT` | 15 s | Timeout spin a waypoint |
| `APPROACH_NAV_TIMEOUT` | 180 s | Timeout navigazione verso PICK/PLACE |
| AMCL cov_xx/yy | < 0.10 m² | Soglia convergenza posizione |
| AMCL cov_yaw | < 0.07 rad² | Soglia convergenza orientazione |
