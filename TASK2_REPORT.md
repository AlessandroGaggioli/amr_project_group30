# Task 2 — Autonomous Mobile Robotics Exam, Group 30

## 1. Obiettivo

Il Task 2 parte con il robot Tiago generato (spawn) in una posizione **casuale e ignota** all'interno di una mappa già nota, costruita nel Task 1. Il robot non conosce la propria posa iniziale e deve, in completa autonomia:

1. Portare il braccio in una configurazione sicura (HOME) e inclinare la testa.
2. Localizzarsi sulla mappa salvata usando AMCL (localizzazione globale).
3. Esplorare l'ambiente con waypoint casuali finché non rileva **entrambi** i marker ArUco a parete (il marker PICK, ID 26, e il marker PLACE, ID 238).
4. Navigare verso la posa di approccio del marker PICK.
5. Navigare verso la posa di approccio del marker PLACE.

L'intero comportamento è realizzato come una **macchina a stati non bloccante** (pattern del Lab 4: avvio asincrono delle operazioni + polling dei risultati a ogni tick), suddivisa in componenti software a responsabilità singola.

---

## 2. Descrizione generale dello svolgimento

Il task viene affrontato componendo cinque sottosistemi indipendenti — braccio/testa, navigazione, localizzazione AMCL, rilevamento ArUco, campionamento dei waypoint — coordinati da una macchina a stati centrale. La filosofia di fondo è che **nessuna operazione blocca il thread della logica**: ogni azione lunga (un piano MoveIt2, un goal Nav2, uno spin, una chiamata di servizio) viene lanciata in modo asincrono, e a ogni iterazione del loop (~20 Hz) la macchina a stati legge dei *flag* aggiornati da metodi `update_flags()` dedicati per decidere se avanzare, riprovare o gestire un timeout.

Il flusso logico è strettamente sequenziale (Stato 0 → 6), ma due stati hanno una struttura interna più ricca:

- lo **Stato 2** (localizzazione AMCL) è organizzato in quattro **sotto-stati** (scatter delle particelle → pulizia costmap → spin → attesa convergenza);
- lo **Stato 3** (ricerca) è una mini macchina a stati a tre **fasi** (`sample` → `nav` → `pan`) che si ripete finché entrambi i marker non sono stati visti.

I problemi affrontati sono tipici dello scenario "spawn casuale + mappa nota": la convergenza di AMCL partendo da incertezza totale, la pulizia degli ostacoli fantasma generati mentre la posa era ancora sbagliata, l'esplorazione efficiente senza tornare sempre sugli stessi punti, e il calcolo di pose di approccio stabili a partire da osservazioni ArUco rumorose.

---

## 3. Architettura software

Il nodo principale `Task2Manager` (`task2_manager.py`) istanzia cinque componenti, ognuno nel proprio file, più la macchina a stati:

| Componente | File | Responsabilità |
|---|---|---|
| `ArmController` | `tiago_arm.py` | Braccio `arm_torso` via MoveIt2 + tilt/pan della testa via `JointTrajectory` |
| `NavClient` | `task2_nav.py` | Client Nav2 `NavigateToPose` (polling) + comandi diretti su `/nav_vel` |
| `AmclLocalizer` | `task2_amcl.py` | Localizzazione globale AMCL + spin Nav2 + pulizia costmap |
| `ArucoTracker` | `task2_aruco.py` | Rilevamento marker a parete + calcolo/broadcast pose di approccio |
| `CostmapSampler` | `task2_costmap.py` | Campionamento waypoint dalla global costmap |
| `StateMachine` | `task2_state_machine.py` | Macchina a stati non bloccante (eredita `CommonStates`) |

Gli stati condivisi (0–6) sono implementati in `common_states.py` (classe `CommonStates`), riutilizzata identicamente dal Task 3; gli helper di basso livello (gestione goal nav, step-runner per braccio/gripper/attach, head scan, lettura posa robot) sono in `state_helpers.py` (classe `StateRunners`). Le funzioni KDL/quaternioni sono in `task2_kdl_helpers.py`. Tutte le costanti sono centralizzate in `constants.py`.

### Modello di concorrenza (`task_runner.py`)

`run_task()` realizza il pattern del Lab 4:

1. crea un `threading.Event` (`executor_ready`);
2. avvia `state_machine.run()` su un **thread daemon** dedicato;
3. crea un `MultiThreadedExecutor` con **4 thread**, ci aggiunge il nodo, e segnala `executor_ready.set()`;
4. fa girare `executor.spin_once()` sul thread principale finché il task non è finito.

La logica vive quindi su un thread separato da quello che processa subscription, service client e action future. La macchina a stati attende `executor_ready` e poi dorme **10 s** aggiuntivi per dare tempo a Nav2/AMCL/MoveIt di completare l'avvio (oltre ai 15 s di ritardo con cui il launch file fa partire il manager).

### Pattern di polling

Tutte le operazioni asincrone vengono avviate con chiamate non bloccanti (es. `send_goal_async`, `move_to_configuration`, `call_async`). A ogni tick il `run()` chiama in sequenza:

```python
self.arm.update_flags()
self.nav.update_flags()
self.amcl.update_spin_flags()
self.amcl.update_amcl_flags()
```

Questi metodi leggono i future / lo stato di MoveIt e aggiornano *latch* one-shot (`motion_done`, `goal_succeeded`, `spin_done`, `converged`, …). Gli stati leggono solo questi flag, senza usare callback per la logica: il vantaggio è che il controllo (cancellazione, retry, timeout) resta interamente nel thread della macchina a stati.

---

## 4. Mappa degli stati (flusso di lavoro)

```
        ┌─────────────────────────────────────────────────────────────┐
        │                                                               │
   [0] Tuck arm ──▶ [1] Wait arm ──▶ [2] AMCL localization             │
   HOME + tilt       (timeout→0)      ├ 2a reinitialize_global_loc      │
                                      ├ 2b clear local costmap          │
                                      ├ 2c spin 2π                      │
                                      └ 2d wait convergence ─(retry)────┘
                                              │
                                              ▼
                              [3] Random search  ◀──────────┐
                              ┌ sample (waypoint costmap)    │ (loop finché
                              ├ nav   (NavigateToPose)       │  entrambi i
                              └ pan   (head sweep ±0.6 rad)──┘  marker visti)
                                              │
                                  entrambi i marker latched
                                  + aruco.freeze()
                                              ▼
                            [4] Nav → PICK approach (retry/timeout 180s)
                                              │ SUCCEEDED
                                              ▼
                            [5] Nav → PLACE approach (retry/timeout 180s)
                                              │ SUCCEEDED
                                              ▼
                                       [6] Done → shutdown
```

Le transizioni sono guidate dai flag aggiornati ogni tick. Le frecce di retry interne (es. `2d → 2b`, timeout in `3.nav → 3.sample`, fallimento in `4 → 4`) garantiscono robustezza rispetto a fallimenti transitori di pianificazione, convergenza o navigazione.

---

## 5. Analisi dettagliata di ogni stato

### State 0 — Tuck arm to HOME (`state_0_arm_tuck`)

Il braccio viene portato alla configurazione `HOME_JOINT_POSITIONS = [0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]` (8 giunti: `torso_lift_joint` + `arm_1..7_joint`) tramite MoveIt2 sul gruppo `arm_torso`, con planner `RRTConnectkConfigDefault`. Si registra il tempo di invio (`self._send_time`) e si passa subito allo Stato 1.

**Perché:** il braccio nella posa di spawn è esteso e ingombrante. Tenerlo esteso (a) è pericoloso vicino agli ostacoli, (b) ostruisce il FOV del LiDAR e della camera RGB, impedendo sia la navigazione sicura sia il rilevamento ArUco. La configurazione HOME è raccolta e compatta.

> Nota: in questo stato `move_to_home` è chiamato con `tilt_head=False`; la testa viene poi inclinata esplicitamente a −0.5 rad nello Stato 3 (e a ogni waypoint), in modo da tenere i marker a parete nel campo visivo durante la ricerca.

### State 1 — Wait for arm motion (`state_1_wait_arm`)

Ogni tick si legge lo stato di MoveIt2 via `arm.update_flags()`, che gestisce due latch:
- `motion_started` quando MoveIt2 entra in `EXECUTING`;
- `motion_done` quando MoveIt2 torna in `IDLE` dopo aver eseguito.

Lo stato applica **due guardie di timeout indipendenti**:
- `planning_timeout` (6 s): il goal è stato inviato ma non è mai partito (`not motion_started`) → probabile fallimento di pianificazione o MoveIt non ancora pronto → si torna allo Stato 0 e si riprova.
- `execution_timeout` (30 s): il moto è partito ma si è bloccato a metà → si torna allo Stato 0.

Quando `motion_done` è vero, si passa allo Stato 2.

**Perché due timeout:** distinguere "non è mai partito" da "partito ma bloccato" evita sia di attendere all'infinito su `motion_done` sia di ripianificare inutilmente quando il moto è semplicemente lungo.

### State 2 — AMCL global localization (`state_2_amcl_localization`)

Risolve il problema dello spawn casuale tramite quattro sotto-stati gestiti dal campo `amcl.loc_substate`.

**Sub-state 2a — `request_global_localization()`**
Si chiama il servizio `/reinitialize_global_localization` (`std_srvs/Empty`) tramite un service client Python asincrono. AMCL ridistribuisce le sue particelle **uniformemente** su tutta la mappa, abbandonando ogni stima precedente. Dopo la chiamata si attendono 2 s perché AMCL ridistribuisca le particelle, poi si passa a 2b.

*Perché un service client Python e non il comando shell equivalente:* il client gira nello stesso executor del nodo, è asincrono, condivide logging e gestione errori, e si integra nel flusso della macchina a stati (un subprocess esterno non avrebbe accesso allo stato interno del nodo).

**Sub-state 2b — Clear local costmap**
Si svuota la local costmap con `/local_costmap/clear_entirely_local_costmap` e si attende 0.5 s. I marcatori di ostacolo accumulati prima della reinizializzazione potrebbero far scattare il check "Collision Ahead" del behavior server e abortire lo spin appena iniziato.

**Sub-state 2c — Spin 2π**
Si incrementa `localization_spin_count` e si invia un goal alla action Nav2 `/spin` con `target_yaw = 2π` e `time_allowance = 30 s`. La rotazione completa espone il LiDAR a tutte le direzioni, fornendo ad AMCL scan sufficienti per convergere.

**Sub-state 2d — Wait for convergence**
Ogni tick `update_amcl_flags()` legge la covarianza da `/amcl_pose` agli indici `[0]` (var. X), `[7]` (var. Y), `[35]` (var. yaw). La convergenza è dichiarata quando:

```
cov_xx  < 0.10 m²
cov_yy  < 0.10 m²
cov_yaw < 0.07 rad²
```

Logica di transizione:
- **se converge** (anche prima della fine dello spin): si cancella lo spin attivo e si va allo Stato 3 (la convergenza ha priorità sul completamento dello spin);
- **se lo spin finisce senza convergenza**: se sono già stati fatti ≥ 3 spin si procede comunque allo Stato 3 (AMCL è di solito già abbastanza vicino), altrimenti si torna a 2b (pulizia + nuovo spin).

Al momento esatto della convergenza, `update_amcl_flags()` chiama `clear_global_costmap()` (`/global_costmap/clear_entirely_global_costmap`).

*Perché le soglie sono "allentate":* il filtro a particelle mantiene sempre una coda di particelle sparse casualmente come protezione anti-rapimento; questa coda impedisce alla covarianza di scendere a valori molto piccoli anche quando la massa principale è già concentrata sulla posa reale. Soglie a 0.10 m² dichiarano convergenza appena la massa principale si raggruppa, ignorando la coda.

*Perché pulire la global costmap alla convergenza:* mentre AMCL riportava una posa errata, l'obstacle_layer ha proiettato letture LiDAR in posizioni sbagliate, creando "phantom obstacle" che resterebbero finché il robot non ci passa sopra. Svuotare la global costmap evita che la ricerca eviti aree in realtà libere. Lo strato statico (la mappa del Task 1) non è toccato, essendo un plugin separato.

### State 3 — Random search (`state_3_random_search`)

Il robot esplora navigando verso waypoint casuali; a ogni waypoint raggiunto esegue un **pan della testa** per guardarsi attorno. Lo stato esce quando entrambe le pose di approccio (`pick_approach_pose` e `place_approach_pose`) sono state latchate dall'`ArucoTracker`.

**Condizione di uscita (controllata per prima a ogni tick):** se entrambe le pose esistono, si cancella l'eventuale goal in volo, si azzerano i flag di navigazione, si riporta la testa a (tilt −0.5, pan 0), si resetta lo stato della ricerca e si chiama `aruco.freeze()`. Poi si passa allo Stato 4.

**Le tre fasi (`self.search_phase`):**

- **`sample`** — Se la costmap non è ancora arrivata, si attende. Altrimenti si campiona un waypoint con `sampler.sample_random_xy()`. Se ritorna `None` (tutte le celle visitate o lookup TF fallito), si svuota la lista `visited_waypoints` e si riprova. Altrimenti il waypoint scelto viene aggiunto a `visited_waypoints`, si invia il goal Nav2 e si passa a `nav`.
- **`nav`** — Si attende l'esito del goal. Su **successo** si avvia il pan: testa a (−0.5, primo angolo) e fase `pan`. Su **fallimento** si torna a `sample`. È attiva una guardia di timeout (`SEARCH_NAV_TIMEOUT = 120 s`): allo scadere si cancella il goal e si torna a `sample`.
- **`pan`** — La testa scorre le posizioni `SEARCH_PAN_POSITIONS = [-0.6, 0.6, 0.0]` rad (sinistra, destra, centro), restando `SEARCH_PAN_DWELL = 1.5 s` su ciascuna. Completato lo sweep senza aver visto entrambi i marker, si torna a `sample`.

*Perché un pan della testa e non uno spin 2π a ogni waypoint:* lo spin per-waypoint era fragile — i critic di DWB (collisione/oscillazione) abortivano spesso la rotazione, e ogni abort disturbava AMCL. Il pan della testa non muove la base, è fire-and-forget, e non "sloshing" della stima AMCL.

*Perché il range del pan è ridotto a ±0.6 rad:* la depth camera è una sorgente di *clearing* della global costmap. Un pan ampio spazzava i raggi di clearing su settori vuoti e cancellava i voxel che marcavano i mobili (es. la cucina), facendo poi pianificare a Nav2 traiettorie dentro l'ostacolo. ±0.6 rad tiene la camera quasi frontale (FOV ~70°) senza spazzare raggi di clearing.

> In `common_states.py` è presente, commentata, una variante alternativa di `state_3` per **ricerca manuale**: l'operatore invia i goal da RViz mentre il robot esegue solo il pan continuo della testa, e la transizione allo Stato 4 avviene non appena entrambi i marker sono visti. È usata solo per test e non è attiva nella build.

### State 4 — Navigate to PICK approach (`state_4_pick`)

Delega all'helper condiviso `_nav_to_approach()` (in `state_helpers.py`), passando `pick_approach_pose` e `next_state = 5`. Il pattern è "Send / Wait / Retry":

1. `_consume_nav_result()` legge una sola volta l'esito del goal: se **SUCCEEDED** → log e transizione allo Stato 5; se **fallito** → log e si ricade nella ri-invio.
2. `_nav_timeout_guard()` cancella il goal se in volo da più di `APPROACH_NAV_TIMEOUT = 180 s`.
3. Se nessun goal è attivo, si converte la `PoseStamped` in `(x, y, yaw)` e si invia un nuovo goal.

*Perché timeout a 180 s:* il behavior tree Nav2 `navigate_with_replanning_and_recovery` può impiegare 10–15 s per ciclo di recovery (ClearCostmap + Spin + Wait + replan); con timeout troppo bassi si cancellavano i goal a metà recovery, perdendo tutti i progressi del BT.

### State 5 — Navigate to PLACE approach (`state_5_place`)

Identico allo Stato 4, ma verso `place_approach_pose`, con `next_state = 6`. Stesso timeout (180 s) e stessa logica di retry.

### State 6 — Done (`state_6_done`)

Imposta `self.finished = True`. Il loop di `run_task()` esce dallo spin e spegne il nodo.

---

## 6. Componenti di supporto in dettaglio

### 6.1 AMCL — localizzazione globale (`task2_amcl.py`)

Espone i servizi/action usati dallo Stato 2 e i metodi di polling:
- `request_global_localization()`, `clear_local_costmap()`, `clear_global_costmap()` — service client `std_srvs/Empty` e `ClearEntireCostmap`.
- `send_spin()` / `cancel_spin()` — action client Nav2 `/spin`.
- `update_spin_flags()` — gestisce la pipeline del future dello spin (accettazione goal → result), impostando `spin_done`/`spin_succeeded` (status 4 = SUCCEEDED).
- `update_amcl_flags()` — calcola la convergenza dalla covarianza e, alla prima convergenza, svuota la global costmap.

La sottoscrizione a `/amcl_pose` salva solo l'ultimo messaggio; tutta la logica di soglia è nel polling, per tenerla nel thread della macchina a stati.

### 6.2 Campionamento dei waypoint (`task2_costmap.py`)

Il `CostmapSampler` si sottoscrive a `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`) con QoS `TRANSIENT_LOCAL` + `RELIABLE`, così da ricevere il messaggio *latched* pubblicato da Nav2 all'avvio.

`sample_random_xy()` procede così:

1. **Maschera delle celle libere** — `free_mask = (arr == 0)`: si tengono solo le celle a costo esattamente 0 (chiaramente navigabili, non inflate, non sconosciute).
2. **Margine dai bordi** — si azzerano le celle entro `EDGE_MARGIN = 1.0 m` dal bordo della mappa. Il global planner NavFn espande un campo di potenziale attorno al goal; se raggiunge pixel fuori mappa, Nav2 logga `worldToMap failed` migliaia di volte per tentativo. 1.0 m è ben oltre il raggio di inflazione, quindi il planner non tocca mai pixel out-of-bounds.
3. **Posa del robot** — via TF `map ← base_link` (se il lookup fallisce, ritorna `None`).
4. **Distanze** — distanza euclidea di ogni cella libera dal robot.
5. **Rejection dei visitati** — si scartano le celle entro `VISITED_RADIUS = 1.5 m` da qualunque waypoint già tentato, per non tornare ripetutamente sugli stessi punti.
6. **Cap di distanza** — candidati = non visitati **e** entro `SOFT_MAX_RANGE = 3.0 m`; se nessuno è disponibile, si rimuove il cap e si accettano tutti i non visitati (fallback per aree quasi sature).
7. **Pesatura esponenziale** — `w = exp(-d / PREFERRED_RANGE)` con `PREFERRED_RANGE = 2.0 m`. Una cella a 2 m pesa `exp(-1) ≈ 0.37`, una a 4 m `exp(-2) ≈ 0.14`: i waypoint vicini sono nettamente preferiti, ma il campionamento resta probabilistico.
8. **Estrazione** — `np.random.choice` sulla distribuzione normalizzata; lo yaw del goal è orientato dal robot verso il punto scelto.

*Esplorazione a cerchi espandenti:* all'inizio le celle vicine sono tutte non visitate e vengono preferite; man mano che entrano in `visited_waypoints`, i candidati rimasti sono sempre più lontani e la ricerca si espande con ordine, senza salti ai bordi e senza pattern fissi che lascerebbero buchi.

> **Nota di accuratezza:** questa implementazione **non** esegue una BFS di raggiungibilità. La selezione delle celle si basa esclusivamente su: costo nullo, margine dai bordi, memoria dei visitati, cap di distanza e pesatura esponenziale. La connessione effettiva del waypoint all'area del robot è garantita di fatto dal cap di distanza (3 m) combinato con la maschera a costo 0 e dalla gestione dei fallimenti di navigazione (timeout → nuovo campione).

### 6.3 Rilevamento ArUco e pose di approccio (`task2_aruco.py`)

**Configurazione (`launch_common.py`):** due istanze di `aruco_single` (pacchetto `aruco_ros`), una per il marker ID 26 (PICK) e una per il 238 (PLACE), entrambe da 0.25 m, con `reference_frame=""`. Con questa impostazione il nodo pubblica la posa del marker nel frame ottico della camera (`head_front_camera_rgb_optical_frame`), senza applicare TF: la composizione nel frame mappa è fatta interamente nel nostro codice (come nel Lab 3). Questo dà controllo completo su *quando* e *come* comporre (gate di convergenza, keep-closest, timestamp preciso).

**Gestione di una detection (`_handle_aruco_detection`):**

1. **Gate di convergenza** — si ignora ogni detection finché AMCL non ha convergito: prima, il TF `map ← camera` sarebbe costruito su una posa errata e il marker atterrerebbe in un punto sbagliato della mappa.
2. **Conversione** in KDL `Frame` della posa marker-in-camera.
3. **Gate di freeze** — se la posa è già memorizzata e il refresh è disattivato (`freeze()`), si ignora.
4. **Filtro di distanza** — distanza camera–marker dalla norma della traslazione; detection oltre `MAX_DETECTION_DISTANCE = 4.0 m` scartate (a quella distanza l'errore PnP è troppo alto).
5. **Keep-closest** — se il marker è già stato visto, si aggiorna solo se la nuova distanza è **minore** della precedente: la stima finale usa l'osservazione più vicina (e quindi più accurata).
6. **Composizione temporale** — si cerca il TF `map ← camera` **al timestamp esatto del messaggio** (`msg.header.stamp`). Questo congela il marker alla sua posizione reale: usare il tempo corrente moltiplicherebbe un TF recente (robot spostato) per una posa vecchia del marker, facendo "scivolare" la stima a ogni movimento. Se il lookup fallisce, la detection viene scartata.
7. La posa marker-in-map si ottiene con `frame_map_cam * aruco_in_cam`.

**Calcolo della posa di approccio (`publish_approach_frames`, timer a 5 Hz):**

1. Si estrae la direzione Z del marker in mappa (`marker_in_map.M * Vector(0,0,1)`).
2. La si **proietta sul piano orizzontale XY** e la si normalizza, eliminando la componente verticale dovuta all'inclinazione di camera/marker (fallback `(1,0)` se il marker punta quasi verticale).
3. La posizione di approccio è a `APPROACH_DISTANCE = 0.80 m` dal marker lungo questa direzione; la quota Z mantiene l'altezza del marker (utile al braccio nel Task 3).
4. Lo yaw è `atan2(marker.y − approach.y, marker.x − approach.x)`: arrivato lì, il robot guarda verso il marker.
5. L'orientazione è `Rotation.RPY(0, 0, yaw)` (roll/pitch nulli, compatibile con Nav2 2D).

La posa viene broadcastata come TF (`aruco_pick_approach` / `aruco_place_approach`) e latchata come `PoseStamped` per il goal Nav2.

**Freeze (transizione 3 → 4):** finché si è in ricerca, la posa di approccio è continuamente aggiornata (keep-closest). Alla transizione allo Stato 4, `aruco.freeze()` imposta `_refresh_approach_poses = False`: da quel momento il goal Nav2 non insegue più una stima in movimento, evitando re-planning continui durante l'avvicinamento.

### 6.4 Client di navigazione (`task2_nav.py`)

`NavClient` incapsula l'action client `/navigate_to_pose` con polling:
- `send_goal(x, y, yaw)` costruisce il `NavigateToPose.Goal` (yaw → quaternione) e invia in modo asincrono;
- `update_flags()` gestisce la pipeline accettazione → result, impostando `goal_done`/`goal_succeeded` (status 4 = SUCCEEDED);
- `cancel()`, `reset_latches()` per il controllo da parte della macchina a stati.

Espone inoltre `publish_forward(speed, angular)` / `stop()` che pubblicano un `Twist` su **`/nav_vel`** (non `/cmd_vel`). Questi non sono usati nel Task 2 ma sono fondamentali nel Task 3: Nav2/velocity_smoother "inghiottono" `/cmd_vel` quando il navigatore è inattivo, mentre `/nav_vel` (ingresso del twist_mux) arriva davvero alla base.

### 6.5 Controllo braccio e testa (`tiago_arm.py`)

`ArmController` avvolge `pymoveit2.MoveIt2` sul gruppo `arm_torso` (end-effector `gripper_grasping_frame`, planner RRTConnect):
- `move_to_home(tilt_head)` invia la configurazione HOME e, opzionalmente, inclina la testa;
- `tilt_head(tilt, pan)` pubblica un `JointTrajectory` su `/head_controller/joint_trajectory` (la testa è un controller separato dal gruppo MoveIt; non serve pianificazione collision-aware, quindi è fire-and-forget);
- `move_to_pose(position, quat, cartesian)` per i goal cartesiani/joint-space (usato nel Task 3);
- `update_flags()` mantiene i latch `motion_started`/`motion_done` interrogando `query_state()`.

---

## 7. Parametri chiave (valori effettivi in `constants.py`)

| Parametro | Valore | Motivazione |
|---|---|---|
| `HOME_JOINT_POSITIONS` | `[0.15, 0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0]` | Configurazione raccolta del braccio per navigazione |
| `APPROACH_DISTANCE` | 0.80 m | Distanza di stop davanti al marker a parete |
| `MAX_DETECTION_DISTANCE` | 4.0 m | Oltre questa soglia l'errore PnP ArUco è eccessivo |
| `VISITED_RADIUS` | 1.5 m | Memoria waypoint: niente ricampionamento in zone già scansionate |
| `PREFERRED_RANGE` | 2.0 m | Scala del peso esponenziale di campionamento |
| `SOFT_MAX_RANGE` | 3.0 m | Cap di distanza per il campionamento |
| `EDGE_MARGIN` | 1.0 m | Margine dai bordi mappa (evita `worldToMap failed` del NavFn) |
| `SEARCH_PAN_POSITIONS` | `[-0.6, 0.6, 0.0]` rad | Pan testa per waypoint (no clearing sulla costmap) |
| `SEARCH_PAN_DWELL` | 1.5 s | Permanenza per posizione di pan |
| `SEARCH_NAV_TIMEOUT` | 120 s | Timeout per singolo waypoint di ricerca |
| `APPROACH_NAV_TIMEOUT` | 180 s | Timeout navigazione verso PICK/PLACE (cicli di recovery del BT) |
| `PLANNING_TIMEOUT` | 6 s | Attesa avvio del moto MoveIt |
| `EXECUTION_TIMEOUT` | 30 s | Attesa completamento del moto MoveIt |
| AMCL cov_xx / cov_yy | < 0.10 m² | Soglia convergenza posizione |
| AMCL cov_yaw | < 0.07 rad² | Soglia convergenza orientazione |

---

## 8. Problemi affrontati e soluzioni

| Problema | Causa | Soluzione | Perché funziona |
|---|---|---|---|
| Lo spin per-waypoint falliva spesso e disturbava AMCL | I critic DWB abortivano la rotazione; ogni abort perturbava la stima | Sostituito con un **pan della testa** (`SEARCH_PAN_POSITIONS`) | Il pan non muove la base, è fire-and-forget, non altera AMCL |
| Il robot pianificava dentro i mobili dopo aver guardato attorno | La depth camera fa *clearing* sulla global costmap; un pan ampio cancellava i voxel degli ostacoli | Range del pan ridotto a **±0.6 rad** | Tiene la camera quasi frontale senza spazzare raggi di clearing |
| Pose di approccio dentro i muri se calcolate troppo presto | TF `map ← camera` costruito su posa AMCL non ancora convergente | **Gate di convergenza** prima di processare le detection | Si compone solo quando la posa mappa è affidabile |
| La stima del marker "scivolava" a ogni movimento | Composizione con TF corrente invece che al timestamp della foto | **Composizione al `msg.header.stamp`** | Congela il marker alla sua posizione reale nel mondo |
| Stime di approccio imprecise da lontano | Errore PnP crescente con la distanza | Filtro a `MAX_DETECTION_DISTANCE` + **keep-closest** | Si tiene l'osservazione più vicina/accurata |
| Il goal Nav2 cambiava durante l'avvicinamento | La posa veniva aggiornata mentre Nav2 già navigava | **`freeze()`** alla transizione 3 → 4 | Target stabile, niente re-planning continuo |
| `worldToMap failed` a raffica nel NavFn | Waypoint troppo vicini al bordo mappa | **`EDGE_MARGIN = 1.0 m`** nel campionamento | Il planner non raggiunge mai pixel out-of-bounds |
| Ostacoli fantasma evitati durante la ricerca | Obstacle_layer popolato mentre la posa era errata | **Clear della global costmap alla convergenza** | Rimuove i marcatori spuri senza toccare lo strato statico |
| Goal di approccio cancellati a metà recovery | Timeout troppo basso vs. cicli del BT (10–15 s) | **`APPROACH_NAV_TIMEOUT = 180 s`** | Lascia completare i cicli di recovery senza perdere progressi |
