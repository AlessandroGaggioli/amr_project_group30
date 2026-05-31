# Task 1 — Autonomous Mobile Robotics Exam, Group 30

## Obiettivo

Il Task 1 richiede di avviare il robot in un ambiente inizialmente sconosciuto e di costruire una mappa autonoma tramite SLAM. Il flusso complessivo è il seguente:

1. Avvio della simulazione Gazebo nel mondo assegnato al gruppo (`group30`).
2. Messa in sicurezza del braccio in configurazione HOME, così da non ostacolare i sensori e ridurre l'ingombro in navigazione.
3. Avvio dello stack di navigazione Nav2 in modalità SLAM.
4. Esplorazione autonoma con strategia *frontier-based* tramite `explore_lite`, fino al completamento della mappatura.

---

## Architettura software

L'architettura del task combina componenti standard ROS 2 e Pal Robotics con un nodo personalizzato per preparare il robot prima della navigazione.

| Componente | File / Pacchetto | Responsabilità |
|---|---|---|
| `task1.launch.py` | `launch/` | Orchestrazione sequenziale: simulazione, ripiegamento del braccio, SLAM e avvio dell'esplorazione. |
| `Task1Manager` | `task1_manager.py` | Macchina a stati non bloccante che porta il braccio in HOME e attende la fine del movimento. |
| `ArmController` | `tiago_arm.py` | Interfaccia verso MoveIt2 e `move_group`. |
| `explore_node` | `explore_lite` | Scelta delle frontiere e assegnazione dei goal di navigazione. |
| `tiago_nav2.yaml` | `pal_navigation_cfg_params/params/` | Configurazione completa di Nav2: AMCL, controller, planner, costmap, behavior server e SLAM toolbox. |

### Orchestrazione del launch

Nel Task 1 il Launch File controlla la sequenza con un `RegisterEventHandler` su `OnProcessExit`. Il `Task1Manager` viene avviato per primo; quando termina, il launch attende un breve assestamento di 5 secondi tramite `TimerAction` e poi lancia `explore_node`. Questa scelta evita che l'esplorazione parta mentre il robot sta ancora completando il tuck del braccio.

---

## Macchina a stati del Task1Manager

Il `Task1Manager` è una macchina a stati semplice, eseguita su un thread dedicato, che supervisiona il tuck del braccio senza bloccare il resto dell'esecuzione.

### State 0 — Tuck arm to HOME

All'avvio, quando l'executor è stabile, il controller comanda il braccio verso la posizione `home`. Nel Task 1 la testa non è vincolata in modo stretto, perché la percezione principale è affidata al LiDAR 2D. Terminato il comando, la macchina passa allo stato successivo e memorizza il tempo di inizio per gestire eventuali timeout.

### State 1 — Wait for arm motion

Il nodo effettua polling periodico sul risultato di MoveIt2. Se il piano non viene eseguito entro il tempo limite, il sistema torna allo State 0 e riprova. Quando il movimento è concluso, la macchina procede allo stato finale.

### State 2 — Task completed

La macchina chiude i loop interni e distrugge il nodo `Task1Manager`. Questo sblocca l'evento di fine processo nel Launch File e consente di avviare l'esplorazione.

---

## Configurazione Nav2

Il file `tiago_nav2.yaml` definisce il comportamento di tutto lo stack Nav2. I parametri non servono solo a far muovere il robot, ma a renderlo stabile durante la costruzione della mappa e capace di recuperare dai blocchi durante l'esplorazione.

### AMCL

AMCL è il localizzatore probabilistico che stima la posa del robot sulla mappa.

- `use_sim_time: true` allinea tutti i timestamp al clock della simulazione.
- `base_frame_id: base_footprint`, `odom_frame_id: odom`, `global_frame_id: map` definiscono la catena TF usata da Nav2.
- `scan_topic: /scan_raw` indica il LaserScan usato per la correzione della posa.
- `laser_model_type: likelihood_field` usa il modello standard più robusto in ambienti strutturati.
- `max_particles`, `min_particles`, `pf_err`, `pf_z` regolano la qualità del filtro particellare: più particelle aumentano la robustezza, a costo di più calcolo.
- `update_min_d` e `update_min_a` evitano aggiornamenti troppo frequenti quando il robot si muove poco.
- `set_initial_pose: true` con `initial_pose` inizializza AMCL con una posa dummy. Nel Task 1 serve solo a far pubblicare subito la TF `map -> odom`, così planner, costmap e RViz non restano in attesa all'avvio.

### Behavior server

Il behavior server contiene le azioni di recupero usate da Nav2 quando il robot si blocca.

- `behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]` abilita le quattro recovery di base.
- `global_frame: odom` e `robot_base_frame: base_footprint` fanno sì che le manovre di recovery lavorino nel frame locale del robot.
- `simulate_ahead_time: 1.0` riduce l'orizzonte di simulazione delle recovery: un valore più basso rende meno conservativa la verifica di collisione e impedisce che lo spin si autoannulli troppo presto in spazi stretti.
- `max_rotational_vel`, `min_rotational_vel`, `rotational_acc_lim` definiscono quanto rapidamente il comportamento di spin può accelerare e ruotare.

### BT Navigator

Il BT Navigator esegue il Behavior Tree che coordina pianificazione, controllo e recuperi.

- `default_bt_xml_filename: navigate_w_replanning_and_recovery.xml` seleziona un albero già predisposto per ripianificare e recuperare automaticamente.
- `plugin_lib_names` elenca tutti i nodi BT caricabili: planner, controller, recovery, condizioni di stato e nodi di selezione.
- `enable_groot_monitoring: True` insieme alle porte `1666` e `1667` abilita il monitoraggio visivo del Behavior Tree con Groot.
- `global_frame: map`, `robot_base_frame: base_footprint` e `odom_topic: /mobile_base_controller/odom` collegano il BT al sistema TF e all'odometria del robot.

### Controller server

Il controller server produce i comandi di velocità per seguire il path calcolato dal planner.

- `controller_frequency: 20.0` impone un controllo a 20 Hz.
- `min_x_velocity_threshold`, `min_theta_velocity_threshold` evitano oscillazioni di comando molto piccole e rumorose.
- `controller_plugins: ["FollowPath"]` usa il local planner DWB per seguire il path globale.
- `progress_checker_plugin: "progress_checker"` e `movement_time_allowance: 5.0` fanno fallire prima il controllo se il robot non avanza realmente.
- `required_movement_radius: 0.2` rende il rilevamento dello stallo più sensibile, utile per passare più in fretta ai recuperi quando il robot resta fermo.
- `general_goal_checker` imposta le tolleranze finali sul goal: `xy_goal_tolerance: 0.30` e `yaw_goal_tolerance: 0.50` bilanciano precisione e robustezza.
- Nel profilo `FollowPath`, i parametri di velocità e accelerazione (`max_vel_x`, `max_vel_theta`, `acc_lim_x`, `acc_lim_theta`, `decel_lim_*`) limitano il moto a valori compatibili con il robot.
- `vx_samples`, `vy_samples`, `vtheta_samples`, `sim_time`, `linear_granularity` e `angular_granularity` controllano quante traiettorie candidate vengono simulate e con quale dettaglio.
- I critic DWB (`BaseObstacle`, `PathAlign`, `GoalAlign`, `PathDist`, `GoalDist`, `RotateToGoal`) bilanciano sicurezza, aderenza al path e orientamento finale.

### Costmap globale

La global costmap rappresenta il mondo a livello di navigazione globale.

- `global_frame: map` e `robot_base_frame: base_footprint` fissano il riferimento cartografico.
- `track_unknown_space: true` mantiene le zone non osservate come ignote, cosa utile nella fase di esplorazione.
- `plugins: ["static_layer", "obstacle_layer", "inflation_layer"]` combina mappa statica, ostacoli osservati e zona di sicurezza attorno agli ostacoli.
- `robot_radius: 0.28` approssima l'ingombro del robot nella costmap.
- Nell'`obstacle_layer`, `observation_sources: scan` usa il LiDAR come sensore principale. `raytrace_*` cancella ostacoli lungo il raggio del fascio, mentre `obstacle_*` controlla fino a che distanza un ostacolo viene marcato.
- `static_layer` con `map_subscribe_transient_local: True` e `subscribe_to_updates: True` permette di ricevere aggiornamenti persistenti della mappa.
- `inflation_layer` con `inflation_radius: 0.7` e `cost_scaling_factor: 3.5` crea una zona di rispetto attorno agli ostacoli, rendendo il piano più prudente.

### Costmap locale

La local costmap serve al controller per reagire agli ostacoli vicini.

- `global_frame: odom` la fa muovere con il robot, così la navigazione locale è stabile rispetto all'odometria.
- `rolling_window: true` mantiene la finestra centrata sul robot.
- `width: 5`, `height: 5`, `resolution: 0.05` definiscono una finestra locale abbastanza ampia ma leggera da calcolare.
- `plugins: ["voxel_layer", "inflation_layer"]` combina ostacoli osservati in 3D con una zona di sicurezza locale.
- `voxel_layer` usa sia `scan` sia `pointcloud`, permettendo di vedere ostacoli bassi e alti; questo è importante per oggetti visibili solo dalla camera RGB-D.
- `z_voxels: 40`, `z_resolution: 0.05` e `origin_z: 0.0` definiscono un volume verticale sufficiente a includere l'origine della camera e l'intero corpo del robot.
- `min_obstacle_height: 0.05` e `max_obstacle_height: 2.0` filtrano il rumore del pavimento e mantengono solo ostacoli realmente rilevanti.
- `inflation_radius: 0.45` e `cost_scaling_factor: 10.0` producono una decrescita dei costi più ripida, così il controller riesce a trovare traiettorie praticabili in spazi stretti senza essere troppo bloccato.

### Planner server

Il planner server genera il path globale.

- `planner_plugins: ["GridBased"]` usa `NavfnPlanner`, cioè un planner classico su griglia.
- `use_astar: true` seleziona A* per ottenere piani più informati rispetto a Dijkstra.
- `tolerance: 1.99` consente una certa flessibilità nel raggiungere il goal quando la posa esatta è difficile da occupare.
- `allow_unknown: true` è fondamentale durante l'esplorazione, perché il robot deve poter pianificare anche in regioni non ancora osservate.

### SLAM Toolbox

`slam_toolbox` costruisce la mappa incrementale del Task 1.

- `mode: mapping` avvia il sistema in modalità costruzione mappa.
- `scan_topic: /scan_raw`, `odom_frame: odom`, `map_frame: map`, `base_frame: base_footprint` definiscono i segnali di input e i frame usati per l'ottimizzazione.
- `use_scan_matching: true` e `use_scan_barycenter: true` migliorano l'allineamento tra scansioni successive.
- `minimum_travel_distance` e `minimum_travel_heading` evitano aggiornamenti inutili quando il robot è quasi fermo.
- `transform_timeout`, `tf_buffer_duration`, `transform_publish_period` e `map_update_interval` regolano rispettivamente l'attesa TF, la memoria del buffer, la frequenza di pubblicazione e la frequenza di aggiornamento della mappa.
- I parametri di ottimizzazione Ceres (`solver_plugin`, `ceres_linear_solver`, `ceres_preconditioner`, `ceres_trust_strategy`, `ceres_dogleg_type`) determinano come viene risolto il problema di pose graph in SLAM.
- I parametri di loop closure e correlazione (`do_loop_closing`, `loop_search_*`, `loop_match_*`, `correlation_search_*`) influenzano quanto aggressivamente il sistema cerca di chiudere i cicli e correggere la deriva.

### Waypoint follower e map server

- `waypoint_follower` è configurato con `stop_on_failure: false`, quindi un eventuale goal fallito non interrompe automaticamente tutta la sequenza di navigazione.
- `wait_at_waypoint` con `waypoint_pause_duration: 0` evita soste superflue nei waypoint.
- `map_saver` e `map_server` preparano rispettivamente il salvataggio e la pubblicazione della mappa una volta conclusa l'esplorazione.

---

## Esplorazione autonoma con explore_lite

Il comportamento *where to go next* è affidato a `explore_lite`, che analizza le frontiere sulla mappa e invia goal a Nav2.

- `costmap_topic: "/map"` collega l'esploratore alla mappa SLAM incrementale.
- `planner_frequency: 0.33 Hz` riduce la frequenza di rivalutazione delle frontiere, con un costo computazionale più basso.
- `progress_timeout: 60.0` limita il tempo concesso a un goal bloccato prima di abbandonarlo e scegliere una nuova frontiera.
- `gain_scale: 2.0` privilegia le frontiere che promettono maggiore guadagno informativo.
- `min_frontier_size: 0.25` scarta regioni troppo piccole o frammentate, che tendono a produrre goal poco utili.

La mappa è considerata completa quando `explore_lite` non trova più frontiere esplorabili. A quel punto il salvataggio finale può essere effettuato con gli strumenti standard di Nav2, cristallizzando il risultato del Task 1 per l'uso nei task successivi.
