# Task 3 — Autonomous Mobile Robotics Exam, Group 30

## 1. Obiettivo

Il Task 3 estende il Task 2 con un ciclo di **pick-and-place autonomo di due cubi** marcati ArUco (ID 63 per primo, poi ID 582). Partendo dal robot in posa casuale e ignota, il sistema deve, in autonomia, per ciascun cubo:

1. Eseguire l'intero flusso del Task 2 (HOME → localizzazione AMCL → ricerca → navigazione alla posa di approccio del marker PICK a parete).
2. Rilevare il marker sul cubo da prelevare e raffinare l'avvicinamento fino a portarlo nel workspace del braccio.
3. Afferrare il cubo con MoveIt2 e renderlo solidale alla pinza tramite il servizio `/ATTACHLINK`.
4. Navigare verso la posa di approccio del marker PLACE a parete, ri-localizzare il marker da vicino e avvicinarsi alla superficie.
5. Depositare il cubo, sganciarlo con `/DETACHLINK`, e ritirarsi.
6. Ripetere per il secondo cubo; al termine, concludere il task.

Come nel Task 2, tutto è realizzato con una **macchina a stati non bloccante**. A differenza del Task 2, la fase di manipolazione è **data-driven**: le sequenze di grasp e di drop sono *liste di passi* eseguite da un unico driver, invece di una lunga scala di stati numerati quasi identici.

---

## 2. Descrizione generale dello svolgimento

Il Task 3 riusa integralmente i componenti del Task 2 e ne aggiunge tre nuovi (gripper, link attacher, tracker dei marker sui cubi). La macchina a stati (`task3_state_machine.py`) usa un `IntEnum` `Phase` i cui valori 0–6 **coincidono** con gli stati di `CommonStates`: questo permette di riutilizzare *as-is* gli stati 0–5 del Task 2 e di innestare, sui successi degli stati 4 (PICK) e 5 (PLACE), i sotto-flussi di manipolazione (fasi 10–17).

Tre idee architetturali guidano l'implementazione:

- **Riuso dei flussi base.** Stati 0–3 identici al Task 2. Lo Stato 4 (arrivo al marker PICK) non termina ma avvia il sotto-flusso di pick; lo Stato 5 (arrivo al marker PLACE) avvia il sotto-flusso di drop.
- **Manipolazione data-driven.** Le sequenze GRASP e DROP sono liste di *callable* (uno per passo) eseguite dalla fase `RUN_SEQUENCE` tramite il driver `_run_sequence()`. Ogni passo pilota uno *step-runner* condiviso (`_arm_motion_step`, `_gripper_step`, `_link_step`) e ritorna `True` quando completo. Questo elimina la duplicazione e rende le due sequenze leggibili come elenchi.
- **Fasi parametriche.** Le fasi `PUSH` (avvicinamento finale closed-loop) e `BACK_OUT` (retromarcia di sicurezza) sono uniche e parametrizzate dal flag `self._push_mode` (`"pick"` o `"place"`), riusando lo stesso motore di controllo per entrambe le superfici.

Il filo conduttore dei problemi risolti è la **precisione dell'avvicinamento finale**: Nav2 si ferma lontano dall'oggetto (per via dell'inflation_radius), il workspace del braccio Tiago è limitato (~1 m), e le pose ArUco contengono rumore PnP. Il sistema chiude il divario con un push closed-loop su `/nav_vel`, ri-localizza i marker da vicino prima di agire, e costruisce le pose di grasp/drop in frame `base_link` scartando le componenti rumorose dell'orientazione.

---

## 3. Architettura software

Il nodo `Task3Manager` (`task3_manager.py`) istanzia i componenti del Task 2 più i tre nuovi:

| Componente | File | Responsabilità |
|---|---|---|
| Componenti Task 2 | `task2_*.py`, `tiago_arm.py` | Nav2, AMCL, ArUco a parete, braccio MoveIt2, costmap sampler |
| `GripperController` | `tiago_gripper.py` | Apertura/chiusura pinza via `pymoveit2.GripperInterface` (polling) |
| `LinkAttacher` | `task3_link_attacher.py` | Servizi asincroni `/ATTACHLINK` e `/DETACHLINK` (plugin IFRA) |
| `CubeTracker` | `task3_cube_tracker.py` | Marker sui cubi, gating spaziotemporale, keep-closest |
| `Task3StateMachine` | `task3_state_machine.py` | Macchina a stati completa (fasi 0–6 base + 10–17 manipolazione) |

**Concorrenza:** `run_task()` fa girare la macchina a stati su un thread daemon e il nodo su un `MultiThreadedExecutor` con **5 thread** (uno in più del Task 2): servono per far girare in concorrenza i feedback di action di braccio e nav, il callback group del gripper, e i future dei servizi attach/detach. Anche qui la macchina a stati attende `executor_ready` e dorme 10 s per dare tempo allo stack.

**Pattern di polling:** a ogni tick il `run()` chiama `arm.update_flags()`, `nav.update_flags()`, `amcl.update_spin_flags()`, `amcl.update_amcl_flags()` e — novità del Task 3 — `gripper.update_flags()`.

**Configurazione ArUco (`task3.launch.py`):** quattro istanze di `aruco_single` — i due marker a parete da 0.25 m (ID 26 PICK, ID 238 PLACE) e i due marker sui cubi da 0.07 m (ID 63, ID 582) — tutte con `reference_frame=""` (composizione manuale nel codice).

---

## 4. Mappa degli stati (flusso di lavoro)

```
  [0]ARM_HOME ─▶ [1]WAIT_ARM ─▶ [2]AMCL ─▶ [3]SEARCH ─▶ [4]NAV_PICK
                                                              │ SUCCEEDED
                                                              ▼
  ┌────────────────────── PICK SUB-FLOW ──────────────────────────────┐
  │ [10]HEAD_TILT ─▶ [11]WAIT_CUBE                                      │
  │                     │ prima detection, cubo lontano                 │
  │                     ▼                                               │
  │            [12]WAIT_CUBE_NAV ─(SUCCEEDED)─▶ [14]PUSH(pick)          │
  │                     │                            │ stop @0.75 m     │
  │                     │                            ▼                  │
  │                     └────────────────▶ [11]WAIT_CUBE (close)        │
  │                                              │ freeze + grasp poses │
  │                                              ▼                      │
  │                                     [13]RUN_SEQUENCE(grasp)         │
  │                                              │ on_complete          │
  │                                              ▼                      │
  │                                     [15]BACK_OUT(pick)              │
  └──────────────────────────────────────────────│────────────────────┘
                                                  ▼
                                            [5]NAV_PLACE
                                                  │ SUCCEEDED
                                                  ▼
  ┌────────────────────── DROP SUB-FLOW ──────────────────────────────┐
  │ [16]REFRESH_PLACE ─▶ [14]PUSH(place) ─▶ [13]RUN_SEQUENCE(drop)      │
  │  (ri-detect close)     stop @0.40 m          │ on_complete          │
  │                                              ▼                      │
  │                                     [15]BACK_OUT(place)             │
  │                                              ▼                      │
  │                                     [17]NEXT_OR_DONE                │
  └──────────────────────────────────────────────│────────────────────┘
                              cubo successivo ─────┤───── ultimo cubo
                                     │                         │
                                     ▼                         ▼
                              [4]NAV_PICK                  [6]DONE
```

Il `Phase` IntEnum: `ARM_HOME=0, WAIT_ARM=1, AMCL=2, SEARCH=3, NAV_PICK=4, NAV_PLACE=5, DONE=6, HEAD_TILT=10, WAIT_CUBE=11, WAIT_CUBE_NAV=12, RUN_SEQUENCE=13, PUSH=14, BACK_OUT=15, REFRESH_PLACE=16, NEXT_OR_DONE=17`.

---

## 5. Analisi dettagliata di ogni stato/fase

### 5.1 Fasi 0–3 (riuso del Task 2)

`ARM_HOME`, `WAIT_ARM`, `AMCL`, `SEARCH` sono **identiche** al Task 2 (vedi TASK2_REPORT). La ricerca si conclude quando entrambi i marker a parete sono latchati, le pose vengono congelate e si passa a `NAV_PICK`.

### 5.2 NAV_PICK (4) — override (`state_4_pick`)

Riusa l'helper `_nav_to_approach` verso `pick_approach_pose`, **ma su successo non va allo Stato 5**: transita a `HEAD_TILT (10)` per iniziare il sotto-flusso di pick del cubo corrente (`current_cube_id`).

### 5.3 HEAD_TILT (10) — `_phase_head_tilt`

- **Abilita il CubeTracker** (`cube_tracker.enable()`): durante la ricerca del Task 2 era disattivato per non memorizzare pose PnP rumorose viste da lontano/obliquo.
- Inclina la testa a `HEAD_TILT_DOWN_FOR_CUBE = -1.0 rad` (pan 0): la posa di approccio del marker a parete ha già allineato il robot alla superficie, quindi il cubo dovrebbe essere frontale.
- Inizializza lo stato di head-scan e passa a `WAIT_CUBE`.

### 5.4 WAIT_CUBE (11) — `_phase_wait_cube`

Stato centrale del pick, con tre comportamenti a seconda del contesto:

1. **Marker non ancora visto:** per i primi ~2 s la testa resta ferma (per non invalidare una detection immediata con il movimento). Poi, se ancora nulla, parte un **head scan** (`HEAD_SCAN_PANS = [0.0, -1.3, 1.3]`, `HEAD_SCAN_DWELL = 3.0 s`) per portare il cubo nel FOV.
2. **Marker visto, ma cubo lontano (`_cube_approached == False`):** la posa di approccio del marker a parete lascia tipicamente il cubo fuori dal workspace del braccio. Si invia un goal Nav2 cubo-centrico (`_send_cube_approach_nav`) e si passa a `WAIT_CUBE_NAV`.
3. **Marker visto e robot già avvicinato (`_cube_approached == True`):** la policy keep-closest fa sì che `cube_marker_in_map` sia ora la stima a corto raggio. Si **congela** il tracker (`freeze()`), si calcolano le pose di grasp (`_precompute_grasp_poses`) e si avvia la sequenza di grasp (`RUN_SEQUENCE`).

*Perché congelare prima del grasp:* una detection rumorosa tardiva potrebbe spostare lateralmente il target durante la presa.

### 5.5 WAIT_CUBE_NAV (12) — `_phase_wait_cube_nav`

Attende solo l'esito del goal cubo-centrico emesso in WAIT_CUBE:
- **SUCCEEDED:** Nav2 si è fermato al limite dell'inflation. Si reinclina la testa giù, si imposta `_push_mode = "pick"` e si passa a `PUSH` per chiudere il divario.
- **Fallito:** si imposta `_cube_approached = True` e si torna a `WAIT_CUBE` per tentare comunque la presa.
- Guardia di timeout (`APPROACH_NAV_TIMEOUT`).

**`_send_cube_approach_nav` (perché così):** il goal mette `base_link` a `SAFE_NAV_DISTANCE` dal centro del cubo, **lungo la normale al muro PICK** (yaw estratto da `pick_approach_pose`), non lungo la direzione robot→cubo. Usare la direzione robot→cubo poteva parcheggiare il robot **di fianco** alla superficie, dove la footprint entra nell'inscribed_radius e DWB rifiuta ogni traiettoria ("Trajectory Hits Obstacle"). L'errore PnP sullo yaw del muro è < 10°, più affidabile dell'heading robot→cubo (affetto da drift AMCL e tolleranza XY di Nav2).

### 5.6 PUSH (14) — `_phase_push` + `_cmd_vel_push` (parametrica pick/place)

Avvicinamento finale closed-loop, condiviso da pick e place. A ogni tick legge `map → base_link`, calcola la distanza dal marker di riferimento (`_push_reference_xy`: cubo per pick, marker place per place) e pubblica un `Twist` su `/nav_vel`:
- avanza a `DRIVE_SPEED = 0.15 m/s` finché non è entro `stop_dist` (`CUBE_APPROACH_DISTANCE = 0.75` per pick, `PLACE_APPROACH_DISTANCE = 0.40` per place);
- **termine di sterzata** (`steer=True`): velocità angolare proporzionale all'errore di heading (`YAW_GAIN = 1.5`, saturata a `YAW_MAX = 0.5`), per arrivare **frontale e centrato** sul target.

Esiti:
- **pick** completato: `_cube_approached = True`; si inclina la testa più giù (−1.15 rad) per una lettura ottica pulita ravvicinata, si attende l'assestamento dell'inerzia, si **resetta** la detection del cubo (`reset_cube`) e si **sblocca** il tracker (`unfreeze`) così che vinca la nuova stima a corto raggio; si torna a `WAIT_CUBE` (che ora grasperà).
- **place** completato: si avvia direttamente la sequenza di drop (`_start_drop_sequence`).
- Se il marker di riferimento manca, si ferma e si procede comunque.

*Perché un push su `/nav_vel` e non `/drive_on_heading` né `/cmd_vel`:* `/drive_on_heading` (open-loop) era fragile e abortito dai behavior; `/cmd_vel` viene assorbito dal velocity_smoother quando Nav2 è inattivo; `/nav_vel` (ingresso del twist_mux) arriva davvero alla base. La velocità 0.15 m/s evita che lo smoother rampi il comando quasi a zero.

### 5.7 RUN_SEQUENCE (13) — `_run_sequence` + sequenze grasp/drop

Il driver esegue il passo corrente; quando ritorna `True` avanza, resetta il latch `_step_sent` e, a lista esaurita, esegue la callback `on_complete`.

**Sequenza GRASP (`_start_grasp_sequence`):**

1. **Apri il gripper** (fire-and-forget con pausa visibile in Gazebo).
2. **Braccio a PRE-GRASP** — sopra il cubo, a `PRE_GRASP_LIFT = 0.40 m` sopra il centro, con l'orientazione di discesa verticale.
3. **Braccio a GRASP — discesa cartesiana** — `cartesian=True` così l'end-effector scende **in linea retta** (un piano joint-space curverebbe di lato e urterebbe il cubo); si ferma a `GRASP_Z_ABOVE_TOP = 0.04 m` sopra la faccia superiore.
4. **Chiudi il gripper** — con `wait=True`: si fa il polling di `action_done` (le dita devono **assestarsi** prima dell'attach rigido) + settle 0.5 s.
5. **Attach link** (`/ATTACHLINK`) — rende il cubo solidale alla pinza; su fallimento riprova.
6. **Sollevamento cartesiano** — inverso della discesa, così il cubo sale dritto senza urtare il vicino o il bordo del tavolo.
7. **Tuck del braccio a HOME** — per non interferire con path-planning e laser scan durante la navigazione.

`on_complete = _grasp_complete`: riporta la testa a livello e avvia `BACK_OUT("pick")`.

**Sequenza DROP (`_start_drop_sequence`):**

1. **Braccio a PRE-DROP** — `prepare = _precompute_drop_poses` calcola le pose; PRE-DROP è sollevato di `POST_GRASP_LIFT = 0.30 m` sopra il target.
2. **Braccio a DROP — discesa cartesiana** (mantiene l'orientazione, niente spin del polso).
3. **Apri il gripper** per rilasciare.
4. **Detach link** (`/DETACHLINK`) — avanza anche su timeout (`advance_on_timeout=True`).
5. **Sollevamento** a PRE-DROP, per liberarsi del cubo depositato.
6. **Tuck a HOME** per la navigazione di ritorno.

`on_complete` → `BACK_OUT("place")`.

### 5.8 BACK_OUT (15) — `_phase_back_out` (parametrica pick/place)

Specchio del PUSH: retromarcia (`publish_forward(-DRIVE_SPEED)`) finché `base_link` non dista almeno `SAFE_NAV_DISTANCE + 0.5 = 1.6 m` dal marker di riferimento, poi ferma, resetta i latch nav e passa alla gamba successiva (`NAV_PLACE` per pick, `NEXT_OR_DONE` per place). Guardia di timeout 60 s.

*Perché:* dopo il push il robot è dentro l'inflation_radius della superficie; Nav2 non riuscirebbe a pianificare (il BT entrerebbe in recovery/thrashing). Allontanarsi in closed-loop libera la zona prima di ripianificare.

### 5.9 REFRESH_PLACE (16) — `_phase_refresh_place`

Il marker PLACE è stato localizzato **da lontano** durante la ricerca del Task 2 e poi congelato, con errore PnP anche di ~metri. Prima del drop bisogna ri-localizzarlo da vicino:
- si riazzera `place_detection_distance` e si riattiva `_refresh_approach_poses` (sblocco del keep-closest), così la detection ravvicinata vince su quella stale;
- si fa un head scan (tilt giù + pan sweep) per portare il marker basso nel FOV (il robot raramente parcheggia perfettamente centrato);
- quando `place_detection_distance < 1.5 m` (o al timeout), si ri-congela, si imposta `_push_mode = "place"` e si passa a `PUSH`.

Questa fase è la mitigazione **implementata** al problema "drop fuori dal tavolo": invece di fidarsi della posa stimata da lontano, si ri-misura il marker da ~1 m.

### 5.10 NEXT_OR_DONE (17) — `_phase_next_or_done`

Incrementa l'indice del cubo. Se i cubi sono finiti → `DONE`. Altrimenti, per il cubo successivo: `reset_cube(next_id)` (scarta eventuali osservazioni stale a lungo raggio), `unfreeze()` del tracker, `_cube_approached = False` (così anche il prossimo cubo attiva il proprio raffinamento), reset dei latch nav, e ritorno a `NAV_PICK`.

### 5.11 DONE (6)

`state_6_done()` imposta `finished = True`: `run_task()` esce e spegne il nodo.

---

## 6. Geometrie di grasp e di drop

### 6.1 Pose di grasp (`_precompute_grasp_poses`)

Costruite in frame **`base_link`** (anti-drift, stile Lab 3):

- **Sorgente preferita:** marker visto in camera (`cube_marker_in_camera`) trasformato `base_link ← camera`. **Fallback:** marker in mappa trasformato `base_link ← map`.
- **Posizione:** il marker è sulla faccia superiore del cubo; il centro è `CUBE_TOP_TO_CENTER = 0.035 m` più in basso. GRASP è a `cube_top_z + GRASP_Z_ABOVE_TOP`, PRE-GRASP a `cube_center_z + PRE_GRASP_LIFT`.
- **Orientazione (punto chiave):** il roll/pitch del PnP ArUco è rumoroso; usare l'intera rotazione del marker inclinerebbe l'asse di discesa, e l'end-effector **non scenderebbe verticale**. Si tiene quindi **solo lo yaw** del marker attorno a Z di `base_link`, e si ricostruisce l'orientazione da zero:
  ```
  Identity → DoRotZ(cube_yaw)  → asse verticale puro, dita parallele alle facce
           → DoRotY(π/2)       → X_gripper punta dritto verso il basso (top-down)
  ```
  Così la discesa è garantita perfettamente verticale e le dita restano parallele alle facce del cubo, indipendentemente dal rumore di roll/pitch.

### 6.2 Pose di drop (`_precompute_drop_poses`)

Per il drop non si usa un marker del cubo (il cubo è nella pinza). Il target è "`PLACE_FORWARD_OFFSET` davanti al robot, sulla superficie di place":

- **Posizione frontale:** `drop_x_base = PLACE_FORWARD_OFFSET = 0.65 m` lungo X di `base_link`. Si misura dalla **posa attuale della base**, non dalla `place_approach_pose` stale: il push place avvicina il robot di ~0.4 m **dopo** che la posa era stata latchata, quindi un drop misurato da quella sarebbe finito a ~0.24 m dalla base — troppo vicino per il braccio (OMPL "Unable to sample valid states").
- **Offset laterale:** ancorato al **centro tavolo** (marker place proiettato sull'asse Y di `base_link`), più `±PLACE_LATERAL_OFFSET = 0.15 m` a seconda del cubo (primo a +, secondo a −): separa i due cubi sulla superficie.
- **Quota:** `PLACE_TARGET_Z = PLACE_SURFACE_TOP_Z + PLACE_DROP_CLEARANCE + CUBE_SIDE + GRASP_Z_ABOVE_TOP = 0.30 + 0.02 + 0.07 + 0.04 = 0.43 m` (mappa), convertita in `base_link`; PRE-DROP è `POST_GRASP_LIFT` più in alto.
- **Orientazione:** stesso pattern verticale del grasp (`DoRotY(π/2)`).

---

## 7. Componenti nuovi in dettaglio

### 7.1 GripperController (`tiago_gripper.py`)

Wrapper di `pymoveit2.GripperInterface` con API a polling, in stile `ArmController`:
- `open()` / `close()` usano `GRIPPER_OPEN_POSITIONS = [0.045, 0.045]` e `GRIPPER_CLOSED_POSITIONS = [0.037, 0.037]`;
- `update_flags()` mantiene i latch `action_started`/`action_done` via `query_state()`.

*Perché chiusura a 0.037 e non 0:* con 0/0 le dita andrebbero ~7 cm oltre le facce (cubo espulso); con valori troppo grandi (es. 0.032) si schiaccia il cubo rigido generando forze che lo fanno ruotare. 0.037 dà un contatto piatto sul cubo da 7 cm con precarico minimo; la vera tenuta è demandata all'attach IFRA. La pausa esplicita dopo open/close serve a rendere il movimento **visibile in Gazebo** come richiesto dalla specifica d'esame.

### 7.2 LinkAttacher (`task3_link_attacher.py`)

Service client nativi rclpy verso il plugin IFRA `gazebo_ros_link_attacher`:
- `attach(cube_id)` / `detach(cube_id)` costruiscono la richiesta `(TIAGO_MODEL_NAME.gripper_left_finger_link) ↔ (CUBE_MODEL_NAMES[id].link)` e la inviano in modo asincrono (`call_async` + done callback);
- i latch `action_done` / `action_succeeded` sono letti dallo `_link_step`.

*Perché servizi nativi e non un subprocess:* condividono executor, logging e gestione errori del nodo, e si integrano nel polling senza bloccare.

### 7.3 CubeTracker (`task3_cube_tracker.py`)

Analogo all'`ArucoTracker` ma per i marker sui cubi, con gating più stretto:
- **`enabled`** — ignora ogni detection finché `enable()` non è chiamato (in `HEAD_TILT`), per non memorizzare pose da lontano durante la ricerca;
- **`frozen`** — blocca gli aggiornamenti durante il grasp;
- **gate di convergenza AMCL**, **filtro `MAX_DETECTION_DISTANCE = 4.0 m`**, **keep-closest** per cubo;
- **composizione temporale con fallback:** si cerca `map ← camera` al timestamp della detection; se fallisce (TF in ritardo sotto carico CPU), si ricade sul **TF più recente** — il robot è fermo durante la detection, quindi il drift è trascurabile. Senza questo fallback il tracker scartava tutte le detection e si bloccava ("Waiting for cube…" per ~56 s);
- mantiene sia `cube_marker_in_map` (per la navigazione) sia `cube_marker_in_camera` (sorgente preferita anti-drift per il grasp);
- `reset_cube(id)` azzera la memoria di un cubo dopo averlo gestito.

---

## 8. Parametri chiave (`constants.py`)

| Parametro | Valore | Motivazione |
|---|---|---|
| `CUBE_PICK_SEQUENCE` | `[63, 582]` | Ordine di prelievo dei cubi |
| `CUBE_SIDE` | 0.07 m | Lato fisico del cubo |
| `CUBE_TOP_TO_CENTER` | 0.035 m | Il marker è sulla faccia superiore |
| `CUBE_APPROACH_DISTANCE` | 0.75 m | Stop del push pick davanti al cubo |
| `SAFE_NAV_DISTANCE` | 1.1 m | Riferimento per goal cubo-centrico e back-out |
| `DRIVE_SPEED` | 0.15 m/s | Velocità del push/back-out su `/nav_vel` |
| `GRIPPER_OPEN / CLOSED` | `0.045` / `0.037` | Apertura; chiusura a contatto piatto (tenuta via attach) |
| `GRASP_Z_ABOVE_TOP` | 0.04 m | Quota del grasping_frame sopra la faccia superiore |
| `PRE_GRASP_LIFT` | 0.40 m | Pre-grasp sopra il centro cubo |
| `POST_GRASP_LIFT` | 0.30 m | Sollevamento di carry / pre-drop |
| `PLACE_SURFACE_TOP_Z` | 0.30 m | Quota della superficie di place |
| `PLACE_TARGET_Z` | 0.43 m | Derivata: surface + clearance + lato + offset |
| `PLACE_FORWARD_OFFSET` | 0.65 m | Distanza frontale del drop dalla base |
| `PLACE_APPROACH_DISTANCE` | 0.40 m | Stop del push place |
| `PLACE_LATERAL_OFFSET` | 0.15 m | Separazione laterale dei due cubi |
| `HEAD_TILT_DOWN_FOR_CUBE` | −1.0 rad | Inclinazione per inquadrare il tavolo |
| `HEAD_SCAN_PANS` / `DWELL` | `[0.0, -1.3, 1.3]` / 3.0 s | Scan di ricerca del cubo/marker |
| `ARM_MOTION_TIMEOUT` | 20 s | Timeout per goal MoveIt di manipolazione |
| `GRIPPER_TIMEOUT` / `ATTACH_TIMEOUT` | 5 s / 3 s | Timeout chiusura pinza / attach |

---

## 9. Problemi affrontati e soluzioni (cronologico)

| # | Problema | Causa | Soluzione |
|---|---|---|---|
| 1 | Nav2 si ferma lontano dal cubo | inflation_radius della superficie | **PUSH** closed-loop su `/nav_vel` (dopo aver scartato `/drive_on_heading` e `/cmd_vel`); velocità 0.15 m/s |
| 2 | Dopo il push, Nav2 non pianifica / DWB thrasha | Robot dentro l'inflation della superficie | **BACK_OUT** fino a `SAFE_NAV_DISTANCE+0.5` prima della prossima nav |
| 3 | Cubo afferrato di lato, braccio urta il vicino, DWB bloccato | Push dritto + tolleranza yaw di Nav2 → robot non frontale | **Termine di sterzata** nel push (ω ∝ errore di heading) |
| 4 | Stallo "Waiting for cube…" (~56 s) | Lookup TF allo stamp esatto fallisce sotto carico CPU | **Fallback al TF più recente** (robot fermo → drift trascurabile) |
| 5 | OMPL "Unable to sample valid states" in pre-grasp | Cubo al limite del workspace | Risolto dalla **sterzata** (robot frontale → cubo più vicino e centrato) |
| 6 | Un dito urtava il cubo | Asse di apertura dita allineato alla linea di approccio | Orientazione **DoRotY(π/2)** → asse di apertura perpendicolare, dita ai lati |
| 7 | Dita che intersecano il tavolo nella discesa | Mira al centro cubo, dita lunghe ~10 cm | **`GRASP_Z_ABOVE_TOP`**: stop sopra la faccia superiore |
| 8 | Cubo che scivola / ruota in presa | Chiusura sbagliata (0/0 espelle, 0.032 schiaccia) | **`GRIPPER_CLOSED = 0.037`** (contatto piatto) + tenuta via attach IFRA |
| 9 | Cubo che "glitcha" + base che thrasha dopo la chiusura | Attach scattava mentre le dita erano in moto | Chiusura con **`wait=True`** (assestamento) prima dell'attach |
| 10 | DROP fallisce OMPL (collisione tavolo) | `PLACE_TARGET_Z` stale | Derivato in modo coerente da surface + clearance + lato + offset |
| 11 | DROP troppo vicino alla base (~0.24 m) | Calcolato dalla `place_approach_pose` stale | **Drop misurato dalla posa attuale della base** + `PLACE_FORWARD_OFFSET` |
| 12 | Duplicazione degli stati push/back-out | Tre stati quasi identici (pick/place) | **Fasi parametriche** `PUSH`/`BACK_OUT` con flag `_push_mode` |
| 13 | Drop fuori dal tavolo | Marker place localizzato da lontano (errore PnP) | **`REFRESH_PLACE`**: ri-localizza il marker da ~1 m prima del push/drop |
| 14 | Grasp non deterministico (a volte storto) | Yaw PnP del marker oscilla ±5° tra run | In `_precompute_grasp_poses` si scarta roll/pitch e si tiene solo lo yaw (Identity→DoRotZ→DoRotY(π/2)); head scan se il cubo non è subito in FOV |
