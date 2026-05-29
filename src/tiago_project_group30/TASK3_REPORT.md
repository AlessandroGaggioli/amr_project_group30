# Task 3 — Autonomous Mobile Robotics Exam, Group 30

## Obiettivo

Il Task 3 estende la logica del Task 2 integrandovi una sequenza di pick-and-place autonomo per due cubi ArUco. Partendo dal robot inizializzato in posizione casuale, i requisiti sono:
1. Eseguire l'intero Task 2 (localizzazione globale AMCL, ricerca dei waypoint, rilevamento dei marker a parete e navigazione verso la posa di approccio PICK).
2. Rilevare il marker sul primo cubo da manipolare (ID 63).
3. Afferrare il cubo tramite MoveIt2 e l'interfaccia `/ATTACHLINK`.
4. Navigare verso la posa di approccio PLACE (già calcolata nel Task 2).
5. Depositare il cubo sulla superficie di destinazione sganciandolo tramite `/DETACHLINK`.
6. Ripetere la sequenza per il secondo cubo (ID 582).

---

## Architettura software

L'architettura riutilizza i componenti del Task 2, aggiungendo tre nuovi moduli e portando i thread dell'executor a 5 per gestire i nuovi task concorrenti (es. attacchi ai link virtuali e manipolazione del gripper):

| Componente | File | Responsabilità |
|---|---|---|
| Tutti i nodi Task 2 | `task2_*.py` / `tiago_arm.py` | Navigazione, AMCL, ArUco a parete, move_group arm (MoveIt2). |
| `GripperController` | `tiago_gripper.py` | Controllo dita del gripper via action server (`gripper_controller/joint_trajectory`). |
| `LinkAttacher` | `task3_link_attacher.py` | Chiamate asincrone ai servizi `/ATTACHLINK` e `/DETACHLINK`. |
| `CubeTracker` | `task3_cube_tracker.py` | Rilevamento ArUco dei marker posti sopra ai cubi, filtrato con gating spaziotemporale. |
| `Task3StateMachine` | `task3_state_machine.py` | Macchina a stati completa (Stati 0-6 base + Stati 10-27 per pick-and-place). |

Tutti i componenti girano all'interno del nodo principale `Task3Manager` (`task3_manager.py`).

---

## Macchina a stati

La macchina a stati riutilizza le transizioni e i concetti non bloccanti del Task 2. Gli stati `0` - `3` e `6` sono identici. Le transizioni differiscono per gli stati `4` e `5`:

- **State 4 (PICK reached)**: Una volta raggiunta la posa di approccio generica "wall-marker" PICK, la ricerca a parete termina. Per il picking però non si passa allo State 5 (PLACE), ma si salta allo **State 10** per iniziare il flusso di identificazione e presa del cubo.
- **State 5 (PLACE reached)**: Una volta navigato alla posa "wall-marker" PLACE, non termina il task, ma si salta allo **State 20** per iniziare il flusso di rilascio.

### Task 3: Pick sub-flow (State 10 - 19)

**State 10 — Head tilt**
La macchina abilita il `CubeTracker` (che finora ignorava i rilevamenti per evitare false pose calcolate da lontano sulle diagonali della stanza) e abbassa la testa di `HEAD_TILT_DOWN_FOR_CUBE` (-1.0 rad) per inquadrare adeguatamente il tavolo. Se necessario, attiva uno sweep (pan) della telecamera.

**State 11 & 19 & 27 — Wait for cube & Refine approach**
Una volta rilevato il marker del cubo desiderato nello **State 11**, il robot avvia un raffinamento di avvicinamento locale. Dal momento che la posa di approccio generica di base (Task 2) potrebbe porre il cubo fuori dal workspace limite del Tiago, il robot calcola una `NavigateToPose` a `CUBE_APPROACH_DISTANCE` (0.65 m) in linea d'aria dal centro del cubo. 
Spesso questo calcolo urta i valori di _inflation_ della costmap bloccando o interrompendo la navigazione. Pertanto lo stato lancia un completamento con `/drive_on_heading` (**State 27**) senza l'ausilio di costmap, spingendo "alla cieca" il Tiago per gli ultimi centimetri lungo la linea ideale di congiunzione, chiudendo il varco in sicurezza.

**State 12 — Pre-grasp gripper**
Si apre il gripper prima dell'approccio. Un breve ritardo è incluso esplicitamente per permettere all'animazione in Gazebo di rendersi visualmente completata.

**State 13 — Arm pre-grasp**
MoveIt2 sposta il `gripper_grasping_frame` sulla posa di PRE-GRASP: quota maggiorata di `PRE_GRASP_LIFT` (0.20 m) sul centro del cubo con orientamento ruotato tale da incalzare il piano dall'alto verso il basso (linea asse Z passante ortogonalmente rispetto ai fianchi del cubo).

**State 14 — Arm grasp**
L'end-effector scende verticalmente verso la posa definitiva (entro il baricentro in quota).

**State 15 & 16 — Close gripper & Attach link**
Il gripper si serra. Segue una chiamata asincrona servizio `/ATTACHLINK` (**State 16**) a Gazebo, che aggancia logicamente il link specificato del `CUBE_MODEL_NAMES` a garanzia di non perdere l'oggetto durante le oscillazioni del moto.

**State 17 & 18 — Post-grasp & Carry**
L'end-effector si solleva per disimpegnarsi e ripristina il braccio nella conformazione HOME per non creare conflitti col path-planning e laser scan. Il passaggio successivo riconduce allo **State 5** (Nav2 al PLACE).

---

### Task 3: Drop sub-flow (State 20 - 27)

L'arrivo allo **State 5** innesca la fasa di rilascio. A differenza del Pick, non sussiste alcun Tracker per ArUco. Lo spazio in cui effettuare il drop è statico da geometrie. 

**State 20 — Arm pre-drop**
Il modulo calcola una posizione di posizionamento pre-rilascio sulla geometria derivabile dal Nav2: `PLACE_TARGET_Z` più sollevamento in asse Z verticale, con offset lineare distaccato rispetto al Tiago di `PLACE_FORWARD_OFFSET` (0.45 m) garantendo che il luogo non sia sovrapposto allo scaffale in fondo né che costringa il corpo.

**State 21 — Arm drop**
MoveIt2 comanda l'affondo ai target di quota al limite con la collisione sul tavolo di stazionamento, mantenendo per convenzione il gripper aperto dopo un sollevamento in pre-drop per lo scalino.

**State 22 & 23 — Gripper release & Detach link**
I pin del gripper si allargano e il link col modello di Gazebo è richiamato al `/DETACHLINK` staccando il vincolo d'integrità fisico.

**State 24 & 25 — Lift & Tuck**
Esattamente come nelle manovre conclusive del Pick, il gripper risale per sgravarsi dai profili del cubo ed esegue il rientro nei footprint configurati a HOME position per eventuali prossime navigazioni libere.

**State 26 — Sequence handling**
Lo state esegue lo switch su `CUBE_PICK_SEQUENCE[1]`. L'ID corrente si aggiorna a quello del prossimo cubo da raccogliere. La macchina viene forzata a invalidare o sbiancare (`reset_cube()`) il rilevamento del Cube Tracker (evitando interferenza e l'accavallamento dei rilevamenti long-range scartati temporalmente) e torna ad eseguire le transizioni a partire dallo **State 4** per riavvicinarsi al marker predefinito di `PICK`. Esauriti tutti i cubi elencati, il sub-flow defluisce al **State 6** arrestando il `Task3Manager` a completamento.

---

## Logiche specifiche del Task 3

### Gestione dinamica dei limiti operativi (Workspace)
Uno dei problemi critici gestiti per il pick di cubi eterogenei a spaglio in posizione imprevedibile dal `PICK` anchor (come visto nello *State 11*) è il tracciato fine di avvicinamento. Il raggio d'inflazione impedisce il raggiungimento diretto tramite `Nav2`, ma il workspace massimo del manipolatore Pal Tiago non si estende al di là di un metro. 

Il codice bypassa localmente la mappa d'ostacolo aggirando il Behavior Tree: invoca un piano d'affondo in Drive puramente odometrico-inerziale dopo essere giunto in prossimità col NavigateToPose. La logica impone al controllore asincrono del robot un drive_on_heading limitato che lo traguarda fino a 0.65m dal blocco esatti in configurazione baricentrica garantendo stabilità di solver MoveIt2.

### Gating temporale sul CubeTracker
Analogamente all'esperienza del Task 2, il tracker dei cubi (sfruttando il pacchetto customizzato `CubeTracker`) raccatta i messaggi in background. Tuttavia possiede limitatori imposti dallo `_cube_approached` state flag e convergenze pre-assimilate (filtra via messaggi vecchi o precedenti al completamento di pre-condizioni come i tilting thella telecamera, evitando "falsi postivi" lontani con offset PnP esagerati e non corretti in TF/Map). La logica segue la _Best Keep Closest Observation Policy_, aggiornando iterativamente il fix se e solo se la stima PnP viene effettuosata a minore raggio euclideo dalla testa. Il calcolo della posa tiene conto della faccia +Z dei cubi impostando in rotazione un orientamento per caduta del gripper perpendicolare verso il marker (+Z ruotato pi/2 in Y locale).

### Ciclicità pulita e Polling Event-driven a N=5 Thread
Tutti gli stati di navigazione e cinematiche non sono strutturalmente implementati e riutilizzano le pipeline asincrone del framework (lab4_state_machine pattern). Vengono lanciati come chiamate asincrone One-Shot via Node Flags monitorati periodicamente con tick di 0.05ms nel multi thread Executor, portati in instanziamento da classe `Task3Manager` a ben 5 Thread attivi, per assorbire simultaneamente loop rclpy per Nav2, Moveit2, Gripper, LinkAttacher e messaggistica Sensor senza perdere reattività o incappare in code interlock non smaltite in locale.