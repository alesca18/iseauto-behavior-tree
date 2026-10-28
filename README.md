# iseauto_bt: Milestone v1

Pacchetto ROS 2 Humble per lo shuttle iseAuto: Behavior Tree C++ (BehaviorTree.CPP v3) per il controllo longitudinale, bridge passivo verso il simulatore MetaDrive e agente PPO di confronto.

> **Nota sulla provenienza.** Il codice originale è andato perso. `bt_trees/iseauto_tree.xml` è l'originale recuperato. `src/brain_node.cpp.recuperato` è lo scheletro originale ritrovato. Tutto il resto è stato **ricostruito a partire dalla tesi** (Capitoli 3, 4 e 5). I numeri riportati nella tesi (0/50 collisioni, reward PPO +102.22, ecc.) vengono dalle esecuzioni originali. Il codice ricostruito non li riprodurrà necessariamente identici.

## Struttura

| File | Ruolo |
|---|---|
| `bt_trees/iseauto_tree.xml` | Albero BTCPP v3 (Listato 4.1) |
| `src/brain_node.cpp` | Nodo C++: sottoscrive `/closest_obstacle`, tick a 30 Hz, pubblica `/cmd_vel` |
| `scripts/metadrive_bridge.py` | Bridge passivo: telemetria e attuazione, statistiche sui 50 test |
| `scripts/scenario.py` | Configurazione MetaDrive comune (rettilineo, ostacoli, spawn a 0 km/h) |
| `scripts/train_ppo.py` / `eval_ppo.py` | Addestramento (100 episodi) e valutazione (50 test) dell'agente PPO |
| `scripts/kill_ghosts.sh` | Pulizia dei processi fantasma (Sez. 3.3) |
| `launch/iseauto_v1.launch.py` | Avvia bridge e brain insieme |

Convenzione `/cmd_vel`: `linear.x` = pedale in [-1, 1] (positivo gas, negativo freno), `angular.z` = sterzo in [-1, 1]. L'azione MetaDrive è `[angular.z, linear.x]`.

## Installazione (Ubuntu 22.04 / WSL2)

```bash
sudo apt install ros-humble-desktop ros-humble-behaviortree-cpp-v3
pip install -r requirements.txt

mkdir -p ~/ros2_ws/src && cp -r iseauto_bt ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select iseauto_bt && source install/setup.bash
chmod +x src/iseauto_bt/scripts/*.py src/iseauto_bt/scripts/*.sh
```

## Esecuzione

```bash
ros2 run iseauto_bt kill_ghosts.sh                       # prima di ogni sessione
ros2 launch iseauto_bt iseauto_v1.launch.py episodes:=50
```

Oppure in due terminali: `ros2 run iseauto_bt metadrive_bridge.py --episodes 50` e `ros2 run iseauto_bt brain_node`.
Al termine il bridge stampa il tasso di collisione e la distanza d'arresto (media/min/max). Il brain stampa ogni 5 s il WCET del tick.

## Agente PPO

```bash
python3 scripts/train_ppo.py --episodes 100      # salva models/ppo_iseauto.zip
python3 scripts/eval_ppo.py --episodes 50        # collisioni, reward medio, tempo di inferenza
```

## Test senza ROS

`test/sim_senza_ros.py` fa girare il vero `metadrive_bridge.py`, con ROS sostituito da moduli finti, collegato a una replica Python riga per riga di `brain_node.cpp`. Usa un orologio simulato. Serve per provare scenario e legge di controllo dove ROS 2 non c'è (anche su Windows).

```bash
python test/sim_senza_ros.py scripts 50
```

Risultati ottenuti il 28/10/2026 (Windows, Python 3.14, MetaDrive 0.4.3 da GitHub, SB3 2.9.2):

| | Behavior Tree (freno calibrato) | PPO, 100 episodi | PPO, 600k passi (~1180 episodi) |
|---|---|---|---|
| Collisioni su 50 test | 0/50 | 0/50 (esce sempre di strada prima degli ostacoli) | 16/50 (32%) |
| Distanza d'arresto | 5.04 m media (5.01–5.07) | - | - |
| Reward medio in valutazione | - | +47.1 | +383.4 |
| Tempo per decisione | 0.18 ms max (replica Python) | 0.24 ms medio, 1.09 ms max | 0.22 ms medio, 1.18 ms max |

Il PPO con 100 episodi usa i parametri migliori tra 5 configurazioni provate (`n_steps=1024`, `lr=5e-4`). Andamento del reward medio durante l'addestramento lungo, per blocchi di 100 episodi:

| Episodi | 100 | 300 | 500 | 600 | 700 | 900 | 1100 |
|---|---|---|---|---|---|---|---|
| Reward medio | +30 | +39 | +74 | +127 | +189 | +206 | +195 |

Il modello da 600k passi è in `models/ppo_iseauto_600k.zip`:

```bash
python3 scripts/train_ppo.py --timesteps 600000        # addestramento lungo, con resoconto ogni 100 episodi
python3 scripts/eval_ppo.py --model models/ppo_iseauto_600k
```

Per confronto, il pilota esperto IDM integrato in MetaDrive ottiene un reward medio di +447 su questo scenario.

## Integrazione continua (GitHub Actions)

`.github/workflows/ci.yml` gira a ogni push in un container `ros:humble` (Ubuntu 22.04):

1. `colcon build` del pacchetto, compreso `brain_node` C++;
2. test senza ROS (`test/sim_senza_ros.py`, 20 test);
3. test di integrazione ROS 2 reale (`test/ros_integration_test.sh`, 10 test): `brain_node` e `metadrive_bridge.py` comunicano via topic. Fallisce se c'è una collisione o se un arresto cade fuori da 4.8–5.2 m;
4. prova rapida di addestramento e valutazione PPO.

Il test di integrazione si lancia anche a mano, dopo `colcon build` e `source install/setup.bash`:

```bash
bash test/ros_integration_test.sh 10
```

### Calibrazione del freno

MetaDrive estrae a caso a ogni episodio la forza frenante del veicolo (circa 90–170). A queste forze il freno satura già intorno a −0.3 di pedale (circa 9 m/s²). Con la rampa quadratica della tesi lo shuttle si fermava quindi a circa 20 m.

`IseAutoEnv` fissa ora i parametri dello shuttle (`SHUTTLE_PARAMS` in `scenario.py`). `max_brake_force` è calibrato in modo che la legge del Capitolo 4, invariata, porti all'arresto a 5 m:

| `max_brake_force` | 8.0 | 8.6 | **8.7** | 8.8 | 9.0 | 10.0 |
|---|---|---|---|---|---|---|
| Arresto medio su 50 test [m] | 4.32 | 4.95 | **5.04** | 5.14 | 5.32 | 6.17 |

Con 8.7 il pedale −1.0 corrisponde a circa 1.9 m/s² (arresto da 19.5 km/h in 7.9 m). È una frenata di servizio confortevole per uno shuttle, ma più debole del freno di emergenza di un veicolo reale. Per ostacoli che compaiono improvvisamente a meno di circa 8 m lo shuttle non riuscirebbe a fermarsi in tempo.

Installazione su Windows: `pip install --no-deps git+https://github.com/metadriverse/metadrive.git`, poi le dipendenze a mano, con `pygame-ce` al posto di `pygame`. La versione 0.2.6 su PyPI è troppo vecchia.

## Scelte di ricostruzione da verificare

- **`duration="1.2"` di `ApolloStopTask`**: la tesi non ne specifica il significato. Qui è il tempo (in secondi) per cui la strada deve restare libera (d ≥ 30 m) prima di riprendere la marcia. Serve perché la `Sequence` di BTCPP v3 non rivaluta la condizione mentre un figlio è RUNNING.
- **Rampa di frenata**: parte dall'ultimo pedale pubblicato (+0.40) e scende verso `p_target(d)` con passo massimo 0.08 per tick.
- **Fail-safe**: senza telemetria per 0.5 s il brain considera un ostacolo a 0 m e frena. Senza `/cmd_vel` per 0.5 s il bridge frena. Prima del primo comando lo shuttle resta fermo.
- **Velocità**: il pedale 0.40 è costante, quindi la velocità di crociera è limitata a 20 km/h da `IseAutoEnv` (`scenario.py`). In MetaDrive `max_speed_km_h` è un parametro del tipo di veicolo, non una chiave di `vehicle_config`.
- **Passo di simulazione**: `physics_world_step_size=1/60` e `decision_repeat=2` danno un `env.step` di 1/30 s. Così il bridge a 30 Hz gira in tempo reale ed è allineato al tick del BT. Con il default di MetaDrive (0.1 s) la simulazione andrebbe 3 volte più veloce del reale.
- **Distanza dall'ostacolo**: calcolata geometricamente (paraurti-paraurti) sugli oggetti MetaDrive dentro un corridoio di ±1.75 m davanti allo shuttle, con portata di 50 m.
- Le chiavi di configurazione (`num_scenarios`, `start_seed`, `spawn_velocity`, `env.agent`) seguono MetaDrive ≥ 0.4. Con versioni più vecchie potrebbero chiamarsi diversamente (`environment_num`, `env.vehicle`).
