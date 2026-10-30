# iseauto_bt: Milestone v1

🇬🇧 [English version](README.en.md)

Pacchetto ROS 2 Humble per lo shuttle iseAuto: Behavior Tree C++ (BehaviorTree.CPP v3) per il controllo longitudinale, bridge passivo verso il simulatore MetaDrive e agente PPO di confronto.

> **Nota sulla provenienza.** Il codice originale è andato perso. `bt_trees/iseauto_tree.xml` è l'originale recuperato. `src/brain_node.cpp.recuperato` è lo scheletro originale ritrovato. Tutto il resto è stato **ricostruito a partire dalla tesi**. I risultati riportati nella tesi consegnata (Capitolo 5) sono quelli prodotti dal codice al tag [`tesi`](https://github.com/alesca18/iseauto-behavior-tree/tree/tesi); le correzioni successive sono descritte in [Differenze rispetto alla tesi](#differenze-rispetto-alla-tesi).

## Video

[![Shuttle iseAuto guidato dal Behavior Tree in MetaDrive](media/milestone_v1.jpg)](media/milestone_v1.mp4)

Scenari 49 e 14: ostacolo sulla corsia, frenata progressiva e arresto a circa 5 m. Scenario 4: coni sulla corsia adiacente, lo shuttle prosegue. In sovrimpressione: nodo attivo dell'albero, distanza, velocità e pedale. I tratti di sola crociera sono accelerati x3. Per rigenerarlo: `python tools/record_video.py` (richiede `pip install imageio imageio-ffmpeg`).

## Struttura

| File | Ruolo |
|---|---|
| `bt_trees/iseauto_tree.xml` | Albero BTCPP v3 (Listato 4.1) |
| `src/brain_node.cpp` | Nodo C++: sottoscrive `/closest_obstacle`, tick a 30 Hz, pubblica `/cmd_vel` |
| `scripts/metadrive_bridge.py` | Bridge passivo: telemetria e attuazione, statistiche sui 50 test |
| `scripts/scenario.py` | Configurazione MetaDrive comune (rettilineo, ostacoli, spawn a 0 km/h) e veicolo `IseAutoVehicle` |
| `assets/iseauto/` | Modello 3D dello shuttle iseAuto (carrozzeria e ruote, glTF) |
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

## Vedere la simulazione

La finestra 3D di MetaDrive si apre da sola con il bridge. Senza grafica serve `--headless` (come in CI).

| Cosa | Comando |
|---|---|
| Behavior Tree con ROS 2 (Ubuntu / WSL2) | `ros2 launch iseauto_bt iseauto_v1.launch.py episodes:=5` |
| Behavior Tree senza ROS (anche su Windows) | `python test/sim_senza_ros.py scripts 5 --render` |
| Agente PPO | `python3 scripts/eval_ppo.py --model models/ppo_iseauto_600k --episodes 5 --render` |

- **Senza ROS:** con `--render` il test va in tempo reale (30 Hz) per poterlo seguire. Senza l'opzione gira alla massima velocità e senza finestra.
- **WSL2 su Windows 11:** la finestra appare tramite WSLg, senza configurare nulla. Se non si apre, controlla che `echo $DISPLAY` non sia vuoto e aggiorna WSL con `wsl --update` da PowerShell.
- **Su un server o in un container senza schermo:** usa `--headless`.
- **Requisito per la finestra:** MetaDrive richiede `panda3d-gltf` tra la 0.13 e la 1.0. Con la 1.x il test senza grafica funziona, ma la finestra si blocca con `Could not load Actor model ... scene.gltf`. Per correggere: `pip install "panda3d-gltf>=0.13,<1.0"`.

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

## Risultati

Metodologia: 50 scenari fissi (semi 0–49), un'esecuzione per scenario; i valori sono aggregati sui 50 scenari (tassi, medie, min–max). Il PPO è addestrato su 100 scenari diversi (semi 100–199) e, poiché il risultato dipende dal seme dell'addestramento, ogni configurazione è addestrata **tre volte** (`--ppo-seed` 0, 1, 2). Veicolo: `IseAutoVehicle` (3.60 × 1.50 × 2.40 m, 1250 kg). Risultati ottenuti il 29/10/2026 (Windows, Python 3.14, MetaDrive 0.4.3 da GitHub, SB3 2.9.2):

| | Behavior Tree (freno calibrato) | PPO, 100 episodi (semi 0 / 1 / 2) | PPO, 600k passi (semi 0 / 1 / 2) |
|---|---|---|---|
| Collisioni su 50 test | 0/50 | 0 / 0 / 0 (esce sempre di strada prima degli ostacoli) | 1 / 6 / 18 (media 17%) |
| Esito | 36 arresti, 14 arrivi | 50 uscite di strada in tutti e tre | arrivi 49 / 32 / 17, uscite di strada 0 / 12 / 15 |
| Distanza d'arresto | 5.05 m media (5.03–5.08), std 0.017 m | - | - |
| Reward medio in valutazione | - | +24.0 / +29.3 / +20.7 | +441.7 / +352.2 / +313.2 (media +369.0) |
| Tempo per decisione | 0.34 ms max (replica Python) | 0.2–0.3 ms medio, ≤ 1.6 ms max | 0.2–0.3 ms medio, ≤ 3.0 ms max |

Gli iperparametri del PPO (`n_steps=1024`, `lr=5e-4`) sono i migliori tra 5 configurazioni provate in una fase preliminare con il veicolo generico di MetaDrive. Andamento del reward medio durante l'addestramento lungo con seme 0, per blocchi di 100 episodi:

| Episodi | 100 | 300 | 500 | 600 | 700 | 800 | 1000 |
|---|---|---|---|---|---|---|---|
| Reward medio | +28 | +41 | +91 | +186 | +208 | +227 | +249 |

Il modello da 600k passi con seme 0 è in `models/ppo_iseauto_600k.zip`. Per rigenerare le tre ripetizioni:

```bash
for s in 0 1 2; do
  python3 scripts/train_ppo.py --timesteps 600000 --ppo-seed $s --out models/ppo600_s$s
  python3 scripts/eval_ppo.py --model models/ppo600_s$s       # collisioni, arrivi, uscite di strada
done
```

Per confronto, il pilota esperto IDM integrato in MetaDrive ottiene sugli stessi 50 scenari 49 arrivi, 0 collisioni e un reward medio di +440.7.

### Differenze rispetto alla tesi

Dopo la consegna sono state corrette due cose nel bridge. Il codice esatto della tesi resta disponibile al tag `tesi` (`git checkout tesi`).

1. **Scenari a seme fisso.** Il bridge chiamava `env.reset()` senza seme, e in quel caso MetaDrive estrae a caso uno scenario dell'intervallo a ogni reset. I 50 test del Behavior Tree riportati nella tesi sono quindi 50 estrazioni casuali tra i semi 0–49, non ciascun seme una volta (alcuni ripetuti, altri assenti), e i conteggi di arresti e arrivi cambiavano da un'esecuzione all'altra. Ora il test *i* usa il seme `seed + i` e due esecuzioni danno lo stesso risultato. Le valutazioni del PPO passavano già il seme esplicitamente e non cambiano.
2. **Corridoio del sensore.** Il corridoio era largo quanto la corsia (±1.75 m): in alcuni scenari lo shuttle si fermava davanti ai coni che chiudono la corsia adiacente, che avrebbe superato con circa 0.9 m di spazio. Ora il corridoio è largo quanto lo shuttle più 0.5 m per lato (±1.25 m).

| Behavior Tree, 50 scenari | Tesi (tag `tesi`) | Codice attuale |
|---|---|---|
| Collisioni | 0/50 | 0/50 |
| Esito | 33 arresti, 17 arrivi (estrazioni casuali) | 36 arresti, 14 arrivi (semi 0–49) |
| Distanza d'arresto | 5.05 m (5.03–5.08), std 0.018 m | 5.05 m (5.03–5.08), std 0.017 m |

Le conclusioni della tesi non cambiano: nessuna collisione e arresto a 5 m con pochi centimetri di variazione.

## Integrazione continua (GitHub Actions)

`.github/workflows/ci.yml` gira a ogni push in un container `ros:humble` (Ubuntu 22.04):

1. `colcon build` del pacchetto, compreso `brain_node` C++;
2. test senza ROS (`test/sim_senza_ros.py`, 20 test);
3. test di integrazione ROS 2 reale (`test/ros_integration_test.sh`, 10 test): `brain_node` e `metadrive_bridge.py` comunicano via topic. Fallisce se c'è una collisione o se un arresto cade fuori da 4.8–5.2 m;
4. prova rapida di addestramento e valutazione PPO.

Risultati del test di integrazione ROS 2 reale in CI (tre esecuzioni dei primi 10 scenari, `brain_node` C++ compilato): 0 collisioni; in tutte e tre gli stessi 6 arresti (scenari 0, 2, 3, 5, 8, 9) fra 4.89 e 5.05 m e 4 arrivi; WCET del tick al massimo 0.29 ms. (Nella tesi, con gli scenari estratti a caso: 6 / 6 / 8 arresti fra 4.89 e 5.08 m.) Negli arresti sotto i 5 m interviene `ApolloEmergencyStop` (pedale −1.0).

Il test di integrazione si lancia anche a mano, dopo `colcon build` e `source install/setup.bash`:

```bash
bash test/ros_integration_test.sh 10
```

### Calibrazione del freno

MetaDrive estrae a caso a ogni episodio la forza frenante del veicolo (circa 90–170). A queste forze lo shuttle decelera già di circa 2.6 m/s² con −0.1 di pedale, e con la rampa quadratica della tesi si fermava a circa 19.3 m (19.6 m previsti dall'equazione del Capitolo 4).

`IseAutoEnv` fissa ora i parametri dello shuttle (`SHUTTLE_PARAMS` in `scenario.py`). `max_brake_force` è calibrato in modo che la legge del Capitolo 4, invariata, porti all'arresto a 5 m:

| `max_brake_force` | 8.0 | 9.0 | 9.6 | 9.8 | **9.9** | 10.0 | 10.2 | 12.0 |
|---|---|---|---|---|---|---|---|---|
| Arresto medio su 50 test [m] | 3.10 | 4.23 | 4.79 | 4.97 | **5.05** | 5.13 | 5.30 | 6.59 |

Con il veicolo generico di MetaDrive (1100 kg) il valore corretto era 8.7; per lo shuttle da 1250 kg la proporzionalità forza/massa dà 8.7 · 1250/1100 ≈ 9.9, confermato dalla tabella. Con 9.9 il pedale −1.0 corrisponde a 1.90 m/s² (arresto da 20 km/h in 8.3 m). È una frenata di servizio confortevole per uno shuttle, ma più debole del freno di emergenza di un veicolo reale. Per ostacoli che compaiono improvvisamente a meno di circa 8 m lo shuttle non riuscirebbe a fermarsi in tempo.

Installazione su Windows: `pip install --no-deps git+https://github.com/metadriverse/metadrive.git`, poi le dipendenze a mano, con `pygame-ce` al posto di `pygame`. La versione 0.2.6 su PyPI è troppo vecchia.

## Scelte di ricostruzione da verificare

- **`duration="1.2"` di `ApolloStopTask`**: la tesi non ne specifica il significato. Qui è il tempo (in secondi) per cui la strada deve restare libera (d ≥ 30 m) prima di riprendere la marcia. Serve perché la `Sequence` di BTCPP v3 non rivaluta la condizione mentre un figlio è RUNNING.
- **Rampa di frenata**: parte dall'ultimo pedale pubblicato (+0.40) e scende verso `p_target(d)` con passo massimo 0.08 per tick.
- **Fail-safe**: senza telemetria per 0.5 s il brain considera un ostacolo a 0 m e frena. Senza `/cmd_vel` per 0.5 s il bridge frena. Prima del primo comando lo shuttle resta fermo.
- **Velocità**: il pedale 0.40 è costante, quindi la velocità di crociera è limitata a 20 km/h da `IseAutoEnv` (`scenario.py`). In MetaDrive `max_speed_km_h` è un parametro del tipo di veicolo, non una chiave di `vehicle_config`.
- **Passo di simulazione**: `physics_world_step_size=1/60` e `decision_repeat=2` danno un `env.step` di 1/30 s. Così il bridge a 30 Hz gira in tempo reale ed è allineato al tick del BT. Con il default di MetaDrive (0.1 s) la simulazione andrebbe 3 volte più veloce del reale.
- **Veicolo**: `IseAutoVehicle` (in `scenario.py`) eredita da `DefaultVehicle` di MetaDrive con dimensioni, massa, ruote e modello 3D dello shuttle reale, ed è registrato come `vehicle_model="iseauto"`. I sorgenti di MetaDrive non vengono modificati e il traffico resta quello predefinito. Con la grafica attiva, `scenario.py` copia `assets/iseauto/` in `metadrive/assets/models/iseauto/`, dove MetaDrive cerca i modelli.
- **Distanza dall'ostacolo**: calcolata geometricamente (paraurti-paraurti) sugli oggetti MetaDrive dentro il corridoio che lo shuttle occupa avanzando, allargato di 0.5 m per lato (±1.25 m per lo shuttle da 1.50 m, più metà della larghezza dell'oggetto), con portata di 50 m. Una prima versione usava un corridoio largo quanto la corsia (±1.75 m) e fermava lo shuttle anche davanti ai coni che chiudono la corsia adiacente.
- Le chiavi di configurazione (`num_scenarios`, `start_seed`, `spawn_velocity`, `env.agent`) seguono MetaDrive ≥ 0.4. Con versioni più vecchie potrebbero chiamarsi diversamente (`environment_num`, `env.vehicle`).

## Licenza

Distribuito con licenza MIT: vedi [LICENSE](LICENSE).
