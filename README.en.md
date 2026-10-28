# iseauto_bt: Milestone v1

🇮🇹 [Versione italiana](README.md)

ROS 2 Humble package for the iseAuto shuttle: a C++ Behavior Tree (BehaviorTree.CPP v3) for longitudinal control, a passive bridge to the MetaDrive simulator, and a PPO agent used as a baseline for comparison.

This is the code of the bachelor's thesis *"Modellazione e analisi di alberi di comportamento per la guida autonoma"* (Universitas Mercatorum). The thesis is written in Italian; this README is enough to build, run and reproduce the results.

> **Provenance.** The original code was lost. `bt_trees/iseauto_tree.xml` is the recovered original. `src/brain_node.cpp.recuperato` is the recovered original skeleton. Everything else was **rebuilt from the thesis**. The results reported in the final version of the thesis (Chapter 5) are the ones produced by this code, and can be regenerated with the commands below.

## Layout

| File | Role |
|---|---|
| `bt_trees/iseauto_tree.xml` | BTCPP v3 tree (Listing 4.1 of the thesis) |
| `src/brain_node.cpp` | C++ node: subscribes to `/closest_obstacle`, ticks at 30 Hz, publishes `/cmd_vel` |
| `scripts/metadrive_bridge.py` | Passive bridge: telemetry and actuation, statistics over the 50 test scenarios |
| `scripts/scenario.py` | Shared MetaDrive setup (straight road, obstacles, spawn at 0 km/h, fixed shuttle parameters) |
| `scripts/train_ppo.py` / `eval_ppo.py` | Training and evaluation of the PPO agent |
| `scripts/kill_ghosts.sh` | Cleans up leftover ("ghost") ROS processes (Sec. 3.3) |
| `launch/iseauto_v1.launch.py` | Starts bridge and brain together |
| `test/sim_senza_ros.py` | ROS-free test bench (see below) |
| `test/ros_integration_test.sh` | Real ROS 2 integration test |

`/cmd_vel` convention: `linear.x` = pedal in [-1, 1] (positive throttle, negative brake), `angular.z` = steering in [-1, 1]. The MetaDrive action is `[angular.z, linear.x]`.

## Installation (Ubuntu 22.04 / WSL2)

```bash
sudo apt install ros-humble-desktop ros-humble-behaviortree-cpp-v3
pip install -r requirements.txt

mkdir -p ~/ros2_ws/src && cp -r iseauto_bt ~/ros2_ws/src/
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select iseauto_bt && source install/setup.bash
chmod +x src/iseauto_bt/scripts/*.py src/iseauto_bt/scripts/*.sh
```

## Running

```bash
ros2 run iseauto_bt kill_ghosts.sh                       # before each session
ros2 launch iseauto_bt iseauto_v1.launch.py episodes:=50
```

Or in two terminals: `ros2 run iseauto_bt metadrive_bridge.py --episodes 50` and `ros2 run iseauto_bt brain_node`.
At the end the bridge prints the collision rate and the stopping distance (mean/min/max). The brain prints the worst-case tick time (WCET) every 5 s.

## Watching the simulation

The MetaDrive 3D window opens automatically with the bridge. Use `--headless` to run without graphics (as in CI).

| What | Command |
|---|---|
| Behavior Tree with ROS 2 (Ubuntu / WSL2) | `ros2 launch iseauto_bt iseauto_v1.launch.py episodes:=5` |
| Behavior Tree without ROS (Windows too) | `python test/sim_senza_ros.py scripts 5 --render` |
| PPO agent | `python3 scripts/eval_ppo.py --model models/ppo_iseauto_600k --episodes 5 --render` |

- **Without ROS:** with `--render` the test runs in real time (30 Hz) so it can be followed. Without the flag it runs as fast as possible with no window.
- **WSL2 on Windows 11:** the window appears through WSLg with no configuration. If it does not open, check that `echo $DISPLAY` is not empty and update WSL with `wsl --update` from PowerShell.
- **Headless server or container:** use `--headless`.
- **Window requirement:** MetaDrive needs `panda3d-gltf` between 0.13 and 1.0. With 1.x the headless test works, but the window fails with `Could not load Actor model ... scene.gltf`. Fix: `pip install "panda3d-gltf>=0.13,<1.0"`.

## PPO agent

```bash
python3 scripts/train_ppo.py --episodes 100      # saves models/ppo_iseauto.zip
python3 scripts/eval_ppo.py --episodes 50        # collisions, mean reward, inference time
```

## ROS-free test bench

`test/sim_senza_ros.py` runs the real `metadrive_bridge.py`, with ROS replaced by stub modules, connected to a line-by-line Python port of `brain_node.cpp`. It uses a simulated clock. It is meant for trying the scenario and the control law where ROS 2 is not available (Windows included).

```bash
python test/sim_senza_ros.py scripts 50
```

## Results

Methodology: 50 fixed scenarios (seeds 0–49), one run per scenario; values are aggregated over the 50 scenarios (rates, means, min–max). The PPO agent is trained on 100 different scenarios (seeds 100–199). Obtained on 28/10/2026 (Windows, Python 3.14, MetaDrive 0.4.3 from GitHub, SB3 2.9.2):

| | Behavior Tree (calibrated brake) | PPO, 100 episodes | PPO, 600k steps (~1180 episodes) |
|---|---|---|---|
| Collisions over 50 tests | 0/50 | 0/50 (always leaves the road before reaching obstacles) | 16/50 (32%) |
| Outcome | 36 stops, 14 arrivals at destination | 50 off-road | - |
| Stopping distance | 5.04 m mean (5.01–5.07), std 0.018 m | - | - |
| Mean evaluation reward | - | +47.1 | +383.4 |
| Time per decision | 0.18 ms max (Python port) | 0.24 ms mean, 1.09 ms max | 0.22 ms mean, 1.18 ms max |

The 100-episode PPO uses the best of 5 tried configurations (`n_steps=1024`, `lr=5e-4`). Mean reward during the long training run, in blocks of 100 episodes:

| Episodes | 100 | 300 | 500 | 600 | 700 | 900 | 1100 |
|---|---|---|---|---|---|---|---|
| Mean reward | +30 | +39 | +74 | +127 | +189 | +206 | +195 |

The 600k-step model is in `models/ppo_iseauto_600k.zip`:

```bash
python3 scripts/train_ppo.py --timesteps 600000        # long training, progress report every 100 episodes
python3 scripts/eval_ppo.py --model models/ppo_iseauto_600k
```

For reference, MetaDrive's built-in expert driver (IDM) gets a mean reward of +447 on this scenario.

## Continuous integration (GitHub Actions)

`.github/workflows/ci.yml` runs on every push in a `ros:humble` container (Ubuntu 22.04):

1. `colcon build` of the package, including the C++ `brain_node`;
2. ROS-free test (`test/sim_senza_ros.py`, 20 scenarios);
3. real ROS 2 integration test (`test/ros_integration_test.sh`, 10 scenarios): `brain_node` and `metadrive_bridge.py` talk over topics. It fails on any collision or if a stop falls outside 4.8–5.2 m;
4. quick PPO training and evaluation run.

Results of the real ROS 2 integration test in CI (three runs of the first 10 scenarios, compiled C++ `brain_node`): 0 collisions, stops between 4.87 and 5.07 m, tick WCET at most 0.565 ms. In stops below 5 m, `ApolloEmergencyStop` kicks in (pedal −1.0).

The integration test can also be run by hand, after `colcon build` and `source install/setup.bash`:

```bash
bash test/ros_integration_test.sh 10
```

### Brake calibration

MetaDrive randomizes the vehicle's brake force at every episode (about 90–170). At those values the brake already saturates at around −0.3 pedal (about 9 m/s²), so with the thesis' quadratic ramp the shuttle stopped about 20 m from the obstacle.

`IseAutoEnv` now fixes the shuttle parameters (`SHUTTLE_PARAMS` in `scenario.py`). `max_brake_force` is calibrated so that the unchanged control law of Chapter 4 stops the shuttle at 5 m:

| `max_brake_force` | 8.0 | 8.6 | **8.7** | 8.8 | 9.0 | 10.0 |
|---|---|---|---|---|---|---|
| Mean stop over 50 tests [m] | 4.32 | 4.95 | **5.04** | 5.14 | 5.32 | 6.17 |

With 8.7, pedal −1.0 gives about 1.9 m/s² (stop from 19.5 km/h in 7.9 m). That is a comfortable service brake for a shuttle, but weaker than a real vehicle's emergency brake. For obstacles appearing suddenly closer than about 8 m the shuttle could not stop in time.

Windows installation: `pip install --no-deps git+https://github.com/metadriverse/metadrive.git`, then the dependencies by hand, with `pygame-ce` instead of `pygame`. Version 0.2.6 on PyPI is too old.

## Reconstruction choices

- **`duration="1.2"` of `ApolloStopTask`**: the original meaning is not documented. Here it is the time (in seconds) the road must stay clear (d ≥ 30 m) before cruising resumes. It is needed because the BTCPP v3 `Sequence` does not re-evaluate the condition while a child is RUNNING.
- **Braking ramp**: starts from the last published pedal (+0.40) and moves towards `p_target(d)` with a maximum step of 0.08 per tick.
- **Fail-safe**: with no telemetry for 0.5 s the brain treats it as an obstacle at 0 m and brakes. With no `/cmd_vel` for 0.5 s the bridge brakes. Before the first command the shuttle stays still.
- **Speed**: the 0.40 pedal is constant, so cruise speed is capped at 20 km/h by `IseAutoEnv` (`scenario.py`). In MetaDrive `max_speed_km_h` is a vehicle-type parameter, not a `vehicle_config` key.
- **Simulation step**: `physics_world_step_size=1/60` and `decision_repeat=2` give an `env.step` of 1/30 s, so the 30 Hz bridge runs in real time, aligned with the BT tick. With MetaDrive's default (0.1 s) the simulation would run 3 times faster than real time.
- **Obstacle distance**: computed geometrically (bumper to bumper) on the MetaDrive objects inside a ±1.75 m corridor ahead of the shuttle, with a 50 m range.
- The configuration keys (`num_scenarios`, `start_seed`, `spawn_velocity`, `env.agent`) follow MetaDrive ≥ 0.4. Older versions may use different names (`environment_num`, `env.vehicle`).

## License

Released under the MIT license: see [LICENSE](LICENSE).
