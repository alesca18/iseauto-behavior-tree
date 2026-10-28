"""Banco di prova senza ROS: metadrive_bridge.py reale + replica Python di brain_node.cpp.

Uso: python test/sim_senza_ros.py scripts [episodi]
"""
import sys
import time
import types

SCRIPTS = sys.argv[1]
EPISODES = int(sys.argv[2]) if len(sys.argv) > 2 else 50
sys.path.insert(0, SCRIPTS)

# ---- stub minimi di rclpy / std_msgs / geometry_msgs ----
published = {}


class _Logger:
    def info(self, m): print("[bridge]", m, flush=True)
    def warn(self, m): print("[bridge WARN]", m, flush=True)


class _Pub:
    def __init__(self, topic): self.topic = topic
    def publish(self, msg): published[self.topic] = msg


class Node:
    def __init__(self, name): pass
    def create_publisher(self, t, topic, q): return _Pub(topic)
    def create_subscription(self, t, topic, cb, q): return None
    def create_timer(self, period, cb): return None
    def get_logger(self): return _Logger()
    def count_publishers(self, topic): return 1
    def destroy_node(self): pass


class Float32:
    def __init__(self, data=0.0): self.data = data


class _Vec:
    x = y = z = 0.0


class Twist:
    def __init__(self):
        self.linear, self.angular = _Vec(), _Vec()


for name in ["rclpy", "rclpy.node", "std_msgs", "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg"]:
    sys.modules[name] = types.ModuleType(name)
sys.modules["rclpy.node"].Node = Node
sys.modules["std_msgs.msg"].Float32 = Float32
sys.modules["geometry_msgs.msg"].Twist = Twist

import metadrive_bridge as mb  # noqa: E402

# Orologio simulato: ogni passo avanza di 1/30 s, cosi' duration e timeout
# seguono il tempo di simulazione anche se il test gira piu' veloce del reale.
SIM_DT = 1.0 / 30.0
sim_clock = [0.0]
mb.time = types.SimpleNamespace(monotonic=lambda: sim_clock[0])

# ---- replica di brain_node.cpp ----
CRUISE, BRAKE_MAX, START, STOP, JERK = 0.40, -1.0, 30.0, 5.0, 0.08


class Brain:
    def __init__(self):
        self.last_pedal = 0.0
        self.stop_running = False
        self.max_ramp_step = 0.0

    def tick(self, d):
        # Fallback( Sequence(Check, StopTask, EmergencyStop), Sequence(StraightDrive) )
        if self.stop_running:
            st = self.stop_on_running(d)
        elif d < START:                      # ApolloPathObstacleCheck threshold=30
            st = self.stop_on_start(d)
        else:
            st = "FAILURE"
        if st == "RUNNING":
            self.stop_running = True
            return
        self.stop_running = False
        if st == "SUCCESS":
            self.publish(BRAKE_MAX)          # ApolloEmergencyStop
        else:
            self.publish(CRUISE)             # ApolloStraightDrive

    def stop_on_start(self, d):
        self.pedal, self.duration, self.clearing = self.last_pedal, 1.2, False
        if d <= STOP:
            return "SUCCESS"
        return self.stop_on_running(d)

    def stop_on_running(self, d):
        if d <= STOP:
            return "SUCCESS"
        if d >= START:
            now = sim_clock[0]
            if not self.clearing:
                self.clearing, self.clear_since = True, now
            if now - self.clear_since >= self.duration:
                return "FAILURE"
        else:
            self.clearing = False
        ratio = min(max((START - d) / (START - STOP), 0.0), 1.0)
        target = -ratio * ratio
        delta = min(max(target - self.pedal, -JERK), JERK)
        self.max_ramp_step = max(self.max_ramp_step, abs(delta))
        self.pedal += delta
        self.publish(self.pedal)
        return "RUNNING"

    def publish(self, pedal):
        self.last_pedal = pedal
        self.cmd = Twist()
        self.cmd.linear.x = pedal


class Args:
    episodes = EPISODES
    seed = 0
    rate = 30.0
    headless = True


bridge = mb.MetaDriveBridge(Args())
brain = Brain()
tick_times = []
t_start = time.time()
try:
    while True:
        d = published["/closest_obstacle"].data if "/closest_obstacle" in published else 0.0
        t0 = time.perf_counter()
        brain.tick(d)
        tick_times.append((time.perf_counter() - t0) * 1000)
        bridge.on_cmd(brain.cmd)
        bridge.step()
        sim_clock[0] += SIM_DT
except SystemExit:
    pass
finally:
    bridge.env.close()

print(f"\nMassima variazione pedale nella rampa: {brain.max_ramp_step:.3f} (limite {JERK})")
print(f"Tempo massimo del tick (replica Python): {max(tick_times):.3f} ms")
print(f"Durata totale: {time.time() - t_start:.0f} s")
