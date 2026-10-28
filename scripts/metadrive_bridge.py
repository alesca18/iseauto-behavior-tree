#!/usr/bin/env python3
"""Bridge passivo MetaDrive <-> ROS 2 (Capitolo 3).

Non prende decisioni: genera la telemetria /closest_obstacle (std_msgs/Float32)
e applica i comandi /cmd_vel (geometry_msgs/Twist) ricevuti dal brain_node C++,
con a_t = [angular.z, linear.x] = [sterzo, pedale].
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario import IseAutoEnv, make_config, get_ego, is_crash  # noqa: E402

NO_OBSTACLE = 999.0
CORRIDOR_HALF_WIDTH = 1.75   # meta' corsia [m]
SENSOR_RANGE = 50.0          # portata del LiDAR virtuale [m]
CMD_TIMEOUT = 0.5            # [s] senza comandi => freno
STOP_SPEED_KMH = 0.5
STOP_HOLD_S = 2.0            # fermo per questo tempo => test concluso
START_DIST = 30.0


def _size(obj, attr, fallback):
    try:
        return float(getattr(obj, attr))
    except Exception:
        return fallback


def closest_obstacle_ahead(env, ego):
    """Distanza (paraurti-paraurti) dall'oggetto piu' vicino nel corridoio di avanzamento."""
    ex, ey = ego.position
    c, s = math.cos(ego.heading_theta), math.sin(ego.heading_theta)
    ego_half_len = _size(ego, "top_down_length", 4.5) / 2.0
    best = NO_OBSTACLE
    for obj in env.engine.get_objects().values():
        if obj is ego or not hasattr(obj, "position"):
            continue
        dx, dy = obj.position[0] - ex, obj.position[1] - ey
        x = dx * c + dy * s        # longitudinale nel frame ego
        y = -dx * s + dy * c       # laterale nel frame ego
        half_w = _size(obj, "top_down_width", 1.0) / 2.0
        half_len = _size(obj, "top_down_length", 1.0) / 2.0
        if x <= 0.0 or abs(y) > CORRIDOR_HALF_WIDTH + half_w:
            continue
        gap = max(0.0, x - ego_half_len - half_len)
        if gap <= SENSOR_RANGE:
            best = min(best, gap)
    return best


class MetaDriveBridge(Node):
    def __init__(self, args):
        super().__init__("metadrive_bridge")
        self.args = args
        self.obstacle_pub = self.create_publisher(Float32, "/closest_obstacle", 10)
        self.speed_pub = self.create_publisher(Float32, "/ego_speed", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

        self.action = [0.0, 0.0]
        self.last_cmd_time = None
        self.timeout_warned = False

        self.env = IseAutoEnv(make_config(render=not args.headless, seed=args.seed,
                                            num_scenarios=args.episodes))
        self.env.reset()
        self.get_logger().info(f"MetaDrive avviato, osservazione {self.env.observation_space.shape}")

        self.episode = 0
        self.collisions = 0
        self.stop_distances = []
        self._reset_episode_state()

        self.create_timer(1.0 / args.rate, self.step)
        self.create_timer(5.0, self.check_ghost_publishers)

    def _reset_episode_state(self):
        self.stopped_since = None
        self.was_moving = False

    def on_cmd(self, msg):
        self.action = [max(-1.0, min(1.0, msg.angular.z)), max(-1.0, min(1.0, msg.linear.x))]
        self.last_cmd_time = time.monotonic()
        self.timeout_warned = False

    def check_ghost_publishers(self):
        n = self.count_publishers("/cmd_vel")
        if n > 1:
            self.get_logger().warn(f"{n} publisher su /cmd_vel: possibili nodi fantasma, "
                                   "eseguire scripts/kill_ghosts.sh")

    def step(self):
        if self.last_cmd_time is None:
            action = [0.0, 0.0]          # nessun comando ancora: shuttle fermo
        elif time.monotonic() - self.last_cmd_time > CMD_TIMEOUT:
            action = [0.0, -1.0]         # brain non risponde: freno
            if not self.timeout_warned:
                self.get_logger().warn("Nessun /cmd_vel recente: freno applicato")
                self.timeout_warned = True
        else:
            action = self.action

        _, _, terminated, truncated, info = self.env.step(action)
        ego = get_ego(self.env)
        dist = closest_obstacle_ahead(self.env, ego)
        speed = float(ego.speed_km_h)
        self.obstacle_pub.publish(Float32(data=float(dist)))
        self.speed_pub.publish(Float32(data=speed))

        if speed > STOP_SPEED_KMH:
            self.was_moving = True
            self.stopped_since = None
        elif self.was_moving and dist < START_DIST:
            self.stopped_since = self.stopped_since or time.monotonic()

        crashed = is_crash(info)
        stopped = self.stopped_since is not None and time.monotonic() - self.stopped_since >= STOP_HOLD_S
        if crashed or stopped or terminated or truncated:
            self.finish_episode(crashed, dist if stopped else None)

    def finish_episode(self, crashed, stop_distance):
        self.episode += 1
        self.collisions += int(crashed)
        if stop_distance is not None:
            self.stop_distances.append(stop_distance)
        esito = "COLLISIONE" if crashed else (f"arresto a {stop_distance:.2f} m"
                                              if stop_distance is not None else "fine episodio")
        self.get_logger().info(f"Test {self.episode}/{self.args.episodes}: {esito}")

        if self.episode >= self.args.episodes:
            self.print_summary()
            raise SystemExit
        self.env.reset()
        self._reset_episode_state()

    def print_summary(self):
        n = self.episode
        log = self.get_logger()
        log.info(f"Collisioni: {self.collisions}/{n} ({100.0 * self.collisions / n:.1f}%)")
        if self.stop_distances:
            d = self.stop_distances
            log.info(f"Distanza d'arresto: media {sum(d) / len(d):.2f} m, "
                     f"min {min(d):.2f} m, max {max(d):.2f} m")

    def destroy_node(self):
        self.env.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--rate", type=float, default=30.0, help="passi di simulazione al secondo")
    parser.add_argument("--headless", action="store_true")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MetaDriveBridge(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
