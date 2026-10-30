#!/usr/bin/env python3
"""Registra un video MP4 della Milestone v1: shuttle iseAuto guidato dal Behavior Tree in MetaDrive.

Usa la stessa distanza del bridge (closest_obstacle_ahead) e la stessa logica del brain_node
replicata in test/sim_senza_ros.py; sovrappone a ogni fotogramma distanza, pedale, velocita'
e nodo attivo dell'albero.

Uso:  python tools/record_video.py --seeds 0 2 4 --out media/milestone_v1.mp4
Richiede: pip install imageio imageio-ffmpeg
"""
import argparse
import os
import sys
import types

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))

# metadrive_bridge importa rclpy: per riusarne solo la funzione di distanza bastano moduli vuoti
for name in ["rclpy", "rclpy.node", "std_msgs", "std_msgs.msg", "geometry_msgs", "geometry_msgs.msg"]:
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["rclpy.node"].Node = object
sys.modules["std_msgs.msg"].Float32 = object
sys.modules["geometry_msgs.msg"].Twist = object

from metadrive_bridge import closest_obstacle_ahead  # noqa: E402
from scenario import IseAutoEnv, make_config, get_ego, is_crash  # noqa: E402

FPS = 30
DT = 1.0 / FPS
CRUISE, BRAKE_MAX, START, STOP, JERK, CLEAR_S = 0.40, -1.0, 30.0, 5.0, 0.08, 1.2


class Brain:
    """Stessa logica di brain_node.cpp (vedi test/sim_senza_ros.py), con il nome del nodo attivo."""

    def __init__(self):
        self.pedal, self.running, self.clear_since, self.t = 0.0, False, None, 0.0

    def tick(self, d):
        self.t += DT
        if self.running or d < START:
            if not self.running:                       # onStart
                self.running, self.clear_since = True, None
                if d <= STOP:
                    return self._emergency()
            if d <= STOP:
                return self._emergency()
            if d >= START:
                self.clear_since = self.clear_since if self.clear_since is not None else self.t
                if self.t - self.clear_since >= CLEAR_S:
                    self.running = False
                    return self._cruise()
            else:
                self.clear_since = None
            ratio = min(max((START - d) / (START - STOP), 0.0), 1.0)
            self.pedal += min(max(-ratio * ratio - self.pedal, -JERK), JERK)
            return self.pedal, "ApolloStopTask (frenata progressiva)"
        return self._cruise()

    def _emergency(self):
        self.running, self.pedal = False, BRAKE_MAX
        return self.pedal, "ApolloEmergencyStop (freno massimo)"

    def _cruise(self):
        self.pedal = CRUISE
        return self.pedal, "ApolloStraightDrive (crociera)"


def font(size, bold=False):
    for name in (("arialbd.ttf", "DejaVuSans-Bold.ttf") if bold else ("arial.ttf", "DejaVuSans.ttf")):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


F_TITLE, F_BIG, F_TXT = font(44, True), font(26, True), font(22)


def grab(env):
    tex = env.engine.win.getScreenshot()
    img = np.frombuffer(bytes(tex.getRamImageAs("RGB")), np.uint8)
    return img.reshape(tex.getYSize(), tex.getXSize(), 3)[::-1]


def overlay(frame, seed, d, pedal, node, kmh):
    img = Image.fromarray(frame)
    dr = ImageDraw.Draw(img, "RGBA")
    dr.rounded_rectangle((16, 16, 560, 214), 12, fill=(0, 0, 0, 165))
    dr.text((32, 26), f"Behavior Tree - scenario {seed}", font=F_BIG, fill="white")
    color = (255, 120, 90) if "Emergency" in node else (255, 210, 90) if "Stop" in node else (120, 230, 140)
    dr.text((32, 64), node, font=F_TXT, fill=color)
    dist = "strada libera" if d >= 999 else f"{d:5.1f} m"
    dr.text((32, 98), f"Distanza ostacolo: {dist}", font=F_TXT, fill="white")
    dr.text((32, 128), f"Velocita': {kmh:4.1f} km/h", font=F_TXT, fill="white")
    dr.text((32, 158), f"Pedale: {pedal:+.2f}", font=F_TXT, fill="white")
    x0, x1, y = 190, 540, 172                     # barra del pedale, zero al centro
    xm = (x0 + x1) / 2
    dr.rectangle((x0, y - 8, x1, y + 8), outline=(200, 200, 200, 255))
    xe = xm + pedal * (x1 - x0) / 2
    dr.rectangle((min(xm, xe), y - 7, max(xm, xe), y + 7),
                 fill=(120, 230, 140, 255) if pedal >= 0 else (255, 120, 90, 255))
    return np.asarray(img)


def card(w, h, lines, n_frames):
    img = Image.new("RGB", (w, h), (18, 24, 38))
    dr = ImageDraw.Draw(img)
    y = h / 2 - 40 * len(lines)
    for i, txt in enumerate(lines):
        f = F_TITLE if i == 0 else F_TXT
        tw = dr.textlength(txt, font=f)
        dr.text(((w - tw) / 2, y), txt, font=f, fill="white" if i == 0 else (190, 200, 215))
        y += 70 if i == 0 else 38
    return [np.asarray(img)] * n_frames


def record(seed, writer, max_s, hold_s):
    cfg = make_config(render=True, seed=seed, num_scenarios=1)
    cfg["window_size"] = (1280, 720)
    env = IseAutoEnv(cfg)
    env.reset()
    ego, brain = get_ego(env), Brain()
    stopped_for, after_stop, result = 0.0, 0.0, "fine episodio"
    for _ in range(int(max_s * FPS)):
        d = closest_obstacle_ahead(env, ego)
        pedal, node = brain.tick(d)
        _, _, term, trunc, info = env.step([0.0, pedal])
        writer.append_data(overlay(grab(env), seed, d, pedal, node, ego.speed_km_h))
        if is_crash(info):
            result = "COLLISIONE"
            break
        if info.get("arrive_dest") or term or trunc:
            result = "arrivo a destinazione" if info.get("arrive_dest") else "fine episodio"
            break
        stopped_for = stopped_for + DT if (ego.speed_km_h < 0.5 and d < START) else 0.0
        if stopped_for >= 2.0:
            after_stop += DT
            result = f"arresto a {d:.2f} m"
            if after_stop >= hold_s:
                break
    env.close()
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 2, 4])
    ap.add_argument("--out", default=os.path.join(HERE, "..", "media", "milestone_v1.mp4"))
    ap.add_argument("--max-seconds", type=float, default=40.0, help="durata massima per scenario")
    ap.add_argument("--hold", type=float, default=2.0, help="secondi mostrati dopo l'arresto")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    w, h = 1280, 720
    with imageio.get_writer(args.out, fps=FPS, codec="libx264", quality=7, macro_block_size=8,
                            ffmpeg_params=["-pix_fmt", "yuv420p"]) as writer:
        for f in card(w, h, ["Shuttle iseAuto - Milestone v1",
                             "Behavior Tree C++ (BehaviorTree.CPP v3) + ROS 2 + MetaDrive",
                             "Crociera a 20 km/h, frenata progressiva da 30 m, arresto a 5 m"], 3 * FPS):
            writer.append_data(f)
        for seed in args.seeds:
            for f in card(w, h, [f"Scenario {seed}", "ostacolo fermo sulla corsia"], int(1.5 * FPS)):
                writer.append_data(f)
            res = record(seed, writer, args.max_seconds, args.hold)
            print(f"scenario {seed}: {res}", flush=True)
        for f in card(w, h, ["github.com/alesca18/iseauto-behavior-tree",
                             "Codice, istruzioni e risultati riproducibili"], 3 * FPS):
            writer.append_data(f)
    print("video salvato in", os.path.abspath(args.out))


if __name__ == "__main__":
    main()
