#!/usr/bin/env python3
"""Valutazione dell'agente PPO sugli stessi 50 scenari di test del Behavior Tree (Tabella 5.1)."""
import argparse
import os
import sys
import time

from stable_baselines3 import PPO

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario import IseAutoEnv, make_config, is_crash  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/ppo_iseauto")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()

    env = IseAutoEnv(make_config(render=args.render, seed=args.seed, num_scenarios=args.episodes))
    model = PPO.load(args.model)

    collisions, rewards, infer_ms = 0, [], []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + ep)
        done, total, crashed = False, 0.0, False
        while not done:
            t0 = time.perf_counter()
            action, _ = model.predict(obs, deterministic=True)
            infer_ms.append((time.perf_counter() - t0) * 1000.0)
            obs, reward, terminated, truncated, info = env.step(action)
            total += reward
            crashed = crashed or is_crash(info)
            done = terminated or truncated
        collisions += int(crashed)
        rewards.append(total)
        print(f"Test {ep + 1}/{args.episodes}: reward {total:+.2f} {'COLLISIONE' if crashed else ''}")

    n = args.episodes
    print(f"\nCollisioni: {collisions}/{n} ({100.0 * collisions / n:.1f}%)")
    print(f"Reward medio: {sum(rewards) / n:+.2f}")
    print(f"Tempo di inferenza per passo: medio {sum(infer_ms) / len(infer_ms):.2f} ms, "
          f"max {max(infer_ms):.2f} ms")
    env.close()


if __name__ == "__main__":
    main()
