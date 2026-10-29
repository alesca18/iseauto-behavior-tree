#!/usr/bin/env python3
"""Addestramento dell'agente PPO End-to-End (Capitolo 5, Stable Baselines 3).

Osservazione nativa di MetaDrive a 259 dimensioni (LiDAR 240 raggi + stato),
azione [sterzo, pedale]. L'addestramento si ferma dopo --episodes episodi.
"""
import argparse
import os
import sys

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, StopTrainingOnMaxEpisodes
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scenario import IseAutoEnv, make_config  # noqa: E402


class ProgressReport(BaseCallback):
    """Stampa ogni `every` episodi il reward medio degli ultimi `every` episodi."""

    def __init__(self, every=100):
        super().__init__()
        self.every = every
        self.rewards = []

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.rewards.append(info["episode"]["r"])
                if len(self.rewards) % self.every == 0:
                    last = self.rewards[-self.every:]
                    print(f"episodi {len(self.rewards):5d}  passi {self.num_timesteps:8d}  "
                          f"reward medio {sum(last) / len(last):+.2f}", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--timesteps", type=int, default=None,
                        help="se indicato, addestra per questo numero di passi invece che per --episodes")
    parser.add_argument("--scenarios", type=int, default=100, help="scenari MetaDrive distinti")
    parser.add_argument("--seed", type=int, default=100, help="seed distinti da quelli di test")
    parser.add_argument("--ppo-seed", type=int, default=None,
                        help="seed della rete e del campionamento PPO (default: --seed)")
    parser.add_argument("--out", default="models/ppo_iseauto")
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--n-steps", type=int, default=1024, help="passi raccolti per ogni aggiornamento")
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--n-epochs", type=int, default=10)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    args = parser.parse_args()

    env = Monitor(IseAutoEnv(make_config(seed=args.seed, num_scenarios=args.scenarios)))
    print("Spazio delle osservazioni:", env.observation_space.shape)

    # SB3 passa il seed anche a env.reset(), dove MetaDrive lo interpreta come indice di scenario:
    # un --ppo-seed diverso inizializza quindi solo rete e campionamento, senza toccare gli scenari.
    if args.ppo_seed is not None:
        set_random_seed(args.ppo_seed)
    model = PPO("MlpPolicy", env, learning_rate=args.lr, n_steps=args.n_steps,
                batch_size=args.batch_size, n_epochs=args.n_epochs, ent_coef=args.ent_coef,
                gamma=0.99, verbose=0,
                seed=args.seed if args.ppo_seed is None else None, tensorboard_log="logs/ppo")
    if args.timesteps:
        model.learn(total_timesteps=args.timesteps, callback=ProgressReport(every=100))
    else:
        model.learn(total_timesteps=10_000_000,
                    callback=StopTrainingOnMaxEpisodes(max_episodes=args.episodes, verbose=1))

    rewards = env.get_episode_rewards()
    if rewards:
        last = rewards[-10:]
        print(f"Episodi: {len(rewards)}  reward medio ultimi {len(last)}: {sum(last) / len(last):+.2f}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    model.save(args.out)
    print("Modello salvato in", args.out + ".zip")
    env.close()


if __name__ == "__main__":
    main()
