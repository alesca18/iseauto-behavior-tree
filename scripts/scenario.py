"""Configurazione comune dello scenario MetaDrive (Capitolo 5).

Percorso rettilineo con ostacoli stazionari (accident_prob) e traffico dinamico,
usato sia dal bridge per il Behavior Tree sia dall'addestramento PPO.
"""
from metadrive.envs import MetaDriveEnv

# Parametri fissi dello shuttle iseAuto. MetaDrive li estrae a caso a ogni episodio
# (es. max_brake_force tra ~90 e ~170) e non li accetta in vehicle_config, quindi
# vengono applicati al veicolo ego dopo ogni reset.
SHUTTLE_PARAMS = {
    "max_speed_km_h": 20,       # limite operativo iseAuto (15-20 km/h)
    "max_engine_force": 800.0,
    "max_brake_force": 8.7,     # calibrato: vedi README, "Calibrazione del freno"
}


class IseAutoEnv(MetaDriveEnv):
    """MetaDriveEnv con i parametri dinamici dello shuttle applicati al veicolo ego."""

    def reset(self, *args, **kwargs):
        out = super().reset(*args, **kwargs)
        get_ego(self).config.update(SHUTTLE_PARAMS, allow_add_new_key=True)
        return out


def make_config(render=False, seed=0, num_scenarios=50):
    return dict(
        use_render=render,
        manual_control=False,
        map="SSSS",               # quattro blocchi rettilinei
        start_seed=seed,
        num_scenarios=num_scenarios,
        traffic_density=0.05,     # ostacoli dinamici
        accident_prob=0.8,        # ostacoli stazionari sulla carreggiata
        horizon=3000,
        # un env.step = 1/30 s, cosi' il bridge a 30 Hz gira in tempo reale
        # allineato al tick del Behavior Tree (default MetaDrive: 0.1 s)
        physics_world_step_size=1.0 / 60.0,
        decision_repeat=2,
        vehicle_config=dict(
            spawn_velocity=[0.0, 0.0],  # lo shuttle parte fermo (Sez. 3.3)
        ),
    )


def get_ego(env):
    """Veicolo ego: env.agent nelle versioni recenti di MetaDrive, env.vehicle nelle precedenti."""
    return getattr(env, "agent", None) or env.vehicle


def is_crash(info):
    return bool(info.get("crash") or info.get("crash_vehicle") or info.get("crash_object"))
