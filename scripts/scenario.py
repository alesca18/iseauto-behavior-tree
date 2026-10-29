"""Configurazione comune dello scenario MetaDrive (Capitolo 5).

Percorso rettilineo con ostacoli stazionari (accident_prob) e traffico dinamico,
usato sia dal bridge per il Behavior Tree sia dall'addestramento PPO.
"""
import os
import shutil
from pathlib import Path

from metadrive.component.vehicle import vehicle_type as md_vehicle_type
from metadrive.component.vehicle.vehicle_type import DefaultVehicle
from metadrive.engine.asset_loader import AssetLoader
from metadrive.envs import MetaDriveEnv

# Parametri fissi dello shuttle iseAuto. MetaDrive li estrae a caso a ogni episodio
# (es. max_brake_force tra ~90 e ~170) e non li accetta in vehicle_config, quindi
# vengono applicati al veicolo ego dopo ogni reset.
SHUTTLE_PARAMS = {
    "max_speed_km_h": 20,       # limite operativo iseAuto (15-20 km/h)
    "max_engine_force": 800.0,
    "max_brake_force": 9.9,     # calibrato: vedi README, "Calibrazione del freno"
}


ASSET_FILES = ("vehicle.gltf", "vehicle.bin", "right_tire_front.gltf", "right_tire_front.bin",
               "right_tire_back.gltf", "right_tire_back.bin")


class IseAutoVehicle(DefaultVehicle):
    """Shuttle iseAuto: dimensioni, massa e modello 3D del veicolo reale (TalTech)."""
    DEFAULT_LENGTH = 3.60
    DEFAULT_WIDTH = 1.50
    DEFAULT_HEIGHT = 2.40
    MASS = 1250
    TIRE_RADIUS = 0.30
    TIRE_WIDTH = 0.17
    FRONT_WHEELBASE = 1.275
    REAR_WHEELBASE = 1.275
    LATERAL_TIRE_TO_CENTER = 0.68
    path = ("iseauto/vehicle.gltf", (1, 1, 1), (0, -1.25, 0.27), (180, 0, 0))  # asset, scala, offset, HPR


def _asset_source():
    """Cartella assets/iseauto del pacchetto: sorgente o share/ dopo colcon install."""
    here = Path(__file__).resolve().parent
    candidates = [here.parent / "assets" / "iseauto"]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("iseauto_bt")) / "assets" / "iseauto")
    except Exception:
        pass
    for c in candidates:
        if (c / "vehicle.gltf").is_file():
            return c
    raise FileNotFoundError("assets/iseauto non trovato (cercato in: %s)" % ", ".join(map(str, candidates)))


def install_iseauto_assets():
    """Copia il modello 3D in metadrive/assets/models/iseauto, dove MetaDrive lo cerca."""
    src = _asset_source()
    dst = Path(AssetLoader.asset_path) / "models" / "iseauto"
    dst.mkdir(parents=True, exist_ok=True)
    for name in ASSET_FILES:
        s, d = src / name, dst / name
        if not d.is_file() or os.path.getsize(d) != os.path.getsize(s):
            shutil.copyfile(s, d)


# registra il tipo "iseauto" accanto a quelli di MetaDrive (senza modificarne i sorgenti)
md_vehicle_type.vehicle_type["iseauto"] = IseAutoVehicle
md_vehicle_type.vehicle_class_to_type[IseAutoVehicle] = "iseauto"


class IseAutoEnv(MetaDriveEnv):
    """MetaDriveEnv con i parametri dinamici dello shuttle applicati al veicolo ego."""

    def __init__(self, config=None):
        if config and config.get("use_render"):
            install_iseauto_assets()
        super().__init__(config)

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
            vehicle_model="iseauto",    # IseAutoVehicle: dimensioni e massa reali
            spawn_velocity=[0.0, 0.0],  # lo shuttle parte fermo (Sez. 3.3)
        ),
    )


def get_ego(env):
    """Veicolo ego: env.agent nelle versioni recenti di MetaDrive, env.vehicle nelle precedenti."""
    return getattr(env, "agent", None) or env.vehicle


def is_crash(info):
    return bool(info.get("crash") or info.get("crash_vehicle") or info.get("crash_object"))
