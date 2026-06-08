import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = PROJECT_ROOT / "data" / "missiles"
CONFIG_PATH = CONFIG_DIR / "configurations.json"


FIELD_ORDER = [
    "cruise_speed",
    "min_speed",
    "max_speed",
    "max_acceleration",
    "min_altitude",
    "max_altitude",
    "max_g_force",
    "sustained_turn_rate",
    "sustained_g_force",
    "evasive_turn_rate",
]


FIELD_UNITS = {
    "cruise_speed": "km/h",
    "min_speed": "km/h",
    "max_speed": "km/h",
    "max_acceleration": "m/s^2",
    "min_altitude": "m AGL",
    "max_altitude": "m AGL",
    "max_g_force": "g",
    "sustained_turn_rate": "deg/s",
    "sustained_g_force": "g",
    "evasive_turn_rate": "deg/s",
}


DEFAULT_CONFIGURATION = {
    "name": "Tomahawk Block V",
    "cruise_speed": 800.0,
    "min_speed": 400.0,
    "max_speed": 920.0,
    "max_acceleration": 9.8,
    "min_altitude": 30.0,
    "max_altitude": 1200.0,
    "max_g_force": 6.89,
    "sustained_turn_rate": 8.0,
    "sustained_g_force": 2.0,
    "evasive_turn_rate": 25.0,
}


def _default_payload() -> dict:
    return {"configurations": [DEFAULT_CONFIGURATION.copy()]}


def ensure_store() -> Path:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    if not CONFIG_PATH.exists():
        with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
            json.dump(_default_payload(), handle, indent=2)
    return CONFIG_PATH


def validate_configuration(config: dict) -> dict:
    name = str(config.get("name", "")).strip()
    if not name:
        raise ValueError("Configuration name is required.")

    normalized = {"name": name}
    for field in FIELD_ORDER:
        if field not in config:
            raise ValueError(f"Missing field: {field}")
        try:
            normalized[field] = float(config[field])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid numeric value for {field}.") from exc
    return normalized


def load_configurations() -> list[dict]:
    ensure_store()
    with open(CONFIG_PATH, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    configs = payload.get("configurations", [])
    normalized = [validate_configuration(config) for config in configs]
    if not normalized:
        normalized = [DEFAULT_CONFIGURATION.copy()]
        save_configurations(normalized)
    return normalized


def save_configurations(configurations: list[dict]) -> None:
    if not configurations:
        raise ValueError("At least one missile configuration is required.")

    normalized = [validate_configuration(config) for config in configurations]
    names = [config["name"].lower() for config in normalized]
    if len(names) != len(set(names)):
        raise ValueError("Configuration names must be unique.")

    ensure_store()
    with open(CONFIG_PATH, "w", encoding="utf-8") as handle:
        json.dump({"configurations": normalized}, handle, indent=2)


def get_configuration(name: str) -> dict | None:
    target = name.strip().lower()
    for config in load_configurations():
        if config["name"].lower() == target:
            return config
    return None


def load_missile_specs():
    """
    Return configurations as the forward-path MissileSpec model.

    Kept here so existing UI/config code can remain the source of truth while
    new planner and simulation services use typed contracts.
    """
    try:
        from missile_guidance.domain import MissileSpec
    except ImportError:
        from src.missile_guidance.domain import MissileSpec

    return [MissileSpec.from_mapping(config) for config in load_configurations()]


def get_missile_spec(name: str):
    try:
        from missile_guidance.domain import MissileSpec
    except ImportError:
        from src.missile_guidance.domain import MissileSpec

    config = get_configuration(name)
    return MissileSpec.from_mapping(config) if config is not None else None
