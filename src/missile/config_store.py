import json
import re
from dataclasses import asdict
from pathlib import Path

from missile.profile import DetailedSpec, MissileProfile
from paths import PROJECT_ROOT

CONFIG_DIR = PROJECT_ROOT / "data" / "missiles"

# Legacy single-file store (an array of configs). Migrated on first access to
# the new layout: one JSON file per missile in CONFIG_DIR.
LEGACY_CONFIG_PATH = CONFIG_DIR / "configurations.json"


# SECTION 1 — fields the user must provide (publicly findable).
BASIC_FIELDS = [
    "cruise_speed",
    "min_speed",
    "max_speed",
    "max_acceleration",
    "min_altitude",
    "max_altitude",
    "max_g_force",
    "max_longitudinal_g_boost", # max g force going forward (longitudinal) during BOOST phase
    "sustained_turn_rate",
    "sustained_g_force",
    "evasive_turn_rate",
    "max_range",
    "cruise_agl_min",
    "cruise_agl_max",
]

BASIC_UNITS = {
    "cruise_speed": "km/h",
    "min_speed": "km/h",
    "max_speed": "km/h",
    "max_acceleration": "m/s^2",
    "min_altitude": "m MSL",
    "max_altitude": "m MSL",
    "max_g_force": "g",
    "max_longitudinal_g_boost": "g",
    "sustained_turn_rate": "deg/s",
    "sustained_g_force": "g",
    "evasive_turn_rate": "deg/s",
    "max_range": "km",
    "cruise_agl_min": "m AGL",
    "cruise_agl_max": "m AGL",
}

# Defaults for basic fields added after the initial schema, so older config
# files (and migrations) still load instead of hard-failing.
BASIC_DEFAULTS = {
    "max_range": 0.0,
    "max_longitudinal_g_boost": 20.0,
    "cruise_agl_min": 30.0,
    "cruise_agl_max": 100.0,
}

# SECTION 2 — detailed / INS-facing fields (all optional, defaults applied).
DETAILED_FIELDS = [
    "mass_kg",
    "imu_grade",
    "fuel_capacity_kg",
    "fuel_burn_rate_kgps",
    "accel_bias_sigma",
    "gyro_bias_sigma_dph",
    "accel_noise_std",
    "gyro_noise_std_dps",
    "accel_bias_walk_std",
    "gyro_bias_walk_std_dpm",
    "ins_update_rate_hz",
    "gps_update_rate_hz",
    "tercom_update_rate_hz",
]


# ----------------------------------------------------------------------
# Default profiles
#
# These are NOT hard-coded here. The defaults live as editable JSON files in
# CONFIG_DIR (data/missiles/), version-controlled with the repo. Update those
# files with accurate data — no code change needed. This is the same path a
# future frontend uses: each missile is just a JSON file in this folder.
# ----------------------------------------------------------------------
DEFAULT_PROFILE_FILENAMES = (
    "tomahawk_block_v.json",
    "kh_101.json",
)

# Missile selected when none is specified.
DEFAULT_PROFILE_NAME = "Tomahawk Block V"


def default_profile_paths() -> list[Path]:
    """Paths to the bundled default missile JSON files."""
    return [CONFIG_DIR / name for name in DEFAULT_PROFILE_FILENAMES]


# ----------------------------------------------------------------------
# Filename helpers (one file per missile)
# ----------------------------------------------------------------------
def slugify(name: str) -> str:
    """Turn a missile name into a safe filename stem."""
    slug = name.strip().lower()
    slug = re.sub(r"[^a-z0-9]+", "_", slug).strip("_")
    return slug or "missile"


def config_path_for(name: str) -> Path:
    """Path to the JSON file that stores the missile with this name."""
    return CONFIG_DIR / f"{slugify(name)}.json"


def _iter_config_files() -> list[Path]:
    """All per-missile JSON files, excluding the legacy combined store."""
    if not CONFIG_DIR.exists():
        return []
    return sorted(p for p in CONFIG_DIR.glob("*.json") if p != LEGACY_CONFIG_PATH)


def _migrate_legacy_store() -> None:
    """
    Split a legacy `configurations.json` (array) into one file per missile,
    then remove the legacy file. Safe to call repeatedly.
    """
    if not LEGACY_CONFIG_PATH.exists():
        return

    try:
        with open(LEGACY_CONFIG_PATH, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        for config in payload.get("configurations", []):
            normalized = validate_configuration(config)
            _write_config_file(normalized)
    finally:
        LEGACY_CONFIG_PATH.unlink(missing_ok=True)


# ----------------------------------------------------------------------
# Validation
# ----------------------------------------------------------------------
def validate_configuration(config: dict) -> dict:
    """
    Validate and normalize a stored configuration into the two-section layout.

    SECTION 1 (basic) is required in full. SECTION 2 (detailed) is optional;
    any missing keys are filled from DetailedSpec defaults. Accepts both the
    nested {"basic": {...}, "detailed": {...}} layout and a flat legacy dict.
    """
    name = str(config.get("name", "")).strip()
    if not name:
        raise ValueError("Configuration name is required.")

    basic_in = config.get("basic", config)

    basic_norm = {}
    for f in BASIC_FIELDS:
        if f not in basic_in:
            # Fields added after the initial schema fall back to a default so
            # older config files still load; only truly-required fields raise.
            if f in BASIC_DEFAULTS:
                basic_norm[f] = float(BASIC_DEFAULTS[f])
                continue
            raise ValueError(f"Missing basic field: {f}")
        try:
            basic_norm[f] = float(basic_in[f])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid numeric value for {f}.") from exc

    detailed_in = config.get("detailed") or {}
    detailed = DetailedSpec(**{k: v for k, v in detailed_in.items() if k in DETAILED_FIELDS})

    return {
        "name": name,
        "basic": basic_norm,
        "detailed": asdict(detailed),
    }


# ----------------------------------------------------------------------
# Read / write
# ----------------------------------------------------------------------
def _write_config_file(normalized: dict) -> Path:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path = config_path_for(normalized["name"])
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(normalized, handle, indent=2)
    return path


def ensure_store() -> Path:
    """
    Make sure the store directory exists and migrate any legacy combined file.

    Does NOT seed defaults from code — the default missiles are the JSON files
    shipped in CONFIG_DIR. Returns CONFIG_DIR.
    """
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    _migrate_legacy_store()
    return CONFIG_DIR


def load_configurations() -> list[dict]:
    """Load every missile config file as a normalized dict."""
    ensure_store()
    configs = []
    for path in _iter_config_files():
        with open(path, "r", encoding="utf-8") as handle:
            configs.append(validate_configuration(json.load(handle)))
    return configs


def save_configuration(config: dict) -> Path:
    """
    Persist a single missile configuration to its own JSON file.

    This is the path the frontend will use: creating a missile writes one file.
    """
    normalized = validate_configuration(config)
    return _write_config_file(normalized)


def save_profile(profile: MissileProfile) -> Path:
    """Persist a MissileProfile to its own JSON file."""
    return save_configuration(profile.to_config())


def delete_configuration(name: str) -> bool:
    """Remove a missile's JSON file. Returns True if a file was deleted."""
    path = config_path_for(name)
    if path.exists():
        path.unlink()
        return True
    return False


def get_configuration(name: str) -> dict | None:
    target = name.strip().lower()
    for config in load_configurations():
        if config["name"].lower() == target:
            return config
    return None


def load_profiles() -> list[MissileProfile]:
    """Return stored configurations as MissileProfile objects."""
    return [MissileProfile.from_config(config) for config in load_configurations()]


def get_profile(name: str) -> MissileProfile | None:
    """Return a single stored configuration as a MissileProfile, or None."""
    config = get_configuration(name)
    return MissileProfile.from_config(config) if config is not None else None


def get_default_profile() -> MissileProfile | None:
    """
    Return the default missile profile (DEFAULT_PROFILE_NAME), falling back to
    the first available profile if that name is not present.
    """
    profile = get_profile(DEFAULT_PROFILE_NAME)
    if profile is not None:
        return profile
    profiles = load_profiles()
    return profiles[0] if profiles else None
