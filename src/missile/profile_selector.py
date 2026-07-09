"""
profile_selector.py -- interactive "pick existing or create new" for a MissileProfile.

A UI-only layer on top of missile.config_store. It lists the stored missiles (the
JSON files in data/missiles/), lets the user pick one or build a new one by
cloning-and-overriding an existing missile, persists new missiles through
config_store, and returns a ready MissileProfile for the simulation to consume.

`prompt` / `echo` are injected (default input / print) so the selector is unit-
testable and the terminal UI can later be swapped for a GUI / web frontend --
config_store.save_profile is explicitly "the path the frontend will use".

    profile = ProfileSelector().choose()      # or: choose_profile()
    Simulation(profile, config).run()
"""
from __future__ import annotations

from dataclasses import asdict
from typing import Callable

from missile import config_store
from missile.profile import MissileProfile, BasicSpec, WarheadSpec


# BasicSpec fields to prompt for, in a friendly order, with unit hints.
_BASIC_FIELD_ORDER = [
    "cruise_speed", "min_speed", "max_speed",
    "max_acceleration", "min_altitude", "max_altitude",
    "max_g_force", "sustained_turn_rate", "sustained_g_force",
    "evasive_turn_rate", "max_range", "cruise_agl_min", "cruise_agl_max",
    "max_longitudinal_g_boost",
]
_BASIC_UNITS = {
    "cruise_speed": "km/h", "min_speed": "km/h", "max_speed": "km/h",
    "max_acceleration": "m/s^2", "min_altitude": "m MSL", "max_altitude": "m MSL",
    "max_g_force": "g", "sustained_turn_rate": "deg/s", "sustained_g_force": "g",
    "evasive_turn_rate": "deg/s", "max_range": "km",
    "cruise_agl_min": "m AGL", "cruise_agl_max": "m AGL",
    "max_longitudinal_g_boost": "g",
}


class ProfileSelector:
    """Ask the user to pick a stored missile or build a new one."""

    def __init__(
        self,
        prompt: Callable[[str], str] = input,
        echo: Callable[[str], None] = print,
    ) -> None:
        self._prompt = prompt
        self._echo = echo

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------
    def choose(self) -> MissileProfile:
        """List the stored missiles and return the chosen (or newly created) profile."""
        profiles = config_store.load_profiles()

        self._echo("\nSelect a missile:")
        for i, p in enumerate(profiles, 1):
            self._echo(f"  {i}. {p.name}")
        create_idx = len(profiles) + 1
        self._echo(f"  {create_idx}. Create new...")

        choice = self._ask_int("Choice", 1, create_idx)
        if choice <= len(profiles):
            selected = profiles[choice - 1]
            self._echo(f"Using '{selected.name}'.")
            return selected
        return self._create_new(profiles)

    # ------------------------------------------------------------------
    # Create-new: clone an existing missile, override what you want
    # ------------------------------------------------------------------
    def _create_new(self, existing: list[MissileProfile]) -> MissileProfile:
        base = existing[0] if existing else config_store.get_default_profile()

        name = self._ask_nonempty("New missile name")
        if base is not None:
            self._echo(f"\nCloning '{base.name}'. Press Enter to keep each [default].")
        else:
            self._echo("\nNo existing missile to clone; enter every value.")

        basic = self._prompt_basic(base.basic if base is not None else None)
        warhead = self._prompt_warhead(base.warhead if base is not None else WarheadSpec())

        # Sections we don't prompt for (detailed / booster) are inherited from the
        # base, or fall back to their dataclass defaults when there is no base.
        if base is not None:
            profile = MissileProfile(
                name=name, basic=basic,
                detailed=base.detailed, booster=base.booster, warhead=warhead,
            )
        else:
            profile = MissileProfile(name=name, basic=basic, warhead=warhead)

        path = config_store.save_profile(profile)
        self._echo(f"Saved '{name}' -> {path}")
        return profile

    def _prompt_basic(self, base: BasicSpec | None) -> BasicSpec:
        defaults = asdict(base) if base is not None else {}
        values: dict[str, float] = {}
        for field_name in _BASIC_FIELD_ORDER:
            unit = _BASIC_UNITS.get(field_name, "")
            label = f"  {field_name} [{unit}]" if unit else f"  {field_name}"
            values[field_name] = self._ask_float(label, default=defaults.get(field_name))
        return BasicSpec(**values)

    def _prompt_warhead(self, base: WarheadSpec) -> WarheadSpec:
        name = self._ask_str("  warhead_name", default=base.warhead_name)
        blast = self._ask_float("  blast_radius_m [m]", default=base.blast_radius_m)
        return WarheadSpec(warhead_name=name, blast_radius_m=blast)

    # ------------------------------------------------------------------
    # Prompt helpers (blank keeps the default; otherwise validated)
    # ------------------------------------------------------------------
    def _ask_int(self, label: str, lo: int, hi: int) -> int:
        while True:
            raw = self._prompt(f"{label} [{lo}-{hi}]: ").strip()
            try:
                value = int(raw)
                if lo <= value <= hi:
                    return value
            except ValueError:
                pass
            self._echo(f"  Please enter a whole number {lo}-{hi}.")

    def _ask_float(self, label: str, default: float | None = None) -> float:
        hint = f" (default {default})" if default is not None else ""
        while True:
            raw = self._prompt(f"{label}{hint}: ").strip()
            if not raw and default is not None:
                return float(default)
            try:
                return float(raw)
            except ValueError:
                self._echo("  Please enter a number.")

    def _ask_str(self, label: str, default: str | None = None) -> str:
        hint = f" (default {default})" if default else ""
        raw = self._prompt(f"{label}{hint}: ").strip()
        if not raw and default is not None:
            return default
        return raw

    def _ask_nonempty(self, label: str) -> str:
        while True:
            raw = self._prompt(f"{label}: ").strip()
            if raw:
                return raw
            self._echo("  A value is required.")


def choose_profile(
    prompt: Callable[[str], str] = input,
    echo: Callable[[str], None] = print,
) -> MissileProfile:
    """Module-level convenience wrapper for ProfileSelector(...).choose()."""
    return ProfileSelector(prompt=prompt, echo=echo).choose()
