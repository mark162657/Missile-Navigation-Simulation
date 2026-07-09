"""
mission_result.py -- the single end-of-run record for one simulation.

The result layer INTERPRETS what happened -- it assigns the Outcome verdict, holds
the summary numbers, and serialises one JSON file per run. It does NOT detect
impact; that is the impact package's job (simulation.impact). This record is built
from the facts detection produces (miss distance, impact angle, etc.).

Typical use from the sim loop:

    result = MissionResult(
        outcome=MissionResult.classify(miss_m, cfg.impact_radius_m),
        miss_distance_m=miss_m,
        impact_angle_deg=gamma_deg,
        ...
    )
    result.save()                 # -> data/results/<id>_<timestamp>.json
    print(result.summary())
"""
from __future__ import annotations

import json
from dataclasses import dataclass, asdict, field
from datetime import datetime, timezone
from pathlib import Path

from paths import PROJECT_ROOT
from simulation.result.outcome import Outcome

# Default directory to drop one JSON per run.
DEFAULT_RESULTS_DIR = PROJECT_ROOT / "data" / "results"


@dataclass
class MissionResult:
    """One run's outcome + the numbers worth keeping. All impact fields are
    optional so a timed-out (never-impacted) run is still representable."""

    outcome: Outcome

    # Impact geometry (None when the run timed out before impact)
    miss_distance_m: float | None = None      # ground distance to target at impact
    impact_angle_deg: float | None = None     # flight-path angle at impact (neg = diving)
    impact_speed_ms: float | None = None
    impact_gps: tuple[float, float, float] | None = None

    # Mission bookkeeping
    flight_time_s: float | None = None
    distance_flown_m: float | None = None

    # Detonation / warhead (from the profile's WarheadSpec)
    detonated: bool | None = None             # did the warhead go off on impact?
    warhead_name: str | None = None           # e.g. "WDU-36/B"
    blast_radius_m: float | None = None        # lethal radius used for the HIT/MISS call

    # Context (handy when scanning a folder full of result files)
    start_gps: tuple[float, float, float] | None = None
    target_gps: tuple[float, float, float] | None = None
    missile_id: str = ""
    command_centre_id: str = ""

    # Stamped at construction so every saved file is self-dating.
    timestamp_utc: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )

    # ------------------------------------------------------------------
    # Verdict / factories
    # ------------------------------------------------------------------
    @staticmethod
    def classify(
        miss_distance_m: float,
        lethal_radius_m: float,
        *,
        hit_terrain: bool = False,
        detonated: bool = True,
    ) -> Outcome:
        """
        Map raw impact facts to an Outcome (the hit/miss/CFIT verdict).

        Order matters: hitting the surface short of the target (CFIT) is decided
        before any lethal-radius scoring against the intended target.
        """
        if hit_terrain:
            return Outcome.CFIT
        if not detonated:
            return Outcome.MISS
        return Outcome.HIT if miss_distance_m <= lethal_radius_m else Outcome.MISS

    @classmethod
    def timeout(cls, **context) -> "MissionResult":
        """Run hit the flight-time guard without ever impacting."""
        return cls(outcome=Outcome.TIMEOUT, **context)

    @classmethod
    def aborted(cls, **context) -> "MissionResult":
        """Run terminated early (error / manual abort)."""
        return cls(outcome=Outcome.ABORTED, **context)

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------
    def to_dict(self) -> dict:
        d = asdict(self)
        d["outcome"] = self.outcome.value  # plain string, not the Enum member
        return d

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def save(
        self,
        directory: str | Path | None = None,
        filename: str | None = None,
    ) -> Path:
        """
        Write this result as one JSON file and return the path written.

        Args:
            directory: target folder (default: data/results/). Created if missing.
            filename:  override the file name. Default is
                       "<missile_id or 'mission'>_<timestamp>.json".
        """
        directory = Path(directory) if directory is not None else DEFAULT_RESULTS_DIR
        directory.mkdir(parents=True, exist_ok=True)

        if filename is None:
            stamp = self.timestamp_utc.replace(":", "-")  # colons are illegal on Windows
            stem = self.missile_id or "mission"
            filename = f"{stem}_{stamp}.json"

        path = directory / filename
        path.write_text(self.to_json())
        return path

    def summary(self) -> str:
        """One-line human summary for the console."""
        parts = [f"outcome={self.outcome.value}"]
        if self.miss_distance_m is not None:
            parts.append(f"miss={self.miss_distance_m:.1f} m")
        if self.impact_angle_deg is not None:
            parts.append(f"impact_angle={self.impact_angle_deg:.1f} deg")
        if self.detonated is not None:
            warhead = self.warhead_name or "warhead"
            parts.append(f"{warhead}={'detonated' if self.detonated else 'dud'}")
        if self.flight_time_s is not None:
            parts.append(f"t={self.flight_time_s:.1f} s")
        return "  ".join(parts)
