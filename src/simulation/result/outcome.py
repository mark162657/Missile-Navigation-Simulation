"""
outcome.py -- the exhaustive set of ways one simulation run can end.

Exactly one Outcome is assigned per run. HIT is the only "success"; everything
else is a way the mission ended without destroying the target. Values are
lowercase strings so the enum serialises straight into JSON.
"""
from enum import Enum


class Outcome(str, Enum):
    HIT = "hit"          # warhead detonated within lethal radius of the target
    MISS = "miss"        # reached the ground/target area but outside lethal radius
    CFIT = "cfit"        # controlled flight into terrain: hit a hill en route
    WATER = "water"      # ground contact over ocean / no-data terrain
    DUD = "dud"          # impacted on/near target but the warhead did not detonate
    TIMEOUT = "timeout"  # never impacted within the flight-time guard
    ABORTED = "aborted"  # run terminated early (error / manual abort)

    def is_success(self) -> bool:
        """Only a HIT counts as a successful mission."""
        return self is Outcome.HIT
