# Physics, Propulsion & Boost — Module Guide

This document explains the flight-physics layer of the cruise-missile simulation:
the atmosphere, aerodynamics, engine, solid booster, flight sequencer, and the
3-DoF point-mass dynamics — plus the **public API other layers will call**
(autopilot, navigation, guidance, terrain-following, pathfinding).

> **Status:** Phase 1 (physics/dynamics) complete and verified. 3-DoF point-mass
> model, structured so the 6-DoF upgrade touches only `dynamics.py` and
> `aerodynamics.py`.

---

## 1. Where this lives, and why

The codebase is split by a single principle:

| Layer | What it is | Package |
|-------|-----------|---------|
| **The simulated world ("plant")** | What *actually* happens to the airframe, and the noisy sensor readings it produces | `src/simulation/` |
| **The onboard avionics (under test)** | Your navigation, autopilot, guidance, mission plan | `src/missile/` |

Physics is the *plant*, so it lives in `simulation/`, beside the sensor models:

```
src/
├── simulation/
│   ├── physics/          ← THIS DOCUMENT
│   │   ├── atmosphere.py     ISA air properties + shared physical constants
│   │   ├── aerodynamics.py   lift / drag / side force / pitching moment
│   │   ├── engine.py         turbofan sustainer (thrust, fuel)
│   │   ├── booster.py        jettisonable solid rocket booster
│   │   ├── sequencer.py      flight-stage machine (BOOST → CRUISE) + launch modes
│   │   └── dynamics.py       3-DoF equations of motion (RK4) + IMU synthesis
│   └── sensors/          baro / radar altimeter, gps receiver
└── missile/              navigation/  control/  guidance/  planning/
    ├── state.py          MissileState (shared truth + estimate)
    └── profile.py        MissileProfile (BasicSpec / DetailedSpec / BoosterSpec)
```

**Dependencies point one way: `simulation → missile`.** Physics reads the
missile's `MissileProfile` and writes a new `MissileState`, but it never imports
navigation/guidance/control. Data flows in one direction:

```
            control (autopilot)                 IMU (accel + gyro)
                  │                                    ▲
                  ▼                                    │
   ┌──────────────────────────────────────────────────────────┐
   │  MissileDynamics.step(state, control, dt)                 │
   │     atmosphere → aerodynamics → engine/booster → RK4      │
   └──────────────────────────────────────────────────────────┘
                  │
                  ▼
            new MissileState (true_*) ──► sensor models ──► navigation
```

---

## 2. Frames, conventions & units (read this first)

Everything matches the existing nav stack. Get these wrong and nothing lines up.

- **Navigation frame: ENU** — East, North, Up. `z` is **positive up**. (Not NED.)
- **Position: geographic** — `lat°, lon°, alt_m_MSL`. Integrated via the
  `terrain.coordinates.meter_per_deg_*` helpers, exactly like INS.
- **Velocity:** ENU components `[vel_east, vel_north, vel_up]` in m/s.
- **Attitude: Euler** `[roll, pitch, yaw]` in radians. `yaw` is heading,
  clockwise from north. (No DCM/quaternion exists yet.)
- **Body axes:** x forward, y right, z down (standard aerospace).
- **Angles of incidence:** `alpha` (angle of attack), `beta` (sideslip), rad.
- **Control sign convention:** **+elevator = nose-up = +AoA = climb.**

> ⚠️ **The IMU contract is non-standard** (it matches the existing INS, not a
> textbook strapdown IMU). `INS.predict()` integrates a **nav-frame (ENU)
> kinematic acceleration** straight into velocity — it does **not** rotate
> body→nav or remove gravity. So the INS-facing field `imu.accel_enu` is the
> true ENU kinematic acceleration **with gravity included**, not gravity-free
> body specific force. A separate `imu.specific_force_body` field carries the
> "real" gravity-free body reading for a future strapdown upgrade, but the
> current INS ignores it. See §8.

---

## 3. Module reference

### 3.1 `atmosphere.py` — ISA model + physical constants

The single home for universal physical constants. International Standard
Atmosphere (troposphere lapse 0–11 km, isothermal 11–20 km).

**Constants** (import from here, don't redefine):
`G0` (9.80665), `R_AIR`, `GAMMA_AIR`, `T0_ISA`, `P0_ISA`, `RHO0_ISA`,
`LAPSE_RATE`, `H_TROPOPAUSE`, `T_TROPOPAUSE`.

**Functions** (input = altitude in m MSL):

| Call | Returns |
|------|---------|
| `temperature(alt)` | K |
| `pressure(alt)` | Pa |
| `density(alt)` | kg/m³ |
| `speed_of_sound(alt)` | m/s |
| `sample(alt)` | `AtmosphereSample(temperature, pressure, density, speed_of_sound)` |

Verified: sea level → 288.15 K, 1.225 kg/m³, 340.29 m/s (textbook exact).

### 3.2 `aerodynamics.py` — forces & moments

Computes aerodynamic forces from `(Mach, alpha, beta, dynamic_pressure)`.
Coefficients are representative of a subsonic cruise airframe (tune freely).

**Key exposed constants:** `S_REF` (reference area), `CL_ALPHA`, `CD0`,
`K_INDUCED`, `CY_BETA`, `BOOST_DRAG_CD`, `ALPHA_PER_ELEVATOR` (≈ +1.78 rad AoA
per rad elevator), `BETA_PER_RUDDER`, `MAX_ALPHA`, `MAX_BETA`.

**`class Aerodynamics(reference_area=S_REF, reference_chord=C_REF)`**

| Method | Purpose |
|--------|---------|
| `trim_alpha(delta_elevator)` | elevator (rad) → quasi-steady AoA (rad), clamped to `MAX_ALPHA` |
| `trim_beta(delta_rudder)` | rudder (rad) → sideslip (rad), clamped to `MAX_BETA` |
| `compute(mach, alpha, beta, dynamic_pressure, delta_elevator=0.0)` | → `AeroForces(lift, drag, side_force, pitching_moment)` (N, N, N, N·m) |

> **3-DoF trim model:** with no rotational states, fin deflection is mapped to a
> *static-trim* incidence (`trim_alpha`). The pitching-moment coefficients are
> computed and exposed so the 6-DoF upgrade only swaps "assume trim" for
> "integrate the moment" — here, not in any other module.

### 3.3 `engine.py` — turbofan sustainer

Airbreathing cruise engine. Thrust lapses with air density; fuel bookkeeping is
separate from thrust so RK4 can call `thrust()` purely.

**`class Engine(profile)`**

| Member | Purpose |
|--------|---------|
| `thrust(throttle, altitude_m)` | N. 0 when out of fuel. Lapses with `(ρ/ρ0)**0.7` |
| `fuel_burn_rate(throttle)` | kg/s |
| `consume_fuel(throttle, dt)` | burns fuel for one step, returns kg burned (call once/step) |
| `fuel_remaining_kg`, `fuel_capacity_kg`, `is_out_of_fuel` | state |

**Constants:** `SEA_LEVEL_MAX_THRUST` (3100 N), `THRUST_LAPSE_EXP`,
`CRUISE_THROTTLE`. Verified: 3100 N at SL full throttle → 1448 N at 10 km.

### 3.4 `booster.py` — solid rocket booster

High-thrust, short-burn, altitude-independent motor that is jettisoned at burnout.

**`class SolidBooster(spec=None)`** (spec is a `BoosterSpec`, §4)

| Member | Purpose |
|--------|---------|
| `thrust()` | N (constant while burning; 0 once separated/empty) |
| `consume(dt)` | burns propellant, returns kg burned |
| `total_mass` | casing + remaining propellant, kg (0 after separation) |
| `is_burnt_out`, `separated` | state |
| `separate()` | jettison the casing (irreversible) |

You normally don't construct this directly — the `FlightSequencer` builds it from
`profile.booster`.

### 3.5 `sequencer.py` — flight-stage machine

Owns the **physical** stage of flight (which motor fires, current vehicle mass,
booster separation) and the programmed boost pitch-over. This is **distinct from
the guidance "mode"** — see §6.3.

**`enum LaunchMode`**: `GROUND` (shallow ~12°), `SURFACE_VLS` (near-vertical 88°),
`SUBMARINE` (abstracted to near-vertical from the surface).

**`class FlightSequencer(launch_mode=None, cruise_heading_rad=0.0, booster=None, profile=None, handoff_pitch_rad=…)`**

Pass `profile=` to source the booster spec and default launch mode from it.

| Member | Purpose |
|--------|---------|
| `stage` | current `FlightStage` (`BOOST` → `CRUISE`) |
| `is_boosting` | convenience bool |
| `booster_thrust()` | booster thrust this step (0 outside BOOST) |
| `attached_booster_mass()` | booster mass still on the vehicle (0 after sep) |
| `commanded_attitude()` | `(roll, pitch, yaw)` during BOOST (programmed), else `None` |
| `advance(dt)` | burn propellant, step the schedule; on burnout → jettison + switch to CRUISE |

Stage machine: `BOOST --(booster burnt out)--> CRUISE`. The pitch-over is an
open-loop **smoothstep** from the launch pitch to a handoff pitch over the burn.

### 3.6 `dynamics.py` — equations of motion + IMU (the main entry point)

**`@dataclass ControlInput`** — the autopilot's command, the *only* input that
drives the physics:

```python
ControlInput(throttle=0.0,   # 0..1
             elevator=0.0,    # rad, + = nose-up
             rudder=0.0,      # rad
             aileron=0.0)     # rad (defined for 6-DoF; unused in 3-DoF)
```

**`@dataclass IMUMeasurement`** — the simulated IMU for one step:

```python
IMUMeasurement(
    accel_enu,            # ENU kinematic accel [a_E, a_N, a_U], m/s² — feeds INS
    angular_velocity,     # body rates [roll_rate, pitch_rate, yaw_rate], rad/s — feeds INS
    specific_force_body,  # gravity-free body accel [fwd, right, down], m/s² — future strapdown
    time,                 # s
)
```

**`class MissileDynamics(profile, aerodynamics=None, engine=None, sequencer=None, *, imu_accel_noise_std=0.0, imu_gyro_noise_std=0.0, rng=None)`**

| Member | Purpose |
|--------|---------|
| `step(state, control, dt)` | **→ `(new_state, IMUMeasurement)`.** The single integration step. |
| `current_mass_kg` | cruise-vehicle mass (dry + remaining fuel), kg |
| `engine`, `aero`, `sequencer` | the attached models |

`step()` returns a **new** `MissileState` with `true_*`, `vel_*`, attitude and
`time` advanced; it leaves the navigation estimate (`est_*`) and the sensor flags
untouched (physics never writes the nav estimate). RK4 integration. If a
`sequencer` is attached it governs the boost stage; with `sequencer=None` the
missile is modelled purely in cruise (the original behaviour).

**Noise:** off by default — the INS already adds its own bias/noise, so the
dynamics emits clean truth. Turn on `imu_accel_noise_std` / `imu_gyro_noise_std`
only if you want the *sensor* (not the INS) to own the error.

---

## 4. Profile additions (`missile/profile.py`)

`MissileProfile` now has a third section, `BoosterSpec` (`profile.booster`),
alongside `BasicSpec` (`profile.basic`) and `DetailedSpec` (`profile.detailed`).

```python
@dataclass
class BoosterSpec:
    booster_thrust_N: float = 45000.0     # constant thrust while burning, N
    burn_time_s: float = 7.0
    propellant_mass_kg: float = 145.0     # burned over burn_time_s
    casing_mass_kg: float = 150.0         # jettisoned at burnout
    launch_mode: str = "SURFACE_VLS"      # default launch platform
    # properties: total_mass_kg, burn_rate_kgps
```

> **Note:** profiles loaded through `config_store` currently arrive **without** a
> booster section (its validator keeps only `name`/`basic`/`detailed`), so the
> `BoosterSpec` *defaults* apply on that path. To load real per-missile booster
> values from JSON, `config_store.py` + the missile JSON files need extending
> (pending authorization). The defaults already match Tomahawk-class values.

---

## 5. The boost → cruise sequence (worked example)

A surface-VLS launch, from `tests/test_boost_phase.py`:

```
 t(s)   stage   alt(m)  V(m/s)  pitch  mass(kg)
  0.0   BOOST      5.0     0.2   88.0   1609.8   ← at rest, near-vertical (cruise vehicle + booster)
  3.0   BOOST     86.3    56.0   54.9   1547.7   ← climbing, pitching over, propellant burning
  6.0   BOOST    275.0   113.9    8.7   1485.5
  >> booster separation at t=7.01s  (mass 1315 kg, V=135.6 m/s, alt=335 m)
  7.0  CRUISE    334.8   135.6    4.0   1315.0   ← turbofan, velocity-derived attitude
```

Burnout delivers ~0.4 Mach — where a turbofan ignites. A **clean INS fed the
boost IMU stream tracked truth to 0.11 m** through the whole high-g transient,
confirming the IMU format survives the boost (a useful nav stress test).

---

## 6. API you'll consume from other layers

This section maps each upcoming layer to the methods it will call.

### 6.1 Autopilot / control (`missile/control/`)

The autopilot turns a guidance command (e.g. `Az_cmd`, a commanded normal
acceleration) into a `ControlInput`, which it hands to the dynamics.

- **Produce:** `ControlInput(throttle, elevator, rudder, aileron)`.
- **Read (nav estimate):** `MissileState.get_speed()`, `.get_velocity()`,
  `.get_attitude()`, `.est_position()`.
- **Useful physics linkage** (to size an elevator command for a desired Az):
  `A_z ≈ L/m = q·S_REF·CL_ALPHA·alpha / m`, and `alpha = ALPHA_PER_ELEVATOR ·
  elevator`. So `elevator_cmd ≈ A_z·m / (q·S_REF·CL_ALPHA·ALPHA_PER_ELEVATOR)`.
  Respect `MAX_ALPHA` (≈0.35 rad) and `profile.get_max_lateral_acceleration()`.
- Atmosphere/aero helpers (`atmosphere.density`, `Aerodynamics.compute`) are
  available if the autopilot wants the true `q` for gain scheduling.

> During **BOOST** the sequencer ignores `ControlInput` (booster + programmed
> pitch). Your autopilot only takes effect once `sequencer.stage == CRUISE`.

### 6.2 Navigation (`missile/navigation/`)

- **Feed the INS directly** (no adapter):
  ```python
  new_state, imu = dynamics.step(state, control, dt)
  ins.predict(imu.accel_enu, dt, imu.angular_velocity)
  ```
- **Build the INS from the profile:**
  `profile.create_ins(init_pos, init_vel, init_att, rng=…)` (uses `DetailedSpec`
  IMU error terms).
- **Contract:** `imu.accel_enu` is ENU kinematic accel (gravity included);
  `imu.angular_velocity` is `[roll_rate, pitch_rate, yaw_rate]`. Ignore
  `imu.specific_force_body` until the INS is upgraded to a real strapdown
  mechanization.

### 6.3 Guidance (`missile/guidance/`, future)

- **Read the nav estimate** (never the truth): `MissileState.est_position()`,
  `.get_velocity()`, `.get_attitude()`, `.get_speed()`, `.distance_to_target`.
- **Mode logic:** the guidance manager keeps its OWN phase
  (`BOOST/MIDCOURSE/TERMINAL/IMPACT` from `missile.state.FlightStage`) for
  choosing laws. It can observe the physical stage via `sequencer.stage`, but the
  two are deliberately separate — physics owns staging, guidance owns law
  selection. They coincide only at the boost→cruise instant.
- **Emit abstract commands** (e.g. `Az_cmd` or a desired attitude) — never
  actuator deflections. The autopilot (§6.1) converts them.
- Geometry you'll re-implement on the nav state: heading `atan2(v_E, v_N)`,
  flight-path angle `atan2(v_U, hypot(v_E, v_N))`. (Private equivalents exist in
  `dynamics.py` for the truth model; guidance should compute its own from the
  estimate.)

### 6.4 Flight-path preference / terrain-following

`MissileProfile` carries the altitude-band logic (all in m **AGL**, height above
ground):

| Call | Returns |
|------|---------|
| `profile.preferred_agl()` | midpoint of the cruise band, m AGL |
| `profile.is_within_cruise_band(agl_m)` | bool |
| `profile.clamp_to_cruise_band(agl_m)` | clamp into the band |
| `profile.target_msl_altitude(ground_elev_m)` | preferred AGL → absolute MSL target, clamped to the envelope |
| `profile.basic.cruise_agl_min / cruise_agl_max` | the band |
| `profile.basic.min_altitude / max_altitude` | absolute MSL floor/ceiling |

Combine `profile.target_msl_altitude(ground_elev)` (ground elevation from
`terrain.dem_loader`) with a climb-rate limit to drive terrain-following.

### 6.5 Pathfinding / mission planning (`missile/planning/`)

Maneuver-envelope helpers (SI):

| Call | Returns |
|------|---------|
| `profile.min_turn_radius()` | tightest sustained turn radius at cruise, m |
| `profile.calculate_turning_radius(speed_ms, turn_rate_rads)` | r = v/ω, m |
| `profile.get_max_lateral_acceleration()` | from `max_g_force`, m/s² |
| `profile.get_turn_rate_for_maneuver("manual"\|"evasive")` | rad/s |
| `profile.validate_maneuver(cur_v, des_v, turn_rate)` | bool — checks the envelope |
| `profile.basic.max_range`, `estimate_fuel_range_km()`, `estimate_endurance_s()` | range/endurance limits |
| `profile.basic.cruise_speed_ms` (and `min_/max_speed_ms`) | SI speeds |

Use `min_turn_radius()` to constrain RHA*/A* path curvature, and
`estimate_fuel_range_km()` vs `basic.max_range` (take the smaller) as the hard
range limit.

---

## 7. Running the checks

Both are plain scripts (not pytest):

```bash
python tests/test_physics_nav_integration.py   # cruise: IMU↔INS format compatibility (~0.0 m error)
python tests/test_boost_phase.py               # full VLS boost→cruise; switch LAUNCH for GROUND/SUBMARINE
```

The existing nav suite (`pytest tests/navigation`, 85 tests) is unaffected.

---

## 8. Known limitations & the 6-DoF path

- **3-DoF point mass.** Attitude in cruise is derived from the velocity vector +
  trim AoA; roll is 0. The 6-DoF upgrade integrates the rotational dynamics and
  touches only `dynamics.py` + `aerodynamics.py` (moment coefficients already
  exposed).
- **IMU is ENU kinematic accel, not body specific force** — matched to the
  current INS (which omits gravity removal and body→nav rotation). The
  `specific_force_body` field is ready for a real strapdown INS upgrade.
- **No autopilot yet**, so open-loop cruise *porpoises* (phugoid oscillation) —
  expected; the accel autopilot will damp it.
- **No ground-collision check** — a shallow `GROUND` launch will fly below
  terrain. Add a terrain check when wiring the sim loop.
- **Booster JSON persistence** is pending (`config_store` strips the section;
  defaults apply on that path).
