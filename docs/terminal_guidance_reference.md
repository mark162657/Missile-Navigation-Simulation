# Terminal Guidance — Formula / Variable / Data Reference

Extracted for `src/missile/guidance/terminal_guidance.py`. Impact-angle-constrained
end-game guidance in the vertical plane. Works with an INS only (no seeker required),
which matches the project's INS interface contract and stationary/known-target case.

Primary source: **Ryoo, Cho, Tahk, "Optimal Guidance Laws with Terminal Impact Angle
Constraint," J. Guidance, Control, and Dynamics, Vol. 28, No. 4, 2005** — the
closed-form lag-free law is the recommended one to implement.

Secondary source: **Kim & Grider, "Terminal Guidance for Impact Attitude Angle
Constrained Flight Trajectories," IEEE T-AES, Vol. AES-9, No. 6, 1973** — an
alternative LQ / Riccati route with time-varying feedback gains.

The JHU/APL "Global Engagement" article is conceptual only (nav definitions, TERCOM,
DSMAC) — no terminal-guidance equations; not used here.

---

## 1. Geometry & variables (Ryoo Fig. 1)

Planar homing on a stationary / slowly-moving target. All angles small for the
linearization; work in the vertical (down-range vs. altitude) plane.

| Symbol            | Meaning                                                        | In your stack |
|-------------------|---------------------------------------------------------------|---------------|
| `V_m`             | Missile speed (assumed constant over the terminal phase)       | `‖v_enu‖` |
| `θ_m(t)`          | **Flight-path angle** (velocity direction)                     | already computed (git: flight-path-angle) |
| `θ_mf`            | **Desired terminal impact angle** (the constraint)             | mission input |
| `θ_m0`            | Initial (launch/handover) heading angle                        | state at TG handover |
| `a_m(t)` = `u`    | Lateral acceleration command, ⟂ to velocity; `a_m = −V_m·θ̇_m` | guidance output |
| `z(t)`            | Lateral offset from the reference line, `z(0)=0`               | from INS pos + target |
| `R(t)`            | Range missile→target                                           | `TargetGeometry.direct_3d_distance` |
| `θ(t)`            | Line-of-sight (LOS) angle                                      | from INS pos + known target |
| `t_go = t_f − t`  | Time-to-go                                                     | est. — see §4 |
| `t_f`             | Total time of flight                                           | — |

Equations of motion (Ryoo Eq. 1):
```
ż(t)      = V_m · sin θ_m(t),      z(0) = 0
V_m·θ̇_m(t) = −a_m(t),              θ_m(0) = θ_m0
```
Linearized for small `θ_m`, constant `V_m` (Eq. 3):
```
ż  = V_m · θ_m
θ̇_m = −a_m / V_m
```

---

## 2. PRIMARY law — lag-free Optimal Guidance Law (Ryoo Eq. 23 / 26)

Limit `s1, s2 → ∞` (zero miss distance **and** exact impact angle). This is a
closed-form linear feedback — no online optimization needed.

### 2a. INS form, using lateral offset `z` (Eq. 23)
```
a_m^cmd = (V_m / t_go²) · [ 6·z/V_m + 4·t_go·θ_m + 2·t_go·θ_mf ]

        = (6 / t_go²)·z  +  (4·V_m / t_go)·θ_m  +  (2·V_m / t_go)·θ_mf
```
Requires only INS states (`z`, `θ_m`) + the desired `θ_mf` + `t_go`.

### 2b. LOS form, using LOS angle `θ` (Eq. 26)  ← recommended
Substituting `z ≈ −V_m·t_go·θ`:
```
a_m^cmd = (V_m / t_go) · [ −6·θ  +  4·θ_m  +  2·θ_mf ]
```
With a **known stationary target**, `θ` (LOS) is computed directly from INS position
and the target position — so this form is fully usable without a seeker. This is the
classic impact-angle guidance: an N=6 PN-like `−6θ` term plus impact-angle bias.

Decomposition (Eqs. 23–25), the command is a ramp + step in `t_go`:
```
a_m^cmd = C_R·t_go + C_S
C_R = (6·V_m / t_go³)·[ 2·z/V_m + t_go·θ_m + t_go·θ_mf ]        (ramp coeff.)
C_S = −(2·V_m / t_go²)·[ 3·z/V_m + t_go·θ_m + 2·t_go·θ_mf ]     (step coeff.)
```
In the ideal case `C_R`, `C_S` are constant along the trajectory.

---

## 3. Launch/handover angle & command-magnitude sizing (Ryoo §III)

Closed-form trajectory (Eqs. 28–30), useful for picking a handover heading and for
sanity-checking peak acceleration:
```
θ_m(t) = (1/V_m)·( ½·C_R·t_go² + C_S·t_go + V_m·θ_mf )
z(t)   = −(1/6)·C_R·t_go³ − ½·C_S·t_go² − V_m·θ_mf·t_go
C_R = (6·V_m / t_f²)·(θ_m0 + θ_mf)
C_S = −(2·V_m / t_f²)·(θ_m0 + 2·θ_mf)
```

Special heading choices:
- **Energy-optimal launch:** `θ_m0* ≈ −½·θ_mf`  (Eq. 32); min energy `J̄* ≈ 3·V_m²·θ_mf²/t_f`.
- **Minimum-command launch:** `θ̃_m0 = −θ_mf`  (Eq. 38) → missile flies a **circular arc**:
```
t_f       = (R0 / V_m) · θ_mf / sin θ_mf                (Eq. 40)
a_m^min   = 2·V_m²·|sin θ_mf| / R0                       (Eq. 41)
```
  where `R0` = initial range. Use `a_m^min` to check against the airframe/accel limit
  before committing to a `θ_mf`.

Command saturation guidance: launch with `θ_m0 = −θ_mf` if the accel limit is below
`|3·V_m·θ_mf / t_f|`; energy-optimal `θ_m0 = −0.5·θ_mf` is safe only when the limit
exceeds that.

---

## 4. Time-to-go estimation (Ryoo §IV, Table 1) — CRITICAL

`t_go = R/V_m` is **not** adequate for curved impact-angle trajectories (it
under-estimates). Use the curvature-corrected estimators. Define look-ahead angles:
```
θ̄_m  = θ_m  + θ        (flight-path angle relative to LOS)
θ̄_mf = θ_mf + θ
```

**Method 2 — range over mean velocity (best per the paper, use this):**
```
t_go = R / V̄_m
V̄_m ≈ V_m · [ 1 − (θ̄_m² + θ̄_mf²)/15 + θ̄_m·θ̄_mf/30
              + (θ̄_m⁴ + θ̄_mf⁴)/420 − θ̄_m·θ̄_mf·(θ̄_m² + θ̄_mf² − θ̄_m·θ̄_mf)/840 ]
```

**Method 1 — curved-path length over speed (alternative):**
```
t_go ≈ (R / V_m) · [ 1 + (θ̄_m² + θ̄_mf²)/15 − θ̄_m·θ̄_mf/30
                     − (θ̄_m⁴ + θ̄_mf⁴)/140
                     + θ̄_m·θ̄_mf·(θ̄_m² + θ̄_mf² − θ̄_m·θ̄_mf)/280 ]
```
Approximations valid for `−1 < (z')² < 1` (Method 1) i.e. small look angles; Method 2
holds more broadly. For large `θ̄_m0`, `θ̄_mf` (near ±90°) Method 1 degrades most.

---

## 5. Fidelity upgrade — first-order autopilot lag (Ryoo §III.B, Eqs. 42–46)

If/when you model actuator lag `a_m(s)/u(s) = 1/(τ·s + 1)`, the state-feedback law is
```
u* ≈ V_m·[ −t_go·W1·θ + W2·θ_m + W3·θ_mf ] + W4·a_m
```
with gains `W1..W4`, `Δ`, `K1..K4`, `D1,D2`, `α = t_go/τ` given in Eqs. 44–46.
Key design rule from the adjoint analysis: **miss/impact-angle error is negligible only
when `t_f > 12·τ`.** As `τ → 0` it reduces to the lag-free law of §2. Deferred per the
Stage-2 roadmap unless autopilot lag is added.

---

## 6. ALTERNATIVE — LQ terminal guidance (Kim & Grider 1973)

Vertical-plane LQ with time-varying feedback gains. More machinery than §2; included
for completeness / cross-check.

State (Eq. 1): `X = [Y_d, Ẏ_d, A_L, θ]ᵀ = [Y_t−Y_m, Ẏ_t−Ẏ_m, A_L, θ]ᵀ`
- `Y_d` = vehicle→target position projected on ground; `A_L` = lateral accel;
  `θ` = body attitude ≈ flight-path angle (small AoA); `Ÿ_m = A_L·cos θ`.

Zero-lag autopilot (case 1): `A_L = (K1/w1)·u`, `θ̇ = K_a·u`; linearize `cos θ ≈ b`.

Cost (Eq. 10): `J = Y_d²(t_f) + γ·θ²(t_f) + β·∫ u² dt`

Optimal control, case 1 (Eqs. 11–14):
```
u* = [C1y, C1ẏ, C1θ] · [Y_d, Ẏ_d, θ]ᵀ ,   let τ' = t_f − t,  g ≡ b·K1/w1

C1y  = ( −β·g·τ'        − γ·g·K_a²·τ'²/2 ) / Δ
C1ẏ  = ( −β·g·τ'²       − γ·g·K_a²·τ'³/2 ) / Δ
C1θ  = ( −β·γ·K_a       + γ·K_a·g²·τ'³/6 ) / Δ
Δ    = ( β² + γ·β·K_a²·τ' + β·g²·τ'³ )/3  +  γ·g²·K_a²·τ'⁴/12
```
Impact time from initial altitude `H0` (Eq. 16): `0 = H0 − ∫ V_m·cos θ dt`.

Constant-gain LOS feedback (Eqs. 22–23), simpler hardware:
```
λ̇ = (cos²λ·V_v / H²)·Y_d + (cos²λ / H)·Ẏ_d
u  = K0·λ̇ + K·θ
   = (K0·cos²λ·V_v / H²)·Y_d + (K0·cos²λ / H)·Ẏ_d + K·θ
```
`H` = vehicle altitude (monotonically decreasing), `V_v` = vertical velocity, `λ` = LOS.

Note: gains change rapidly near `t_f` → sensitive to `t_f` error. Guidance is
suboptimal; the valid region of initial states shrinks as initial altitude `H0` drops.

---

## 7. Reference data / constants

**Ryoo nonlinear-sim setup (Table 2)** — good defaults for a test scenario:
| Parameter                 | Value |
|---------------------------|-------|
| Missile start `x_m0,z_m0` | 0 m, 0 m |
| Missile speed `V_m`       | 200 m/s |
| Target position           | 0 m, 5000 m (range 5 km) |
| Initial heading `θ_m0`    | 90° |
| Impact angle `θ_mf`       | −90° … +90°, 30° steps |

Capture region (for `θ_m0 = 180°`): roughly `−140° ≤ θ_mf ≤ 180°`.

**Kim & Grider parameters:** `w1 = 5 rad/s`, `K1 = 1`, `K_a = 0.0005`, `γ = 3283`,
`β = 6.94×10⁻⁵`. Terminal specs `|Y_d(t_f)| ≤ 5 ft`, `|θ(t_f)| ≤ 5°`. Table I case:
`H0 = 10⁴ ft`, `θ0 = 45°`, `Y_d(0) = 10⁴ ft`, `V_m = 2000 ft/s`, `V_T = 40 ft/s`,
hard limit `|u| ≤ 120`.

---

## 8. Suggested implementation path

1. At TG handover, read `V_m`, `θ_m` (flight-path angle), current position; get `R`
   from `TargetGeometry`; compute LOS angle `θ` from INS position + known target.
2. Estimate `t_go` with **Method 2** (§4).
3. Command `a_m` with the **LOS form Eq. 26** (§2b) — or the `z` form Eq. 23 if you
   prefer pure INS lateral offset.
4. Convert `a_m` (⟂ to velocity, in-plane) to your ENU acceleration command and pass to
   the flight computer; clamp to the airframe accel limit (size with §3 `a_m^min`).
5. Later: add first-order-lag gains (§5) if/when autopilot lag is modeled.

Sign convention: fix a consistent sense for `θ`, `θ_m`, `θ_mf`, and `a_m` (positive =
pitch-up / +altitude side) once, and verify with the circular-arc check (§3, Eq. 40):
`θ_m0 = −θ_mf` should produce a constant `a_m = a_m^min` and `t_f = (R0/V_m)·θ_mf/sin θ_mf`.
