# MPPI_CONTROLLER_SPEC.md

**Component:** Sampling-based MPPI path-tracking controller for F1TENTH / RoboRacer
**Target:** ROS 2 (Humble+), Python first, GPU backend optional
**Status:** Planning handoff for Claude Code. This is a *spec*, not an implementation.

---

## 0. Purpose & scope

Build a Model Predictive Path Integral (MPPI) controller that:

1. Loads a precomputed raceline file from the `f1tenth/f1tenth_racetracks` format.
2. Follows the **geometric path** of the raceline (x, y, ψ).
3. Tracks a **scaled** version of the raceline speed profile, `v_ref(s) = α · vx(s)`, with `α ∈ (0, 1]`.
4. Publishes `AckermannDriveStamped` commands closed-loop in the F1TENTH gym and (later) on the physical car.

**Out of scope for v1:** opponents / dynamic obstacles, online raceline re-optimization, overtaking, learned dynamics. The cost function is structured so these can be added later without architectural change.

**Non-goals / constraints from the requester:**
- This controller follows a *fixed* geometric raceline at *scaled* speed. It is intentionally simpler than a full MPCC racer. Treat track-limit avoidance as an optional hardening gate, not a v1 requirement.

---

## 1. Background & references

MPPI is an information-theoretic, derivative-free MPC. Each control cycle it samples `K` perturbed control sequences, rolls each through a vehicle model over a horizon `T`, scores each rollout with a cost, and updates the nominal control sequence as a softmax-weighted average of the samples. The first control is applied; the sequence is shifted and reused as a warm start.

Anchor references for the implementer (concepts only, do not vendor code):
- Williams et al., *Information-Theoretic MPC for Model-Based Reinforcement Learning* / *Model Predictive Path Integral Control: From Theory to Parallel Computation* (the AutoRally line).
- The f1tenth_gym dynamic single-track model for parameter values and the sim command interface.

### 1.1 Raceline / track data format (verified against the repo)

Per track folder (e.g. `Spielberg/`):

| File | Columns | Delimiter | Notes |
|---|---|---|---|
| `*_raceline.csv` | `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2` | `;` | Comment/header lines start with `#`. **ψ is in 0–2π**. Curvature-minimizing line, within track bounds. |
| `*_centerline.csv` | `x_m, y_m, w_tr_right_m, w_tr_left_m` | `,` | Center line + track widths (right/left). Optional, used only for track-limit cost. |
| `*_map.png` + `*_map.yaml` | — | — | Occupancy grid + `resolution`, `origin`. Optional, only for occupancy-based limits/viz. |

Key gotchas the loader **must** handle:
- `;` delimiter and `#` comment lines in the raceline file.
- ψ given in **[0, 2π)** → wrap to [−π, π) before any heading arithmetic.
- The track is a **closed loop**: arc length `s` wraps; nearest-point search and warm-start indexing must wrap modulo total length `L_track`.
- **Frame alignment**: the raceline coordinates are in the track's map frame. The localization source (sim odom or particle filter `map` frame) must be in the same frame. Provide a configurable static offset `(x0, y0, yaw0)` and verify alignment as an explicit integration check (§14).

---

## 2. System overview

```
                 raceline.csv
                     │
            ┌────────▼─────────┐
            │  RacelineModel    │  parse, resample(ds), KD-tree, spline
            │  (offline-built)  │  query: project(x,y)->s ; sample(s)->ref
            └────────┬─────────┘
   /odom (pose,v)    │ reference queries
        │            │
   ┌────▼────────────▼─────┐         /drive (AckermannDriveStamped)
   │   MPPIController       │────────────────────────────────────────►
   │  (ReactivePlanner)     │
   │  - sample K×T noise    │         /mppi/optimal_path (nav_msgs/Path)
   │  - rollout (veh model) │────────►/mppi/samples (MarkerArray)  [viz]
   │  - cost + softmax      │         /mppi/reference_path (Path)
   │  - shift / warm start  │
   └────────────────────────┘
```

### 2.1 Integration with `ReactivePlannerBase`

The MPPI controller **must conform to the existing `ReactivePlannerBase` interface** used in the RoboRacer stack rewrite. Expected contract (adjust names to match the actual base class — confirm before Gate 1):

- A `plan(state) -> control` (or `compute(...) -> AckermannDrive`) method that takes the current vehicle state and returns a drive command.
- Lifecycle hooks for `configure(params)` / `reset()`.
- No ROS dependencies inside the core solver — ROS lives only in the node wrapper, so the solver is unit-testable headless.

**Action for Claude Code:** locate `ReactivePlannerBase`, read its method signatures, and write `MPPIPlanner(ReactivePlannerBase)` to match. If the base does not exist yet or differs, flag it rather than guessing.

---

## 3. Coordinate frames & assumptions

- All planning happens in the **map frame** (same frame as the raceline CSV).
- Vehicle pose `(x, y, θ)` and speed `v` arrive from odometry; `θ` already in [−π, π].
- Steering angle `δ` is **not** reliably sensed on the car (no encoder on the servo). Maintain it as **internal controller state**, initialized to the last commanded value; integrate it open-loop through the model.
- Single-rate control loop; the solver must finish within one control period (§9).

---

## 4. RacelineModel (reference geometry)

Offline, built once at startup. Responsibilities:

1. **Parse** raceline CSV (`;`, skip `#`). Extract arrays `s, x, y, psi, kappa, vx, ax`. Wrap ψ to [−π, π].
2. **Resample** to uniform arc length `ds` (param, e.g. 0.05–0.10 m) via cubic spline on `x(s), y(s)` (and interpolate ψ via `atan2` of the tangent for consistency, or unwrap-then-interp). Store the resampled polyline plus interpolated `kappa`, `vx`.
3. **Build a KD-tree** over the resampled `(x, y)` for nearest-point projection.
4. **Closed-loop wrap**: store total length `L_track`; index arithmetic is modulo `N`.

Query API (headless, no ROS):

- `project(x, y, hint_idx=None) -> (s, idx, e_lat, psi_ref, kappa_ref, vx_ref)`
  - Nearest point on the polyline. If `hint_idx` given, restrict search to a **forward/backward window** around it (e.g. ±W points) for speed and to avoid snapping across the loop. Refine between the two nearest samples by projecting onto the local segment.
  - `e_lat` = **signed** lateral (contouring) error: positive to the left of the path tangent. Compute via the 2D cross product of the unit tangent `t = (cosψ_ref, sinψ_ref)` and `(p − p_ref)`.
- `sample(s) -> (x, y, psi, kappa, vx)` — spline/interp lookup for terminal-cost or lookahead queries.

**Speed scaling** lives here or in the cost: `v_ref_used(s) = clip(α · vx(s), v_min, v_max)`. Default `α = 0.5`.

**Batched projection for rollouts:** the cost step must project `K×T` predicted points. Provide a vectorized `project_batch(points, around_idx, window)` that:
- restricts candidates to a forward window of length `window` starting near the car's current projected index (rollouts are local, so a window of e.g. 2–4 m of arc length is sufficient), and
- returns nearest index + signed lateral error for every point at once.
This windowing is what makes the cost real-time on GPU; a global KD-tree query per rollout point is too slow.

---

## 5. Vehicle model (rollout dynamics)

**Default: kinematic bicycle, rear-axle reference, with steering and speed as states.** Chosen because it's robust, cheap to batch, and matches the gym command interface well at scaled speeds.

State (5): `X = [x, y, θ, v, δ]`
Control (2): `U = [δ̇, a]` (steering rate, longitudinal acceleration)

Continuous dynamics:
```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = (v / L)·tan(δ)
v̇ = a
δ̇ = δ̇_cmd
```
- `L` = wheelbase (F1TENTH ≈ 0.33 m).
- Discretize with **semi-implicit (symplectic) Euler** at `dt` (update `v, δ` first, then propagate pose) — adequate at `dt = 0.05`. Offer RK4 as a config flag for higher fidelity / larger `dt`.
- Enforce limits **after** integration: clip `δ ∈ [−δ_max, δ_max]`, `v ∈ [v_min, v_max]`.

**Why sample `δ̇` and `a` (rates) instead of `δ` and `v` directly:** integrating rates yields smooth steering/speed trajectories and lets the steering-rate limit be a natural constraint, which matters a lot for chatter on the real servo.

**Command mapping (what gets published):** after the optimal `U*` is computed, apply only `U*₀` for one step and publish:
- `steering_angle = clip(δ_cur + δ̇*₀·dt, ±δ_max)`
- `speed = clip(v_cur + a*₀·dt, v_min, v_max)`
Persist the published steering angle as `δ_cur` for the next cycle's internal state.

**Optional upgrade (Gate 4): dynamic single-track (ST) model** with linear/Pacejka tires, matching the gym ST parameters, for high-`α` regimes where slip matters. Keep the same control interface so the cost and node are unchanged.

---

## 6. MPPI formulation

### 6.1 Notation
- `T` horizon steps, `K` samples, `dt` step.
- `Û = (û₀ … û_{T−1})` nominal (mean) control sequence, carried across cycles.
- `Σ = diag(σ_δ̇², σ_a²)` sampling covariance; `λ` temperature (inverse-confidence).

### 6.2 Per-cycle algorithm
1. Read current state `x₀ = [x, y, θ, v, δ_cur]`. Project to get `idx₀` (warm-start hint for windowed projection).
2. For `k = 1…K`, for `t = 0…T−1`: draw `ε_t^k ~ N(0, Σ)`. Perturbed control `v_t^k = clip(û_t + ε_t^k, U_min, U_max)`.
   - **Recommended:** time-correlate the noise (smooth `ε` along `t`, e.g. low-pass / colored noise, or sample in an AR(1) form) to produce smoother rollouts and reduce steering chatter. Make this a config toggle.
3. Roll out each `k` through §5 dynamics from `x₀` to get state trajectory `{x_t^k}`.
4. Accumulate cost (per §7): `S^k = φ(x_T^k) + Σ_t [ q(x_t^k) + control_cost(û_t, ε_t^k) ]`.
5. Baseline-subtract for numerical stability: `ρ = min_k S^k`. Weights `w^k = exp(−(1/λ)(S^k − ρ))`, then normalize `w^k ← w^k / Σ_j w^j`.
   - Edge case: if all `S^k` are equal / non-finite, fall back to uniform weights and log a warning.
6. Update nominal: `û_t ← û_t + Σ_k w^k · ε_t^k` for all `t`. Clip to limits.
7. **(Optional) smooth** `Û` across time (Savitzky–Golay or moving average) to further suppress chatter.
8. Extract command from `û₀` per §5 mapping; publish.
9. **Warm start / receding shift:** `û_t ← û_{t+1}` for `t < T−1`; set `û_{T−1}` to a default (zeros, or a curvature-feedforward term `δ̇=0, a` toward `v_ref`).

### 6.3 Control cost term
Two acceptable forms — pick one, expose as config:
- **Vanilla (quadratic effort):** `control_cost = w_a·a_t² + w_δ̇·δ̇_t²` evaluated on the perturbed control. Simple, works well; start here.
- **Information-theoretic:** add `λ · û_tᵀ Σ⁻¹ ε_t^k` (importance-sampling correction). More principled; enable once vanilla is stable.

### 6.4 Real-time shaping
- `T·dt` = lookahead horizon. At `v ≈ 5 m/s`, `dt=0.05`, `T=30` → 1.5 s ≈ 7.5 m. Scale `T` so the horizon covers a few car lengths beyond the braking distance at current speed.
- Fresh noise each cycle; seed the RNG from config for reproducible tests, but reseed/advance per cycle in deployment.

---

## 7. Cost function

For each predicted state `x_t^k`, project onto the raceline (windowed) to get `e_lat`, `ψ_ref`, `v_ref_used`. Running cost:

```
q(x_t) =  w_ct · e_lat²                          # contouring / cross-track
        + w_head · wrap(θ − ψ_ref)²              # heading alignment  (wrap to [−π,π])
        + w_v  · (v − v_ref_used)²               # scaled speed tracking
        + w_a  · a²  +  w_δ̇ · δ̇²                  # control effort/smoothness (if not in control_cost)
        + w_lim · barrier(e_lat)        [opt]    # track-limit soft barrier (Gate 3)
```

Terminal cost `φ(x_T)`: same contour + heading + speed terms, optionally with larger weights to stabilize the horizon end. Optionally add a small **progress reward** (negative cost proportional to arc-length advanced) if pure speed-tracking under-drives at standstill; usually unnecessary because `v_ref_used > 0` already pulls `v` up from rest.

**Track-limit barrier (optional, Gate 3):** using centerline widths or a fixed half-width `w_half`, penalize `|e_lat|` approaching the boundary, e.g. a one-sided quadratic past a threshold, or `+∞`/large constant if outside (hard kill of that rollout). Requires the centerline file or map.

**Angle wrapping is mandatory** wherever headings are differenced. A single missed wrap will cause violent steering near the ψ = 0/2π seam.

---

## 8. Hyperparameters & config schema

Provide a ROS 2 params YAML. Starting values below are **tuning seeds**, not final. Document each so tuning is traceable.

```yaml
mppi_controller:
  ros__parameters:
    # --- raceline ---
    raceline_csv: "/path/to/Spielberg_raceline.csv"
    resample_ds: 0.10          # m, uniform arc-length resampling
    speed_scale_alpha: 0.5     # α: fraction of raceline speed to track
    v_min: 0.0                 # m/s (0 = no reverse)
    v_max: 8.0                 # m/s, clamp on v_ref and rollout speed
    frame_offset: [0.0, 0.0, 0.0]   # x0,y0,yaw0 raceline->localization alignment

    # --- vehicle model ---
    wheelbase_L: 0.33          # m
    delta_max: 0.4189          # rad (~24°), gym s_max
    delta_rate_max: 3.2        # rad/s, gym sv_max
    a_max: 7.0                 # m/s^2
    a_min: -7.0                # m/s^2 (braking)
    integrator: "symplectic_euler"   # or "rk4"

    # --- MPPI core ---
    horizon_T: 30
    num_samples_K: 1024        # CPU; raise to 2048-4096 on GPU
    dt: 0.05                   # s  -> 1.5 s lookahead at T=30
    lambda_temperature: 0.10   # tune with cost scale
    sigma_delta_rate: 0.40     # rad/s, steering-rate sampling std
    sigma_accel: 2.0           # m/s^2, accel sampling std
    colored_noise: true        # time-correlate sampled noise
    control_cost_mode: "vanilla"   # or "information_theoretic"
    smooth_nominal: true       # Savitzky-Golay on Û after update

    # --- cost weights ---
    w_cross_track: 20.0
    w_heading: 5.0
    w_speed: 2.0
    w_accel: 0.01
    w_steer_rate: 0.01
    w_track_limit: 0.0         # 0 disables; enable in Gate 3
    terminal_scale: 2.0        # multiplies contour/heading/speed at t=T

    # --- ROS / runtime ---
    control_rate_hz: 20.0
    odom_topic: "/ego_racecar/odom"   # sim; "/pf/pose/odom" on car
    drive_topic: "/drive"
    publish_viz: true
    backend: "numpy"           # "numpy" | "torch_cpu" | "torch_cuda"
    rng_seed: 0
```

---

## 9. ROS 2 node interface

**Subscriptions**
- `odom_topic` → `nav_msgs/Odometry`: pose `(x, y, yaw from quaternion)` and `twist.linear.x` for `v`. (On the car, `v` may be better taken from VESC; expose a param.)

**Publications**
- `drive_topic` → `ackermann_msgs/AckermannDriveStamped` (`steering_angle`, `speed`).
- `/mppi/optimal_path` → `nav_msgs/Path` (the optimal rollout, for RViz).
- `/mppi/reference_path` → `nav_msgs/Path` (resampled raceline, latched, for RViz).
- `/mppi/samples` → `visualization_msgs/MarkerArray` (subset of sampled rollouts; **throttle** — draw ~50, not all K).

**Control loop**
- Timer at `control_rate_hz`. The solver must complete within `1/rate` seconds. If it overruns, log and **skip publishing a stale command** rather than queueing.
- Latency hygiene: timestamp the state used; do not block the loop on viz publishing (publish viz on a slower timer or every Nth cycle).

**Safety**
- If no odom for `> timeout` (param), publish zero speed.
- Clamp every published command to limits as a final guard, independent of the solver.

---

## 10. Module / file structure

```
mppi_controller/
├── package.xml
├── setup.py / CMakeLists.txt
├── config/
│   └── mppi_params.yaml
├── mppi_controller/
│   ├── raceline_model.py        # parse, resample, project, sample (headless)
│   ├── vehicle_model.py         # kinematic bicycle (+ optional ST), batched rollout
│   ├── cost.py                  # running + terminal cost, batched
│   ├── mppi_core.py             # sampling, rollout orchestration, weighting, warm start
│   ├── backends.py              # numpy / torch abstraction for the hot loops
│   ├── mppi_planner.py          # MPPIPlanner(ReactivePlannerBase): headless plan(state)->control
│   └── mppi_node.py             # ROS 2 wrapper: subs/pubs/timer/viz only
├── test/
│   ├── test_raceline_model.py
│   ├── test_vehicle_model.py
│   ├── test_cost.py
│   └── test_mppi_core.py
└── launch/
    └── mppi.launch.py
```

Hard rule: **no ROS imports** in `raceline_model / vehicle_model / cost / mppi_core / mppi_planner`. ROS only in `mppi_node`. This keeps the solver unit-testable and reusable across sim and car.

---

## 11. Compute backend

MPPI is embarrassingly parallel over `K`. Provide a thin backend abstraction so the same math runs on:
- **numpy** — reference correctness, small `K`, CI.
- **torch_cuda** — production speed. The whole rollout (state `K×T×5`), cost, and weighting should be fully vectorized tensor ops with **no Python loop over `K`**; the only loop is over `T` (sequential dynamics) and even that stays vectorized across `K`.

The windowed batched projection (§4) is the one nontrivial piece on GPU: precompute the resampled raceline as a tensor, gather a forward window of candidate points around the car's current index, and compute nearest index + signed lateral error for all `K×T` points with broadcasted distance math.

Given available hardware (RTX 5090 / PRO 6000 Blackwell), `K = 2048–4096`, `T = 30` is comfortably real-time at 20–50 Hz. Validate the actual per-cycle solve time as an acceptance metric, not an assumption.

---

## 12. Staged delivery gates

Each gate has a demo and acceptance criteria. Do not advance until the current gate's criteria pass.

### Gate 0 — Scaffolding & raceline loader
- Package builds; params load; `RacelineModel` parses a real `*_raceline.csv` (e.g. Spielberg), resamples, builds KD-tree.
- **Accept:** unit tests pass for parse (correct `;`/`#` handling), ψ wrapping, closed-loop `s` wrap, and `project()` returning correct nearest `s` and signed `e_lat` on hand-checked points. A plot of resampled raceline overlays the raw CSV.

### Gate 1 — MPPI core, headless (numpy)
- `vehicle_model` batched rollout; `cost`; `mppi_core` sampling/weighting/warm-start; `MPPIPlanner` conforming to `ReactivePlannerBase`.
- **Accept:** from a fixed synthetic state on a known raceline, the planner returns a command that steers toward the line and tracks `v_ref` (verify by inspection + a deterministic seeded test). Unit tests: control limits respected after clip, weights sum to 1, baseline subtraction stable, all-equal-cost fallback works, rollout determinism under fixed seed.

### Gate 2 — Closed loop in f1tenth_gym
- `mppi_node` wired; runs in the gym on a downloaded track.
- **Accept:** completes ≥1 full lap at `α = 0.5` without leaving the track. Report: max & RMS cross-track error, max heading error, lap time, mean/95p solve time per cycle. RViz shows reference path, optimal path, and a sample of rollouts.

### Gate 3 — Performance, viz, hardening
- torch_cuda backend; windowed batched projection; track-limit barrier; viz throttling; tuning pass.
- **Accept:** real-time at target `control_rate_hz` with `K ≥ 2048` (95p solve time < period). Stable laps at progressively higher `α` (e.g. 0.5 → 0.7 → 0.9), documenting where tracking degrades. Track-limit cost demonstrably prevents boundary cuts.

### Gate 4 — (Optional) car deployment & dynamic model
- Frame-alignment verification on the real map; odom/VESC speed source; dynamic ST model option for high-`α`.
- **Accept:** stable indoor laps at conservative `α`; documented deployment checklist (frame offset, topic remaps, e-stop behavior, command clamps).

---

## 13. Testing & validation

**Unit (headless, CI):**
- Raceline: parse correctness, resampling monotonic `s`, projection nearest/sign, wrap at loop seam.
- Vehicle model: known-input rollout matches hand integration; limits enforced; RK4 vs Euler sanity.
- Cost: heading wrap correctness; speed term zero at `v = v_ref_used`; monotonic in `e_lat`.
- Core: weight normalization, baseline subtraction, degenerate-cost fallback, warm-start shift correctness, seeded determinism.

**Integration (sim):**
- Headless gym rollout scoring lap completion + the metrics in Gate 2.
- Sweep `α` and key weights; record metrics to a CSV for the tuning log.

**Metrics to log every run:** lap time, RMS/max lateral error, RMS/max heading error, mean/95p/max solve time, command rate, any limit clamps hit.

---

## 14. Integration checks / open questions (resolve before Gate 2)

1. **`ReactivePlannerBase` signature** — confirm the exact method names and lifecycle so `MPPIPlanner` matches. Flag if it doesn't exist.
2. **Frame alignment** — confirm raceline CSV frame == localization `map` frame for the chosen track/map pair. Overlay raceline on the occupancy map; set `frame_offset` if there's a translation/rotation. This is the single most likely source of silent failure.
3. **Speed source on car** — odom twist vs VESC; pick and parameterize.
4. **Steering state** — confirm there's no servo feedback; rely on internal `δ` integration initialized from last command.
5. **Gym command semantics** — confirm the gym env step consumes `[steering_angle, speed]` and reaches them via its own low-level controller, so publishing `(δ, v)` is correct rather than publishing rates.

---

## 15. Tuning guide (starting heuristics)

- Set cost-term scales so contouring dominates, then heading, then speed; effort terms small.
- `λ` couples to total cost magnitude: too small → overconfident, jittery / collapses to one sample; too large → mushy, ignores cost. Tune `λ` after weights are roughly set; watch effective sample size `1 / Σ(w^k)²`.
- `σ_δ̇`, `σ_a`: must be large enough to explore feasible maneuvers within one horizon but small enough to stay near feasible. If the car can't make a corner, increase `σ_δ̇` or `T`; if it's twitchy, decrease `σ_δ̇` and/or enable colored noise + nominal smoothing.
- Raise `α` only after the geometry tracks cleanly at `α = 0.5`.
- If steering chatters: enable `colored_noise`, enable `smooth_nominal`, lower `σ_δ̇`, raise `w_steer_rate`.

---

## 16. Future extensions (architecture should not preclude)

- Obstacle / opponent cost layer (occupancy or detected boxes) added to `cost.py` without touching core.
- Dynamic ST model swap in `vehicle_model.py` (same control interface).
- Online speed-scale scheduling (`α` as a function of localization confidence or lap count).
- MPCC-style explicit lag/contour split with a virtual arc-length state if progress control becomes important.
- Replace fixed raceline with a planner-provided local reference (drop-in `RacelineModel` interface).
