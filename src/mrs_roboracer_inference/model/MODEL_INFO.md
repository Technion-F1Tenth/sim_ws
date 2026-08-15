# roboracer policy export

## Files

- `policy.pt` — trained weights, plain `torch.save`d `OrderedDict` state dict
  (loadable with `weights_only=True`, no custom classes pickled in).
- `model.py` — self-contained PyTorch `nn.Module` definitions that reproduce
  the exact architecture `policy.pt` was trained with, plus `load_policy()`
  to build+load in one call. No PufferLib installation required to run this
  file; run `python3 model.py` for a smoke test.
- `checkpoint_run_meta.json` — provenance/metrics for this specific checkpoint,
  written by `run.py` at training time.

## Provenance

Trained in this repo (`puffertank/roboracer`) via
`python ocean/roboracer/run.py --arm enc_default_b64 --seed 1 --slowly
--torch.encoder DefaultEncoder --train.total-timesteps 100_000_000`,
config otherwise from `roboracer.ini` (TAL reward, `reward_mode=tal`, 18 real
F1TENTH circuits, `anneal_ent_coef=0`). Checkpoint step 99,876,864 of
100,000,000, selected as the best of that run's checkpoints on the **train**
split (per-checkpoint selection, see `run.py --select-on train`), then scored
on the held-out **val** split (5 circuits not seen during training):

| metric | value |
|---|---|
| `lap_completion_rate` | **0.987** |
| `crash_frac` | 0.203 |
| `crashes_per_min` | 0.010 |
| `lap_time` | 51.1 s |
| `speed` | 7.64 m/s |
| `progress_mps` | 7.67 |

This is the best-scoring checkpoint, among all runs in `results.csv` whose
architecture matches the **current** `roboracer.h` (`N_BEAMS = 64`). A small
number of runs (`enc_default_b256`, `enc_convnet_b256`, up to 0.993
`lap_completion_rate`) scored marginally higher but were trained with
`N_BEAMS = 256`, an observation layout the current env no longer produces —
not used here for that reason.

Eval numbers above are from **stochastic** rollout (the training-time action
distribution is sampled, matching how `lap_completion_rate` is normally
reported in this repo). See "Inference" below for the deterministic-vs-
stochastic tradeoff at deployment time.

## Architecture

```
observation [B, 257]
    -> DefaultEncoder: Linear(257 -> 128)
    -> MinGRU recurrent core: 4 layers, hidden_size 128
    -> DefaultDecoder:
         - decoder_mean: Linear(128 -> 2)      -> Normal mean
         - decoder_logstd: Parameter [1, 2]    -> Normal log-std (state-independent)
         - value_function: Linear(128 -> 1)    -> critic value (inference doesn't need this)
```

~230K parameters total. The recurrent core is a
[minGRU](https://arxiv.org/abs/2410.01201v1) (parallel-scannable during
training via `forward_train`; a plain per-step recurrence during rollout via
`forward_eval`, which is what `model.py`'s `Policy.forward_eval` calls and
what real-time inference should use). State carried between steps is a single
tensor of shape `[num_layers=4, batch, hidden_size=128]`, initialised to
zeros at the start of an episode (`Policy.initial_state`).

## Observation — 257 floats, must be assembled in this exact order

```
obs[0:64]    = LIDAR scan at t-3   (oldest)
obs[64:128]  = LIDAR scan at t-2
obs[128:192] = LIDAR scan at t-1
obs[192:256] = LIDAR scan at t     (newest / current)
obs[256]     = current forward speed v, normalised: v / V_MAX
```

Each LIDAR scan is 64 beams, **normalised distance in [0, 1]**
(`raw_distance_m / MAX_RANGE`, clamped to `[0, 1]` — a reading beyond max
range or negative from noise must be clamped, not left out-of-band; the
policy never saw out-of-[0,1] values during training). Beam `i` (`0..63`) is
at body-frame angle:

```
angle_i = -FOV/2 + i * (FOV / 63),   FOV = 270 deg (4.712 rad)
```

i.e. beam 0 is 135° to one side of the car's heading, beam 63 is 135° to the
other side, beam ~31.5 points straight ahead — a fixed fan relative to the
car body, independent of world heading. `MAX_RANGE = 30.0 m`. `V_MAX = 10.0
m/s` (the training config's speed cap used for normalisation — use this exact
value even if a deployed vehicle's real top speed differs, since it's a
normalisation constant baked into the trained weights, not a physical limit
to enforce).

On the very first step of an episode/rollout there is no history yet — prime
all 4 stacked slots with the same first-frame scan (`compute_obs_reset` in
`roboracer.h`) rather than leaving stale/zero data in the older slots.

LIDAR beams were simulated with `LIDAR_NOISE_STD = 0.01` (m, Gaussian, added
post ray-cast) during this checkpoint's training — real sensor noise at a
similar scale should be in-distribution; a much noisier real sensor may need
retraining or explicit noise augmentation.

## Action — `Normal(mean, std)` over 2 continuous dims

```
action[0] = steering_velocity_cmd   (rad/s),  |.| <= MAX_STEER_VEL = 3.2 rad/s
action[1] = acceleration_cmd        (m/s^2),  |.| <= MAX_ACCEL     = 9.51 m/s^2
```

The network's raw output is **unclamped** (`decoder_mean` is a plain linear
layer, no tanh/sigmoid squashing) — the simulator clamps to the bounds above
internally every substep (`c_step` in `roboracer.h`), and a real-vehicle
integration must apply the same clamp itself, since the policy relies on it
and was never penalised for exceeding it. Note `action[0]` is a steering
**rate** (rad/s) to integrate against the current steering angle, not an
absolute steering angle — the vehicle's actual steering angle `delta` is
itself clamped to `MAX_STEER = 0.4189 rad` (~24°) by the environment's
`steering_constraint`, not by the policy.

The policy runs at the simulator's control rate: `DT * N_SUBSTEPS = 0.01 *
5 = 0.05 s` per env step → **20 Hz**. A physical control loop should run at
this rate (or the policy's recurrent state / action deltas will represent a
different real-world dt than it was trained on).

## Inference

```python
from model import load_policy
import torch

policy = load_policy("policy.pt")               # eval() mode, weights loaded
state = policy.initial_state(batch_size=1, device="cpu")

# per control step, at 20 Hz:
obs = torch.as_tensor(obs_257_float_vector).unsqueeze(0)   # [1, 257]
with torch.no_grad():
    dist, value, state = policy.forward_eval(obs, state)
    action = dist.mean            # deterministic (recommended for deployment)
    # action = dist.sample()      # stochastic (matches the eval numbers above)
steer_vel_cmd, accel_cmd = action[0].tolist()
```

Use `dist.mean` (not `dist.sample()`) for real-vehicle deployment —
deterministic actions are the standard recommendation for physical robots
(no risk of the very rare, high-variance action tail seen during stochastic
training rollouts). The `lap_completion_rate = 0.987` figure above was
measured stochastically; `eval.py --deterministic` in this repo measures the
mean-action variant if a directly comparable number is needed before
deploying.

`state` must be carried forward between calls (it is the minGRU's hidden
state) and reset to `policy.initial_state(1, device)` at the start of each
episode / after any discontinuous reset of the vehicle's pose.
