"""
Fit the rate-input noise covariance Sigma = diag(sigma1^2, sigma2^2).

Input: the segment table written by `sysid_bag_to_dataset --segments-output`
(see `segments.py` for the residual definitions). For each channel the
noise is modeled as a zero-mean Gaussian fit by maximum likelihood
(sigma^2 = mean(r^2)), then truncated to +/- k*sigma (default k = 3).

Selection:
  - all channels: drop segments with a stale pose, and segments at or after
    a safety_node override in the same program,
  - acceleration: both endpoint speeds >= --min-speed (excludes the
    standstill end of the run-out brake segment, where v saturates at 0),
  - steering rate: arc gate (v*T >= min_arc_m) passed and predicted steering
    angle not saturated.

Two diagnostics are reported alongside each fit, and both should be near
zero before the zero-mean model is trusted:
  - `mean`: constant bias (e.g. steering trim),
  - `corr_with_u` / `slope_vs_u`: residual vs. the commanded rate of the
    same channel. Uncompensated first-order actuator lag gives a residual
    of about -(tau/T) * (u - u_prev), so slope ~ -tau/T against u; this
    averages out of `mean` over a balanced set of segments but is not noise. Compensate it with
    steps.tau_v / steps.tau_delta (README section 13).
"""

import argparse
import json
import sys

import numpy as np
import pandas as pd


def fit_zero_mean_gaussian(r: np.ndarray, k_sigma: float = 3.0) -> dict:
    """MLE zero-mean Gaussian fit plus diagnostics; NaNs are dropped."""
    r = np.asarray(r, dtype=float)
    r = r[np.isfinite(r)]
    n = int(len(r))
    if n == 0:
        return {"n": 0}
    sigma = float(np.sqrt(np.mean(r ** 2)))
    return {
        "n": n,
        "sigma": sigma,
        "truncation": [-k_sigma * sigma, k_sigma * sigma],
        "mean": float(np.mean(r)),
        "std_free_mean": float(np.std(r)),
        "frac_outside_truncation": float(np.mean(np.abs(r) > k_sigma * sigma)),
    }


def input_dependence(r: np.ndarray, u: np.ndarray) -> dict:
    """Correlation and least-squares slope of residual `r` against input `u`."""
    r, u = np.asarray(r, dtype=float), np.asarray(u, dtype=float)
    ok = np.isfinite(r) & np.isfinite(u)
    r, u = r[ok], u[ok]
    if len(r) < 3 or np.std(u) == 0.0 or np.std(r) == 0.0:
        return {"corr_with_u": None, "slope_vs_u": None}
    return {"corr_with_u": float(np.corrcoef(u, r)[0, 1]),
            "slope_vs_u": float(np.polyfit(u, r, 1)[0])}


def select_channels(seg: pd.DataFrame, min_speed: float = 0.3,
                    steer_column: str = "steer_rate_resid"):
    """Return (accel rows, steering-rate rows, counts dict) after selection."""
    ok = ~seg["flag_stale_pose"].astype(bool) & ~seg["flag_after_override"].astype(bool)
    if "flag_segment_aborted" in seg:
        ok &= ~seg["flag_segment_aborted"].astype(bool)
    moving = seg[ok & (seg["v_meas_a"] >= min_speed) & (seg["v_meas_b"] >= min_speed)]
    accel = moving
    steer = moving[moving["gate_arc"].astype(bool)
                   & ~moving["flag_delta_saturated"].astype(bool)]
    counts = {
        "segments_total": int(len(seg)),
        "segments_clean": int(ok.sum()),
        "moving_segments": int(len(moving)),
        "steer_segments_gated": int(len(steer)),
    }
    return accel, steer, counts


def _plot(r: np.ndarray, fit: dict, title: str, xlabel: str, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.stats import truncnorm

    r = r[np.isfinite(r)]
    sigma = fit["sigma"]
    lo, hi = fit["truncation"]
    fig, ax = plt.subplots(figsize=(4.5, 3.2))
    ax.hist(r, bins=max(10, len(r) // 4), density=True, alpha=0.6)
    xs = np.linspace(min(lo, r.min()), max(hi, r.max()), 400)
    ax.plot(xs, truncnorm.pdf(xs, lo / sigma, hi / sigma, scale=sigma))
    ax.axvline(lo, linestyle="--")
    ax.axvline(hi, linestyle="--")
    ax.set_title(f"{title} (n={fit['n']}, sigma={sigma:.3f})")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("density")
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("segments_csv", nargs="+",
                        help="One or more segment tables (concatenated).")
    parser.add_argument("--min-speed", type=float, default=0.3)
    parser.add_argument("--k-sigma", type=float, default=3.0)
    parser.add_argument("--steer-reference", choices=["inferred", "cmd"], default="inferred",
                        help="Steering-angle reference at segment start: inferred from "
                             "measured yaw rate (default) or the commanded angle.")
    parser.add_argument("--output", help="Write the fit as JSON here.")
    parser.add_argument("--plot-dir", help="Write residual histograms (PNG) here.")
    args = parser.parse_args(argv)

    seg = pd.concat([pd.read_csv(p) for p in args.segments_csv], ignore_index=True)
    steer_column = ("steer_rate_resid" if args.steer_reference == "inferred"
                    else "steer_rate_resid_cmd")
    accel, steer, counts = select_channels(seg, args.min_speed, steer_column)
    r_accel = accel["accel_resid"].to_numpy()
    r_steer = steer[steer_column].to_numpy()

    result = {
        "counts": counts,
        "accel_channel_w1": {**fit_zero_mean_gaussian(r_accel, args.k_sigma),
                             **input_dependence(r_accel, accel["u1_cmd"])},
        "steer_rate_channel_w2": {**fit_zero_mean_gaussian(r_steer, args.k_sigma),
                                  **input_dependence(r_steer, steer["u2_cmd"])},
        "steer_reference": args.steer_reference,
        "lag_s": float(seg["lag_s"].iloc[0]) if "lag_s" in seg and len(seg) else None,
    }
    print(json.dumps(result, indent=2))

    if args.output:
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
    if args.plot_dir:
        import os
        os.makedirs(args.plot_dir, exist_ok=True)
        if result["accel_channel_w1"]["n"]:
            _plot(r_accel, result["accel_channel_w1"], "Acceleration channel",
                  "w1 [m/s^2]", os.path.join(args.plot_dir, "residual_acceleration.png"))
        if result["steer_rate_channel_w2"]["n"]:
            _plot(r_steer, result["steer_rate_channel_w2"], "Steering-rate channel",
                  "w2 [rad/s]", os.path.join(args.plot_dir, "residual_steering.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
