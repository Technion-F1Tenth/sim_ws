"""
Offline extraction: sysid rosbag2 -> one row per executed command step.

This tool reads `/sysid/meta` (`mrs_roboracer_sysid_msgs/SysidMeta`) plus the
raw ground-truth pose/twist/odom streams from a sysid session bag and joins
them into a flat table for downstream noise-model fitting.

It explicitly does NOT:
  - evaluate the motion model f(x_l, u_l) or compute residuals e_l,
  - compute B_l = (df/du)*dt or any orthogonal-complement projection
    (see `motion_model.jacobian_df_du` for that future tool),
  - compute coverage/binning statistics (see the `*_coverage_report.json`
    written by sysid_data_collector at session end),
  - perform sync-chirp cross-correlation / lag estimation.

Output columns: path_id, step_index, phase, t_cmd, t_pose_start, t_pose_end, dt,
x_l, y_l, theta_l, v_meas_l, x_lp1, y_lp1, theta_lp1, v_cmd, delta_cmd,
kappa_cmd, coverage_bin, lat_accel_est, flag_sync_chirp, flag_high_lat_accel,
flag_stale_pose, segment_id, u1_cmd, u2_cmd, v_ref, delta_ref, dt_cmd,
thetadot_meas_l,
flag_safety_override, plus optional source-tagged v_meas_l_mocap_twist /
v_meas_l_onboard_odom columns if the corresponding topics are present.

`flag_safety_override` is true when a `/drive` message published during the
step differs from the `/safe_drive` command it relays (clipping or AEB), i.e.
the car did not receive what `/sysid/meta` says was commanded.

With `--segments-output`, also writes the per-segment input-noise residual
table for held-rate-step programs (`paths.mode: rate_steps`); see
`segments.py`. Fit it with `sysid_fit_input_noise`.
"""

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mrs_roboracer_sysid.motion_model import quaternion_to_yaw
from mrs_roboracer_sysid.segments import compute_residuals, mark_aborted, segment_table
from mrs_roboracer_sysid.velocity_estimators import (
    MocapTwistEstimator,
    OnboardOdomEstimator,
    PoseDerivativeEstimator,
)


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _read_topics(bag_path: str, topics) -> dict:
    """
    Return per-topic message lists, sorted by header stamp.

    Returns {topic: [(t_header_sec, msg), ...]} for each topic that carries
    a std_msgs/Header. Topics absent from the bag are returned with an
    empty list.
    """
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(topics)
    messages = {topic: [] for topic in topics}

    while reader.has_next():
        topic, data, _recv_time = reader.read_next()
        if topic not in wanted:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        messages[topic].append((_stamp_to_sec(msg.header.stamp), msg))

    for topic in messages:
        messages[topic].sort(key=lambda pair: pair[0])
    return messages


def _nearest_at_or_before(times: np.ndarray, t: float) -> int:
    """
    Return the index of the largest `times[i] <= t`.

    Returns -1 if `times` is empty or `t` is before the first sample.
    """
    if len(times) == 0 or np.isnan(t):
        return -1
    idx = int(np.searchsorted(times, t, side='right')) - 1
    return idx if idx >= 0 else -1


def _build_pose_arrays(pose_msgs):
    t = np.array([ts for ts, _ in pose_msgs], dtype=float)
    x = np.array([m.pose.position.x for _, m in pose_msgs], dtype=float)
    y = np.array([m.pose.position.y for _, m in pose_msgs], dtype=float)
    theta = np.array([
        quaternion_to_yaw(m.pose.orientation.x, m.pose.orientation.y,
                          m.pose.orientation.z, m.pose.orientation.w)
        for _, m in pose_msgs
    ], dtype=float)
    return t, x, y, theta


_OVERRIDE_TOL = 1e-3


def _override_times(drive_msgs, safe_drive_msgs) -> np.ndarray:
    """
    Stamps of `/drive` messages that differ from the `/safe_drive` they relay.

    safety_node restamps `/drive` with its publish time, so each `/drive`
    is matched to the latest `/safe_drive` stamped at or before it.
    """
    if not drive_msgs or not safe_drive_msgs:
        return np.array([], dtype=float)
    safe_t = np.array([ts for ts, _ in safe_drive_msgs], dtype=float)
    out = []
    for ts, msg in drive_msgs:
        idx = _nearest_at_or_before(safe_t, ts)
        if idx < 0:
            continue
        ref = safe_drive_msgs[idx][1].drive
        if (abs(msg.drive.speed - ref.speed) > _OVERRIDE_TOL
                or abs(msg.drive.steering_angle - ref.steering_angle) > _OVERRIDE_TOL):
            out.append(ts)
    return np.array(out, dtype=float)


def extract_dataset(bag_path: str, pose_topic: str, twist_topic: str, odom_topic: str,
                    meta_topic: str, sg_window: int, sg_order: int,
                    drive_topic: str = '/drive',
                    safe_drive_topic: str = '/safe_drive') -> pd.DataFrame:
    """Return the per-step table (see module docstring)."""
    return _extract(bag_path, pose_topic, twist_topic, odom_topic, meta_topic,
                    sg_window, sg_order, drive_topic, safe_drive_topic)[0]


def _extract(bag_path, pose_topic, twist_topic, odom_topic, meta_topic, sg_window, sg_order,
             drive_topic, safe_drive_topic):
    """Return (per-step DataFrame, pose_t, v_meas, thetadot_meas)."""
    topics = [meta_topic, pose_topic]
    for topic in (twist_topic, odom_topic, drive_topic, safe_drive_topic):
        if topic:
            topics.append(topic)

    streams = _read_topics(bag_path, topics)

    metas = streams[meta_topic]
    if not metas:
        raise ValueError(f"no messages found on meta topic '{meta_topic}' in {bag_path}")
    metas.sort(key=lambda pair: pair[0])

    pose_msgs = streams[pose_topic]
    if not pose_msgs:
        raise ValueError(f"no messages found on pose topic '{pose_topic}' in {bag_path}")
    pose_t, pose_x, pose_y, pose_theta = _build_pose_arrays(pose_msgs)
    v_meas, thetadot_meas = PoseDerivativeEstimator.compute_batch(
        pose_t, pose_x, pose_y, pose_theta, window=sg_window, polyorder=sg_order)

    twist_msgs = streams.get(twist_topic, []) if twist_topic else []
    odom_msgs = streams.get(odom_topic, []) if odom_topic else []
    odom_t = np.array([ts for ts, _ in odom_msgs], dtype=float)
    twist_t = np.array([ts for ts, _ in twist_msgs], dtype=float)
    override_t = _override_times(streams.get(drive_topic, []) if drive_topic else [],
                                 streams.get(safe_drive_topic, []) if safe_drive_topic else [])

    rows = []
    for i, (t_cmd, meta) in enumerate(metas):
        t_cmd_next = metas[i + 1][0] if i + 1 < len(metas) else float('nan')

        idx_start = _nearest_at_or_before(pose_t, t_cmd)
        idx_end = _nearest_at_or_before(pose_t, t_cmd_next)

        t_pose_start = pose_t[idx_start] if idx_start >= 0 else float('nan')
        t_pose_end = pose_t[idx_end] if idx_end >= 0 else float('nan')

        row = {
            'path_id': meta.path_id,
            'step_index': meta.step_index,
            'phase': meta.phase,
            't_cmd': t_cmd,
            't_pose_start': t_pose_start,
            't_pose_end': t_pose_end,
            'dt': (t_pose_end - t_pose_start) if idx_end >= 0 and idx_start >= 0 else float('nan'),
            'x_l': pose_x[idx_start] if idx_start >= 0 else float('nan'),
            'y_l': pose_y[idx_start] if idx_start >= 0 else float('nan'),
            'theta_l': pose_theta[idx_start] if idx_start >= 0 else float('nan'),
            'v_meas_l': v_meas[idx_start] if idx_start >= 0 else float('nan'),
            'x_lp1': pose_x[idx_end] if idx_end >= 0 else float('nan'),
            'y_lp1': pose_y[idx_end] if idx_end >= 0 else float('nan'),
            'theta_lp1': pose_theta[idx_end] if idx_end >= 0 else float('nan'),
            'v_cmd': meta.v_cmd,
            'delta_cmd': meta.delta_cmd,
            'kappa_cmd': meta.kappa_cmd,
            'coverage_bin': meta.coverage_bin_id,
            'lat_accel_est': meta.lat_accel_est,
            'flag_sync_chirp': bool(meta.flag_sync_chirp),
            'flag_high_lat_accel': bool(meta.flag_high_lat_accel),
            'flag_stale_pose': bool(meta.flag_stale_pose),
            'segment_id': meta.segment_id,
            'u1_cmd': meta.u1_cmd,
            'u2_cmd': meta.u2_cmd,
            'v_ref': meta.v_ref,
            'delta_ref': meta.delta_ref,
            'dt_cmd': meta.dt,
            'thetadot_meas_l': thetadot_meas[idx_start] if idx_start >= 0 else float('nan'),
            'flag_safety_override': bool(np.any(
                (override_t >= t_cmd) & (override_t < t_cmd_next)))
            if np.isfinite(t_cmd_next) else bool(np.any(override_t >= t_cmd)),
        }

        if twist_msgs:
            idx_twist = _nearest_at_or_before(twist_t, t_cmd)
            if idx_twist >= 0:
                tw = twist_msgs[idx_twist][1].twist
                row['v_meas_l_mocap_twist'] = MocapTwistEstimator.velocity(
                    tw.linear.x, tw.linear.y, pose_theta[idx_start] if idx_start >= 0 else 0.0)
            else:
                row['v_meas_l_mocap_twist'] = float('nan')

        if odom_msgs:
            idx_odom = _nearest_at_or_before(odom_t, t_cmd)
            if idx_odom >= 0:
                row['v_meas_l_onboard_odom'] = OnboardOdomEstimator.velocity(
                    odom_msgs[idx_odom][1].twist.twist.linear.x)
            else:
                row['v_meas_l_onboard_odom'] = float('nan')

        rows.append(row)

    return pd.DataFrame(rows), pose_t, v_meas, thetadot_meas


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_path', help='Path to the input rosbag2 directory.')
    parser.add_argument('output_path', help='Path to write the output dataset.')
    parser.add_argument('--pose-topic', default='/sysid/pose')
    parser.add_argument('--twist-topic', default='/sysid/twist',
                        help='Set to "" to skip mocap-twist comparison column.')
    parser.add_argument('--odom-topic', default='/ego_racecar/odom',
                        help='Set to "" to skip onboard-odom comparison column.')
    parser.add_argument('--meta-topic', default='/sysid/meta')
    parser.add_argument('--sg-window', type=int, default=9)
    parser.add_argument('--sg-order', type=int, default=2)
    parser.add_argument('--drive-topic', default='/drive',
                        help='Arbitrated command topic (safety_node output); "" to skip '
                             'override detection.')
    parser.add_argument('--safe-drive-topic', default='/safe_drive')
    parser.add_argument('--format', choices=['csv', 'parquet'], default='csv')
    parser.add_argument('--segments-output',
                        help='Also write the per-segment input-noise residual table '
                             '(rate_steps sessions) here, as CSV.')
    parser.add_argument('--wheelbase', type=float, default=0.33)
    parser.add_argument('--delta-max', type=float, default=0.4189)
    parser.add_argument('--lag-s', type=float, default=0.0,
                        help='Command-to-response delay; shifts each segment\'s measurement '
                             'window by this much.')
    parser.add_argument('--min-arc-m', type=float, default=0.2,
                        help='Steering-channel arc-length gate (v*T).')
    parser.add_argument('--aborted-log',
                        help='Collector abort log (default: <bag_path>_aborted.json if it '
                             'exists). Flags the segment each abort cut short.')
    args = parser.parse_args(argv)

    df, pose_t, v_meas, thetadot_meas = _extract(
        args.bag_path, args.pose_topic, args.twist_topic, args.odom_topic, args.meta_topic,
        args.sg_window, args.sg_order, args.drive_topic, args.safe_drive_topic)

    aborted_log = args.aborted_log or os.path.normpath(args.bag_path) + '_aborted.json'
    aborted = []
    if os.path.exists(aborted_log):
        with open(aborted_log) as f:
            aborted = json.load(f).get('aborted', [])
        print(f"{len(aborted)} aborted program(s) in {aborted_log}")
    elif args.aborted_log:
        raise FileNotFoundError(aborted_log)
    df = mark_aborted(df, aborted)

    if args.format == 'csv':
        df.to_csv(args.output_path, index=False)
    else:
        df.to_parquet(args.output_path, index=False)

    print(f"wrote {len(df)} rows to {args.output_path}")

    if args.segments_output:
        seg = segment_table(df)
        if seg.empty:
            print("no measurement segments (segment_id >= 0) in this bag; "
                  "was it recorded with paths.mode: rate_steps?")
            return 1
        seg = compute_residuals(seg, pose_t, v_meas, thetadot_meas, L=args.wheelbase,
                                lag_s=args.lag_s, min_arc_m=args.min_arc_m,
                                delta_max=args.delta_max)
        seg.to_csv(args.segments_output, index=False)
        print(f"wrote {len(seg)} segments to {args.segments_output}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
