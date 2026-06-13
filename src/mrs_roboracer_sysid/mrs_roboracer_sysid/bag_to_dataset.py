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

Output columns: path_id, step_index, t_cmd, t_pose_start, t_pose_end, dt,
x_l, y_l, theta_l, v_meas_l, x_lp1, y_lp1, theta_lp1, v_cmd, delta_cmd,
kappa_cmd, coverage_bin, lat_accel_est, flag_sync_chirp, flag_high_lat_accel,
flag_stale_pose, plus optional source-tagged v_meas_l_mocap_twist /
v_meas_l_onboard_odom columns if the corresponding topics are present.
"""

import argparse
import sys

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mrs_roboracer_sysid.motion_model import quaternion_to_yaw
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


def extract_dataset(bag_path: str, pose_topic: str, twist_topic: str, odom_topic: str,
                    meta_topic: str, sg_window: int, sg_order: int) -> pd.DataFrame:
    topics = [meta_topic, pose_topic]
    if twist_topic:
        topics.append(twist_topic)
    if odom_topic:
        topics.append(odom_topic)

    streams = _read_topics(bag_path, topics)

    metas = streams[meta_topic]
    if not metas:
        raise ValueError(f"no messages found on meta topic '{meta_topic}' in {bag_path}")
    metas.sort(key=lambda pair: pair[0])

    pose_msgs = streams[pose_topic]
    if not pose_msgs:
        raise ValueError(f"no messages found on pose topic '{pose_topic}' in {bag_path}")
    pose_t, pose_x, pose_y, pose_theta = _build_pose_arrays(pose_msgs)
    v_meas, _thetadot = PoseDerivativeEstimator.compute_batch(
        pose_t, pose_x, pose_y, pose_theta, window=sg_window, polyorder=sg_order)

    twist_msgs = streams.get(twist_topic, []) if twist_topic else []
    odom_msgs = streams.get(odom_topic, []) if odom_topic else []
    odom_t = np.array([ts for ts, _ in odom_msgs], dtype=float)
    twist_t = np.array([ts for ts, _ in twist_msgs], dtype=float)

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

    return pd.DataFrame(rows)


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
    parser.add_argument('--format', choices=['csv', 'parquet'], default='csv')
    args = parser.parse_args(argv)

    df = extract_dataset(
        bag_path=args.bag_path,
        pose_topic=args.pose_topic,
        twist_topic=args.twist_topic,
        odom_topic=args.odom_topic,
        meta_topic=args.meta_topic,
        sg_window=args.sg_window,
        sg_order=args.sg_order,
    )

    if args.format == 'csv':
        df.to_csv(args.output_path, index=False)
    else:
        df.to_parquet(args.output_path, index=False)

    print(f"wrote {len(df)} rows to {args.output_path}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
