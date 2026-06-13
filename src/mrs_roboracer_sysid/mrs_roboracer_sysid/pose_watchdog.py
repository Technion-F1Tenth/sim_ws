"""
Pose-staleness watchdog.

If the pose stream stalls for longer than `pose_timeout_ms`, open-loop
execution with no ground truth is both useless and unsafe, so the node
must stop. Geofencing is explicitly out of scope -- this watchdog only
checks pose staleness.

Pure Python, no rclpy dependency: the node converts rclpy.time.Time <->
float seconds at the boundary so this module stays trivially testable.
"""

from enum import Enum
from typing import Optional


class WatchdogStatus(Enum):
    OK = "ok"
    POSE_STALE = "pose_stale"


class PoseStalenessWatchdog:
    """Reports POSE_STALE if no pose has been received within the timeout."""

    def __init__(self, pose_timeout_ms: float):
        self.pose_timeout_s = pose_timeout_ms / 1000.0

    def check(self, now: float, last_pose_time: Optional[float]) -> WatchdogStatus:
        """Return OK or POSE_STALE given the current time and last pose receive time."""
        if last_pose_time is None:
            return WatchdogStatus.POSE_STALE
        if (now - last_pose_time) > self.pose_timeout_s:
            return WatchdogStatus.POSE_STALE
        return WatchdogStatus.OK
