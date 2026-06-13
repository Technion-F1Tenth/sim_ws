from mrs_roboracer_sysid.pose_watchdog import PoseStalenessWatchdog, WatchdogStatus


def test_ok_when_pose_recent():
    wd = PoseStalenessWatchdog(pose_timeout_ms=200.0)
    assert wd.check(now=10.0, last_pose_time=9.95) == WatchdogStatus.OK


def test_stale_when_pose_old():
    wd = PoseStalenessWatchdog(pose_timeout_ms=200.0)
    assert wd.check(now=10.0, last_pose_time=9.5) == WatchdogStatus.POSE_STALE


def test_stale_when_no_pose_ever_received():
    wd = PoseStalenessWatchdog(pose_timeout_ms=200.0)
    assert wd.check(now=10.0, last_pose_time=None) == WatchdogStatus.POSE_STALE


def test_boundary_is_stale():
    wd = PoseStalenessWatchdog(pose_timeout_ms=100.0)
    # just under the timeout -> ok; just over -> stale (strictly greater triggers)
    assert wd.check(now=1.099, last_pose_time=1.0) == WatchdogStatus.OK
    assert wd.check(now=1.101, last_pose_time=1.0) == WatchdogStatus.POSE_STALE
