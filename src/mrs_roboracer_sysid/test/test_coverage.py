import pytest

from mrs_roboracer_sysid.coverage import CoverageTracker


def _tracker(**overrides):
    kwargs = dict(
        v_bins=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
        kappa_bins=[-3.0, -2.0, -1.0, -0.3, 0.3, 1.0, 2.0, 3.0],
        target_revisits=3,
        include_figure8=True,
        figure8_period=5,
        prioritize_corner_first=True,
        corner_probe_paths=2,
    )
    kwargs.update(overrides)
    return CoverageTracker(**kwargs)


def test_bin_for_in_range():
    tracker = _tracker()
    # v in [1.0, 1.5) -> v_idx 1; kappa in [-0.3, 0.3) -> kappa_idx 3
    v_idx, kappa_idx = tracker.bin_for(1.2, 0.0)
    assert (v_idx, kappa_idx) == (1, 3)


def test_bin_for_out_of_range():
    tracker = _tracker()
    assert tracker.bin_for(-1.0, 0.0) == (-1, 3)
    assert tracker.bin_for(10.0, 0.0) == (-1, 3)
    assert tracker.bin_for(1.0, 10.0) == (1, -1)
    assert tracker.bin_for(1.0, -10.0) == (1, -1)


def test_bin_id():
    tracker = _tracker()
    assert tracker.n_kappa == 7
    assert tracker.bin_id(0, 0) == 0
    assert tracker.bin_id(1, 0) == 7
    assert tracker.bin_id(0, 3) == 3
    assert tracker.bin_id(-1, 3) == -1
    assert tracker.bin_id(1, -1) == -1


def test_record_step_increments_occupancy_and_ignores_out_of_range():
    tracker = _tracker()
    v_idx, kappa_idx = tracker.record_step(1.2, 0.0)
    assert (v_idx, kappa_idx) == (1, 3)
    assert tracker.occupancy[1, 3] == 1

    tracker.record_step(1.2, 0.0)
    assert tracker.occupancy[1, 3] == 2

    out_v_idx, out_kappa_idx = tracker.record_step(-5.0, 0.0)
    assert (out_v_idx, out_kappa_idx) == (-1, 3)
    assert tracker.occupancy.sum() == 2  # unchanged


def test_next_target_figure8_cadence():
    tracker = _tracker(prioritize_corner_first=False, figure8_period=5, corner_probe_paths=0)
    profile_types = [tracker.next_target().profile_type for _ in range(5)]
    assert profile_types[4] == "figure8"
    assert all(p != "figure8" for p in profile_types[:4])


def test_next_target_corner_first():
    tracker = _tracker(prioritize_corner_first=True, corner_probe_paths=2, figure8_period=0)
    first = tracker.next_target()
    second = tracker.next_target()
    assert first.profile_type == "corner_probe"
    assert second.profile_type == "corner_probe"
    assert first.v_idx == tracker.n_v - 1
    assert first.kappa_idx == tracker.n_kappa - 1


def test_next_target_greedy_picks_lowest_occupancy():
    tracker = _tracker(prioritize_corner_first=False, figure8_period=0)
    # saturate every bin except (0, 0)
    tracker.occupancy[:, :] = tracker.target_revisits
    tracker.occupancy[0, 0] = 0

    target = tracker.next_target()
    assert (target.v_idx, target.kappa_idx) == (0, 0)


def test_next_target_diversifies_back_to_back_calls_with_no_execution():
    tracker = _tracker(prioritize_corner_first=False, figure8_period=0)
    targets = [tracker.next_target() for _ in range(tracker.n_v * tracker.n_kappa)]
    seen = {(t.v_idx, t.kappa_idx) for t in targets}
    # every bin should be requested exactly once before any repeats
    assert len(seen) == tracker.n_v * tracker.n_kappa


def test_next_target_profile_type_mapping():
    tracker = _tracker(prioritize_corner_first=False, figure8_period=0)

    # straight-ish bin (kappa_idx=3 -> center 0.0) -> speed_sweep
    cb = tracker._profile_for_bin(0, 3)
    assert cb.profile_type == "speed_sweep"

    # curvy but not the corner -> curvature_sweep
    cb = tracker._profile_for_bin(0, 0)
    assert cb.profile_type == "curvature_sweep"

    # highest v, highest |kappa| -> corner_probe
    cb = tracker._profile_for_bin(tracker.n_v - 1, tracker.n_kappa - 1)
    assert cb.profile_type == "corner_probe"
    cb = tracker._profile_for_bin(tracker.n_v - 1, 0)
    assert cb.profile_type == "corner_probe"


def test_report_structure():
    tracker = _tracker(prioritize_corner_first=False, figure8_period=0)
    tracker.record_step(1.2, 0.0)
    tracker.record_step(1.2, 0.0)
    tracker.record_step(1.2, 0.0)  # meets target_revisits=3 for bin (1,3)

    report = tracker.report()
    assert report["bins_total"] == tracker.n_v * tracker.n_kappa
    assert 0 <= report["bins_met"] <= report["bins_total"]
    assert report["pct_met"] == pytest.approx(100.0 * report["bins_met"] / report["bins_total"])

    bin_1_3 = next(b for b in report["bins"] if b["v_idx"] == 1 and b["kappa_idx"] == 3)
    assert bin_1_3["visits"] == 3
    assert bin_1_3["met"] is True
