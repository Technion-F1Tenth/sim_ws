import numpy as np
import pytest

from mrs_roboracer_inference.inference_node import resample_scan


def test_resample_identity_when_layout_matches():
    """Same beam count and FOV as the target layout should pass through unshuffled."""
    n_beams = 64
    fov_rad = np.deg2rad(270.0)
    max_range = 30.0
    angle_min = -fov_rad / 2.0
    angle_increment = fov_rad / (n_beams - 1)

    ranges = np.linspace(1.0, 20.0, n_beams)
    out = resample_scan(
        ranges, angle_min, angle_increment, max_range, n_beams, fov_rad, max_range)

    np.testing.assert_allclose(out, ranges / max_range, atol=1e-6)


def test_resample_clamps_nan_and_inf_to_far_reading():
    n_beams = 8
    fov_rad = np.deg2rad(270.0)
    max_range = 30.0
    angle_min = -fov_rad / 2.0
    angle_increment = fov_rad / (n_beams - 1)

    ranges = np.array(
        [1.0, float('nan'), float('inf'), -float('inf'), 5.0, 5.0, 5.0, 5.0])
    out = resample_scan(
        ranges, angle_min, angle_increment, max_range, n_beams, fov_rad, max_range)

    assert np.all(np.isfinite(out))
    assert np.all(out >= 0.0) and np.all(out <= 1.0)
    # the nan/inf slots should have become "far" (clamped to max_range -> 1.0)
    assert out[1] == pytest.approx(1.0)
    assert out[2] == pytest.approx(1.0)
    assert out[3] == pytest.approx(1.0)


def test_resample_clamps_out_of_range_readings_to_unit_interval():
    n_beams = 4
    fov_rad = np.deg2rad(270.0)
    max_range = 30.0
    angle_min = -fov_rad / 2.0
    angle_increment = fov_rad / (n_beams - 1)

    ranges = np.array([-5.0, 0.0, 100.0, 15.0])
    out = resample_scan(
        ranges, angle_min, angle_increment, max_range, n_beams, fov_rad, max_range)

    assert np.all(out >= 0.0) and np.all(out <= 1.0)


def test_resample_downsamples_a_denser_scan_to_n_beams():
    """A real/sim LIDAR (e.g. 819 beams) must resample down to n_lidar_beams."""
    n_in = 819
    n_beams = 64
    fov_rad = np.deg2rad(270.0)
    max_range = 30.0
    angle_min = -fov_rad / 2.0
    angle_increment = fov_rad / (n_in - 1)

    ranges = np.full(n_in, 10.0)
    out = resample_scan(
        ranges, angle_min, angle_increment, max_range, n_beams, fov_rad, max_range)

    assert out.shape == (n_beams,)
    np.testing.assert_allclose(out, 10.0 / max_range, atol=1e-6)
