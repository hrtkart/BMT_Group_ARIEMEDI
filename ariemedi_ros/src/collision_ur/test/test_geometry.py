import numpy as np

from collision_ur.geometry import (
    closest_points_on_segments,
    interpolate_transform,
)


def test_crossing_segments_have_zero_distance():
    result = closest_points_on_segments(
        np.array([-1.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
    )
    assert result.distance == 0.0
    np.testing.assert_allclose(result.point_a, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(result.point_b, [0.0, 0.0, 0.0])


def test_parallel_segments_return_gap():
    result = closest_points_on_segments(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.2, 0.0, 0.0]),
        np.array([0.2, 0.0, 1.0]),
    )
    assert abs(result.distance - 0.2) < 1.0e-12


def test_degenerate_segment_is_supported():
    result = closest_points_on_segments(
        np.array([0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
    )
    assert abs(result.distance - 1.0) < 1.0e-12


def test_transform_interpolation_translation():
    start = np.eye(4)
    end = np.eye(4)
    end[:3, 3] = [1.0, 2.0, 3.0]
    middle = interpolate_transform(start, end, 0.5)
    np.testing.assert_allclose(middle[:3, 3], [0.5, 1.0, 1.5])
