from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


EPS = 1.0e-12


@dataclass
class SegmentDistance:
    distance: float
    point_a: np.ndarray
    point_b: np.ndarray


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = float(np.dot(quaternion, quaternion))
    if norm < EPS:
        return np.eye(3, dtype=np.float64)
    quaternion *= np.sqrt(2.0 / norm)
    quaternion = np.outer(quaternion, quaternion)
    return np.array(
        [
            [
                1.0 - quaternion[1, 1] - quaternion[2, 2],
                quaternion[0, 1] - quaternion[2, 3],
                quaternion[0, 2] + quaternion[1, 3],
            ],
            [
                quaternion[0, 1] + quaternion[2, 3],
                1.0 - quaternion[0, 0] - quaternion[2, 2],
                quaternion[1, 2] - quaternion[0, 3],
            ],
            [
                quaternion[0, 2] - quaternion[1, 3],
                quaternion[1, 2] + quaternion[0, 3],
                1.0 - quaternion[0, 0] - quaternion[1, 1],
            ],
        ],
        dtype=np.float64,
    )


def matrix_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm > EPS else np.array([0.0, 0.0, 0.0, 1.0])


def pose_matrix(position, orientation) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_matrix(
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    transform[:3, 3] = [position.x, position.y, position.z]
    return transform


def transform_point(transform: np.ndarray, point: np.ndarray) -> np.ndarray:
    return transform[:3, :3] @ point + transform[:3, 3]


def transform_segment(
    tcp_transform: np.ndarray, start_tcp: np.ndarray, end_tcp: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    return (
        transform_point(tcp_transform, start_tcp),
        transform_point(tcp_transform, end_tcp),
    )


def interpolate_transform(
    start: np.ndarray, end: np.ndarray, fraction: float
) -> np.ndarray:
    fraction = float(np.clip(fraction, 0.0, 1.0))
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = (1.0 - fraction) * start[:3, 3] + fraction * end[:3, 3]
    quaternion_start = matrix_quaternion(start[:3, :3])
    quaternion_end = matrix_quaternion(end[:3, :3])
    dot_product = float(np.dot(quaternion_start, quaternion_end))
    if dot_product < 0.0:
        quaternion_end = -quaternion_end
        dot_product = -dot_product
    if dot_product > 0.9995:
        quaternion = quaternion_start + fraction * (
            quaternion_end - quaternion_start
        )
        quaternion /= np.linalg.norm(quaternion)
    else:
        angle = float(np.arccos(np.clip(dot_product, -1.0, 1.0)))
        sine = np.sin(angle)
        quaternion = (
            np.sin((1.0 - fraction) * angle) / sine * quaternion_start
            + np.sin(fraction * angle) / sine * quaternion_end
        )
    result[:3, :3] = quaternion_matrix(*quaternion)
    return result


def closest_points_on_segments(
    start_a: np.ndarray,
    end_a: np.ndarray,
    start_b: np.ndarray,
    end_b: np.ndarray,
) -> SegmentDistance:
    """Return the closest points between two finite 3-D line segments."""
    direction_a = end_a - start_a
    direction_b = end_b - start_b
    offset = start_a - start_b
    length_a_sq = float(np.dot(direction_a, direction_a))
    length_b_sq = float(np.dot(direction_b, direction_b))
    direction_dot = float(np.dot(direction_a, direction_b))
    a_offset_dot = float(np.dot(direction_a, offset))
    b_offset_dot = float(np.dot(direction_b, offset))

    if length_a_sq <= EPS and length_b_sq <= EPS:
        point_a = start_a.copy()
        point_b = start_b.copy()
    elif length_a_sq <= EPS:
        parameter_b = np.clip(b_offset_dot / length_b_sq, 0.0, 1.0)
        point_a = start_a.copy()
        point_b = start_b + parameter_b * direction_b
    elif length_b_sq <= EPS:
        parameter_a = np.clip(-a_offset_dot / length_a_sq, 0.0, 1.0)
        point_a = start_a + parameter_a * direction_a
        point_b = start_b.copy()
    else:
        denominator = length_a_sq * length_b_sq - direction_dot * direction_dot
        if abs(denominator) > EPS:
            parameter_a = np.clip(
                (direction_dot * b_offset_dot - a_offset_dot * length_b_sq)
                / denominator,
                0.0,
                1.0,
            )
        else:
            parameter_a = 0.0

        parameter_b = (direction_dot * parameter_a + b_offset_dot) / length_b_sq
        if parameter_b < 0.0:
            parameter_b = 0.0
            parameter_a = np.clip(-a_offset_dot / length_a_sq, 0.0, 1.0)
        elif parameter_b > 1.0:
            parameter_b = 1.0
            parameter_a = np.clip(
                (direction_dot - a_offset_dot) / length_a_sq, 0.0, 1.0
            )

        point_a = start_a + parameter_a * direction_a
        point_b = start_b + parameter_b * direction_b

    return SegmentDistance(
        distance=float(np.linalg.norm(point_a - point_b)),
        point_a=point_a,
        point_b=point_b,
    )


def clamp_position(position: np.ndarray, minimum: np.ndarray, maximum: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(position, minimum), maximum)
