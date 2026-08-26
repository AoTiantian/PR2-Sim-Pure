"""Small, dependency-free SO(3) helpers for the comparison demo."""

from __future__ import annotations

import math

import numpy as np


def normalize_quaternion(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(q_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if q.shape != (4,) or norm < 1.0e-12:
        raise ValueError("quaternion must be a non-zero wxyz vector")
    return q / norm


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def quaternion_to_matrix(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize_quaternion(q_wxyz)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("rotation matrix must be 3x3")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = np.array(
            [0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale,
             (matrix[0, 2] - matrix[2, 0]) / scale,
             (matrix[1, 0] - matrix[0, 1]) / scale]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0)) * 2.0
            q = np.array([(matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale])
        elif index == 1:
            scale = math.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0)) * 2.0
            q = np.array([(matrix[0, 2] - matrix[2, 0]) / scale,
                          (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale])
        else:
            scale = math.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0)) * 2.0
            q = np.array([(matrix[1, 0] - matrix[0, 1]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale])
    q = normalize_quaternion(q)
    return -q if q[0] < 0.0 else q


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle < 1.0e-10:
        cross = skew(vector)
        return np.eye(3) + cross + 0.5 * cross @ cross
    axis_cross = skew(vector / angle)
    return np.eye(3) + math.sin(angle) * axis_cross + (1.0 - math.cos(angle)) * axis_cross @ axis_cross


def matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    cosine = float(np.clip(0.5 * (np.trace(matrix) - 1.0), -1.0, 1.0))
    angle = math.acos(cosine)
    vee = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if angle < 1.0e-8:
        return 0.5 * vee
    if math.pi - angle < 1.0e-5:
        eigenvalues, eigenvectors = np.linalg.eigh(0.5 * (matrix + np.eye(3)))
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        if float(axis @ vee) < 0.0:
            axis = -axis
        return angle * axis
    return 0.5 * angle / math.sin(angle) * vee


def orientation_error_world(current_q: np.ndarray, target_q: np.ndarray) -> np.ndarray:
    current = quaternion_to_matrix(current_q)
    target = quaternion_to_matrix(target_q)
    return matrix_to_rotvec(target @ current.T)


def so3_left_jacobian(rotvec: np.ndarray) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    cross = skew(vector)
    if angle < 1.0e-7:
        return np.eye(3) + 0.5 * cross + (1.0 / 6.0) * cross @ cross
    return (
        np.eye(3)
        + (1.0 - math.cos(angle)) / (angle * angle) * cross
        + (angle - math.sin(angle)) / (angle ** 3) * cross @ cross
    )

