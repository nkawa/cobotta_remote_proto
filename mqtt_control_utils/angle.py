"""角度変換のためのモジュール。"""
import numpy as np


def norm_angle_360(degree: float) -> float:
    return degree % 360

def norm_angle_180(degree: float) -> float:
    return (degree + 180) % 360 - 180

def norm_pose_360(pose: np.ndarray) -> np.ndarray:
    """
    pose: 姿勢 ([x, y, z, rx, ry, rz])。角度の単位はdeg。
    """
    pose = pose.copy()
    pose[3] = norm_angle_360(pose[3])
    pose[4] = norm_angle_360(pose[4])
    pose[5] = norm_angle_360(pose[5])
    return pose

def norm_pose_180(pose: np.ndarray) -> np.ndarray:
    """
    pose: 姿勢 ([x, y, z, rx, ry, rz])。角度の単位はdeg。
    """
    pose = pose.copy()
    pose[3] = norm_angle_180(pose[3])
    pose[4] = norm_angle_180(pose[4])
    pose[5] = norm_angle_180(pose[5])
    return pose
