"""
ロボットのひな型モジュール。
ロボットレベルでの共通インタフェース化検討の例であるが、
ロボットレベルでの共通化は実際には難しいため、
現状、サブクラスは異なるメソッド、引数を持つことがある。
"""
from abc import ABC, abstractmethod
import logging
import time
from typing import List, Optional

import numpy as np

logger = logging.Logger(__name__)
logger.addHandler(logging.NullHandler())

class Robot(ABC):
    """
    ロボットのひな型クラス。
    """
    def __init__(self, name="Robot"):
        self.name = name

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def enable(self):
        pass

    @abstractmethod
    def set_default_pose(self, pose):
        pass

    @abstractmethod
    def get_default_pose(self):
        pass

    @abstractmethod
    def move_default_pose(self):
        pass

    @abstractmethod
    def move_pose(self, pose: List[float]) -> None:
        pass

    def move_default_pose_until_completion(
        self,
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        pose = self.get_default_pose()
        self.move_pose(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_pose()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                logger.warning("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_default_joint_until_completion(
        self,
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> bool:
        pose = self.get_default_joint()
        ret = self.move_joint(pose)
        
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_joint()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                logger.warning("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    def move_pose_until_completion(
        self,
        pose: List[float],
        precisions: Optional[List[float]] = None,
        check_interval: float = 1,
        timeout: float = 60,
    ) -> None:
        self.move_pose(pose)
        if precisions is None:
            precisions = [1, 1, 1, 1, 1, 1]
        precisions = np.asarray(precisions)
        t_start = time.time()
        while True:
            current_pose = self.get_current_pose()
            diff = np.abs(np.asarray(current_pose) - np.asarray(pose))
            if np.all(diff < precisions):
                done = True
                break
            time.sleep(check_interval)
            if time.time() - t_start > timeout:
                logger.info("Timeout before reaching destination.")
                done = False
                break
        # 位置が十分近くなった後念のため少し待つ
        time.sleep(1)
        return done

    @abstractmethod
    def get_current_pose(self) -> List[float]:
        pass

    @abstractmethod
    def enter_servo_mode(self):
        pass

    @abstractmethod
    def move_pose_servo(self, pose):
        pass

    @abstractmethod
    def leave_servo_mode(self):
        pass

    @abstractmethod
    def disable(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def get_suggested_servo_interval(self):
        pass
