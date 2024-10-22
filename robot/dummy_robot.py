import logging
import time
from typing import List

import numpy as np

from .robot import Robot


logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())


class DummyRobot(Robot):
    """ダミーのロボットクラス。"""
    def __init__(
        self,
        name: str = "dummy",
        servo_control_mode: str = "diff",
        command_time: float = 0,
    ):
        super().__init__(name)
        self.defPose = [0, 0, 0, 0, 0, 0]
        self.current_pose = [1, 1, 1, 1, 1, 1]
        self.servo_control_mode = servo_control_mode
        self.command_time = command_time
        self.servo_mode = False

    def start(self):
        logger.info("start")

    def enable(self):
        logger.info("enable")

    def set_default_pose(self, pose):
        logger.info("set_default_pose")
        self.defPose = pose

    def get_default_pose(self):
        return self.defPose

    def move_default_pose(self):
        logger.info("move_default_pose")
        self.current_pose = self.defPose

    def move_pose(self, pose: List[float]) -> None:
        logger.info("move_pose")
        self.current_pose = pose

    def get_current_pose(self):
        logger.debug("get_current_pose")
        return self.current_pose

    def get_current_pose_rt(self):
        return [time.time()] + self.current_pose

    def enter_servo_mode(self, robot_interval: float = 0.001):
        logger.info("enter_servo_mode")
        self.robot_interval = robot_interval
        self.servo_mode = True

    def move_pose_servo(self, pose):
        logger.debug("move_pose_servo")
        if self.servo_control_mode == "diff":
            self.move_pose_servo_by_diff(pose)
        elif self.servo_control_mode == "abs":
            self.move_pose_servo_by_pos(pose)
        elif self.servo_control_mode == "vel":
            self.move_pose_servo_by_vel(pose)
        else:
            raise NotImplementedError
        time.sleep(self.command_time)
        return time.time(), self.current_pose

    def move_pose_servo_by_pos(self, pose):
        self.current_pose = pose

    def move_pose_servo_by_diff(self, pose):
        self.current_pose = (np.asarray(self.current_pose) + np.asarray(pose)).tolist()

    def move_pose_servo_by_vel(self, vel):
        diff = np.asarray(vel) * self.robot_interval
        self.current_pose = (np.asarray(self.current_pose) + np.asarray(diff)).tolist()

    def leave_servo_mode(self):
        logger.info("leave_servo_mode")
        self.servo_mode = False

    def disable(self):
        logger.info("disable")

    def stop(self):
        logger.info("stop")

    def get_suggested_servo_interval(self):
        return 0.008

    def log_error(self, e: Exception):
        logger.error(e)

    def try_restart(self, e: Exception) -> bool:
        restarted = True
        return restarted

    def stop_move_pose_servo(self, pose):
        pass
    
    def is_in_servomove(self):
        return self.servo_mode

    def get_default_joint(self):
        return self.defPose

    def move_joint(self, joints):
        self.current_pose = joints

    def get_current_joint(self):
        return self.current_pose
