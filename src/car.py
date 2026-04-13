#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import math
import numpy as np
import pybullet as p


class RaceCar:
    STEER_JOINTS = [4, 6]
    DRIVE_JOINTS = [2, 3, 5, 7]

    def __init__(self):
        self.car_id: int | None = None

    def spawn(self, x: float, y: float, yaw: float = 0.0) -> int:
        self.car_id = p.loadURDF(
            "racecar/racecar.urdf",
            [x, y, 0.2],
            p.getQuaternionFromEuler([0, 0, yaw])
        )
        for j in range(p.getNumJoints(self.car_id)):
            p.changeDynamics(self.car_id, j, lateralFriction=1.2, rollingFriction=0.02, spinningFriction=0.02)
        p.changeDynamics(self.car_id, -1, lateralFriction=1.2, rollingFriction=0.02, spinningFriction=0.02)
        return self.car_id

    def set_control(self, steering: float, throttle: float):
        if self.car_id is None:
            return
        max_steer = 0.6
        steering = float(np.clip(steering, -max_steer, +max_steer))
        for j in self.STEER_JOINTS:
            p.setJointMotorControl2(self.car_id, j, p.POSITION_CONTROL, targetPosition=steering, force=200)
        for j in self.DRIVE_JOINTS:
            p.setJointMotorControl2(self.car_id, j, p.VELOCITY_CONTROL, targetVelocity=throttle, force=80)

    def stop(self):
        self.set_control(0.0, 0.0)

    def get_pose(self):
        if self.car_id is None:
            return None
        pos, orn = p.getBasePositionAndOrientation(self.car_id)
        yaw = p.getEulerFromQuaternion(orn)[2]
        return pos, orn, yaw
