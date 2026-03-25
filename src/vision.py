#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import math
from typing import Tuple
import cv2
import numpy as np
import pybullet as p


class CarVision:
    def get_car_camera_image(self, car_id: int, img_w: int = 320, img_h: int = 240):
        pos, orn = p.getBasePositionAndOrientation(car_id)
        yaw = p.getEulerFromQuaternion(orn)[2]

        cam_height = 1.2
        cam_forward = 1.0
        cam_pos = [
            pos[0] + cam_forward * math.cos(yaw),
            pos[1] + cam_forward * math.sin(yaw),
            cam_height,
        ]

        tgt = [
            pos[0] + 3.0 * math.cos(yaw),
            pos[1] + 3.0 * math.sin(yaw),
            0.05,
        ]

        up = [0, 0, 1]
        view = p.computeViewMatrix(cam_pos, tgt, up)
        proj = p.computeProjectionMatrixFOV(fov=65, aspect=img_w / img_h, nearVal=0.05, farVal=50.0)
        _, _, rgba, _, _ = p.getCameraImage(img_w, img_h, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        img = np.reshape(rgba, (img_h, img_w, 4))[:, :, :3].astype(np.uint8)
        return img

    def steering_from_yellow_line(self, img_bgr: np.ndarray) -> Tuple[float, np.ndarray]:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([15, 70, 70], dtype=np.uint8)
        upper = np.array([40, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        h, w = mask.shape
        roi = mask[int(h * 0.55):, :]
        roi_y0 = int(h * 0.55)
        roi = cv2.medianBlur(roi, 5)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)

        M = cv2.moments(roi)
        debug = img_bgr.copy()
        steering = 0.0

        if M["m00"] > 1e4:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"]) + roi_y0
            cv2.circle(debug, (cx, cy), 6, (0, 0, 255), -1)
            cv2.line(debug, (w // 2, h - 1), (cx, h - 1), (255, 0, 0), 2)
            err = (cx - (w / 2)) / (w / 2)
            steering = -0.7 * float(err)

        return steering, debug
