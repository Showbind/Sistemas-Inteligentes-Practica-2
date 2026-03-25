#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import pybullet as p
import pybullet_data


class Simulation:
    def __init__(self, gui: bool):
        self.gui = gui

    def connect(self):
        p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        p.loadURDF("plane.urdf")

    def set_topdown_camera(self, plane_size: float):
        dist = plane_size * 0.55
        p.resetDebugVisualizerCamera(dist, 0.0, -89.0, [0, 0, 0])

    def add_goal_marker(self, x: float, y: float):
        p.addUserDebugText("GOAL", [x, y, 0.2], [1, 0, 0], 1.6, 0)

    def add_info_text(self, text: str, x: float, y: float, color=(1, 1, 0), size: float = 1.4, life_time: float = 0):
        p.addUserDebugText(text, [x, y, 0.8], list(color), size, life_time)

    def step(self):
        p.stepSimulation()

    def is_connected(self) -> bool:
        return bool(p.isConnected())
