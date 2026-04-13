#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import numpy as np
import pybullet as p

GridPos = Tuple[int, int]


def create_static_box(pos_xyz, half_extents_xyz, rgba=(1, 1, 1, 1), yaw=0.0, z_lift=0.0) -> int:
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents_xyz)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents_xyz, rgbaColor=rgba)
    orn = p.getQuaternionFromEuler([0, 0, yaw])
    x, y, z = pos_xyz
    return p.createMultiBody(0.0, col, vis, [x, y, z + z_lift], orn)


@dataclass
class MapParams:
    plane_size: float = 40.0
    road_half_width: float = 5.5
    sidewalk_width: float = 2.0
    intersection_half: float = 7.5
    border_road_half_width: float = 4.5
    border_inset: float = 1.0
    cell: float = 0.25
    road_color: Tuple[float, float, float, float] = (0.20, 0.20, 0.22, 1.0)
    sidewalk_color: Tuple[float, float, float, float] = (0.75, 0.75, 0.76, 1.0)
    center_line_color: Tuple[float, float, float, float] = (0.95, 0.80, 0.10, 1.0)
    lane_line_color: Tuple[float, float, float, float] = (0.90, 0.90, 0.90, 1.0)


class CityMap:
    def __init__(self, params: MapParams):
        self.params = params
        self.road_rects: List[Tuple[float, float, float, float]] = []
        self.grid = self._make_grid()

    @property
    def half(self) -> float:
        return self.params.plane_size / 2.0

    def _make_grid(self) -> np.ndarray:
        size = self.params.plane_size
        h = int(math.ceil(size / self.params.cell))
        w = int(math.ceil(size / self.params.cell))
        return np.ones((h, w), dtype=np.int16)

    def world_to_grid(self, x: float, y: float) -> GridPos:
        relx = x + self.half
        rely = y + self.half
        c = int(relx / self.params.cell)
        r = int(rely / self.params.cell)
        r = int(np.clip(r, 0, self.grid.shape[0] - 1))
        c = int(np.clip(c, 0, self.grid.shape[1] - 1))
        return (r, c)

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = -self.half + (c + 0.5) * self.params.cell
        y = -self.half + (r + 0.5) * self.params.cell
        return x, y

    def add_road_rect(self, xmin: float, xmax: float, ymin: float, ymax: float):
        self.road_rects.append((xmin, xmax, ymin, ymax))

    def bake_grid_roads(self):
        self.grid[:] = -1
        for (xmin, xmax, ymin, ymax) in self.road_rects:
            (r0, c0) = self.world_to_grid(xmin, ymin)
            (r1, c1) = self.world_to_grid(xmax, ymax)
            rr0, rr1 = sorted([r0, r1])
            cc0, cc1 = sorted([c0, c1])
            self.grid[rr0:rr1 + 1, cc0:cc1 + 1] = 1

    def nearest_free(self, x: float, y: float, max_rad_cells: int = 80) -> Optional[GridPos]:
        start = self.world_to_grid(x, y)
        if self.grid[start] == 0:
            return start
        sr, sc = start
        h, w = self.grid.shape
        for rad in range(1, max_rad_cells + 1):
            for dr in range(-rad, rad + 1):
                for dc in (-rad, rad):
                    r, c = sr + dr, sc + dc
                    if 0 <= r < h and 0 <= c < w and self.grid[r, c] == 0:
                        return (r, c)
            for dc in range(-rad, rad + 1):
                for dr in (-rad, rad):
                    r, c = sr + dr, sc + dc
                    if 0 <= r < h and 0 <= c < w and self.grid[r, c] == 0:
                        return (r, c)
        return None
    
    def draw_path(self, x_start, y_start, x_end, y_end, color: tuple[int, int, int] = [1,0,0]):
        p.addUserDebugLine(
            lineFromXYZ=[x_start, y_start, 0.05],
            lineToXYZ=[x_end, y_end, 0.05],
            lineColorRGB=color,
            lineWidth=3
        )

    def build_world(self):
        pz = self.params
        L = self.half
        z = 0.0

        create_static_box((0, 0, z - 0.02), (L, L, 0.02), rgba=(0.30, 0.55, 0.30, 1.0))

        rw = pz.road_half_width
        create_static_box((0, 0, z + 0.001), (L, rw, 0.01), rgba=pz.road_color)
        create_static_box((0, 0, z + 0.001), (rw, L, 0.01), rgba=pz.road_color)
        self.add_road_rect(-L, +L, -rw, +rw)
        self.add_road_rect(-rw, +rw, -L, +L)

        bw = pz.border_road_half_width
        inset = pz.border_inset
        y_top = +(L - inset - bw)
        y_bot = -(L - inset - bw)
        x_left = -(L - inset - bw)
        x_right = +(L - inset - bw)

        create_static_box((0, y_top, z + 0.001), (L - inset, bw, 0.01), rgba=pz.road_color)
        create_static_box((0, y_bot, z + 0.001), (L - inset, bw, 0.01), rgba=pz.road_color)
        create_static_box((x_left, 0, z + 0.001), (bw, L - inset, 0.01), rgba=pz.road_color)
        create_static_box((x_right, 0, z + 0.001), (bw, L - inset, 0.01), rgba=pz.road_color)

        self.add_road_rect(-(L - inset), +(L - inset), y_top - bw, y_top + bw)
        self.add_road_rect(-(L - inset), +(L - inset), y_bot - bw, y_bot + bw)
        self.add_road_rect(x_left - bw, x_left + bw, -(L - inset), +(L - inset))
        self.add_road_rect(x_right - bw, x_right + bw, -(L - inset), +(L - inset))

        create_static_box((0, 0, z + 0.004), (L, 0.08, 0.012), rgba=pz.center_line_color)
        create_static_box((0, 0, z + 0.004), (0.08, L, 0.012), rgba=pz.center_line_color)

        seg, gap = 1.2, 0.9
        stripe_t = 0.06
        offset = rw * 0.45

        x = -L
        while x < L:
            create_static_box((x + seg / 2, +offset, z + 0.004), (seg / 2, stripe_t, 0.012), rgba=pz.lane_line_color)
            create_static_box((x + seg / 2, -offset, z + 0.004), (seg / 2, stripe_t, 0.012), rgba=pz.lane_line_color)
            x += seg + gap

        y = -L
        while y < L:
            create_static_box((+offset, y + seg / 2, z + 0.004), (stripe_t, seg / 2, 0.012), rgba=pz.lane_line_color)
            create_static_box((-offset, y + seg / 2, z + 0.004), (stripe_t, seg / 2, 0.012), rgba=pz.lane_line_color)
            y += seg + gap

        self.bake_grid_roads()
