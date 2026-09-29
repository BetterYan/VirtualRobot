"""对数概率占据栅格地图（Occupancy Grid Map）。

log-odds 更新：l += l_hit(命中格) / l_miss(射线途经格)，clamp 到 [l_min, l_max]。
世界坐标为 Godot 像素；栅格行号沿 y 向下增长，与传感器坐标系一致。
"""

from __future__ import annotations

import numpy as np


class OccupancyGrid:
    def __init__(
        self,
        cell_size: float = 8.0,
        width: int = 200,
        height: int = 160,
        l_hit: float = 0.9,
        l_miss: float = -0.35,
        l_min: float = -4.0,
        l_max: float = 4.0,
    ):
        self.cell_size = float(cell_size)
        self.width = int(width)
        self.height = int(height)
        self.l_hit = float(l_hit)
        self.l_miss = float(l_miss)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self._logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self._n_updates = 0

    # ---- 坐标变换 ----

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """世界像素 -> (col, row)。"""
        return int(x // self.cell_size), int(y // self.cell_size)

    def cell_center(self, col: int, row: int) -> tuple[float, float]:
        return (col + 0.5) * self.cell_size, (row + 0.5) * self.cell_size

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    # ---- 更新 ----

    def update(self, pose: tuple[float, float, float], points: np.ndarray) -> None:
        """用一帧扫描更新地图。

        pose: (x, y, theta) 机器人世界位姿
        points: (N,2) 命中点世界坐标（scan_world_points 的输出）
        """
        self._n_updates += 1
        rc = self.world_to_cell(pose[0], pose[1])
        for p in points:
            pc = self.world_to_cell(p[0], p[1])
            self._update_ray(rc, pc)

    def _update_ray(self, start: tuple[int, int], end: tuple[int, int]) -> None:
        """Bresenham 射线：途经格 l_miss，终点格 l_hit。越界格安全忽略。"""
        c0, r0 = start
        c1, r1 = end
        dc, dr = abs(c1 - c0), abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr
        c, r = c0, r0
        while True:
            if (c, r) == (c1, r1):
                self._apply(c, r, self.l_hit)
                return
            self._apply(c, r, self.l_miss)
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
            if not (-1 <= c <= self.width and -1 <= r <= self.height):
                return  # 射线越出栅格，提前结束
        # unreachable

    def _apply(self, col: int, row: int, delta: float) -> None:
        if 0 <= col < self.width and 0 <= row < self.height:
            v = self._logodds[row, col] + delta
            self._logodds[row, col] = min(self.l_max, max(self.l_min, v))

    # ---- 读取 ----

    def prob(self) -> np.ndarray:
        """占据概率图 (H, W)，sigmoid(log-odds)。"""
        return 1.0 - 1.0 / (1.0 + np.exp(self._logodds))

    def prob_at_points(self, points: np.ndarray) -> np.ndarray:
        """双线性插值查询世界坐标点的占据概率，出界为 0。points: (N,2)。

        双线性消除"格内平台"，使扫描匹配获得亚格（sub-cell）分辨率。
        """
        n = points.shape[0]
        if n == 0:
            return np.zeros(0, dtype=np.float32)
        p = self.prob()
        # 以格中心为锚点的连续坐标
        fx = points[:, 0] / self.cell_size - 0.5
        fy = points[:, 1] / self.cell_size - 0.5
        x0 = np.floor(fx).astype(np.int64)
        y0 = np.floor(fy).astype(np.int64)
        dx = (fx - x0).astype(np.float32)
        dy = (fy - y0).astype(np.float32)

        def gather(cx, cy):
            ok = (cx >= 0) & (cx < self.width) & (cy >= 0) & (cy < self.height)
            out = np.zeros(n, dtype=np.float32)
            out[ok] = p[cy[ok], cx[ok]]
            return out, ok

        p00, ok00 = gather(x0, y0)
        p10, ok10 = gather(x0 + 1, y0)
        p01, ok01 = gather(x0, y0 + 1)
        p11, ok11 = gather(x0 + 1, y0 + 1)
        # 任一邻域出界时，仅用界内角点近似（边界平滑退化）
        w00 = ((1 - dx) * (1 - dy)) * ok00
        w10 = (dx * (1 - dy)) * ok10
        w01 = ((1 - dx) * dy) * ok01
        w11 = (dx * dy) * ok11
        wsum = w00 + w10 + w01 + w11
        out = (p00 * w00 + p10 * w10 + p01 * w01 + p11 * w11)
        return np.where(wsum > 1e-6, out / np.maximum(wsum, 1e-6), 0.0).astype(np.float32)

    def obstacles(self, thresh: float = 0.65) -> np.ndarray:
        """占据格布尔图 (H, W)。严格大于：未观测格（l=0）不算占据。"""
        return self._logodds > np.log(thresh / (1.0 - thresh))

    @property
    def n_updates(self) -> int:
        return self._n_updates
