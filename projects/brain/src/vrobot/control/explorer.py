"""自动探索：雷达避障走行（算法端自动驾驶，用于无人值守采集建图数据）。"""

from __future__ import annotations

import numpy as np

from vrobot.sensors.models import LaserScan2D


class AutoExplorer:
    """简单反应式控制：前方近障碍则转向开阔侧，否则直行。"""

    def __init__(
        self,
        linear: float = 55.0,
        angular: float = 1.6,
        front_dist: float = 60.0,
        beam_spread: int = 7,
    ):
        self.linear = float(linear)
        self.angular = float(angular)
        self.front_dist = float(front_dist)
        self.beam_spread = int(beam_spread) | 1  # 强制奇数

    def update(self, scan: LaserScan2D) -> tuple[float, float]:
        """返回 (linear, angular) 指令。"""
        n = len(scan.ranges)
        if n == 0:
            return (0.0, 0.0)
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        half = self.beam_spread // 2
        idx = [(center + i) % n for i in range(-half, half + 1)]
        front = [scan.ranges[i] for i in idx if scan.ranges[i] > 0]
        d_front = min(front) if front else scan.range_max

        if d_front >= self.front_dist:
            return (self.linear, 0.0)

        # 转向更开阔的一侧：比较左右半区平均距离
        left_idx = [(center + i) % n for i in range(half + 1, half + 1 + 8)]
        right_idx = [(center - i) % n for i in range(half + 1, half + 1 + 8)]
        left = [scan.ranges[i] for i in left_idx if scan.ranges[i] > 0]
        right = [scan.ranges[i] for i in right_idx if scan.ranges[i] > 0]
        d_left = np.mean(left) if left else 0.0
        d_right = np.mean(right) if right else 0.0
        turn = self.angular if d_left >= d_right else -self.angular
        return (10.0, turn)
