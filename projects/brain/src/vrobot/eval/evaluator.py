"""真值评估：SLAM 位姿 vs 真值位姿（ground_truth 通道，绝不进算法）。"""

from __future__ import annotations

import math

import numpy as np

from vrobot.sensors.models import GroundTruth


class PoseEvaluator:
    def __init__(self):
        self.pos_errors: list[float] = []
        self.th_errors: list[float] = []

    def update(self, est_pose: tuple[float, float, float], gt: GroundTruth) -> None:
        self.pos_errors.append(math.hypot(est_pose[0] - gt.x, est_pose[1] - gt.y))
        dth = (est_pose[2] - gt.theta + math.pi) % (2 * math.pi) - math.pi
        self.th_errors.append(abs(dth))

    def stats(self) -> dict:
        if not self.pos_errors:
            return {"n": 0}
        pos = np.array(self.pos_errors)
        th = np.array(self.th_errors)
        return {
            "n": len(pos),
            "pos_mean": float(pos.mean()),
            "pos_last": float(pos[-1]),
            "pos_max": float(pos.max()),
            "th_mean_deg": float(np.degrees(th.mean())),
            "th_last_deg": float(np.degrees(th[-1])),
        }
