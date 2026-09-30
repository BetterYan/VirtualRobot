"""真值评估：SLAM 位姿 vs 真值位姿（ground_truth 通道，绝不进算法）。"""

from __future__ import annotations

import math

import numpy as np

from vrobot.sensors.models import GroundTruth


class PoseEvaluator:
    def __init__(self):
        self.pos_errors: list[float] = []
        self.th_errors: list[float] = []
        self.gt_path_len: float = 0.0
        self._last_gt: tuple[float, float] | None = None

    def update(self, est_pose: tuple[float, float, float], gt: GroundTruth) -> None:
        # 真值里程累积（评估漂移率的分母）
        if self._last_gt is not None:
            self.gt_path_len += math.hypot(gt.x - self._last_gt[0], gt.y - self._last_gt[1])
        self._last_gt = (gt.x, gt.y)
        self.pos_errors.append(math.hypot(est_pose[0] - gt.x, est_pose[1] - gt.y))
        dth = (est_pose[2] - gt.theta + math.pi) % (2 * math.pi) - math.pi
        self.th_errors.append(abs(dth))

    def stats(self) -> dict:
        if not self.pos_errors:
            return {"n": 0}
        pos = np.array(self.pos_errors)
        th = np.array(self.th_errors)
        out = {
            "n": len(pos),
            "pos_mean": float(pos.mean()),
            "pos_last": float(pos[-1]),
            "pos_max": float(pos.max()),
            "th_mean_deg": float(np.degrees(th.mean())),
            "th_last_deg": float(np.degrees(th[-1])),
        }
        # 相对漂移率：绝对误差会背着"初始坐标系对齐误差"走（SLAM 系由首帧里程计定义，
        # 匹配只保证自我一致，不保证回到真值系）；漂移率衡量"每走 100px 漂多少"，更公平。
        if self.gt_path_len > 1e-3:
            out["drift_per_100px"] = float(pos[-1] / self.gt_path_len * 100.0)
            out["gt_path_px"] = float(self.gt_path_len)
        return out
