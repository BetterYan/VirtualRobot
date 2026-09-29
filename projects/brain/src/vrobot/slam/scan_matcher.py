"""相关性扫描匹配（Correlative Scan Matching, scan-to-map）。

在先验位姿附近的 (dx, dy, dtheta) 窗口内穷举候选位姿，
得分 = 扫描命中点在栅格概率图上的采样均值；两级搜索：粗定位 → 精修。

参考：E. Olson, "Real-Time Correlative Scan Matching"（2009）的简化实现。
"""

from __future__ import annotations

import numpy as np

from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid


class CorrelativeScanMatcher:
    def __init__(
        self,
        grid: OccupancyGrid,
        search_xy: float = 24.0,
        search_th: float = 0.20,
        coarse_step_xy: float = 4.0,
        coarse_step_th: float = 0.05,
        min_score: float = 0.30,
    ):
        self.grid = grid
        self.search_xy = float(search_xy)
        self.search_th = float(search_th)
        self.coarse_step_xy = float(coarse_step_xy)
        self.coarse_step_th = float(coarse_step_th)
        self.min_score = float(min_score)

    # ---- 打分 ----

    def score(self, scan: LaserScan2D, pose: tuple[float, float, float]) -> float:
        """给定位姿的匹配得分：命中点概率均值，空扫描记 0。"""
        pts = scan.local_endpoints()
        if pts.shape[0] == 0:
            return 0.0
        c, s = np.cos(pose[2]), np.sin(pose[2])
        world = pts @ np.array([[c, -s], [s, c]]).T + np.array([pose[0], pose[1]])
        probs = self.grid.prob_at_points(world)
        return float(probs.mean()) if probs.size else 0.0

    def _score_batch(self, local_pts: np.ndarray, candidates: np.ndarray) -> np.ndarray:
        """批量打分。candidates: (C,3) 即 (x, y, theta)；返回 (C,) 得分。

        local_pts: (N,2) 机器人坐标系下的命中点。
        """
        n_cand = candidates.shape[0]
        if n_cand == 0 or local_pts.shape[0] == 0:
            return np.zeros(max(n_cand, 0), dtype=np.float32)
        # 旋转: (C,2,2) @ (N,2)^T -> (C,N,2)
        c = np.cos(candidates[:, 2])[:, None]
        s = np.sin(candidates[:, 2])[:, None]
        rot = np.stack(
            [np.concatenate([c, -s], axis=1), np.concatenate([s, c], axis=1)], axis=1
        )  # (C,2,2)
        world = np.einsum("cij,nj->cni", rot, local_pts)  # (C,N,2)
        world[..., 0] += candidates[:, 0][:, None]
        world[..., 1] += candidates[:, 1][:, None]
        probs = self.grid.prob_at_points(world.reshape(-1, 2)).reshape(n_cand, -1)
        return probs.mean(axis=1)

    # ---- 匹配 ----

    def match(
        self, scan: LaserScan2D, pose_guess: tuple[float, float, float]
    ) -> tuple[tuple[float, float, float], float]:
        """在 pose_guess 附近两级搜索最优位姿。返回 ((x,y,theta), score)。"""
        local = scan.local_endpoints()
        if local.shape[0] < 5:
            return pose_guess, 0.0

        # --- 粗搜索 ---
        nx = max(1, int(round(self.search_xy / self.coarse_step_xy)))
        nth = max(1, int(round(self.search_th / self.coarse_step_th)))
        dxs = np.arange(-nx, nx + 1) * self.coarse_step_xy
        dys = np.arange(-nx, nx + 1) * self.coarse_step_xy
        dts = np.arange(-nth, nth + 1) * self.coarse_step_th
        cand = self._make_candidates(pose_guess, dxs, dys, dts)
        scores = self._score_batch(local, cand)
        best = cand[int(np.argmax(scores))]
        best_score = float(scores.max())

        # --- 精修（粗步长窗口内按 1/4 步长细化，双线性插值下亚格精度有效）---
        fx = self.coarse_step_xy
        fth = self.coarse_step_th
        steps_f = np.array([-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0])
        cand2 = self._make_candidates(
            tuple(best),
            steps_f * fx,
            steps_f * fx,
            steps_f * fth,
        )
        scores2 = self._score_batch(local, cand2)
        i2 = int(np.argmax(scores2))
        if scores2[i2] >= best_score:
            best = cand2[i2]
            best_score = float(scores2[i2])

        return (float(best[0]), float(best[1]), float(best[2])), best_score

    def match_and_accept(
        self, scan: LaserScan2D, pose_guess: tuple[float, float, float]
    ) -> tuple[tuple[float, float, float], float, bool]:
        """match + 质量门限：得分低于 min_score 时拒绝（用预测位姿兜底）。"""
        pose, score = self.match(scan, pose_guess)
        return pose, score, score >= self.min_score

    # ---- helpers ----

    @staticmethod
    def _make_candidates(
        base: tuple[float, float, float],
        dxs: np.ndarray,
        dys: np.ndarray,
        dts: np.ndarray,
    ) -> np.ndarray:
        DX, DY, DT = np.meshgrid(dxs, dys, dts, indexing="ij")
        cand = np.stack(
            [base[0] + DX.ravel(), base[1] + DY.ravel(), base[2] + DT.ravel()], axis=1
        )
        return cand.astype(np.float32)
