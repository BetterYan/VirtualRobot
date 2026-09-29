"""matplotlib 实时地图查看器：占据概率图 + 估计轨迹 + 真值对照。"""

from __future__ import annotations

import logging

import numpy as np

log = logging.getLogger(__name__)


class MapViewer:
    def __init__(self, enabled: bool = True, cell_size: float = 8.0):
        self.enabled = enabled
        self.cell_size = cell_size
        self._fig = None
        self._ax = None
        self._img = None
        self._traj_line = None
        self._gt_marker = None
        self._robot_marker = None
        self._frame = 0
        if not enabled:
            return
        try:
            import matplotlib.pyplot as plt

            self._plt = plt
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(9, 7))
            self._fig.canvas.manager.set_window_title("vrobot SLAM 实时地图") if hasattr(
                self._fig, "canvas"
            ) and self._fig.canvas.manager else None
        except Exception as e:  # 无显示环境等情况
            log.warning("可视化不可用，已禁用: %s", e)
            self.enabled = False

    def update(self, slam, gt_pose: tuple[float, float, float] | None = None) -> None:
        if not self.enabled:
            return
        try:
            prob = slam.grid.prob()
            if self._img is None:
                self._img = self._ax.imshow(prob, cmap="gray_r", vmin=0, vmax=1, animated=True)
                (self._traj_line,) = self._ax.plot([], [], "r-", lw=1.2, label="est traj")
                (self._robot_marker,) = self._ax.plot([], [], "r+", ms=12)
                (self._gt_marker,) = self._ax.plot([], [], "g+", ms=12, label="ground truth")
                self._ax.legend(loc="lower right", fontsize=8)
            else:
                self._img.set_data(prob)
            traj = np.array(slam.trajectory) / self.cell_size - 0.5
            if len(traj):
                self._traj_line.set_data(traj[:, 0], traj[:, 1])
                self._robot_marker.set_data([traj[-1, 0]], [traj[-1, 1]])
            if gt_pose is not None:
                self._gt_marker.set_data([gt_pose[0] / self.cell_size - 0.5], [gt_pose[1] / self.cell_size - 0.5])
            self._ax.set_title(f"SLAM map  frame={self._frame}")
            self._plt.pause(0.001)
            self._frame += 1
        except Exception as e:
            log.debug("viewer update failed: %s", e)

    def close(self) -> None:
        if self.enabled and self._fig is not None:
            try:
                self._plt.ioff()
                self._plt.close(self._fig)
            except Exception:
                pass
