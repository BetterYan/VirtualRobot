"""matplotlib 实时地图查看器：占据概率图 + 估计轨迹 + 真值对照。"""

from __future__ import annotations

import logging

import numpy as np

log = logging.getLogger(__name__)
from vrobot.viz.event_pump import pump as _pump_events


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
            if self._fig.canvas.manager:
                self._fig.canvas.manager.set_window_title("vrobot SLAM 实时地图")
                self._unset_topmost()
        except Exception as e:  # 无显示环境等情况
            log.warning("可视化不可用，已禁用: %s", e)
            self.enabled = False

    def _unset_topmost(self) -> None:
        """防御性关闭 topmost（部分后端/WM 会把交互式窗口置顶）。"""
        try:
            win = self._fig.canvas.manager.window
            if hasattr(win, "attributes"):  # Tk
                win.attributes("-topmost", False)
            if hasattr(win, "setWindowFlag"):  # Qt
                win.setWindowFlag(win.windowFlag() & ~0x00000001)  # 去 WindowStaysOnTopHint
        except Exception:
            pass

    def update(self, slam, gt_pose: tuple[float, float, float] | None = None) -> None:
        if not self.enabled:
            return
        try:
            prob = slam.grid.prob()
            if self._img is None:
                # 不加 animated=True：无 blit 循环时 TkAgg 常规重绘会跳过
                # animated artist —— 表现为只见轨迹不见地图（全白）
                self._img = self._ax.imshow(prob, cmap="gray_r", vmin=0, vmax=1)
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
            # 不用 plt.pause()：内部 manager.show() 会反复抬顶+抢焦点。
            # draw_idle 后必须主动泵 GUI 事件（asyncio 循环无 Tk mainloop，
            # 否则 Windows 判定窗口未响应并冻结重绘）。
            self._fig.canvas.draw_idle()
            _pump_events(self._fig)
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
