"""调试窗口：实时对比 真值 / 里程计 / SLAM 位姿 与误差曲线。

四个子图（2x2）：
  1. 轨迹对照（世界坐标 px）：GT 绿 / 里程计 蓝 / SLAM 红
  2. 位置误差随帧数变化：|odom-GT|、|slam-GT|
  3. 航向误差随帧数变化（deg，wrap 到 [-180,180]）
  4. 扫描匹配得分与接受状态：score 曲线 + min_score 门限线 + 接受/拒绝着色

只依赖传入数据，不感知通信 / SLAM 内部，方便单测与复用。
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque

import numpy as np

log = logging.getLogger(__name__)
from vrobot.viz.event_pump import pump as _pump_events


class DebugViewer:
    """第二窗口：真值 vs 传感器位姿的差异可视化。"""

    def __init__(self, enabled: bool = True, max_points: int = 4000, draw_interval: float = 0.1):
        self.enabled = enabled
        self.max_points = max_points
        self.draw_interval = draw_interval
        self._t: deque[int] = deque(maxlen=max_points)
        self._gt: deque[tuple[float, float, float]] = deque(maxlen=max_points)
        self._odom: deque[tuple[float, float, float]] = deque(maxlen=max_points)
        self._slam: deque[tuple[float, float, float]] = deque(maxlen=max_points)
        self._score: deque[float] = deque(maxlen=max_points)
        self._matched: deque[bool] = deque(maxlen=max_points)
        self._fig = None
        self._axes = None
        self._lines: dict[str, object] = {}
        self._last_draw = 0.0
        if not enabled:
            return
        try:
            import matplotlib.pyplot as plt

            self._plt = plt
            # 中文字体（Windows: 微软雅黑/黑体；Linux/macOS 依次回退）
            plt.rcParams["font.sans-serif"] = [
                "Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                "WenQuanYi Micro Hei", "Arial Unicode MS", "sans-serif",
            ]
            plt.rcParams["axes.unicode_minus"] = False  # 负号随 CJK 字体一起丢失的问题
            plt.ion()
            self._fig, self._axes = plt.subplots(2, 2, figsize=(14, 9))
            self._fig.tight_layout(pad=3.0)
            if self._fig.canvas.manager:
                self._fig.canvas.manager.set_window_title("vrobot Debug: GT vs Sensor")
                self._unset_topmost()
            (ax_traj, ax_pos), (ax_th, ax_score) = self._axes
            ax_traj.set_title("轨迹对照 (world px)")
            ax_traj.set_aspect("equal")
            ax_traj.grid(True, ls=":", alpha=0.5)
            ax_pos.set_title("位置误差 (px)")
            ax_pos.grid(True, ls=":", alpha=0.5)
            ax_th.set_title("航向误差 (deg)")
            ax_th.grid(True, ls=":", alpha=0.5)
            ax_score.set_title("扫描匹配得分")
            ax_score.set_ylim(-0.05, 1.05)
            ax_score.grid(True, ls=":", alpha=0.5)
        except Exception as e:  # 无显示环境等情况
            log.warning("debug 窗口不可用，已禁用: %s", e)
            self.enabled = False

    # ---- 数据入口 ----

    def push(self, gt: tuple[float, float, float], odom: tuple[float, float, float],
             slam_pose: tuple[float, float, float],
             score: float = 0.0, matched: bool = False) -> None:
        """追加一帧三方位姿（GT / 里程计 / SLAM）及本帧匹配得分，必要时重绘。"""
        if not self.enabled:
            return
        idx = (self._t[-1] + 1) if self._t else 0
        self._t.append(idx)
        self._gt.append(gt)
        self._odom.append(odom)
        self._slam.append(slam_pose)
        self._score.append(float(score))
        self._matched.append(bool(matched))
        now = time.time()
        if now - self._last_draw >= self.draw_interval:
            self._last_draw = now
            self._draw()

    # ---- 绘制 ----

    def _draw(self) -> None:
        n = len(self._t)
        if n == 0:
            return
        try:
            gt = np.array(self._gt, dtype=np.float64)
            odom = np.array(self._odom, dtype=np.float64)
            slam = np.array(self._slam, dtype=np.float64)
            score = np.array(self._score, dtype=np.float64)
            matched = np.array(self._matched, dtype=bool)
            t = np.array(self._t, dtype=np.int64)

            # 数据过长时抽稀绘制（窗口只保留最近 max_points 帧，已是天然滑窗）
            step = max(1, n // 1500)
            gt_d, odom_d, slam_d, t_d = gt[::step], odom[::step], slam[::step], t[::step]

            pos_err_odom = np.hypot(odom[:, 0] - gt[:, 0], odom[:, 1] - gt[:, 1])
            pos_err_slam = np.hypot(slam[:, 0] - gt[:, 0], slam[:, 1] - gt[:, 1])
            th_err = np.degrees(
                (odom[:, 2] - gt[:, 2] + math.pi) % (2 * math.pi) - math.pi
            )
            th_err_slam = np.degrees(
                (slam[:, 2] - gt[:, 2] + math.pi) % (2 * math.pi) - math.pi
            )

            (ax_traj, ax_pos), (ax_th, ax_score) = self._axes
            if not self._lines:
                (self._lines["gt"],) = ax_traj.plot([], [], "g-", lw=1.2, label="ground truth")
                (self._lines["odom"],) = ax_traj.plot([], [], "b--", lw=1.0, label="odom")
                (self._lines["slam"],) = ax_traj.plot([], [], "r-", lw=1.2, label="slam est")
                (self._lines["gt_end"],) = ax_traj.plot([], [], "g+", ms=12)
                (self._lines["odom_end"],) = ax_traj.plot([], [], "b+", ms=12)
                (self._lines["slam_end"],) = ax_traj.plot([], [], "r+", ms=12)
                (self._lines["err_odom"],) = ax_pos.plot([], [], "b-", lw=1.0, label="odom err")
                (self._lines["err_slam"],) = ax_pos.plot([], [], "r-", lw=1.2, label="slam err")
                (self._lines["th_odom"],) = ax_th.plot([], [], "b-", lw=1.0, label="odom dth")
                (self._lines["th_slam"],) = ax_th.plot([], [], "r-", lw=1.2, label="slam dth")
                # 得分图：接受=绿点，拒绝（有得分但未过门限）=红点，未匹配（预热/无点）不画
                (self._lines["sc_ok"],) = ax_score.plot([], [], "g.", ms=3, label="matched")
                (self._lines["sc_rej"],) = ax_score.plot([], [], "r.", ms=3, label="rejected")
                (self._lines["sc_line"],) = ax_score.plot([], [], "k-", lw=0.8, alpha=0.5, label="score")
                ax_traj.legend(loc="upper right", fontsize=8)
                ax_pos.legend(loc="upper left", fontsize=8)
                ax_th.legend(loc="upper left", fontsize=8)
                ax_score.legend(loc="lower right", fontsize=8)

            self._lines["gt"].set_data(gt_d[:, 0], gt_d[:, 1])
            self._lines["odom"].set_data(odom_d[:, 0], odom_d[:, 1])
            self._lines["slam"].set_data(slam_d[:, 0], slam_d[:, 1])
            self._lines["gt_end"].set_data([gt[-1, 0]], [gt[-1, 1]])
            self._lines["odom_end"].set_data([odom[-1, 0]], [odom[-1, 1]])
            self._lines["slam_end"].set_data([slam[-1, 0]], [slam[-1, 1]])

            def _fit_limits(ax, arr_xy) -> None:
                pad = 30.0
                ax.set_xlim(arr_xy[:, 0].min() - pad, arr_xy[:, 0].max() + pad)
                ax.set_ylim(arr_xy[:, 1].min() - pad, arr_xy[:, 1].max() + pad)

            _fit_limits(ax_traj, np.vstack([gt[:, :2], odom[:, :2], slam[:, :2]]))

            self._lines["err_odom"].set_data(t_d, pos_err_odom[::step])
            self._lines["err_slam"].set_data(t_d, pos_err_slam[::step])
            ax_pos.set_xlim(t[0], max(t[-1], t[0] + 50))
            ax_pos.set_ylim(0, max(5.0, float(pos_err_odom.max()), float(pos_err_slam.max())) * 1.1)

            self._lines["th_odom"].set_data(t_d, th_err[::step])
            self._lines["th_slam"].set_data(t_d, th_err_slam[::step])
            ax_th.set_xlim(t[0], max(t[-1], t[0] + 50))
            y_max = max(5.0, float(np.abs(th_err).max()), float(np.abs(th_err_slam).max())) * 1.1
            ax_th.set_ylim(-y_max, y_max)

            # 匹配得分图：得分曲线 + 接受/拒绝散点
            has_score = score > 0.0
            sc_t, sc_v = t[has_score], score[has_score]
            if sc_t.size:
                acc = matched[has_score]
                self._lines["sc_line"].set_data(sc_t[::step], sc_v[::step])
                self._lines["sc_ok"].set_data(sc_t[acc][::step], sc_v[acc][::step])
                rej = ~acc
                self._lines["sc_rej"].set_data(sc_t[rej][::step], sc_v[rej][::step])
                ax_score.set_title(f"扫描匹配得分  accepted={int(matched.sum())}/{len(matched)}")
            ax_score.set_xlim(t[0], max(t[-1], t[0] + 50))

            ax_pos.set_title(f"位置误差 (px)  last: odom={pos_err_odom[-1]:.1f} slam={pos_err_slam[-1]:.1f}")
            ax_th.set_title(f"航向误差 (deg)  last: odom={th_err[-1]:.1f} slam={th_err_slam[-1]:.1f}")
            # 不用 plt.pause()：它每帧把窗口抬到 Z 序顶端并抢焦点，导致"永远置顶"、无法拖动。
            # draw_idle 后主动泵 GUI 事件（event_pump.py），窗口 z 序/焦点交给系统正常管理。
            self._fig.canvas.draw_idle()
            _pump_events(self._fig)
        except Exception as e:
            log.debug("debug viewer draw failed: %s", e)

    def set_min_score(self, min_score: float) -> None:
        """在得分图上画 min_score 门限水平线（调用一次即可）。"""
        if not self.enabled or self._fig is None:
            return
        try:
            (_, _), (_, ax_score) = self._axes
            ax_score.axhline(min_score, color="orange", ls="--", lw=1.0,
                             label=f"min_score={min_score:.2f}")
            ax_score.legend(loc="lower right", fontsize=8)
        except Exception:
            pass

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

    def close(self) -> None:
        if self.enabled and self._fig is not None:
            try:
                self._plt.ioff()
                self._plt.close(self._fig)
            except Exception:
                pass
