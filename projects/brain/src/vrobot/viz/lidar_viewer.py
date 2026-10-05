"""实时雷达视图：以机器人为中心绘制激光扫描，直观显示障碍分布。

- 绿色射线：该方向有回波，长度 = 首个障碍距离（射线覆盖范围 = 无障碍空间）
- 红色圆点：扫描命中点（障碍）
- 灰色细线：超量程无回波（range_max 内该方向无障碍）
- 坐标系与 Godot 一致：x 向右、y 向下（视图 y 轴反转保持与地图窗口同向）
"""

from __future__ import annotations

import logging

import numpy as np

log = logging.getLogger(__name__)
from vrobot.viz.event_pump import pump as _pump_events


class LidarViewer:
    """matplotlib 交互式雷达窗口（与 MapViewer 同风格，事件循环内刷新）。"""

    def __init__(self, enabled: bool = True, range_max: float = 320.0):
        self.enabled = enabled
        self.range_max = range_max
        self._fig = None
        self._frame = 0
        if not enabled:
            return
        try:
            import matplotlib.pyplot as plt
            from matplotlib.collections import LineCollection

            self._plt = plt
            try:  # 中文字体（Windows 常见：微软雅黑/黑体），避免中文显示为方框
                from matplotlib import font_manager

                for name in ("Microsoft YaHei", "SimHei", "PingFang SC", "Noto Sans CJK SC"):
                    try:
                        font_manager.findfont(name, fallback_to_default=False)
                        plt.rcParams["font.sans-serif"] = [name]
                        plt.rcParams["axes.unicode_minus"] = False
                        break
                    except ValueError:
                        continue
            except Exception:
                pass
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(7, 7))
            if self._fig.canvas.manager:
                self._fig.canvas.manager.set_window_title("vrobot 激光雷达视图")
                self._unset_topmost()

            ax = self._ax
            lim = range_max * 1.08
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.set_aspect("equal")
            ax.invert_yaxis()  # Godot y 向下，与地图窗口方向一致
            ax.set_facecolor("#0d1117")
            ax.grid(True, color="#30363d", lw=0.4, alpha=0.6)

            # 量程圈 + 刻度圈（ls=线型，ec=颜色）
            for r, style in ((range_max, "--"), (range_max / 2, ":")):
                ax.add_patch(
                    plt.Circle((0, 0), r, fill=False, ec="#8b949e", ls=style, lw=0.8)
                )

            # 有回波的射线（绿，表示自由空间）与命中点（红，障碍）
            self._hit_rays = LineCollection([], colors="#2ea043", lw=1.0, alpha=0.55)
            self._miss_rays = LineCollection([], colors="#8b949e", lw=0.7, alpha=0.35)
            ax.add_collection(self._hit_rays)
            ax.add_collection(self._miss_rays)
            (self._hit_pts,) = ax.plot([], [], "o", color="#f85149", ms=4, label="障碍")
            (self._robot,) = ax.plot(
                [], [], "^", color="#58a6ff", ms=11, label="机器人"
            )
            ax.legend(loc="upper right", fontsize=8, labelcolor="#c9d1d9")
            ax.set_xlabel("x (px，机器人前方为车头朝向)")
            self._fig.tight_layout()
        except Exception as e:  # 无显示环境等
            log.warning("雷达视图不可用，已禁用: %s", e)
            self.enabled = False

    def _unset_topmost(self) -> None:
        """防御性关闭 topmost（与 MapViewer 相同的处理）。"""
        try:
            win = self._fig.canvas.manager.window
            if hasattr(win, "attributes"):  # Tk
                win.attributes("-topmost", False)
        except Exception:
            pass

    def update(self, scan) -> None:
        """传入 LaserScan2D 刷新一帧雷达图。"""
        if not self.enabled or self._fig is None:
            return
        try:
            ranges = np.asarray(scan.ranges, dtype=float)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_inc
            hit = (ranges > 0.0) & (ranges < scan.range_max)
            miss = ~hit

            hx, hy = ranges[hit], angles[hit]
            hit_pts = np.stack([hx * np.cos(hy), hx * np.sin(hy)], axis=1)
            self._hit_rays.set_segments(
                np.stack([np.zeros_like(hit_pts), hit_pts], axis=1)
                if hit_pts.size
                else []
            )
            self._hit_pts.set_data(
                (hit_pts[:, 0], hit_pts[:, 1]) if hit_pts.size else ([], [])
            )

            mx, my = scan.range_max, angles[miss]
            miss_pts = np.stack([mx * np.cos(my), mx * np.sin(my)], axis=1)
            self._miss_rays.set_segments(
                np.stack([np.zeros_like(miss_pts), miss_pts], axis=1)
                if miss_pts.size
                else []
            )

            self._robot.set_data([0.0], [0.0])
            self._ax.set_title(
                f"雷达视图  frame={self._frame}  障碍束 {int(hit.sum())}/{len(ranges)}",
                color="#c9d1d9",
                fontsize=10,
            )
            # 不用 plt.pause()：事件泵方案见 event_pump.py（同 MapViewer）
            self._fig.canvas.draw_idle()
            _pump_events(self._fig)
            self._frame += 1
        except Exception as e:
            if not getattr(self, "_warned", False):
                self._warned = True
                log.warning("雷达视图刷新异常（仅提示一次）: %s", e)

    def close(self) -> None:
        if self.enabled and self._fig is not None:
            try:
                self._plt.ioff()
                self._plt.close(self._fig)
            except Exception:
                pass
