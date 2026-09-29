"""里程计增量合成：把两次里程计读数差映射到当前位姿估计上。

约定（与 Godot 端一致）：
  pose = (x, y, theta)，前进方向 = (cos(theta), sin(theta))。
  delta 从 odom_k-1 -> odom_k 提取（在 odom_k-1 的机体坐标系表达），
  再以同样方式作用到当前位姿估计 pose 上：
    pose' = pose ⊕ delta
"""

from __future__ import annotations

import math


def wrap_angle(a: float) -> float:
    """归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def odom_delta(
    o_prev: tuple[float, float, float], o_cur: tuple[float, float, float]
) -> tuple[float, float, float]:
    """提取相邻两次里程计读数间的机体系运动增量 (dx, dy, dtheta)。

    dx/dy 在 o_prev 机体坐标系下表达（x 前 y 右，此处 y 与世界同向为"机体右侧"）。
    """
    dx_w = o_cur[0] - o_prev[0]
    dy_w = o_cur[1] - o_prev[1]
    dth = wrap_angle(o_cur[2] - o_prev[2])
    c, s = math.cos(o_prev[2]), math.sin(o_prev[2])
    # 世界增量旋转到 o_prev 机体系：R(-theta) @ [dx_w, dy_w]
    dx = dx_w * c + dy_w * s
    dy = -dx_w * s + dy_w * c
    return dx, dy, dth


def compose(pose: tuple[float, float, float], delta: tuple[float, float, float]) -> tuple:
    """pose ⊕ delta：把机体系增量作用到世界位姿上。"""
    x, y, th = pose
    dx, dy, dth = delta
    c, s = math.cos(th), math.sin(th)
    return (x + dx * c - dy * s, y + dx * s + dy * c, wrap_angle(th + dth))


class OdomTracker:
    """维护上一帧里程计，输出每帧增量。"""

    def __init__(self):
        self._last: tuple[float, float, float] | None = None

    def reset(self) -> None:
        self._last = None

    def delta_since_last(self, odom: tuple[float, float, float]) -> tuple[float, float, float] | None:
        """返回与上一帧的增量；首帧返回 None。"""
        if self._last is None:
            self._last = odom
            return None
        d = odom_delta(self._last, odom)
        self._last = odom
        return d
