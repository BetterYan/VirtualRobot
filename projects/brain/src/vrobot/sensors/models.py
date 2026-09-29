"""传感器数据模型（与 protocols/message-schema.md 对齐）。

坐标系：Godot 像素坐标，x 向右、y 向下、theta 同 Godot rotation；
前进方向 = (cos(theta), sin(theta))。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Odometry2D:
    """带噪里程计位姿（Godot 积分后的绝对里程计读数）。"""

    x: float
    y: float
    theta: float
    seq: int = 0
    ts: int = 0
    dt: float = 0.05

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)


@dataclass
class LaserScan2D:
    """2D 激光扫描。ranges[i] 对应局部角 angle_min + i*angle_inc；无回波为 -1。"""

    ranges: np.ndarray
    angle_min: float = -np.pi
    angle_inc: float = np.pi / 36
    range_max: float = 320.0
    seq: int = 0
    ts: int = 0

    def valid_ranges(self) -> np.ndarray:
        """剔除超量程读数后的有效距离数组（含对应角度）。"""
        mask = (self.ranges > 0.0) & (self.ranges < self.range_max)
        angles = self.angle_min + np.arange(len(self.ranges)) * self.angle_inc
        return np.stack([angles[mask], self.ranges[mask]], axis=1)

    def local_endpoints(self) -> np.ndarray:
        """扫描命中点在机器人局部坐标系的 (N,2) 数组。"""
        ra = self.valid_ranges()
        if ra.size == 0:
            return np.zeros((0, 2))
        angles = ra[:, 0]
        r = ra[:, 1]
        return np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)


@dataclass
class SensorFrame:
    """一次完整传感器快照 = 里程计 + 激光（SLAM 的唯一输入）。"""

    odom: Odometry2D
    scan: LaserScan2D
    collision: bool = False


@dataclass
class GroundTruth:
    """真值位姿 —— 仅 eval/ 使用；slam/ 模块禁止引用本类型。"""

    x: float
    y: float
    theta: float
    seq: int = 0
    ts: int = 0
    extra: dict = field(default_factory=dict)


def scan_world_points(scan: LaserScan2D, pose: tuple[float, float, float]) -> np.ndarray:
    """把扫描命中点变换到世界坐标 (N,2)。"""
    pts = scan.local_endpoints()
    if pts.size == 0:
        return pts
    c, s = np.cos(pose[2]), np.sin(pose[2])
    rot = np.array([[c, -s], [s, c]])
    return pts @ rot.T + np.array([pose[0], pose[1]])
