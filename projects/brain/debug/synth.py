"""Stage 2 共享激励：理想差速驱动合成器（完美里程计 + 解析射线扫描）。

用于隔离测试 cartographer 引擎契约 —— 不含 Godot、不含噪声、不含网络。
坐标系约定与 Godot 一致：px 坐标 y 向下，theta 正 = 使 y 增大方向旋转。
"""

from __future__ import annotations

import math

import numpy as np

from vrobot.sensors.models import LaserScan2D, Odometry2D

DT = 0.05  # 20Hz


class SynthBot:
    """理想运动学机器人：完美里程计、解析射线扫描、简单碰撞（撞墙停平移）。"""

    def __init__(self, world, start=(100.0, 90.0, 0.0), beams=144, range_max=320.0):
        self.world = world
        self.x, self.y, self.th = start
        self.spawn = start
        self.beams = beams
        self.range_max = range_max
        self.path = 0.0

    def step(self, v: float, w: float):
        self.th += w * DT
        nx = self.x + v * math.cos(self.th) * DT
        ny = self.y + v * math.sin(self.th) * DT
        if self.world.inside_free(nx, ny):
            self.path += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        odom = Odometry2D(self.x, self.y, self.th, dt=DT)
        return odom, self.scan()

    def scan(self) -> LaserScan2D:
        n = self.beams
        ranges = np.empty(n, dtype=np.float32)
        step = 2 * math.pi / n
        for i in range(n):
            a = -math.pi + i * step
            g = self.world.cast(self.x, self.y, self.th + a, self.range_max)
            ranges[i] = -1.0 if g < 0 else g
        return LaserScan2D(ranges=ranges, angle_min=-math.pi, angle_inc=step,
                           range_max=self.range_max)


def wrap_deg(d: float) -> float:
    return math.degrees((d + math.pi) % (2 * math.pi) - math.pi)


def report(name: str, checks: list[tuple[str, bool, str]]) -> bool:
    """打印一项用例的判定清单，返回是否全部通过。"""
    ok = all(passed for _, passed, _ in checks)
    print(f"\n=== {name}: {'PASS' if ok else 'FAIL'} ===")
    for label, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    return ok
