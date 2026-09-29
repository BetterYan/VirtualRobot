"""测试用合成理想 2D 世界：轴对齐矩形墙体 + 解析式射线投射。

不依赖 Godot —— 用于在纯 Python 环境端到端验证 SLAM 算法。
布局与 projects/godot/scripts/core/config.gd 一致（房间 + 4 件家具）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from vrobot.sensors.models import LaserScan2D, Odometry2D, SensorFrame

# --- 与 Godot config.gd 对齐的世界尺寸 ---
ROOM_W = 900.0
ROOM_H = 640.0
ROBOT_RADIUS = 14.0
WALL_T = 14.0

DEFAULT_FURNITURE = [
    (180.0, 140.0, 140.0, 100.0),
    (520.0, 120.0, 160.0, 90.0),
    (300.0, 380.0, 100.0, 160.0),
    (640.0, 420.0, 140.0, 120.0),
]

DEFAULT_BEAMS = 72
DEFAULT_RANGE_MAX = 320.0


@dataclass
class SimParams:
    beams: int = DEFAULT_BEAMS
    range_max: float = DEFAULT_RANGE_MAX
    range_noise_std: float = 2.0        # 激光高斯噪声 px
    odom_speed_noise: float = 0.02      # 里程计速度比例噪声
    dt: float = 0.05                    # 传感器帧间隔 s（20Hz）
    seed: int = 7


class SimWorld:
    """理想世界：射线-AABB 求交，完美几何。"""

    def __init__(self, furniture: list | None = None):
        self.rects: list[tuple[float, float, float, float]] = []
        # 四面墙（Godot 同款：厚度 WALL_T 的实心条）
        self.rects.append((0.0, -WALL_T, ROOM_W, WALL_T))                    # 上
        self.rects.append((0.0, ROOM_H, ROOM_W, WALL_T))                     # 下
        self.rects.append((-WALL_T, 0.0, WALL_T, ROOM_H))                    # 左
        self.rects.append((ROOM_W, 0.0, WALL_T, ROOM_H))                     # 右
        self.rects.extend(furniture or DEFAULT_FURNITURE)

    @staticmethod
    def _ray_aabb(ox, oy, dx, dy, rect) -> float | None:
        """slab 法求最近交点距离；无交返回 None。"""
        x0, y0, w, h = rect
        x1, y1 = x0 + w, y0 + h
        tmin, tmax = 0.0, float("inf")
        for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return None
            else:
                t1, t2 = (lo - o) / d, (hi - o) / d
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
                if tmin > tmax:
                    return None
        return tmin if tmin > 1e-6 else None

    def cast(self, ox: float, oy: float, angle: float, r_max: float) -> float:
        """从 (ox,oy) 沿 angle 打一条射线，返回最近命中距离；无命中返回 -1。"""
        dx, dy = math.cos(angle), math.sin(angle)
        best = None
        for rect in self.rects:
            t = self._ray_aabb(ox, oy, dx, dy, rect)
            if t is not None and 0 < t <= r_max and (best is None or t < best):
                best = t
        return -1.0 if best is None else best

    def inside_free(self, x: float, y: float, margin: float = ROBOT_RADIUS) -> bool:
        """点（含机器人半径余量）是否在可行区域内。"""
        for x0, y0, w, h in self.rects:
            if (x0 - margin) < x < (x0 + w + margin) and (y0 - margin) < y < (y0 + h + margin):
                return False
        return True


class SimRobot:
    """差速驱动机器人：真值积分 + 带噪里程计 + 带噪激光。"""

    def __init__(self, world: SimWorld, start: tuple[float, float, float], params: SimParams | None = None):
        self.world = world
        self.p = params or SimParams()
        self.rng = np.random.default_rng(self.p.seed)
        self.true_pose = list(start)
        self.odom_pose = list(start)
        self.seq = 0

    def step(self, linear: float, angular: float) -> SensorFrame:
        """前进一步（dt），返回一帧带噪传感器数据。"""
        dt = self.p.dt
        # 1) 真值积分 + 简单碰撞（撞墙不动）
        th = self.true_pose[2] + angular * dt
        nx = self.true_pose[0] + linear * math.cos(th) * dt
        ny = self.true_pose[1] + linear * math.sin(th) * dt
        if self.world.inside_free(nx, ny):
            self.true_pose = [nx, ny, th]
        else:
            self.true_pose = [self.true_pose[0], self.true_pose[1], th]

        # 2) 带噪里程计（速度比例噪声 + 积分漂移）
        v_n = linear * (1.0 + self.rng.normal(0, self.p.odom_speed_noise))
        w_n = angular * (1.0 + self.rng.normal(0, self.p.odom_speed_noise))
        th_o = self.odom_pose[2] + w_n * dt
        self.odom_pose = [
            self.odom_pose[0] + v_n * math.cos(th_o) * dt,
            self.odom_pose[1] + v_n * math.sin(th_o) * dt,
            th_o,
        ]

        # 3) 带噪激光
        angles = -math.pi + np.arange(self.p.beams) * (2 * math.pi / self.p.beams)
        ranges = np.empty(self.p.beams, dtype=np.float32)
        for i, a in enumerate(angles):
            g = self.world.cast(self.true_pose[0], self.true_pose[1], th + a, self.p.range_max)
            ranges[i] = -1.0 if g < 0 else g
        hit = ranges > 0
        ranges[hit] += self.rng.normal(0, self.p.range_noise_std, size=int(hit.sum()))
        ranges[~hit] = -1.0

        self.seq += 1
        odom = Odometry2D(*self.odom_pose, seq=self.seq, ts=self.seq, dt=dt)
        scan = LaserScan2D(
            ranges=ranges,
            angle_min=-math.pi,
            angle_inc=2 * math.pi / self.p.beams,
            range_max=self.p.range_max,
            seq=self.seq,
            ts=self.seq,
        )
        return SensorFrame(odom=odom, scan=scan)

    @property
    def truth(self) -> tuple[float, float, float]:
        return tuple(self.true_pose)


def drive_to_waypoints(
    robot: SimRobot, waypoints: list[tuple[float, float]], max_steps: int = 4000
):
    """简单逐点驾驶（控制器允许使用真值——它只负责采集数据，与 SLAM 无关）。

    卡死保护：真值位置连续 40 步几乎不动则跳过当前路点（仿真碰撞卡住）。
    返回 frames: list[SensorFrame]
    """
    frames = []
    v, w = 90.0, 2.2
    target_i = 0
    stuck_count = 0
    last_pos = robot.truth[:2]
    for _ in range(max_steps):
        if target_i >= len(waypoints):
            break
        tx, ty = waypoints[target_i]
        x, y, th = robot.truth
        if math.hypot(x - last_pos[0], y - last_pos[1]) < 0.5:
            stuck_count += 1
            if stuck_count > 40:  # 卡死：跳过该路点
                target_i += 1
                stuck_count = 0
                continue
        else:
            stuck_count = 0
            last_pos = (x, y)
        dx, dy = tx - x, ty - y
        if math.hypot(dx, dy) < 12.0:
            target_i += 1
            continue
        desired = math.atan2(dy, dx)
        diff = (desired - th + math.pi) % (2 * math.pi) - math.pi
        cmd_w = max(-w, min(w, 2.5 * diff))
        cmd_v = v if abs(diff) < 0.5 else v * 0.3
        frames.append(robot.step(cmd_v, cmd_w))
    return frames


def gt_wall_mask(world: SimWorld, grid, band: float = 4.0) -> np.ndarray:
    """真值墙面栅格：格中心落在任一矩形边界 band 内视为墙面。"""
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    cs = grid.cell_size
    for row in range(grid.height):
        for col in range(grid.width):
            cx, cy = (col + 0.5) * cs, (row + 0.5) * cs
            for x0, y0, w, h in world.rects:
                x1, y1 = x0 + w, y0 + h
                near_x = (x0 - band) <= cx <= (x1 + band)
                near_y = (y0 - band) <= cy <= (y1 + band)
                on_edge = (near_x and (abs(cy - y0) <= band or abs(cy - y1) <= band)) or (
                    near_y and (abs(cx - x0) <= band or abs(cx - x1) <= band)
                )
                if on_edge:
                    mask[row, col] = True
                    break
    return mask
