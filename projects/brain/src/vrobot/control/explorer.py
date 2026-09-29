"""自主探索与运动控制。

- FrontierExplorer：自主探索建图（frontier-based exploration, Yamauchi 1997）。
  不断选择"已知自由区与未知区边界"中信息价值最高的区域，A* 规划前往，
  直到图中再无可达 frontier —— 得到当前可达范围内完整一致的地图。
- ReactiveExplorer：反应式避障走行（保留：调试 / frontier 兜底模式）。

两者接口一致：update(slam, scan) -> (linear, angular)。
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass

import numpy as np

from vrobot.planning.path_finder import astar, make_passable
from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 反应式（原简单避障，保留为兜底/调试模式）
# ---------------------------------------------------------------------------


class ReactiveExplorer:
    """雷达避障走行：前方近障碍则转向开阔侧，否则直行。"""

    def __init__(
        self,
        linear: float = 55.0,
        angular: float = 1.6,
        front_dist: float = 60.0,
        beam_spread: int = 7,
    ):
        self.linear = float(linear)
        self.angular = float(angular)
        self.front_dist = float(front_dist)
        self.beam_spread = int(beam_spread) | 1

    def update(self, slam, scan: LaserScan2D) -> tuple[float, float]:
        n = len(scan.ranges)
        if n == 0:
            return (0.0, 0.0)
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        half = self.beam_spread // 2
        idx = [(center + i) % n for i in range(-half, half + 1)]
        front = [scan.ranges[i] for i in idx if scan.ranges[i] > 0]
        d_front = min(front) if front else scan.range_max

        if d_front >= self.front_dist:
            return (self.linear, 0.0)
        left_idx = [(center + i) % n for i in range(half + 1, half + 1 + 8)]
        right_idx = [(center - i) % n for i in range(half + 1, half + 1 + 8)]
        left = [scan.ranges[i] for i in left_idx if scan.ranges[i] > 0]
        right = [scan.ranges[i] for i in right_idx if scan.ranges[i] > 0]
        d_left = np.mean(left) if left else 0.0
        d_right = np.mean(right) if right else 0.0
        turn = self.angular if d_left >= d_right else -self.angular
        return (10.0, turn)


# 向后兼容别名（旧配置/调用引用）
AutoExplorer = ReactiveExplorer


# ---------------------------------------------------------------------------
# Frontier 自主探索
# ---------------------------------------------------------------------------


@dataclass
class FrontierParams:
    linear: float = 60.0            # 巡航速度 px/s
    linear_slow: float = 22.0       # 大转向时的速度 px/s
    angular: float = 1.5            # 最大角速度 rad/s（过高会加剧旋转涂抹）
    free_below: float = -0.3        # log-odds 低于此视为（观测过的）自由区；-0.35=单次穿越
    occupied_above: float = 0.62    # 高于此视为占据
    inflate_cells: int = 2          # 障碍膨胀格数（≈ 机器人半径 14px / 8px 格）
    min_frontier_size: int = 4      # 忽略过小的未知碎片
    unknown_cost: float = 3.0       # A* 穿越未知区的额外代价
    goal_tol: float = 14.0          # 路点到达半径 px
    replan_every: int = 60          # 兜底重规划间隔（帧，20Hz → 3s）；目标承诺制下很少触发
    stuck_window: int = 50          # 卡死检测窗口（帧）
    stuck_move: float = 6.0         # 窗口内位移低于此视为卡死 px
    area_window: int = 250          # 区域停滞检测窗口（帧）
    area_move: float = 55.0         # 窗口内活动半径低于此视为停滞 px
    front_panic: float = 26.0       # 前方障碍紧急转向距离 px
    max_blacklist: int = 12         # 黑名单上限（防死循环）


@dataclass
class _Frontier:
    centroid: tuple[int, int]       # (col, row)
    size: int


def find_frontiers(
    logodds: np.ndarray,
    free_below: float,
    occupied_above: float,
    min_size: int,
) -> list[_Frontier]:
    """在 log-odds 栅格上检测 frontier（模块级函数，供多种探索器复用）。

    未知且与已确认自由区相邻的格，聚类成 frontier。
    聚类前对掩码做 1 格膨胀：雷达射线锥间隙（144 线在 320px 处 ≈1.75 格）
    会把边界未知格切成孤立碎片，不膨胀则整条边界全被 min_size 过滤。
    簇大小按膨胀前的原始格数统计，质心也取原始格。
    """
    free = logodds <= free_below
    occupied = logodds >= occupied_above
    unknown = (~free) & (~occupied)
    if not unknown.any():
        return []
    # free 的 8 邻域膨胀 ∩ unknown = frontier 候选
    free_dil = free.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            free_dil |= np.roll(np.roll(free, dy, axis=0), dx, axis=1)
    frontier = unknown & free_dil
    if not frontier.any():
        return []

    # 膨胀 1 格用于连通，原始格用于计数/质心
    fd = frontier.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            fd |= np.roll(np.roll(frontier, dy, axis=0), dx, axis=1)

    h, w = fd.shape
    seen = np.zeros_like(fd, dtype=bool)
    clusters: list[_Frontier] = []
    for r0, c0 in np.argwhere(fd):
        if seen[r0, c0]:
            continue
        q = deque([(int(r0), int(c0))])
        seen[r0, c0] = True
        members = []
        while q:
            r, c = q.popleft()
            members.append((c, r))
            for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
                if 0 <= nc < w and 0 <= nr < h and fd[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    q.append((nr, nc))
        orig = [(c, r) for c, r in members if frontier[r, c]]
        if len(orig) >= min_size:
            arr = np.array(orig)
            clusters.append(
                _Frontier(
                    centroid=(int(arr[:, 0].mean()), int(arr[:, 1].mean())),
                    size=len(orig),
                )
            )
    return clusters


class FrontierExplorer:
    """Frontier 探索：驶向未知边界，直到无可达边界（建图完成）。"""

    def __init__(self, cell_size: float = 8.0, **kwargs):
        self.p = FrontierParams(**kwargs)
        self.cell_size = float(cell_size)
        self.done = False
        self._path: list[tuple[int, int]] = []       # (col,row) 格路径
        self._wp_i = 0
        self._replan_countdown = 0
        self._stuck_hist: deque[tuple[float, float]] = deque(maxlen=self.p.stuck_window)
        self._area_hist: deque[tuple[float, float]] = deque(maxlen=self.p.area_window)
        self._blacklist: list[tuple[int, int]] = []  # 被放弃的 frontier 质心
        self._target: _Frontier | None = None

    # ---- 主入口 ----

    def update(self, slam, scan: LaserScan2D) -> tuple[float, float]:
        """每帧调用：返回 (linear, angular) 指令。"""
        if self.done:
            return (0.0, 0.0)
        pose = slam.pose
        x, y = pose[0], pose[1]

        # 卡死检测：窗口内几乎没动 → 放弃当前目标并强制重规划
        self._stuck_hist.append((x, y))
        if len(self._stuck_hist) == self.p.stuck_window:
            pts = np.array(self._stuck_hist)
            if float(np.linalg.norm(pts[-1] - pts[0])) < self.p.stuck_move:
                self._drop_target("卡死")
                self._stuck_hist.clear()

        # 区域停滞检测：长时间在小范围内打转（未触发卡死但无进展）→ 换目标
        self._area_hist.append((x, y))
        if len(self._area_hist) == self.p.area_window:
            pts = np.array(self._area_hist)
            span = float(max(pts[:, 0].max() - pts[:, 0].min(), pts[:, 1].max() - pts[:, 1].min()))
            if span < self.p.area_move:
                self._drop_target("区域停滞")
                self._area_hist.clear()

        # 兜底重规划（目标承诺制下很少触发：目标存活时不换目标）
        self._replan_countdown -= 1
        if self._replan_countdown <= 0 or not self._path:
            self._plan(slam.grid, pose)
            self._replan_countdown = self.p.replan_every
            if self.done:
                return (0.0, 0.0)

        # 路点跟随
        v, w = self._follow(pose)

        # 反应式安全层：贴脸障碍原地转向（不前顶，避免撞墙推挤）
        d_front = self._front_distance(scan)
        if 0 < d_front < self.p.front_panic:
            v, w = 0.0, self.p.angular * self._avoid_sign(scan)

        return (v, w)

    def _drop_target(self, reason: str) -> None:
        if self._target is not None and len(self._blacklist) < self.p.max_blacklist:
            log.debug("%s：放弃目标 %s", reason, self._target.centroid)
            self._blacklist.append(self._target.centroid)
        self._target = None
        self._path = []
        self._replan_countdown = 0

    # ---- 规划 ----

    def _plan(self, grid: OccupancyGrid, pose: tuple[float, float, float]) -> None:
        frontiers = self._find_frontiers(grid)
        path = self._pick_and_plan(grid, pose, frontiers)
        if path is None:
            if frontiers:
                log.info("存在 %d 个 frontier 但均不可达 → 视为建图完成", len(frontiers))
            else:
                log.info("无 frontier → 建图完成（栅格更新 %d 次）", grid.n_updates)
            self.done = True
            self._path = []
            return
        self._path = path
        self._wp_i = 0

    def _find_frontiers(self, grid: OccupancyGrid) -> list[_Frontier]:
        return find_frontiers(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.min_frontier_size
        )

    def _pick_and_plan(
        self, grid: OccupancyGrid, pose: tuple[float, float, float], frontiers: list[_Frontier]
    ) -> list[tuple[int, int]] | None:
        """目标承诺制：当前目标 frontier 仍存活 → 继续去；否则按 信息量/距离 选新的。"""
        if not frontiers:
            return None
        passable, unknown = make_passable(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.inflate_cells
        )
        start = grid.world_to_cell(pose[0], pose[1])
        # 起点逃逸气泡：机器人已物理存在于该处，其周围 3 格内的自由区允许通行，
        # 否则贴墙起步时 A* 无路可走（严格膨胀会封掉机器人自身邻域）
        sc, sr = start
        lo = grid._logodds
        r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
        c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
        passable[r0:r1, c0:c1] |= lo[r0:r1, c0:c1] <= self.p.free_below

        def try_plan_to(f: _Frontier) -> list[tuple[int, int]] | None:
            goal = self._nearest_free_cell(passable, f.centroid)
            if goal is None:
                return None
            res = astar(passable, unknown, start, goal, unknown_cost=self.p.unknown_cost)
            return res.path or None

        # 1) 目标承诺：当前目标仍存活（质心 ±4 格内还有同类 frontier）→ 继续前往
        if self._target is not None:
            t = self._target
            alive = any(
                abs(f.centroid[0] - t.centroid[0]) <= 4 and abs(f.centroid[1] - t.centroid[1]) <= 4
                for f in frontiers
            )
            if alive:
                path = try_plan_to(t)
                if path:
                    return path
            # 目标已消亡或不可达 → 清空，走正常选择
            self._target = None

        def blacklisted(f: _Frontier) -> bool:
            return any(
                abs(f.centroid[0] - b[0]) <= 2 and abs(f.centroid[1] - b[1]) <= 2
                for b in self._blacklist
            )

        def score(f: _Frontier) -> float:
            d = abs(f.centroid[0] - start[0]) + abs(f.centroid[1] - start[1])
            return f.size / (1.0 + d)

        for f in sorted(frontiers, key=score, reverse=True):
            if blacklisted(f):
                continue
            path = try_plan_to(f)
            if path:
                self._target = f
                return path
            log.debug("frontier %s 不可达，加入黑名单", f.centroid)
            if len(self._blacklist) < self.p.max_blacklist:
                self._blacklist.append(f.centroid)
        return None

    @staticmethod
    def _nearest_free_cell(passable: np.ndarray, cell: tuple[int, int]) -> tuple[int, int] | None:
        """frontier 质心附近 5 格内找可通行目标格（质心本身常是未知格）。"""
        c0, r0 = cell
        best, best_d = None, 1e9
        for dr in range(-5, 6):
            for dc in range(-5, 6):
                c, r = c0 + dc, r0 + dr
                if 0 <= c < passable.shape[1] and 0 <= r < passable.shape[0] and passable[r, c]:
                    d = dc * dc + dr * dr
                    if d < best_d:
                        best, best_d = (c, r), d
        return best

    # ---- 执行 ----

    def _follow(self, pose: tuple[float, float, float]) -> tuple[float, float]:
        """沿 A* 路径跟随；丢弃已到达路点，走完则下帧重规划。"""
        cs = self.cell_size
        while self._wp_i < len(self._path):
            wc, wr = self._path[self._wp_i]
            wx, wy = (wc + 0.5) * cs, (wr + 0.5) * cs
            if np.hypot(wx - pose[0], wy - pose[1]) < self.p.goal_tol:
                self._wp_i += 1
                continue
            desired = float(np.arctan2(wy - pose[1], wx - pose[0]))
            diff = (desired - pose[2] + np.pi) % (2 * np.pi) - np.pi
            w_cmd = max(-self.p.angular, min(self.p.angular, 2.5 * diff))
            v = self.p.linear if abs(diff) < 0.5 else self.p.linear_slow
            return (v, w_cmd)
        self._path = []
        return (self.p.linear_slow, 0.0)

    # ---- 传感器辅助 ----

    @staticmethod
    def _front_distance(scan: LaserScan2D) -> float:
        n = len(scan.ranges)
        if n == 0:
            return -1.0
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        front = [scan.ranges[(center + i) % n] for i in range(-3, 4) if scan.ranges[(center + i) % n] > 0]
        return min(front) if front else -1.0

    @staticmethod
    def _avoid_sign(scan: LaserScan2D) -> float:
        n = len(scan.ranges)
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        left = [scan.ranges[(center + i) % n] for i in range(4, 12)]
        right = [scan.ranges[(center - i) % n] for i in range(4, 12)]
        d_l = np.mean([r for r in left if r > 0]) if any(r > 0 for r in left) else 0.0
        d_r = np.mean([r for r in right if r > 0]) if any(r > 0 for r in right) else 0.0
        return 1.0 if d_l >= d_r else -1.0
