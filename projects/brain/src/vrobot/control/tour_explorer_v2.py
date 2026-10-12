"""FUEL-lite 分层自主探索器 v2（2D 简化版，重构版）。

任务目标：探索地图的**可行走区域**，直到完整绘制地图（frontier 耗尽且
排除了"观测手段暂时耗尽"的假象，见收敛判据）。

设计依据：docs/tour_explorer_v2_physical_notes.md（探索问题的物理描述）。

相对 tour_explorer.py（v1）的结构性改进 —— 目标是"可扩展、可说明"：

1. **状态机显式化**（`State`）：PROBE / FOLLOW / WAIT / COOLDOWN / ESCAPE /
   DONE，每个状态的进入/退出条件集中在一处（`update` 的状态推进段），
   而非散落在多层 if 里。
2. **异常登记表**（`Anomaly` + `AnomalyBook` + 具名处理器）：
   探索中所有"卡住"的物理现象（卡死 / 绕圈 / 动态障碍 / 位姿漂移 /
   空图 / 观察点饥饿 / 规划异常…）集中枚举，每种异常对应一个处理器。
   新增异常 = 新增枚举项 + 注册一个处理器，主循环不改动。
3. **层拆分为独立类**，主类 `TourExplorerV2` 只做编排：
   - `MapSnapshot`    一次规划所需的全部语义掩码（含位姿出界守卫）
   - `TourPlanner`    层1：frontier 聚类 → 观察点（信息增益）→ 环游排序
   - `PathFollower`   层2：确定性"对准-直行"跟随（自含游标与到达判定）
   - `SafetyGuard`    层3：贴脸障碍 / 卡死检测（自含证据窗口）
   - `ConvergenceJudge` 收敛判据（区分"暂时观测不到"与"物理不可达"）
   - `Blacklist`      观察点黑名单（TTL）
4. **外部接口与 v1 完全兼容**：`update(slam, scan) -> (linear, angular)`、
   `done`（可被外部复位）、`revive()`、`close()`、`n_plans`。

线程约定（与 v1 相同的单写者原则）：
- `_pending`（规划产出）由规划线程写、主线程收集；
- `_path`/游标只由主线程写；
- 规划线程可以设置**帧基门控**（`_cooldown_until` / `_probe_until` /
  `done`），主线程在状态推进段读取这些门控并完成状态转移——
  规划线程从不直接改 `self.state`。
"""

from __future__ import annotations

import enum
import heapq
import logging
import math
import threading
from collections import Counter, deque
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from vrobot.planning.path_finder import astar, inflate, make_passable
from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 参数（沿用 v1 的基本参数，按层分组）
# ---------------------------------------------------------------------------


@dataclass
class ExplorerParams:
    # ---- 运动（层2 输出上限） ----
    linear: float = 60.0            # 巡航速度 px/s（更高会劣化扫描匹配）
    linear_slow: float = 22.0       # 末段（距目标 <40px）速度下限 px/s
    angular: float = 1.5            # 最大角速度 rad/s
    # ---- 栅格语义（自由/占据/未知的 log-odds 阈值） ----
    free_below: float = -0.3
    occupied_above: float = 0.62
    inflate_cells: int = 2
    unknown_cost: float = 3.0       # A*/Dijkstra 穿越未知区的额外代价（风险项）
    # ---- frontier 聚类 ----
    min_frontier_size: int = 4      # 忽略过小的未知碎片
    # ---- 观察点 / 信息增益 ----
    gain_beams: int = 48            # 每个候选观察点的射线数
    gain_range_cells: int = 40      # 增益射线最大步长（格，40 格 = 320px = 量程）
    vp_radius: int = 8              # 观察点候选：簇成员周围该半径内的自由格
    vp_step: int = 2                # 候选采样步距（格）
    vp_max_cands: int = 8           # 每簇最多评估的候选数（按距离场取最近的）
    vp_min_gain: int = 12           # 期望可见未知格低于此视为无价值
    vp_min_dist: int = 4            # 观察点距机器人的最小距离（格）
                                    # 防"脚下观察点"死锁：代价≈0 的近点永远赢
    lambda_cost: float = 20.0       # score = gain / (lambda + path_cost)
    # ---- 环游 ----
    tour_refresh: int = 500         # 环游重建周期（帧，20Hz → 25s）
    replan_check: int = 50          # 地图变化检查周期（帧）
    map_change_frac: float = 0.03   # 未知格数变化比例 ≥ 此值 → 作废环游
    # ---- 路径跟随（层2：确定性"对准-直行"，无圆弧转向律） ----
    lookahead: float = 50.0         # 前视目标点距离 px（沿路径弧长，随游标单调前移）
    align_th: float = 0.35          # 航向误差超过此值 → 满速原地转（rad）
    align_kp: float = 2.0           # 已对准时的比例航向修正增益（一阶收敛，不振荡）
    w_rate_limit: float = 0.8       # 每帧角速度变化上限 rad（20Hz → 16 rad/s²）
    goal_tol: float = 14.0          # 观察点到达半径 px
    # ---- 安全（层3） ----
    front_panic: float = 26.0       # 前方障碍紧急转向距离 px
    stuck_window: int = 60          # 卡死检测窗口（帧，3s）
    stuck_move: float = 6.0         # 窗口内位移低于此视为卡死 px
    progress_timeout: int = 150     # 路径游标无进展帧数上限（防绕圈）
    escape_frames: int = 25         # 卡死后原地转向开阔侧的帧数
    # ---- 打转看门狗 ----
    spin_turns_limit: float = 2.0   # 无学习累计总转角 ≥ 此圈数 → SPIN_LOCK（奔赴最近未知区）
    spin_cooldown: int = 100        # SPIN_LOCK 打破后的冷却帧数
    # ---- 黑名单 / 收敛 ----
    blacklist_ttl: int = 900        # 黑名单条目存活帧数（45s）
    max_blacklist: int = 24
    stall_builds: int = 10          # 跨位移的环游重建次数达到该值且 unknown 不减 → 收敛
    stall_frac: float = 0.998       # unknown 降幅不足（当前/历史 ≥ 该值）视为无进展
    # ---- 空图引导 ----
    bootstrap_free_min: int = 100   # 自由格低于此视为地图空白 → 直行探测
    bootstrap_frames: int = 60      # 每轮直行探测帧数（3s）
    # ---- 异步规划 ----
    async_plan: bool = True         # 重规划放后台线程（防同步阻塞 → 走停）
    planner_poll: float = 0.01      # 规划线程轮询间隔 s


# ---------------------------------------------------------------------------
# 异常登记表（docs §6 —— 每一项都对应一个物理成因）
# ---------------------------------------------------------------------------


class Anomaly(enum.Enum):
    """探索逻辑需要处理的异常情况（物理成因见 docs §6 表格）。

    扩展方式：新增枚举项 + 在 TourExplorerV2._register_handlers 注册
    一个处理器（签名 `handler(detail: str, ctx: dict) -> None`）。
    """

    FRONT_PANIC = enum.auto()       # 贴脸障碍：视锥内近距占据物，继续前进必碰撞
    STUCK = enum.auto()             # 物理卡死：指令在而动量未建立（3s 位移<6px）
    NO_PROGRESS = enum.auto()       # 绕圈/死锁：机器人在动但游标不前进
    PATH_BLOCKED = enum.auto()      # 动态障碍：新观测截断剩余路径
    LEG_UNREACHABLE = enum.auto()   # 环游腿不可达：观察点被新观测封闭
    VP_STARVATION = enum.auto()     # 观察点饥饿：合法观察点全被黑名单屏蔽
    POSE_DRIFT = enum.auto()        # 位姿漂移：估计位姿飘出栅格边界
    BLANK_MAP = enum.auto()         # 空图：SLAM 未同步，定义上无 frontier
    PLANNER_CRASH = enum.auto()     # 后台规划线程未捕获异常（软件层故障）
    SPIN_LOCK = enum.auto()         # 打转死锁：无学习累计总转角 ≥2 圈（含带
                                    #   位移的小半径绕圈）→ 高优先级奔赴未知区


@dataclass
class AnomalyRecord:
    anomaly: Anomaly
    frame: int
    detail: str


class AnomalyBook:
    """异常登记簿：集中记录 + 频率统计，供外部诊断与调参。"""

    def __init__(self, maxlen: int = 256):
        self._recent: deque[AnomalyRecord] = deque(maxlen=maxlen)
        self.counts: Counter[Anomaly] = Counter()

    def record(self, anomaly: Anomaly, frame: int, detail: str = "") -> None:
        self._recent.append(AnomalyRecord(anomaly, frame, detail))
        self.counts[anomaly] += 1

    def count(self, anomaly: Anomaly) -> int:
        return self.counts[anomaly]

    @property
    def recent(self) -> list[AnomalyRecord]:
        return list(self._recent)


# ---------------------------------------------------------------------------
# 状态机（物理时间尺度：PROBE/FOLLOW 有运动输出，其余静止等待）
# ---------------------------------------------------------------------------


class State(enum.Enum):
    PROBE = enum.auto()     # 空图直行探测：积累观测（BLANK_MAP 的响应）
    FOLLOW = enum.auto()    # 执行环游腿（确定性对准-直行）
    WAIT = enum.auto()      # 无路径：收集/提交规划（含后台规划等待）
    COOLDOWN = enum.auto()  # 冷却：观察点饥饿 / 位姿漂移 / 规划异常
    ESCAPE = enum.auto()    # 卡死/绕圈逃逸：原地转向开阔侧
    DONE = enum.auto()      # 建图完成


# ---------------------------------------------------------------------------
# 通用构件：frontier 聚类 / 距离场（与 v1 同语义）
# ---------------------------------------------------------------------------


@dataclass
class Cluster:
    cid: int
    cells: list[tuple[int, int]]    # 原始 frontier 格 (col, row)
    centroid: tuple[int, int]


@dataclass
class Viewpoint:
    cell: tuple[int, int]           # 已确认自由格 (col, row)
    cluster_id: int
    gain: float                     # 射线投影估计的期望可见未知格数（信息增益）
    cost: float                     # 距离场路径代价


def _dilate8(mask: np.ndarray) -> np.ndarray:
    """8 邻域膨胀（零填充，不环形回绕——np.roll 会制造假 frontier）。"""
    h, w = mask.shape
    padded = np.pad(mask, 1)
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            out |= padded[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
    return out


def find_clusters(
    logodds: np.ndarray,
    free_below: float,
    occupied_above: float,
    min_size: int,
) -> list[Cluster]:
    """未知 ∧ 已确认自由区 8 邻域 = frontier；膨胀 1 格连通聚类，保留成员格。"""
    free = logodds <= free_below
    occupied = logodds >= occupied_above
    unknown = (~free) & (~occupied)
    if not unknown.any():
        return []
    frontier = unknown & _dilate8(free)
    if not frontier.any():
        return []
    fd = _dilate8(frontier)

    h, w = fd.shape
    seen = np.zeros_like(fd, dtype=bool)
    clusters: list[Cluster] = []
    cid = 0
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
                Cluster(cid=cid, cells=orig,
                        centroid=(int(arr[:, 0].mean()), int(arr[:, 1].mean())))
            )
            cid += 1
    return clusters


def dijkstra_field(
    passable: np.ndarray,
    unknown: np.ndarray,
    start: tuple[int, int],
    unknown_cost: float = 3.0,
) -> np.ndarray:
    """从 start 到全场的最短代价场 (H, W) float32；不可达 = inf。start=(col,row)。"""
    h, w = passable.shape
    dist = np.full((h, w), np.inf, dtype=np.float32)
    sc, sr = int(start[0]), int(start[1])
    if not (0 <= sc < w and 0 <= sr < h and passable[sr, sc]):
        return dist
    dist[sr, sc] = 0.0
    heap: list[tuple[float, int, int]] = [(0.0, sc, sr)]
    push, pop = heapq.heappush, heapq.heappop
    while heap:
        d, c, r = pop(heap)
        if d > dist[r, c]:
            continue
        for nc, nr in ((c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)):
            if not (0 <= nc < w and 0 <= nr < h) or not passable[nr, nc]:
                continue
            nd = d + (unknown_cost if unknown[nr, nc] else 1.0)
            if nd < dist[nr, nc]:
                dist[nr, nc] = nd
                push(heap, (nd, nc, nr))
    return dist


# ---------------------------------------------------------------------------
# 地图快照：一次规划所需的全部语义掩码（含位姿出界守卫）
# ---------------------------------------------------------------------------


@dataclass
class MapSnapshot:
    """规划输入的不可变视图。`capture` 返回 None = 位姿出界（POSE_DRIFT）。"""

    lo: np.ndarray
    free: np.ndarray
    occ: np.ndarray
    unknown: np.ndarray
    passable: np.ndarray
    field: np.ndarray               # Dijkstra 距离场（起点=机器人格）
    start: tuple[int, int]          # 机器人格 (col, row)
    free_count: int
    unknown_count: int

    @classmethod
    def capture(
        cls, grid: OccupancyGrid, p: ExplorerParams, pose: tuple[float, float, float]
    ) -> MapSnapshot | None:
        lo = grid._logodds
        free = lo <= p.free_below
        occ = lo >= p.occupied_above
        unknown = (~free) & (~occ)
        start = grid.world_to_cell(pose[0], pose[1])
        if not grid.in_bounds(*start):
            # 物理成因（docs §4）：估计位姿飘出栅格 → 距离场起点无效，
            # 全场 inf、观察点全部"消失"——此刻看到的世界是假的，不做规划
            return None
        passable, _ = make_passable(lo, p.free_below, p.occupied_above, p.inflate_cells)
        # 起点逃逸气泡：机器人已物理存在于该处，邻域 3 格内的自由区允许通行
        sc, sr = start
        r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
        c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
        passable[r0:r1, c0:c1] |= free[r0:r1, c0:c1]
        field = dijkstra_field(passable, unknown, start, p.unknown_cost)
        return cls(
            lo=lo, free=free, occ=occ, unknown=unknown, passable=passable,
            field=field, start=start,
            free_count=int(free.sum()), unknown_count=int(unknown.sum()),
        )


# ---------------------------------------------------------------------------
# 层1：环游规划器（frontier 聚类 → 观察点 → 排序；纯逻辑，无共享状态）
# ---------------------------------------------------------------------------


class TourPlanner:
    """FUEL-lite 层1：信息增益 × 路径代价的效用规划。

    无内部状态（除参数），所有输入显式传入——便于单测与替换策略。
    """

    def __init__(self, p: ExplorerParams):
        self.p = p

    # -- 观察点生成 --

    def build_viewpoints(
        self, snap: MapSnapshot, clusters: list[Cluster], blacklist: Blacklist
    ) -> list[Viewpoint]:
        """每簇选一个观察点：score = gain / (λ + cost) 最优。

        两遍策略：优先取"距机器人 ≥ vp_min_dist"的候选（防脚下观察点
        死锁——代价≈0 的近点在 score 下永远赢，机器人原地不动）；
        全部簇都无候选时再放宽距离约束兜底。
        """
        vps: list[Viewpoint] = []
        for min_dist in (self.p.vp_min_dist, 0):
            for cl in clusters:
                vp = self._best_viewpoint(snap, cl, blacklist, min_dist)
                if vp is not None:
                    vps.append(vp)
            if vps:
                break
        return vps

    def _best_viewpoint(
        self, snap: MapSnapshot, cluster: Cluster,
        blacklist: Blacklist, min_dist: int,
    ) -> Viewpoint | None:
        p = self.p
        h, w = snap.passable.shape
        R, step = p.vp_radius, p.vp_step
        md2 = float(min_dist * min_dist)
        cands: set[tuple[int, int]] = set()
        for c, r in cluster.cells[::2]:
            c0, c1 = max(0, c - R), min(w, c + R + 1)
            r0, r1 = max(0, r - R), min(h, r + R + 1)
            for nr in range(r0, r1, step):
                for nc in range(c0, c1, step):
                    if not (snap.free[nr, nc] and snap.passable[nr, nc]
                            and np.isfinite(snap.field[nr, nc])):
                        continue
                    if min_dist > 0:
                        dc, dr = nc - snap.start[0], nr - snap.start[1]
                        if dc * dc + dr * dr < md2:
                            continue
                    cands.add((nc, nr))
        if not cands:
            return None
        ordered = sorted(cands, key=lambda t: snap.field[t[1], t[0]])[: p.vp_max_cands]
        best: tuple[float, Viewpoint] | None = None
        for cell in ordered:
            if cell in blacklist:
                continue
            cost = float(snap.field[cell[1], cell[0]])
            gain = self.ray_gain(cell, snap.occ, snap.unknown)
            if gain < p.vp_min_gain:
                continue
            s = gain / (p.lambda_cost + cost)
            if best is None or s > best[0]:
                best = (s, Viewpoint(cell=cell, cluster_id=cluster.cid,
                                     gain=gain, cost=cost))
        return best[1] if best else None

    def ray_gain(
        self, cell: tuple[int, int], occ: np.ndarray, unknown: np.ndarray
    ) -> float:
        """信息增益的物理估计：从观察格向 gain_beams 个方向步进，
        数可见未知格数（光沿直线传播，撞占据即遮挡）。"""
        r0, c0 = cell[1], cell[0]
        n = self.p.gain_beams
        max_r = self.p.gain_range_cells
        count = 0
        for k in range(n):
            a = 2.0 * math.pi * k / n
            dx, dy = math.cos(a), math.sin(a)
            fx, fy = float(c0), float(r0)
            for _ in range(max_r):
                fx += dx
                fy += dy
                c, r = int(fx), int(fy)
                if not (0 <= c < occ.shape[1] and 0 <= r < occ.shape[0]):
                    break
                if occ[r, c]:
                    break
                if unknown[r, c]:
                    count += 1
        return float(count)

    # -- 环游排序 --

    def order_tour(
        self, vps: list[Viewpoint], start: tuple[int, int], lo: np.ndarray
    ) -> list[Viewpoint]:
        """效用加权最近邻 + 2-opt（腿间代价用欧氏 × 线段穿越惩罚近似，
        避免 N² 次 Dijkstra）。"""
        p = self.p

        def leg_cost(a: tuple[int, int], b: tuple[int, int]) -> float:
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            if d < 1e-6:
                return 0.0
            pen = 1.0
            steps = max(int(d), 1)
            for i in range(1, steps + 1):
                t = i / steps
                c = int(a[0] + (b[0] - a[0]) * t)
                r = int(a[1] + (b[1] - a[1]) * t)
                if not (0 <= c < lo.shape[1] and 0 <= r < lo.shape[0]):
                    continue
                v = lo[r, c]
                if v >= p.occupied_above:
                    pen *= 1.02
                elif v > p.free_below:
                    pen *= 1.01
            return d * pen

        # 最近邻（效用加权：score = gain/(λ+est)）
        remaining = list(vps)
        order: list[Viewpoint] = []
        cur = start
        while remaining:
            best_i, best_s = 0, -1.0
            for i, vp in enumerate(remaining):
                s = vp.gain / (p.lambda_cost + leg_cost(cur, vp.cell))
                if s > best_s:
                    best_i, best_s = i, s
            vp = remaining.pop(best_i)
            order.append(vp)
            cur = vp.cell

        # 2-opt（标准 delta + 传递上限：口径不一致的假改进 → 无界震荡）
        max_passes = 50
        improved = True
        while improved and max_passes > 0:
            improved = False
            max_passes -= 1
            for i in range(len(order) - 1):
                for j in range(i + 2, len(order)):
                    a = order[i - 1].cell if i > 0 else start
                    b, c = order[i].cell, order[j - 1].cell
                    d = order[j].cell
                    before = leg_cost(a, b) + leg_cost(c, d)
                    after = leg_cost(a, c) + leg_cost(b, d)
                    if after < before - 1e-6:
                        order[i:j] = reversed(order[i:j])
                        improved = True
        return order


# ---------------------------------------------------------------------------
# 层2：确定性"对准-直行"路径跟随（无圆弧转向律 → 物理上不可能绕圈）
# ---------------------------------------------------------------------------


class PathFollower:
    """确定性跟随器。自含状态（路径、只进游标），`step` 返回
    (v, w, reached)。

    只有两个稳定模态：未对准则满速原地转（最短方向），已对准则直行 +
    小比例修正。锁定前视目标后坚定直线执行，不逐帧重瞄 → 无过冲、无
    极限环、无绕圈。角速度速率限幅由主类统一施加（安全层覆写之后）。"""

    def __init__(self, p: ExplorerParams, cell_size: float):
        self.p = p
        self.cs = float(cell_size)
        self._path: list[tuple[int, int]] = []   # (col, row)
        self._cursor = 0                          # 最近路径点游标（只前进）

    @property
    def path(self) -> list[tuple[int, int]]:
        return self._path

    @property
    def cursor(self) -> int:
        return self._cursor

    def set_path(self, path: list[tuple[int, int]]) -> None:
        self._path = list(path)
        self._cursor = 0

    def clear(self) -> None:
        self._path = []
        self._cursor = 0

    def step(self, pose: tuple[float, float, float]) -> tuple[float, float, bool]:
        p = self.p
        pts = [((c + 0.5) * self.cs, (r + 0.5) * self.cs) for c, r in self._path]
        x, y, th = pose[0], pose[1], pose[2]

        # 到达当前腿终点 → 由主类推进环游
        d_goal = math.hypot(pts[-1][0] - x, pts[-1][1] - y)
        if d_goal < p.goal_tol:
            return (0.0, 0.0, True)

        # 最近点投影：取距机器人最近的路径点（切角绕过角点后，投影会
        # 自动跳到更靠后的路点，不再被"14px 邻近"卡在角点）。游标取
        # 投影的单调最大值（只前进），供看门狗判定进展。
        nearest = 0
        dmin = math.inf
        for k, (px, py) in enumerate(pts):
            d = (px - x) * (px - x) + (py - y) * (py - y)
            if d < dmin:
                nearest, dmin = k, d
        self._cursor = max(self._cursor, nearest)

        # 目标点：末段直接瞄准终点；否则沿路径从投影点起累计弧长，取
        # ≥ lookahead 的第一个点——保证目标距机器人 ≥ lookahead，永不
        # 落在脚下（atan2 对近零向量方向跳变是原地打转的另一源头）。
        L = p.lookahead
        if d_goal < L:
            tx, ty = pts[-1]
        else:
            j, acc = nearest, 0.0
            while j + 1 < len(pts):
                seg = math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
                if acc + seg >= L:
                    break
                acc += seg
                j += 1
            if j + 1 < len(pts):
                seg = math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
                t = (L - acc) / max(seg, 1e-6)
                tx = pts[j][0] + (pts[j + 1][0] - pts[j][0]) * t
                ty = pts[j][1] + (pts[j + 1][1] - pts[j][1]) * t
            else:
                tx, ty = pts[-1]

        desired = math.atan2(ty - y, tx - x)
        alpha = (desired - th + math.pi) % (2.0 * math.pi) - math.pi

        # 确定性"对准-直行"控制（无圆弧转向律）。
        #
        # 为什么彻底废弃纯追踪圆弧律 w = 2v·sinα/L：它在低速+近距离时会
        # 过冲穿越目标 → α 变号 → 以 v/w 为半径绕目标公转，是绕圈的物理
        # 来源，任何看门狗都是事后补救。
        #
        # 本控制器只有两个稳定模态，物理上不可能绕圈：
        #  1) |α| > align_th：满速原地转（固定转速、最短方向，不随 sinα
        #     缩放——α≈±π 也不会停转）。α 单调下降，≤ π/angular ≈ 2s 必
        #     离开此态；
        #  2) |α| ≤ align_th：直行 + 小比例航向修正，航向误差一阶收敛。
        # 一旦锁定目标（前视点沿路径单调前移），就坚定直线执行，不再
        # 逐帧重瞄产生左右摆动。
        if abs(alpha) > p.align_th:
            return (0.0, math.copysign(p.angular, alpha), False)

        v = p.linear
        if d_goal < 40.0:
            v = min(v, max(p.linear_slow, d_goal * 1.2))
        w = max(-p.angular, min(p.angular, p.align_kp * alpha))
        return (v, w, False)


# ---------------------------------------------------------------------------
# 层3：反应安全（贴脸障碍 / 卡死证据窗口）
# ---------------------------------------------------------------------------


class SafetyGuard:
    """贴脸检测无状态；卡死检测自含证据窗口（只在指令运动的状态喂入）。"""

    def __init__(self, window: int, move: float):
        self._window = window
        self._move = move
        self._hist: deque[tuple[float, float]] = deque(maxlen=window)

    def feed(self, pose: tuple[float, float, float]) -> bool:
        """喂入一帧位姿；返回 True = 判定卡死（证据一次性消费）。"""
        self._hist.append((pose[0], pose[1]))
        if len(self._hist) == self._window:
            pts = np.array(self._hist)
            if float(np.linalg.norm(pts[-1] - pts[0])) < self._move:
                self._hist.clear()
                return True
        return False

    def reset(self) -> None:
        """按设计静止的状态（WAIT/COOLDOWN/ESCAPE）不计卡死证据。"""
        self._hist.clear()

    @staticmethod
    def front_distance(scan: LaserScan2D | None) -> float:
        if scan is None:
            return -1.0
        n = len(scan.ranges)
        if n == 0:
            return -1.0
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        front = [scan.ranges[(center + i) % n]
                 for i in range(-3, 4) if scan.ranges[(center + i) % n] > 0]
        return min(front) if front else -1.0

    @staticmethod
    def avoid_sign(scan: LaserScan2D | None) -> float:
        """转向开阔侧：光学比较左右舷测距均值（哪边空旷转向哪边）。"""
        if scan is None:
            return 1.0
        n = len(scan.ranges)
        if n == 0:
            return 1.0
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        left = [scan.ranges[(center + i) % n] for i in range(4, 12)]
        right = [scan.ranges[(center - i) % n] for i in range(4, 12)]
        d_l = np.mean([r for r in left if r > 0]) if any(r > 0 for r in left) else 0.0
        d_r = np.mean([r for r in right if r > 0]) if any(r > 0 for r in right) else 0.0
        return 1.0 if d_l >= d_r else -1.0


# ---------------------------------------------------------------------------
# 收敛判据 / 黑名单（docs §5.3、§6）
# ---------------------------------------------------------------------------


class ConvergenceJudge:
    """跨重建收敛判据：区分"观测手段暂时耗尽"与"物理不可达碎片"。

    物理依据（docs §5.3）：只有**跨越实际位移**（≥30px）的多次环游重建
    而 unknown 几乎不减，才能证明剩余边界均为不可解碎片。
    原地搅动型重建（零位移）不计入，否则局部卡位会被误判为全局收敛。
    """

    def __init__(self, builds: int, frac: float, move_gate: float = 30.0):
        self.builds = builds
        self.frac = frac
        self.move_gate = move_gate
        self._hist: deque[tuple[float, float, int]] = deque(maxlen=builds)

    def judge(self, pose: tuple[float, float, float], unknown: int) -> bool:
        """记录一次环游重建；返回 True = 判定建图完成。"""
        px, py = pose[0], pose[1]
        moved = (self._hist and math.hypot(
            px - self._hist[-1][0], py - self._hist[-1][1]) >= self.move_gate)
        progressed = (self._hist and unknown < self._hist[-1][2] * self.frac)
        if not self._hist or moved or progressed:
            self._hist.append((px, py, unknown))
        return (len(self._hist) >= self.builds
                and unknown >= self._hist[0][2] * self.frac)

    def reset(self) -> None:
        self._hist.clear()


class Blacklist:
    """观察点黑名单（TTL）：防止不可达腿被环游反复重选。

    条目自动过期（默认 45s）——"永不过期"会误杀临时不可达的好边界
    并连锁触发观察点饥饿 → 过早判完成。
    """

    def __init__(self, ttl: int, max_items: int):
        self.ttl = ttl
        self.max_items = max_items
        self._cells: dict[tuple[int, int], int] = {}   # cell -> 过期帧

    def add(self, cell: tuple[int, int], now: int) -> None:
        # 只拉黑单格：3×3 块拉黑会封锁过多合法观察点，造成观察点饥荒
        if len(self._cells) < self.max_items:
            self._cells[cell] = now + self.ttl

    def purge(self, now: int) -> None:
        self._cells = {c: exp for c, exp in self._cells.items() if exp > now}

    def __contains__(self, cell: object) -> bool:
        return cell in self._cells

    def __len__(self) -> int:
        return len(self._cells)


# ---------------------------------------------------------------------------
# 主类：状态机编排 + 异常分发 + 异步规划边界
# ---------------------------------------------------------------------------


class PlanOutcome(enum.Enum):
    """`_build_tour` 的产出类型（每种都对应 docs §5.3 的一种物理情形）。"""
    TOUR = enum.auto()          # 得到新环游
    DONE = enum.auto()          # 建图完成（frontier 耗尽 / 收敛 / 饥饿超限）
    BLANK = enum.auto()         # 地图空白 → 直行探测
    STARVED = enum.auto()       # 观察点饥饿 → 冷却重试
    POSE_DRIFT = enum.auto()    # 位姿出界 → 冷却等待恢复


class TourExplorerV2:
    """FUEL-lite 分层探索 v2：聚类 → 观察点 → 环游 → 确定性跟随。

    外部接口与 v1（TourExplorer）一致：
    - `update(slam, scan) -> (linear, angular)`，20Hz 每帧调用；
    - `done` 属性（可被外部复位后调用 `revive()`）；
    - `revive()` / `close()` / `n_plans`。
    """

    def __init__(self, cell_size: float = 8.0, **kwargs):
        self.p = ExplorerParams(**kwargs)
        self.cell_size = float(cell_size)
        self.done = False
        self.state = State.WAIT
        self.n_plans = 0
        self.anomalies = AnomalyBook()

        # 分层构件
        self.follower = PathFollower(self.p, cell_size)
        self.guard = SafetyGuard(self.p.stuck_window, self.p.stuck_move)
        self.planner = TourPlanner(self.p)
        self.convergence = ConvergenceJudge(self.p.stall_builds, self.p.stall_frac)
        self.blacklist = Blacklist(self.p.blacklist_ttl, self.p.max_blacklist)

        # 环游 / 腿
        self._frame = 0
        self._tour: list[Viewpoint] = []
        self._tour_i = 0
        self._tour_frame = -10**9
        self._target_vp: Viewpoint | None = None
        self._last_scan: LaserScan2D | None = None

        # 帧基门控（规划线程可写，主线程读后完成状态转移）
        self._probe_until = 0         # BLANK_MAP → PROBE 截止帧
        self._cooldown_until = 0      # 饥饿/漂移/异常 → COOLDOWN 截止帧
        self._escape_until = 0
        self._escape_sign = 1.0

        # 地图变化 / 看门狗
        self._replan_cd = 0
        self._last_unknown = -1
        self._last_cursor = -1
        self._no_progress = 0
        self._vps_empty_streak = 0

        # 指令连续性（速率限幅的状态）
        self._w_prev = 0.0
        self._panic_active = False

        # 打转看门狗：无学习累计总转角（|dθ| 求和，跨状态存活），
        # 由"学习事件"（unknown 减少）清零——绕圈的本质是"转动很多、
        # 学到很少"，用地图熵降作为唯一健康信号，位移不再参与判定。
        self._theta_prev: float | None = None
        self._spin_abs = 0.0
        self._seeking_unknown = False    # 高优先级指令态：直奔最近未知区

        # 异步规划（单写者：_path/游标只由主线程写；_pending 由规划线程产出）
        self._plan_lock = threading.Lock()
        self._plan_request: tuple | None = None
        self._pending: tuple | None = None      # (path, vp)
        self._busy = False
        self._closed = False
        self._wakeup = threading.Event()
        if self.p.async_plan:
            threading.Thread(target=self._plan_loop, daemon=True,
                             name="tour-planner-v2").start()

        # 异常处理器注册表（可扩展点：新增 Anomaly → 注册处理器）
        self._handlers: dict[Anomaly, Callable[[str, dict], None]] = {}
        self._register_handlers()

    # ---- 异常分发（可扩展 / 可说明的核心） ----

    def _register_handlers(self) -> None:
        """每种异常 → 一个具名处理器。子类可覆写本方法增删处理器。"""
        self._handlers = {
            Anomaly.FRONT_PANIC: self._on_front_panic,
            Anomaly.STUCK: self._on_stuck,
            Anomaly.NO_PROGRESS: self._on_no_progress,
            Anomaly.PATH_BLOCKED: self._on_path_blocked,
            Anomaly.LEG_UNREACHABLE: self._on_leg_unreachable,
            Anomaly.VP_STARVATION: self._on_vp_starvation,
            Anomaly.POSE_DRIFT: self._on_pose_drift,
            Anomaly.BLANK_MAP: self._on_blank_map,
            Anomaly.PLANNER_CRASH: self._on_planner_crash,
            Anomaly.SPIN_LOCK: self._on_spin_lock,
        }

    def _raise(self, anomaly: Anomaly, detail: str = "",
               **ctx: object) -> None:
        """统一异常入口：登记 → 分发到处理器。

        处理器可以只设置帧基门控（规划线程安全的做法），
        也可以直接转移状态（仅主线程调用的异常）。
        """
        self.anomalies.record(anomaly, self._frame, detail)
        log.debug("[anomaly] %s @%d %s", anomaly.name, self._frame, detail)
        handler = self._handlers.get(anomaly)
        if handler is not None:
            handler(detail, ctx)

    # -- 处理器（物理成因注释见 docs §6） --

    def _on_front_panic(self, detail: str, ctx: dict) -> None:
        """贴脸障碍：输出级覆写已在 update 末段处理（v=0 + 转开阔侧）。"""
        del detail, ctx

    def _on_stuck(self, detail: str, ctx: dict) -> None:
        """物理卡死：指令在而动量未建立。目标未必是坏目标（可能是外部
        阻挡），不拉黑——拉黑会误杀好观察点并连锁触发过早终止。"""
        del detail, ctx
        self._drop_target()
        self._tour = []
        self._enter_escape()

    def _on_no_progress(self, detail: str, ctx: dict) -> None:
        """绕圈/窄角死锁：位移检测抓不住（机器人在动），但游标不前进。
        目标本身可疑 → 拉黑（TTL）后换目标。"""
        del detail, ctx
        if self._target_vp is not None:
            self.blacklist.add(self._target_vp.cell, self._frame)
        self._drop_target()
        self._tour = []
        self._no_progress = 0
        self._last_cursor = -1
        self._enter_escape()

    def _on_path_blocked(self, detail: str, ctx: dict) -> None:
        """动态障碍：规划基于过期世界模型 → 作废剩余路径，下帧重规划。"""
        del detail, ctx
        self.follower.clear()
        self._target_vp = None

    def _on_leg_unreachable(self, detail: str, ctx: dict) -> None:
        """环游腿不可达：观察点被新观测封闭 → 拉黑防反复重选。"""
        del detail
        cell = ctx.get("cell")
        if isinstance(cell, tuple):
            self.blacklist.add(cell, self._frame)

    def _on_vp_starvation(self, detail: str, ctx: dict) -> None:
        """观察点饥饿：黑名单多来自卡死/临时不可达（TTL 会解除）。
        冷却 150 帧重试；连续 3 次才允许判完成（_build_tour 内裁决）。"""
        del detail, ctx
        self._cooldown_until = max(self._cooldown_until, self._frame + 150)
        self._tour = []

    def _on_pose_drift(self, detail: str, ctx: dict) -> None:
        """位姿飘出栅格：此刻规划器看到的世界是假的 → 原地冷却等待
        位姿恢复（pose graph 优化/新观测），不做任何规划。"""
        del ctx
        log.warning("估计位姿 %s 超出栅格边界 → 冷却等待，暂停规划", detail)
        self._cooldown_until = max(self._cooldown_until, self._frame + 150)
        self._tour = []

    def _on_blank_map(self, detail: str, ctx: dict) -> None:
        """地图空白（SLAM 桥接未同步/首帧未入图）：空白图上定义上不
        存在 frontier，宣布完成是错的 → 直行探测积累观测。"""
        del detail, ctx
        self._probe_until = self._frame + self.p.bootstrap_frames
        self._tour = []

    def _on_planner_crash(self, detail: str, ctx: dict) -> None:
        """后台规划异常（软件层故障）：冷却重试，不让异常杀死控制线程。"""
        del ctx
        self._cooldown_until = max(self._cooldown_until, self._frame + 150)
        self._tour = []

    def _on_spin_lock(self, detail: str, ctx: dict) -> None:
        """打转死锁：无学习累计总转角 ≥ spin_turns_limit 圈。

        高优先级响应（简单物理逻辑）：机器人应**主动探索未探索区域**。
        放弃当前环游与目标，置入 _seeking_unknown 指令态——由 _plan 锁定
        **最近的可达 frontier 自由格**为硬目标、直行奔赴，优先于环游。
        不做 enter_escape（刚转过两圈，再原地转向无意义）；拉黑当前目标
        防止回选。
        """
        del ctx
        log.warning("无学习打转 %s 圈 → 高优先级奔赴最近未知区", detail)
        if self._target_vp is not None:
            self.blacklist.add(self._target_vp.cell, self._frame)
        self._drop_target()
        self._tour = []
        self._no_progress = 0
        self._last_cursor = -1
        self._seeking_unknown = True
        self._cooldown_until = 0        # 不冷却：立刻规划奔赴

    # ---- 主入口（20Hz） ----

    def update(self, slam, scan: LaserScan2D) -> tuple[float, float]:
        """每帧调用：返回 (linear, angular) 指令。

        流程（与 docs §7 的分层对应）：
        1) 卡死证据积累（仅运动状态）
        2) 状态推进（事件驱动的转移，集中可读）
        3) 规划（仅 WAIT 且无路径；异步则后台执行）
        4) 按状态输出指令（层2）
        5) 安全覆写（层3 贴脸）+ 速率限幅（指令连续性）
        """
        if self.done:
            self.state = State.DONE
            return (0.0, 0.0)
        self._frame += 1
        self._last_scan = scan
        pose = slam.pose
        grid = slam.grid

        # 1) 卡死检测：只在指令了运动的状态积累证据（静止待命按设计不动）
        if self.state in (State.FOLLOW, State.PROBE):
            if self.guard.feed(pose):
                self._raise(Anomaly.STUCK, "窗口内位移 < %.0fpx" % self.p.stuck_move)
        else:
            self.guard.reset()

        # 2) 状态推进（所有转移集中在此段，可逐条说明）
        if self.state == State.ESCAPE:
            if self._frame >= self._escape_until:
                self.state = State.WAIT      # 路径已在进入 ESCAPE 时清空
        elif self.state == State.PROBE:
            if self._frame >= self._probe_until:
                self.state = State.WAIT      # 探测期结束 → 自然回到规划
        elif self.state == State.COOLDOWN:
            if self._frame >= self._cooldown_until:
                self.state = State.WAIT      # TTL 到期/位姿恢复 → 重试
        elif self.state == State.WAIT:
            # 规划线程设置的帧基门控 → 状态转移（规划线程从不直接改 state）
            if self._frame < self._probe_until:
                self.state = State.PROBE
            elif self._frame < self._cooldown_until:
                self.state = State.COOLDOWN
            else:
                # 地图显著变化检查在等待期也运行：丢弃后台产出的过期路径
                # （否则旧地图上的 pending 会在收集后被执行）
                self._periodic_map_check(grid)
        elif self.state == State.FOLLOW:
            self._periodic_map_check(grid)
            if self.follower.path and not self._path_valid(grid):
                self._raise(Anomaly.PATH_BLOCKED, "剩余路径落入膨胀占据区")
            if not self.follower.path:
                self.state = State.WAIT

        # 3) 规划（仅 WAIT 且无路径）
        if self.state == State.WAIT and not self.follower.path:
            self._collect_plan()
            if self.done:
                self.state = State.DONE
            elif self.follower.path:
                self.state = State.FOLLOW
            elif not self._busy and self._frame >= self._cooldown_until:
                if self.p.async_plan:
                    self._submit_plan(grid, pose)
                else:
                    self._plan(grid, pose)
                    self._collect_plan()
                    if self.done:
                        self.state = State.DONE
                    elif self.follower.path:
                        self.state = State.FOLLOW

        # 4) 按状态输出（层2）
        if self.state == State.ESCAPE:
            v, w = 0.0, self.p.angular * self._escape_sign
        elif self.state == State.PROBE:
            v, w = self.p.linear, 0.0        # 直行探测积累观测
        elif self.state == State.FOLLOW:
            v, w, reached = self.follower.step(pose)
            if reached:
                self._on_leg_done()
                v, w = 0.0, 0.0
            elif self._watchdog():
                self._raise(Anomaly.NO_PROGRESS,
                            "游标 %d 连续 %d 帧无进展（疑似绕圈/窄角）"
                            % (self.follower.cursor, self.p.progress_timeout))
                v, w = 0.0, self.p.angular * self._escape_sign
        else:                                # WAIT / COOLDOWN / DONE
            v, w = 0.0, 0.0

        # 5) 安全覆写（层3）+ 速率限幅（指令连续性）
        d_front = SafetyGuard.front_distance(scan)
        panic = 0 < d_front < self.p.front_panic
        if panic and not self._panic_active:  # 上升沿才登记，避免刷爆记录
            self._raise(Anomaly.FRONT_PANIC, "%.0fpx" % d_front)
        self._panic_active = panic
        if panic:
            v = 0.0
            w = self.p.angular * SafetyGuard.avoid_sign(scan)

        self._track_spin(pose)
        return self._final_cmd(v, w)

    # ---- 状态机辅助 ----

    def _final_cmd(self, v: float, w: float) -> tuple[float, float]:
        """角速度速率限幅：转向通道的惯性飞轮（角加速度上限）。"""
        lim = self.p.w_rate_limit
        w = max(self._w_prev - lim, min(self._w_prev + lim, w))
        self._w_prev = w
        return (v, w)

    def _track_spin(self, pose: tuple[float, float, float]) -> None:
        """打转看门狗：累计**总转角**（帧间 |dθ| 求和），由学习事件清零。

        物理判别（docs §6）：绕圈的本质是"转动很多、学到很少"。总转角
        达到 spin_turns_limit 圈而地图熵未降 → 判定已进入打转（含带位移
        的小半径绕圈），触发高优先级指令：奔赴最近未知区（见 _on_spin_lock）。

        为什么用总 |dθ| 而非净转角：来回画 8 字（换向）也会被抓到；
        为什么位移不参与判定：绕圈带一点位移时，位移门控会被反复重置
        而漏判。正常行进/跨已探区直行总转角很小，不会误触；一旦地图
        熵下降（学到了），累计清零，看门狗自动"失忆"。
        """
        th = pose[2]
        if self._theta_prev is not None:
            d = (th - self._theta_prev + math.pi) % (2.0 * math.pi) - math.pi
            self._spin_abs += abs(d)
        self._theta_prev = th
        limit = self.p.spin_turns_limit * 2.0 * math.pi
        if self._spin_abs >= limit:
            turns = self._spin_abs / (2.0 * math.pi)
            self._spin_abs = 0.0
            self._raise(Anomaly.SPIN_LOCK, "%.1f" % turns)

    def _enter_escape(self) -> None:
        self._escape_until = self._frame + self.p.escape_frames
        self._escape_sign = (SafetyGuard.avoid_sign(self._last_scan)
                             if self._last_scan else 1.0)
        self.state = State.ESCAPE

    def _drop_target(self) -> None:
        self._target_vp = None
        self.follower.clear()
        self._pending = None

    def _on_leg_done(self) -> None:
        """当前腿完成：推进环游（若环游已重建则不推进，由 _plan 重裁决）。"""
        if (self._target_vp is not None and self._tour_i < len(self._tour)
                and self._tour[self._tour_i] is self._target_vp):
            self._tour_i += 1
        self._drop_target()
        self.state = State.WAIT

    def _watchdog(self) -> bool:
        """无进展看门狗：游标持续不前进（含绕圈——位移检测抓不住）。"""
        if self.follower.cursor != self._last_cursor:
            self._last_cursor = self.follower.cursor
            self._no_progress = 0
        else:
            self._no_progress += 1
        if self._no_progress >= self.p.progress_timeout:
            return True
        return False

    def _periodic_map_check(self, grid: OccupancyGrid) -> None:
        """地图显著变化（低频）：只作废环游、丢弃过期产出，保持当前腿
        承诺——否则建图初期会高频打断执行。"""
        self._replan_cd -= 1
        if self._replan_cd > 0:
            return
        self._replan_cd = self.p.replan_check
        lo = grid._logodds
        u = int(((~(lo <= self.p.free_below)) & (~(lo >= self.p.occupied_above))).sum())
        if self._last_unknown >= 0:
            if u < self._last_unknown:
                # 学习事件：地图熵下降 → 打转看门狗累计清零 + 退出
                # "直奔未知区"指令态（已学到新东西，交还环游裁决）
                self._spin_abs = 0.0
                self._seeking_unknown = False
            if u > 0:
                frac = abs(u - self._last_unknown) / u
                if frac >= self.p.map_change_frac:
                    log.debug("未知格变化 %.1f%% → 标记环游待重建", frac * 100)
                    self._tour = []
                    self._pending = None   # 后台可能产出自旧地图的过期路径
        self._last_unknown = u

    def _path_valid(self, grid: OccupancyGrid) -> bool:
        """剩余路径逐 4 格采样，不得落入膨胀占据区（动态障碍即时响应）。"""
        lo = grid._logodds
        blocked = inflate(lo >= self.p.occupied_above, 1)
        h, w = blocked.shape
        for c, r in self.follower.path[max(self.follower.cursor, 0)::4]:
            if 0 <= c < w and 0 <= r < h and blocked[r, c]:
                return False
        return True

    # ---- 规划（层1；可在主线程同步执行，也可在后台线程异步执行） ----

    def _submit_plan(self, grid: OccupancyGrid,
                     pose: tuple[float, float, float]) -> None:
        if self._busy or self._closed:
            return
        self._busy = True
        with self._plan_lock:
            self._plan_request = (grid, (pose[0], pose[1], pose[2]))
        self._wakeup.set()

    def _collect_plan(self) -> None:
        """主线程收集后台产出（_path/游标只由主线程写）。"""
        if self._pending is None:
            return
        path, vp = self._pending
        self._pending = None
        self.follower.set_path(path)
        self._target_vp = vp

    def _plan_loop(self) -> None:
        while not self._closed:
            self._wakeup.wait(self.p.planner_poll)
            self._wakeup.clear()
            with self._plan_lock:
                req = self._plan_request
                self._plan_request = None
            if req is None:
                continue
            grid, pose = req
            try:
                self._plan(grid, pose)
            except Exception:
                log.exception("后台规划异常")
                self._raise(Anomaly.PLANNER_CRASH, "规划线程未捕获异常")
            self._busy = False

    def _plan(self, grid: OccupancyGrid,
              pose: tuple[float, float, float], _depth: int = 0) -> None:
        """沿环游推进：当前腿仍有效 → 产出路径（写入 _pending）。

        环游耗尽 → 重建（限制递归深度防闭环）。
        """
        # 高优先级指令态：打转后奔赴最近未知区（优先于环游）。
        # 目标是"最近的可达 frontier 自由格"——主动探索未探索区域。
        if self._seeking_unknown:
            goal = self._nearest_frontier_goal(grid, pose)
            if goal is not None:
                path = self._astar_path(grid, pose, goal)
                if path:
                    self._pending = (path, Viewpoint(cell=goal, cluster_id=-1,
                                                     gain=1e9, cost=0.0))
                    self.n_plans += 1
                    return
            # 无未知区 / 最近未知区不可达 → 退出指令态，交还环游与收敛判据
            self._seeking_unknown = False

        # 环游过期 → 重建
        if self._tour and self._frame - self._tour_frame > self.p.tour_refresh:
            self._tour = []

        while self._tour_i < len(self._tour):
            vp = self._tour[self._tour_i]
            if self._vp_valid(grid, vp):
                path = self._astar_path(grid, pose, vp.cell)
                if path:
                    self._pending = (path, vp)   # 由主线程收集应用
                    self.n_plans += 1
                    return
                # 不可达 → 拉黑该观察点（否则重建环游后反复重选，永不收敛）
                self._raise(Anomaly.LEG_UNREACHABLE, str(vp.cell), cell=vp.cell)
            log.debug("环游腿 %d 失效（不可达/黑名单/障碍）→ 跳过", self._tour_i)
            self._tour_i += 1

        # 环游耗尽 → 重建（限制递归深度防闭环）
        if _depth == 0:
            outcome = self._build_tour(grid, pose)
            if outcome in (PlanOutcome.DONE, PlanOutcome.BLANK,
                           PlanOutcome.STARVED, PlanOutcome.POSE_DRIFT):
                return   # 完成 / 探测期 / 冷却（帧基门控已由处理器设置）
            self._plan(grid, pose, _depth=1)
        else:
            # 腿全部失败：不判完成（黑名单已记录），冷却后重建重试；
            # 连续多次重建均无观察点时由 _build_tour 的收敛判据终止
            self._raise(Anomaly.VP_STARVATION, "重建后所有腿仍不可达")

    def _build_tour(self, grid: OccupancyGrid,
                    pose: tuple[float, float, float]) -> PlanOutcome:
        """重建环游：聚类 → 观察点 → 排序。所有"异常终止"路径都经由
        `_raise` 分发（可说明性：每种终止都有 Anomaly 记录可查）。"""
        self.blacklist.purge(self._frame)
        lo = grid._logodds
        clusters = find_clusters(
            lo, self.p.free_below, self.p.occupied_above, self.p.min_frontier_size
        )
        u_now = int(((~(lo <= self.p.free_below)) & (~(lo >= self.p.occupied_above))).sum())

        if not clusters:
            free_count = int((lo <= self.p.free_below).sum())
            if free_count < self.p.bootstrap_free_min:
                self._raise(Anomaly.BLANK_MAP, "自由格 %d" % free_count)
                return PlanOutcome.BLANK
            log.info("无 frontier → 建图完成（栅格更新 %d 次，规划 %d 次）",
                     grid.n_updates, self.n_plans)
            self.done = True
            self._tour = []
            return PlanOutcome.DONE

        # 收敛终止判据（docs §5.3）：跨位移的多次重建而 unknown 几乎不减
        if self.convergence.judge(pose, u_now):
            log.info("跨 %d 次环游重建无地图进展 → 建图完成（栅格更新 %d 次，规划 %d 次）",
                     self.p.stall_builds, grid.n_updates, self.n_plans)
            self.done = True
            self._tour = []
            return PlanOutcome.DONE

        snap = MapSnapshot.capture(grid, self.p, pose)
        if snap is None:
            self._raise(Anomaly.POSE_DRIFT, "(%.0f,%.0f)" % (pose[0], pose[1]))
            return PlanOutcome.POSE_DRIFT

        vps = self.planner.build_viewpoints(snap, clusters, self.blacklist)
        if not vps:
            # 观察点全被屏蔽 ≠ 建图完成：黑名单多来自卡死/临时不可达。
            # 只有连续多次重建都找不到观察点才判定收敛（真不可达 frontier）。
            self._vps_empty_streak += 1
            if self._vps_empty_streak >= 3:
                log.info("连续 %d 次重建均无有效观察点 → 建图完成（栅格更新 %d 次，规划 %d 次）",
                         self._vps_empty_streak, grid.n_updates, self.n_plans)
                self.done = True
                self._tour = []
                return PlanOutcome.DONE
            self._raise(Anomaly.VP_STARVATION,
                        "黑名单 %d 项（第 %d 次）" % (len(self.blacklist),
                                                      self._vps_empty_streak))
            return PlanOutcome.STARVED
        self._vps_empty_streak = 0

        self._tour = self.planner.order_tour(vps, snap.start, lo)
        self._tour_i = 0
        self._tour_frame = self._frame
        log.info("[tour] %d 簇 → %d 观察点环游：%s",
                 len(clusters), len(vps),
                 " → ".join(f"g{int(v.gain)}@{v.cell}" for v in self._tour[:6]))
        return PlanOutcome.TOUR

    def _vp_valid(self, grid: OccupancyGrid, vp: Viewpoint) -> bool:
        c, r = vp.cell
        if (c, r) in self.blacklist:
            return False
        lo = grid._logodds
        if not (0 <= c < lo.shape[1] and 0 <= r < lo.shape[0]):
            return False
        return lo[r, c] < self.p.occupied_above

    def _nearest_frontier_goal(self, grid: OccupancyGrid,
                               pose: tuple[float, float, float]) -> tuple[int, int] | None:
        """最近未知区硬目标：距机器人最近的可达"frontier 相邻自由格"。

        物理含义：未知区 = 阴影，frontier = 阴影与已照亮自由区的交界；
        主动探索 = 走到最近的阴影边缘去照亮它。返回 (col,row) 自由格；
        无未知区 / 位姿出界 / 不可达返回 None。
        """
        lo = grid._logodds
        free = lo <= self.p.free_below
        occ = lo >= self.p.occupied_above
        unknown = (~free) & (~occ)
        if not unknown.any():
            return None
        frontier = unknown & _dilate8(free)
        if not frontier.any():
            return None
        passable, _ = make_passable(lo, self.p.free_below,
                                    self.p.occupied_above, self.p.inflate_cells)
        start = grid.world_to_cell(pose[0], pose[1])
        if not grid.in_bounds(*start):
            return None
        sc, sr = start
        r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
        c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
        passable[r0:r1, c0:c1] |= free[r0:r1, c0:c1]
        adj_free = _dilate8(frontier) & free & passable
        if not adj_free.any():
            return None
        field = dijkstra_field(passable, unknown, start, self.p.unknown_cost)
        best: tuple[int, int] | None = None
        best_d = math.inf
        for r, c in np.argwhere(adj_free):
            d = float(field[r, c])
            if math.isfinite(d) and d < best_d:
                best_d = d
                best = (int(c), int(r))
        return best

    def _astar_path(self, grid: OccupancyGrid,
                    pose: tuple[float, float, float],
                    goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        passable, unknown = make_passable(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.inflate_cells
        )
        lo = grid._logodds
        free = lo <= self.p.free_below
        start = grid.world_to_cell(pose[0], pose[1])
        if grid.in_bounds(*start):
            # 起点逃逸气泡（同 MapSnapshot.capture）
            sc, sr = start
            r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
            c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
            passable[r0:r1, c0:c1] |= free[r0:r1, c0:c1]
        res = astar(passable, unknown, start, goal, unknown_cost=self.p.unknown_cost)
        return res.path or None

    # ---- 外部接口（与 v1 兼容） ----

    def revive(self) -> None:
        """供外部（如 HybridExplorer 兜底覆盖后）复活：重置规划状态，
        不清黑名单（TTL 自然过期，v1 语义）。"""
        self.done = False
        self.state = State.WAIT
        self._drop_target()
        self._tour = []
        self._tour_i = 0
        self._last_unknown = -1
        self._replan_cd = 0
        self.convergence.reset()
        self._probe_until = 0
        self._cooldown_until = 0
        self._vps_empty_streak = 0
        self._no_progress = 0
        self._last_cursor = -1
        self._theta_prev = None
        self._spin_abs = 0.0
        self._seeking_unknown = False

    def close(self) -> None:
        """停止后台规划线程（daemon 线程，进程退出时亦自动结束）。"""
        self._closed = True
        self._wakeup.set()


# 兼容别名：drop-in 替换 v1 的 TourExplorer（外部按名导入时可切换模块）。
TourExplorer = TourExplorerV2
