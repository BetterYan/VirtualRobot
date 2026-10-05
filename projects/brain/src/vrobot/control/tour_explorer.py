"""FUEL-lite 分层自主探索器（2D 简化版）。

对应行业分层规划家族（FUEL / TARE 的 2D 保守简化）：

    层1 全局调度（低频，事件触发）
        frontier 检测（复用 Yamauchi 语义）→ 聚类（保留簇成员格）
        → 每簇生成观察点：射线投影估计"到此能看见多少未知格"（信息增益）
        → 一次 Dijkstra 距离场取全部观察点的真实路径代价
        → score = gain / (λ + cost)
        → 最近邻（效用加权）+ 2-opt 构建环游，逐腿执行
    层2 局部执行（20Hz）
        纯追踪（pure pursuit）路径跟随：瞄准路径上 lookahead 前视点，
        w = 2·v·sin(α)/L，配指令速率限幅 —— 蛇形摆动的根治手段
        （旧实现追最近路点 + P 控制微分阻尼，是极限环补丁）
    层3 反应安全
        贴脸紧急转向 / 卡死逃逸（转向开阔侧后重规划）
        / 动态障碍路径失效检测（膨胀占据图逐帧校验剩余路径）
        / 黑名单 TTL（旧实现永不过期会误杀好边界）

异步约定：重规划为重操作（Dijkstra + 射线投影），只在事件触发时执行
（无路径 / 观察点失效 / 地图显著变化 / tour 过期），控制层始终 20Hz。
"""

from __future__ import annotations

import heapq
import logging
import math
import threading
from collections import deque
from dataclasses import dataclass

import numpy as np

from vrobot.planning.path_finder import astar, inflate, make_passable
from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid

log = logging.getLogger(__name__)


@dataclass
class TourParams:
    # ---- 运动 ----
    linear: float = 60.0            # 巡航速度 px/s（更高会劣化扫描匹配，建图反而更慢）
    linear_slow: float = 22.0       # 大航向误差 / 末段速度下限 px/s
    angular: float = 1.5            # 最大角速度 rad/s
    # ---- 栅格语义 ----
    free_below: float = -0.3
    occupied_above: float = 0.62
    inflate_cells: int = 2
    unknown_cost: float = 3.0       # A*/Dijkstra 穿越未知区的额外代价
    # ---- frontier 聚类 ----
    min_frontier_size: int = 4      # 忽略过小的未知碎片
    # ---- 观察点 / 信息增益 ----
    gain_beams: int = 48            # 每个候选观察点的射线数
    gain_range_cells: int = 40      # 增益射线最大步长（格，40 格 = 320px = 传感器量程）
    vp_radius: int = 8              # 观察点候选：簇成员周围该半径内的已确认自由格
    vp_step: int = 2                # 候选采样步距（格）
    vp_max_cands: int = 8           # 每簇最多评估的候选数（按距离场取最近的）
    vp_min_gain: int = 12           # 期望可见未知格低于此的观察点视为无价值
    vp_min_dist: int = 4            # 观察点距机器人当前位置的最小距离（格）
                                    # 防"脚下观察点"死锁：代价≈0 的近点会永远赢
    lambda_cost: float = 20.0       # score = gain / (lambda + path_cost)
    # ---- 环游 ----
    tour_refresh: int = 500         # 环游重建周期（帧，20Hz → 25s）
    replan_check: int = 50          # 地图变化检查周期（帧）
    map_change_frac: float = 0.03   # 未知格数变化比例 ≥ 此值 → 立即重建环游
    # ---- 纯追踪路径跟随 ----
    lookahead: float = 50.0         # 前视距离 px（≈ 车道中心线平滑尺度）
    w_rate_limit: float = 0.8       # 每帧角速度变化上限 rad（20Hz → 16 rad/s²）
    rotate_alpha: float = 1.2       # 航向误差超过此值 → 原地转向（v=0），防绕圈
    goal_tol: float = 14.0          # 观察点到达半径 px
    # ---- 安全 ----
    front_panic: float = 26.0       # 前方障碍紧急转向距离 px
    stuck_window: int = 60          # 卡死检测窗口（帧）
    stuck_move: float = 6.0         # 窗口内位移低于此视为卡死 px
    progress_timeout: int = 150     # 路径游标无进展帧数上限（防绕圈，位移检测抓不住）
    escape_frames: int = 25         # 卡死后原地转向开阔侧的帧数
    blacklist_ttl: int = 900        # 黑名单条目存活帧数（45s），过期自动解除
    max_blacklist: int = 24
    stall_builds: int = 10          # 连续重建环游次数达到该值且地图无进展 → 判定收敛
    stall_frac: float = 0.998       # unknown 降幅不足（当前/历史 ≥ 该值）视为无进展
    # ---- 空图引导 ----
    bootstrap_free_min: int = 100   # 已确认自由格低于此视为地图空白 → 直行探测而非宣布完成
    bootstrap_frames: int = 60      # 每轮直行探测帧数（3s，等 cartographer 桥接同步/观测积累）
    # ---- 异步规划 ----
    async_plan: bool = True         # 重规划放后台线程（否则 Dijkstra+射线投影 0.3-0.5s
                                    #   同步阻塞指令下发，Godot 0.5s 无指令即急停 → 走停）
    planner_poll: float = 0.01      # 规划线程轮询间隔 s


@dataclass
class _Cluster:
    cid: int
    cells: list[tuple[int, int]]    # 原始 frontier 格 (col, row)
    centroid: tuple[int, int]


@dataclass
class _Viewpoint:
    cell: tuple[int, int]           # 已确认自由格 (col, row)
    cluster_id: int
    gain: float                     # 射线投影估计的期望可见未知格数
    cost: float                     # 距离场路径代价


# ---------------------------------------------------------------------------
# frontier 聚类（与 explorer.find_frontiers 同语义，额外保留簇成员格）
# ---------------------------------------------------------------------------


def _dilate8(mask: np.ndarray) -> np.ndarray:
    """8 邻域膨胀（零填充，不做环形回绕——np.roll 会把边缘自由区
    绕到对侧制造假 frontier）。"""
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
) -> list[_Cluster]:
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
    clusters: list[_Cluster] = []
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
                _Cluster(cid=cid, cells=orig,
                         centroid=(int(arr[:, 0].mean()), int(arr[:, 1].mean())))
            )
            cid += 1
    return clusters


# ---------------------------------------------------------------------------
# 距离场（Dijkstra，4 邻接，未知区加价）
# ---------------------------------------------------------------------------


def dijkstra_field(
    passable: np.ndarray,
    unknown: np.ndarray,
    start: tuple[int, int],
    unknown_cost: float = 3.0,
) -> np.ndarray:
    """从 start 到全场的最短代价场 (H, W) float32；不可达 = inf。坐标 start=(col,row)。"""
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
# TourExplorer
# ---------------------------------------------------------------------------


class TourExplorer:
    """FUEL-lite 分层探索：聚类 → 观察点 → 环游 → 纯追踪，接口与旧探索器一致。"""

    def __init__(self, cell_size: float = 8.0, **kwargs):
        self.p = TourParams(**kwargs)
        self.cell_size = float(cell_size)
        self.done = False

        self._frame = 0
        self._path: list[tuple[int, int]] = []      # (col,row) 当前腿的格路径
        self._nearest_i = 0                          # 纯追踪最近路径点游标（只前进）
        self._w_prev = 0.0                           # 上一帧角速度（速率限幅）
        self._last_scan: LaserScan2D | None = None

        self._tour: list[_Viewpoint] = []            # 环游（剩余腿从 _tour_i 起）
        self._tour_i = 0
        self._tour_frame = -10**9
        self._target_vp: _Viewpoint | None = None
        self._blacklist: dict[tuple[int, int], int] = {}  # cell -> 过期帧

        self._replan_cd = 0
        self._last_unknown = -1
        self._stuck_hist: deque[tuple[float, float]] = deque(maxlen=self.p.stuck_window)
        self._escape_until = 0
        self._escape_sign = 1.0
        self._last_nearest = -1
        self._no_progress = 0
        self._stall_hist: list[tuple[float, float, int]] = []   # (x, y, unknown) 计入收敛统计的重建
        self._bootstrap_until = 0                    # 空图探测期截止帧

        # ---- 异步规划状态（单写者：_path/_nearest_i 只由主线程写） ----
        self._plan_lock = threading.Lock()
        self._plan_request: tuple | None = None      # (grid, pose) 待规划请求
        self._pending: tuple | None = None           # 后台规划产出 (path, vp)，主线程收集
        self._busy = False                           # 后台规划进行中
        self._closed = False
        self._wakeup = threading.Event()
        self._vps_empty_cd = 0       # 观察点全被屏蔽后的冷却（帧）
        self._vps_empty_streak = 0   # 连续"无有效观察点"的重建次数
        if self.p.async_plan:
            threading.Thread(target=self._plan_loop, daemon=True,
                             name="tour-planner").start()

        self.n_plans = 0                             # 诊断统计

    # ---- 主入口 ----

    def update(self, slam, scan: LaserScan2D) -> tuple[float, float]:
        """每帧调用（20Hz）：返回 (linear, angular) 指令。"""
        if self.done:
            return (0.0, 0.0)
        self._frame += 1
        self._last_scan = scan
        pose = slam.pose
        grid = slam.grid

        # 1) 卡死检测：窗口内几乎没动 → 逃逸（丢弃目标并列入黑名单 TTL）。
        #    规划等待期 / 观察点冷却期机器人按设计静止，不计入卡死。
        if self._busy or self._vps_empty_cd > 0:
            self._stuck_hist.clear()
        else:
            self._stuck_hist.append((pose[0], pose[1]))
            if len(self._stuck_hist) == self.p.stuck_window:
                pts = np.array(self._stuck_hist)
                if float(np.linalg.norm(pts[-1] - pts[0])) < self.p.stuck_move:
                    self._stuck_hist.clear()
                    self._on_stuck()

        # 2) 卡死逃逸：原地转向开阔侧，结束后路径已清空、下帧重规划
        if self._frame < self._escape_until:
            return (0.0, self.p.angular * self._escape_sign)

        # 2.5) 空图探测期：直行积累观测（cartographer 桥接同步前的空白地图阶段）
        if self._bootstrap_until > self._frame:
            v, w = self.p.linear, 0.0
        else:
            # 3) 地图显著变化检查（低频，只算一次 unknown 计数）
            self._replan_cd -= 1
            if self._replan_cd <= 0:
                self._replan_cd = self.p.replan_check
                lo = grid._logodds
                u = int(((~(lo <= self.p.free_below)) & (~(lo >= self.p.occupied_above))).sum())
                if self._last_unknown >= 0 and u > 0:
                    frac = abs(u - self._last_unknown) / u
                    if frac >= self.p.map_change_frac:
                        # 只重建环游，保持当前腿承诺（否则建图初期会高频打断执行）
                        log.debug("未知格变化 %.1f%% → 标记环游待重建", frac * 100)
                        self._tour = []
                        self._pending = None   # 后台可能产出自旧地图的过期路径
                self._last_unknown = u

            # 4) 动态障碍：剩余路径穿过新观测的膨胀占据区 → 立即失效
            if self._path and not self._path_valid(grid):
                log.debug("路径被新障碍截断 → 重规划")
                self._path = []

            # 5) 无路径 → 规划（事件触发，重操作；async 在后台线程执行，
            #    20Hz 控制不被 Dijkstra/射线投影阻塞 —— "走一段停一段"的根治）
            if not self._path:
                self._collect_plan()
                if self._vps_empty_cd > 0:
                    self._vps_empty_cd -= 1
                if self.done:
                    return (0.0, 0.0)
                if (not self._path and self._bootstrap_until <= self._frame
                        and self._vps_empty_cd <= 0):
                    if self.p.async_plan:
                        self._submit_plan(grid, pose)
                    else:
                        self._plan(grid, pose)
                        if self.done:
                            return (0.0, 0.0)

            # 6) 跟随 / 等待 / 探测
            if self._path:
                v, w = self._follow(pose)
            elif self._busy or self._vps_empty_cd > 0:
                v, w = 0.0, 0.0            # 后台规划中 / 观察点冷却：原地待命
            else:
                v, w = self.p.linear, 0.0  # 空图探测期：直行积累观测

            # 无进展看门狗：游标持续不前进（含绕圈——位移检测抓不住）→ 换目标
            if self._path:
                if self._nearest_i != self._last_nearest:
                    self._last_nearest = self._nearest_i
                    self._no_progress = 0
                else:
                    self._no_progress += 1
                if self._no_progress >= self.p.progress_timeout:
                    self._no_progress = 0
                    self._last_nearest = -1
                    self._drop_target("路径无进展（疑似绕圈/窄角）")
                    self._tour = []
                    self._escape_until = self._frame + self.p.escape_frames
                    self._escape_sign = self._avoid_sign(self._last_scan)
                    return (0.0, self.p.angular * self._escape_sign)
            else:
                self._last_nearest = -1
                self._no_progress = 0

        # 7) 反应式安全层：贴脸障碍原地转向（不前顶）
        d_front = self._front_distance(scan)
        if 0 < d_front < self.p.front_panic:
            v = 0.0
            w = self.p.angular * self._avoid_sign(scan)

        # 指令速率限幅：即使安全层/重规划打断，角速度也连续
        w = max(self._w_prev - self.p.w_rate_limit,
                min(self._w_prev + self.p.w_rate_limit, w))
        self._w_prev = w
        return (v, w)

    def revive(self) -> None:
        """供 HybridExplorer 在兜底覆盖后复活（重置规划状态，不清黑名单）。"""
        self.done = False
        self._path = []
        self._tour = []
        self._tour_i = 0
        self._target_vp = None
        self._last_unknown = -1
        self._replan_cd = 0
        self._stall_hist = []
        self._bootstrap_until = 0
        self._pending = None
        self._vps_empty_cd = 0
        self._vps_empty_streak = 0
        self._plan_cd = 0

    def close(self) -> None:
        """停止后台规划线程（daemon 线程，进程退出时亦自动结束）。"""
        self._closed = True
        self._wakeup.set()

    # ---- 后台规划（异步模式） ----

    def _submit_plan(self, grid: OccupancyGrid, pose: tuple[float, float, float]) -> None:
        if self._busy or self._closed:
            return
        self._busy = True
        with self._plan_lock:
            self._plan_request = (grid, (pose[0], pose[1], pose[2]))
        self._wakeup.set()

    def _collect_plan(self) -> None:
        """主线程收集后台产出（_path/_nearest_i 只由主线程写）。"""
        if self._pending is None:
            return
        path, vp = self._pending
        self._pending = None
        self._path = path
        self._nearest_i = 0
        self._target_vp = vp
        self.n_plans += 1

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
                self._plan(grid, pose, defer=True)
            except Exception:
                log.exception("后台规划异常")
            self._busy = False

    # ---- 规划（层1） ----

    def _plan(self, grid: OccupancyGrid, pose: tuple[float, float, float],
              _depth: int = 0, defer: bool = False) -> None:
        # 环游过期 → 重建
        if self._tour and self._frame - self._tour_frame > self.p.tour_refresh:
            self._tour = []
        # 沿环游推进：当前腿仍有效 → 规划路径
        while self._tour_i < len(self._tour):
            vp = self._tour[self._tour_i]
            if self._vp_valid(grid, vp):
                path = self._astar_path(grid, pose, vp.cell)
                if path:
                    self._target_vp = vp
                    if defer:
                        self._pending = (path, vp)   # 后台模式：由主线程收集应用
                    else:
                        self._path = path
                        self._nearest_i = 0
                    self.n_plans += 1
                    return
                # 不可达 → 拉黑该观察点邻域（否则重建环游后反复重选，永不收敛）
                self._blacklist_cell(vp.cell, grid.height, grid.width)
            log.debug("环游腿 %d 失效（不可达/黑名单/障碍）→ 跳过", self._tour_i)
            self._tour_i += 1
        # 环游耗尽 → 重建（限制递归深度防闭环）
        if _depth == 0:
            self._build_tour(grid, pose)
            if self.done or self._bootstrap_until > self._frame:
                return   # 完成或进入空图探测期（探测结束后自然回到这里）
            self._plan(grid, pose, _depth=1)
        else:
            # 腿全部 astar 失败：不判完成（黑名单已记录），冷却后重建重试；
            # 连续多次重建均无观察点时由 _build_tour 的收敛判据终止
            self._vps_empty_cd = max(self._vps_empty_cd, 150)

    def _build_tour(self, grid: OccupancyGrid, pose: tuple[float, float, float]) -> None:
        self._purge_blacklist()
        lo = grid._logodds
        clusters = find_clusters(
            lo, self.p.free_below, self.p.occupied_above, self.p.min_frontier_size
        )
        if not clusters:
            free_count = int((lo <= self.p.free_below).sum())
            if free_count < self.p.bootstrap_free_min:
                # 地图空白（cartographer 桥接未同步/首帧未入图）：空白图上
                # 定义上不存在 frontier，宣布完成是错的 → 直行探测积累观测
                log.info("地图空白（自由格 %d）→ 直行探测，待观测积累后重新规划", free_count)
                self._bootstrap_until = self._frame + self.p.bootstrap_frames
                self._tour = []
                return
            log.info("无 frontier → 建图完成（栅格更新 %d 次，规划 %d 次）",
                     grid.n_updates, self.n_plans)
            self.done = True
            self._tour = []
            return

        # 收敛终止判据：跨越实际位移的多次环游重建而 unknown 几乎不减 →
        # 剩余边界均为不可解碎片。原地搅动型重建（零位移）不计入，
        # 否则局部卡位会被误判为全局收敛
        u_now = int(((~(lo <= self.p.free_below)) & (~(lo >= self.p.occupied_above))).sum())
        px, py = pose[0], pose[1]
        moved = (self._stall_hist
                 and math.hypot(px - self._stall_hist[-1][0], py - self._stall_hist[-1][1]) >= 30.0)
        progressed = (self._stall_hist and u_now < self._stall_hist[-1][2] * self.p.stall_frac)
        if not self._stall_hist or moved or progressed:
            self._stall_hist.append((px, py, u_now))
            if len(self._stall_hist) > self.p.stall_builds:
                self._stall_hist.pop(0)
        if (len(self._stall_hist) >= self.p.stall_builds
                and u_now >= self._stall_hist[0][2] * self.p.stall_frac):
            log.info("跨 %d 次环游重建无地图进展 → 建图完成（栅格更新 %d 次，规划 %d 次）",
                     self.p.stall_builds, grid.n_updates, self.n_plans)
            self.done = True
            self._tour = []
            return

        passable, unknown = make_passable(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.inflate_cells
        )
        lo = grid._logodds
        free = lo <= self.p.free_below
        start = grid.world_to_cell(pose[0], pose[1])
        if not grid.in_bounds(*start):
            # 估计位姿飘出栅格（cartographer 快速旋转/长直线下漂移发散）：
            # 距离场起点无效会导致全场 inf、观察点全部消失 —— 原地冷却等待
            # 位姿恢复（pose graph 优化/新观测），不在此状态下做任何规划
            log.warning("估计位姿 (%.0f,%.0f) 超出栅格边界 → 冷却等待，暂停规划", pose[0], pose[1])
            self._vps_empty_cd = 150
            self._tour = []
            return
        self._carve_bubble(passable, free, start)
        field = dijkstra_field(passable, unknown, start, self.p.unknown_cost)

        occ_mask = lo >= self.p.occupied_above
        unknown_mask = (~free) & (~occ_mask)
        vps: list[_Viewpoint] = []
        for min_dist in (self.p.vp_min_dist, 0):
            for cl in clusters:
                vp = self._best_viewpoint(
                    grid, cl, free, occ_mask, unknown_mask, passable, field, start, min_dist
                )
                if vp is not None:
                    vps.append(vp)
            if vps:
                break   # 优先取"距机器人 ≥ min_dist"的观察点；全空再无距离约束兜底
        if not vps:
            # 观察点全被屏蔽 ≠ 建图完成：黑名单多来自卡死/临时不可达（TTL 会解除）。
            # 只有连续多次重建都找不到观察点才判定收敛（真不可达 frontier）。
            self._vps_empty_streak += 1
            if self._vps_empty_streak >= 3:
                log.info("连续 %d 次重建均无有效观察点 → 建图完成（栅格更新 %d 次，规划 %d 次）",
                         self._vps_empty_streak, grid.n_updates, self.n_plans)
                self.done = True
                self._tour = []
                return
            log.info("观察点暂不可用（黑名单 %d 项）→ 冷却 %d 帧后重试（第 %d 次）",
                     len(self._blacklist), 150, self._vps_empty_streak)
            self._vps_empty_cd = 150
            self._tour = []
            return
        self._vps_empty_streak = 0

        self._tour = self._order_tour(vps, start, lo)
        self._tour_i = 0
        self._tour_frame = self._frame
        log.info("[tour] %d 簇 → %d 观察点环游：%s",
                 len(clusters), len(vps),
                 " → ".join(f"g{int(v.gain)}@{v.cell}" for v in self._tour[:6]))

    def _best_viewpoint(
        self,
        grid: OccupancyGrid,
        cluster: _Cluster,
        free: np.ndarray,
        occ: np.ndarray,
        unknown: np.ndarray,
        passable: np.ndarray,
        field: np.ndarray,
        start: tuple[int, int],
        min_dist: int,
    ) -> _Viewpoint | None:
        """簇周围采样已确认自由格为候选，射线投影估增益，取 gain/(λ+cost) 最优。"""
        h, w = passable.shape
        R, step = self.p.vp_radius, self.p.vp_step
        md2 = float(min_dist * min_dist)
        cands: set[tuple[int, int]] = set()
        for c, r in cluster.cells[::2]:
            c0, c1 = max(0, c - R), min(w, c + R + 1)
            r0, r1 = max(0, r - R), min(h, r + R + 1)
            for nr in range(r0, r1, step):
                for nc in range(c0, c1, step):
                    if not (free[nr, nc] and passable[nr, nc] and np.isfinite(field[nr, nc])):
                        continue
                    if min_dist > 0:
                        dc, dr = nc - start[0], nr - start[1]
                        if dc * dc + dr * dr < md2:
                            continue
                    cands.add((nc, nr))
        if not cands:
            return None
        ordered = sorted(cands, key=lambda t: field[t[1], t[0]])[: self.p.vp_max_cands]
        best: tuple[float, _Viewpoint] | None = None
        for cell in ordered:
            if cell in self._blacklist:
                continue
            cost = float(field[cell[1], cell[0]])
            gain = self._ray_gain(cell, occ, unknown)
            if gain < self.p.vp_min_gain:
                continue
            s = gain / (self.p.lambda_cost + cost)
            if best is None or s > best[0]:
                best = (s, _Viewpoint(cell=cell, cluster_id=cluster.cid, gain=gain, cost=cost))
        return best[1] if best else None

    def _ray_gain(self, cell: tuple[int, int], occ: np.ndarray, unknown: np.ndarray) -> float:
        """从观察格向 gain_beams 个方向步进，数可见未知格（撞占据即断）。"""
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

    def _order_tour(
        self, vps: list[_Viewpoint], start: tuple[int, int], lo: np.ndarray
    ) -> list[_Viewpoint]:
        """效用加权最近邻 + 2-opt。

        腿间代价用"欧氏 × 线段穿越惩罚"近似（避免 N² 次 Dijkstra）：
        线段穿占据格 ×4，穿未知格 ×1.5。
        """
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
                if v >= self.p.occupied_above:
                    pen *= 1.02
                elif v > self.p.free_below:
                    pen *= 1.01
            return d * pen

        # 最近邻（效用加权：score = gain/(λ+est)）
        remaining = list(vps)
        order: list[_Viewpoint] = []
        cur = start
        while remaining:
            best_i, best_s = 0, -1.0
            for i, vp in enumerate(remaining):
                s = vp.gain / (self.p.lambda_cost + leg_cost(cur, vp.cell))
                if s > best_s:
                    best_i, best_s = i, s
            vp = remaining.pop(best_i)
            order.append(vp)
            cur = vp.cell

        # 2-opt 改善总腿长（标准 delta：反转 [i, j-1] 只改变 a→b 与 c→d 两条边，
        # d→e 不受影响 —— delta 口径不一致会造成假改进 → 无界震荡死循环）
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

    # ---- 执行（层2：纯追踪） ----

    def _follow(self, pose: tuple[float, float, float]) -> tuple[float, float]:
        cs = self.cell_size
        pts = [((c + 0.5) * cs, (r + 0.5) * cs) for c, r in self._path]
        x, y, th = pose[0], pose[1], pose[2]

        # 到达当前腿终点 → 推进环游并清空路径，下帧规划下一腿
        d_goal = math.hypot(pts[-1][0] - x, pts[-1][1] - y)
        if d_goal < self.p.goal_tol:
            if (self._target_vp is not None and self._tour_i < len(self._tour)
                    and self._tour[self._tour_i] is self._target_vp):
                self._tour_i += 1   # 该腿完成（若环游已重建则不推进，由 _plan 重裁决）
            self._path = []
            self._nearest_i = 0
            self._target_vp = None
            return (0.0, 0.0)

        # 最近路径点游标（只前进，吸收走过路点）
        i = self._nearest_i
        while i + 1 < len(pts) and math.hypot(pts[i][0] - x, pts[i][1] - y) < self.p.goal_tol:
            i += 1
        self._nearest_i = i

        d_goal = math.hypot(pts[-1][0] - x, pts[-1][1] - y)
        L = self.p.lookahead
        if d_goal < L:
            # 末段：直接瞄准终点（自然收敛，无绕弧）
            tx, ty = pts[-1]
        else:
            # 从最近点沿路径累计弧长找前视点
            j, acc = i, 0.0
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

        L_eff = max(math.hypot(tx - x, ty - y), 1.0)
        desired = math.atan2(ty - y, tx - x)
        alpha = (desired - th + math.pi) % (2.0 * math.pi) - math.pi

        # 纯追踪转向律 + 连续调速
        w_raw = 2.0 * self.p.linear * math.sin(alpha) / L_eff
        w_raw = max(-self.p.angular, min(self.p.angular, w_raw))

        # 先转向后行进：大航向误差下前进会绕着目标打转（差速底盘标准做法）
        if abs(alpha) > self.p.rotate_alpha:
            return (0.0, w_raw)

        v = self.p.linear * max(self.p.linear_slow / self.p.linear, 1.0 - 1.1 * abs(alpha))
        if d_goal < 40.0:
            v = min(v, max(self.p.linear_slow, d_goal * 1.2))
        return (v, w_raw)

    # ---- 安全与状态 ----

    def _blacklist_cell(self, cell: tuple[int, int], grid_h: int, grid_w: int) -> None:
        """拉黑单个观察点格（3×3 块拉黑会封锁过多合法观察点，造成观察点饥荒）。"""
        del grid_h, grid_w
        if len(self._blacklist) < self.p.max_blacklist:
            self._blacklist[cell] = self._frame + self.p.blacklist_ttl

    def _on_stuck(self) -> None:
        # 卡死只做逃逸 + 重规划，不拉黑目标：机器人静止可能是外部原因
        # （Godot manual / 规划等待 / 临时阻挡），拉黑会误杀好观察点并
        # 连锁触发"均无有效观察点 → 建图完成"的过早终止。
        self._target_vp = None
        self._path = []
        self._pending = None
        self._tour = []
        self._escape_until = self._frame + self.p.escape_frames
        self._escape_sign = self._avoid_sign(self._last_scan) if self._last_scan else 1.0

    def _drop_target(self, reason: str) -> None:
        if self._target_vp is not None:
            self._blacklist_cell(self._target_vp.cell, 10**6, 10**6)
        log.debug("%s：放弃当前观察点", reason)
        self._target_vp = None
        self._path = []
        self._pending = None

    def _vp_valid(self, grid: OccupancyGrid, vp: _Viewpoint) -> bool:
        c, r = vp.cell
        if (c, r) in self._blacklist:
            return False
        lo = grid._logodds
        if not (0 <= c < lo.shape[1] and 0 <= r < lo.shape[0]):
            return False
        return lo[r, c] < self.p.occupied_above

    def _path_valid(self, grid: OccupancyGrid) -> bool:
        """剩余路径逐 4 格采样，不得落入膨胀占据区（动态障碍即时响应）。"""
        lo = grid._logodds
        blocked = inflate(lo >= self.p.occupied_above, 1)
        h, w = blocked.shape
        for c, r in self._path[max(self._nearest_i, 0)::4]:
            if 0 <= c < w and 0 <= r < h and blocked[r, c]:
                return False
        return True

    def _carve_bubble(
        self, passable: np.ndarray, free: np.ndarray, start: tuple[int, int]
    ) -> None:
        """起点逃逸气泡：机器人已物理存在于该处，邻域 3 格内的自由区允许通行。"""
        sc, sr = start
        r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
        c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
        passable[r0:r1, c0:c1] |= free[r0:r1, c0:c1]

    def _astar_path(
        self, grid: OccupancyGrid, pose: tuple[float, float, float], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        passable, unknown = make_passable(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.inflate_cells
        )
        lo = grid._logodds
        free = lo <= self.p.free_below
        start = grid.world_to_cell(pose[0], pose[1])
        self._carve_bubble(passable, free, start)
        res = astar(passable, unknown, start, goal, unknown_cost=self.p.unknown_cost)
        return res.path or None

    def _purge_blacklist(self) -> None:
        self._blacklist = {
            c: exp for c, exp in self._blacklist.items() if exp > self._frame
        }

    # ---- 传感器辅助 ----

    @staticmethod
    def _front_distance(scan: LaserScan2D | None) -> float:
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
    def _avoid_sign(scan: LaserScan2D | None) -> float:
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
