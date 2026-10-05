"""混合探索器：沿边探测 → 弓形覆盖 → Frontier 收尾。

行业两阶段建图与本项目的结合（对应 docs/04 第 10 节）：

    seek_wall     起始直行，直到雷达前方发现边界（墙面）
    wall_follow   右手沿边探测：保持侧向距离贴边走；
                  轨迹闭环（回到起点且弧长足够）后进入覆盖
    coverage      弓形（boustrophedon）覆盖：对"已确认自由 ∧ 未扫过"的
                  区域按行生成车道，贪心最近端点排序，A* 逐段导航
    frontier      委托 FrontierExplorer 驶向未知边界（标准 SLAM→frontier
                  →导航 闭环）；途中发现大面积新自由区时交还 coverage
    done          无可达 frontier 且无未覆盖自由区

swept 掩码：以机器人历史位置为圆心、sweep_radius 格为半径的圆盘。
"新自由区" = 当前确认自由 ∧ 未扫过 —— 驱动 coverage↔frontier 交替，
直到地图收敛。
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from vrobot.control.explorer import FrontierExplorer
from vrobot.planning.path_finder import astar, inflate, make_passable
from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid

log = logging.getLogger(__name__)

SEEK_WALL = "seek_wall"
WALL_FOLLOW = "wall_follow"
COVERAGE = "coverage"
FRONTIER = "frontier"


@dataclass
class HybridParams:
    # 运动
    linear: float = 60.0
    linear_slow: float = 22.0
    angular: float = 1.5
    # 栅格语义（与 FrontierExplorer 一致）
    free_below: float = -0.3
    occupied_above: float = 0.62
    inflate_cells: int = 2
    # ---- 沿边阶段 ----
    wall_detect: float = 70.0     # 前方距离小于此 → 找到边界（px）
    side_target: float = 45.0     # 沿边目标侧向距离（px）
    side_lost: float = 110.0      # 侧向距离大于此 → 沿边丢失，转向找回（px）
    corner_ratio: float = 0.85    # 转角触发：d_front < corner_ratio × 当前墙垂距（自适应，
                                  # 替代固定 front_turn——固定 48px 会与 45px 目标侧距冲突，
                                  # 平行跟随时前方锥内读数 ≈45-47px 恒触发 → 蛇形摆动）
    side_gain: float = 0.03       # 侧距误差 → 角速度增益（1/px），PD 的 P 项
    align_gain: float = 3.0       # 墙-航向偏角 → 角速度增益（1/rad），PD 的 D/对齐项
    smooth: float = 0.25          # 侧距/偏角 EMA 平滑系数（新样本权重）
    alpha_clamp: float = 0.5      # 墙-航向偏角可信上限（rad），超过视为跨角假值清零
    loop_radius: float = 40.0     # 闭环判定：回到沿边起点该半径内（px）
    min_loop_len: float = 500.0   # 闭环判定：沿边弧长下限（px），防"刚起步就算闭环"
    follow_timeout: int = 4500    # 沿边超时（帧），到时强制进入覆盖
    # ---- 弓形覆盖阶段 ----
    lane_pitch: int = 4           # 车道行距（格，4 格 = 32px）
    min_run: int = 4              # 车道最小长度（格，4 格 = 32px）
    goal_tol: float = 14.0        # 路点到达半径（px）
    # ---- 阶段切换 ----
    sweep_radius: int = 20        # swept 圆盘半径（格，20 格 = 160px）
    new_area_min: int = 60        # 新自由区超过该格数 → 交还 coverage
    final_cov_budget: int = 2     # frontier 全不可达时，强制兜底弓形覆盖的轮数上限
    final_cov_min: int = 30       # 触发兜底覆盖的最小未扫格数
    frontier_check_every: int = 100   # frontier 阶段检查新区的间隔（帧）
    cov_cooldown: int = 200       # coverage 空手而归后，禁止立刻再进 coverage（帧）
    # ---- frontier 阶段后端 ----
    frontier_mode: str = "tour"   # "tour"=FUEL-lite（默认）| "classic"=旧 FrontierExplorer
    async_plan: bool = True       # tour 后端的后台规划线程（防指令阻塞走停）
    # ---- 安全 ----
    front_panic: float = 26.0     # 前方紧急转向距离（px）
    stuck_window: int = 50
    stuck_move: float = 6.0


class HybridExplorer:
    """三阶段混合自主探索：沿边 → 弓形 → frontier，接口与 FrontierExplorer 一致。"""

    def __init__(self, cell_size: float = 8.0, **kwargs):
        self.p = HybridParams(**kwargs)
        self.cell_size = float(cell_size)
        self.done = False
        self.phase = SEEK_WALL
        self.visited_phases: list[str] = [SEEK_WALL]

        frontier_kwargs = dict(
            cell_size=cell_size,
            linear=self.p.linear,
            linear_slow=self.p.linear_slow,
            angular=self.p.angular,
            free_below=self.p.free_below,
            occupied_above=self.p.occupied_above,
            inflate_cells=self.p.inflate_cells,
        )
        if self.p.frontier_mode == "tour":
            from vrobot.control.tour_explorer import TourExplorer
            self._frontier_ex: TourExplorer | FrontierExplorer = TourExplorer(
                **frontier_kwargs, async_plan=self.p.async_plan
            )
        else:
            self._frontier_ex = FrontierExplorer(**frontier_kwargs)

        self._swept: np.ndarray | None = None       # (h,w) bool，延迟到首帧建
        self._disk = self._make_disk(self.p.sweep_radius)

        # 沿边状态
        self._follow_start: tuple[float, float] | None = None
        self._follow_arc = 0.0
        self._follow_prev: tuple[float, float] | None = None
        self._follow_frames = 0
        self._prev_diff = 0.0                        # 上一帧航向误差（微分阻尼用）
        self._d_s: float | None = None    # 平滑后墙垂距
        self._a_s: float | None = None    # 平滑后墙-航向偏角

        # 覆盖状态
        self._cov_wps: list[tuple[int, int]] = []   # (col,row) 车道端点序列
        self._cov_i = 0
        self._cov_path: list[tuple[int, int]] = []
        self._cov_path_wp: tuple[int, int] | None = None
        self._cov_fails: dict[tuple[int, int], int] = {}

        # 通用
        self._stuck_hist: deque[tuple[float, float]] = deque(maxlen=self.p.stuck_window)
        self._front_check_cd = 0
        self._cov_cd = 0
        self._final_cov_budget = int(self.p.final_cov_budget)
        self._seg_replan_cd = 0

    # ---- 主入口 ----

    def update(self, slam, scan: LaserScan2D) -> tuple[float, float]:
        if self.done:
            return (0.0, 0.0)
        pose = slam.pose
        self._mark_swept(slam.grid, pose)

        # 卡死检测（所有阶段共用）
        self._stuck_hist.append((pose[0], pose[1]))
        if len(self._stuck_hist) == self.p.stuck_window:
            pts = np.array(self._stuck_hist)
            if float(np.linalg.norm(pts[-1] - pts[0])) < self.p.stuck_move:
                self._stuck_hist.clear()
                self._on_stuck()

        if self._cov_cd > 0:
            self._cov_cd -= 1

        if self.phase == SEEK_WALL:
            cmd = self._update_seek(pose, scan)
        elif self.phase == WALL_FOLLOW:
            cmd = self._update_follow(pose, scan)
        elif self.phase == COVERAGE:
            cmd = self._update_coverage(slam, pose, scan)
        else:
            cmd = self._update_frontier(slam, scan)

        # 反应式安全层：贴脸障碍原地转向（不前顶）。
        # wall_follow 豁免——其自适应转角逻辑已覆盖近障场景，
        # 通用避障"转向更开阔侧"会在凸角处与右手沿边方向相反，导致折返摆动。
        if self.phase in (SEEK_WALL, COVERAGE):
            d_front = self._front_distance(scan)
            if 0 < d_front < self.p.front_panic:
                cmd = (0.0, self.p.angular * self._avoid_sign(scan))
        return cmd

    def _on_stuck(self) -> None:
        if self.phase == WALL_FOLLOW:
            log.info("沿边卡死 → 提前进入弓形覆盖")
            self._enter(COVERAGE)
        elif self.phase == COVERAGE:
            # 放弃当前路点，跳到下一个
            wp = self._current_wp()
            if wp is not None:
                self._cov_fails[wp] = self._cov_fails.get(wp, 0) + 1
            self._cov_path = []
            self._cov_path_wp = None
            self._advance_wp()

    # ---- 阶段 1：直行找边界 ----

    def _update_seek(self, pose, scan: LaserScan2D) -> tuple[float, float]:
        d = self._front_distance(scan)
        if 0 < d < self.p.wall_detect:
            log.info("[seek_wall] 前方 %.0fpx 发现边界 → 沿边探测（右手）", d)
            self._follow_start = (pose[0], pose[1])
            self._follow_prev = (pose[0], pose[1])
            self._follow_arc = 0.0
            self._follow_frames = 0
            self._enter(WALL_FOLLOW)
            return (0.0, 0.0)
        return (self.p.linear, 0.0)

    # ---- 阶段 2：右手沿边 ----

    def _update_follow(self, pose, scan: LaserScan2D) -> tuple[float, float]:
        self._follow_frames += 1
        x, y = pose[0], pose[1]
        self._follow_arc += float(np.hypot(x - self._follow_prev[0], y - self._follow_prev[1]))
        self._follow_prev = (x, y)

        # 闭环判定：弧长足够 + 回到起点附近
        if self._follow_arc >= self.p.min_loop_len and self._follow_start is not None:
            d0 = float(np.hypot(x - self._follow_start[0], y - self._follow_start[1]))
            if d0 < self.p.loop_radius:
                log.info("[wall_follow] 弧长 %.0fpx，闭环（距起点 %.0fpx）→ 弓形覆盖",
                         self._follow_arc, d0)
                self._enter(COVERAGE)
                return (0.0, 0.0)
        if self._follow_frames >= self.p.follow_timeout:
            log.info("[wall_follow] 超时 %d 帧（弧长 %.0fpx）→ 弓形覆盖",
                     self._follow_frames, self._follow_arc)
            self._enter(COVERAGE)
            return (0.0, 0.0)

        # 前方转角判定（自适应）：d_front < corner_ratio × 参考距离。
        # 参考距离取当前所跟墙的垂距——平行贴墙跟随（d≈45px）时前方锥内波束
        # 读数 ≈45-47px，若用固定阈值 48px 会恒触发原地转角 → 蛇形摆动极限环；
        # 只有前方显著贴近（真转角/障碍）才触发。正转让前墙（相对角 0°）
        # 变为 -90° 侧的新跟踪墙。
        d_front = self._front_distance(scan)
        d_ref = self._d_s if (self._d_s is not None and self._d_s > 0) else self.p.side_target
        if 0 < d_front < self.p.corner_ratio * d_ref:
            return (0.0, self.p.angular * 0.8)

        # 双波束墙线拟合 → PD 控制（纯 P 距离环无阻尼会蛇形摆动，
        # 偏角项同时承担"航向对齐"与"微分阻尼"）
        fit = self._wall_fit(scan)
        if fit is None or not (0 < fit[0] < self.p.side_lost):
            # 沿边丢失（门口/开阔区/拟合失效）：慢速右转找回
            self._d_s = self._a_s = None
            return (self.p.linear_slow, -0.5 * self.p.angular)
        d_raw, a_raw = fit
        # 偏角可靠性钳制：两波束跨越墙角/家具角时拟合出的 α 是假值（可达 ±90°），
        # 直进控制量会造成甩头；|α| 过大时视为不可靠，只保留垂距项
        if abs(a_raw) > self.p.alpha_clamp:
            a_raw = 0.0
            d_raw = min(d_raw, self.p.side_target)  # 贴太近时保守推远
        # EMA 平滑：抑制雷达噪声与跨墙角读数跳变
        self._d_s = d_raw if self._d_s is None else (1 - self.p.smooth) * self._d_s + self.p.smooth * d_raw
        self._a_s = a_raw if self._a_s is None else (1 - self.p.smooth) * self._a_s + self.p.smooth * a_raw

        err = self._d_s - self.p.side_target           # >0 离墙太远
        w = -self.p.side_gain * err + self.p.align_gain * self._a_s
        w = max(-self.p.angular, min(self.p.angular, w))
        # 连续调速：转向越大速度越低（替代二值死区，避免 0.5 阈值附近忽快忽慢）
        v = self.p.linear * max(0.3, 1.0 - 0.7 * abs(w) / self.p.angular)
        return (v, w)

    # ---- 阶段 3：弓形覆盖 ----

    def _update_coverage(self, slam, pose, scan) -> tuple[float, float]:
        # 尚无车道规划 → 生成弓形车道
        if not self._cov_wps:
            if not self._plan_coverage(slam, pose):
                self._enter_frontier(slam.grid, "无可覆盖区域")
                return (0.0, 0.0)
        # 当前段走完或无路径 → 规划下一段
        if not self._cov_path:
            if self._cov_i >= len(self._cov_wps):
                self._enter_frontier(slam.grid, "车道耗尽")
                return (0.0, 0.0)
            wp = self._cov_wps[self._cov_i]
            if self._cov_fails.get(wp, 0) >= 2:
                self._advance_wp()
                return (0.0, 0.0)
            path = self._plan_to(slam.grid, pose, wp)
            if path is None:
                log.debug("[coverage] 车道点 %s 不可达，跳过", wp)
                self._cov_fails[wp] = self._cov_fails.get(wp, 0) + 1
                self._advance_wp()
                return (0.0, 0.0)
            self._cov_path = path
            self._cov_path_wp = wp
            self._seg_replan_cd = 30
            log.info("[coverage] 车道 %d/%d → %s", self._cov_i // 2 + 1,
                     max(len(self._cov_wps) // 2, 1), wp)
        else:
            # 周期性重规划当前段：扫描持续更新地图，旧路径可能穿过
            # 新标记的障碍/膨胀区，贴旧路径走会撞墙
            self._seg_replan_cd -= 1
            if self._seg_replan_cd <= 0:
                self._seg_replan_cd = 30
                wp = self._cov_wps[self._cov_i]
                path = self._plan_to(slam.grid, pose, wp)
                if path:
                    self._cov_path = path

        # 路点跟随
        return self._follow_path(pose)

    def _advance_wp(self) -> None:
        self._cov_i += 1
        self._cov_path = []
        self._cov_path_wp = None

    def _current_wp(self) -> tuple[int, int] | None:
        if 0 <= self._cov_i < len(self._cov_wps):
            return self._cov_wps[self._cov_i]
        return None

    def _enter_frontier(self, grid: OccupancyGrid, reason: str) -> None:
        log.info("[coverage] %s（新增未扫自由格 %d）→ frontier 阶段",
                 reason, self._new_free_cells(grid))
        self._cov_cd = self.p.cov_cooldown
        self._cov_wps = []
        self._enter(FRONTIER)

    def _plan_coverage(self, slam, pose) -> bool:
        """生成弓形车道并排序；无车道返回 False。"""
        grid = slam.grid
        lanes = self._generate_lanes(grid)
        if not lanes:
            return False
        start = grid.world_to_cell(pose[0], pose[1])
        cur = start
        wps: list[tuple[int, int]] = []
        remaining = list(lanes)
        while remaining:
            best_i, best_d, best_pair = -1, float("inf"), None
            for i, (r, c0, c1) in enumerate(remaining):
                for entry, exit in (((c0, r), (c1, r)), ((c1, r), (c0, r))):
                    d = abs(entry[0] - cur[0]) + abs(entry[1] - cur[1])
                    if d < best_d:
                        best_i, best_d, best_pair = i, d, (entry, exit)
            entry, exit = best_pair
            wps.extend((entry, exit))
            cur = exit
            remaining.pop(best_i)
        self._cov_wps = wps
        self._cov_i = 0
        self._cov_path = []
        self._cov_path_wp = None
        self._cov_fails.clear()
        log.info("[coverage] 规划 %d 条车道 / %d 个路点", len(lanes), len(wps))
        return True

    def _generate_lanes(self, grid: OccupancyGrid) -> list[tuple[int, int, int]]:
        """对"确认自由 ∧ 非膨胀障碍 ∧ 未扫过"区域按行生成水平车道 (row, c0, c1)。

        行距 lane_pitch 稀疏化：雷达半径 320px 远大于行距，细缝遗漏交给
        frontier 阶段兜底。
        """
        lo = grid._logodds
        free = lo <= self.p.free_below
        occupied = lo >= self.p.occupied_above
        blocked = inflate(occupied, self.p.inflate_cells)
        target = free & ~blocked & ~self._swept
        h, w = target.shape
        lanes: list[tuple[int, int, int]] = []
        last_row = -10 ** 9
        for r in range(h):
            row = target[r]
            c = 0
            while c < w:
                if row[c]:
                    c0 = c
                    while c < w and row[c]:
                        c += 1
                    if (c - c0) >= self.p.min_run and (r - last_row) >= self.p.lane_pitch:
                        lanes.append((r, c0, c - 1))
                        last_row = r
                else:
                    c += 1
        return lanes

    # ---- 阶段 4：frontier（委托） ----

    def _update_frontier(self, slam, scan) -> tuple[float, float]:
        cmd = self._frontier_ex.update(slam, scan)
        if self._frontier_ex.done:
            # frontier 全部消失或均不可达：若仍有未扫自由区，强制兜底弓形覆盖
            # （否则 60 格级的剩余碎片既够不着阈值又被 A* 判不可达 → 过早 done）。
            # 兜底轮扫描会更新地图，frontier 需解除 done 重新规划。
            new_free = self._new_free_cells(slam.grid)
            if (self._final_cov_budget > 0 and self._cov_cd <= 0
                    and new_free >= self.p.final_cov_min):
                self._final_cov_budget -= 1
                if self._plan_coverage(slam, slam.pose):
                    log.info("[frontier] frontier 不可达但未扫区 %d 格 → 兜底弓形覆盖（剩 %d 轮）",
                             new_free, self._final_cov_budget)
                    self._frontier_ex.done = False
                    if hasattr(self._frontier_ex, "revive"):
                        self._frontier_ex.revive()   # TourExplorer：重置规划状态
                    else:
                        self._frontier_ex._replan_countdown = 0
                    self._enter(COVERAGE)
                    return (0.0, 0.0)
            log.info("[frontier] 无可达 frontier，且无未扫自由区 → 建图完成")
            self.done = True
            return (0.0, 0.0)
        # 周期性检查：出现大面积新自由区 → 交还弓形覆盖
        self._front_check_cd -= 1
        if self._front_check_cd <= 0 and self._cov_cd <= 0:
            self._front_check_cd = self.p.frontier_check_every
            if self._new_free_cells(slam.grid) >= self.p.new_area_min:
                if self._plan_coverage(slam, slam.pose):
                    log.info("[frontier] 新自由区 → 弓形覆盖")
                    self._enter(COVERAGE)
                    return (0.0, 0.0)
                self._cov_cd = self.p.cov_cooldown
        return cmd

    # ---- 导航基础设施（与 FrontierExplorer 相同的语义） ----

    def _plan_to(self, grid: OccupancyGrid, pose, goal: tuple[int, int]) -> list[tuple[int, int]] | None:
        passable, unknown = make_passable(
            grid._logodds, self.p.free_below, self.p.occupied_above, self.p.inflate_cells
        )
        sc, sr = grid.world_to_cell(pose[0], pose[1])
        # 起点逃逸气泡（机器人已物理存在于该处）
        r0, r1 = max(0, sr - 3), min(passable.shape[0], sr + 4)
        c0, c1 = max(0, sc - 3), min(passable.shape[1], sc + 4)
        lo = grid._logodds
        passable[r0:r1, c0:c1] |= lo[r0:r1, c0:c1] <= self.p.free_below
        # 目标放宽到附近可通行格
        goal_adj = self._nearest_passable(passable, goal)
        if goal_adj is None:
            return None
        res = astar(passable, unknown, (sc, sr), goal_adj, unknown_cost=3.0)
        return res.path or None

    @staticmethod
    def _nearest_passable(passable: np.ndarray, cell: tuple[int, int]) -> tuple[int, int] | None:
        c0, r0 = cell
        best, best_d = None, 1e9
        for dr in range(-4, 5):
            for dc in range(-4, 5):
                c, r = c0 + dc, r0 + dr
                if 0 <= c < passable.shape[1] and 0 <= r < passable.shape[0] and passable[r, c]:
                    d = dc * dc + dr * dr
                    if d < best_d:
                        best, best_d = (c, r), d
        return best

    def _follow_path(self, pose) -> tuple[float, float]:
        cs = self.cell_size
        while self._cov_path:
            wc, wr = self._cov_path[0]
            wx, wy = (wc + 0.5) * cs, (wr + 0.5) * cs
            if np.hypot(wx - pose[0], wy - pose[1]) < self.p.goal_tol:
                self._cov_path.pop(0)
                continue
            desired = float(np.arctan2(wy - pose[1], wx - pose[0]))
            diff = (desired - pose[2] + np.pi) % (2 * np.pi) - np.pi
            # 微分阻尼：抑制误差收敛时的超调（蛇形根源）
            d_diff = (diff - self._prev_diff) * 20.0     # 20Hz 控制频率
            self._prev_diff = diff
            w_cmd = max(-self.p.angular,
                        min(self.p.angular, 2.5 * diff - 0.6 * d_diff))
            # 连续调速：替代二值硬切换，避免速度跳变诱发超调极限环
            ratio = max(self.p.linear_slow / self.p.linear, 1.0 - 1.2 * abs(diff))
            v = self.p.linear * ratio
            return (v, w_cmd)
        # 当前段走完 → 路点达成，推进
        if self._cov_path_wp is not None:
            self._advance_wp()
        return (self.p.linear_slow, 0.0)

    # ---- swept 掩码 ----

    @staticmethod
    def _make_disk(radius: int) -> np.ndarray:
        d = np.arange(-radius, radius + 1)
        dy, dx = np.meshgrid(d, d, indexing="ij")
        return (dx * dx + dy * dy) <= radius * radius

    def _mark_swept(self, grid: OccupancyGrid, pose) -> None:
        if self._swept is None:
            self._swept = np.zeros(grid._logodds.shape, dtype=bool)
        c, r = grid.world_to_cell(pose[0], pose[1])
        rad = self.p.sweep_radius
        h, w = self._swept.shape
        r0, r1 = max(0, r - rad), min(h, r + rad + 1)
        c0, c1 = max(0, c - rad), min(w, c + rad + 1)
        if r1 <= r0 or c1 <= c0:
            return
        dr0, dc0 = r0 - (r - rad), c0 - (c - rad)
        self._swept[r0:r1, c0:c1] |= self._disk[dr0:dr0 + (r1 - r0), dc0:dc0 + (c1 - c0)]

    def _new_free_cells(self, grid: OccupancyGrid) -> int:
        """确认自由 ∧ 未扫过 的格数（coverage↔frontier 切换依据）。"""
        if self._swept is None:
            return 0
        lo = grid._logodds
        free = lo <= self.p.free_below
        return int((free & ~self._swept).sum())

    # ---- 传感器辅助 ----

    @staticmethod
    def _front_distance(scan: LaserScan2D) -> float:
        n = len(scan.ranges)
        if n == 0:
            return -1.0
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        front = [scan.ranges[(center + i) % n]
                 for i in range(-3, 4) if scan.ranges[(center + i) % n] > 0]
        return min(front) if front else -1.0

    def _wall_fit(self, scan: LaserScan2D) -> tuple[float, float] | None:
        """右侧 -60°/-120° 两波束拟合墙线 → (垂距 d, 墙-航向偏角 alpha)。

        alpha > 0 表示机器人右偏航（朝墙转），需左转对齐；
        任一波束无效（无回波/超量程）或两束打到不同平面时返回 None。
        机器人系约定：x 朝前，y 朝左，角度 CCW 为正。
        """
        n = len(scan.ranges)
        if n < 8:
            return None
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        i1 = (center - int(round(math.radians(60.0) / scan.angle_inc))) % n
        i2 = (center - int(round(math.radians(120.0) / scan.angle_inc))) % n
        r1, r2 = float(scan.ranges[i1]), float(scan.ranges[i2])
        rmax = float(scan.range_max)
        if not (0 < r1 < rmax * 0.95 and 0 < r2 < rmax * 0.95):
            return None
        a1, a2 = math.radians(-60.0), math.radians(-120.0)
        x1, y1 = r1 * math.cos(a1), r1 * math.sin(a1)
        x2, y2 = r2 * math.cos(a2), r2 * math.sin(a2)
        seg = math.hypot(x2 - x1, y2 - y1)
        if seg < 1e-6:
            return None
        vx, vy = x2 - x1, y2 - y1
        if vx < 0:                       # 统一墙方向向量指向机器人前方
            vx, vy = -vx, -vy
        alpha = math.atan2(vy, vx)
        d = abs(x1 * y2 - y1 * x2) / seg  # 原点到墙线的垂距
        return (d, alpha)

    @staticmethod
    def _avoid_sign(scan: LaserScan2D) -> float:
        n = len(scan.ranges)
        center = int(round((0.0 - scan.angle_min) / scan.angle_inc)) % n
        left = [scan.ranges[(center + i) % n] for i in range(4, 12)]
        right = [scan.ranges[(center - i) % n] for i in range(4, 12)]
        d_l = np.mean([r for r in left if r > 0]) if any(r > 0 for r in left) else 0.0
        d_r = np.mean([r for r in right if r > 0]) if any(r > 0 for r in right) else 0.0
        return 1.0 if d_l >= d_r else -1.0

    # ---- 阶段迁移 ----

    def _enter(self, phase: str) -> None:
        self.phase = phase
        self.visited_phases.append(phase)
        if phase == WALL_FOLLOW:
            self._d_s = self._a_s = None
