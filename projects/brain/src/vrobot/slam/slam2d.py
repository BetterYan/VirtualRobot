"""2D SLAM 主类（v1：里程计预测 + 扫描匹配修正 + 占据栅格建图）。

TinySLAM 式管线：
  1. 里程计增量 -> 位姿预测
  2. 扫描匹配（scan-to-map）在预测附近搜索修正
  3. 质量门限：匹配得分过低则信任预测位姿
  4. 用修正后的位姿把扫描写入占据栅格

纯算法模块：只依赖 sensors/ 的数据模型，不感知通信 / 可视化 / 真值。
v2 扩展点：process() 内的位姿估计可整体替换为 RBPF 粒子滤波（particle.py）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from vrobot.sensors.models import LaserScan2D, Odometry2D, scan_world_points
from vrobot.slam.grid_map import OccupancyGrid
from vrobot.slam.odom import OdomTracker, wrap_angle
from vrobot.slam.scan_matcher import CorrelativeScanMatcher


@dataclass
class SlamConfig:
    grid: dict = field(
        default_factory=lambda: dict(
            cell_size=8.0, width=200, height=160,
            l_hit=0.9, l_miss=-0.35, l_min=-4.0, l_max=4.0,
        )
    )
    scan_match: dict = field(
        default_factory=lambda: dict(
            search_xy=24.0, search_th=0.20,
            coarse_step_xy=4.0, coarse_step_th=0.05, min_score=0.30,
        )
    )
    use_match: bool = True
    # 匹配预热：入图次数不足前地图不可信（首帧附近格点稀疏，得分面噪声大），
    # 跳过匹配、纯里程计预测，可消除启动阶段的尖刺。
    match_warmup_updates: int = 3
    # 迟滞接受：候选位姿得分须超过"当前预测位姿得分 + margin"才接受。
    # 沿墙/开阔地等得分面平坦场景下，argmax 会随激光噪声在相邻候选间翻转，
    # 导致静止时位姿在两个局部最优间跳变（位置/角度互补振荡）。margin 熄灭这种翻转。
    accept_margin: float = 0.02
    # 关键帧插入策略：位姿相对上次入图变化足够大才写入栅格。
    # 原地旋转时逐帧入图会让墙体沿角度方向涂抹成"辐条"（旋转角超过雷达角分辨率）。
    insert_min_dist: float = 5.0     # 平移 ≥5px 才入图
    insert_min_angle: float = 0.15   # 旋转 ≥0.15rad(≈8.6°) 才入图


@dataclass
class SlamResult:
    pose: tuple[float, float, float]
    matched: bool
    score: float
    n_points: int


class Slam2D:
    def __init__(self, config: SlamConfig | None = None):
        self.cfg = config or SlamConfig()
        self.grid = OccupancyGrid(**self.cfg.grid)
        self.matcher = CorrelativeScanMatcher(self.grid, **self.cfg.scan_match)
        self._odom = OdomTracker()
        self.pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.trajectory: list[tuple[float, float, float]] = []
        self._initialized = False
        self._last_insert: tuple[float, float, float] | None = None

    @property
    def initialized(self) -> bool:
        return self._initialized

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.pose = pose
        self.trajectory = []
        self._odom.reset()
        self.grid = OccupancyGrid(**self.cfg.grid)
        self.matcher = CorrelativeScanMatcher(self.grid, **self.cfg.scan_match)
        self._initialized = False
        self._last_insert = None

    def process(self, odom: Odometry2D, scan: LaserScan2D) -> SlamResult:
        """喂入一帧传感器数据，返回本帧位姿估计。"""
        odom_t = (odom.x, odom.y, odom.theta)

        if not self._initialized:
            self.pose = odom_t
            self._odom = OdomTracker()
            self._odom.delta_since_last(odom_t)
            self._initialized = True
        else:
            delta = self._odom.delta_since_last(odom_t)
            if delta is not None:
                # 1) 里程计预测
                from vrobot.slam.odom import compose

                self.pose = compose(self.pose, delta)

        pts_local = scan.local_endpoints()
        score, matched = 0.0, False

        if self.cfg.use_match and pts_local.shape[0] >= 5 \
                and self.grid.n_updates >= self.cfg.match_warmup_updates:
            # 2) 扫描匹配修正。两级门限：
            #    a) score >= min_score（绝对质量）
            #    b) score >= 当前预测位姿得分 + accept_margin（迟滞，抑制静止抖动）
            cand, score = self.matcher.match(scan, self.pose)
            base_score = self.matcher.score(scan, self.pose)
            if score >= self.matcher.min_score and score >= base_score + self.cfg.accept_margin:
                self.pose = cand
                matched = True

        # 3) 建图：关键帧插入——位姿变化足够大才写入栅格（抑制旋转涂抹）
        if self._should_insert():
            world_pts = scan_world_points(scan, self.pose)
            self.grid.update(self.pose, world_pts)
            self._last_insert = self.pose
        self.trajectory.append(self.pose)

        return SlamResult(
            pose=self.pose, matched=matched, score=score, n_points=int(pts_local.shape[0])
        )

    @property
    def theta(self) -> float:
        return wrap_angle(self.pose[2])

    def _should_insert(self) -> bool:
        if self._last_insert is None:
            return True
        dx = self.pose[0] - self._last_insert[0]
        dy = self.pose[1] - self._last_insert[1]
        if dx * dx + dy * dy >= self.cfg.insert_min_dist**2:
            return True
        dth = abs(wrap_angle(self.pose[2] - self._last_insert[2]))
        return dth >= self.cfg.insert_min_angle
