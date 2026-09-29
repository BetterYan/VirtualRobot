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

        if self.cfg.use_match and pts_local.shape[0] >= 5:
            # 2) 扫描匹配修正（得分低于 min_score 则拒绝，信任里程计预测）
            cand, score = self.matcher.match(scan, self.pose)
            if score >= self.matcher.min_score:
                self.pose = cand
                matched = True

        # 3) 建图：用当前最优位姿写入栅格
        world_pts = scan_world_points(scan, self.pose)
        self.grid.update(self.pose, world_pts)
        self.trajectory.append(self.pose)

        return SlamResult(
            pose=self.pose, matched=matched, score=score, n_points=int(pts_local.shape[0])
        )

    @property
    def theta(self) -> float:
        return wrap_angle(self.pose[2])
