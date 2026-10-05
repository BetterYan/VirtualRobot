"""Cartographer 引擎适配器：把原生 Cartographer 包装成 Slam2D 同款接口。

slam2d.yaml 中 `slam.engine: cartographer` 时由 slam_node 选用本引擎。

与内置引擎的本质区别——建图完全由 Cartographer 完成：
  - 地图 = 位姿图子图（Submap2D 概率栅格）的合并投影，本模块不做任何
    log-odds 射线建图（l_hit/l_miss 逻辑不参与）；
  - 位姿优先取位姿图全局优化结果（含回环修正），无节点时退化为
    local SLAM 结果 + 首帧 origin 复合；
  - 探索器/可视化继续消费 OccupancyGrid，其内容直接来自 Cartographer 地图。

单位契约（三端一致：Godot ↔ slam2d.yaml ↔ 生成 Lua）：
  - Godot/算法侧长度单位 px，Cartographer 内部 m，换算系数 `scale`（m/px）。
  - Lua 的 min_range/max_range 由 min_range_px*scale 与 range_max*scale 生成；
    激光线数不进 Lua（Cartographer 消费点云），写入注释便于核对。

位姿对齐：Cartographer 全局系原点 = 首帧扫描位姿（identity），用首帧
里程计位姿做一次 SE(2) 复合，输出 Godot 世界系位姿。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from vrobot.sensors.models import LaserScan2D, Odometry2D
from vrobot.slam.grid_map import OccupancyGrid
from vrobot.slam.odom import compose, wrap_angle
from vrobot.slam.scan_matcher import CorrelativeScanMatcher
from vrobot.slam.slam2d import SlamResult

# 原生扩展（包 __init__ 已注册 _bin DLL 目录）
from vrobot.slam import vrobot_slam_native as _native  # noqa: F401

_CARTOGRAPHER_CONFIG_DIR = Path(
    __file__).resolve().parents[3] / "configs"

# 与 Godot 上行帧 / yaml sensor 段对应的兜底默认（三端对齐的文档化常量）
DEFAULT_RANGE_MAX_PX = 320.0   # = Godot sim_main.gd RANGE_MAX
DEFAULT_MIN_RANGE_PX = 14.0    # ≈机器人半径，过滤近距离自身回波
DEFAULT_SCALE = 0.05           # m / px


@dataclass
class CartographerEngineConfig:
    grid: dict = field(
        default_factory=lambda: dict(
            cell_size=8.0, width=200, height=160,
            l_hit=0.9, l_miss=-0.35, l_min=-4.0, l_max=4.0,
        )
    )
    scale: float = DEFAULT_SCALE            # m / px
    min_range_px: float = DEFAULT_MIN_RANGE_PX
    use_odometry: bool = True               # Godot 里程计作为 odom 输入
    num_background_threads: int = 2
    optimize_every_n_nodes: int = 30
    min_score: float = 0.30                 # matched 判定门限（调试视图用）
    map_sync_interval: int = 3              # 每 N 帧同步一次 Cartographer 子图到栅格


class _MatcherInfo:
    """给 DebugViewer 等提供与 Slam2D.matcher 兼容的 min_score 视图。"""

    def __init__(self, min_score: float):
        self.min_score = min_score


class CartographerSLAM2D:
    """与 Slam2D 接口兼容、建图完全由 Cartographer 完成的引擎。"""

    def __init__(self, config: CartographerEngineConfig | None = None,
                 sensor: dict | None = None):
        self.cfg = config or CartographerEngineConfig()
        self.grid = OccupancyGrid(**self.cfg.grid)
        self.matcher = _MatcherInfo(self.cfg.min_score)
        self.pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.trajectory: list[tuple[float, float, float]] = []

        self._range_max_px = float(
            (sensor or {}).get("range_max", DEFAULT_RANGE_MAX_PX))
        self._t = 0.0
        self._frame = 0
        self._origin: tuple[float, float, float] | None = None
        self._local_pose: tuple[float, float, float] | None = None
        self._started = False
        self._mapper = self._new_mapper()

    # ------------------------------------------------------------------
    # Lua 配置生成：所有与传感器相关的数值都由 yaml/Godot 契约推导
    # ------------------------------------------------------------------
    def _build_lua(self) -> str:
        cfg = self.cfg
        min_range_m = cfg.min_range_px * cfg.scale
        max_range_m = self._range_max_px * cfg.scale
        return f"""\
-- 自动生成（vrobot.slam.cartographer_engine）——勿手改，改 slam2d.yaml
-- 一致性契约:
--   激光线数   : Godot BEAMS=144（帧内 angle_inc=TAU/beams，点云直接消费）
--   最近距离   : {cfg.min_range_px:.1f} px * {cfg.scale} m/px = {min_range_m:.3f} m
--   最远距离   : {self._range_max_px:.1f} px * {cfg.scale} m/px = {max_range_m:.3f} m
include "map_builder.lua"
include "trajectory_builder.lua"

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = {cfg.num_background_threads}

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = {min_range_m:.4f}
TRAJECTORY_BUILDER_2D.max_range = {max_range_m:.4f}
TRAJECTORY_BUILDER_2D.min_z = -0.1
TRAJECTORY_BUILDER_2D.max_z = 0.1
TRAJECTORY_BUILDER_2D.missing_data_ray_length = {max_range_m:.4f}
-- 在线相关匹配搜索窗小且评分粗糙：有可靠里程计时反而拉偏位姿，故仅在
-- 无里程计输入时启用（Cartographer 最佳实践）。
-- 已知缺陷（待查 native 层/extrapolator）：纯旋转时角跟踪欠转 ~13%·转角，
-- 误差随转角线性增长（Stage2 T3 可复现），是探索地图角向发散的根源。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = {str(not cfg.use_odometry).lower()}
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = {0.5 * cfg.scale:.4f}
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2)
-- 概率栅格插入器：默认 hit=0.55/miss=0.49 过于保守，单次观测后概率仍贴着 0.5，
-- 桥接成 log-odds 后自由格永远够不着探索器的 free_below(-0.3) → 全图"半观测"。
-- 调强证据强度：单次 miss 即压到 p≈0.30（log-odds≈-0.85），单次 hit≈0.75（+1.10）。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.75
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.30
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true

POSE_GRAPH.optimize_every_n_nodes = {cfg.optimize_every_n_nodes}
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return {{ MAP_BUILDER = MAP_BUILDER, TRAJECTORY_BUILDER = TRAJECTORY_BUILDER }}
"""

    def _new_mapper(self):
        return _native.Mapper(self._build_lua(),
                              [str(_CARTOGRAPHER_CONFIG_DIR)])

    def _ensure_started(self) -> None:
        """首次喂入数据前启动轨迹（注册 lidar / odom 传感器）。"""
        if self._started:
            return
        self._mapper.start_trajectory(
            "lidar", None, "odom" if self.cfg.use_odometry else None)
        self._started = True

    # ------------------------------------------------------------------
    # Slam2D 兼容接口
    # ------------------------------------------------------------------
    @property
    def initialized(self) -> bool:
        return self._origin is not None

    @property
    def theta(self) -> float:
        return wrap_angle(self.pose[2])

    def reset(self, pose: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self.grid = OccupancyGrid(**self.cfg.grid)
        self.pose = pose
        self.trajectory = []
        self._origin = pose
        self._local_pose = None
        self._t = 0.0
        self._frame = 0
        self._started = False
        self._mapper = self._new_mapper()

    def process(self, odom: Odometry2D, scan: LaserScan2D) -> SlamResult:
        """喂入一帧传感器数据，返回本帧位姿估计（Godot 世界系, px）。

        建图完全交给 Cartographer：本方法只喂数据、取位姿、把 Cartographer
        子图概率栅格同步进 OccupancyGrid（供探索器/可视化消费）。
        """
        self._t += max(float(odom.dt), 1e-3)
        self._frame += 1

        if self._origin is None:
            self._origin = odom.as_tuple()
        if self._local_pose is None:
            self._local_pose = (0.0, 0.0, 0.0)
        self._ensure_started()

        # 1) 传感器数据送入 Cartographer（时间戳单调递增）
        if self.cfg.use_odometry:
            self._mapper.add_odometry(
                self._t, odom.x * self.cfg.scale,
                odom.y * self.cfg.scale, odom.theta)

        pts_local = scan.local_endpoints()  # (N,2) px
        n_points = int(pts_local.shape[0])
        if n_points > 0:
            pts_m = pts_local * self.cfg.scale
            pts3 = np.zeros((n_points, 3))
            pts3[:, :2] = pts_m
            self._mapper.add_scan(self._t, pts3)

        # 2) 位姿：优先位姿图全局优化位姿（含回环修正），
        #    否则退化为 origin ⊕ local SLAM 位姿。注意换算 m -> px。
        pose_m: tuple[float, float, float] | None = None
        if self._mapper.num_nodes > 0:
            poses = self._mapper.optimized_poses()
            if len(poses) > 0:
                last = poses[-1]
                pose_m = (float(last[0]), float(last[1]), float(last[2]))
        if pose_m is not None:
            pose_px = (pose_m[0] / self.cfg.scale,
                       pose_m[1] / self.cfg.scale, pose_m[2])
            self.pose = compose(self._origin, pose_px)
        else:
            latest = self._mapper.latest_local_pose
            if latest is not None:
                self._local_pose = (latest[0] / self.cfg.scale,
                                    latest[1] / self.cfg.scale,
                                    latest[2])  # type: ignore[assignment]
            self.pose = compose(self._origin, self._local_pose or (0.0, 0.0, 0.0))

        # 3) 同步 Cartographer 子图 -> OccupancyGrid（建图结果的唯一来源）
        if self._frame % max(self.cfg.map_sync_interval, 1) == 0:
            self._sync_grid()

        self.trajectory.append(self.pose)

        # 4) matched/score 代理：在 Cartographer 地图上对位姿打分（调试视图用）
        score = 0.0
        if self.grid.n_updates > 0 and n_points >= 5:
            score = float(self._scorer().score(scan, self.pose))
        matched = score >= self.cfg.min_score

        return SlamResult(
            pose=self.pose, matched=matched, score=score, n_points=n_points
        )

    # ------------------------------------------------------------------
    def _sync_grid(self) -> None:
        """把 Cartographer 子图概率栅格合并投影进 OccupancyGrid。

        已观测格：log-odds = logit(占据概率)；未知格保持 0（=未观测语义）。
        """
        lo = self.grid._logodds
        height, width = lo.shape
        cell = self.grid.cell_size
        scale = self.cfg.scale

        for sub in self._mapper.submap_grids():
            prob = np.asarray(sub.prob, dtype=np.float32)
            known = np.asarray(sub.known, dtype=bool)
            if not known.any():
                continue
            h, w = prob.shape
            # C++ 导出约定：cell(row0,col0) 中心 = origin，行向世界Y递减、
            # 列向世界X递减（cartographer MapLimits 的镜像几何）
            xs = sub.origin_x - np.arange(w) * sub.resolution
            ys = sub.origin_y - np.arange(h) * sub.resolution
            grid_x, grid_y = np.meshgrid(xs, ys)
            c, s = math.cos(sub.pose_yaw), math.sin(sub.pose_yaw)
            wx = (c * grid_x - s * grid_y + sub.pose_x) / scale  # 世界 px
            wy = (s * grid_x + c * grid_y + sub.pose_y) / scale
            cols = np.floor(wx / cell).astype(np.int64)
            rows = np.floor(wy / cell).astype(np.int64)
            valid = (known & (cols >= 0) & (cols < width)
                     & (rows >= 0) & (rows < height))
            if not valid.any():
                continue
            p = np.clip(prob, 0.03, 0.97)
            lod = np.log(p / (1.0 - p)).astype(np.float32)
            # 合并策略（对规划安全）：占据证据取更强者；自由证据只填充未观测格
            cur = lo[rows[valid], cols[valid]]
            l_v = lod[valid]
            take = ((l_v > 0) & (l_v > cur)) | ((l_v < 0) & (cur == 0))
            r_v, c_v = rows[valid], cols[valid]
            lo[r_v[take], c_v[take]] = l_v[take]

        self.grid._n_updates += 1

    def _scorer(self) -> CorrelativeScanMatcher:
        """按当前栅格构造一次性打分器（仅用于 matched/score 代理）。"""
        return CorrelativeScanMatcher(
            self.grid, min_score=self.cfg.min_score)
