"""Cartographer 引擎适配器测试（接口兼容 Slam2D、单位换算、位姿对齐）。

依赖原生扩展：uv run python scripts/build_native.py
"""

from __future__ import annotations

import time

import numpy as np
import pytest

pytest.importorskip(
    "vrobot.slam.vrobot_slam_native",
    reason="原生扩展未构建: uv run python scripts/build_native.py")

from vrobot.sensors.models import LaserScan2D, Odometry2D  # noqa: E402
from vrobot.slam.cartographer_engine import (  # noqa: E402
    CartographerEngineConfig,
    CartographerSLAM2D,
)

GRID = dict(cell_size=8.0, width=200, height=160,
            l_hit=0.9, l_miss=-0.35, l_min=-4.0, l_max=4.0)


def make_scan(pose_px: tuple[float, float] = (0.0, 0.0),
              beams: int = 144, wall_r_px: float = 100.0,
              range_max: float = 320.0) -> LaserScan2D:
    """静态世界扫描：固定圆墙（圆心=世界原点），机器人位于 pose_px。

    解析射线-圆求交，保证多帧数据对应同一物理世界（非跟随机器人）。
    """
    angles = -np.pi + np.arange(beams) * (2.0 * np.pi / beams)
    px, py = pose_px
    dx, dy = np.cos(angles), np.sin(angles)
    b = px * dx + py * dy
    c = px * px + py * py - wall_r_px * wall_r_px
    disc = np.maximum(b * b - c, 0.0)
    t = -b + np.sqrt(disc)
    return LaserScan2D(
        ranges=t.astype(np.float32),
        angle_min=-np.pi,
        angle_inc=2.0 * np.pi / beams,
        range_max=range_max,
    )


def test_engine_interface_contract():
    cfg = CartographerEngineConfig(grid=GRID)
    slam = CartographerSLAM2D(cfg, sensor={"range_max": 320.0})
    # 与 Slam2D 相同的对外属性（explorer/viewer/debug 依赖）
    assert slam.grid.cell_size == 8.0
    assert slam.matcher.min_score == pytest.approx(0.30)
    assert slam.pose == (0.0, 0.0, 0.0)
    assert not slam.initialized


def test_process_pipeline_and_units():
    cfg = CartographerEngineConfig(grid=GRID, scale=0.05,
                                   use_odometry=True)
    slam = CartographerSLAM2D(cfg, sensor={"range_max": 320.0})

    for i in range(40):
        odom = Odometry2D(x=0.5 * i, y=0.0, theta=0.0, dt=0.05)
        result = slam.process(odom, make_scan((0.5 * i, 0.0)))
        assert np.all(np.isfinite(result.pose))
        assert result.n_points == 144

    # 给后台线程留出发送 local SLAM 结果的时间
    time.sleep(0.3)

    assert len(slam.trajectory) == 40
    assert slam.grid.n_updates >= 1
    # 建图完全来自 Cartographer 子图：桥接栅格中应有已观测的占据格
    submaps = slam._mapper.submap_grids()
    assert len(submaps) >= 1, "Cartographer 应至少产生一个子图"
    assert any(int(np.asarray(s.known).sum()) > 0 for s in submaps)
    assert slam.grid.obstacles().sum() > 0, "栅格应包含 Cartographer 导出的墙壁"
    # 里程计输入 0.5px/帧 * 40 = 20px 前进；位姿应跟踪里程计（px 世界系）
    assert 0.0 <= slam.pose[0] <= 40.0
    assert abs(slam.pose[1]) < 20.0
    assert abs(slam.theta) < 0.5


def test_process_empty_scan_does_not_crash():
    slam = CartographerSLAM2D(
        CartographerEngineConfig(grid=GRID), sensor={"range_max": 320.0})
    empty = LaserScan2D(
        ranges=np.full(144, -1.0, dtype=np.float32),
        angle_min=-np.pi, angle_inc=2 * np.pi / 144, range_max=320.0)
    for i in range(5):
        result = slam.process(
            Odometry2D(x=0.0, y=0.0, theta=0.0, dt=0.05), empty)
        assert result.n_points == 0
    time.sleep(0.1)


def test_reset_restarts_mapper():
    slam = CartographerSLAM2D(
        CartographerEngineConfig(grid=GRID), sensor={"range_max": 320.0})
    slam.process(Odometry2D(x=10.0, y=5.0, theta=0.3, dt=0.05),
                 make_scan((10.0, 5.0)))
    slam.reset(pose=(10.0, 5.0, 0.3))
    assert slam.pose == (10.0, 5.0, 0.3)
    assert slam.trajectory == []
    assert slam.grid.n_updates == 0
    # 重置后仍可继续工作
    result = slam.process(
        Odometry2D(x=10.5, y=5.0, theta=0.3, dt=0.05), make_scan((10.5, 5.0)))
    assert np.all(np.isfinite(result.pose))
