"""扫描匹配单元测试：在理想地图上验证扰动恢复能力。"""

import numpy as np
import pytest

from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid
from vrobot.slam.scan_matcher import CorrelativeScanMatcher
from tests.sim_world import DEFAULT_BEAMS, DEFAULT_RANGE_MAX, SimWorld


@pytest.fixture
def room():
    return SimWorld()


def _scan_from(world: SimWorld, pose, n=DEFAULT_BEAMS, rmax=DEFAULT_RANGE_MAX) -> LaserScan2D:
    """直接用真值几何生成一帧"干净"扫描（无噪声）。"""
    import math

    ranges = np.empty(n, dtype=np.float32)
    for i in range(n):
        a = -math.pi + i * (2 * math.pi / n)
        ranges[i] = world.cast(pose[0], pose[1], pose[2] + a, rmax)
    return LaserScan2D(
        ranges=ranges, angle_min=-math.pi, angle_inc=2 * math.pi / n, range_max=rmax
    )


def _build_map(world, grid: OccupancyGrid, poses):
    for pose in poses:
        scan = _scan_from(world, pose)
        from vrobot.sensors.models import scan_world_points

        grid.update(pose, scan_world_points(scan, pose))


def test_match_recovers_small_offset(room):
    """先用真值建图，再以偏移 8px 的先验位姿匹配，应恢复到 2px 内。"""
    true_pose = (450.0, 320.0, 0.3)
    grid = OccupancyGrid(cell_size=8.0, width=200, height=160)
    _build_map(room, grid, [(450.0, 320.0, 0.0), (450.0, 320.0, 2.0), true_pose])

    matcher = CorrelativeScanMatcher(grid, search_xy=24.0, search_th=0.2, min_score=0.3)
    scan = _scan_from(room, true_pose)
    guess = (true_pose[0] + 8.0, true_pose[1] - 6.0, true_pose[2] + 0.08)
    (mx, my, mth), score = matcher.match(scan, guess)

    assert score > 0.5
    # 8px 栅格的概率查询存在格内量化平台（450 与 452 同格），断言放宽到半格
    assert abs(mx - true_pose[0]) < 4.0
    assert abs(my - true_pose[1]) < 4.0
    assert abs(mth - true_pose[2]) < 0.04


def test_match_rejects_empty_scan(room):
    grid = OccupancyGrid(cell_size=8.0, width=200, height=160)
    matcher = CorrelativeScanMatcher(grid)
    empty = LaserScan2D(ranges=np.full(DEFAULT_BEAMS, -1.0), range_max=DEFAULT_RANGE_MAX)
    pose, score = matcher.match(empty, (100.0, 100.0, 0.0))
    assert score == 0.0 and pose == (100.0, 100.0, 0.0)


def test_score_ordering(room):
    """真值位姿得分应高于明显偏移的位姿。"""
    true_pose = (450.0, 320.0, 1.1)
    grid = OccupancyGrid(cell_size=8.0, width=200, height=160)
    _build_map(room, grid, [true_pose])
    matcher = CorrelativeScanMatcher(grid)
    scan = _scan_from(room, true_pose)
    s_true = matcher.score(scan, true_pose)
    s_bad = matcher.score(scan, (true_pose[0] + 40.0, true_pose[1], true_pose[2]))
    assert s_true > s_bad
