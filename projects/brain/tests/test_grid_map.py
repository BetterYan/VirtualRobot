"""占据栅格单元测试。"""

import math

import numpy as np
import pytest

from vrobot.slam.grid_map import OccupancyGrid


@pytest.fixture
def grid():
    return OccupancyGrid(cell_size=8.0, width=100, height=80, l_hit=0.9, l_miss=-0.35)


def test_coordinate_roundtrip(grid):
    col, row = grid.world_to_cell(100.0, 64.0)
    cx, cy = grid.cell_center(col, row)
    assert (col, row) == (12, 8)
    assert abs(cx - (12.5 * 8)) < 1e-9
    assert abs(cy - (8.5 * 8)) < 1e-9


def test_ray_marks_endpoint_occupied(grid):
    """一条从 (40,40) 到 (280,40) 的命中射线：终点占据、途经空闲。"""
    pose = (40.0, 40.0, 0.0)
    points = np.array([[280.0, 40.0]])
    for _ in range(5):  # 多次观测增强置信
        grid.update(pose, points)
    col_end, row_end = grid.world_to_cell(280.0, 40.0)
    col_mid, row_mid = grid.world_to_cell(160.0, 40.0)
    p = grid.prob()
    assert p[row_end, col_end] > 0.85       # 终点高概率占据
    assert p[row_mid, col_mid] < 0.2        # 途经点低概率
    obs = grid.obstacles()
    assert obs[row_end, col_end] and not obs[row_mid, col_mid]


def test_free_space_from_multiple_views(grid):
    """同一空间被两个视角观测后应保持低占据概率（2 次 miss → l=-0.7 → p≈0.33）。"""
    grid.update((40.0, 40.0, 0.0), np.array([[200.0, 40.0]]))
    grid.update((200.0, 40.0, math.pi), np.array([[40.0, 40.0]]))
    p = grid.prob()
    col_mid, row_mid = grid.world_to_cell(120.0, 40.0)
    assert p[row_mid, col_mid] < 0.4


def test_out_of_bounds_query_returns_zero(grid):
    probs = grid.prob_at_points(np.array([[-50.0, 10.0], [9999.0, 10.0], [50.0, 50.0]]))
    assert probs[0] == 0.0 and probs[1] == 0.0


def test_logodds_clamped(grid):
    pose = (40.0, 40.0, 0.0)
    points = np.array([[200.0, 40.0]])
    for _ in range(100):  # 反复命中不应超过 clamp
        grid.update(pose, points)
    assert grid._logodds.max() <= grid.l_max
