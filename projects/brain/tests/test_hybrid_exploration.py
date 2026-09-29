"""混合探索（沿边→弓形→frontier）端到端测试。

流程：seek_wall 直行找边界 → wall_follow 右手沿边 → 闭环 →
弓形车道覆盖已知自由区 → frontier 收尾，直至无边界且无新自由区。
"""

import math

import numpy as np
import pytest

from vrobot.control.hybrid_explorer import COVERAGE, FRONTIER, WALL_FOLLOW, HybridExplorer
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import ROOM_H, ROOM_W, SimParams, SimRobot, SimWorld, gt_wall_mask
from tests.test_slam2d import _dilate


def _scan_for_wall(d: float, yaw: float = 0.0, n: int = 144) -> "object":
    """构造沿边几何的合成雷达：右侧平行墙，垂距 d，机器人右偏航 yaw。

    墙在世界系 y=-d（机器人位于原点、朝向 yaw）。波束世界角 = 波束相对角 - yaw。
    """
    from vrobot.sensors.models import LaserScan2D

    angle_min = -math.pi
    angle_inc = 2 * math.pi / n
    range_max = 320.0
    ranges = []
    for k in range(n):
        a_world = angle_min + k * angle_inc - yaw
        s = math.sin(a_world)
        if s < -1e-6:
            ranges.append(min(d / (-s), range_max))
        else:
            ranges.append(0.0)
    return LaserScan2D(ranges=np.array(ranges, dtype=np.float32),
                       angle_min=angle_min, angle_inc=angle_inc, range_max=range_max)


def test_wall_fit_parallel_wall():
    """平行墙：拟合垂距 = 真值，偏角 ≈ 0。"""
    ex = HybridExplorer(cell_size=8.0)
    d, a = ex._wall_fit(_scan_for_wall(45.0))
    assert abs(d - 45.0) < 1.0
    assert abs(a) < 0.02


def test_wall_fit_yawed_wall():
    """右偏航 30° 朝墙：拟合应给出垂距不变、偏角 = +30°（需左转对齐）。"""
    ex = HybridExplorer(cell_size=8.0)
    d, a = ex._wall_fit(_scan_for_wall(45.0, yaw=math.radians(30.0)))
    assert abs(d - 45.0) < 1.5
    assert abs(a - math.radians(30.0)) < 0.02


def test_wall_fit_invalid_beams():
    """波束无回波/超量程 → 返回 None（触发丢失找回逻辑）。"""
    ex = HybridExplorer(cell_size=8.0)
    scan = _scan_for_wall(45.0)
    scan.ranges[:] = 0.0
    assert ex._wall_fit(scan) is None


@pytest.fixture(scope="module")
def hybrid_run():
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11))
    slam = Slam2D(SlamConfig(use_match=True))
    explorer = HybridExplorer(cell_size=slam.grid.cell_size)

    max_steps = 12000  # 20Hz → 600s 仿真时长上限（三阶段比纯 frontier 更耗时）
    last_frame = None
    done_step = None
    for k in range(max_steps):
        if last_frame is not None:
            slam.process(last_frame.odom, last_frame.scan)
        if last_frame is None:
            cmd = (0.0, 0.0)
        else:
            cmd = explorer.update(slam, last_frame.scan)
        if explorer.done:
            done_step = k
            break
        last_frame = robot.step(*cmd)

    return world, robot, slam, explorer, done_step


def test_hybrid_completes(hybrid_run):
    """混合探索应在限定步数内自行收敛。"""
    world, robot, slam, explorer, done_step = hybrid_run
    assert done_step is not None, "混合探索未在限定步数内完成"
    assert explorer.done
    print(f"\n[诊断] 完成于第 {done_step} 步（≈{done_step * 0.05:.0f}s），"
          f"阶段演进: {' → '.join(explorer.visited_phases)}")


def test_phase_progression(hybrid_run):
    """应依次经历沿边、弓形覆盖阶段（frontier 可选，取决于覆盖是否遗漏）。"""
    world, robot, slam, explorer, done_step = hybrid_run
    visited = set(explorer.visited_phases)
    assert WALL_FOLLOW in visited, f"未进入沿边阶段: {explorer.visited_phases}"
    assert COVERAGE in visited, f"未进入弓形覆盖阶段: {explorer.visited_phases}"
    print(f"[诊断] 阶段演进: {' → '.join(explorer.visited_phases)}")


def test_hybrid_wall_recall(hybrid_run):
    """全屋真值墙面召回率 ≥ 85%。"""
    world, robot, slam, explorer, done_step = hybrid_run
    gt = gt_wall_mask(world, slam.grid, band=4.0)
    walls = _dilate(slam.grid.obstacles(0.65), 3)
    recall = (gt & walls).sum() / max(gt.sum(), 1)
    print(f"[诊断] 全屋墙面召回率 = {recall:.3f}")
    assert recall > 0.85, f"建图完成但墙面召回率仅 {recall:.2f}"


def test_hybrid_false_positive(hybrid_run):
    """全图误报墙率 < 10%。"""
    world, robot, slam, explorer, done_step = hybrid_run
    gt_wide = gt_wall_mask(world, slam.grid, band=14.0)
    walls = slam.grid.obstacles(0.65)
    fp = (walls & ~gt_wide).sum() / max(walls.sum(), 1)
    print(f"[诊断] 全图误报率 = {fp:.3f}")
    assert fp < 0.10


def test_hybrid_final_pose(hybrid_run):
    """长航时终点位姿误差 < 35px / 0.35rad。"""
    world, robot, slam, explorer, done_step = hybrid_run
    tx, ty, tth = robot.truth
    px, py, pth = slam.pose
    pos_err = math.hypot(px - tx, py - ty)
    th_err = abs((pth - tth + math.pi) % (2 * math.pi) - np.pi)
    print(f"[诊断] 终点误差 = {pos_err:.1f}px / {math.degrees(th_err):.1f}°")
    assert pos_err < 35.0
    assert th_err < 0.35


def test_hybrid_coverage(hybrid_run):
    """已确认自由区（±2 格容差）覆盖房间自由面积 ≥ 80%。"""
    world, robot, slam, explorer, done_step = hybrid_run
    lo = slam.grid._logodds
    cs = slam.grid.cell_size
    slam_free = _dilate(lo <= -0.3, 2)
    # 真值自由格：房间内且不在任何实体矩形内（留 1 格余量）
    free_gt = np.zeros_like(slam_free)
    rects = world.rects
    for row in range(slam.grid.height):
        for col in range(slam.grid.width):
            cx, cy = (col + 0.5) * cs, (row + 0.5) * cs
            if not (0.0 <= cx <= ROOM_W and 0.0 <= cy <= ROOM_H):
                continue  # 房间外（墙外），不属于可探索自由区
            inside = any(
                (x0 + cs) <= cx <= (x0 + w - cs) and (y0 + cs) <= cy <= (y0 + h - cs)
                for x0, y0, w, h in rects
            )
            if not inside:
                free_gt[row, col] = True
    cover = (slam_free & free_gt).sum() / max(free_gt.sum(), 1)
    print(f"[诊断] 自由空间覆盖率(±2格) = {cover:.3f}")
    assert cover > 0.80, f"自由空间覆盖率仅 {cover:.2f}，地图不完整"
