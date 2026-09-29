"""闭环自主建图端到端测试：SLAM + Frontier 探索无人驾驶，直到建图完成。

这正是"得到一张完整地图"的核心闭环：
    robot.step(cmd) → sensor_frame → SLAM(建图+定位) → FrontierExplorer(选目标+A*导航) → cmd …
"""

import math

import numpy as np
import pytest

from vrobot.control.explorer import FrontierExplorer
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import ROOM_H, ROOM_W, SimParams, SimRobot, SimWorld, gt_wall_mask
from tests.test_slam2d import _dilate


@pytest.fixture(scope="module")
def mapping_run():
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11))
    slam = Slam2D(SlamConfig(use_match=True))
    explorer = FrontierExplorer(cell_size=slam.grid.cell_size)

    max_steps = 9000  # 20Hz → 450s 仿真时长上限
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


def test_exploration_completes(mapping_run):
    """frontier 探索应在限定步数内自行收敛（无可达边界 → done）。"""
    world, robot, slam, explorer, done_step = mapping_run
    assert done_step is not None, "探索未在限定步数内完成（可能陷入无进展循环）"
    assert explorer.done
    print(f"\n[诊断] 探索完成于第 {done_step} 步（≈{done_step * 0.05:.0f}s 仿真时间），"
          f"轨迹 {len(slam.trajectory)} 点，栅格更新 {slam.grid.n_updates} 次")


def test_full_map_wall_recall(mapping_run):
    """建图完成后，全屋真值墙面召回率应 ≥ 85%（±3 格漂移容差）。"""
    world, robot, slam, explorer, done_step = mapping_run
    gt = gt_wall_mask(world, slam.grid, band=4.0)
    walls = _dilate(slam.grid.obstacles(0.65), 3)
    recall = (gt & walls).sum() / max(gt.sum(), 1)
    print(f"[诊断] 全屋墙面召回率 = {recall:.3f}")
    assert recall > 0.85, f"建图完成但墙面召回率仅 {recall:.2f}"


def test_low_false_positive_full_map(mapping_run):
    """全图误报墙率 < 10%（排除真值墙面 14px 缓冲带）。"""
    world, robot, slam, explorer, done_step = mapping_run
    gt_wide = gt_wall_mask(world, slam.grid, band=14.0)
    walls = slam.grid.obstacles(0.65)
    fp = (walls & ~gt_wide).sum() / max(walls.sum(), 1)
    print(f"[诊断] 全图误报率 = {fp:.3f}（{int((walls & ~gt_wide).sum())}/{int(walls.sum())} 格）")
    assert fp < 0.10


def test_final_pose_accuracy(mapping_run):
    """探索结束（长航时）后位姿误差仍应 < 35px —— 验证长航时漂移可控。"""
    world, robot, slam, explorer, done_step = mapping_run
    tx, ty, tth = robot.truth
    px, py, pth = slam.pose
    pos_err = math.hypot(px - tx, py - ty)
    th_err = abs((pth - tth + math.pi) % (2 * math.pi) - math.pi)
    print(f"[诊断] 长航时终点误差 = {pos_err:.1f}px / {math.degrees(th_err):.1f}°")
    assert pos_err < 35.0, f"长航时位置误差 {pos_err:.1f}px 超限"
    assert th_err < 0.35


def test_coverage_of_free_space(mapping_run):
    """已确认自由区（±2 格容差）应覆盖房间自由面积 ≥ 80%，证明不是只转了个圈。

    容差说明：关键帧插入（每 5px/0.15rad 入图一次）下，144 线雷达的射线锥
    在远处有 1-2 格间隙，自由格标记天然稀疏；完整性以墙面召回为准，
    此指标只防"没去过的区域"。
    """
    world, robot, slam, explorer, done_step = mapping_run
    lo = slam.grid._logodds
    cs = slam.grid.cell_size
    slam_free = _dilate(lo <= -0.3, 2)  # ±2 格容差吸收射线锥间隙
    # 真值自由格：房间内（四墙之间）且格中心不在任何实体矩形内（留 1 格余量）。
    # 注意不能把"房间外的整个栅格"算进自由区——那是永远无法探索的区域。
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
