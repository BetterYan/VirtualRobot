"""2D SLAM 端到端测试：合成世界环游一圈 → 建图 → 断言位姿与地图质量。

流程完全模拟 Godot 端的数据（带噪里程计 + 带噪激光，20Hz），
SLAM 不知道任何真实地图或真值位姿。
"""

import math

import numpy as np
import pytest

from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import (
    SimParams,
    SimRobot,
    SimWorld,
    drive_to_waypoints,
    gt_wall_mask,
)

# 环形巡航路点（走在房间/家具之间的走廊中央，避免剐蹭）
WAYPOINTS = [
    (100.0, 90.0),
    (820.0, 90.0),
    (820.0, 580.0),
    (100.0, 580.0),
    (100.0, 300.0),
]


@pytest.fixture(scope="module")
def slam_run():
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=7))
    frames = drive_to_waypoints(robot, WAYPOINTS)
    assert len(frames) > 300, "巡航帧数异常，检查仿真驾驶"

    slam = Slam2D(SlamConfig(use_match=True))
    results = [slam.process(f.odom, f.scan) for f in frames]
    return slam, robot, frames, results


def _reachable_mask(grid, traj: np.ndarray, radius: float = 320.0) -> np.ndarray:
    """轨迹点半径 radius 内的栅格区域（传感器可及）。向量化实现。"""
    cs = grid.cell_size
    cols = (np.arange(grid.width) + 0.5) * cs
    rows = (np.arange(grid.height) + 0.5) * cs
    XX, YY = np.meshgrid(cols, rows)
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    r2 = radius * radius
    for x, y in traj[::5, :2]:  # 每 5 个轨迹点采样一次，覆盖性足够
        mask |= (XX - x) ** 2 + (YY - y) ** 2 <= r2
    return mask


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    """方形结构元二值膨胀（无 scipy 依赖）。"""
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def test_pose_accuracy(slam_run):
    """终点位姿误差：位置 < 25px，角度 < 0.25 rad。"""
    slam, robot, frames, results = slam_run
    tx, ty, tth = robot.truth
    px, py, pth = slam.pose
    pos_err = math.hypot(px - tx, py - ty)
    th_err = abs((pth - tth + math.pi) % (2 * math.pi) - math.pi)
    assert pos_err < 25.0, f"位置误差 {pos_err:.1f}px 超限（真值 {tx:.0f},{ty:.0f} vs 估计 {px:.0f},{py:.0f}）"
    assert th_err < 0.25, f"角度误差 {th_err:.3f} rad 超限"


def test_slam_beats_odometry(slam_run):
    """SLAM 位姿误差应显著小于纯里程计漂移。"""
    slam, robot, frames, results = slam_run
    tx, ty, _ = robot.truth
    ox, oy, _ = frames[-1].odom.as_tuple()
    err_odom = math.hypot(ox - tx, oy - ty)
    px, py, _ = slam.pose
    err_slam = math.hypot(px - tx, py - ty)
    assert err_slam < max(err_odom * 0.6, 20.0), (
        f"SLAM 误差 {err_slam:.1f}px 未显著优于纯里程计 {err_odom:.1f}px"
    )


def test_wall_recall(slam_run):
    """传感器可及范围内的真值墙面应被 SLAM 地图覆盖 ≥ 60%。

    容差 ±3 格（24px）：v1 无回环，允许全局漂移导致的墙面整体偏移；
    该断言仍能抓住"墙没建出来/建错位置超过一格墙厚"的严重失败。
    """
    slam, robot, frames, results = slam_run
    gt = gt_wall_mask(SimWorld(), slam.grid, band=4.0)
    traj = np.array(slam.trajectory)
    reachable = _reachable_mask(slam.grid, traj)

    slam_walls = _dilate(slam.grid.obstacles(0.65), 3)
    gt_in_reach = gt & reachable
    recall = (gt_in_reach & slam_walls).sum() / max(gt_in_reach.sum(), 1)
    print(f"\n[诊断] 墙面召回率(±3格容差) = {recall:.3f}")
    assert recall > 0.6, f"墙面召回率 {recall:.2f} 过低"


def test_low_false_positive(slam_run):
    """可及区域内 SLAM 误报墙率 < 15%（排除真值墙面 14px 缓冲带）。"""
    slam, robot, frames, results = slam_run
    world = SimWorld()
    gt_wide = gt_wall_mask(world, slam.grid, band=14.0)
    traj = np.array(slam.trajectory)
    reachable = _reachable_mask(slam.grid, traj)

    slam_walls = slam.grid.obstacles(0.65)
    fp_region = reachable & ~gt_wide
    fp_rate = (slam_walls & fp_region).sum() / max((slam_walls & reachable).sum(), 1)
    print(f"\n[诊断] 误报率 = {fp_rate:.3f}（误报 {(slam_walls & fp_region).sum()} 格 / 总 {((slam_walls & reachable).sum())} 格）")
    assert fp_rate < 0.15, f"误报率 {fp_rate:.2f} 过高"


def test_trajectory_continuity(slam_run):
    """位姿轨迹应连续（单步跳变 < 30px），排除匹配发散。"""
    slam, robot, frames, results = slam_run
    traj = np.array(slam.trajectory)
    if len(traj) < 2:
        return
    steps = np.linalg.norm(np.diff(traj[:, :2], axis=0), axis=1)
    assert steps.max() < 30.0, f"轨迹单步最大跳变 {steps.max():.1f}px，可能匹配发散"


def test_pure_odometry_baseline():
    """对照组：关闭扫描匹配时里程计应有可见漂移（验证测试本身的区分度）。"""
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=7))
    frames = drive_to_waypoints(robot, WAYPOINTS)
    slam = Slam2D(SlamConfig(use_match=False))
    for f in frames:
        slam.process(f.odom, f.scan)
    tx, ty, _ = robot.truth
    px, py, _ = slam.pose
    err_odom_only = math.hypot(px - tx, py - ty)
    print(f"\n[诊断] 纯里程计终点漂移 = {err_odom_only:.1f}px")
    assert err_odom_only > 5.0, "纯里程计几乎无漂移，噪声参数可能失效"
