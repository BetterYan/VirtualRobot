"""FUEL-lite TourExplorer 测试。

单元：纯追踪跟随、角速度速率限幅、Dijkstra 距离场、信息增益射线投影、
黑名单 TTL、动态障碍路径失效。
端到端：SimWorld 闭环建图，额外量化行为质量——
    wobble 率 = "运动中强转向指令符号翻转次数 / 每 100px 里程"
    蛇形摆动的直接特征是转向指令高频反号；纯追踪 + 速率限幅应显著低于阈值。
"""

import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from vrobot.control.tour_explorer import (
    TourExplorer,
    _Viewpoint,
    dijkstra_field,
    find_clusters,
)
from vrobot.sensors.models import LaserScan2D
from vrobot.slam.grid_map import OccupancyGrid
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import ROOM_H, ROOM_W, SimParams, SimRobot, SimWorld, gt_wall_mask
from tests.test_slam2d import _dilate


def _free_grid(w: int = 100, h: int = 80) -> OccupancyGrid:
    g = OccupancyGrid(cell_size=8.0, width=w, height=h)
    g._logodds[:] = -1.0        # 全部观测为自由
    return g


def _flat_scan(n: int = 72, r: float = 320.0) -> LaserScan2D:
    return LaserScan2D(
        ranges=np.full(n, r, dtype=np.float32),
        angle_min=-math.pi, angle_inc=2 * math.pi / n, range_max=r,
    )


# ---------------------------------------------------------------------------
# 单元：纯追踪 + 速率限幅
# ---------------------------------------------------------------------------


def test_pure_pursuit_straight():
    """沿路径方向对准直行：角速度 ≈ 0，速度 ≈ 巡航值。"""
    ex = TourExplorer()
    ex._path = [(c, 40) for c in range(10, 70)]   # 正东向直路
    ex._nearest_i = 0
    ex._w_prev = 0.0
    pose = ((12 + 0.5) * 8.0, (40 + 0.5) * 8.0, 0.0)
    v, w = ex._follow(pose)
    assert abs(w) < 0.05, f"直行航向对准时角速度应≈0，实际 {w:.3f}"
    assert v > 55.0, f"直行速度应接近巡航值，实际 {v:.1f}"


def test_pure_pursuit_rate_limit():
    """90° 航向误差：首帧 |w| ≤ 速率限幅，相邻帧变化 ≤ 限幅（蛇形根治点）。"""
    ex = TourExplorer()
    grid = _free_grid()
    ex._path = [(c, 40) for c in range(10, 70)]
    ex._nearest_i = 0
    slam = SimpleNamespace(pose=((12 + 0.5) * 8.0, (40 + 0.5) * 8.0, math.pi / 2), grid=grid)
    scan = _flat_scan()
    w_prev = 0.0
    for k in range(6):
        v, w = ex.update(slam, scan)
        assert abs(w) <= ex.p.angular + 1e-6
        assert abs(w - w_prev) <= ex.p.w_rate_limit + 1e-6, "角速度帧间跳变超限"
        if k == 0:
            assert abs(w) <= ex.p.w_rate_limit + 1e-6, "首帧角速度应受速率限幅约束"
        w_prev = w
        # 位姿按指令推进（模拟 20Hz 执行）
        x, y, th = slam.pose
        th += w * 0.05
        slam.pose = (x + v * math.cos(th) * 0.05, y + v * math.sin(th) * 0.05, th)


# ---------------------------------------------------------------------------
# 单元：距离场 / 增益 / 黑名单 / 动态障碍
# ---------------------------------------------------------------------------


def test_dijkstra_field_detour():
    """墙隔开的两点：距离场应给出绕行代价 > 曼哈顿距离。"""
    h, w = 10, 20
    passable = np.ones((h, w), dtype=bool)
    unknown = np.zeros((h, w), dtype=bool)
    passable[3:7, 10] = False          # 竖墙
    field = dijkstra_field(passable, unknown, (0, 5), unknown_cost=3.0)
    assert np.isfinite(field[5, 19])
    assert field[5, 19] > 19.0, f"绕墙代价应大于曼哈顿 19，实际 {field[5, 19]}"


def test_dijkstra_unknown_cost():
    """未知区通行加价：全未知场的代价 = 格数 × unknown_cost。"""
    passable = np.ones((5, 6), dtype=bool)
    unknown = np.ones((5, 6), dtype=bool)
    field = dijkstra_field(passable, unknown, (0, 2), unknown_cost=3.0)
    assert abs(field[2, 5] - 15.0) < 1e-3


def test_ray_gain():
    """观察点增益：面向未知区 > 0；全图已知 → 0。"""
    ex = TourExplorer()
    occ = np.zeros((40, 60), dtype=bool)
    unknown = np.zeros((40, 60), dtype=bool)
    unknown[:, 30:] = True             # 右半边未知
    assert ex._ray_gain((10, 20), occ, unknown) > 20.0
    assert ex._ray_gain((10, 20), occ, np.zeros_like(unknown)) == 0.0


def test_blacklist_ttl():
    """黑名单条目到期自动解除（旧实现永不过期会误杀好边界）。"""
    ex = TourExplorer()
    ex._frame = 10
    ex._target_vp = _Viewpoint(cell=(3, 4), cluster_id=0, gain=50.0, cost=30.0)
    ex._drop_target("测试")
    assert (3, 4) in ex._blacklist
    ex._frame = 10 + ex.p.blacklist_ttl + 1
    ex._purge_blacklist()
    assert (3, 4) not in ex._blacklist


def test_path_valid_dynamic_obstacle():
    """路径穿过的格被新观测为占据 → 路径立即失效（动态障碍响应）。"""
    ex = TourExplorer()
    grid = _free_grid()
    ex._path = [(c, 40) for c in range(10, 70)]
    assert ex._path_valid(grid)
    grid._logodds[40, 30] = 4.0        # 新观测到的动态障碍
    assert not ex._path_valid(grid)


def test_async_plan_thread():
    """后台规划：提交 → 线程产出路径 → 主线程收集；20Hz 控制不被阻塞。"""
    ex = TourExplorer(async_plan=True)
    try:
        grid = _free_grid()
        grid._logodds[:, 30:] = 0.0        # 右半未知 → frontier
        slam = SimpleNamespace(pose=(44.0, 164.0, 0.0), grid=grid)
        scan = _flat_scan()

        t0 = time.perf_counter()
        v, w = ex.update(slam, scan)       # 首帧：提交后台规划，立即返回不阻塞
        assert time.perf_counter() - t0 < 0.1, "update 应立即返回（规划在后台）"
        assert ex._busy, "应处于后台规划中"
        assert v == 0.0, "规划期间应原地等待"

        deadline = time.time() + 5.0
        while ex._busy and time.time() < deadline:
            time.sleep(0.01)
        assert not ex._busy, "后台规划 5s 内未完成"

        v, w = ex.update(slam, scan)       # 收集结果
        assert ex._path, "后台规划应产出路径"
        assert not ex.done
        assert v > 0, "路径就绪后应开始跟随"
    finally:
        ex.close()


def test_find_clusters_basic():
    """一条未知带 → 单簇，成员数 ≥ min_size。"""
    lo = np.full((40, 60), 0.0, dtype=np.float32)    # 右半未知（log-odds 0）
    lo[:, :30] = -2.0                                 # 左半已知自由
    clusters = find_clusters(lo, -0.3, 0.62, 4)
    assert len(clusters) == 1
    assert len(clusters[0].cells) >= 4


def test_bootstrap_empty_map():
    """cartographer 桥接同步前的空白地图：直行探测而非宣布完成。

    回归：真实链路首帧地图全空白 → 空白图上定义上无 frontier →
    旧逻辑误判"无边界 → 建图完成" → 机器人原地不动。
    """
    ex = TourExplorer(async_plan=False)
    grid = OccupancyGrid(cell_size=8.0, width=100, height=80)   # log-odds 全 0 = 全未知
    slam = SimpleNamespace(pose=(100.0, 90.0, 0.0), grid=grid)
    scan = _flat_scan()
    v, w = ex.update(slam, scan)
    assert not ex.done, "空白地图不应宣告建图完成"
    assert ex._bootstrap_until > ex._frame, "应进入空图探测期"
    assert v > 0, "探测期应直行积累观测"

    # 探测期结束后再次规划：地图仍空白 → 继续探测；观测出现 → 正常规划
    grid._logodds[:30, :] = -2.0    # 模拟观测到自由区
    for _ in range(70):
        ex.update(slam, scan)
    assert not ex.done
    assert ex._bootstrap_until <= ex._frame


# ---------------------------------------------------------------------------
# 端到端：SimWorld 闭环
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def tour_run():
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11))
    slam = Slam2D(SlamConfig(use_match=True))
    # e2e 用同步规划保证确定性；异步机制单独在 test_async_plan_thread 验证
    explorer = TourExplorer(cell_size=slam.grid.cell_size, async_plan=False)

    max_steps = 12000   # 20Hz → 600s 仿真时长上限（与混合探索同量级）
    cmds: list[tuple[float, float]] = []
    truth: list[tuple[float, float, float]] = [robot.truth]
    last_frame = None
    done_step = None
    for k in range(max_steps):
        if last_frame is not None:
            slam.process(last_frame.odom, last_frame.scan)
        if last_frame is None:
            cmd = (0.0, 0.0)
        else:
            cmd = explorer.update(slam, last_frame.scan)
        cmds.append(cmd)
        truth.append(robot.truth)
        if explorer.done:
            done_step = k
            break
        last_frame = robot.step(*cmd)

    return world, robot, slam, explorer, done_step, cmds, truth


def test_tour_completes(tour_run):
    """TourExplorer 应在限定步数内自行收敛。"""
    world, robot, slam, explorer, done_step, cmds, truth = tour_run
    assert done_step is not None, "TourExplorer 未在限定步数内完成"
    assert explorer.done
    print(f"\n[诊断] 完成于第 {done_step} 步（≈{done_step * 0.05:.0f}s），规划 {explorer.n_plans} 次")


def _wobble_rate(cmds: list[tuple[float, float]], truth: list[tuple[float, float, float]]) -> float:
    """运动中强转向指令符号翻转次数 / 每 100px 里程（蛇形摆动量化）。"""
    flips = 0
    last_sign = 0
    dist = 0.0
    for i in range(1, min(len(cmds), len(truth))):
        v, w = cmds[i]
        dist += math.hypot(truth[i][0] - truth[i - 1][0], truth[i][1] - truth[i - 1][1])
        if v < 20.0 or abs(w) < 0.3:
            continue
        s = 1 if w > 0 else -1
        if last_sign != 0 and s != last_sign:
            flips += 1
        last_sign = s
    return flips * 100.0 / max(dist, 1.0)


def test_tour_behavior_quality(tour_run):
    """行为质量：转向指令反号频率（每 100px）应低于蛇形阈值。"""
    world, robot, slam, explorer, done_step, cmds, truth = tour_run
    rate = _wobble_rate(cmds, truth)
    print(f"[诊断] wobble 率 = {rate:.2f} 次翻转 / 100px")
    assert rate < 5.0, f"转向指令反号过频（{rate:.2f}/100px），存在蛇形摆动"


def test_tour_wall_recall(tour_run):
    """全屋真值墙面召回率 ≥ 85%。"""
    world, robot, slam, explorer, done_step, cmds, truth = tour_run
    gt = gt_wall_mask(world, slam.grid, band=4.0)
    walls = _dilate(slam.grid.obstacles(0.65), 3)
    recall = (gt & walls).sum() / max(gt.sum(), 1)
    print(f"[诊断] 全屋墙面召回率 = {recall:.3f}")
    assert recall > 0.85


def test_tour_coverage(tour_run):
    """已确认自由区（±2 格容差）覆盖房间自由面积 ≥ 80%。"""
    world, robot, slam, explorer, done_step, cmds, truth = tour_run
    lo = slam.grid._logodds
    cs = slam.grid.cell_size
    slam_free = _dilate(lo <= -0.3, 2)
    free_gt = np.zeros_like(slam_free)
    for row in range(slam.grid.height):
        for col in range(slam.grid.width):
            cx, cy = (col + 0.5) * cs, (row + 0.5) * cs
            if not (0.0 <= cx <= ROOM_W and 0.0 <= cy <= ROOM_H):
                continue
            inside = any(
                (x0 + cs) <= cx <= (x0 + w - cs) and (y0 + cs) <= cy <= (y0 + h - cs)
                for x0, y0, w, h in world.rects
            )
            if not inside:
                free_gt[row, col] = True
    cover = (slam_free & free_gt).sum() / max(free_gt.sum(), 1)
    print(f"[诊断] 自由空间覆盖率(±2格) = {cover:.3f}")
    assert cover > 0.80


def test_tour_final_pose(tour_run):
    """长航时终点位姿误差 < 35px / 0.35rad。"""
    world, robot, slam, explorer, done_step, cmds, truth = tour_run
    tx, ty, tth = robot.truth
    px, py, pth = slam.pose
    pos_err = math.hypot(px - tx, py - ty)
    th_err = abs((pth - tth + math.pi) % (2 * math.pi) - math.pi)
    print(f"[诊断] 终点误差 = {pos_err:.1f}px / {math.degrees(th_err):.1f}°")
    assert pos_err < 35.0
    assert th_err < 0.35
