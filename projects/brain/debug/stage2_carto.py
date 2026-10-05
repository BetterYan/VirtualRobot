"""Stage 2: cartographer 引擎契约测试（合成数据激励，无需 Godot / 网络 / 窗口）。

隔离目标：只测 CartographerSLAM2D 的 输入→输出 是否满足设计要求。
激励：理想差速驱动（完美里程计）+ SimWorld 解析射线扫描（144 线）。

用例：
  T1 静止        60 帧不动          → est 位姿应贴着出生点，航向漂移 < 2°
  T2 直线        100 帧 (60, 0)     → est 前进 ≈300px，横向偏差 < 20px
  T3 原地旋转    72 帧 (0, +1.0)    → est 航向 +206°±15°（符号 = 旋转方向约定）
  T4 正方形闭环  4×(200px+90°转)    → est 回到出生点 ±60px，地图自由格 > 1500

运行: .venv/Scripts/python.exe debug_stage2_carto.py
"""

import math
import sys

from pathlib import Path
_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

import numpy as np

from vrobot.slam.cartographer_engine import CartographerEngineConfig, CartographerSLAM2D
from vrobot.viz.fast_map_viewer import create_map_viewer
from tests.sim_world import SimWorld

from synth import DT, SynthBot, report, wrap_deg


def run_case(name: str, segs: list[tuple[float, float, int]],
             checks, start=(100.0, 90.0, 0.0),
             viewer: MapViewer | None = None) -> bool:
    world = SimWorld()
    bot = SynthBot(world, start)
    eng = CartographerSLAM2D(CartographerEngineConfig(scale=0.05, map_sync_interval=3))
    frames = 0
    free = 0
    for v, w, n in segs:
        for _ in range(n):
            odom, scan = bot.step(v, w)
            eng.process(odom, scan)
            frames += 1
            if viewer and viewer.enabled and frames % 3 == 0:
                viewer.update(eng, (bot.x, bot.y, bot.th))
    ex, ey, eth = eng.pose
    lo = eng.grid._logodds
    free = int((lo <= -0.3).sum())
    occ = int((lo >= 0.62).sum())
    ok = report(name, checks(bot, eng, (ex, ey, eth), free, occ))
    if viewer and viewer.enabled:
        input(f"[{name}] 查看完地图快照后按回车继续...")
    return ok


def main() -> None:
    view = "--no-view" not in sys.argv
    viewer = create_map_viewer(enabled=view, cell_size=8.0)
    ok_all = True

    # T1 静止
    def t1(bot, eng, est, free, occ):
        ex, ey, eth = est
        d = math.hypot(ex - bot.spawn[0], ey - bot.spawn[1])
        dth = abs(wrap_deg(eth - bot.spawn[2]))
        return [
            ("位置漂移 < 5px", d < 5.0, f"{d:.1f}px"),
            ("航向漂移 < 2°", dth < 2.0, f"{dth:.1f}°"),
            ("静止观测到自由区属正常（周围 360° 可见），只 sanity 上限", free < 3000,
             f"free={free}"),
        ]
    ok_all &= run_case("T1 静止 60 帧", [(0.0, 0.0, 60)], t1, viewer=viewer)

    # T2 直线
    def t2(bot, eng, est, free, occ):
        ex, ey, eth = est
        ix, iy, ith = bot.x, bot.y, bot.th           # 理想终点
        return [
            ("纵向前进 300±30px", abs((ex - bot.spawn[0]) - 300.0) < 30.0,
             f"est_x-spawn_x={ex - bot.spawn[0]:.1f}px"),
            ("横向偏差 < 20px", abs(ey - iy) < 20.0, f"{ey - iy:.1f}px"),
            ("航向误差 < 5°", abs(wrap_deg(eth - ith)) < 5.0,
             f"{wrap_deg(eth - ith):.1f}°"),
            ("自由格 > 200", free > 200, f"free={free}"),
        ]
    ok_all &= run_case("T2 直线 100 帧 (60,0)", [(60.0, 0.0, 100)], t2, viewer=viewer)

    # T3 原地旋转（连续角记录：wrap 会把 ±360° 藏掉，必须逐帧解缠）
    def t3(bot, eng, est, free, occ):
        # 重新驱动一遍并逐帧解缠航向
        world = SimWorld()
        bot2 = SynthBot(world, bot.spawn)
        eng2 = CartographerSLAM2D(CartographerEngineConfig(scale=0.05, map_sync_interval=3))
        unwrapped = 0.0
        prev = None
        for _ in range(72):
            odom, scan = bot2.step(0.0, 1.0)
            res = eng2.process(odom, scan)
            eth = res.pose[2]
            if prev is None:
                prev = eth
            unwrapped += math.degrees((eth - prev + math.pi) % (2 * math.pi) - math.pi)
            prev = eth
        ideal = math.degrees(1.0 * 72 * DT)           # +206.3°
        err = unwrapped - ideal
        return [
            ("旋转方向为正（约定校验）", unwrapped > 90.0, f"连续角 {unwrapped:.1f}°"),
            ("旋转幅值误差 < 8°（当前引擎 ≈ -27°，已知缺陷）", abs(err) < 8.0,
             f"{err:+.1f}°（欠转 {(err / ideal) * 100:.1f}%）"),
        ]
    ok_all &= run_case("T3 原地旋转 72 帧 (0,+1.0)", [(0.0, 1.0, 72)], t3, viewer=viewer)

    # T4 正方形闭环
    segs = []
    for _ in range(4):
        segs.append((60.0, 0.0, 67))      # 200px
        segs.append((0.0, 1.5708, 20))    # 90°
    def t4(bot, eng, est, free, occ):
        ex, ey, eth = est
        d = math.hypot(ex - bot.spawn[0], ey - bot.spawn[1])
        return [
            ("闭环误差 < 60px", d < 60.0, f"{d:.1f}px"),
            ("自由格 > 1500", free > 1500, f"free={free} occ={occ}"),
        ]
    ok_all &= run_case("T4 正方形闭环 (800px + 4×90°)", segs, t4, viewer=viewer)

    if viewer and viewer.enabled:
        input("查看完最终地图后按回车退出...")
        viewer.close()
    print(f"\n{'=' * 50}\nStage 2 总结: {'全部 PASS' if ok_all else '存在 FAIL —— cartographer 契约不满足'}")
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
