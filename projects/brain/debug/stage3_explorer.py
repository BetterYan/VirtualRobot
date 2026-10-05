"""Stage 3: 探索闭环离线测试（TourExplorer + SLAM 引擎 + 仿真世界）。

隔离目标：完整闭环（探索器 + SLAM + 仿真物理），但不含 Godot / 网络 / 可视化窗口。
分别用 cartographer 与 builtin 引擎跑同一探索任务，对比：
  - 自由格增长曲线（建图是否持续推进）
  - 估计位姿 vs 真值漂移（px/100px）
  - 是否触发完成/卡死异常

运行（可指定帧数上限，默认 1500）:
  .venv/Scripts/python.exe debug_stage3_explorer.py [max_frames]
"""

import math
import sys

from pathlib import Path
_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

from vrobot.control.tour_explorer import TourExplorer
from vrobot.slam.cartographer_engine import CartographerEngineConfig, CartographerSLAM2D
from vrobot.slam.slam2d import Slam2D, SlamConfig
from vrobot.viz.fast_map_viewer import create_map_viewer
from tests.sim_world import SimParams, SimRobot, SimWorld


def run(engine: str, max_frames: int, viewer: MapViewer | None) -> dict:
    world = SimWorld()
    # 144 线，与 Godot 实际配置一致
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11, beams=144))
    if engine == "cartographer":
        slam = CartographerSLAM2D(CartographerEngineConfig(scale=0.05, map_sync_interval=3))
    else:
        slam = Slam2D(SlamConfig(use_match=True))
    slam.pose = (100.0, 90.0, 0.0)   # 首帧处理前的初始位姿（否则 (0,0,0) 会污染漂移统计）
    ex = TourExplorer(cell_size=slam.grid.cell_size, async_plan=False)

    truth_path = 0.0
    prev = robot.truth
    free_curve: list[int] = []
    drift_hist: list[float] = []
    done_at = None
    events: list[str] = []
    last_frame = None

    for k in range(max_frames):
        if k > 0:
            slam.process(last_frame.odom, last_frame.scan)
        cmd = ex.update(slam, last_frame.scan) if last_frame else (0.0, 0.0)
        if viewer and viewer.enabled and k % 3 == 0:
            viewer.update(slam, robot.truth)   # 地图 + 估计轨迹 + 真值(绿+)
        lo = slam.grid._logodds
        free_curve.append(int((lo <= -0.3).sum()))
        t = robot.truth
        truth_path += math.hypot(t[0] - prev[0], t[1] - prev[1])
        prev = t
        drift = math.hypot(slam.pose[0] - t[0], slam.pose[1] - t[1])
        drift_hist.append(drift)
        if ex.done and done_at is None:
            done_at = k
            events.append(f"done@{k}")
        if drift > 250 and "diverge" not in events:
            events.append(f"diverge@{k}(drift={drift:.0f}px)")
        last_frame = robot.step(*cmd)
        if k % 300 == 0:
            events.append(f"f{k}:free={free_curve[-1]},drift={drift:.0f}")

    lo = slam.grid._logodds
    free = int((lo <= -0.3).sum())
    drift_per_100 = drift_hist[-1] * 100.0 / max(truth_path, 1.0)
    return {
        "engine": engine,
        "free": free,
        "occ": int((lo >= 0.62).sum()),
        "truth_path": truth_path,
        "drift": drift_hist[-1],
        "drift_per_100": drift_per_100,
        "done_at": done_at,
        "events": events,
        "free_curve": free_curve,
    }


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    view = "--no-view" not in sys.argv
    max_frames = int(args[0]) if args else 1500
    print(f"Stage 3: 探索闭环离线对比（max_frames={max_frames}, beams=144, "
          f"可视化={'开' if view else '关'}）")
    for engine in ("cartographer", "builtin"):
        viewer = create_map_viewer(enabled=view, cell_size=8.0)
        r = run(engine, max_frames, viewer)
        growth = [r["free_curve"][i] for i in range(0, len(r["free_curve"]), max(1, len(r["free_curve"]) // 8))]
        print(f"\n=== 引擎 {engine} ===")
        print(f"  自由格: {r['free']}  占据格: {r['occ']}")
        print(f"  真值里程: {r['truth_path']:.0f}px  终点漂移: {r['drift']:.0f}px "
              f"({r['drift_per_100']:.1f}px/100px)")
        print(f"  自由格曲线: {growth}")
        print(f"  事件: {r['events']}  完成于: {r['done_at']}")
        verdict = []
        verdict.append(("建图推进（free > 2500）", r["free"] > 2500))
        verdict.append(("位姿漂移 < 15px/100px", r["drift_per_100"] < 15.0))
        verdict.append(("无发散事件", not any(e.startswith("diverge") for e in r["events"])))
        for label, passed in verdict:
            print(f"  [{'PASS' if passed else 'FAIL'}] {label}")
        if viewer and viewer.enabled:
            input(f"[{engine}] 查看完地图后按回车继续...")
        viewer.close()
    print("\n说明: builtin 漂移小是参照基线; cartographer 漂移大 → 问题在引擎/约定层(Stage 2/4 深挖)")


if __name__ == "__main__":
    main()
