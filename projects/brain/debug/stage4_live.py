"""Stage 4: slam_node 全链路插桩（真实 Godot + 完整控制逻辑，无窗口）。

隔离目标：真实 Godot 数据流下，逐帧记录 控制指令 / 估计位姿 / 真值 / 地图统计 /
探索器状态，并周期性保存地图 PNG —— 用于离线定位发散/停摆发生的精确时刻。

输出：
  debug_out/frames.csv   逐帧记录（excel/数据分析可直接打开）
  debug_out/map_*.png    每 300 帧一张地图快照（含轨迹）
  控制台: 关键事件日志

运行（Godot 运行中、HUD 切 auto）:
  .venv/Scripts/python.exe debug_stage4_live.py [max_frames，默认 3000]
"""

import asyncio
import csv
import math
import sys
import time
from pathlib import Path

from pathlib import Path
_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from vrobot.comm.server import SimServer
from vrobot.control.tour_explorer import TourExplorer
from vrobot.slam.cartographer_engine import CartographerEngineConfig, CartographerSLAM2D
from vrobot.viz.fast_map_viewer import create_map_viewer

OUT = Path(__file__).parent / "debug_out"


async def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    view = "--no-view" not in sys.argv
    max_frames = int(args[0]) if args else 3000
    OUT.mkdir(exist_ok=True)

    server = SimServer("127.0.0.1", 9094)
    slam = CartographerSLAM2D(CartographerEngineConfig(scale=0.05, map_sync_interval=3))
    ex = TourExplorer(async_plan=True)   # 与生产配置一致（异步规划）
    viewer = create_map_viewer(enabled=view, cell_size=slam.grid.cell_size)
    await server.start()
    print(f"probe listening... max_frames={max_frames}, 输出目录 {OUT}", flush=True)

    csv_file = open(OUT / "frames.csv", "w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["frame", "t", "cmd_v", "cmd_w", "est_x", "est_y", "est_th_deg",
                     "gt_x", "gt_y", "gt_th_deg", "free", "occ", "busy",
                     "tour_len", "tour_i", "blacklist", "escape", "bootstrap"])

    seq = 0
    n = 0
    t0 = time.time()
    try:
        while n < max_frames:
            frame = server.consume_frame()
            if frame is not None:
                res = slam.process(frame.odom, frame.scan)
                n += 1
                v, w = ex.update(slam, frame.scan)
                server.send_cmd_vel(v, w, seq)
                seq += 1

                lo = slam.grid._logodds
                free = int((lo <= -0.3).sum())
                occ = int((lo >= 0.62).sum())
                gt = server.ground_truth
                writer.writerow([
                    n, round(time.time() - t0, 2), round(v, 1), round(w, 2),
                    round(slam.pose[0], 1), round(slam.pose[1], 1),
                    round(math.degrees(slam.pose[2]), 1),
                    round(gt.x, 1) if gt else "", round(gt.y, 1) if gt else "",
                    round(math.degrees(gt.theta), 1) if gt else "",
                    free, occ, int(ex._busy), len(ex._tour), ex._tour_i,
                    len(ex._blacklist), int(ex._escape_until > ex._frame),
                    int(ex._bootstrap_until > ex._frame),
                ])

                if viewer.enabled and n % 3 == 0:
                    viewer.update(slam, (gt.x, gt.y, gt.theta) if gt else None)

                if n % 300 == 0:
                    prob = slam.grid.prob()
                    fig, ax = plt.subplots(figsize=(8, 6))
                    ax.imshow(prob, cmap="gray_r", vmin=0, vmax=1)
                    traj = np.array(slam.trajectory) / slam.grid.cell_size - 0.5
                    ax.plot(traj[:, 0], traj[:, 1], "r-", lw=1)
                    ax.set_title(f"frame={n} free={free} occ={occ}")
                    fig.savefig(OUT / f"map_{n:05d}.png", dpi=90, bbox_inches="tight")
                    plt.close(fig)
                    print(f"frame {n}: free={free} occ={occ} "
                          f"est=({slam.pose[0]:.0f},{slam.pose[1]:.0f}) "
                          f"tour={len(ex._tour)} bl={len(ex._blacklist)}", flush=True)
            await asyncio.sleep(0.002)
    finally:
        csv_file.close()
        await server.stop()
        if viewer.enabled:
            input("查看完最终地图后按回车退出...")
        viewer.close()
        print("done. 用 excel/pandas 打开 debug_out/frames.csv 分析", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
