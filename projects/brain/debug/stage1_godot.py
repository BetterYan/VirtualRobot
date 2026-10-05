"""Stage 1: Godot 数据通路检查（需要 Godot 运行并切换到 auto 模式）。

隔离目标：只测 Godot 前端 → WS 数据链路，不涉及 SLAM/探索器。
检查项：
  A. hello 参数与扫描帧一致性（线数、角度约定、有效率）
  B. 静止扫描稳定性（相邻 1s 两帧 range 向量差）
  C. 平移方向约定：指令 (60, 0) 2s → 真值位移应沿航向 (点积 > 0)
  D. 旋转方向约定：指令 (0, +1.5) 1s → 真值 theta 应增大
  E. 里程计 vs 真值一致性（位置差 < 10px / 航向差 < 5°）

运行（Godot 场景运行中、HUD 切 auto 后）:
  .venv/Scripts/python.exe debug_stage1_godot.py
"""

import asyncio
import math
import sys
import time

from pathlib import Path
_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

import numpy as np

from vrobot.comm.server import SimServer

DT = 0.05


def heading_dot(ax, ay, gx, gy, gth) -> float:
    """位移向量在航向方向上的投影（>0 = 沿航向前进）。"""
    return (gx - ax) * math.cos(gth) + (gy - ay) * math.sin(gth)


async def main() -> None:
    server = SimServer("127.0.0.1", 9094)
    await server.start()
    print("等待 Godot 连接（确保 Godot HUD 已切 auto）...", flush=True)
    while not server.connected.is_set():
        await asyncio.sleep(0.1)

    results: list[tuple[str, bool, str]] = []

    # ---- A. 静止采集 + 参数检查 ----
    frame = server.consume_frame()
    while frame is None:
        await asyncio.sleep(0.01)
        frame = server.consume_frame()
    scan0 = frame.scan
    valid = (scan0.ranges > 0) & (scan0.ranges < scan0.range_max)
    results.append((
        "扫描有效率 > 30%",
        valid.mean() > 0.3,
        f"{valid.mean() * 100:.0f}% ({int(valid.sum())}/{len(scan0.ranges)} 线)",
    ))
    n_from_inc = round(2 * math.pi / scan0.angle_inc)
    results.append((
        "angle_inc 与线数自洽",
        n_from_inc == len(scan0.ranges),
        f"inc 推得 {n_from_inc} 线, 实际 {len(scan0.ranges)} 线",
    ))

    # ---- B. 静止稳定性（1 秒） ----
    r0 = scan0.ranges.copy()
    await asyncio.sleep(1.0)
    frame = server.consume_frame()
    while frame is None:
        await asyncio.sleep(0.01)
        frame = server.consume_frame()
    d = np.abs(frame.scan.ranges - r0)
    d = d[(frame.scan.ranges > 0) & (r0 > 0)]
    results.append((
        "静止扫描稳定（1s 内中位差 < 6px）",
        float(np.median(d)) < 6.0,
        f"中位差 {np.median(d):.1f}px, 95 分位 {np.quantile(d, 0.95):.1f}px",
    ))

    # ---- C. 平移方向约定：(60,0) 2s ----
    gt0 = server.ground_truth
    for _ in range(40):  # 2s
        server.send_cmd_vel(60.0, 0.0, 0)
        await asyncio.sleep(DT)
    gt1 = server.ground_truth
    if gt0 and gt1:
        fwd = heading_dot(gt0.x, gt0.y, gt1.x, gt1.y, gt0.theta)
        results.append((
            "平移沿航向（点积 > 0）",
            fwd > 30.0,
            f"航向投影 {fwd:.0f}px, 位移 {math.hypot(gt1.x - gt0.x, gt1.y - gt0.y):.0f}px",
        ))
        # ---- E. odom vs gt ----
        od = frame.odom
        pos_d = math.hypot(od.x - gt1.x, od.y - gt1.y)
        results.append((
            "里程计 vs 真值 位置差 < 10px",
            pos_d < 10.0,
            f"{pos_d:.1f}px",
        ))

    # ---- D. 旋转方向约定：(0, +1.5) 1s ----
    gt2 = server.ground_truth
    for _ in range(20):  # 1s
        server.send_cmd_vel(0.0, 1.5, 0)
        await asyncio.sleep(DT)
    gt3 = server.ground_truth
    if gt2 and gt3:
        dth = math.degrees((gt3.theta - gt2.theta + math.pi) % (2 * math.pi) - math.pi)
        results.append((
            "正角速度 → gt theta 增大（旋转约定）",
            dth > 40.0,
            f"{dth:.1f}°（1s × 1.5rad/s 理论 ≈ 86°）",
        ))

    await server.stop()

    print("\n=== Stage 1: Godot 数据通路 ===")
    ok = True
    for label, passed, detail in results:
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}: {detail}")
    print(f"\nStage 1 总结: {'全部 PASS' if ok else '存在 FAIL —— Godot 数据/约定有问题'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    asyncio.run(main())
