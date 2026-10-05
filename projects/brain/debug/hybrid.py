"""临时调试脚本：插桩 HybridExplorer，量化沿边摆动与覆盖阶段行为。

用法: cd projects/brain && uv run python debug/hybrid.py
"""

import logging
import math
import sys
from pathlib import Path

_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

logging.basicConfig(level=logging.INFO, format="%(message)s")

import numpy as np

from vrobot.control.hybrid_explorer import COVERAGE, FRONTIER, WALL_FOLLOW, HybridExplorer
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import SimParams, SimRobot, SimWorld

world = SimWorld()
robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11, beams=144))  # Godot 同款 144 线
slam = Slam2D(SlamConfig(use_match=True))
ex = HybridExplorer(cell_size=slam.grid.cell_size)

# ---- 插桩 wall_follow ----
orig_follow = ex._update_follow
st = {"follow_n": 0, "corner_trig": 0, "abs_w_sum": 0.0, "abs_w_max": 0.0,
      "d_samples": [], "v_mode_changes": 0, "last_v": None, "trig_poses": []}

def patched_follow(pose, scan):
    cmd = orig_follow(pose, scan)
    if ex.phase == WALL_FOLLOW:
        st["follow_n"] += 1
        st["abs_w_sum"] += abs(cmd[1])
        st["abs_w_max"] = max(st["abs_w_max"], abs(cmd[1]))
        if cmd[0] == 0.0 and cmd[1] > 0:      # 转角指令（原地正转）
            st["corner_trig"] += 1
            st["trig_poses"].append((round(pose[0]), round(pose[1])))
        if st["last_v"] is not None and abs(cmd[0] - st["last_v"]) > 15:
            st["v_mode_changes"] += 1
        st["last_v"] = cmd[0]
        fit = ex._wall_fit(scan)
        if fit and len(st["d_samples"]) < 100000:
            st["d_samples"].append(fit[0])
    return cmd

ex._update_follow = patched_follow

# ---- 插桩 coverage ----
orig_cov = ex._update_coverage
cov_st = {"n": 0, "adv": 0, "last_i": 0}

def patched_cov(slam_, pose, scan):
    cmd = orig_cov(slam_, pose, scan)
    if ex.phase == COVERAGE:
        cov_st["n"] += 1
        if ex._cov_i != cov_st["last_i"]:
            cov_st["adv"] += 1
            cov_st["last_i"] = ex._cov_i
    return cmd

ex._update_coverage = patched_cov

last_frame = None
phase_marks = []
for k in range(9000):
    if last_frame is not None:
        slam.process(last_frame.odom, last_frame.scan)
    cmd = (0.0, 0.0) if last_frame is None else ex.update(slam, last_frame.scan)
    if len(ex.visited_phases) and (not phase_marks or phase_marks[-1][1] != ex.phase):
        phase_marks.append((k, ex.phase))
        print(f"[frame {k}] → {ex.phase}  pose=({slam.pose[0]:.0f},{slam.pose[1]:.0f})")
    if ex.done:
        print(f"[frame {k}] DONE")
        break
    last_frame = robot.step(*cmd)

print("\n===== wall_follow 统计 =====")
print(f"沿边帧数: {st['follow_n']}, 转角触发次数: {st['corner_trig']}"
      f"（矩形房间合理值 ≈ 4 外墙 + 家具角，<20 正常）")
print(f"平均 |w| = {st['abs_w_sum'] / max(st['follow_n'],1):.3f} rad/s, 峰值 = {st['abs_w_max']:.2f}")
print(f"速度突变次数(Δv>15): {st['v_mode_changes']}")
d = np.array(st["d_samples"])
if len(d):
    print(f"墙垂距: mean={d.mean():.1f} std={d.std():.1f} min={d.min():.1f} max={d.max():.1f}")
if st["trig_poses"]:
    print("转角触发位置（前 40 个）:")
    print("  " + ", ".join(f"({x},{y})" for x, y in st["trig_poses"][:40]))

print("\n===== coverage 统计 =====")
print(f"覆盖帧数: {cov_st['n']}, 路点推进次数: {cov_st['adv']}, 最终 _cov_i = {ex._cov_i}, "
      f"路点总数 = {len(ex._cov_wps)}")
print(f"阶段演进: {' → '.join(ex.visited_phases)}")

# ---- 渲染轨迹分段图（按相位着色）----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

traj = np.array(slam.trajectory)  # (N,3)
fig, ax = plt.subplots(figsize=(9, 7))
ax.plot(traj[:, 0], traj[:, 1], "-", color="0.85", lw=1, zorder=1)
# 按阶段着色重绘
segs = {}
for i, ph in enumerate(phase_marks):
    end = phase_marks[i + 1][0] if i + 1 < len(phase_marks) else len(traj) - 1
    segs[ph[1]] = (ph[0], end)
colors = {"seek_wall": "tab:green", "wall_follow": "tab:red",
          "coverage": "tab:blue", "frontier": "tab:orange"}
for ph, (a, b) in segs.items():
    b = min(b + 1, len(traj))
    ax.plot(traj[a:b, 0], traj[a:b, 1], "-", color=colors.get(ph, "k"), lw=1.2,
            label=f"{ph} [{a}-{b}]", zorder=2)
ax.set_aspect("equal"); ax.legend(fontsize=8); ax.set_title("hybrid trajectory")
ax.invert_yaxis()
fig.savefig("debug_traj.png", dpi=110, bbox_inches="tight")
print("轨迹图已保存: debug_traj.png")
