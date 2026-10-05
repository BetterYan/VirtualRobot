"""TourExplorer 行为诊断：采样输出环游状态 + 慢调用计时，定位停滞根因。"""

import sys
import time

import numpy as np

from pathlib import Path
_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_root))          # brain 根: tests 包
sys.path.insert(0, str(_root / "src"))  # vrobot 包

from vrobot.control import tour_explorer as te
from vrobot.control.tour_explorer import TourExplorer
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import SimParams, SimRobot, SimWorld

# 给重操作包计时探针
_build_tour = TourExplorer._build_tour
_plan = TourExplorer._plan


def timed_build(self, grid, pose):
    t0 = time.perf_counter()
    _build_tour(self, grid, pose)
    dt = time.perf_counter() - t0
    if dt > 0.2:
        print(f"    [slow] _build_tour {dt:.2f}s @frame {self._frame}")
    self._build_dt = dt


def timed_plan(self, grid, pose, _depth=0):
    t0 = time.perf_counter()
    _plan(self, grid, pose, _depth)
    dt = time.perf_counter() - t0
    if dt > 0.2:
        print(f"    [slow] _plan {dt:.2f}s @frame {self._frame} depth={_depth}")
    self._plan_dt = dt


TourExplorer._build_tour = timed_build
TourExplorer._plan = timed_plan

world = SimWorld()
robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=11))
slam = Slam2D(SlamConfig(use_match=True))
ex = TourExplorer(cell_size=slam.grid.cell_size)

last = None
prev_plans = 0
for k in range(11500):
    if last is not None:
        slam.process(last.odom, last.scan)
    if last is None:
        cmd = (0.0, 0.0)
    else:
        t0 = time.perf_counter()
        cmd = ex.update(slam, last.scan)
        dt = time.perf_counter() - t0
        if dt > 0.5:
            print(f"    [slow] update {dt:.2f}s @frame {k}")
    if ex.done:
        print(f"frame {k}: DONE")
        break
    if k % 50 == 0:
        lo = slam.grid._logodds
        u = int(((lo > -0.3) & (lo < 0.62)).sum())
        d_plans = ex.n_plans - prev_plans
        prev_plans = ex.n_plans
        print(
            f"frame {k:5d} pose=({slam.pose[0]:6.0f},{slam.pose[1]:6.0f}) "
            f"unknown={u:6d} tour={len(ex._tour):2d} ti={ex._tour_i:2d} "
            f"path={len(ex._path):3d} ni={ex._nearest_i:3d} plans=+{d_plans} "
            f"bl={len(ex._blacklist):2d} escape={ex._escape_until > ex._frame} "
            f"cmd=({cmd[0]:5.1f},{cmd[1]:5.2f})"
        )
    last = robot.step(*cmd)
