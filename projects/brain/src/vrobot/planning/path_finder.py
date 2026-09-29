"""栅格 A* 路径规划（占据栅格上的 4 邻接寻路）。

供自主探索导航使用：起点=机器人所在格，目标=frontier 附近的自由格。
未知区可通行但代价更高（鼓励走已确认的空地），膨胀后的障碍不可通行。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass

import numpy as np


@dataclass
class AStarResult:
    path: list[tuple[int, int]]      # (col, row) 序列，含起终点；不可达为空
    cost: float


def inflate(mask: np.ndarray, cells: int) -> np.ndarray:
    """二值掩码方形膨胀（无 scipy 依赖）。cells=0 时返回副本。"""
    out = mask.copy()
    for _ in range(cells):
        prev = out
        out = prev.copy()
        out[1:, :] |= prev[:-1, :]
        out[:-1, :] |= prev[1:, :]
        out[:, 1:] |= prev[:, :-1]
        out[:, :-1] |= prev[:, 1:]
    return out


def make_passable(
    logodds: np.ndarray,
    free_below: float = -0.3,
    occupied_above: float = 0.62,
    inflate_cells: int = 2,
) -> tuple[np.ndarray, np.ndarray]:
    """由 log-odds 图生成 (passable, unknown)。

    passable: bool 图，严格膨胀——可通行格中心距占据格中心 ≥ inflate_cells+1 格，
    保证路径离墙面 ≥ 约 18px（机器人半径 14px + 余量）。
    起点若落在膨胀区内由调用方 carve 逃逸气泡（机器人已物理存在于该处）。
    """
    free = logodds <= free_below
    occupied = logodds >= occupied_above
    unknown = (~free) & (~occupied)
    blocked = inflate(occupied, inflate_cells)
    passable = ~blocked
    return passable, unknown


def astar(
    passable: np.ndarray,
    unknown: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    unknown_cost: float = 3.0,
    max_expand: int = 200_000,
) -> AStarResult:
    """4 邻接 A*。cost: 自由格 1，未知格 unknown_cost。坐标为 (col, row)。

    不可达时返回 AStarResult(path=[], cost=inf)。
    """
    h, w = passable.shape
    sc, sr = int(start[0]), int(start[1])
    gc, gr = int(goal[0]), int(goal[1])
    if not (0 <= sc < w and 0 <= sr < h and 0 <= gc < w and 0 <= gr < h):
        return AStarResult([], float("inf"))
    if not passable[gr, gc]:
        return AStarResult([], float("inf"))

    def heuristic(c: int, r: int) -> float:
        return abs(c - gc) + abs(r - gr)  # 4 邻接下的可采纳启发

    open_heap = [(heuristic(sc, sr), 0.0, sc, sr)]
    g_cost = np.full((h, w), np.inf, dtype=np.float32)
    g_cost[sr, sc] = 0.0
    came = {}
    expanded = 0

    while open_heap:
        _, g, c, r = heapq.heappop(open_heap)
        if (c, r) == (gc, gr):
            # 回溯路径
            path = [(c, r)]
            while (c, r) in came:
                c, r = came[(c, r)]
                path.append((c, r))
            path.reverse()
            return AStarResult(path=path, cost=float(g))
        if g > g_cost[r, c]:
            continue
        expanded += 1
        if expanded > max_expand:
            break
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = c + dc, r + dr
            if not (0 <= nc < w and 0 <= nr < h):
                continue
            if not passable[nr, nc]:
                continue
            step = unknown_cost if unknown[nr, nc] else 1.0
            ng = g + step
            if ng < g_cost[nr, nc]:
                g_cost[nr, nc] = ng
                came[(nc, nr)] = (c, r)
                heapq.heappush(open_heap, (ng + heuristic(nc, nr), ng, nc, nr))

    return AStarResult([], float("inf"))
