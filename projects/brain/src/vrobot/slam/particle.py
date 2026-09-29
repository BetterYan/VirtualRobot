"""RBPF 粒子滤波 2D SLAM（v2 预留占位）。

规划接口（Gmapping 思路）：
    class Particle:
        pose: (x, y, theta)
        grid: OccupancyGrid          # 每粒子独立地图
        weight: float

    class RBPFSlam2D:
        def process(self, odom, scan) -> SlamResult:
            1. 采样：按里程计增量 + 噪声模型扰动每个粒子位姿
            2. 扫描匹配：每粒子在其地图上做相关性匹配（低频/抽样）
            3. 权重更新：w ∝ ∏ p(scan | pose_i, map_i)
            4. 重采样：有效粒子数 N_eff < N/2 时系统重采样
            5. 地图更新：每粒子按其位姿写入自己的地图

替换点：Slam2D.process 的位姿估计段。接口保持一致即可无缝切换。
"""

from __future__ import annotations


class RBPFSlam2D:  # noqa: N801 - 占位，v2 实现
    """占位类：实现规划见模块 docstring。"""

    def __init__(self, *args, **kwargs):
        raise NotImplementedError("v2: RBPF 粒子滤波待实现，见模块 docstring 的接口规划")
