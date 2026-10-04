"""原生扩展 smoke 测试：导入 -> 建图 -> 优化位姿 -> 保存 pbstream。

依赖已构建并部署的 vrobot_slam_native.pyd（见 scripts/build_native.py）。
"""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("vrobot.slam.vrobot_slam_native",
                    reason="原生扩展未构建: uv run python scripts/build_native.py")

from vrobot.slam.cartographer import CartographerSLAM  # noqa: E402


def _synthetic_scan(x: float, y: float, n: int = 180) -> np.ndarray:
    """机器人位于 (x, y)，四周一圈半径约 5m 的墙壁。"""
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    r = 5.0 + 0.02 * np.sin(angles * 7) + np.random.default_rng(0).normal(
        0, 0.005, n)
    return np.stack([x + r * np.cos(angles),
                     y + r * np.sin(angles),
                     np.zeros(n)], axis=1)


def test_smoke_slam_pipeline(tmp_path):
    slam = CartographerSLAM()
    slam.start_trajectory()
    t0, dt = 1000.0, 0.1  # 10 Hz

    for i in range(40):
        x = 0.01 * i  # 缓慢平移
        slam.add_scan(t0 + i * dt, _synthetic_scan(x, 0.0))

    slam.finish_trajectory()
    assert slam.num_nodes >= 1, "local SLAM 应至少产生一个轨迹节点"

    poses = slam.optimized_poses()
    assert poses.shape == (slam.num_nodes, 3)
    assert np.all(np.isfinite(poses))
    # 墙壁半径 5m，平移极小，位姿应接近原点附近
    assert np.abs(poses[:, :2]).max() < 1.0

    slam.run_final_optimization()

    out = tmp_path / "map.pbstream"
    slam.write_pbstream(out)
    assert out.exists() and out.stat().st_size > 0


def test_scan_shape_validation():
    slam = CartographerSLAM()
    slam.start_trajectory()
    with pytest.raises(ValueError):
        slam.add_scan(0.0, np.zeros((10, 2)))  # 非法形状


def test_imu_requires_configuration():
    slam = CartographerSLAM()
    slam.start_trajectory()
    with pytest.raises(ValueError):
        slam.add_imu(0.0, [0.0, 0.0, 9.8], [0.0, 0.0, 0.0])
