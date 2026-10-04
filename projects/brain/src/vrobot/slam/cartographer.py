"""Google Cartographer 高层封装。

底层为 pybind11 原生扩展 ``vrobot_slam_native``（静态链接 cartographer.lib），
扩展依赖的第三方 DLL 位于包内 ``_bin/`` 目录，导入前需注册 DLL 搜索路径。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable, Optional, Tuple, Union

import numpy as np

_HERE = Path(__file__).resolve().parent
_BIN_DIR = _HERE / "_bin"
if _BIN_DIR.is_dir():
    os.add_dll_directory(str(_BIN_DIR))

# 扩展依赖 _bin/ 下的 DLL，必须在 add_dll_directory 之后再导入
from vrobot.slam import vrobot_slam_native as _native  # noqa: E402

_DEFAULT_CONFIG_DIR = _HERE.parents[2] / "configs"
_DEFAULT_CONFIG = "cartographer_2d.lua"
_CARTOGRAPHER_CONFIG_DIR = Path(
    os.environ.get("VROBOT_CARTOGRAPHER_CONFIGS",
                   "D:/Projects/OpenSourceLibs/cartographer-2.0.0/configuration_files"))

ConfigLike = Union[str, Path, None]


class CartographerSLAM:
    """2D SLAM 会话：接收激光/IMU/里程计数据，产出优化位姿与 .pbstream 地图。

    用法::

        slam = CartographerSLAM()          # 默认 configs/cartographer_2d.lua
        slam.start_trajectory()            # 可选 imu_id="imu", odom_id="odom"
        for ts, points in scans:           # points: (N,3) 传感器坐标系 [x,y,z] 米
            slam.add_scan(ts, points)
        slam.finish_trajectory()
        slam.run_final_optimization()
        poses = slam.optimized_poses()     # (N,3) [x, y, yaw] 全局系
        slam.write_pbstream("map.pbstream")
    """

    def __init__(self, config: ConfigLike = None,
                 extra_search_paths: Iterable[Union[str, Path]] = ()) -> None:
        if config is None:
            config_path = _DEFAULT_CONFIG_DIR / _DEFAULT_CONFIG
        else:
            config_path = Path(config)
            if not config_path.exists():
                config_path = _DEFAULT_CONFIG_DIR / str(config)
        search_paths = [str(config_path.parent), str(_CARTOGRAPHER_CONFIG_DIR)]
        search_paths.extend(str(p) for p in extra_search_paths)
        self._impl = _native.Mapper(
            config_path.read_text(encoding="utf-8"), search_paths)
        self._started = False

    # ---- 数据输入 -------------------------------------------------------
    def start_trajectory(self, range_id: str = "lidar",
                         imu_id: Optional[str] = None,
                         odom_id: Optional[str] = None) -> int:
        """开始一条新轨迹，返回 trajectory_id。数据必须按时间递增送入。"""
        tid = self._impl.start_trajectory(range_id, imu_id, odom_id)
        self._started = True
        return tid

    def add_scan(self, timestamp: float, points: np.ndarray,
                 origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """送入一帧激光扫描。

        points: (N,3) [x,y,z] 或 (N,4) [x,y,z,rel_time]，传感器坐标系，米。
        timestamp: Unix 秒。
        """
        if not self._started:
            raise RuntimeError("call start_trajectory() before add_scan()")
        self._impl.add_scan(float(timestamp), np.asarray(points, dtype=np.float64),
                            [float(v) for v in origin])

    def add_imu(self, timestamp: float, accel, gyro) -> None:
        """送入 IMU 数据（m/s^2 与 rad/s），需 start_trajectory(imu_id=...)。"""
        self._impl.add_imu(float(timestamp), list(map(float, accel)),
                           list(map(float, gyro)))

    def add_odometry(self, timestamp: float, x: float, y: float,
                     yaw: float) -> None:
        """送入里程计位姿，需 start_trajectory(odom_id=...)。"""
        self._impl.add_odometry(float(timestamp), float(x), float(y), float(yaw))

    # ---- 输出 -----------------------------------------------------------
    def finish_trajectory(self) -> None:
        self._impl.finish_trajectory()

    def run_final_optimization(self) -> None:
        self._impl.run_final_optimization()

    def optimized_poses(self) -> np.ndarray:
        """全局优化后的轨迹节点位姿，(N,3) [x, y, yaw]。"""
        return np.asarray(self._impl.optimized_poses())

    def write_pbstream(self, path: Union[str, Path],
                       include_unfinished: bool = False) -> None:
        self._impl.write_pbstream(str(path), include_unfinished)

    def load_state(self, path: Union[str, Path], frozen: bool = False) -> None:
        self._impl.load_state(str(path), frozen)

    # ---- 状态查询 ---------------------------------------------------------
    @property
    def trajectory_id(self) -> int:
        return self._impl.trajectory_id

    @property
    def num_nodes(self) -> int:
        return self._impl.num_nodes

    @property
    def num_submaps(self) -> int:
        return self._impl.num_submaps

    @property
    def latest_local_pose(self) -> Optional[Tuple[float, float, float]]:
        return self._impl.latest_local_pose

    # ---- 上下文管理 ---------------------------------------------------------
    def close(self) -> None:
        self.finish_trajectory()

    def __enter__(self) -> "CartographerSLAM":
        return self

    def __exit__(self, *exc) -> None:
        self.finish_trajectory()
