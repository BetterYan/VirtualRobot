"""消息编解码：protocols/message-schema.md v1.0 的 Python 实现。

comm/ 只做传输与格式转换；业务语义（SLAM/控制/评估）不在这里。
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import numpy as np

from vrobot.sensors.models import GroundTruth, LaserScan2D, Odometry2D, SensorFrame

ENVELOPE_KEYS = ("type", "seq", "ts", "payload")


def envelope(msg_type: str, seq: int, payload: dict) -> str:
    return json.dumps(
        {"type": msg_type, "seq": seq, "ts": int(time.time() * 1000), "payload": payload},
        ensure_ascii=False,
    )


def parse_envelope(raw: str | bytes) -> tuple[str, int, dict]:
    """解析信封；返回 (type, seq, payload)。未知字段忽略（前向兼容）。"""
    data = json.loads(raw)
    return data.get("type", ""), int(data.get("seq", 0)), data.get("payload", {}) or {}


def decode_sensor_frame(payload: dict, seq: int, ts: int) -> SensorFrame:
    odom_raw = payload.get("odom", {})
    lidar_raw = payload.get("lidar", {})
    ranges = np.asarray(lidar_raw.get("ranges", []), dtype=np.float32)
    odom = Odometry2D(
        x=float(odom_raw.get("x", 0.0)),
        y=float(odom_raw.get("y", 0.0)),
        theta=float(odom_raw.get("theta", 0.0)),
        seq=seq,
        ts=ts,
        dt=float(payload.get("dt", 0.05)),
    )
    scan = LaserScan2D(
        ranges=ranges,
        angle_min=float(lidar_raw.get("angle_min", -np.pi)),
        angle_inc=float(lidar_raw.get("angle_inc", np.pi / 36)),
        range_max=float(lidar_raw.get("range_max", 320.0)),
        seq=seq,
        ts=ts,
    )
    return SensorFrame(odom=odom, scan=scan, collision=bool(payload.get("collision", False)))


def decode_ground_truth(payload: dict, seq: int, ts: int) -> GroundTruth:
    return GroundTruth(
        x=float(payload.get("x", 0.0)),
        y=float(payload.get("y", 0.0)),
        theta=float(payload.get("theta", 0.0)),
        seq=seq,
        ts=ts,
    )


def encode_cmd_vel(linear: float, angular: float, seq: int) -> str:
    return envelope("cmd_vel", seq, {"linear": float(linear), "angular": float(angular)})


def encode_set_mode(mode: str, seq: int) -> str:
    return envelope("set_mode", seq, {"mode": mode})


@dataclass
class HelloInfo:
    world: str = "sim2d"
    world_version: int = 1
    lidar_beams: int = 72
    range_max: float = 320.0
    raw: dict = None

    @classmethod
    def from_payload(cls, payload: dict) -> "HelloInfo":
        sensor = payload.get("sensor", {}) or {}
        return cls(
            world=str(payload.get("world", "sim2d")),
            world_version=int(payload.get("world_version", 1)),
            lidar_beams=int(sensor.get("lidar_beams", 72)),
            range_max=float(sensor.get("range_max", 320.0)),
            raw=payload,
        )
