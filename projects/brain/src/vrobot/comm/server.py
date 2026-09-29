"""WebSocket 服务端（Python = server，Godot = client）。

职责：收 sensor_frame / ground_truth / hello，缓存最新一帧供主循环消费；
提供 cmd_vel 下行队列（满则丢弃最旧，永不阻塞 SLAM 循环）。
"""

from __future__ import annotations

import asyncio
import json
import logging

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from vrobot.comm.messages import (
    HelloInfo,
    decode_ground_truth,
    decode_sensor_frame,
    encode_cmd_vel,
    encode_set_mode,
    parse_envelope,
)
from vrobot.sensors.models import GroundTruth, SensorFrame

log = logging.getLogger(__name__)


class SimServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 9094):
        self.host = host
        self.port = port
        self.latest: SensorFrame | None = None
        self.ground_truth: GroundTruth | None = None
        self.hello: HelloInfo | None = None
        self.connected = asyncio.Event()
        self.remote_mode = "manual"  # Godot 侧当前模式（set_mode 同步）
        self.on_mode_changed = None  # callable(mode: str)，模式变化时回调
        self._outgoing: asyncio.Queue[str] = asyncio.Queue(maxsize=50)
        self._server = None

    # ---- 生命周期 ----

    async def start(self) -> None:
        self._server = await serve(self._handler, self.host, self.port)
        log.info("WS server listening on ws://%s:%d （等待 Godot 连接）", self.host, self.port)

    async def stop(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    # ---- 收发 ----

    def send_cmd_vel(self, linear: float, angular: float, seq: int) -> None:
        """非阻塞入队；队列满则丢弃（保持 SLAM 循环节拍）。"""
        try:
            self._outgoing.put_nowait(encode_cmd_vel(linear, angular, seq))
        except asyncio.QueueFull:
            pass

    def consume_frame(self) -> SensorFrame | None:
        """取走最新一帧传感器数据（读后置空，保证不重复处理）。"""
        f, self.latest = self.latest, None
        return f

    # ---- 内部 ----

    async def _handler(self, ws) -> None:
        peer = getattr(ws, "remote_address", None)
        log.info("Godot 已连接: %s", peer)
        self.connected.set()
        # 连接建立即同步当前模式，让 Godot HUD 与探索器状态对齐
        try:
            self._outgoing.put_nowait(encode_set_mode(self.remote_mode, 0))
        except asyncio.QueueFull:
            pass
        sender = asyncio.create_task(self._sender(ws))
        try:
            async for raw in ws:
                self._on_raw(raw)
        except ConnectionClosed:
            pass
        finally:
            sender.cancel()
            self.connected.clear()
            self.latest = None
            log.info("Godot 断开: %s", peer)

    async def _sender(self, ws) -> None:
        while True:
            msg = await self._outgoing.get()
            await ws.send(msg)

    def _on_raw(self, raw) -> None:
        try:
            if isinstance(raw, (bytes, bytearray)):
                raw = raw.decode("utf-8")
            msg_type, seq, payload = parse_envelope(raw)
            ts = int(json.loads(raw).get("ts", 0)) if msg_type else 0
        except (json.JSONDecodeError, UnicodeDecodeError, ValueError):
            log.warning("无法解析的消息已丢弃")
            return
        if msg_type == "sensor_frame":
            self.latest = decode_sensor_frame(payload, seq, ts)
        elif msg_type == "ground_truth":
            self.ground_truth = decode_ground_truth(payload, seq, ts)
        elif msg_type == "hello":
            self.hello = HelloInfo.from_payload(payload)
            log.info("hello: world=%s beams=%d", self.hello.world, self.hello.lidar_beams)
        elif msg_type == "set_mode":
            mode = str(payload.get("mode", ""))
            if mode in ("auto", "manual") and mode != self.remote_mode:
                self.remote_mode = mode
                if self.on_mode_changed:
                    self.on_mode_changed(mode)
        # 未知 type 忽略（前向兼容）
