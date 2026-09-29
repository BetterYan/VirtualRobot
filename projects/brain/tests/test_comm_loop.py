"""通信链路集成冒烟测试：模拟 Godot 客户端 ↔ SimServer ↔ SLAM 全链路。"""

import asyncio
import json

import pytest

from vrobot.comm.messages import encode_cmd_vel, envelope, parse_envelope
from vrobot.comm.server import SimServer
from vrobot.sensors.models import scan_world_points
from vrobot.slam.slam2d import Slam2D, SlamConfig
from tests.sim_world import SimParams, SimRobot, SimWorld, drive_to_waypoints


def _make_frames(n=80):
    world = SimWorld()
    robot = SimRobot(world, (100.0, 90.0, 0.0), SimParams(seed=3))
    frames = drive_to_waypoints(robot, [(400.0, 90.0), (400.0, 200.0)], max_steps=n)
    return world, robot, frames


def test_server_slam_pipeline():
    """模拟 Godot 连上来发 60 帧，SLAM 应正常消费；cmd_vel 能下发到客户端。"""

    async def run():
        world, robot, frames = _make_frames(80)
        server = SimServer("127.0.0.1", 0)  # port=0 → 由 asyncio 分配
        # SimServer 用固定端口启动不方便拿真实端口，这里直接内嵌 handler 不行；
        # 改用实际端口启动：
        server2 = SimServer("127.0.0.1", 9971)
        await server2.start()
        received_cmds = []
        received_hello_ack = []

        async def fake_godot():
            import websockets

            async with websockets.connect("ws://127.0.0.1:9971") as ws:
                # 握手
                hello = envelope("hello", 0, {"world": "sim2d", "world_version": 1})
                await ws.send(hello)
                # 推帧
                for i, f in enumerate(frames):
                    payload = {
                        "odom": {"x": f.odom.x, "y": f.odom.y, "theta": f.odom.theta},
                        "lidar": {
                            "ranges": [float(r) for r in f.scan.ranges],
                            "angle_min": f.scan.angle_min,
                            "angle_inc": f.scan.angle_inc,
                            "range_max": f.scan.range_max,
                        },
                        "collision": False,
                        "dt": 0.05,
                    }
                    await ws.send(envelope("sensor_frame", i + 1, payload))
                    await asyncio.sleep(0.001)
                    # 收下行
                    try:
                        while True:
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.01)
                            t, s, p = parse_envelope(raw)
                            if t == "cmd_vel":
                                received_cmds.append(p)
                    except (asyncio.TimeoutError, TimeoutError):
                        pass
                await asyncio.sleep(0.05)
                try:
                    while True:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                        t, s, p = parse_envelope(raw)
                        if t == "cmd_vel":
                            received_cmds.append(p)
                except (asyncio.TimeoutError, TimeoutError):
                    pass

        slam = Slam2D(SlamConfig(use_match=True))
        client_task = asyncio.create_task(fake_godot())

        # 服务端消费循环（跑 4 秒，容忍调度抖动）
        async def consume():
            n = 0
            for _ in range(800):
                f = server2.consume_frame()
                if f is not None:
                    slam.process(f.odom, f.scan)
                    n += 1
                    server2.send_cmd_vel(50.0, 0.1, n)
                await asyncio.sleep(0.005)
            return n

        try:
            n = await asyncio.wait_for(consume(), timeout=20)
        finally:
            client_task.cancel()
            await server2.stop()

        assert n >= 40, f"服务端仅消费 {n} 帧"
        # 关键帧插入机制下（每 ≥5px/0.15rad 才入图），栅格更新数 < 消费帧数；
        # 本路径约 410px，下界取消费帧数的 1/3 即可证明 SLAM 持续在更新地图。
        assert slam.grid.n_updates >= n // 3, \
            f"栅格更新 {slam.grid.n_updates} 次（消费 {n} 帧），SLAM 疑似未持续建图"
        assert len(received_cmds) >= 1, "cmd_vel 未送达客户端"
        assert "linear" in received_cmds[0]

    asyncio.run(run())
