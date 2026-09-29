"""vrobot 主入口：WS 服务端 + 2D SLAM 循环 + 可视化 + 评估。

用法：
    python -m vrobot.apps.slam_node --config configs/slam2d.yaml
    # 先启动本程序，再启动 Godot（sim2d 场景）；HUD 显示已连接后即可遥控/自动探索
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path

import yaml

from vrobot.comm.server import SimServer
from vrobot.control.explorer import AutoExplorer
from vrobot.eval.evaluator import PoseEvaluator
from vrobot.slam.slam2d import Slam2D, SlamConfig
from vrobot.viz.map_viewer import MapViewer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
log = logging.getLogger("slam_node")


def load_config(path: str | None) -> dict:
    if not path:
        return {}
    p = Path(path)
    if not p.exists():
        log.warning("配置文件不存在: %s，使用默认配置", p)
        return {}
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


async def run(cfg: dict, show_view: bool, force_auto: bool | None) -> None:
    srv_cfg = cfg.get("server", {})
    server = SimServer(srv_cfg.get("host", "127.0.0.1"), int(srv_cfg.get("port", 9094)))

    slam_cfg = SlamConfig(
        grid=cfg.get("grid", {}),
        scan_match=cfg.get("scan_match", {}),
        use_match=cfg.get("slam", {}).get("use_match", True),
    )
    slam = Slam2D(slam_cfg)

    exp_cfg = cfg.get("explorer", {})
    explorer = AutoExplorer(
        linear=exp_cfg.get("linear", 55.0),
        angular=exp_cfg.get("angular", 1.6),
        front_dist=exp_cfg.get("front_dist", 60.0),
        beam_spread=exp_cfg.get("beam_spread", 7),
    )
    auto_enabled = force_auto if force_auto is not None else bool(exp_cfg.get("enabled", False))
    mode_state = {"auto": auto_enabled}

    def _on_mode_changed(mode_name: str) -> None:
        """Godot 侧 set_mode 上行请求：动态开关自动探索。"""
        mode_state["auto"] = mode_name == "auto"
        log.info("模式切换: %s（自动探索 %s）", mode_name, "开" if mode_state["auto"] else "关")

    server.on_mode_changed = _on_mode_changed

    evaluator = PoseEvaluator() if cfg.get("eval", {}).get("enabled", True) else None
    viewer = MapViewer(enabled=show_view, cell_size=slam.grid.cell_size)

    await server.start()
    print("=" * 62)
    print(" vrobot 2D SLAM 节点已启动")
    print(f" 监听: ws://{server.host}:{server.port}  （请随后启动 Godot sim2d 场景）")
    print(f" 自动探索: {'开' if mode_state['auto'] else '关'}（Godot HUD 按钮可随时切换）")
    print(" Ctrl+C 退出")
    print("=" * 62)

    seq = 0
    frame_count = 0
    last_stats_t = time.time()
    cmd_interval = 0.1  # cmd_vel 下行 10Hz
    last_cmd_t = 0.0

    try:
        while True:
            frame = server.consume_frame()
            if frame is not None:
                result = slam.process(frame.odom, frame.scan)
                frame_count += 1
                if evaluator and server.ground_truth is not None:
                    evaluator.update(result.pose, server.ground_truth)

            now = time.time()
            if mode_state["auto"] and server.connected.is_set() and now - last_cmd_t >= cmd_interval:
                if frame is not None:
                    linear, angular = explorer.update(frame.scan)
                    server.send_cmd_vel(linear, angular, seq)
                    seq += 1
                    last_cmd_t = now

            if viewer.enabled and frame is not None and frame_count % 3 == 0:
                gt = (server.ground_truth.x, server.ground_truth.y, server.ground_truth.theta) \
                    if (evaluator and server.ground_truth) else None
                viewer.update(slam, gt)

            if now - last_stats_t >= 3.0:
                last_stats_t = now
                if frame_count:
                    s = evaluator.stats() if evaluator else {"n": 0}
                    if s.get("n"):
                        print(
                            f"[stats] frames={frame_count} pose=({slam.pose[0]:.0f},{slam.pose[1]:.0f}) "
                            f"误差: mean={s['pos_mean']:.1f}px last={s['pos_last']:.1f}px "
                            f"max={s['pos_max']:.1f}px | {s['th_mean_deg']:.1f}deg"
                        )
                    else:
                        print(f"[stats] frames={frame_count} pose=({slam.pose[0]:.0f},{slam.pose[1]:.0f})（无真值）")

            await asyncio.sleep(0.002)  # ~200Hz 空转，消费节拍由 Godot 20Hz 决定
    except asyncio.CancelledError:
        pass
    finally:
        viewer.close()
        await server.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="vrobot 2D SLAM node")
    parser.add_argument("--config", default=None, help="yaml 配置路径")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--no-view", action="store_true", help="禁用地图可视化窗口")
    parser.add_argument("--auto", dest="auto", action="store_true", default=None,
                        help="强制开启自动探索（覆盖配置）")
    parser.add_argument("--no-auto", dest="auto", action="store_false", default=None,
                        help="强制关闭自动探索（覆盖配置）")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.host:
        cfg.setdefault("server", {})["host"] = args.host
    if args.port:
        cfg.setdefault("server", {})["port"] = args.port

    try:
        asyncio.run(run(cfg, show_view=not args.no_view, force_auto=args.auto))
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    main()
