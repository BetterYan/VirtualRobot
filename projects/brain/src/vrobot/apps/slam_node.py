"""vrobot 主入口：WS 服务端 + 2D SLAM 循环 + 可视化 + 评估。

用法：
    python -m vrobot.apps.slam_node --config configs/slam2d.yaml
    # 先启动本程序，再启动 Godot（sim2d 场景）；HUD 显示已连接后即可遥控/自动探索
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import time
from pathlib import Path

import yaml

from vrobot.comm.server import SimServer
from vrobot.control.explorer import FrontierExplorer, ReactiveExplorer
from vrobot.control.hybrid_explorer import HybridExplorer
from vrobot.eval.evaluator import PoseEvaluator
from vrobot.slam.slam2d import Slam2D, SlamConfig
from vrobot.viz.debug_viewer import DebugViewer
from vrobot.viz.lidar_viewer import LidarViewer
from vrobot.viz.fast_map_viewer import create_map_viewer

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s"
)
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


async def run(
    cfg: dict,
    show_view: bool,
    show_debug: bool,
    show_lidar: bool,
    force_auto: bool | None,
) -> None:
    srv_cfg = cfg.get("server", {})
    server = SimServer(srv_cfg.get("host", "127.0.0.1"), int(srv_cfg.get("port", 9094)))

    slam_cfg = SlamConfig(
        grid=cfg.get("grid", {}),
        scan_match=cfg.get("scan_match", {}),
        use_match=cfg.get("slam", {}).get("use_match", True),
    )

    slam_section = cfg.get("slam", {})
    engine = str(slam_section.get("engine", "builtin")).lower()
    if engine == "cartographer":
        from vrobot.slam.cartographer_engine import (
            CartographerEngineConfig,
            CartographerSLAM2D,
        )

        eng_cfg = CartographerEngineConfig(
            grid=cfg.get("grid", {}),
            **slam_section.get("cartographer", {}),
        )
        slam = CartographerSLAM2D(eng_cfg, sensor=cfg.get("sensor", {}))
        log.info(
            "SLAM 引擎: cartographer（scale=%.3f m/px, max_range=%.0fpx）",
            eng_cfg.scale,
            slam._range_max_px,
        )
    elif engine in ("builtin", "tinyslam", ""):
        slam = Slam2D(slam_cfg)
        log.info("SLAM 引擎: builtin（TinySLAM 式）")
    else:
        raise ValueError(f"未知 SLAM 引擎: {engine!r}（可选 builtin / cartographer）")

    exp_cfg = cfg.get("explorer", {})
    exp_mode = str(exp_cfg.get("mode", "hybrid"))
    explorer: HybridExplorer | FrontierExplorer | ReactiveExplorer | object | None
    if exp_mode == "hybrid":
        explorer = HybridExplorer(
            cell_size=slam.grid.cell_size,
            linear=exp_cfg.get("linear", 60.0),
            angular=exp_cfg.get("angular", 1.5),
            inflate_cells=exp_cfg.get("inflate_cells", 2),
            lane_pitch=exp_cfg.get("hybrid", {}).get("lane_pitch", 4),
            min_run=exp_cfg.get("hybrid", {}).get("min_run", 4),
            frontier_mode=exp_cfg.get("hybrid", {}).get("frontier_mode", "tour"),
        )
    elif exp_mode == "frontier":
        explorer = FrontierExplorer(
            cell_size=slam.grid.cell_size,
            linear=exp_cfg.get("linear", 60.0),
            angular=exp_cfg.get("angular", 2.4),
            inflate_cells=exp_cfg.get("inflate_cells", 2),
            min_frontier_size=exp_cfg.get("min_frontier_size", 4),
        )
    elif exp_mode == "tour":
        from vrobot.control.tour_explorer import TourExplorer

        explorer = TourExplorer(
            cell_size=slam.grid.cell_size,
            linear=exp_cfg.get("linear", 60.0),
            angular=exp_cfg.get("angular", 1.5),
            inflate_cells=exp_cfg.get("inflate_cells", 2),
            min_frontier_size=exp_cfg.get("min_frontier_size", 4),
        )
    elif exp_mode == "reactive":
        explorer = ReactiveExplorer(
            linear=exp_cfg.get("reactive", {}).get("linear", 55.0),
            angular=exp_cfg.get("reactive", {}).get("angular", 1.6),
            front_dist=exp_cfg.get("reactive", {}).get("front_dist", 60.0),
            beam_spread=exp_cfg.get("reactive", {}).get("beam_spread", 7),
        )
    else:  # off
        explorer = None
    auto_enabled = (
        force_auto if force_auto is not None else bool(exp_cfg.get("enabled", False))
    )
    mode_state = {"auto": auto_enabled}

    def _on_mode_changed(mode_name: str) -> None:
        """Godot 侧 set_mode 上行请求：动态开关自动探索。"""
        mode_state["auto"] = mode_name == "auto"
        log.info(
            "模式切换: %s（自动探索 %s）",
            mode_name,
            "开" if mode_state["auto"] else "关",
        )

    server.on_mode_changed = _on_mode_changed

    evaluator = PoseEvaluator() if cfg.get("eval", {}).get("enabled", True) else None
    viewer = create_map_viewer(enabled=show_view, cell_size=slam.grid.cell_size)
    lidar_view = LidarViewer(
        enabled=show_lidar,
        range_max=float(cfg.get("lidar", {}).get("range_max", 320.0)),
    )
    debug = DebugViewer(enabled=show_debug)
    debug.set_min_score(slam.matcher.min_score)

    await server.start()
    print("=" * 62)
    print(" vrobot 2D SLAM 节点已启动")
    print(f" 监听: ws://{server.host}:{server.port}  （请随后启动 Godot sim2d 场景）")
    print(
        f" 探索模式: {exp_mode}  自动驾驶: {'开' if mode_state['auto'] else '关'}（Godot HUD 按钮可切换）"
    )
    print(" Ctrl+C 退出")
    print("=" * 62)

    seq = 0
    frame_count = 0
    last_stats_t = time.time()
    cmd_interval = 0.05  # cmd_vel 下行 20Hz（与传感器帧率一致；过低会加剧沿边摆动）
    last_cmd_t = 0.0
    done_reported = False

    try:
        while True:
            frame = server.consume_frame()
            if frame is not None:
                result = slam.process(frame.odom, frame.scan)
                frame_count += 1
                if server.ground_truth is not None:
                    if evaluator:
                        evaluator.update(result.pose, server.ground_truth)
                    if debug.enabled:
                        truth = server.ground_truth
                        debug.push(
                            (truth.x, truth.y, truth.theta),
                            frame.odom.as_tuple(),
                            result.pose,
                            score=result.score,
                            matched=result.matched,
                        )

            now = time.time()
            if (  # noqa: SIM102
                mode_state["auto"]
                and explorer is not None
                and server.connected.is_set()
                and now - last_cmd_t >= cmd_interval
            ):
                if frame is not None:
                    linear, angular = explorer.update(slam, frame.scan)
                    server.send_cmd_vel(linear, angular, seq)
                    seq += 1
                    last_cmd_t = now
                    if getattr(explorer, "done", False) and not done_reported:
                        done_reported = True
                        print(
                            ">>> 探索完成：可达范围内已无可达边界，地图构建完毕。"
                            "（Godot 切回 manual 模式可手动查看）"
                        )

            if lidar_view.enabled and frame is not None and frame_count % 2 == 0:
                lidar_view.update(frame.scan)

            if viewer.enabled and frame is not None and frame_count % 3 == 0:
                gt = (
                    (
                        server.ground_truth.x,
                        server.ground_truth.y,
                        server.ground_truth.theta,
                    )
                    if (evaluator and server.ground_truth)
                    else None
                )
                viewer.update(slam, gt)

            if now - last_stats_t >= 3.0:
                last_stats_t = now
                if frame_count:
                    s = evaluator.stats() if evaluator else {"n": 0}
                    if s.get("n"):
                        drift = (
                            f" 漂移率: {s['drift_per_100px']:.2f}px/100px（真值里程 {s['gt_path_px']:.0f}px）"
                            if "drift_per_100px" in s
                            else ""
                        )
                        _lo = slam.grid._logodds
                        _fr = int((_lo <= -0.3).sum())
                        _oc = int((_lo >= 0.62).sum())
                        print(
                            f"[stats] frames={frame_count} pose=({slam.pose[0]:.0f},{slam.pose[1]:.0f}) "
                            f"误差: mean={s['pos_mean']:.1f}px last={s['pos_last']:.1f}px "
                            f"max={s['pos_max']:.1f}px | {s['th_mean_deg']:.1f}deg{drift} "
                            f"| 地图: free={_fr} occ={_oc}"
                        )
                    else:
                        _lo = slam.grid._logodds
                        print(
                            f"[stats] frames={frame_count} pose=({slam.pose[0]:.0f},{slam.pose[1]:.0f})（无真值）"
                            f" 地图: free={int((_lo <= -0.3).sum())} occ={int((_lo >= 0.62).sum())}"
                        )

            await asyncio.sleep(0.002)  # ~200Hz 空转，消费节拍由 Godot 20Hz 决定
    except asyncio.CancelledError:
        pass
    finally:
        debug.close()
        lidar_view.close()
        viewer.close()
        await server.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="vrobot 2D SLAM node")
    parser.add_argument("--config", default=None, help="yaml 配置路径")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--no-view", action="store_true", help="禁用地图可视化窗口")
    parser.add_argument(
        "--no-debug", action="store_true", help="禁用调试窗口（真值 vs 传感器差异曲线）"
    )
    parser.add_argument("--no-lidar", action="store_true", help="禁用激光雷达视图窗口")
    parser.add_argument(
        "--auto",
        dest="auto",
        action="store_true",
        default=None,
        help="强制开启自动探索（覆盖配置）",
    )
    parser.add_argument(
        "--no-auto",
        dest="auto",
        action="store_false",
        default=None,
        help="强制关闭自动探索（覆盖配置）",
    )
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.host:
        cfg.setdefault("server", {})["host"] = args.host
    if args.port:
        cfg.setdefault("server", {})["port"] = args.port

    try:
        asyncio.run(
            run(
                cfg,
                show_view=not args.no_view,
                show_debug=args.no_debug,
                show_lidar=args.no_lidar,
                force_auto=args.auto,
            )
        )
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    main()
