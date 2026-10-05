"""高性能实时地图查看器：FastMapViewer。

matplotlib 的整图渲染管线不适合逐帧图像刷新（每帧重走完整渲染树，
200×160 栅格 + 叠加也要几十毫秒且卡顿）。本查看器每帧只做
numpy 数组 → 图像 blit，快一个数量级。

后端自动选择：
  - opencv-python 可用 → cv2.imshow（推荐，若已安装）
  - 否则 → Pillow ImageTk + Tk（零新增依赖，matplotlib 自带 Pillow）

接口与 MapViewer 一致（update(slam, gt_pose=None) / close() / enabled），
slam 参数鸭子类型：要求 slam.grid.prob() 与 slam.trajectory（px 坐标列表）。
"""

from __future__ import annotations

import logging

import numpy as np

log = logging.getLogger(__name__)

try:
    import cv2

    HAS_CV2 = True
except Exception as e:  # cv2 加载偶发失败（DLL 锁/插件冲突），重试一次
    log.warning("cv2 首次导入失败（%s: %s），重试...", type(e).__name__, e)
    try:
        import time as _time

        _time.sleep(0.2)
        import cv2

        HAS_CV2 = True
    except Exception as e2:
        cv2 = None
        HAS_CV2 = False
        log.warning("cv2 不可用（%s），地图窗口回退后端", e2)


class FastMapViewer:
    SCALE = 4  # 栅格放大倍数（200×160 → 800×640）

    def __init__(self, enabled: bool = True, cell_size: float = 8.0):
        self.enabled = enabled
        self.cell_size = float(cell_size)
        self._frame = 0
        self._backend: str | None = None
        self._win_ready = False
        self._win_title = "vrobot SLAM Map"   # cv2 HighGUI 标题仅支持 ASCII
        self._font = self._load_cjk_font()
        self._caption = "vrobot SLAM 实时地图"
        self._caption_strip = self._make_caption_strip()  # 静态模板，只渲染一次
        self._select_backend()

    def _make_caption_strip(self) -> np.ndarray:
        """标题条（含中文）用 PIL 渲染一次并缓存 —— 逐帧 PIL 往返是性能杀手。"""
        strip = np.full((30, 340, 3), 255, dtype=np.uint8)
        try:
            from PIL import Image, ImageDraw

            pil = Image.fromarray(cv2.cvtColor(strip, cv2.COLOR_BGR2RGB))
            ImageDraw.Draw(pil).text((12, 4), self._caption,
                                     font=self._font, fill=(30, 30, 30))
            return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        except Exception as e:
            log.debug("标题条渲染失败: %s", e)
            return strip

    @staticmethod
    def _load_cjk_font():
        """加载系统中文字体（PIL 渲染，cv2.putText 不支持 CJK）。"""
        from PIL import ImageFont

        for name in ("msyh.ttc", "simhei.ttf", "simsun.ttc", "msyhbd.ttc"):
            try:
                return ImageFont.truetype(f"C:/Windows/Fonts/{name}", 20)
            except Exception:
                continue
        return ImageFont.load_default()

    def _select_backend(self) -> None:
        if not self.enabled:
            return
        if HAS_CV2:
            self._backend = "cv2"
        else:
            try:
                import tkinter  # noqa: F401
                from PIL import Image, ImageTk  # noqa: F401

                self._backend = "tk"
            except ImportError as e:
                log.warning("FastMapViewer 后端不可用（需 opencv-python 或 Pillow+Tk）: %s", e)
                self.enabled = False
                return
            self._setup_tk()

    # ---- Tk 后端 ----

    def _setup_tk(self) -> None:
        import tkinter as tk
        from PIL import Image, ImageTk

        self._tk, self._ImageTk, self._PILImage = tk, ImageTk, Image
        self._own_root = tk._default_root is None
        self._root = tk._default_root or tk.Tk()
        if self._own_root:
            self._root.withdraw()          # 根只作事件宿主，不显示
        self._win = tk.Toplevel(self._root)
        self._win.title("vrobot SLAM 实时地图")
        self._label = tk.Label(self._win, bg="#202020")
        self._label.pack()
        self._win.protocol("WM_DELETE_WINDOW", self._on_close)
        self._photo = None                 # 持引用防 GC

    def _on_close(self) -> None:
        self.enabled = False               # 用户关窗 → 自动禁用
        try:
            self._win.destroy()
        except Exception:
            pass

    # ---- 主入口 ----

    def update(self, slam, gt_pose=None) -> None:
        if not self.enabled:
            return
        try:
            rgb = self._render(slam, gt_pose)
            if self._backend == "cv2":
                if not self._win_ready:
                    cv2.namedWindow(self._win_title, cv2.WINDOW_AUTOSIZE)
                    self._win_ready = True
                cv2.imshow(self._win_title, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)
                if cv2.getWindowProperty(self._win_title, cv2.WND_PROP_VISIBLE) < 1:
                    self.enabled = False
            else:
                photo = self._ImageTk.PhotoImage(self._PILImage.fromarray(rgb))
                self._label.configure(image=photo)
                self._photo = photo        # 防 GC
                self._label.update()       # 泵事件 + 立即重绘
        except Exception as e:
            log.debug("viewer update failed: %s", e)

    # ---- 帧合成（纯数组操作，无 GUI 依赖，可独立测试） ----

    def _render(self, slam, gt_pose) -> np.ndarray:
        prob = slam.grid.prob()
        # gray_r 语义：prob 0(自由)→白, 0.5(未知)→中灰, 1(墙)→黑
        gray = (255.0 * (1.0 - prob)).astype(np.uint8)
        h, w = gray.shape
        s = self.SCALE
        if HAS_CV2:
            img = cv2.cvtColor(
                cv2.resize(gray, (w * s, h * s), interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR,
            )
        else:  # 无 cv2 兜底：kron 放大（无绘制图元，只能画点块）
            img = np.kron(np.stack([gray] * 3, axis=-1),
                          np.ones((s, s, 1), dtype=np.uint8))

        # 标题条：静态中文模板 + ASCII 帧号（cv2.putText 不支持 CJK，帧号够用）
        img[:30, :self._caption_strip.shape[1]] = self._caption_strip
        if HAS_CV2:
            cv2.putText(img, f"frame={self._frame}",
                        (350, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (30, 30, 30), 2, cv2.LINE_AA)

        # 轨迹：连续线段（逐点色块会在高速移动时断续）
        traj = np.asarray(slam.trajectory, dtype=np.float64)
        if len(traj):
            pts = ((traj[:, :2] / self.cell_size - 0.5) * s + s * 0.5).astype(np.int32)
            if HAS_CV2:
                for i in range(1, len(pts)):
                    cv2.line(img, tuple(pts[i - 1]), tuple(pts[i]),
                             (0, 0, 255), 1, cv2.LINE_AA)
            else:
                for p in pts:
                    img[max(p[1] - 1, 0):p[1] + 2, max(p[0] - 1, 0):p[0] + 2] = (0, 0, 255)
            # 绘制顺序：真值绿环在下、机器人红点在上（重叠时两者都可见）
            if gt_pose is not None:
                gp = (int((gt_pose[0] / self.cell_size - 0.5) * s + s * 0.5),
                      int((gt_pose[1] / self.cell_size - 0.5) * s + s * 0.5))
                cv2.circle(img, gp, 8, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(img, tuple(pts[-1]), 5, (0, 0, 255), -1, cv2.LINE_AA)
        self._frame += 1
        return img

    def close(self) -> None:
        if self._backend == "cv2":
            try:
                cv2.destroyWindow(self._win_title)
            except Exception:
                pass
        else:
            try:
                self._win.destroy()
            except Exception:
                pass


def create_map_viewer(enabled: bool = True, cell_size: float = 8.0):
    """工厂：优先 FastMapViewer，后端不可用则回退 matplotlib MapViewer。"""
    if enabled:
        try:
            v = FastMapViewer(enabled=enabled, cell_size=cell_size)
            if v.enabled:
                return v
        except Exception as e:
            log.warning("FastMapViewer 创建失败，回退 matplotlib: %s", e)
    from vrobot.viz.map_viewer import MapViewer

    return MapViewer(enabled=enabled, cell_size=cell_size)
