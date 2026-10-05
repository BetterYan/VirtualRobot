"""GUI 事件泵：asyncio 主循环里没有常驻的 Tk mainloop 处理窗口消息，
Windows 会在消息超时（5s）后把窗口标记为"未响应"并冻结重绘
—— 表现为窗口停在早期帧、只见轨迹不见地图刷新。

不用 plt.pause()/show()：那会把窗口反复抬到 Z 序顶端并抢焦点。
直接调用 Tk widget 的 update() 处理全部待处理消息；非 Tk 后端退回 flush_events。
"""

from __future__ import annotations


def pump(fig) -> None:
    """重绘 fig 并泵其 GUI 事件循环。"""
    canvas = fig.canvas
    canvas.draw_idle()
    try:
        widget = canvas.get_tk_widget()
        widget.update_idletasks()
        widget.update()
    except Exception:
        canvas.flush_events()
