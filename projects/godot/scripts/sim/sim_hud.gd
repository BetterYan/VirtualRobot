extends CanvasLayer
## 仿真 HUD：连接状态、模式、上行帧率、真值/里程计对照。

var _label: Label
var _btn_mode: Button
var _lines := {}


func _ready() -> void:
	_label = Label.new()
	_label.position = Vector2(12, 10)
	_label.add_theme_font_size_override("font_size", 14)
	add_child(_label)

	_btn_mode = Button.new()
	_btn_mode.text = "模式：遥控 (manual)"
	_btn_mode.position = Vector2(12, 190)
	_btn_mode.pressed.connect(func() -> void: mode_toggle_requested.emit())
	add_child(_btn_mode)


signal mode_toggle_requested


func update_text(ws_ok: bool, mode: String, send_hz: float, seq: int,
		truth: Vector3, odom: Vector3) -> void:
	_label.text = "\n".join([
		"WS: %s   模式: %s" % ["已连接" if ws_ok else "未连接（等待 Python 端 ws://127.0.0.1:9094）", mode],
		"上行: %.1f Hz   seq=%d" % [send_hz, seq],
		"真值:   (%6.1f, %6.1f, %5.2f)" % [truth.x, truth.y, truth.z],
		"里程计: (%6.1f, %6.1f, %5.2f)" % [odom.x, odom.y, odom.z],
		"键盘: 方向键/WASD 遥控（manual 模式）",
	])


func set_mode_text(mode: String) -> void:
	_btn_mode.text = "模式：%s" % ("自动 (auto)" if mode == "auto" else "遥控 (manual)")
