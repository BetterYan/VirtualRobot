extends CanvasLayer
## HUD：状态 / 覆盖率 / 电量 显示 + 开始暂停 / 重置 按钮。代码构建 UI。

signal start_toggled(running: bool)
signal reset_requested

var _lbl_state: Label
var _lbl_cov: Label
var _lbl_bat: Label
var _btn_start: Button


func _ready() -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(16, 16)
	add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	panel.add_child(vbox)

	_lbl_state = _make_label(vbox, "状态: IDLE")
	_lbl_cov = _make_label(vbox, "覆盖率: 0.0%")
	_lbl_bat = _make_label(vbox, "电量: 100%")

	var btn_box := HBoxContainer.new()
	btn_box.add_theme_constant_override("separation", 12)
	btn_box.position = Vector2(16, 740)  # 窗口固定 1280x800，固定坐标可靠
	add_child(btn_box)

	_btn_start = Button.new()
	_btn_start.text = "开始"
	_btn_start.custom_minimum_size = Vector2(90, 36)
	_btn_start.pressed.connect(_on_start_pressed)
	btn_box.add_child(_btn_start)

	var btn_reset := Button.new()
	btn_reset.text = "重置"
	btn_reset.custom_minimum_size = Vector2(90, 36)
	btn_reset.pressed.connect(func() -> void: reset_requested.emit())
	btn_box.add_child(btn_reset)


func _make_label(parent: Node, text: String) -> Label:
	var lbl := Label.new()
	lbl.text = text
	parent.add_child(lbl)
	return lbl


func _on_start_pressed() -> void:
	var running := _btn_start.text == "开始"
	_btn_start.text = "暂停" if running else "开始"
	start_toggled.emit(running)


func set_running(running: bool) -> void:
	_btn_start.text = "暂停" if running else "开始"


func set_stats(state_name: String, coverage: float, battery: float) -> void:
	_lbl_state.text = "状态: %s" % state_name
	_lbl_cov.text = "覆盖率: %.1f%%" % (coverage * 100.0)
	_lbl_bat.text = "电量: %d%%" % int(round(battery * 100.0))
