extends CanvasLayer
## 运行看板：决策流水线高亮 + 实时指标 + 控制按钮。代码构建 UI。

signal start_toggled(running: bool)
signal reset_requested
signal view_toggle_requested(overview: bool)

const STAGES := ["传感器采集", "栅格世界模型", "弓字形覆盖规划", "A* 路径搜索", "运动控制执行"]

var _btn_start: Button
var _btn_view: Button
var _lbl_state: Label
var _lbl_algo: Label
var _stages: Array[Label] = []
var _lbl_cov: Label
var _lbl_bat: Label
var _lbl_wp: Label
var _lbl_replan: Label
var _lbl_lidar: Label
var _lbl_col: Label
var _lbl_time: Label


func _ready() -> void:
	_build_panel()
	_build_buttons()


func _build_panel() -> void:
	var panel := PanelContainer.new()
	panel.position = Vector2(984, 16)
	panel.custom_minimum_size = Vector2(280, 0)
	add_child(panel)
	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 6)
	panel.add_child(vbox)

	var title := Label.new()
	title.text = "机器人运行看板"
	vbox.add_child(title)

	_lbl_state = _label(vbox, "状态机: CLEAN")
	_lbl_algo = _label(vbox, "当前算法: 弓字形覆盖")

	_sep(vbox, "── 决策流水线 ──")
	for stage in STAGES:
		_stages.append(_label(vbox, "○ " + stage))

	_sep(vbox, "── 实时指标 ──")
	_lbl_cov = _label(vbox, "覆盖率: 0.0%")
	_lbl_bat = _label(vbox, "电量: 100%")
	_lbl_wp = _label(vbox, "路点进度: 0/0")
	_lbl_replan = _label(vbox, "A* 重规划: 0 次")
	_lbl_lidar = _label(vbox, "雷达最近: —")
	_lbl_col = _label(vbox, "碰撞: 否")
	_lbl_time = _label(vbox, "运行时长: 00:00")


func _build_buttons() -> void:
	var box := HBoxContainer.new()
	box.add_theme_constant_override("separation", 12)
	box.position = Vector2(16, 740)
	add_child(box)

	_btn_start = Button.new()
	_btn_start.text = "开始"
	_btn_start.custom_minimum_size = Vector2(90, 36)
	_btn_start.pressed.connect(_on_start_pressed)
	box.add_child(_btn_start)

	var btn_reset := Button.new()
	btn_reset.text = "重置"
	btn_reset.custom_minimum_size = Vector2(90, 36)
	btn_reset.pressed.connect(func() -> void: reset_requested.emit())
	box.add_child(btn_reset)

	_btn_view = Button.new()
	_btn_view.text = "视角: 全景"
	_btn_view.custom_minimum_size = Vector2(110, 36)
	_btn_view.pressed.connect(_on_view_pressed)
	box.add_child(_btn_view)


func _label(parent: Control, text: String) -> Label:
	var lbl := Label.new()
	lbl.text = text
	parent.add_child(lbl)
	return lbl


func _sep(parent: Control, text: String) -> void:
	var lbl := Label.new()
	lbl.text = text
	lbl.add_theme_color_override("font_color", Color(0.6, 0.62, 0.68))
	parent.add_child(lbl)


func _on_start_pressed() -> void:
	var running := _btn_start.text == "开始"
	set_running(running)
	start_toggled.emit(running)


func _on_view_pressed() -> void:
	var overview := _btn_view.text.contains("全景")
	_btn_view.text = "视角: 跟随" if overview else "视角: 全景"
	view_toggle_requested.emit(overview)


func set_running(running: bool) -> void:
	_btn_start.text = "暂停" if running else "开始"


func set_status(state_name: String, algorithm: String, is_running: bool) -> void:
	_lbl_state.text = "状态机: %s%s" % [state_name, "（运行中）" if is_running else "（已暂停）"]
	_lbl_algo.text = "当前算法: %s" % algorithm
	var active := -1
	match algorithm:
		"弓字形覆盖":
			active = 4
		"A* 回充导航":
			active = 3
		"恒流充电管理":
			active = 1
	for i in _stages.size():
		var on := is_running and i <= active
		_stages[i].text = ("● " if on else "○ ") + STAGES[i]
		_stages[i].add_theme_color_override("font_color",
				Color(0.35, 0.9, 0.55) if on else Color(0.6, 0.6, 0.6))


func set_metrics(coverage: float, battery: float, wp_text: String, replans: int,
		lidar_min: float, collision: bool, elapsed_text: String) -> void:
	_lbl_cov.text = "覆盖率: %.1f%%" % (coverage * 100.0)
	_lbl_bat.text = "电量: %d%%" % int(round(battery * 100.0))
	_lbl_wp.text = "路点进度: %s" % wp_text
	_lbl_replan.text = "A* 重规划: %d 次" % replans
	_lbl_lidar.text = "雷达最近: %s" % (("%.0f px" % lidar_min) if lidar_min >= 0.0 else "—")
	_lbl_col.text = "碰撞: %s" % ("是" if collision else "否")
	_lbl_col.add_theme_color_override("font_color",
			Color(0.95, 0.4, 0.35) if collision else Color(1, 1, 1, 1))
	_lbl_time.text = "运行时长: %s" % elapsed_text
