extends RefCounted
## 机器人大脑：传感器进 → 指令出。纯逻辑，二期整体迁移到 Python 进程。
##
## 策略：弓字形全图遍历 + 障碍断点用 A* 就近衔接 + 低电量回充后续扫。

const Config = preload("res://scripts/core/config.gd")
const PathFinder = preload("res://scripts/core/planner/path_finder.gd")
const CoveragePlanner = preload("res://scripts/core/planner/coverage_planner.gd")
const BehaviorState = preload("res://scripts/core/behavior/behavior_state.gd")

var model
var state: int = BehaviorState.State.CLEAN
var battery: float = 1.0

# 运行看板统计
var current_algorithm := "弓字形覆盖"
var waypoints_total := 0
var waypoints_done := 0
var replans := 0

var _waypoints: Array[Vector2i] = []
var _skip := {}                       # 不可达路点集合（key 为 Vector2i）
var _path: Array[Vector2i] = []       # 当前正在执行的 A* 路径
var _path_i := 0
var _path_kind := 0                   # 0=无 1=清扫路点 2=回充


func setup(p_model, start_cell: Vector2i) -> void:
	model = p_model
	state = BehaviorState.State.CLEAN
	battery = 1.0
	_skip = {}
	_path = []
	_path_i = 0
	_path_kind = 0
	_waypoints = CoveragePlanner.plan(model)
	waypoints_total = _waypoints.size()
	waypoints_done = 0
	replans = 0
	current_algorithm = "弓字形覆盖"
	# 机器人位置所在的弓字路点视为已完成
	var idx := _waypoints.find(start_cell)
	if idx >= 0:
		_waypoints.remove_at(idx)


## 每物理帧调用。snapshot: {pos: Vector2, heading: float, collision: bool}
## 返回指令：{type:"goto", pos: Vector2} 或 {type:"stop"}
func update(snapshot: Dictionary, dt: float) -> Dictionary:
	var pos: Vector2 = snapshot["pos"]
	match state:
		BehaviorState.State.CLEAN:
			current_algorithm = "弓字形覆盖"
			battery = maxf(0.0, battery - Config.BATTERY_DRAIN * dt)
			if battery <= Config.BATTERY_LOW:
				_start_dock(pos)
				return _follow_path(pos)
			return _clean_step(pos)
		BehaviorState.State.DOCK:
			current_algorithm = "A* 回充导航"
			battery = maxf(0.0, battery - Config.BATTERY_DOCK_DRAIN * dt)
			var charger: Vector2 = model.cell_center(Config.CHARGER_CELL)
			if pos.distance_to(charger) <= Config.ARRIVE_DIST:
				state = BehaviorState.State.CHARGE
				return {"type": "stop"}
			return _follow_path(pos)
		BehaviorState.State.CHARGE:
			current_algorithm = "恒流充电管理"
			battery = minf(1.0, battery + Config.BATTERY_CHARGE * dt)
			if battery >= 0.995:
				state = BehaviorState.State.CLEAN
				_path = []
				_path_i = 0
				return {"type": "stop"}
			return {"type": "stop"}
		_:
			current_algorithm = "任务完成"
			return {"type": "stop"}


func _clean_step(pos: Vector2) -> Dictionary:
	var cur: Vector2i = model.cell_of(pos)
	if _path.is_empty():
		var wp := _next_uncovered()
		if wp.x < 0:
			# 全部覆盖完成：在桩边即进入终态，否则回充待命
			if pos.distance_to(model.cell_center(Config.CHARGER_CELL)) <= Config.ARRIVE_DIST:
				state = BehaviorState.State.DONE
				return {"type": "stop"}
			_start_dock(pos)
			return _follow_path(pos)
		_path = PathFinder.find_path(model, cur, wp)
		_path_i = 0
		replans += 1
		_path_kind = 1
		if _path.is_empty():
			# 不可达，跳过该路点，下一帧尝试下一个
			_skip[wp] = true
			_path_kind = 0
			return {"type": "stop"}
		if _path.size() > 1 and _path[0] == cur:
			_path_i = 1
	return _follow_path(pos)


func _start_dock(pos: Vector2) -> void:
	var cur: Vector2i = model.cell_of(pos)
	_path = PathFinder.find_path(model, cur, Config.CHARGER_CELL)
	_path_i = 0
	replans += 1
	_path_kind = 2
	if _path.size() > 1 and _path[0] == cur:
		_path_i = 1
	state = BehaviorState.State.DOCK


## 从弓字序列中取下一个未覆盖且未跳过的路点；没有则返回 (-1,-1)
func _next_uncovered() -> Vector2i:
	for wp in _waypoints:
		if _skip.has(wp):
			continue
		if not model.is_covered(wp):
			return wp
	return Vector2i(-1, -1)


## 沿 _path 逐点输出 goto；走完清空路径
func _follow_path(pos: Vector2) -> Dictionary:
	if _path.is_empty() or _path_i >= _path.size():
		_path = []
		_path_i = 0
		return {"type": "stop"}
	var target := _path[_path_i]
	var target_pos: Vector2 = model.cell_center(target)
	if pos.distance_to(target_pos) <= Config.ARRIVE_DIST:
		_path_i += 1
		if _path_i >= _path.size():
			# 本段路径走完：清扫路点计数，回充路径只清标记
			if _path_kind == 1:
				waypoints_done += 1
			_path_kind = 0
			_path = []
			_path_i = 0
			return {"type": "stop"}
		target = _path[_path_i]
		target_pos = model.cell_center(target)
	return {"type": "goto", "pos": target_pos}
