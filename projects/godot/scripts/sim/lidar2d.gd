extends Node2D
## 2D 激光雷达仿真：绕机体一圈射线扫描 + 高斯测距噪声。
## 挂在机器人下（作为子节点跟随位姿），每帧产出 ranges 数组。

signal scanned(ranges: PackedFloat32Array)

var beams := 72
var range_max := 320.0
var noise_std := 2.0
var angle_min := -PI
var angle_step := TAU / 72.0

var last_ranges := PackedFloat32Array()
var _rng := RandomNumberGenerator.new()

# 复用查询参数，避免每条射线新建对象
var _query := PhysicsRayQueryParameters2D.new()


func _ready() -> void:
	angle_step = TAU / beams
	_rng.randomize()
	last_ranges.resize(beams)
	_query.exclude = [get_parent().get_rid()]
	_query.collision_mask = 1


func scan() -> PackedFloat32Array:
	## 以父节点（机器人）位姿打一圈射线，返回带噪距离（无回波 = -1）
	var space := get_world_2d().direct_space_state
	var origin := global_position
	var heading := global_rotation
	var out := PackedFloat32Array()
	out.resize(beams)
	for i in beams:
		var a := heading + angle_min + i * angle_step
		var dir := Vector2(cos(a), sin(a))
		_query.from = origin
		_query.to = origin + dir * range_max
		var hit := space.intersect_ray(_query)
		if hit.is_empty():
			out[i] = -1.0
		else:
			var d: float = origin.distance_to(hit.position)
			out[i] = maxf(0.0, d + _rng.randfn() * noise_std)
	last_ranges = out
	scanned.emit(out)
	return out
