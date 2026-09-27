extends CharacterBody3D
## 3D 机器人实体：消费大脑指令驱动移动，产出传感器快照（含 36 线水平雷达）。
## 逻辑坐标约定：2D (x, y) 映射到 3D (x, 0, z)，机身贴地 y=0。

const Config = preload("res://scripts/core/config.gd")

const LIDAR_RAYS := 36
const LIDAR_RANGE := 220.0

var _speed := Config.MOVE_SPEED
var _turn_speed := Config.TURN_SPEED
var _target_point := Vector2.ZERO   # 2D 逻辑坐标 (x, z)
var _has_target := false


func _ready() -> void:
	var shape := CollisionShape3D.new()
	var cyl := CylinderShape3D.new()
	cyl.radius = Config.ROBOT_RADIUS
	cyl.height = 8.0
	shape.shape = cyl
	shape.position = Vector3(0, 4, 0)
	add_child(shape)
	_build_visual()


func _build_visual() -> void:
	var body_mat := StandardMaterial3D.new()
	body_mat.albedo_color = Color(0.3, 0.55, 0.95)
	body_mat.roughness = 0.4
	body_mat.metallic = 0.3
	var mesh := MeshInstance3D.new()
	var cyl := CylinderMesh.new()
	cyl.top_radius = Config.ROBOT_RADIUS
	cyl.bottom_radius = Config.ROBOT_RADIUS
	cyl.height = 8.0
	mesh.mesh = cyl
	mesh.material_override = body_mat
	mesh.position = Vector3(0, 4, 0)
	add_child(mesh)

	var top_mat := StandardMaterial3D.new()
	top_mat.albedo_color = Color(0.85, 0.88, 0.95)
	top_mat.roughness = 0.3
	var top := MeshInstance3D.new()
	var disc := CylinderMesh.new()
	disc.top_radius = Config.ROBOT_RADIUS * 0.55
	disc.bottom_radius = Config.ROBOT_RADIUS * 0.55
	disc.height = 1.0
	top.mesh = disc
	top.material_override = top_mat
	top.position = Vector3(0, 8.5, 0)
	add_child(top)

	var ind_mat := StandardMaterial3D.new()
	ind_mat.albedo_color = Color(0.15, 0.3, 0.6)
	ind_mat.emission_enabled = true
	ind_mat.emission = Color(0.1, 0.5, 1.0)
	ind_mat.emission_energy_multiplier = 1.2
	var ind := MeshInstance3D.new()
	var box := BoxMesh.new()
	box.size = Vector3(8, 2, 3)
	ind.mesh = box
	ind.material_override = ind_mat
	ind.position = Vector3(Config.ROBOT_RADIUS * 0.55, 8.5, 0)
	add_child(ind)


## 执行大脑指令：goto 为目标点（2D 逻辑坐标），其余一律停车
func execute(cmd: Dictionary) -> void:
	if cmd.get("type", "stop") == "goto":
		_target_point = cmd["pos"]
		_has_target = true
	else:
		_has_target = false


func _physics_process(dt: float) -> void:
	if not _has_target:
		velocity = Vector3.ZERO
		return
	var to_target := Vector3(_target_point.x, global_position.y, _target_point.y) - global_position
	if Vector2(to_target.x, to_target.z).length() < 2.0:
		velocity = Vector3.ZERO
		return
	# 3D 偏航角 phi 对应 2D 航向角 theta = -phi
	var desired := atan2(-to_target.z, to_target.x)
	var diff := wrapf(desired - rotation.y, -PI, PI)
	var max_turn := _turn_speed * dt
	rotation.y += clampf(diff, -max_turn, max_turn)
	var forward := _speed
	if absf(diff) > 0.6:
		forward *= 0.25
	velocity = Vector3.RIGHT.rotated(Vector3.UP, rotation.y) * forward
	move_and_slide()


## 传感器快照（上行数据，字段与 docs/01 接口定义一致）
func get_snapshot() -> Dictionary:
	return {
		"pos": Vector2(global_position.x, global_position.z),
		"heading": -rotation.y,
		"collision": get_slide_collision_count() > 0,
		"lidar_hits": _scan_lidar(),
	}


## 36 线水平雷达：返回各方向命中距离（未命中返回满量程）
func _scan_lidar() -> Array[float]:
	var hits: Array[float] = []
	var space := get_world_3d().direct_space_state
	var origin := global_position + Vector3(0, 5, 0)
	for i in LIDAR_RAYS:
		var ang := TAU * float(i) / float(LIDAR_RAYS)
		var dir := Vector3(cos(ang), 0.0, sin(ang))
		var q := PhysicsRayQueryParameters3D.create(origin, origin + dir * LIDAR_RANGE)
		q.exclude = [get_rid()]
		var hit := space.intersect_ray(q)
		if hit.is_empty():
			hits.append(LIDAR_RANGE)
		else:
			hits.append(origin.distance_to(hit["position"]))
	return hits
