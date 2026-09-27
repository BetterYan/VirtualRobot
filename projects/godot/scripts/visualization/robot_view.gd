extends CharacterBody2D
## 机器人实体：消费大脑指令驱动移动，产出传感器快照。只做"躯体"，不算逻辑。

const Config = preload("res://scripts/core/config.gd")

var _speed := Config.MOVE_SPEED
var _turn_speed := Config.TURN_SPEED
var _target_point := Vector2.ZERO
var _has_target := false


func _ready() -> void:
	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = Config.ROBOT_RADIUS
	shape.shape = circle
	add_child(shape)
	queue_redraw()


func _draw() -> void:
	# 机身
	draw_circle(Vector2.ZERO, Config.ROBOT_RADIUS, Color(0.25, 0.5, 0.9))
	draw_circle(Vector2.ZERO, Config.ROBOT_RADIUS * 0.55, Color(0.85, 0.88, 0.95))
	# 朝向指示
	draw_circle(Vector2(Config.ROBOT_RADIUS * 0.55, 0), 3.0, Color(0.15, 0.3, 0.6))


## 执行大脑指令
func execute(cmd: Dictionary) -> void:
	match cmd.get("type", "stop"):
		"goto":
			_target_point = cmd["pos"]
			_has_target = true
		_:
			_has_target = false


func _physics_process(dt: float) -> void:
	if not _has_target:
		velocity = Vector2.ZERO
		return
	var to_target := _target_point - global_position
	if to_target.length() < 2.0:
		velocity = Vector2.ZERO
		return
	var desired := to_target.angle()
	var diff := wrapf(desired - rotation, -PI, PI)
	var max_turn := _turn_speed * dt
	rotation += clampf(diff, -max_turn, max_turn)
	var forward := _speed
	if absf(diff) > 0.6:
		forward *= 0.25  # 大幅转向时减速
	velocity = Vector2.RIGHT.rotated(rotation) * forward
	move_and_slide()


## 传感器快照（上行数据，与 protocols/schema 对齐的字段）
func get_snapshot() -> Dictionary:
	return {
		"pos": global_position,
		"heading": rotation,
		"collision": get_slide_collision_count() > 0,
		"lidar_hits": [],  # 预留：二期雷达仿真
	}
