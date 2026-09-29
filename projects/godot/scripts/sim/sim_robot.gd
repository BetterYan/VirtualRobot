extends CharacterBody2D
## 差速驱动机器人"躯体"：消费 (linear, angular) 速度指令，提供碰撞标志。
## 只做物理执行，不算逻辑（与一期 robot_view.gd 同哲学）。

const RADIUS := 14.0

var linear := 0.0    # px/s
var angular := 0.0   # rad/s
var collided := false
## 允许移动的开关：manual 模式下由遥控写入，auto 模式下由 cmd_vel 写入
var control_enabled := true

# cmd_vel 安全超时：超过该时长未刷新指令则停车（协议规定 0.5s）
var _cmd_age := 999.0


func _ready() -> void:
	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = RADIUS
	shape.shape = circle
	add_child(shape)
	set_z_index(5)
	queue_redraw()


func _draw() -> void:
	draw_circle(Vector2.ZERO, RADIUS, Color(0.25, 0.5, 0.9))
	draw_circle(Vector2.ZERO, RADIUS * 0.55, Color(0.85, 0.88, 0.95))
	draw_circle(Vector2(RADIUS * 0.55, 0), 3.0, Color(0.15, 0.3, 0.6))


## 外部指令入口（遥控或 cmd_vel），自动刷新安全计时器
func command(p_linear: float, p_angular: float) -> void:
	linear = clampf(p_linear, -90.0, 90.0)
	angular = clampf(p_angular, -3.0, 3.0)
	_cmd_age = 0.0


func stop() -> void:
	linear = 0.0
	angular = 0.0


func _physics_process(dt: float) -> void:
	_cmd_age += dt
	if _cmd_age > 0.5 or not control_enabled:
		stop()
	rotation += angular * dt
	velocity = Vector2.RIGHT.rotated(rotation) * linear
	move_and_slide()
	collided = get_slide_collision_count() > 0
	queue_redraw()
