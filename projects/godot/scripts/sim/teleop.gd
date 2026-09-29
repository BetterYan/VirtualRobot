extends RefCounted
## 键盘遥控（manual 模式）：方向键 / WASD → (linear, angular)。

const LIN := 60.0
const ANG := 2.0


static func read_input() -> Vector2:
	## x = linear（前+），y = angular（左转为负？与 Godot rotation 一致：正 = 顺时针视觉/数学逆时针 y 系）
	var lin := 0.0
	var ang := 0.0
	if Input.is_key_pressed(KEY_UP) or Input.is_key_pressed(KEY_W):
		lin += LIN
	if Input.is_key_pressed(KEY_DOWN) or Input.is_key_pressed(KEY_S):
		lin -= LIN
	if Input.is_key_pressed(KEY_LEFT) or Input.is_key_pressed(KEY_A):
		ang -= ANG
	if Input.is_key_pressed(KEY_RIGHT) or Input.is_key_pressed(KEY_D):
		ang += ANG
	return Vector2(lin, ang)
