extends RefCounted
## 带噪里程计模型：在真实速度上叠加比例噪声后自行积分，
## 与真值位姿完全解耦（SLAM 端只能拿到这里的输出）。

var noise_std := 0.02   # 速度比例噪声
var pose := Vector3.ZERO  # (x, y, theta) 里程计坐标系

var _rng := RandomNumberGenerator.new()


func _init(start: Vector3 = Vector3.ZERO, p_noise: float = 0.02) -> void:
	pose = start
	noise_std = p_noise
	_rng.randomize()


func reset(start: Vector3) -> void:
	pose = start


func integrate(true_linear: float, true_angular: float, dt: float) -> Vector3:
	## 每物理帧调用：噪声速度积分，返回当前里程计位姿
	var v := true_linear * (1.0 + _rng.randf_range(-noise_std, noise_std))
	var w := true_angular * (1.0 + _rng.randf_range(-noise_std, noise_std))
	var th := pose.z + w * dt
	pose = Vector3(pose.x + v * cos(th) * dt, pose.y + v * sin(th) * dt, th)
	return pose
