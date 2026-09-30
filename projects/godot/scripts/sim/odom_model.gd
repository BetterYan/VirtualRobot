extends RefCounted
## 带噪里程计模型：在真实速度上叠加高斯比例噪声后自行积分，
## 与真值位姿完全解耦（SLAM 端只能拿到这里的输出）。

var noise_std := 0.02 # 速度比例噪声（高斯标准差，每帧独立采样）
var pose := Vector3.ZERO # (x, y, theta) 里程计坐标系

var _rng := RandomNumberGenerator.new()


func _init(start: Vector3 = Vector3.ZERO, p_noise: float = 0.02) -> void:
	pose = start
	noise_std = p_noise
	_rng.randomize()


func reset(start: Vector3) -> void:
	pose = start


func integrate(true_linear: float, true_angular: float, dt: float) -> Vector3:
	## 每物理帧调用：以"编码器测得的速度"（真值 + 高斯比例噪声）做中点积分，
	## 返回当前里程计位姿。
	# 1) 里程计感知的速度：轮速误差（打滑/量化）近似高斯，与速度成比例
	var v := true_linear * (1.0 + _rng.randfn(0.0, noise_std))
	var w := true_angular * (1.0 + _rng.randfn(0.0, noise_std))
	# 2) 中点积分：位移方向取帧首尾航向均值，避免用更新后航向造成的系统性外偏
	var th_mid := pose.z + w * dt * 0.5
	var th_new := pose.z + w * dt
	# 3) 航向角归约到 [0, TAU)，防止长期运行浮点精度退化
	pose = Vector3(
		pose.x + v * cos(th_mid) * dt,
		pose.y + v * sin(th_mid) * dt,
		fposmod(th_new, TAU)
	)
	return pose
