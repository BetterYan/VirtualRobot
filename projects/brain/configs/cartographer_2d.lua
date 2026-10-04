-- VirtualRobot 2D SLAM 默认配置（无 IMU / 可选里程计）
-- 说明：LuaParameterDictionary 以本文件 return 的表作为参数字典根，
--       因此必须同时返回 MAP_BUILDER 与 TRAJECTORY_BUILDER 两个表。

include "map_builder.lua"
include "trajectory_builder.lua"

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 16.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return {
  MAP_BUILDER = MAP_BUILDER,
  TRAJECTORY_BUILDER = TRAJECTORY_BUILDER,
}
