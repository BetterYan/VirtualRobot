// Mapper - Cartographer MapBuilder 的薄封装（纯 C++，无 pybind 依赖）。
// 供 pybind11 绑定与控制台冒烟测试共用。
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace vrobot {
namespace slam {

using cartographer::common::Time;
using cartographer::mapping::MapBuilder;
using cartographer::mapping::TrajectoryBuilderInterface;
using cartographer::transform::Rigid3d;

// 一个 Cartographer 子图的概率栅格快照（局部系几何 + 全局位姿）。
// 注意：导出时已把 cartographer 的 (cell.x=row 反向Y, cell.y=col 反向X)
// 约定规整为图像语义：row 0/col 0 = origin 角，行向 = 世界Y递减，列向 = 世界X递减。
struct SubmapGridData {
  double origin_x = 0.0;       // cell(row0,col0) 中心的世界坐标（局部系，米）
  double origin_y = 0.0;
  double resolution = 0.05;    // 米/格
  int width = 0;               // 列数（沿世界X递减方向）
  int height = 0;              // 行数（沿世界Y递减方向）
  bool finished = false;
  double pose_x = 0.0;         // 子图全局位姿（位姿图优化后）
  double pose_y = 0.0;
  double pose_yaw = 0.0;
  std::vector<float> prob;     // (height,width) 行主序，占据概率（未知格为兜底值）
  std::vector<unsigned char> known;  // (height,width) 1=已观测
};

// Unix 秒 -> Cartographer UTS 时间（UTS 纪元为公元 1 年，见 cartographer/common/time.h）
Time UnixToTime(double unix_seconds);
double YawOf(const Rigid3d& pose);
Rigid3d PoseFromXYYaw(double x, double y, double yaw);

// 一次 SLAM 会话：Lua 配置 -> MapBuilder -> 单轨迹数据输入 -> 位姿/地图输出。
class Mapper {
 public:
  // lua_code: 完整 Lua 配置文本（需含 MAP_BUILDER / TRAJECTORY_BUILDER 表，
  //           可 include "map_builder.lua" 等官方默认配置）。
  // search_paths: include 解析搜索路径（按优先级）。
  Mapper(const std::string& lua_code,
         const std::vector<std::string>& search_paths);
  // 未 finish_trajectory() 就析构会触发 cartographer 内部
  // OrderedMultiQueue 的 CHECK 失败（abort），析构时自动补齐。
  ~Mapper();

  int start_trajectory(const std::string& range_id,
                       const std::optional<std::string>& imu_id,
                       const std::optional<std::string>& odom_id,
                       const TrajectoryBuilderInterface::LocalSlamResultCallback&
                           local_slam_result_callback = nullptr);

  // points: 交错数组 [x0,y0,z0,(t0), x1,y1,z1,(t1), ...]，每点 3 或 4 个 double。
  void add_scan(double timestamp, const std::vector<double>& points,
                int stride, const std::vector<double>& origin);
  void add_imu(double timestamp, const std::vector<double>& accel,
               const std::vector<double>& gyro);
  void add_odometry(double timestamp, double x, double y, double yaw);

  void finish_trajectory();
  void run_final_optimization();

  // 全局优化后的轨迹节点位姿，交错数组 [x,y,yaw,...]。
  std::vector<double> optimized_poses() const;

  // 导出全部子图的概率栅格（Cartographer 的建图结果）。
  std::vector<SubmapGridData> submap_grids() const;

  void write_pbstream(const std::string& path, bool include_unfinished) const;
  void load_state(const std::string& path, bool frozen);

  int trajectory_id() const { return trajectory_id_; }
  int num_nodes() const;
  int num_submaps() const;

  // 最近一次 local SLAM 位姿（局部系），无节点时返回 nullopt。
  std::optional<std::array<double, 3>> latest_local_pose() const;

  // 供 local SLAM 结果回调线程写入（线程安全）。
  void notify_local_slam_result(cartographer::common::Time time,
                                Rigid3d local_pose);

 private:
  TrajectoryBuilderInterface* builder() const;

  std::unique_ptr<MapBuilder> map_builder_;
  cartographer::mapping::proto::TrajectoryBuilderOptions trajectory_options_;
  std::string range_id_;
  std::optional<std::string> imu_id_;
  std::optional<std::string> odom_id_;
  int trajectory_id_ = -1;
  bool finished_ = true;

  mutable std::mutex mutex_;
  bool have_local_pose_ = false;
  Rigid3d last_local_pose_ = Rigid3d::Identity();
  cartographer::common::Time last_local_pose_time_;
};

}  // namespace slam
}  // namespace vrobot
