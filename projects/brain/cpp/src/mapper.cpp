// Mapper 实现（纯 C++）。
#include "mapper.h"

#include <set>
#include <utility>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/sensor/rangefinder_point.h"

#ifndef CARTOGRAPHER_CONFIGURATION_FILES
#define CARTOGRAPHER_CONFIGURATION_FILES ""
#endif

namespace vrobot {
namespace slam {

namespace {
constexpr std::int64_t kTicksPerSecond = 10000000;  // UTS: 100ns ticks
}  // namespace

Time UnixToTime(double unix_seconds) {
  return cartographer::common::FromUniversal(
      static_cast<std::int64_t>(unix_seconds * kTicksPerSecond) +
      cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
          kTicksPerSecond);
}

double YawOf(const Rigid3d& pose) {
  const Eigen::Quaterniond& q = pose.rotation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

Rigid3d PoseFromXYYaw(double x, double y, double yaw) {
  return Rigid3d(Eigen::Vector3d(x, y, 0.0),
                 Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Mapper::Mapper(const std::string& lua_code,
               const std::vector<std::string>& search_paths) {
  std::vector<std::string> paths = search_paths;
  paths.push_back(CARTOGRAPHER_CONFIGURATION_FILES);

  auto file_resolver =
      std::make_unique<cartographer::common::ConfigurationFileResolver>(paths);
  auto lua = std::make_unique<cartographer::common::LuaParameterDictionary>(
      lua_code, std::move(file_resolver));

  auto map_builder_options = cartographer::mapping::CreateMapBuilderOptions(
      lua->GetDictionary("MAP_BUILDER").get());
  trajectory_options_ = cartographer::mapping::CreateTrajectoryBuilderOptions(
      lua->GetDictionary("TRAJECTORY_BUILDER").get());

  map_builder_ = std::make_unique<MapBuilder>(map_builder_options);
}

Mapper::~Mapper() {
  try {
    if (map_builder_ && !finished_) {
      map_builder_->FinishTrajectory(trajectory_id_);
      finished_ = true;
    }
  } catch (...) {
    // 析构期间不得抛异常
  }
}

int Mapper::start_trajectory(
    const std::string& range_id, const std::optional<std::string>& imu_id,
    const std::optional<std::string>& odom_id,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback) {
  std::set<TrajectoryBuilderInterface::SensorId> sensor_ids;
  sensor_ids.insert(
      {TrajectoryBuilderInterface::SensorId::SensorType::RANGE, range_id});
  range_id_ = range_id;

  if (imu_id) {
    imu_id_ = imu_id;
    sensor_ids.insert(
        {TrajectoryBuilderInterface::SensorId::SensorType::IMU, *imu_id_});
  } else {
    imu_id_.reset();
  }
  if (odom_id) {
    odom_id_ = odom_id;
    sensor_ids.insert({TrajectoryBuilderInterface::SensorId::SensorType::ODOMETRY,
                       *odom_id_});
  } else {
    odom_id_.reset();
  }

  trajectory_id_ = map_builder_->AddTrajectoryBuilder(
      sensor_ids, trajectory_options_, local_slam_result_callback);
  finished_ = false;
  return trajectory_id_;
}

void Mapper::add_scan(double timestamp, const std::vector<double>& points,
                      int stride, const std::vector<double>& origin) {
  if (stride != 3 && stride != 4) {
    throw std::invalid_argument("points stride must be 3 or 4");
  }
  if (origin.size() != 3) {
    throw std::invalid_argument("origin must have 3 elements [x, y, z]");
  }
  if (points.size() % stride != 0) {
    throw std::invalid_argument("points size not a multiple of stride");
  }

  cartographer::sensor::TimedPointCloudData cloud;
  cloud.time = UnixToTime(timestamp);
  cloud.origin = Eigen::Vector3f(static_cast<float>(origin[0]),
                                 static_cast<float>(origin[1]),
                                 static_cast<float>(origin[2]));
  const size_t n = points.size() / stride;
  cloud.ranges.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double* p = &points[i * stride];
    const float rel_time = stride == 4 ? static_cast<float>(p[3]) : 0.f;
    cloud.ranges.push_back(cartographer::sensor::TimedRangefinderPoint{
        Eigen::Vector3f(static_cast<float>(p[0]), static_cast<float>(p[1]),
                        static_cast<float>(p[2])),
        rel_time});
  }
  builder()->AddSensorData(range_id_, cloud);
}

void Mapper::add_imu(double timestamp, const std::vector<double>& accel,
                     const std::vector<double>& gyro) {
  if (!imu_id_) {
    throw std::logic_error("no IMU sensor configured");
  }
  if (accel.size() != 3 || gyro.size() != 3) {
    throw std::invalid_argument("accel/gyro must have 3 elements each");
  }
  cartographer::sensor::ImuData data;
  data.time = UnixToTime(timestamp);
  data.linear_acceleration = Eigen::Vector3d(accel[0], accel[1], accel[2]);
  data.angular_velocity = Eigen::Vector3d(gyro[0], gyro[1], gyro[2]);
  builder()->AddSensorData(*imu_id_, data);
}

void Mapper::add_odometry(double timestamp, double x, double y, double yaw) {
  if (!odom_id_) {
    throw std::logic_error("no odometry sensor configured");
  }
  cartographer::sensor::OdometryData data;
  data.time = UnixToTime(timestamp);
  data.pose = PoseFromXYYaw(x, y, yaw);
  builder()->AddSensorData(*odom_id_, data);
}

void Mapper::finish_trajectory() {
  if (finished_) return;
  map_builder_->FinishTrajectory(trajectory_id_);
  finished_ = true;
}

void Mapper::run_final_optimization() {
  map_builder_->pose_graph()->RunFinalOptimization();
}

std::vector<double> Mapper::optimized_poses() const {
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  std::vector<double> rows;
  for (const auto& node : node_poses.trajectory(trajectory_id_)) {
    const Rigid3d& p = node.data.global_pose;
    rows.push_back(p.translation().x());
    rows.push_back(p.translation().y());
    rows.push_back(YawOf(p));
  }
  return rows;
}

std::vector<SubmapGridData> Mapper::submap_grids() const {
  std::vector<SubmapGridData> out;
  const auto all = map_builder_->pose_graph()->GetAllSubmapData();
  out.reserve(all.size());
  for (const auto& entry : all) {
    const auto* submap_2d = dynamic_cast<const cartographer::mapping::Submap2D*>(
        entry.data.submap.get());
    if (submap_2d == nullptr || submap_2d->grid() == nullptr) continue;
    const auto* grid = submap_2d->grid();
    const auto& limits = grid->limits();
    const auto& cell_limits = limits.cell_limits();
    if (cell_limits.num_x_cells <= 0 || cell_limits.num_y_cells <= 0) continue;

    SubmapGridData g;
    g.resolution = limits.resolution();
    // cartographer 约定：cell_index.x() 是"行"（界 num_x_cells，沿世界Y递减），
    // cell_index.y() 是"列"（界 num_y_cells，沿世界X递减）。导出为图像语义。
    g.height = cell_limits.num_x_cells;
    g.width = cell_limits.num_y_cells;
    g.finished = submap_2d->insertion_finished();
    const Eigen::Vector2f c00 = limits.GetCellCenter(Eigen::Array2i(0, 0));
    g.origin_x = c00.x();
    g.origin_y = c00.y();
    const Rigid3d& pose = entry.data.pose;
    g.pose_x = pose.translation().x();
    g.pose_y = pose.translation().y();
    g.pose_yaw = YawOf(pose);

    const size_t n = static_cast<size_t>(g.width) * g.height;
    g.prob.resize(n);
    g.known.resize(n);
    for (int row = 0; row < g.height; ++row) {
      for (int col = 0; col < g.width; ++col) {
        const Eigen::Array2i idx(row, col);
        const size_t flat = static_cast<size_t>(row) * g.width + col;
        g.known[flat] = grid->IsKnown(idx) ? 1 : 0;
        // correspondence cost（越低越占据）-> 占据概率
        g.prob[flat] = 1.0f - grid->GetCorrespondenceCost(idx);
      }
    }
    out.push_back(std::move(g));
  }
  return out;
}

void Mapper::write_pbstream(const std::string& path,
                            bool include_unfinished) const {
  if (!map_builder_->SerializeStateToFile(include_unfinished, path)) {
    throw std::runtime_error("failed to write pbstream: " + path);
  }
}

void Mapper::load_state(const std::string& path, bool frozen) {
  map_builder_->LoadStateFromFile(path, frozen);
}

int Mapper::num_nodes() const {
  return static_cast<int>(
      map_builder_->pose_graph()->GetTrajectoryNodePoses().size());
}

int Mapper::num_submaps() const {
  return static_cast<int>(
      map_builder_->pose_graph()->GetAllSubmapPoses().size());
}

std::optional<std::array<double, 3>> Mapper::latest_local_pose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_local_pose_) return std::nullopt;
  return std::array<double, 3>{last_local_pose_.translation().x(),
                               last_local_pose_.translation().y(),
                               YawOf(last_local_pose_)};
}

void Mapper::notify_local_slam_result(cartographer::common::Time time,
                                      Rigid3d local_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_local_pose_ = std::move(local_pose);
  last_local_pose_time_ = time;
  have_local_pose_ = true;
}

TrajectoryBuilderInterface* Mapper::builder() const {
  auto* b = map_builder_->GetTrajectoryBuilder(trajectory_id_);
  if (b == nullptr) {
    throw std::logic_error("trajectory not started; call start_trajectory()");
  }
  return b;
}

}  // namespace slam
}  // namespace vrobot
