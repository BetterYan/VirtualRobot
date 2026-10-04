// 控制台冒烟测试：脱离 Python 直接跑 2D SLAM 全流程，用于隔离原生层问题。
// 构建后在 build/native/Release/map_builder_smoke.exe 运行。
#include <cmath>
#include <iostream>

#include "mapper.h"

namespace {

std::vector<double> SyntheticScan(double x, double y, int n = 180) {
  std::vector<double> pts;
  pts.reserve(n * 3);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * M_PI * i / n;
    const double r = 5.0 + 0.02 * std::sin(a * 7);
    pts.push_back(x + r * std::cos(a));
    pts.push_back(y + r * std::sin(a));
    pts.push_back(0.0);
  }
  return pts;
}

}  // namespace

int main() {
  const std::string lua_code = R"(
include "map_builder.lua"
include "trajectory_builder.lua"
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 16.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
POSE_GRAPH.optimize_every_n_nodes = 30
return { MAP_BUILDER = MAP_BUILDER, TRAJECTORY_BUILDER = TRAJECTORY_BUILDER }
)";

  std::cout << "[1] creating mapper..." << std::endl;
  vrobot::slam::Mapper mapper(
      lua_code, {"D:/Projects/VirtualRobot/projects/brain/configs"});
  std::cout << "[1] mapper created" << std::endl;

  std::cout << "[2] starting trajectory..." << std::endl;
  const int tid = mapper.start_trajectory("lidar", std::nullopt, std::nullopt);
  std::cout << "[2] trajectory id = " << tid << std::endl;

  const double t0 = 1000.0;
  for (int i = 0; i < 40; ++i) {
    mapper.add_scan(t0 + i * 0.1, SyntheticScan(0.01 * i, 0.0), 3,
                    {0., 0., 0.});
  }
  std::cout << "[3] 40 scans sent, local pose = ";
  if (const auto p = mapper.latest_local_pose()) {
    std::cout << "(" << (*p)[0] << ", " << (*p)[1] << ", " << (*p)[2] << ")";
  } else {
    std::cout << "none";
  }
  std::cout << ", nodes = " << mapper.num_nodes() << std::endl;

  std::cout << "[4] finishing trajectory..." << std::endl;
  mapper.finish_trajectory();
  std::cout << "[4] finished, submaps = " << mapper.num_submaps() << std::endl;

  std::cout << "[5] final optimization..." << std::endl;
  mapper.run_final_optimization();
  std::cout << "[5] done" << std::endl;

  const auto poses = mapper.optimized_poses();
  std::cout << "[6] optimized poses: " << poses.size() / 3 << " nodes"
            << std::endl;
  if (poses.size() >= 3) {
    std::cout << "    first = (" << poses[0] << ", " << poses[1] << ", "
              << poses[2] << ")" << std::endl;
    const size_t last = poses.size() - 3;
    std::cout << "    last  = (" << poses[last] << ", " << poses[last + 1]
              << ", " << poses[last + 2] << ")" << std::endl;
  }

  std::cout << "[7] write pbstream..." << std::endl;
  mapper.write_pbstream("smoke_map.pbstream", false);
  std::cout << "[7] OK -> smoke_map.pbstream" << std::endl;

  std::cout << "SMOKE TEST PASSED" << std::endl;
  return 0;
}
