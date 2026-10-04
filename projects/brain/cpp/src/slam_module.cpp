// vrobot_slam_native - pybind11 绑定层。
// 全部业务逻辑在 mapper.h/cpp（纯 C++），此文件只做 Python 类型转换。
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstring>
#include <stdexcept>

#include "mapper.h"

#ifndef CARTOGRAPHER_CONFIGURATION_FILES
#define CARTOGRAPHER_CONFIGURATION_FILES ""
#endif

namespace py = pybind11;

namespace {

// 把 C++ 异常翻译为 Python 异常的通用包装
template <typename Fn>
auto guarded(Fn&& fn) -> decltype(fn()) {
  try {
    return fn();
  } catch (py::builtin_exception&) {
    throw;  // pybind 自身的异常原样转发（也是 std::exception，必须先接住）
  } catch (const std::invalid_argument& e) {
    throw py::value_error(e.what());
  } catch (const std::logic_error& e) {
    throw py::value_error(e.what());
  } catch (const std::exception& e) {
    // pybind11 3.x 无 py::runtime_error，走 C API 抛 RuntimeError
    PyErr_SetString(PyExc_RuntimeError, e.what());
    throw py::error_already_set();
  }
}

}  // namespace

PYBIND11_MODULE(vrobot_slam_native, m) {
  m.doc() = "pybind11 binding for Google Cartographer (MapBuilder, 2D/3D SLAM)";

  py::class_<vrobot::slam::SubmapGridData>(m, "SubmapGridData")
      .def_readonly("origin_x", &vrobot::slam::SubmapGridData::origin_x)
      .def_readonly("origin_y", &vrobot::slam::SubmapGridData::origin_y)
      .def_readonly("resolution", &vrobot::slam::SubmapGridData::resolution)
      .def_readonly("width", &vrobot::slam::SubmapGridData::width)
      .def_readonly("height", &vrobot::slam::SubmapGridData::height)
      .def_readonly("finished", &vrobot::slam::SubmapGridData::finished)
      .def_readonly("pose_x", &vrobot::slam::SubmapGridData::pose_x)
      .def_readonly("pose_y", &vrobot::slam::SubmapGridData::pose_y)
      .def_readonly("pose_yaw", &vrobot::slam::SubmapGridData::pose_yaw)
      .def_property_readonly(
          "prob",
          [](const vrobot::slam::SubmapGridData& g) {
            py::array_t<float> arr(
                {static_cast<py::ssize_t>(g.height),
                 static_cast<py::ssize_t>(g.width)});
            std::memcpy(arr.mutable_data(), g.prob.data(),
                        g.prob.size() * sizeof(float));
            return arr;
          })
      .def_property_readonly(
          "known",
          [](const vrobot::slam::SubmapGridData& g) {
            py::array_t<unsigned char> arr(
                {static_cast<py::ssize_t>(g.height),
                 static_cast<py::ssize_t>(g.width)});
            std::memcpy(arr.mutable_data(), g.known.data(),
                        g.known.size() * sizeof(unsigned char));
            return arr;
          });

  py::class_<vrobot::slam::Mapper>(m, "Mapper")
      .def(py::init<const std::string&, const std::vector<std::string>&>(),
           py::arg("lua_code"), py::arg("search_paths"))
      .def(
          "start_trajectory",
          [](vrobot::slam::Mapper& self, const std::string& range_id,
             py::object imu_id, py::object odom_id) {
            return guarded([&] {
              std::optional<std::string> imu, odom;
              if (!imu_id.is_none()) imu = imu_id.cast<std::string>();
              if (!odom_id.is_none()) odom = odom_id.cast<std::string>();

              // 后台线程回调：拿 GIL 后通过线程安全接口更新状态
              vrobot::slam::TrajectoryBuilderInterface::LocalSlamResultCallback
                  cb = [&self](int, cartographer::common::Time time,
                               vrobot::slam::Rigid3d local_pose,
                               cartographer::sensor::RangeData,
                               std::unique_ptr<
                                   const vrobot::slam::TrajectoryBuilderInterface::
                                       InsertionResult>) {
                py::gil_scoped_acquire acquire;
                self.notify_local_slam_result(time, std::move(local_pose));
              };
              return self.start_trajectory(range_id, imu, odom, std::move(cb));
            });
          },
          py::arg("range_id") = "lidar", py::arg("imu_id") = py::none(),
          py::arg("odom_id") = py::none(),
          "开始一条轨迹。返回 trajectory_id。传感器数据必须按时间递增送入。")
      .def(
          "add_scan",
          [](vrobot::slam::Mapper& self, double timestamp,
             py::array_t<double, py::array::c_style | py::array::forcecast>
                 points,
             std::vector<double> origin) {
            return guarded([&] {
              if (points.ndim() != 2 ||
                  (points.shape(1) != 3 && points.shape(1) != 4)) {
                throw py::value_error(
                    "points must be shaped (N,3) [x,y,z] or (N,4) "
                    "[x,y,z,rel_time]");
              }
              auto pts = points.unchecked<2>();
              std::vector<double> flat;
              flat.reserve(static_cast<size_t>(pts.shape(0)) *
                           static_cast<size_t>(pts.shape(1)));
              for (py::ssize_t i = 0; i < pts.shape(0); ++i)
                for (py::ssize_t j = 0; j < pts.shape(1); ++j)
                  flat.push_back(pts(i, j));
              self.add_scan(timestamp, flat, static_cast<int>(pts.shape(1)),
                            origin);
            });
          },
          py::arg("timestamp"), py::arg("points"),
          py::arg("origin") = std::vector<double>{0., 0., 0.},
          "送入一帧激光扫描。points: (N,3)[x,y,z] 或 (N,4)[x,y,z,rel_time]，"
          "传感器坐标系，米。")
      .def(
          "add_imu",
          [](vrobot::slam::Mapper& self, double timestamp,
             std::vector<double> accel, std::vector<double> gyro) {
            return guarded(
                [&] { self.add_imu(timestamp, accel, gyro); });
          },
          py::arg("timestamp"), py::arg("accel"), py::arg("gyro"),
          "送入 IMU 数据 (m/s^2, rad/s)。需在 start_trajectory 时启用。")
      .def(
          "add_odometry",
          [](vrobot::slam::Mapper& self, double timestamp, double x, double y,
             double yaw) {
            return guarded(
                [&] { self.add_odometry(timestamp, x, y, yaw); });
          },
          py::arg("timestamp"), py::arg("x"), py::arg("y"), py::arg("yaw"),
          "送入里程计位姿 (x, y, yaw)。需在 start_trajectory 时启用。")
      .def("finish_trajectory", &vrobot::slam::Mapper::finish_trajectory,
           "结束当前轨迹（此后不可再送数据）。")
      .def("run_final_optimization",
           &vrobot::slam::Mapper::run_final_optimization,
           "运行最终全局优化（阻塞）。")
      .def(
          "optimized_poses",
          [](vrobot::slam::Mapper& self) {
            return guarded([&] {
              const std::vector<double> rows = self.optimized_poses();
              const py::ssize_t n = static_cast<py::ssize_t>(rows.size() / 3);
              py::array_t<double> result({n, static_cast<py::ssize_t>(3)});
              auto view = result.mutable_unchecked<2>();
              for (py::ssize_t i = 0; i < n; ++i) {
                view(i, 0) = rows[i * 3];
                view(i, 1) = rows[i * 3 + 1];
                view(i, 2) = rows[i * 3 + 2];
              }
              return result;
            });
          },
          "返回全局优化后的轨迹节点位姿 (N,3) [x, y, yaw]。")
      .def(
          "submap_grids",
          [](vrobot::slam::Mapper& self) {
            return guarded([&] { return self.submap_grids(); });
          },
          "导出全部子图的概率栅格（Cartographer 建图结果）。")
      .def(
          "write_pbstream",
          [](vrobot::slam::Mapper& self, const std::string& path,
             bool include_unfinished) {
            return guarded(
                [&] { self.write_pbstream(path, include_unfinished); });
          },
          py::arg("path"), py::arg("include_unfinished") = false,
          "保存地图到 .pbstream 文件。")
      .def(
          "load_state",
          [](vrobot::slam::Mapper& self, const std::string& path,
             bool frozen) { return guarded([&] { self.load_state(path, frozen); }); },
          py::arg("path"), py::arg("frozen") = false,
          "从 .pbstream 加载地图状态。")
      .def_property_readonly("trajectory_id",
                             &vrobot::slam::Mapper::trajectory_id)
      .def_property_readonly(
          "num_nodes", [](vrobot::slam::Mapper& self) { return self.num_nodes(); },
          "位姿图中的轨迹节点数。")
      .def_property_readonly(
          "num_submaps",
          [](vrobot::slam::Mapper& self) { return self.num_submaps(); },
          "位姿图中的子图数。")
      .def_property_readonly(
          "latest_local_pose",
          [](vrobot::slam::Mapper& self) -> py::object {
            const auto p = self.latest_local_pose();
            if (!p) return py::none();
            return py::make_tuple((*p)[0], (*p)[1], (*p)[2]);
          },
          "最近一次 local SLAM 位姿 (x, y, yaw) 或 None。");
}
