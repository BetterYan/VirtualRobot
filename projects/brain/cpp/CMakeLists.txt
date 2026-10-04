cmake_minimum_required(VERSION 3.24)
project(vrobot_slam_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# 输入路径（默认指向本机 cartographer 构建环境，可用 -D 覆盖）
# ---------------------------------------------------------------------------
set(CARTOGRAPHER_SRC "D:/Projects/OpenSourceLibs/cartographer-2.0.0"
    CACHE PATH "Cartographer 源码目录（头文件 + lua 配置）")
set(CARTOGRAPHER_BUILD "D:/b/cart"
    CACHE PATH "Cartographer 构建目录（cartographer.lib + 生成的 *.pb.h）")

# ---------------------------------------------------------------------------
# 依赖查找：pybind11 来自 venv，其余全部来自 vcpkg x64-windows（与 cartographer.lib
# 的构建环境一致，保证 ABI 兼容）
# ---------------------------------------------------------------------------
find_package(pybind11 CONFIG REQUIRED)
find_package(absl CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)

find_library(CARTOGRAPHER_LIB NAMES cartographer
             PATHS "${CARTOGRAPHER_BUILD}" NO_DEFAULT_PATH REQUIRED)
find_library(LUA_LIB NAMES lua REQUIRED)
find_library(CAIRO_LIB NAMES cairo REQUIRED)

# ---------------------------------------------------------------------------
# 扩展模块本体
# ---------------------------------------------------------------------------
pybind11_add_module(vrobot_slam_native
  src/slam_module.cpp
  src/mapper.cpp)

# 与 cartographer.lib 的构建保持一致（见 cartographer CMakeLists.txt 的 MSVC 分支）：
#  - /FI 强制包含两个兼容头（Abseil >= LTS 2025 移除了无前缀线程注解宏，
#    且有 ceres 版本兼容层）——cartographer.lib 内部按此约定编译，头文件
#    中直接使用 LOCKS_EXCLUDED/GUARDED_BY 等宏，我们的 TU 也必须看到它们
#  - /utf-8：源码含中文注释/字符串，避免 MSVC 按 GBK 误读
target_compile_options(vrobot_slam_native PRIVATE
  "/FI${CARTOGRAPHER_SRC}/cartographer/common/thread_annotation_compat.h"
  "/FI${CARTOGRAPHER_SRC}/cartographer/common/ceres_compat.h"
  /utf-8)

target_include_directories(vrobot_slam_native SYSTEM PRIVATE
  "${CARTOGRAPHER_SRC}"
  "${CARTOGRAPHER_BUILD}"    # 生成的 cartographer/*/*.pb.h
)

target_compile_definitions(vrobot_slam_native PRIVATE
  CARTOGRAPHER_CONFIGURATION_FILES="${CARTOGRAPHER_SRC}/configuration_files"
  GLOG_NO_ABBREVIATED_SEVERITIES
  _ENABLE_EXTENDED_ALIGNED_STORAGE
  _USE_MATH_DEFINES
  NOMINMAX
  WIN32_LEAN_AND_MEAN
)

target_link_libraries(vrobot_slam_native PRIVATE
  ${CARTOGRAPHER_LIB}
  ${LUA_LIB}
  ${CAIRO_LIB}
  Ceres::ceres
  Eigen3::Eigen
  glog::glog
  gflags::gflags
  protobuf::libprotobuf
  Boost::iostreams
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility
)

# ---------------------------------------------------------------------------
# 控制台冒烟测试（脱离 Python 直接验证原生层）
# ---------------------------------------------------------------------------
add_executable(map_builder_smoke src/map_builder_smoke.cpp src/mapper.cpp)
target_compile_options(map_builder_smoke PRIVATE
  "/FI${CARTOGRAPHER_SRC}/cartographer/common/thread_annotation_compat.h"
  "/FI${CARTOGRAPHER_SRC}/cartographer/common/ceres_compat.h"
  /utf-8)
target_include_directories(map_builder_smoke SYSTEM PRIVATE
  "${CARTOGRAPHER_SRC}"
  "${CARTOGRAPHER_BUILD}")
target_compile_definitions(map_builder_smoke PRIVATE
  CARTOGRAPHER_CONFIGURATION_FILES="${CARTOGRAPHER_SRC}/configuration_files"
  GLOG_NO_ABBREVIATED_SEVERITIES
  _ENABLE_EXTENDED_ALIGNED_STORAGE
  _USE_MATH_DEFINES
  NOMINMAX
  WIN32_LEAN_AND_MEAN)
target_link_libraries(map_builder_smoke PRIVATE
  ${CARTOGRAPHER_LIB}
  ${LUA_LIB}
  ${CAIRO_LIB}
  Ceres::ceres
  Eigen3::Eigen
  glog::glog
  gflags::gflags
  protobuf::libprotobuf
  Boost::iostreams
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility)
