#!/usr/bin/env python
"""构建原生 SLAM 扩展（pybind11 -> cartographer.lib）并部署到 vrobot.slam 包。

流程：
  1. 用 Visual Studio 生成器配置并编译 cpp/ 工程，产出 vrobot_slam_native.pyd
     （不依赖 vcvars64.bat / cmd，由 CMake 自行定位 MSVC 工具链）
  2. 把 pyd 复制到 src/vrobot/slam/
  3. 把 cartographer 构建目录里的第三方依赖 DLL 复制到 src/vrobot/slam/_bin/
     （cartographer.py 在导入扩展前会通过 os.add_dll_directory 注册该目录）

用法：
  uv run python scripts/build_native.py            # 完整构建 + 部署
  uv run python scripts/build_native.py --no-dlls  # 只重编译 pyd（DLL 已就位）
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

BRAIN = Path(__file__).resolve().parents[1]
CPP_DIR = BRAIN / "cpp"
BUILD_DIR = BRAIN / "build" / "native"
PACKAGE_DIR = BRAIN / "src" / "vrobot" / "slam"
BIN_DIR = PACKAGE_DIR / "_bin"

VS_GENERATOR = os.environ.get("VROBOT_VS_GENERATOR", "Visual Studio 18 2026")

# ---- 环境路径（可用环境变量覆盖） ------------------------------------------
CARTOGRAPHER_BUILD = Path(
    os.environ.get("VROBOT_CARTOGRAPHER_BUILD", "D:/b/cart"))
VCPKG_TOOLCHAIN = Path(
    os.environ.get("VROBOT_VCPKG_TOOLCHAIN",
                   "D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"))
VCPKG_PREFIX = Path(
    os.environ.get("VROBOT_VCPKG_PREFIX", "D:/dev/vcpkg/installed/x64-windows"))
CARTOGRAPHER_SRC = Path(
    os.environ.get("VROBOT_CARTOGRAPHER_SRC",
                   "D:/Projects/OpenSourceLibs/cartographer-2.0.0"))


def pybind11_cmake_dir() -> str:
    import pybind11

    return pybind11.get_cmake_dir()


def find_cmake() -> str:
    cmake = shutil.which("cmake")
    if cmake:
        return cmake
    for candidate in (r"C:\Program Files\CMake\bin\cmake.exe",):
        if Path(candidate).exists():
            return candidate
    raise FileNotFoundError("cmake not found in PATH")


def run(cmd: list[str]) -> None:
    print("[build_native] $", " ".join(cmd))
    subprocess.run(cmd, check=True, env=sanitized_env())


def sanitized_env() -> dict[str, str]:
    """清理大小写冲突的环境变量（如 HTTP_PROXY 与 http_proxy 并存）。

    MSBuild 用不区分大小写的字典加载环境变量，同名不同大小写的变量会导致
    MSB6001: System.ArgumentException "已添加项"。这里统一保留大写版本。
    """
    env = dict(os.environ)
    by_lower: dict[str, str] = {}
    for key in env:
        lower = key.lower()
        if lower in by_lower:
            drop = key if key.islower() else by_lower[lower]
            by_lower[lower] = key if not key.islower() else by_lower[lower]
            env.pop(drop, None)
        else:
            by_lower[lower] = key
    return env


def configure(cmake: str, pybind11_dir: str) -> None:
    run([
        cmake, "-S", str(CPP_DIR), "-B", str(BUILD_DIR),
        "-G", VS_GENERATOR, "-A", "x64",
        f"-DCMAKE_TOOLCHAIN_FILE={VCPKG_TOOLCHAIN}",
        f"-DCMAKE_PREFIX_PATH={VCPKG_PREFIX}",
        f"-DPython_EXECUTABLE={sys.executable}",
        f"-Dpybind11_DIR={pybind11_dir}",
        f"-DCARTOGRAPHER_SRC={CARTOGRAPHER_SRC}",
        f"-DCARTOGRAPHER_BUILD={CARTOGRAPHER_BUILD}",
    ])


def build(cmake: str) -> Path:
    run([cmake, "--build", str(BUILD_DIR), "--config", "Release"])
    # pybind11 3.x 默认带 ABI 标签: vrobot_slam_native.cp313-win_amd64.pyd
    for pattern in ("vrobot_slam_native*.pyd",):
        matches = sorted((BUILD_DIR / "Release").glob(pattern)) + sorted(
            BUILD_DIR.glob(pattern))
        if matches:
            return matches[0]
    raise FileNotFoundError("build finished but pyd not found")


def deploy_dlls() -> int:
    """复制依赖 DLL 到包内 _bin 目录（按 名称+大小 跳过已存在的）。"""
    BIN_DIR.mkdir(parents=True, exist_ok=True)
    copied = 0
    for dll in CARTOGRAPHER_BUILD.glob("*.dll"):
        dst = BIN_DIR / dll.name
        if dst.exists() and dst.stat().st_size == dll.stat().st_size:
            continue
        shutil.copy2(dll, dst)
        copied += 1
    return copied


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-dlls", action="store_true",
                        help="跳过 DLL 复制（仅重编译 pyd）")
    parser.add_argument("--clean", action="store_true",
                        help="删除构建目录后重新配置")
    args = parser.parse_args()

    if not (CARTOGRAPHER_BUILD / "cartographer.lib").exists():
        print(f"[build_native] 未找到 cartographer.lib: {CARTOGRAPHER_BUILD}",
              file=sys.stderr)
        return 1

    if args.clean and BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)

    cmake = find_cmake()
    pybind11_dir = pybind11_cmake_dir()
    print(f"[build_native] Python      : {sys.executable}")
    print(f"[build_native] cmake       : {cmake}")
    print(f"[build_native] generator   : {VS_GENERATOR}")
    print(f"[build_native] pybind11    : {pybind11_dir}")
    print(f"[build_native] cartographer: {CARTOGRAPHER_BUILD}")

    configure(cmake, pybind11_dir)
    pyd = build(cmake)

    PACKAGE_DIR.mkdir(parents=True, exist_ok=True)
    dst = PACKAGE_DIR / pyd.name
    shutil.copy2(pyd, dst)
    print(f"[build_native] 已部署 pyd: {dst} ({dst.stat().st_size / 1e6:.1f} MB)")

    if not args.no_dlls:
        n = deploy_dlls()
        print(f"[build_native] DLL 部署到 {BIN_DIR}: 新复制 {n} 个")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
