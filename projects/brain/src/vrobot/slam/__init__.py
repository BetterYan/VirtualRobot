"""vrobot.slam - 2D SLAM 子系统。

原生扩展（Cartographer 绑定）通过惰性导入提供，
避免在未部署 pyd 的环境下破坏包导入。
"""

import os
from pathlib import Path

# 原生扩展依赖的第三方 DLL 位于包内 _bin/ 目录。
# 必须在导入任何 .pyd 之前注册搜索路径（Windows: LOAD_LIBRARY_SEARCH_USER_DIRS）。
_BIN_DIR = Path(__file__).resolve().parent / "_bin"
if _BIN_DIR.is_dir():
    os.add_dll_directory(str(_BIN_DIR))


def __getattr__(name):
    if name == "CartographerSLAM":
        from vrobot.slam.cartographer import CartographerSLAM

        return CartographerSLAM
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
