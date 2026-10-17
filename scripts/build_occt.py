# -*- coding: utf-8 -*-
"""
OCCT (Open CASCADE Technology v7.8.1) 最小構成ビルド自動化スクリプト
Windows (MSVC + CMake) 環境において、サードパーティ依存（FreeType, Tcl/Tk, OpenGL, VTK等）を
すべて無効化し、FerroCADに必要な幾何カーネル・メッシュ化・STEP入出力ツールキットのみをビルドします。
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

# FerroCADで必要となる最小限のOCCTツールキット群
REQUIRED_TOOLKITS = [
    "TKernel",
    "TKMath",
    "TKG2d",
    "TKG3d",
    "TKGeomBase",
    "TKBRep",
    "TKGeomAlgo",
    "TKTopAlgo",
    "TKPrim",
    "TKBO",
    "TKBool",
    "TKHLR",
    "TKFillet",
    "TKOffset",
    "TKShHealing",
    "TKMesh",
    "TKBinXCAF",
    "TKXSBase",
    "TKDE",
    "TKDESTEP",
]


def run_cmd(cmd, cwd=None):
    print(f"[build_occt] 実行: {' '.join(str(c) for c in cmd)}")
    res = subprocess.run(cmd, cwd=cwd)
    if res.returncode != 0:
        print(f"[build_occt] エラー: 終了コード {res.returncode}", file=sys.stderr)
        sys.exit(res.returncode)


def main():
    parser = argparse.ArgumentParser(description="OCCT 最小構成CMakeビルドスクリプト")
    parser.add_argument(
        "--configure-only",
        action="store_true",
        help="CMakeのConfigureのみを実行して確認する",
    )
    parser.add_argument(
        "--skip-configure",
        action="store_true",
        help="既にCMakeCache.txtが存在する場合にConfigureをスキップしてビルドのみ実行する",
    )
    parser.add_argument(
        "--lib-type",
        choices=["Static", "Shared"],
        default="Static",
        help="ビルドするライブラリ種別（デフォルト: Static）",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=os.cpu_count() or 8,
        help="並列ビルドジョブ数",
    )
    args = parser.parse_args()

    root_dir = Path(__file__).resolve().parent.parent
    occt_src = root_dir / "references" / "repos" / "occt"
    build_dir = root_dir / "build" / "occt-build"
    install_dir = root_dir / "build" / "occt-install"

    if not (occt_src / "CMakeLists.txt").exists():
        print(
            f"[build_occt] OCCTソースが見つかりません: {occt_src}\n"
            "先に `py references/fetch_references.py --target occt` を実行してください。",
            file=sys.stderr,
        )
        sys.exit(1)

    build_dir.mkdir(parents=True, exist_ok=True)
    install_dir.mkdir(parents=True, exist_ok=True)

    toolkits_str = ";".join(REQUIRED_TOOLKITS)

    cmake_configure_cmd = [
        "cmake",
        "-S",
        str(occt_src),
        "-B",
        str(build_dir),
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        f"-DCMAKE_INSTALL_PREFIX={install_dir.as_posix()}",
        f"-DINSTALL_DIR={install_dir.as_posix()}",
        "-DINSTALL_DIR_LAYOUT=Unix",
        f"-DBUILD_LIBRARY_TYPE={args.lib_type}",
        "-DBUILD_CPP_STANDARD=C++17",
        # ReleaseビルドでもStandard_Failure例外を送出させ、Rust側で安全にcatchできるようにする
        "-DBUILD_RELEASE_DISABLE_EXCEPTIONS=OFF",
        # 全モジュールの一括ビルドをOFFにし、必要なツールキットとその依存のみを厳選ビルド
        "-DBUILD_MODULE_FoundationClasses=OFF",
        "-DBUILD_MODULE_ModelingData=OFF",
        "-DBUILD_MODULE_ModelingAlgorithms=OFF",
        "-DBUILD_MODULE_Visualization=OFF",
        "-DBUILD_MODULE_ApplicationFramework=OFF",
        "-DBUILD_MODULE_DataExchange=OFF",
        "-DBUILD_MODULE_Draw=OFF",
        f"-DBUILD_ADDITIONAL_TOOLKITS={toolkits_str}",
        # サードパーティ依存をすべてOFFにする
        "-DUSE_TK=OFF",
        "-DUSE_FREETYPE=OFF",
        "-DUSE_FREEIMAGE=OFF",
        "-DUSE_FFMPEG=OFF",
        "-DUSE_OPENVR=OFF",
        "-DUSE_RAPIDJSON=OFF",
        "-DUSE_DRACO=OFF",
        "-DUSE_TBB=OFF",
        "-DUSE_EIGEN=OFF",
        "-DUSE_OPENGL=OFF",
        "-DUSE_GLES2=OFF",
        "-DUSE_D3D=OFF",
        "-DUSE_VTK=OFF",
        "-DBUILD_DOC_Overview=OFF",
        "-DINSTALL_SAMPLES=OFF",
        "-DINSTALL_TEST_CASES=OFF",
    ]

    if not (args.skip_configure and (build_dir / "CMakeCache.txt").exists()):
        print("[build_occt] CMake Configure を開始します...")
        run_cmd(cmake_configure_cmd)

    if args.configure_only:
        print("[build_occt] Configure が正常に完了しました。")
        return

    print(f"[build_occt] CMake Build & Install (Release, jobs={args.jobs}) を開始します...")
    cmake_build_cmd = [
        "cmake",
        "--build",
        str(build_dir),
        "--config",
        "Release",
        "--target",
        "install",
        "--parallel",
        str(args.jobs),
    ]
    run_cmd(cmake_build_cmd)
    print(f"[build_occt] OCCTのビルドとインストールが完了しました: {install_dir}")


if __name__ == "__main__":
    main()
