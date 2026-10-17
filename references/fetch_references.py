# -*- coding: utf-8 -*-
"""
リファレンスリポジトリ取得スクリプト
FreeCADおよびOCCT等の参照用ソースコードを浅いクローン（shallow clone）で取得・管理します。
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

# Windowsコンソールでの文字化け防止
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass


# 参照リポジトリの定義
REPOSITORIES = {
    "freecad": {
        "name": "FreeCAD",
        "url": "https://github.com/FreeCAD/FreeCAD.git",
        "branch": "main",
        "target_dir": "freecad",
        "description": "FreeCAD 公式ソースコード（App/Gui/Modアーキテクチャ参照用）"
    },
    "occt": {
        "name": "Open CASCADE Technology (OCCT)",
        "url": "https://github.com/Open-CASCADE-Technology/OCCT.git",
        "branch": "master",
        "target_dir": "occt",
        "description": "OCCT 公式ソースコード（B-Repカーネル・幾何演算・FFI参照用）"
    }
}


def run_cmd(cmd, cwd=None):
    """コマンドを実行し、標準出力・標準エラーを表示する"""
    print(f"実行コマンド: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, text=True)
    if result.returncode != 0:
        print(f"エラー: コマンドが終了コード {result.returncode} で失敗しました。", file=sys.stderr)
        return False
    return True


def clone_or_update(repo_info, repos_dir):
    """リポジトリを浅いクローンで取得、または最新状態に更新する"""
    target_path = repos_dir / repo_info["target_dir"]
    print("\n" + "=" * 60)
    print(f"対象: {repo_info['name']}")
    print(f"説明: {repo_info['description']}")
    print(f"保存先: {target_path}")
    print("=" * 60)

    if (target_path / ".git").exists():
        print(f"既存のリポジトリが検出されました。更新を確認します: {target_path}")
        return run_cmd(["git", "pull", "--depth", "1"], cwd=target_path)
    else:
        print(f"浅いクローン（depth=1）を開始します: {repo_info['url']}")
        cmd = [
            "git", "clone",
            "--depth", "1",
            "--branch", repo_info["branch"],
            "--single-branch",
            repo_info["url"],
            str(target_path)
        ]
        return run_cmd(cmd)


def main():
    parser = argparse.ArgumentParser(
        description="FreeCADおよびOCCTのリファレンスソースコードを取得・管理するツール"
    )
    parser.add_argument(
        "--target",
        choices=["freecad", "occt", "all"],
        default="all",
        help="取得する対象（freecad, occt, または all）"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="実際のクローンを行わず、対象と保存先のみを表示する"
    )

    args = parser.parse_args()

    # スクリプト自身の位置から references/repos ディレクトリを特定
    script_dir = Path(__file__).resolve().parent
    repos_dir = script_dir / "repos"

    if not repos_dir.exists():
        repos_dir.mkdir(parents=True, exist_ok=True)
        print(f"リポジトリ保存ディレクトリを作成しました: {repos_dir}")

    targets = list(REPOSITORIES.keys()) if args.target == "all" else [args.target]

    if args.dry_run:
        print("【ドライラン（模擬実行）モード】")
        for key in targets:
            info = REPOSITORIES[key]
            print(f"- {info['name']} -> {repos_dir / info['target_dir']} ({info['url']})")
        return

    success_count = 0
    for key in targets:
        info = REPOSITORIES[key]
        if clone_or_update(info, repos_dir):
            success_count += 1

    print("\n処理完了:")
    print(f"成功: {success_count}/{len(targets)}")


if __name__ == "__main__":
    main()
