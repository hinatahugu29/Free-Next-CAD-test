# Windows 開発環境構築・再現手順

## 動作確認済み環境

- **OS**: Windows 10 / 11 (x64)
- **Rust**: `rustc 1.98.1` / `cargo 1.98.1` (Edition 2024)
- **CMake**: `4.3.4` 以上（3.24以上互換）
- **C++ コンパイラ**: Visual Studio 2022 (MSVC v143 x64 ツールチェーン)
- **Python**: `py` ランチャー (Python 3.10+)
- **OCCT**: `V7_8_1`

## ゼロからのビルド再現手順

```powershell
# 1. OCCT公式ソース(V7_8_1)を浅いクローンで取得
py references/fetch_references.py --target occt

# 2. OCCT最小構成（Foundation, ModelingData, ModelingAlgorithms, DataExchange）をビルド
py scripts/build_occt.py

# 3. Rust Workspace全体のビルドとテスト
cargo test --workspace
```

> **補足**: 環境変数 `OCCT_ROOT` で既存のOCCTインストール先を指定することも可能です。未指定時は `build/occt-install` を自動検出します。
