# FerroCAD (Free-Next-CAD)

RustとOpen CASCADE Technology (OCCT) による、拡張可能な次世代パラメトリック3D CADシステム。

## 特徴

- **Safe Rust コアエンジン**: UIと幾何・ドキュメントエンジンを完全分離し、OCCTのC++ APIは `cad-occt` クレート内部（`cxx` ブリッジ）に厳格にカプセル化。
- **明示的なFeature DAGと差分再計算**: パラメータ式（`width / 4` 等）とフィーチャー間の依存有向非巡回グラフ（DAG）により、変更箇所と影響範囲のみを再計算。
- **Commandパターンとバージョン付きJSON永続化**: OCCTの生ポインタに依存しないクリーンな設計データ保存と完全な Undo / Redo。
- **モダンな3D描画とGUI (`wgpu` + `egui`)**: エッジ強調シェーディング、ヘッドレスPNGレンダリング検証、3Dビューポート上のHUD直接編集、統合コマンドパレット。

## クレート構成

| クレート | 役割 |
|---|---|
| `crates/cad-occt` | OCCT (v7.8.1) との `cxx` FFIブリッジ、プリミティブ・ブーリアン・STEP入出力・B-Rep三角形メッシュ抽出 |
| `crates/cad-geometry` | 純粋Rustの数学・幾何型（`Length`, `Transform`, `BoundingBox`, `TriMesh`）および世代付きID管理の `ShapeStore` |
| `crates/cad-core` | `Document`, `Parameter`（数式評価対応）, `FeatureGraph` (DAG), 差分再計算エンジン, `Command` (Undo/Redo), JSON保存 |
| `crates/cad-render` | `wgpu` による3Dビューポート描画・カメラ制御・レイキャストピッキング・ヘッドレス画像出力 |
| `crates/cad-ui` | `egui` / `eframe` によるデスクトップCADアプリケーション |
| `crates/cad-cli` | ヘッドレス検証・STEP出力・レンダリング画像生成・実用デモ（穴あきプレート等）実行用CLI |

## セットアップとビルド手順（Windows / MSVC）

1. **OCCT (v7.8.1) ソースの取得と最小構成ビルド**
   ```powershell
   py references/fetch_references.py --target occt
   py scripts/build_occt.py
   ```
2. **ビルドとテストの実行**
   ```powershell
   cargo build --workspace
   cargo test --workspace
   ```
3. **CLIデモ（Milestone M1: 穴あきプレート生成・STEP出力・ヘッドレス3D描画）の実行**
   ```powershell
   cargo run -p cad-cli -- demo-plate --output-dir build/demo
   ```
4. **デスクトップGUIの起動**
   ```powershell
   cargo run -p cad-ui
   ```
