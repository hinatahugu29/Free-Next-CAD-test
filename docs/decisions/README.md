# ADR Summary — FerroCAD Architecture Decision Records

## ADR-001: Rustを主要言語とする
- **決定**: システム全体の主要開発言語としてRust（Edition 2024）を採用する。
- **理由**: メモリ安全性、型システムによる状態管理（型付きID、エラー処理）、長期保守性において従来のC++/Python混在アーキテクチャより優位であるため。

## ADR-002: OCCTをCXX Bridgeから最小構成ビルドで利用する
- **決定**: 幾何カーネルにはOpen CASCADE Technology (OCCT v7.8.1) を採用し、`cxx` クレートを用いた薄いC++ラッパー経由で接続する。Windows環境等でのサードパーティ依存（FreeType, Tcl/Tk, VTK）を排除するため、`BUILD_MODULE_Draw=OFF`, `BUILD_MODULE_Visualization=OFF` の最小構成でビルドする。
- **理由**: ゼロからのB-Repカーネル実装は非現実的であり、実績あるOCCTを利用しつつ、FFI境界でC++例外を完全に捕捉してSafe Rust APIへ変換することで安全性を担保できるため。

## ADR-003: DocumentとGeometryを分離する
- **決定**: `cad-core` の `Document` はOCCTオブジェクトを直接保持せず、`cad-geometry` の `ShapeStore` が発行する `ShapeId`（世代付きインデックス）によってのみ形状を参照する。
- **理由**: シリアライズ（JSON保存）、Undo/Redo、およびヘッドレス再計算をOCCTのメモリ寿命から切り離すため。

## ADR-004: Feature DAGによる再計算を採用する
- **決定**: 設計履歴を単なる線形リストではなく、明示的な有向非巡回グラフ（DAG）として表現し、パラメータ式やフィーチャー変更時にDirty伝播とトポロジカルソートによる差分再計算を行う。
- **理由**: 不要な再計算を省き、エラー発生時に `Failed` と `Blocked`（上流エラーによる待機）を明確に区別するため。

## ADR-005: Shape Storeによる形状管理とB-Repメッシュ対応保持
- **決定**: `ShapeStore` で形状ライフサイクルを一元管理し、B-Repから生成した表示用 `TriMesh` には元の `FaceIndex` および `EdgeIndex` の対応情報を保持する。
- **理由**: 3Dビューポート上でのピッキング（面・辺の選択）と元のトポロジー要素を正確に対応付けるため。

## ADR-006: 初期描画バックエンドとしてwgpuを採用し、ヘッドレス検証と縦割りスライスに対応する
- **決定**: 3D描画に `wgpu`、GUIに `egui` / `eframe` を採用し、ウィンドウ描画と同一シェーダーでオフスクリーンPNG出力（ヘッドレスレンダリング）を可能にする。
- **理由**: マルチプラットフォームの最新GPU APIに対応しつつ、CIや自動テストでも3D描画結果を画像として検証できるようにするため。
