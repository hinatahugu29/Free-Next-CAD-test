# FerroCAD アーキテクチャ設計書 (ARCHITECTURE.md)

## 1. レイヤー構造と依存関係

FerroCADはUIとモデリングエンジンを厳格に分離し、一方向の依存グラフを維持する。

```text
          ┌────────────┐      ┌─────────────┐
          │   cad-ui   │      │   cad-cli   │
          │(egui/eframe│      │ (Headless)  │
          └─────┬──────┘      └──────┬──────┘
                │                    │
         ┌──────┴──────┐             │
         ▼             ▼             ▼
  ┌────────────┐ ┌─────────────────────────┐
  │ cad-render │ │        cad-core         │
  │   (wgpu)   │ │(Document/DAG/Param/Cmd) │
  └─────┬──────┘ └────────────┬────────────┘
        │                     │
        └──────────┬──────────┘
                   ▼
          ┌────────────────┐
          │  cad-geometry  │
          │(ShapeStore/Mesh│
          └────────┬───────┘
                   ▼
          ┌────────────────┐
          │    cad-occt    │
          │ (cxx + OCCT)   │
          └────────────────┘
```

## 2. コア設計原則

1. **OCCTのカプセル化 (`cad-occt`)**
   - `TopoDS_Shape` 等のOCCTクラスはC++側の不透明ハンドル（`OcctShape`）として保持し、上位クレートには一切露出しない。
   - すべてのC++関数呼び出しは `try { ... } catch (const Standard_Failure& e)` および `catch (const std::exception& e)` で保護し、Rustの `Result<T, OcctError>` に変換する。
2. **ShapeStoreと世代付きID (`cad-geometry`)**
   - `Document` は幾何実体を持たず、`ShapeId`（インデックス＋世代番号）のみを保持する。
   - B-Repから抽出した三角形メッシュ（`TriMesh`）には、各三角形が元のどの `FaceIndex` に属するか、および境界エッジ線分（`EdgeSegment`）の情報を保持し、3D描画とSubshapeピッキングの両方に利用する。
3. **Feature DAGと差分再計算 (`cad-core`)**
   - パラメータ変更やフィーチャー編集時は、有向非巡回グラフ（DAG）上で下流ノードのみを `Dirty` にマークし、トポロジカルソート順に必要なノードだけを再評価する。
   - 評価失敗時は `Failed`、その下流ノードは `Blocked` として記録し、最後に成功した形状キャッシュと論理状態を明確に区別する。
4. **ヘッドレス検証可能な3Dレンダリング (`cad-render`)**
   - `wgpu` パイプラインはウィンドウサーフェス描画だけでなく、オフスクリーンテクスチャへのヘッドレスレンダリング（PNG出力）をサポートし、CIやCLIから視覚的出力を自動検証できるようにする。
