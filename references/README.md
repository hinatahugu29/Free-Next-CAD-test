# リファレンスコード＆アーキテクチャ資料

本ディレクトリ（`references/`）は、**FerroCAD (Free-Next-CAD)** の設計・実装にあたり参照する **FreeCAD** および **Open CASCADE Technology (OCCT)** などの公式ソースコード、アーキテクチャ解析、関連リファレンスを整理・格納するための領域です。

---

## 1. 参照対象リポジトリの概要

| プロジェクト | リポジトリURL | 主な役割・参照目的 |
|---|---|---|
| **FreeCAD** | `https://github.com/FreeCAD/FreeCAD.git` | ・App/Guiの分離アーキテクチャ<br>・トポロジカルネーミング問題 (TNP) の追跡アルゴリズム<br>・Part / PartDesign / Sketcher のモデリングパイプライン |
| **Open CASCADE (OCCT)** | `https://github.com/Open-Cascade-SAS/OCCT.git` (`V7_8_1`) | ・幾何モデリングカーネル（B-Rep、ブーリアン演算、STEP入出力）<br>・C++ FFI境界設計に必要なヘッダー・クラス構造 |
| **opencascade.js / opencascade-rs** | `https://github.com/openscad/` など | ・Rustや他言語バインディングの先行設計例 |

> **注意:** FreeCADやOCCTのリポジトリは履歴を含めると数GB〜十数GBのサイズになります。そのため、`references/repos/` は `.gitignore` に指定されており、**浅いクローン（`--depth 1`）** で取得することを推奨します。

---

## 2. 自動取得スクリプト (`fetch_references.py`) の使い方

本リポジトリでは、Pythonスクリプトを用いて安全かつ軽量にリファレンスコードを取得できます。

```powershell
# ヘルプ表示
py references/fetch_references.py --help

# FreeCAD のみ取得（最新安定ブランチを shallow clone）
py references/fetch_references.py --target freecad

# OCCT のみ取得（shallow clone）
py references/fetch_references.py --target occt

# 全て取得
py references/fetch_references.py --target all
```

取得先は `references/repos/` 配下となります。

---

## 3. FreeCAD アーキテクチャの重要参照ポイント

FreeCADのソースコード（`references/repos/freecad/`）を読む際、特に注目すべき重要モジュールです。

```text
FreeCAD/src/
├── App/                      # コアデータモデル（GUI非依存）
│   ├── Document.cpp/h        # ドキュメント管理、オブジェクトのツリー構造
│   ├── DocumentObject.cpp/h  # 全フィーチャーの基底クラス
│   ├── Property*.cpp/h       # パラメータ・プロパティシステム
│   └── ComplexGeoData.cpp/h  # 幾何データの抽象基底クラス
├── Gui/                      # ユーザーインターフェース基盤
│   ├── View3DInventor*.cpp/h # 3Dビューポートとシーングラフ
│   ├── Selection*.cpp/h      # 形状選択機構（Face/Edge/Vertexピッキング）
│   └── Command*.cpp/h        # コマンドディスパッチとUndo/Redo
└── Mod/                      # ワークベンチ（機能モジュール）
    ├── Part/                 # OCCTブリッジ（TopoDS_Shapeの基本操作）
    │   └── App/TopoShape.cpp # FreeCADにおけるOCCT Shapeのラッパー
    ├── PartDesign/           # フィーチャーベース・モデリング（Pad, Pocket, Fillet等）
    │   └── App/Body.cpp      # 単一ソリッドを構築するBodyコンテナ
    └── Sketcher/             # 幾何拘束スケッチャー
        └── App/planGCS/      # 2D幾何拘束ソルバーエンジン
```

---

## 4. OCCT (Open CASCADE Technology) の重要参照パッケージ

OCCTのソースコード（`references/repos/occt/`）におけるコアパッケージ構造です。

```text
OCCT/src/
├── Standard/                 # 基本型定義、Handle（侵入型スマートポインタ）、例外
├── gp/                       # 幾何プリミティブ数学（gp_Pnt, gp_Vec, gp_Trsf, gp_Ax2）
├── Geom/ / Geom2d/           # 解析幾何・パラメトリック曲線/曲面（B-Spline, Bezier）
├── TopoDS/                   # トポロジーデータ構造（B-Rep）
│   ├── TopoDS_Shape.hxx      # 全トポロジー形状の基底
│   ├── TopoDS_Solid.hxx      # ソリッド
│   ├── TopoDS_Shell.hxx      # シェル（面の集合）
│   ├── TopoDS_Face.hxx       # 面
│   ├── TopoDS_Wire.hxx       # ワイヤー（閉じた/開いた辺のループ）
│   └── TopoDS_Edge.hxx       # 辺
├── BRepPrimAPI/              # プリミティブ作成（MakeBox, MakeCylinder, MakeSphere）
├── BRepAlgoAPI/              # ブーリアン演算（Fuse, Cut, Common, Section）
├── BRepFilletAPI/            # フィレット・面取り（MakeFillet, MakeChamfer）
├── BRepMesh/                 # 描画・メッシュ化（B-Repからポリゴン三角メッシュ生成）
└── STEPCAFControl/           # STEPファイル入出力（色・アセンブリ情報含む）
```

---

## 5. UI/UX 改善に向けた課題と再設計アプローチ

FreeCADが抱えている課題と、Rust + 新アーキテクチャによる改善アプローチの対比です。

| 観点 | 従来のFreeCAD | 次世代CAD（FerroCAD）の設計アプローチ |
|---|---|---|
| **UIフレームワーク** | Qt Widgets + Coin3D（やや重厚・クラシック） | **egui + wgpu**（軽量、高リフレッシュレート、最新GPUパイプライン） |
| **設計履歴管理** | ドキュメントツリー内の線形または暗黙の依存 | **明示的な依存有向非巡回グラフ（DAG）** による変更伝播と部分再計算 |
| **トポロジカルネーミング** | 履歴変更時に面・辺のIDがズレて後続フィーチャーが壊れやすい | **幾何シグネチャ＋要素追跡（Element Mapping）** による安定した形状参照 |
| **操作性 (UX)** | 多数のワークベンチ切り替え、深いモーダルダイアログ | **統合コマンドパレット、文脈に応じたHUD（Heads-Up Display）、リアルタイム直接編集** |
| **言語・安全性** | C++ と Python の混在（ポインタ管理とクラッシュリスク） | **コアエンジンを Safe Rust で構築**、OCCTとの境界を cxx で厳格にカプセル化 |
