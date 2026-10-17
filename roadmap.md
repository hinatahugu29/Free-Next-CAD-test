---
paper: A4
orientation: portrait
margin: 20mm
font-size: 10.5pt
line-height: 1.5
font: gothic
---
# FerroCAD — 開発ロードマップ

**バージョン:** Draft 0.2（実践再構築版）  
**目的:** RustとOCCTによる、拡張可能な次世代パラメトリック3D CADの開発  
**開発方式:** 小規模・段階的・縦割りスライス（Vertical Slice）・AI支援型  
**初期対象OS:** Windows（MSVC + CMake + Rust）。将来的にLinux、macOSへ対応する。

---

## 1. プロジェクトのビジョン

FerroCADは、Rustを中心に開発するオープンアーキテクチャの3D CADシステムである。

FreeCADのような履歴ベースのパラメトリックモデリングを基本機能として持ちながら、設計履歴の管理、幾何形状の参照、設計意図の表現、そしてモダンなUI/UXを再設計する。

長期的には以下を目指す。

1. 設計操作を依存関係グラフ（DAG）として表現する
2. 変更が影響する部分だけを再計算する
3. GUI、CLI、APIから共通のモデリングエンジンを操作できる
4. 幾何形状と設計意図を別々の情報として保持する
5. 安定した形状参照と設計変更への追従（トポロジカルネーミング問題の克服）を実現する
6. AIが安全に設計モデルを操作できる構造を持つ
7. 将来的に製造技術やアセンブリへ拡張できる

### 1.1 開発原則

- Rustを主要言語とする
- OCCTとの接続に必要なC++コードは最小限とする
- UIとCADエンジンを分離する
- エンジンの公開APIは原則としてsafe Rustにする
- 不必要なグローバル状態を持たない
- 設計データに安定したIDを与える
- ファイル保存時にポインタや一時的な形状ハンドルへ依存しない
- 変更処理は原則として検証可能なコマンドで表現する
- 動くコードと自動テストを優先する
- 将来構想のために初期実装を過度に複雑化しない

### 1.2 実践的開発戦略（再構築方針）

机上の設計や水平レイヤーのみの長期実装による手戻りを防ぐため、以下の実践方針を採る。

1. **縦割りスライス（Vertical Slice）方式の採用**
   - Phase 1（幾何層）やPhase 2（Document層）を100%完成させるまでGUI（Phase 3）を後回しにするのではなく、**まずは「直方体（Box）1個」で `OCCT → Document/DAG → wgpu 3D描画 → egui UI操作` を一気通貫させる最小スライス（Milestone M0）**を早期に立ち上げる。
   - これにより、B-Repのメッシュ化やFace/Edgeピッキングに必要なID設計の整合性を初期段階で検証し、手戻りを防止する。
2. **外部依存を排除したOCCT最小構成ビルド**
   - Windows環境でボトルネックになりやすいOCCTのサードパーティ依存（FreeType, Tcl/Tk, VTK等）を排除するため、OCCTの可視化・Drawモジュールを無効化（`BUILD_MODULE_Draw=OFF`, `BUILD_MODULE_Visualization=OFF`）し、**幾何カーネル・メッシュ化・STEP入出力のみをCMake + MSVCでクリーンビルド**する。
3. **リファレンス駆動の設計（`references/` の活用）**
   - `py references/fetch_references.py` により FreeCAD および OCCT の公式ソースコードを浅いクローン（shallow clone）で手元に保持し、FreeCADの `App/Gui` 分離や要素追跡（Element Mapping）、OCCTのAPI利用パターンを直接参照しながら実装を進める。

---

## 2. 技術スタックとクレート責務

| 分野 | 初期採用技術 |
|---|---|
| 主要言語 | Rust（Edition 2024） |
| 幾何カーネル | Open CASCADE Technology（最小構成ヘッドレスビルド） |
| FFI | cxx + 薄いC++ラッパー（C++例外の完全捕捉） |
| ビルド | Cargo Workspace + CMake（MSVCツールチェーン） |
| GUI | egui / eframe |
| グラフィックス | wgpu |
| ウィンドウ | eframe経由、必要に応じてwinit |
| 数学 | glamなどのRust数学ライブラリ |
| ID管理 | 型付きID、UUIDまたは世代付きインデックス（Arena/SlotMap系） |
| データ保存 | serdeを中心としたバージョン付き形式 |
| テスト | cargo test + C++境界テスト |
| CI | GitHub Actions |
| コード品質 | rustfmt / clippy |

依存ライブラリのバージョンは初期技術検証後に固定する。

### 2.1 クレート構成と責務境界

| クレート名 | 主な責務 |
|---|---|
| `cad-occt` | `cxx` ブリッジと薄いC++ラッパー。`TopoDS_Shape` のカプセル化、C++例外キャッチと `Result` 変換、プリミティブ/ブーリアン/STEP入出力、および `BRepMesh_IncrementalMesh` による三角形メッシュ（Face対応情報付き）の抽出を担当。OCCTの生型は外部へ露出しない。 |
| `cad-geometry` | OCCT非依存の純粋Rust幾何・数学層。座標・ベクトル・変換（`Transform`）、`BoundingBox`、単位型、形状の所有・破棄を一元管理する `ShapeStore`（`ShapeId` / `SubshapeId`）、および表示・ピッキング用の `TriMesh` データ構造を提供する。 |
| `cad-core` | パラメトリックCADの中核エンジン。`Document`、`Parameter`、`FeatureGraph`（DAG）、Dirty伝播と差分再計算、`Command` パターンによる Undo/Redo、`serde` によるファイル保存・読込を担当。 |
| `cad-render` | `wgpu` による3Dビューポート描画エンジン。Orbit/Pan/Zoomカメラ制御、CAD向けシェーディング（面＋エッジ強調表示）、および画面上の位置から `FeatureId` / `SubshapeId` を特定するピッキング機構を担当。 |
| `cad-ui` | `egui` / `eframe` によるデスクトップGUI。Feature Tree、Property Panel、3D Viewport上のHUD（Heads-Up Display）直接寸法編集、統合コマンドパレットを提供する。 |
| `cad-cli` | ヘッドレス実行、バッチ変換（設計JSONからのSTEP出力など）、および自動テスト・CI検証用のCLIフロントエンド。 |

### 2.2 バージョン管理方針

- Rustツールチェーンを固定する（`rust-toolchain.toml`）
- OCCTの対象バージョンを固定する
- `Cargo.lock` をGit管理対象に含める
- C++コンパイラ（MSVC）とCMakeの動作確認済み構成を文書化する
- 依存ライブラリ更新は機能開発と分けて実施する

---

# Phase 0 — 開発環境・OCCT最小ビルド・FFI疎通

**目安:** 1〜2週間  
**目的:** Windows（MSVC + CMake）環境でOCCT最小構成をビルドし、Rustから安全に接続・メッシュ抽出・STEP入出力ができることを確認する。

## 実装範囲

### 0.1 リポジトリ・Workspace作成

Cargo Workspaceを作成し、Gitによるバージョン管理とリファレンス環境を整備する。

初期ディレクトリ構成：

```text
Free-Next-CAD/
├── Cargo.toml
├── Cargo.lock
├── rust-toolchain.toml
├── README.md
├── roadmap.md
├── ARCHITECTURE.md
├── CONTRIBUTING.md
├── agent-work-log.json
├── references/
│   ├── README.md
│   ├── fetch_references.py
│   └── repos/              # .gitignore対象（FreeCAD / OCCT shallow clone）
├── scripts/
│   └── build_occt.py       # OCCT最小構成CMakeビルド自動化スクリプト（py実行）
├── docs/
│   ├── decisions/
│   └── development/
├── crates/
│   ├── cad-core/
│   ├── cad-geometry/
│   ├── cad-occt/
│   ├── cad-render/
│   ├── cad-ui/
│   └── cad-cli/
├── tests/
│   └── fixtures/
└── .github/
    └── workflows/
```

最初は空のcrateが存在していても構わない。ただし、実際の実装は必要になった時点で追加する。

### 0.2 RustとC++の接続

- `cxx` を導入する
- C++の関数をRustから呼び出す
- C++で発生したエラー（`Standard_Failure` 等）をC++側 `try-catch` で捕捉し、Rust側の `Result` として受け取る
- C++例外がFFI境界を不適切に越えないようにする
- C++オブジェクト（`TopoDS_Shape` を保持する不透明ハンドル）の生成・破棄を確認する

### 0.3 OCCTの最小ビルドと基本実行

- `BUILD_MODULE_Draw=OFF`、`BUILD_MODULE_Visualization=OFF` 等の最小CMake構成でOCCT（`FoundationClasses`, `ModelingData`, `ModelingAlgorithms`, `DataExchange`）をビルド・リンクする
- 直方体（Box）を生成する
- 体積を取得する
- STEP形式で書き出す
- 書き出したSTEPを読み戻す
- `BRepMesh_IncrementalMesh` を実行し、三角形メッシュ（頂点座標・法線・インデックス・所属Faceインデックス）をRust側へ抽出する

### 完了条件

- [ ] `cargo build` が成功する
- [ ] `cargo test` が成功する
- [ ] RustからOCCTの直方体を生成できる
- [ ] 体積が期待値と許容誤差内で一致する
- [ ] STEP形式で保存・読み込みできる
- [ ] 直方体から表示用の三角形メッシュ（頂点・インデックス）を抽出できる
- [ ] C++例外を含む失敗ケース（寸法0や負数など）でRustプロセスが異常終了せず `Err` が返る
- [ ] 別の開発環境で再現するための手順・スクリプトが存在する

**成果物:** RustでOCCTの直方体生成・体積計算・STEP入出力・メッシュ抽出を行うCLIプログラム（`cad-cli`）

---

# Milestone M0 — 初期縦割りスライス（Thin Vertical Slice）

**目安:** Phase 0完了直後の1〜2週間  
**目的:** Phase 1〜3の本格展開前に、「直方体（Box）1個」だけで全クレート（`cad-occt` → `cad-geometry` → `cad-core` → `cad-render` → `cad-ui`）を一気通貫させ、画面上でパラメータ変更と3D再描画のループを確立する。

## 実装範囲

- `cad-geometry`: 最小の `ShapeStore`（`ShapeId` 発行）と `TriMesh` 構造体
- `cad-core`: Box 1個と寸法パラメータ（`width`, `height`, `depth`）を持つ最小の `Document`
- `cad-render`: `wgpu` で `TriMesh` を描画する最小パイプラインと Orbit/Zoom カメラ
- `cad-ui`: `eframe` ウィンドウ上で `width`, `height`, `depth` のスライダーを操作し、変更時に `cad-core` → `cad-occt` で再計算・再メッシュ化して3Dビューポートを即座に更新する

### 完了条件

- [ ] デスクトップウィンドウが起動し、OCCTで生成したBoxが3D表示される
- [ ] マウスドラッグでカメラの回転・ズームができる
- [ ] GUI上でBoxの寸法を変更すると、リアルタイムに3D形状が更新される

**成果物:** 全レイヤーが結合された最小の動作プロトタイプ（以降のPhase 1〜3はこの縦割り基盤の上で段階的に機能を拡充する）

---

# Phase 1 — 幾何演算エンジン

**目安:** 3〜4週間  
**目的:** OCCTを直接意識せずに利用できる安全なRust幾何APIを作る。

## 1.1 Shape抽象化

以下の概念を導入する。

- Shape
- Solid
- Face
- Edge
- Vertex
- BoundingBox
- Transform
- TriMesh（Face / Edge とメッシュ要素の対応マップを含む）
- GeometryError

OCCTの生ポインタや内部クラスは公開APIへ出さない。

初期段階では、形状の共有方法や所有権をFFIラッパー内部に閉じ込める。

## 1.2 基本形状

実装するプリミティブ：

- Box
- Cylinder
- Sphere
- Cone

寸法は原則としてミリメートルを基準に扱う。

外部APIでは可能な限り単位型を使用する。

## 1.3 幾何演算

- Fuse（結合）
- Cut（切削）
- Common（共通部分）
- Translate（移動）
- Rotate（回転）
- BoundingBox取得
- 体積・表面積取得
- 形状の妥当性確認（`BRepCheck_Analyzer`）

## 1.4 Shape Store

形状の所有・破棄を一元管理する。

Rust側のDocumentが形状の実体を直接所有する構成は避け、`ShapeId` などによる参照を基本とする。

最初は単一スレッド運用とし、スレッド間移動に関する安全性を十分確認するまでFFIオブジェクトへ安易に `Send` や `Sync` を実装しない。

## テスト

- 正しい寸法の直方体・円柱・球・円錐が作られる
- 無効な寸法が拒否される
- ブーリアン演算が正しく終了する
- 空形状や演算失敗がエラーとして返る
- Shape Storeから不要な形状を解放できる
- 繰り返し実行してメモリ使用量に明らかな異常がない

### 完了条件

- [ ] Rust APIから基本形状を生成できる
- [ ] Boolean演算が実行できる
- [ ] 形状検証APIがある
- [ ] エラーがRustのResultとして返る
- [ ] API利用側にOCCTの型が露出しない

**成果物:** 独立して利用できるRust幾何演算ライブラリ（`cad-occt` + `cad-geometry`）

---

# Phase 2 — DocumentとFeature DAG

**目安:** 3〜5週間  
**目的:** パラメトリックCADの中核となるドキュメント管理と差分再計算機構を実装する。

## 2.1 Document

Documentは以下を保持する。

- DocumentId
- Feature一覧
- Parameter一覧
- Feature間の依存関係
- 各Featureの状態
- 設計操作の履歴
- ドキュメントのバージョン

Document自体はOCCTのオブジェクトを保持しない。

## 2.2 Feature Graph

Featureをグラフ上のノードとして管理する。

```text
Parameter
   │
   ▼
 BaseBox ──────┐
               ▼
 Cylinder ──► Cut
               │
               ▼
            Result
```

### 必要な機能

- Feature追加
- Feature削除
- Feature変更
- 依存関係登録
- 循環依存検出
- トポロジカルソート
- 再計算順序の決定
- 無効化（Dirty）状態の伝播

## 2.3 再計算エンジン

Feature変更時、変更されたノードと依存先ノードを再計算する。

変更されていない部分はキャッシュ（`ShapeStore` 上の `ShapeId`）を利用する。

### 計算状態

- Clean
- Dirty
- Computing
- Failed
- Blocked

計算失敗を単なる空形状として扱わない。

失敗したFeatureと、その影響で計算できないFeatureを識別できるようにする。

## 2.4 Undo / Redo

操作をコマンドとして表現する。

例：

- AddFeature
- RemoveFeature
- SetParameter
- ChangeDependency

Undo/RedoはDocumentに対する状態変更を戻す仕組みとして実装する。

OCCTの形状キャッシュは必要に応じて破棄・再生成する。

## 2.5 シリアライズ

Documentの設計情報を独自のバージョン付き形式で保存する。

初期実装はJSONなど可読性の高い形式でよい。

保存対象と計算キャッシュを明確に区別する。

### 完了条件

- [ ] Feature依存関係を登録できる
- [ ] 循環依存を拒否できる
- [ ] 必要なFeatureのみ再計算できる
- [ ] エラー発生時に状態が破綻しない
- [ ] Undo/Redoが動作する
- [ ] 保存と読み込みができる
- [ ] 保存後の再計算結果が一致する

**成果物:** GUI不要でも単体動作・テスト可能なパラメトリックDocumentエンジン（`cad-core`）

---

# Phase 3 — 3Dビューポートと基本GUIの本格化

**目安:** 3〜5週間  
**目的:** Milestone M0の最小GUIを拡張し、Documentの計算結果・Feature Tree・選択機構を画面上で本格的に操作できるようにする。

## 3.1 レンダリング強化

OCCTで作成したB-Repから生成した三角形メッシュとエッジ線群を `wgpu` で描画する。

幾何演算用のB-Repと表示用メッシュは異なるデータとして管理する。

表示メッシュには元形状（`FeatureId`, `FaceIndex`, `EdgeIndex`）との対応関係を保持する。

## 3.2 基本ビュー操作

- 回転（Orbit）
- 平行移動（Pan）
- 拡大縮小（Zoom）
- Fit All
- 正面・上面・側面表示（ViewCubeまたはショートカット）
- 透視・平行投影の切り替え
- 表示／非表示
- ワイヤーフレーム／シェーディング＋エッジ表示の切り替え

## 3.3 基本UIと次世代UX基盤

- メニューバー＆統合コマンドパレット（ショートカットから即座に機能呼び出し）
- Feature Tree（状態アイコン・ロールバックバー表示）
- 3D Viewport（文脈に応じたHUD入力オーバーレイ）
- Property Panel
- エラーパネル（Failed / Blocked の視覚的フィードバック）
- ログ表示

## 3.4 オブジェクト選択（ピッキング）

初期段階ではFeatureまたはSolid単位の選択を実現する。

その後Face・Edge単位の選択へ拡張する。

ピッキングでは画面上の三角形/線分と元のB-Rep要素の対応（IDバッファまたはBVHレイキャスト）を考慮する。

### 完了条件

- [ ] 生成形状をエッジ強調付きで3D表示できる
- [ ] マウスで視点を快適に操作できる
- [ ] 3DビューポートおよびFeature TreeからFeatureを選択できる
- [ ] Property Panelで情報が表示・編集される
- [ ] Document変更後に表示が即座に更新される
- [ ] GUIから設計データをファイルへ保存・再読込できる

**成果物:** 複数フィーチャーを表示・選択・操作できるデスクトップCAD（`cad-render` + `cad-ui`）

---

# Phase 4 — パラメトリックモデリング

**目安:** 6〜10週間  
**目的:** CADとして基本的な部品設計と設計変更ができる状態にする。

## 4.1 Parameter System

寸法やパラメータを独立したデータとして管理する。

例：

```text
width = 100 mm
height = 50 mm
thickness = 10 mm
hole_diameter = 6.6 mm
```

初期実装は数値パラメータ中心とする。

その後、式による参照を追加する。

```text
hole_offset = width / 4
```

循環参照や単位の不整合はエラーとして検出する。

## 4.2 基本フィーチャー

- Primitive
- Extrude
- Revolve
- Boolean
- Transform
- Fillet
- Chamfer
- Linear Pattern

初期MVPではPrimitive、Extrude、Boolean、Transform、Linear Patternを優先する。

FilletやChamferは形状によって失敗しやすいため、後から安定性を高める。

## 4.3 GUIからのパラメータ編集

Property Panelまたは3D Viewport上のHUDで寸法を変更すると、依存するFeatureが再計算される。

正常に再計算された形状は3D Viewportへ反映する。

計算失敗時はエラーを表示し、Documentの論理状態と最後に正常だった表示結果を区別する。

## 4.4 操作履歴

ユーザー操作をコマンドとして記録する。

コマンドはCLIや将来のAI操作からも利用できる共通形式とする。

### 完了条件

- [ ] GUIから寸法・パラメータ式を変更できる
- [ ] 変更後に形状が差分再計算される
- [ ] 複数Featureを組み合わせられる
- [ ] Undo/Redoが安定している
- [ ] 設計データを保存・再読込できる
- [ ] 基本的な部品を作成してSTEP出力できる

### 最初の実用デモ（Milestone M1）

次の部品を作成できること。

**穴あきプレート**

- 幅100mm
- 高さ60mm
- 厚み10mm
- 直径6.6mmの穴4個
- 穴位置を寸法パラメータで制御

幅、厚み、穴径、穴位置を変更しても、正しく再計算される。

**成果物:** 最初のパラメトリックCAD MVP（Milestone M1）

---

# Phase 5 — スケッチャーと安定した形状参照

**目安:** 8〜14週間以上  
**目的:** 実用的な機械系CADに向けた高度な設計機能を導入する。

このPhaseは難度が高いため、2つの独立したサブプロジェクトとして進める。

## 5A. スケッチャー

### 基本図形

- Line
- Circle
- Arc
- Rectangle
- Polyline

### 拘束

- Coincident
- Horizontal
- Vertical
- Parallel
- Perpendicular
- Tangent
- Distance
- Radius
- Equal

### 必要な処理

- 拘束状態の管理
- 自由度の計算
- 数値ソルバーとの接続（FreeCADの `planGCS` 等の設計も参照しつつ既存RustソルバーやFFIを評価採用）
- 過拘束検出
- 未拘束状態の表示
- 閉じた輪郭（Wire/Face）の抽出
- Extrudeとの接続

拘束ソルバーは独自実装に固執せず、既存のソルバーを評価して採用する。

## 5B. Semantic Reference（トポロジカルネーミング問題の解決）

OCCTのFaceやEdgeに対して、設計上の安定した参照を与える仕組みを研究・実装する。

### 基本方針

- `FeatureId` と `SubshapeId` を区別する
- OCCTの形状履歴情報（`BRepAlgoAPI_BooleanOperation::Modified` / `Generated` / `IsDeleted`）を利用する
- 幾何学的特徴（法線方向、重心位置、面積、曲率タイプ等のシグネチャ）を補助情報として使用する
- 参照が曖昧な場合は勝手に確定しない
- 参照解決に失敗した理由を記録し、UI上でユーザーが再割り当て（Re-bind）しやすくする

### テストケース

- 直方体の寸法変更
- 穴の追加・削除
- Filletの追加・変更
- BooleanによるFaceの分割
- 対称形状における参照の曖昧性
- スケッチ参照面の変更

完全に安定したFace IDを保証することは目標にしない。

可能な範囲で参照を維持し、維持できない場合には検出・回復できる構造を目指す。

### 完了条件

- [ ] スケッチを作成・編集できる
- [ ] 基本拘束が機能する
- [ ] スケッチから押し出し（Extrude）できる
- [ ] 基本的な形状変更で参照を維持できる
- [ ] 参照解決に失敗した場合にエラーを通知できる
- [ ] 形状参照の回帰テストが存在する

**成果物:** スケッチを中心としたパラメトリック部品モデリング

---

# Phase 6 — 実用化と独自機能

**期間:** 継続開発

## 6.1 CADの基本機能強化

- アセンブリ
- 部品間拘束
- STEP入出力の強化（色・レイヤー・アセンブリ階層を含む `STEPCAFControl` 対応）
- 2D図面
- 寸法・公差
- 材料情報
- 部品表
- 設計テンプレート

## 6.2 設計履歴の高度化

- Document Snapshot
- 履歴の分岐
- バージョン比較
- 設計差分の可視化
- Feature単位の変更追跡

## 6.3 AI操作API

AIが自然言語から直接OCCTを操作する構成にはしない。

AIはDocumentに対する構造化コマンドを生成する。

実行前にコマンドの妥当性を検証する。

形状変更後は再計算結果とエラー情報をAIに返せるようにする。

## 6.4 製造向け機能

- 穴の規格情報
- 締結部品の関係
- 板金モデル
- 加工可能性の検証
- 工程情報
- 製造上の制約条件

## 6.5 高度な最適化

- Feature単位のキャッシュ
- インクリメンタル再計算
- 並列評価
- 遅延評価
- 大規模Documentへの対応

並列化はOCCTのスレッド安全性と共有状態を検証してから導入する。

---

# 品質保証・開発運用

## テスト方針

1. **Unit Test:** Rustの純粋な計算・データモデル
2. **Integration Test:** RustとOCCTの接続
3. **Geometry Test:** 形状の体積、面積、妥当性、メッシュ抽出の整合性
4. **Regression Test:** 過去に失敗した設計モデル
5. **Serialization Test:** 保存と再読み込み
6. **UI Test:** 主要な操作の動作確認

浮動小数点や幾何演算の結果は、適切な許容誤差で比較する。

## CI

最低限、次を自動実行する。

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

FFI・OCCTを含むテストには、OCCTをセットアップした専用CI環境を使用する。

## AIとの開発ルール

AIには原則として、一度に一つの機能単位を担当させる。

各タスクには以下を指定する。

- 背景
- 目的
- 変更を許可するファイル
- 必要なAPI
- テスト項目
- 完了条件
- 変更しない部分

AIによる大規模なアーキテクチャ変更は、実装前に提案・レビューを行う。

すべての変更はGit上で追跡できるようにする。

## 意思決定記録

重要な設計判断は `docs/decisions/` へADR（Architecture Decision Record）として保存する。

初期ADR候補：

- ADR-001: Rustを主要言語とする
- ADR-002: OCCTをCXX Bridgeから最小構成ビルドで利用する
- ADR-003: DocumentとGeometryを分離する
- ADR-004: Feature DAGによる再計算を採用する
- ADR-005: Shape Storeによる形状管理
- ADR-006: 初期描画バックエンドとしてwgpuを採用し、早期縦割りスライス（M0）で統合検証する

---

# 最終的な開発目標

FerroCADは、当初は小規模なパラメトリックモデラーとして開発する。

ただし、その内部設計は次のような将来の実験に耐えられることを重視する。

- フィーチャーツリーに依存しない設計編集
- 設計意図に基づく形状参照
- GUIとコードの双方向操作
- AIによる設計補助
- 製造条件と結びついたモデリング
- 設計履歴の分岐・比較・統合

**最重要原則：最初から完成されたCADを作ろうとしない。小さく正しいエンジンを、早期に画面と繋ぎながら段階的に育てる。**