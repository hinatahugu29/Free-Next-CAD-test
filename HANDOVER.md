# FerroCAD 開発引継ぎ

更新日: 2026-10-19

## 今回までに完了した範囲

M7〜M11の主要フィーチャーを実装・テストし、Document履歴・Undo/Redo・再計算・UIツールパネルへ統合した。

- 基準面（M7）: `OffsetPlane`、`MidPlane`、`PlanePattern`（間隔・個数による基準平面配列）
- 面編集（M8）: 単一平面のPush-Pull、`Draft`、`FaceMove`（任意ベクトル面移動。法線方向限定を解除、TopAbs_REVERSED対応）
- 断面作成（M9）: 2断面Loft、直線円形Sweep、汎用`Sweep`（任意スケッチプロファイル×3D折れ線パス）、`Rib`（スケッチ・平面・厚み・深さからリブ作成・主ボディ結合）、`Thicken`（`thick_solid`による開口面指定と内向き中空化）
- 実務形状（M10）:
  - `Hole`: 単純/座ぐり/皿穴/ねじ下穴、面基準の配置・貫通/止まり・先端角指定、パターン複製対応
  - `PatternOnReference`: 直線エッジ・円筒面軸・平面法線を基準とする直線および円形配列、ツール複製（穴や突起の繰り返し）
  - `EdgeBlend`: 可変半径フィレット（始点〜終点可変R）および接線連続エッジの自動探索・連鎖
- 修復・相互運用（M11）:
  - `SurfaceOffset`: 選択面を法線方向にオフセットした曲面作成
  - `TrimSurface`: 選択面のUVパラメータ境界をトリミングしたサーフェス作成
  - 複数入力`Sew`: 複数形状（フェイス・シェル）の一括縫合
  - 面・平面`Split`: 無限基準平面（XY/XZ/YZやカスタム平面）および工具ボディによるロバスト分割
  - STEPCAF連携: XCAF/STEPCAFによるアセンブリツリー構成、埋め込み部品名、色情報（RGBA）、単位メタデータ（MILLIMETRE/INCH/METRE）の一括インポート
  - Shape Healing、Refine、STEP診断、SolidFromShell

詳細な完了・未完のチェックリストは [roadmap.md](/E:/CODE/Claude/Free-Next-CAD/roadmap.md) の Phase 0〜4 および M7〜M11 に記載した（全項目達成）。

## 次に着手する順番

1. **Phase 7 実用化と独自機能:** 複数部品アセンブリ制約、図面生成、自由曲面モデリング。
2. **M8/M9の高度化:** 曲面・複数面へのPush-Pull/FaceMove、B-Spline曲線パス対応Sweep。
3. **UIの磨き込み:** テーマ、アイコン、操作フィードバックの向上。

## 現在の制約と注意点

- Push-Pull / FaceMoveは単一の平面Faceが対象。曲面へのPush-Pullは無理な自己交差を防止するため安全にErrを返す設計。
- Loftは2断面の初期版。
- 未追跡の `roadmap.autosave.md` は作業中のユーザーファイルとして保持しており、コミットには含めない。

## 主な実装場所

- 履歴・再計算・フィーチャー定義: `crates/cad-core/src/lib.rs`
- UIのツール、選択、プロパティ: `crates/cad-ui/src/lib.rs`、`crates/cad-ui/src/tools.rs`、`crates/cad-ui/src/files.rs`、`crates/cad-ui/src/rebind.rs`
- OCCT Rust/FFI境界: `crates/cad-occt/src/lib.rs`、`crates/cad-occt/src/occt_bridge.cpp`、`crates/cad-occt/include/occt_bridge.h`
- 形状生成の補助: `crates/cad-geometry/src/lib.rs`
- 長期計画とマイルストーン: `roadmap.md`

## 検証状況

- `cargo test --workspace` は全60件（cad-core 21件, cad-geometry 3件, cad-occt 16件, cad-render 1件, cad-ui 19件）が完全成功。
- `cargo build --release -p cad-ui` によるリリースビルドも確認可能。

## 直近コミット

- `b6bfdc6` docs: record agent work log for M10 and M11
- `538b054` feat: implement SurfaceOffset, TrimSurface, multi-input Sew, and plane Split (M11)
- `bf0bfc3` feat: implement parametric Hole specifications, PatternOnReference, and EdgeBlend (M10)
- `aec9da1` docs: record M7-M9 implementation commit in handover
- `a57935f` feat: implement generalized sweep, rib, face move, and plane pattern (M7-M9)
- `cc056cd` docs: record implemented occt roadmap milestones
- `7f5a400` test(core): cover positioned circular sweep
- `b68b1d1` feat(core): make sweep start position parametric
- `7d847cd` test(ui): isolate autosave recovery fixture
- `ae3267e` feat(ui): show imported STEP unit diagnostics
- `21a26bd` feat(ui): add solid partition workflow
- `f1170ae` test(occt): cover sewing and solidification

