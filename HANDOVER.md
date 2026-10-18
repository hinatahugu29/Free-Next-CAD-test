# FerroCAD 開発引継ぎ

更新日: 2026-10-19

## 今回までに完了した範囲

M7〜M9の主要フィーチャーを実装・テストし、Document履歴・Undo/Redo・再計算・UIツールパネルへ統合した。

- 基準面（M7）: `OffsetPlane`、`MidPlane`、`PlanePattern`（間隔・個数による基準平面配列）
- 面編集（M8）: 単一平面のPush-Pull、`Draft`、`FaceMove`（任意ベクトル面移動。法線方向限定を解除、TopAbs_REVERSED対応）
- 断面作成（M9）: 2断面Loft、直線円形Sweep、汎用`Sweep`（任意スケッチプロファイル×3D折れ線パス）、`Rib`（スケッチ・平面・厚み・深さからリブ作成・主ボディ結合）、`Thicken`（`thick_solid`による開口面指定と内向き中空化）
- 実務形状（M10初期）: 座ぐり・皿穴、平面Face基準のMirror
- 修復・相互運用（M11初期）: Shape Healing、Refine、STEP診断、Sew、SolidFromShell、2ソリッドのSplit

詳細な完了・未完のチェックリストは [roadmap.md](/E:/CODE/Claude/Free-Next-CAD/roadmap.md) の M7〜M11 に記載した。

## 次に着手する順番

1. **M10の実務性:** 面基準の穴位置、穴仕様（貫通/止まり等）、`PatternOnReference`、接線連鎖/可変半径のブレンド。
2. **M11の相互運用:** `SurfaceOffset`、`TrimSurface`、複数入力Sew、面/平面によるSplit、STEPCAFでのアセンブリ・色・単位。
3. **M8/M9の高度化:** 曲面・複数面へのPush-Pull/FaceMove、曲線パス対応Sweep。

## 現在の制約と注意点

- Push-Pull / FaceMoveは単一の平面Faceが対象。曲面・複数面は未対応。
- Loftは2断面の初期版。
- Holeはボディ座標系のXY位置で指定する。選択面に追従する位置指定、ねじ下穴、貫通/止まりの仕様化は後続のM10で拡張。
- Sewは単一入力形状、Splitは2ソリッドを工具にするケースが対象。
- 未追跡の `roadmap.autosave.md` は作業中のユーザーファイルとして保持しており、コミットには含めない。

## 主な実装場所

- 履歴・再計算・フィーチャー定義: `crates/cad-core/src/lib.rs`
- UIのツール、選択、プロパティ: `crates/cad-ui/src/lib.rs`、`crates/cad-ui/src/tools.rs`、`crates/cad-ui/src/rebind.rs`
- OCCT Rust/FFI境界: `crates/cad-occt/src/lib.rs`、`crates/cad-occt/src/occt_bridge.cpp`、`crates/cad-occt/include/occt_bridge.h`
- 形状生成の補助: `crates/cad-geometry/src/lib.rs`
- 長期計画とマイルストーン: `roadmap.md`

## 検証状況

- `cargo test --workspace` は全48件（cad-core 17件, cad-geometry 2件, cad-occt 11件, cad-render 1件, cad-ui 17件）が完全成功。
- `cargo build --release -p cad-ui` によるリリースビルドも確認可能。

## 直近コミット

- `a57935f` feat: implement generalized sweep, rib, face move, and plane pattern (M7-M9)
- `cc056cd` docs: record implemented occt roadmap milestones
- `7f5a400` test(core): cover positioned circular sweep
- `b68b1d1` feat(core): make sweep start position parametric
- `7d847cd` test(ui): isolate autosave recovery fixture
- `ae3267e` feat(ui): show imported STEP unit diagnostics
- `21a26bd` feat(ui): add solid partition workflow
- `f1170ae` test(occt): cover sewing and solidification

