# FerroCAD 開発引継ぎ

更新日: 2026-10-19

## 今回までに完了した範囲

パラメータ一覧とプロパティ編集を、フィーチャーの値・式・利用先が追える方向へ整備した。その上で、OCCTを使う次のフィーチャーをDocumentの履歴、Undo/Redo、保存・再計算の流れに組み込んだ。

- 基準面: `OffsetPlane`、`MidPlane`
- 面編集: 単一平面のPush-Pull、`Draft`
- 断面作成: 2断面Loft、直線パスの円形Sweep
- 実務形状: 座ぐり・皿穴、平面Face基準のMirror
- 修復・相互運用: Shape Healing、Refine、STEP診断、Sew、SolidFromShell、2ソリッドのSplit

Sweepは開始位置 `start_x` / `start_y` / `start_z` と半径をパラメータ式で指定できる。以前の「常に原点から開始」という制約は解消済みである。

詳細な完了・未完のチェックリストは [roadmap.md](/E:/CODE/Claude/Free-Next-CAD/roadmap.md) の M7〜M11 に記載した。

## 次に着手する順番

1. **M9の汎用化:** スケッチまたは辺をパスにするSweep、続けてRibとThicken。現在の直線円形Sweepを安全な最小実装として維持しながら広げる。
2. **M7/M8の編集性:** `PlanePattern`、任意ベクトルの`FaceMove`。基準要素を単なる一時値にせず、SemanticReferenceと保存対象を先に固める。
3. **M10の実務性:** 面基準の穴位置、穴仕様（貫通/止まり等）、`PatternOnReference`、接線連鎖/可変半径のブレンド。
4. **M11の相互運用:** `SurfaceOffset`、`TrimSurface`、複数入力Sew、面/平面によるSplit、STEPCAFでのアセンブリ・色・単位。

## 現在の制約と注意点

- Push-Pullは単一の平面Faceに対する単純ケースまで。曲面・複数面には広げていない。
- Loftは2断面の初期版。Sweepは円形断面を直線方向へ作る初期版であり、曲線パスは未対応。
- Holeはボディ座標系のXY位置で指定する。選択面に追従する位置指定、ねじ下穴、貫通/止まりの仕様化は未実装。
- Sewは単一入力形状、Splitは2ソリッドを工具にするケースが対象。Split出力は複合形状になる場合がある。
- STEP診断は単位ラベル、妥当性、トポロジーを表示する段階。STEPCAFによるアセンブリ構成・色・正確な単位の読込みは未実装。
- 未追跡の `roadmap.autosave.md` は作業中のユーザーファイルとして保持しており、今回の変更・コミットには含めない。

## 主な実装場所

- 履歴・再計算・フィーチャー定義: `crates/cad-core/src/lib.rs`
- UIのツール、選択、プロパティ: `crates/cad-ui/src/lib.rs`、`crates/cad-ui/src/tools.rs`
- OCCT Rust/FFI境界: `crates/cad-occt/src/lib.rs`、`crates/cad-occt/src/occt_bridge.cpp`
- 形状生成の補助: `crates/cad-geometry/src/lib.rs`
- 長期計画とマイルストーン: `roadmap.md`

## 検証状況

- `cargo check --workspace` はSweep開始位置の対応後に成功。
- `cargo test -p cad-core sweep_start_position_moves_the_generated_solid` は成功。
- `cargo test -p cad-ui m6_autosave_recovery_files_and_measure` は、一時ディレクトリ衝突を修正後に成功。
- 修正前の全ワークスペーステストでは、上記autosaveテストだけが一時ディレクトリ衝突で失敗していた。他の実行済みテストは成功している。

再開時はまず次を実行し、全体回帰を確認する。

```powershell
cargo test --workspace
cargo build --release -p cad-ui
```

リリース版の実行ファイルは `target/release/cad-ui.exe`。GUIでの実機確認は、OffsetPlane/MidPlane、Draft、Loft/Sweep、Sew/SolidFromShell/Splitを各1ケースずつ作り、パラメータ変更・Undo/Redo・保存/再読込まで確認する。

## 直近コミット

- `cc056cd` docs: record implemented occt roadmap milestones
- `7f5a400` test(core): cover positioned circular sweep
- `b68b1d1` feat(core): make sweep start position parametric
- `7d847cd` test(ui): isolate autosave recovery fixture
- `ae3267e` feat(ui): show imported STEP unit diagnostics
- `21a26bd` feat(ui): add solid partition workflow
- `f1170ae` test(occt): cover sewing and solidification

