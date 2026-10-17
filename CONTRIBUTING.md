# コントリビューションガイド (CONTRIBUTING.md)

## 開発フローと品質基準

すべての変更は以下の自動チェックを通過する必要があります。

```powershell
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## コーディング規約

1. **Safe Rustの徹底**: `cad-occt` のFFI境界を除き、`unsafe` の使用は原則禁止とします（`wgpu` 等の低レベルAPIで不可避な最小箇所を除く）。
2. **エラーハンドリング**: ライブラリクレート（`cad-occt`, `cad-geometry`, `cad-core`, `cad-render`）では `unwrap()` / `expect()` によるパニックを避け、型付きエラー（`Result`）を返してください。
3. **Python補助スクリプト**: Windows環境でのスクリプト実行は必ず `py` コマンドを使用してください。
4. **ADRの記録**: アーキテクチャ上の重要な意思決定は `docs/decisions/` に記録します。
