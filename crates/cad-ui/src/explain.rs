//! Plain-Japanese explanations of recompute failures: what went wrong and what to do.

use cad_core::{CoreError, Document, ResolutionFailureReason};
use cad_geometry::GeometryError;

pub(crate) struct Explanation {
    pub summary: String,
    pub action: String,
}

fn ex(summary: impl Into<String>, action: impl Into<String>) -> Explanation {
    Explanation {
        summary: summary.into(),
        action: action.into(),
    }
}

fn name_of(doc: &Document, id: cad_core::FeatureId) -> String {
    doc.features
        .get(&id)
        .map(|f| f.name.clone())
        .unwrap_or_else(|| format!("#{}", id.0))
}

pub(crate) fn explain(err: &CoreError, doc: &Document) -> Explanation {
    match err {
        CoreError::SemanticReferenceLost { feature, reason } => {
            let name = name_of(doc, *feature);
            let rebind = "右側の「参照」欄の［選び直す］を押し、3Dビューで新しい面・辺をクリックしてください";
            match reason {
                ResolutionFailureReason::MissingTargetShape(_) => ex(
                    format!("「{name}」の形状が計算できていないため、使っていた面・辺を確認できません"),
                    format!("先に「{name}」のエラーを解消してください"),
                ),
                ResolutionFailureReason::NoMatchingSubshape { .. } => ex(
                    format!("前の工程の変更で、「{name}」の上で使っていた面・辺が見つからなくなりました"),
                    rebind,
                ),
                ResolutionFailureReason::AmbiguousCandidates { candidate_indices, .. } => ex(
                    format!(
                        "「{name}」の上で、参照先の候補が{}つあり、どれか決められません（勝手に選ぶと意図と違う形になるため止めています）",
                        candidate_indices.len()
                    ),
                    rebind,
                ),
            }
        }
        CoreError::Geometry(g) => explain_geometry(g),
        CoreError::ParameterError { name, reason } => ex(
            format!("パラメータ「{name}」に問題があります（{reason}）"),
            "左のパラメータ一覧で値や式を見直してください",
        ),
        CoreError::CircularParameterDependency(name) => ex(
            format!("パラメータ「{name}」の式が、回り回って自分自身を参照しています"),
            "どこかの式で、循環しないように参照先を変えてください",
        ),
        CoreError::FeatureNotFound(id) => ex(
            format!("参照先のフィーチャー（#{}）が見つかりません", id.0),
            "削除してしまった場合は Ctrl+Z で元に戻せます",
        ),
        other => ex(other.to_string(), ""),
    }
}

fn explain_geometry(g: &GeometryError) -> Explanation {
    match g {
        GeometryError::InvalidParameter { name, reason } => {
            if reason.contains("valid closed 2D profile") {
                ex(
                    "スケッチに閉じた輪郭がありません",
                    "スケッチを編集して、線を始点まで戻すか、矩形・円で輪郭を閉じてください",
                )
            } else if reason.contains("only hole loops") {
                ex(
                    "スケッチに外側の輪郭がありません（内側の穴だけになっています）",
                    "外側を囲む輪郭を描いてください",
                )
            } else if reason.contains("Sketch solver failed") {
                ex(
                    "スケッチの拘束を満たす形が見つかりません",
                    "スケッチを編集して、矛盾している拘束や寸法を外してください",
                )
            } else if reason.contains("crosses the revolve axis") {
                ex(
                    "回転の輪郭が軸をまたいでいます",
                    "輪郭を軸の片側に収めるか、別の線を軸に選んでください",
                )
            } else if reason.contains("zero length") {
                ex("回転軸の長さが0です", "軸にする線を引き直してください")
            } else {
                ex(
                    format!("「{name}」の値が使えません（{reason}）"),
                    "0より大きい値など、妥当な値に変更してください",
                )
            }
        }
        GeometryError::InvalidTopology => ex(
            "計算結果の形状が壊れています",
            "直前の変更を Ctrl+Z で元に戻し、少しずつ値を変えてみてください",
        ),
        GeometryError::Kernel(k) => {
            let msg = k.to_string();
            if msg.contains("index out of range") {
                ex(
                    "使っていた面・辺が形状から消えました",
                    "その操作をやり直すか、右側の「参照」欄から選び直してください",
                )
            } else if msg.contains("Fillet") {
                ex(
                    "フィレットを作れませんでした",
                    "半径を小さくするか、隣り合う面との兼ね合いを確認してください",
                )
            } else if msg.contains("Chamfer") {
                ex("面取りを作れませんでした", "距離を小さくしてみてください")
            } else if msg.contains("ThickSolid") || msg.contains("shell") {
                ex(
                    "シェル（くり抜き）を作れませんでした",
                    "壁を薄くするか、別の面を開口にしてください",
                )
            } else if msg.contains("Revol") {
                ex("回転体を作れませんでした", "輪郭と軸の位置関係を確認してください")
            } else if msg.contains("Fuse") || msg.contains("Cut") || msg.contains("Common") || msg.contains("Boolean") {
                ex(
                    "形状の組み合わせ（足す・引く）に失敗しました",
                    "重なり方や位置を少しずらしてみてください",
                )
            } else {
                ex(
                    format!("形状の計算に失敗しました（{msg}）"),
                    "直前に変えた値を Ctrl+Z で戻し、少しずつ変えてみてください",
                )
            }
        }
        other => ex(other.to_string(), ""),
    }
}
