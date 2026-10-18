//! Re-binding a feature's face/edge reference by clicking the new target in the viewport,
//! plus the rollback-marker helper.

use super::*;
use cad_core::RefRole;

/// A reference being re-picked: which feature, which of its references, and the body
/// (shown alone in the viewport) the new face/edge must be picked on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RebindState {
    pub feature: FeatureId,
    pub role: RefRole,
    pub upstream: FeatureId,
}

pub(crate) fn role_label(role: RefRole) -> String {
    match role {
        RefRole::Edge(i) => format!("辺 {}", i + 1),
        RefRole::OpenFace(i) => format!("開口面 {}", i + 1),
        RefRole::Plane => "基準の面".to_string(),
        RefRole::MidPlaneFirst => "中間面の基準面 1".to_string(),
        RefRole::MidPlaneSecond => "中間面の基準面 2".to_string(),
        RefRole::DraftFace(i) => format!("ドラフト面 {}", i + 1),
        RefRole::DraftNeutral => "ドラフトの固定面".to_string(),
        RefRole::PushPullFace => "押出し・引込みの面".to_string(),
        RefRole::FaceMoveFace => "移動する面".to_string(),
        RefRole::PlanePatternFace => "基準平面配列の参照面".to_string(),
        RefRole::PatternAxisRef => "配列の参照軸（辺・面）".to_string(),
    }
}

impl FerroCadApp {
    /// Starts re-picking `role` of `id` (or, if `None`, its first broken reference, else
    /// its first reference).
    pub(crate) fn start_rebind(&mut self, id: FeatureId, role: Option<RefRole>) {
        let doc = &self.controller.document;
        let Some(feat) = doc.features.get(&id) else {
            return;
        };
        let slots = feat.kind.reference_slots();
        let slot = match role {
            Some(r) => slots.into_iter().find(|s| s.role == r),
            None => slots
                .iter()
                .find(|s| doc.check_reference(s, &self.store).is_err())
                .cloned()
                .or_else(|| slots.into_iter().next()),
        };
        let Some(slot) = slot else {
            return;
        };
        self.rebind = Some(RebindState {
            feature: id,
            role: slot.role,
            upstream: slot.upstream,
        });
        self.tool_panel = None;
        self.floating_editor_pos = None;
        self.render_config.selected_feature = Some(id);
        self.render_config.selected_face = None;
        self.render_config.selected_faces.clear();
        self.render_config.selected_edge = None;
        self.render_config.selected_edges.clear();
        self.viewport_dirty = true;
        let what = if slot.is_edge() { "辺" } else { "面" };
        self.status_logs.push(format!(
            "「{}」の{}を選び直します: 「{}」の{what}を3Dビューでクリックしてください（Escで中止）",
            self.feature_name(id),
            role_label(slot.role),
            self.feature_name(slot.upstream)
        ));
    }

    pub(crate) fn cancel_rebind(&mut self) {
        if self.rebind.take().is_some() {
            self.viewport_dirty = true;
        }
    }

    /// The document as the viewport should show it: while re-binding, only the body the
    /// new face/edge must be picked on is visible.
    pub(crate) fn view_document(&self) -> std::borrow::Cow<'_, Document> {
        let doc = &self.controller.document;
        match self.rebind {
            Some(state) => {
                let mut view = doc.clone();
                for f in view.features.values_mut() {
                    f.visible = f.id == state.upstream;
                }
                std::borrow::Cow::Owned(view)
            }
            // Re-editing a sketch feature: show the model as it was before that feature
            // (hide it and what follows; show the bodies it builds on).
            None if self.sketch_mode && self.sketch_editing.is_some() => {
                let editing = self.sketch_editing.unwrap_or(FeatureId(0));
                let mut view = doc.clone();
                for id in doc.downstream_of(editing).into_iter().chain([editing]) {
                    if let Some(f) = view.features.get_mut(&id) {
                        f.visible = false;
                    }
                }
                if let Some(feat) = doc.features.get(&editing) {
                    for input in feat.all_dependencies() {
                        if let Some(f) = view.features.get_mut(&input) {
                            f.visible = true;
                        }
                    }
                }
                std::borrow::Cow::Owned(view)
            }
            // Rolled back: bodies that were hidden because a later feature replaced them
            // are the current result again, so show them (all their users are set aside).
            None if doc.rollback.is_some() => {
                let reveal: Vec<FeatureId> = doc
                    .features
                    .keys()
                    .copied()
                    .filter(|&id| !doc.is_rolled_back(id))
                    .filter(|&id| {
                        let users = doc.direct_dependents(id);
                        !users.is_empty() && users.iter().all(|&u| doc.is_rolled_back(u))
                    })
                    .collect();
                let mut view = doc.clone();
                for id in reveal {
                    if let Some(f) = view.features.get_mut(&id) {
                        f.visible = true;
                    }
                }
                std::borrow::Cow::Owned(view)
            }
            None => std::borrow::Cow::Borrowed(doc),
        }
    }

    /// Applies a click made while re-binding: the face (or nearest edge) on the upstream
    /// body becomes the feature's new reference.
    pub(crate) fn apply_rebind_pick(&mut self, hit_feature: FeatureId, face_index: u32, edge_index: Option<u32>) {
        let Some(state) = self.rebind else {
            return;
        };
        if hit_feature != state.upstream {
            return;
        }
        let Some(shape) = self
            .controller
            .document
            .features
            .get(&state.upstream)
            .and_then(|f| f.cached_shape)
        else {
            return;
        };
        let is_edge = matches!(state.role, RefRole::Edge(_));
        let reference = if is_edge {
            let Some(edge) = edge_index else {
                self.status_logs
                    .push("辺の近くをクリックしてください".to_string());
                return;
            };
            SemanticReference::from_edge(state.upstream, &self.store, shape, edge)
        } else {
            SemanticReference::from_face(state.upstream, &self.store, shape, face_index)
        };
        let Ok(reference) = reference else {
            self.status_logs
                .push("その場所を参照として使えませんでした。別の場所をクリックしてください".to_string());
            return;
        };
        let Some(mut kind) = self
            .controller
            .document
            .features
            .get(&state.feature)
            .map(|f| f.kind.clone())
        else {
            return;
        };
        if !kind.replace_reference(state.role, reference) {
            return;
        }
        self.rebind = None;
        self.execute_cmd(CadCommand::UpdateFeature {
            id: state.feature,
            name: None,
            kind: Some(kind),
            visible: None,
            color_rgba: None,
        });
        let ok = self
            .controller
            .document
            .features
            .get(&state.feature)
            .is_some_and(|f| f.state == FeatureState::Clean);
        self.status_logs.push(if ok {
            format!("「{}」の{}を選び直しました", self.feature_name(state.feature), role_label(state.role))
        } else {
            format!(
                "「{}」の{}を選び直しましたが、まだ計算できません。右側の説明を確認してください",
                self.feature_name(state.feature),
                role_label(state.role)
            )
        });
        self.viewport_dirty = true;
    }

    /// Banner shown over the viewport while re-binding.
    pub(crate) fn render_rebind_banner(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let Some(state) = self.rebind else {
            return;
        };
        let what = if matches!(state.role, RefRole::Edge(_)) { "辺" } else { "面" };
        let panel = Rect::from_center_size(
            Pos2::new(rect.center().x, rect.min.y + 70.0),
            Vec2::new(rect.width().min(640.0) - 24.0, 60.0),
        );
        let mut cancel = false;
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(panel), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(40, 32, 18, 235))
                .show(ui, |ui| {
                    ui.strong(format!(
                        "🎯 「{}」の{}を選び直し中",
                        self.feature_name(state.feature),
                        role_label(state.role)
                    ));
                    ui.horizontal(|ui| {
                        ui.label(format!(
                            "表示している「{}」の{what}をクリックしてください",
                            self.feature_name(state.upstream)
                        ));
                        if ui.button("キャンセル (Esc)").clicked() {
                            cancel = true;
                        }
                    });
                });
        });
        if cancel || ui.input(|i| i.key_pressed(Key::Escape)) {
            self.cancel_rebind();
        }
    }

    /// Moves the rollback marker (`None` rolls forward to the end).
    pub(crate) fn set_rollback(&mut self, marker: Option<FeatureId>) {
        self.execute_cmd(CadCommand::SetRollback { marker });
        self.status_logs.push(match marker {
            Some(id) => format!(
                "「{}」まで戻しました。新しい操作はここに入り、後の操作はそれを使って作り直されます",
                self.feature_name(id)
            ),
            None => "最後まで進めました".to_string(),
        });
        self.viewport_dirty = true;
    }
}
