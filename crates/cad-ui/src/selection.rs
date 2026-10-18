//! Selection in the 3D view (click / Shift+click multi-select of faces and edges), the
//! right-click menu, and deleting with an "also delete what depends on it" confirmation.

use super::*;

impl FerroCadApp {
    /// Applies a click on `face_index` (or, when the click was near one, `edge`) of
    /// `feature`. Plain click selects just that; additive (Shift/Ctrl) toggles it in the
    /// current selection.
    pub(crate) fn select_pick(&mut self, feature: FeatureId, face_index: u32, edge: Option<u32>, additive: bool) {
        let rc = &mut self.render_config;
        if !additive {
            rc.selected_faces.clear();
            rc.selected_edges.clear();
        }
        match edge {
            Some(e) => {
                let item = (feature, e);
                if let Some(i) = rc.selected_edges.iter().position(|x| *x == item) {
                    rc.selected_edges.remove(i);
                } else {
                    rc.selected_edges.push(item);
                }
            }
            None => {
                let item = (feature, face_index);
                if let Some(i) = rc.selected_faces.iter().position(|x| *x == item) {
                    rc.selected_faces.remove(i);
                } else {
                    rc.selected_faces.push(item);
                }
            }
        }
        rc.selected_face = rc.selected_faces.last().copied();
        rc.selected_edge = rc.selected_edges.last().copied();
        rc.selected_feature = Some(feature);
        self.viewport_dirty = true;
    }

    pub(crate) fn clear_selection(&mut self) {
        let rc = &mut self.render_config;
        rc.selected_face = None;
        rc.selected_faces.clear();
        rc.selected_edge = None;
        rc.selected_edges.clear();
        self.viewport_dirty = true;
    }

    /// The selected edges if they all lie on one body, else why they can't be used.
    pub(crate) fn edges_on_one_body(&self) -> Result<(FeatureId, Vec<u32>), &'static str> {
        let edges = &self.render_config.selected_edges;
        let Some(&(body, _)) = edges.first() else {
            return Err("先に3Dビューで辺をクリックして選択してください（Shift+クリックで複数）");
        };
        if edges.iter().any(|(f, _)| *f != body) {
            return Err("別々のボディの辺が含まれています。1つのボディの辺だけを選んでください");
        }
        Ok((body, edges.iter().map(|(_, e)| *e).collect()))
    }

    /// Total length of the selected edges, for display.
    pub(crate) fn selected_edges_length(&self) -> f64 {
        let doc = &self.controller.document;
        self.render_config
            .selected_edges
            .iter()
            .filter_map(|&(fid, idx)| {
                let shape = doc.features.get(&fid)?.cached_shape?;
                let info = self.store.inspect_shape(shape).ok()?;
                info.edges.iter().find(|e| e.index == idx).map(|e| e.length_mm)
            })
            .sum()
    }

    /// Deletes a feature; if others are built on it, asks whether to delete them too.
    pub(crate) fn request_delete(&mut self, id: FeatureId) {
        let dependents = self.controller.document.downstream_of(id);
        if dependents.is_empty() {
            self.delete_feature(id);
        } else {
            self.delete_confirm = Some((id, dependents));
        }
    }

    pub(crate) fn delete_with_dependents(&mut self, id: FeatureId) {
        let name = self.feature_name(id);
        let count = self.controller.document.downstream_of(id).len();
        match self
            .controller
            .execute(CadCommand::RemoveFeatureCascade { id }, Some(&mut self.store))
        {
            Ok(_) => {
                self.render_config.selected_feature = None;
                self.clear_selection();
                self.status_logs.push(format!(
                    "「{name}」と、それを使っていた {count} 個のフィーチャーを削除しました（Ctrl+Zでまとめて元に戻せます）"
                ));
                self.recompute_and_refresh();
            }
            Err(err) => self.status_logs.push(format!("削除できませんでした: {err}")),
        }
    }

    pub(crate) fn render_delete_confirm(&mut self, ctx: &Context) {
        let Some((id, dependents)) = self.delete_confirm.clone() else {
            return;
        };
        let mut choice = None;
        let names: Vec<String> = dependents
            .iter()
            .map(|d| format!("「{}」", self.feature_name(*d)))
            .collect();
        egui::Window::new("まとめて削除しますか？")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
            .show(ctx, |ui| {
                ui.label(format!(
                    "「{}」は {} 個のフィーチャーに使われています:",
                    self.feature_name(id),
                    dependents.len()
                ));
                ui.label(names.join("、"));
                ui.label("これらもまとめて削除します。Ctrl+Z で一度に元に戻せます。");
                ui.horizontal(|ui| {
                    if ui.button("🗑 まとめて削除").clicked() {
                        choice = Some(true);
                    }
                    if ui.button("キャンセル").clicked() || ui.input(|i| i.key_pressed(Key::Escape)) {
                        choice = Some(false);
                    }
                });
            });
        match choice {
            Some(true) => {
                self.delete_confirm = None;
                self.delete_with_dependents(id);
            }
            Some(false) => self.delete_confirm = None,
            None => {}
        }
    }

    /// Right-click menu for the current selection in the 3D view.
    pub(crate) fn render_selection_menu(&mut self, ui: &mut egui::Ui) {
        let Some(id) = self.render_config.selected_feature else {
            ui.weak("何も選択されていません");
            return;
        };
        let Some(feat) = self.controller.document.features.get(&id).cloned() else {
            return;
        };
        ui.label(RichText::new(&feat.name).strong());
        ui.separator();
        if let Ok((_, edges)) = self.edges_on_one_body() {
            let n = edges.len();
            if ui.button(format!("⌒ 選択中の{n}本の辺にフィレット")).clicked() {
                self.add_edge_modifier_on_selection(false);
                ui.close_menu();
            }
            if ui.button(format!("◣ 選択中の{n}本の辺を面取り")).clicked() {
                self.add_edge_modifier_on_selection(true);
                ui.close_menu();
            }
            ui.separator();
        }
        if matches!(feat.kind, FeatureKind::SketchExtrude { .. } | FeatureKind::SketchRevolve { .. })
            && ui.button("✏ スケッチを編集").clicked()
        {
            self.edit_sketch_feature(id);
            ui.close_menu();
        }
        if !feat.kind.reference_slots().is_empty() && ui.button("🎯 参照を選び直す…").clicked() {
            self.start_rebind(id, None);
            ui.close_menu();
        }
        if ui.button("✏ 名前を変更…").clicked() {
            self.rename_target = Some((id, feat.name.clone()));
            ui.close_menu();
        }
        if ui.button("非表示にする").clicked() {
            self.execute_cmd(CadCommand::UpdateFeature {
                id,
                name: None,
                kind: None,
                visible: Some(false),
                color_rgba: None,
            });
            self.clear_selection();
            ui.close_menu();
        }
        ui.separator();
        if ui.button("🗑 削除 (Delete)").clicked() {
            self.request_delete(id);
            ui.close_menu();
        }
    }
}
