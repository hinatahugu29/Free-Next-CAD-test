//! Measuring in the 3D view: click one face for its area, two for distance and angle.

use glam::DVec3;

use super::*;

/// One clicked point on a face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeasurePick {
    pub feature: FeatureId,
    pub face_index: u32,
    pub point: DVec3,
    pub normal: DVec3,
    pub area_mm2: f64,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct MeasureState {
    pub picks: Vec<MeasurePick>,
}

impl FerroCadApp {
    pub(crate) fn start_measure(&mut self) {
        self.tool_panel = None;
        self.rebind = None;
        self.floating_editor_pos = None;
        self.measure = Some(MeasureState::default());
        self.status_logs
            .push("測定: 面をクリックすると面積、2か所クリックすると距離と角度を表示します（Escで終了）".to_string());
    }

    /// Records a click on `face_index` of `feature` at `point`; a third click starts over.
    pub(crate) fn measure_pick(&mut self, feature: FeatureId, face_index: u32, point: DVec3) {
        let Some(shape) = self
            .controller
            .document
            .features
            .get(&feature)
            .and_then(|f| f.cached_shape)
        else {
            return;
        };
        let Ok(info) = self.store.inspect_shape(shape) else {
            return;
        };
        let Some(face) = info.faces.iter().find(|f| f.index == face_index) else {
            return;
        };
        let pick = MeasurePick {
            feature,
            face_index,
            point,
            normal: face.normal.normalize_or_zero(),
            area_mm2: face.area_mm2,
        };
        let state = self.measure.get_or_insert_with(MeasureState::default);
        if state.picks.len() >= 2 {
            state.picks.clear();
        }
        state.picks.push(pick);
        self.render_config.selected_face = Some((feature, face_index));
        self.viewport_dirty = true;
    }

    /// Measurement results as (label, value) rows.
    pub(crate) fn measure_results(&self) -> Vec<(String, String)> {
        let Some(state) = &self.measure else {
            return Vec::new();
        };
        let mut rows = Vec::new();
        for (i, p) in state.picks.iter().enumerate() {
            rows.push((format!("面{}の面積", i + 1), format!("{:.2} mm²", p.area_mm2)));
        }
        if let [a, b] = state.picks.as_slice() {
            let d = b.point - a.point;
            rows.push(("点と点の距離".to_string(), format!("{:.3} mm", d.length())));
            rows.push((
                "　X / Y / Z 方向".to_string(),
                format!("{:.3} / {:.3} / {:.3} mm", d.x.abs(), d.y.abs(), d.z.abs()),
            ));
            let cos = a.normal.dot(b.normal).clamp(-1.0, 1.0);
            let angle = cos.acos().to_degrees();
            rows.push(("面と面の角度".to_string(), format!("{:.2}°", angle.min(180.0 - angle))));
            // Parallel planar faces: the gap between them along the normal.
            if cos.abs() > 0.9999 {
                rows.push(("面と面の距離（平行）".to_string(), format!("{:.3} mm", d.dot(a.normal).abs())));
            }
        }
        rows
    }

    /// Markers, the connecting line and the distance label drawn over the viewport.
    pub(crate) fn draw_measure_overlay(&self, ui: &egui::Ui, rect: Rect) {
        let Some(state) = &self.measure else {
            return;
        };
        let aspect = rect.width() / rect.height().max(1.0);
        let vp = self.camera.view_proj_matrix(aspect);
        let to_screen = |p: DVec3| -> Option<Pos2> {
            let clip = vp * glam::Vec4::new(p.x as f32, p.y as f32, p.z as f32, 1.0);
            if clip.w <= 1e-6 {
                return None;
            }
            let ndc = clip.truncate() / clip.w;
            Some(Pos2::new(
                rect.min.x + (ndc.x * 0.5 + 0.5) * rect.width(),
                rect.min.y + (0.5 - ndc.y * 0.5) * rect.height(),
            ))
        };
        let painter = ui.painter_at(rect);
        let color = Color32::from_rgb(255, 220, 90);
        let screen: Vec<Pos2> = state.picks.iter().filter_map(|p| to_screen(p.point)).collect();
        if let [a, b] = screen.as_slice() {
            painter.line_segment([*a, *b], egui::Stroke::new(2.0_f32, color));
            if let [pa, pb] = state.picks.as_slice() {
                let label = format!("{:.2} mm", (pb.point - pa.point).length());
                painter.text(
                    *a + (*b - *a) * 0.5 + Vec2::new(0.0, -14.0),
                    egui::Align2::CENTER_BOTTOM,
                    label,
                    egui::FontId::proportional(15.0),
                    color,
                );
            }
        }
        for (i, s) in screen.iter().enumerate() {
            painter.circle_filled(*s, 5.0, color);
            painter.text(
                *s + Vec2::new(8.0, -8.0),
                egui::Align2::LEFT_BOTTOM,
                format!("{}", i + 1),
                egui::FontId::proportional(13.0),
                color,
            );
        }
    }

    pub(crate) fn render_measure_panel(&mut self, ctx: &Context) {
        if self.measure.is_none() {
            return;
        }
        let rows = self.measure_results();
        let mut close = false;
        let mut clear = false;
        egui::Window::new("⊿ 測定")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::RIGHT_TOP, Vec2::new(-320.0, 140.0))
            .show(ctx, |ui| {
                ui.set_min_width(260.0);
                if rows.is_empty() {
                    ui.label("測りたい面をクリックしてください");
                } else {
                    egui::Grid::new("measure_rows").num_columns(2).show(ui, |ui| {
                        for (label, value) in &rows {
                            ui.label(label);
                            ui.strong(value);
                            ui.end_row();
                        }
                    });
                    if self.measure.as_ref().is_some_and(|m| m.picks.len() == 1) {
                        ui.small("もう1か所クリックすると、距離と角度が出ます");
                    }
                }
                ui.separator();
                ui.horizontal(|ui| {
                    if ui.button("やり直す").clicked() {
                        clear = true;
                    }
                    if ui.button("終了 (Esc)").clicked() {
                        close = true;
                    }
                });
            });
        if clear && let Some(m) = &mut self.measure {
            m.picks.clear();
        }
        if close || ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.measure = None;
            self.render_config.selected_face = None;
            self.viewport_dirty = true;
        }
    }
}
