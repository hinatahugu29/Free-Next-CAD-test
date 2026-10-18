//! "Select → adjust → confirm" panels for the modify tools on the ribbon: circular and
//! linear patterns, mirror, boolean, and shell.

use super::*;
use cad_core::{BooleanOp, PatternLayout};

const AXES: [(&str, [f64; 3]); 3] = [("X", [1.0, 0.0, 0.0]), ("Y", [0.0, 1.0, 0.0]), ("Z", [0.0, 0.0, 1.0])];
const PLANES: [(&str, SketchPlane); 3] = [
    ("XY平面", SketchPlane::XY),
    ("XZ平面", SketchPlane::XZ),
    ("YZ平面", SketchPlane::YZ),
];

impl FerroCadApp {
    /// For a sketch Join/Cut feature, the body it modifies (its tool gets repeated there).
    fn tool_target(&self, id: FeatureId) -> Option<FeatureId> {
        match &self.controller.document.features.get(&id)?.kind {
            FeatureKind::SketchExtrude { mode, .. } | FeatureKind::SketchRevolve { mode, .. } => match mode {
                ExtrudeMode::Join { target } | ExtrudeMode::Cut { target } => Some(*target),
                ExtrudeMode::NewBody => None,
            },
            _ => None,
        }
    }

    /// Selected faces that lie on `body` (the shell's open faces).
    fn open_faces_on(&self, body: FeatureId) -> Vec<u32> {
        self.render_config
            .selected_faces
            .iter()
            .filter(|(f, _)| *f == body)
            .map(|(_, i)| *i)
            .collect()
    }

    fn bbox_center(&self, id: FeatureId) -> Option<[f64; 3]> {
        let f = self.controller.document.features.get(&id)?;
        let b = f.cached_summary.or(f.last_valid_summary)?.bounds;
        Some(b.center().to_array())
    }

    pub(crate) fn open_tool_panel(&mut self, kind: ToolKind) {
        let Some(input) = self.render_config.selected_feature else {
            return;
        };
        self.floating_editor_pos = None;
        self.tool_panel = Some(match kind {
            ToolKind::Transform => ToolPanel::Transform {
                input,
                insert_after: None,
                translation: [0.0; 3],
                rotation_axis: 2,
                rotation_angle_deg: 0.0,
            },
            ToolKind::CircularPattern => {
                // A ring of holes usually goes around the centre of the body they cut.
                let center = self
                    .tool_target(input)
                    .and_then(|t| self.bbox_center(t))
                    .unwrap_or([0.0; 3]);
                ToolPanel::CircularPattern {
                    input,
                    axis: 2,
                    center,
                    count: 6,
                    angle: 360.0,
                }
            }
            ToolKind::LinearPattern => ToolPanel::LinearPattern {
                input,
                direction: 0,
                spacing: 20.0,
                count: 3,
            },
            ToolKind::Mirror => ToolPanel::Mirror {
                input,
                plane: 2,
                merge: true,
            },
            ToolKind::Boolean => ToolPanel::Boolean {
                a: input,
                b: None,
                op: BooleanOp::Cut,
            },
            ToolKind::Shell => {
                let Some((fid, _)) = self.render_config.selected_face else {
                    return;
                };
                ToolPanel::Shell {
                    input: fid,
                    thickness: 2.0,
                }
            }
            ToolKind::OffsetPlane => {
                let Some((fid, face_index)) = self.render_config.selected_face else {
                    return;
                };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else {
                    return;
                };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, face_index) else {
                    self.status_logs.push("選んだ面を基準にできませんでした".to_string());
                    return;
                };
                ToolPanel::OffsetPlane {
                    input: fid,
                    face,
                    offset: 10.0,
                }
            }
            ToolKind::Draft => {
                let Some((fid, _)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let faces: Option<Vec<_>> = self.open_faces_on(fid).into_iter()
                    .map(|idx| SemanticReference::from_face(fid, &self.store, shape, idx).ok()).collect();
                let Some(faces) = faces.filter(|v| v.len() >= 2) else {
                    self.status_logs.push("同じボディの面を2つ以上選んでください".to_string()); return;
                };
                ToolPanel::Draft { input: fid, faces, neutral_index: 0, angle_deg: 3.0, reversed: false }
            }
            ToolKind::Heal => ToolPanel::Heal { input },
        });
    }

    /// Opens a transform for the Boolean's tool (secondary input), not for the Boolean
    /// result. The confirm path inserts it after the tool so the Boolean is retargeted.
    pub(crate) fn open_boolean_tool_transform(&mut self, _boolean: FeatureId, tool: FeatureId) {
        self.floating_editor_pos = None;
        self.tool_panel = Some(ToolPanel::Transform {
            input: tool,
            insert_after: Some(tool),
            translation: [0.0; 3],
            rotation_axis: 2,
            rotation_angle_deg: 0.0,
        });
    }

    pub(crate) fn render_tool_panel(&mut self, ctx: &Context) {
        let Some(mut panel) = self.tool_panel.clone() else {
            return;
        };
        let title = match panel {
            ToolPanel::Transform { .. } => "↔ 移動・回転",
            ToolPanel::CircularPattern { .. } => "⟲ 円形パターン",
            ToolPanel::LinearPattern { .. } => "… 直線パターン",
            ToolPanel::Mirror { .. } => "∥ ミラー",
            ToolPanel::Boolean { .. } => "◆ ブーリアン",
            ToolPanel::Shell { .. } => "□ シェル",
            ToolPanel::OffsetPlane { .. } => "▱ オフセット面からスケッチ",
            ToolPanel::Draft { .. } => "◢ ドラフト",
            ToolPanel::Heal { .. } => "✚ 形状を修復",
        };
        let mut confirm = false;
        let mut cancel = false;
        let names: Vec<(FeatureId, String)> = self
            .controller
            .document
            .features
            .values()
            .map(|f| (f.id, f.name.clone()))
            .collect();

        egui::Window::new(title)
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::RIGHT_TOP, Vec2::new(-320.0, 140.0))
            .show(ctx, |ui| {
                ui.set_min_width(300.0);
                match &mut panel {
                    ToolPanel::Transform {
                        input,
                        translation,
                        rotation_axis,
                        rotation_angle_deg,
                        ..
                    } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.strong("移動量:");
                        ui.horizontal(|ui| {
                            ui.add(egui::DragValue::new(&mut translation[0]).speed(0.5).prefix("X: ").suffix(" mm"));
                            ui.add(egui::DragValue::new(&mut translation[1]).speed(0.5).prefix("Y: ").suffix(" mm"));
                            ui.add(egui::DragValue::new(&mut translation[2]).speed(0.5).prefix("Z: ").suffix(" mm"));
                        });
                        ui.horizontal(|ui| {
                            if ui.small_button("原点に戻す").clicked() {
                                *translation = [0.0; 3];
                            }
                        });
                        ui.separator();
                        ui.strong("回転（任意）:");
                        ui.horizontal(|ui| {
                            ui.label("軸:");
                            for (i, (label, _)) in AXES.iter().enumerate() {
                                ui.selectable_value(rotation_axis, i, *label);
                            }
                            ui.add(egui::DragValue::new(rotation_angle_deg).speed(1.0).suffix("°"));
                        });
                        ui.small("回転は形状の中心を通る軸を基準にします");
                    }
                    ToolPanel::CircularPattern {
                        input,
                        axis,
                        center,
                        count,
                        angle,
                    } => {
                        self.describe_pattern_input(ui, *input);
                        ui.horizontal(|ui| {
                            ui.label("回転軸:");
                            for (i, (label, _)) in AXES.iter().enumerate() {
                                ui.selectable_value(axis, i, *label);
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("軸の位置:");
                            for v in center.iter_mut() {
                                ui.add(egui::DragValue::new(v).speed(0.5).suffix(" mm"));
                            }
                        });
                        ui.horizontal(|ui| {
                            if ui.small_button("原点").clicked() {
                                *center = [0.0; 3];
                            }
                            let body = self.tool_target(*input).unwrap_or(*input);
                            if ui.small_button("対象ボディの中心").clicked()
                                && let Some(c) = self.bbox_center(body)
                            {
                                *center = c;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("個数:");
                            ui.add(egui::DragValue::new(count).range(2..=360));
                            ui.label("全体の角度:");
                            ui.add(egui::DragValue::new(angle).range(1.0..=360.0).suffix("°"));
                        });
                        ui.small("360°なら等間隔に一周、それより小さければ最初から最後までその角度に収めます");
                    }
                    ToolPanel::LinearPattern {
                        input,
                        direction,
                        spacing,
                        count,
                    } => {
                        self.describe_pattern_input(ui, *input);
                        ui.horizontal(|ui| {
                            ui.label("方向:");
                            for (i, (label, _)) in AXES.iter().enumerate() {
                                ui.selectable_value(direction, i, *label);
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("間隔:");
                            ui.add(egui::DragValue::new(spacing).speed(0.5).suffix(" mm"));
                            ui.label("個数:");
                            ui.add(egui::DragValue::new(count).range(2..=200));
                        });
                        ui.small("間隔をマイナスにすると逆向きに並びます");
                    }
                    ToolPanel::Mirror { input, plane, merge } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("鏡の面:");
                            for (i, (label, _)) in PLANES.iter().enumerate() {
                                ui.selectable_value(plane, i, *label);
                            }
                        });
                        if self.tool_target(*input).is_some() {
                            ui.small("穴や突起は、反対側にも同じものが作られます");
                        } else {
                            ui.checkbox(merge, "元の形状と結合する（左右対称の1つの部品にする）");
                        }
                    }
                    ToolPanel::Boolean { a, b, op } => {
                        ui.label(format!("残す側: 「{}」", self.feature_name(*a)));
                        let current = b
                            .map(|id| self.feature_name(id))
                            .unwrap_or_else(|| "（選んでください）".to_string());
                        ui.horizontal(|ui| {
                            ui.label("もう一方:");
                            egui::ComboBox::from_id_salt("boolean_tool")
                                .selected_text(current)
                                .show_ui(ui, |ui| {
                                    for (id, name) in &names {
                                        if id != a {
                                            ui.selectable_value(b, Some(*id), name);
                                        }
                                    }
                                });
                        });
                        ui.horizontal(|ui| {
                            ui.label("操作:");
                            ui.selectable_value(op, BooleanOp::Fuse, "足す（和）");
                            ui.selectable_value(op, BooleanOp::Cut, "引く（差）");
                            ui.selectable_value(op, BooleanOp::Common, "重なり（積）");
                        });
                    }
                    ToolPanel::Shell { input, thickness } => {
                        // Follow the face selection so the open face can be re-picked.
                        if let Some((fid, _)) = self.render_config.selected_face {
                            *input = fid;
                        }
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        let open = self.open_faces_on(*input);
                        if open.is_empty() {
                            ui.colored_label(Color32::from_rgb(240, 150, 65), "開口にする面を3Dビューでクリックしてください");
                        } else {
                            ui.label(format!(
                                "開口にする面: {}面（Shift+クリックで追加・解除）",
                                open.len()
                            ));
                        }
                        ui.horizontal(|ui| {
                            ui.label("壁の厚さ:");
                            ui.add(egui::DragValue::new(thickness).speed(0.1).range(0.05..=1000.0).suffix(" mm"));
                        });
                        ui.small("外形はそのままで、内側をくり抜きます");
                    }
                    ToolPanel::OffsetPlane { input, offset, .. } => {
                        ui.label(format!("基準面: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("面からの距離:");
                            ui.add(egui::DragValue::new(offset).speed(0.5).suffix(" mm"));
                        });
                        ui.small("プラスは面の法線方向、マイナスは反対方向です。確定すると、この平面でスケッチを開始します。");
                    }
                    ToolPanel::Draft { input, faces, neutral_index, angle_deg, reversed } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.label(format!("選択面: {} 面", faces.len()));
                        ui.horizontal(|ui| {
                            ui.label("固定面:");
                            egui::ComboBox::from_id_salt("draft_neutral").selected_text(format!("面 {}", *neutral_index + 1)).show_ui(ui, |ui| {
                                for i in 0..faces.len() { ui.selectable_value(neutral_index, i, format!("面 {}", i + 1)); }
                            });
                        });
                        ui.horizontal(|ui| { ui.label("角度:"); ui.add(egui::DragValue::new(angle_deg).range(0.1..=88.0).suffix("°")); });
                        ui.checkbox(reversed, "反対方向へ傾ける");
                        ui.small("固定面以外の選択面を傾けます。初期版は平面の固定面に対応します。");
                    }
                    ToolPanel::Heal { input } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.small("OCCTのShape Healingで修復済みコピーを作ります。");
                    }
                }

                ui.separator();
                let ready = match &panel {
                    ToolPanel::Boolean { b, .. } => b.is_some(),
                    ToolPanel::Shell { input, .. } => !self.open_faces_on(*input).is_empty(),
                    _ => true,
                };
                ui.horizontal(|ui| {
                    if ui.add_enabled(ready, egui::Button::new("✓ 確定")).clicked() {
                        confirm = true;
                    }
                    if ui.button("× キャンセル (Esc)").clicked() {
                        cancel = true;
                    }
                });
            });

        if ctx.input(|i| i.key_pressed(Key::Escape)) {
            cancel = true;
        }
        self.tool_panel = if confirm {
            self.confirm_tool(panel);
            None
        } else if cancel {
            None
        } else {
            Some(panel)
        };
    }

    fn describe_pattern_input(&self, ui: &mut egui::Ui, input: FeatureId) {
        ui.label(format!("対象: 「{}」", self.feature_name(input)));
        match self.tool_target(input) {
            Some(target) => ui.small(format!("穴・突起を「{}」の上に並べます", self.feature_name(target))),
            None => ui.small("形状そのものを複製して並べ、1つにまとめます"),
        };
    }

    /// Adds `kind` and hides `replaced` features, whose result the new feature now shows.
    fn add_replacing(&mut self, name: String, kind: FeatureKind, color_from: FeatureId, replaced: &[FeatureId]) {
        let color_rgba = self
            .controller
            .document
            .features
            .get(&color_from)
            .map(|f| f.color_rgba)
            .unwrap_or([0.55, 0.75, 0.95, 1.0]);
        let before = self.controller.document.features.len();
        self.execute_cmd(CadCommand::AddFeature {
            name,
            kind,
            visible: true,
            color_rgba,
        });
        if self.controller.document.features.len() == before {
            return;
        }
        let new_feature = self.render_config.selected_feature;
        for &id in replaced {
            self.execute_cmd(CadCommand::UpdateFeature {
                id,
                name: None,
                kind: None,
                visible: Some(false),
                color_rgba: None,
            });
        }
        self.render_config.selected_feature = new_feature;
        self.render_config.selected_face = None;
        self.render_config.selected_faces.clear();
        self.render_config.selected_edge = None;
        self.render_config.selected_edges.clear();
    }

    pub(crate) fn confirm_tool(&mut self, panel: ToolPanel) {
        let n = self.controller.document.features.len() + 1;
        match panel {
            ToolPanel::Transform {
                input,
                insert_after,
                translation,
                rotation_axis,
                rotation_angle_deg,
            } => {
                let center = self.bbox_center(input).unwrap_or([0.0; 3]);
                let kind = FeatureKind::Transform {
                    input,
                    dx: ScalarParam::Literal(translation[0]),
                    dy: ScalarParam::Literal(translation[1]),
                    dz: ScalarParam::Literal(translation[2]),
                    rot_origin: center,
                    rot_axis: AXES[rotation_axis].1,
                    rot_angle_deg: ScalarParam::Literal(rotation_angle_deg),
                };
                if let Some(marker) = insert_after {
                    // `add_feature` already knows how to insert after a rollback marker
                    // and retarget later uses of its input (including the Boolean tool).
                    self.set_rollback(Some(marker));
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("ToolTransform_{n}"),
                        kind,
                        visible: false,
                        color_rgba: self.controller.document.features[&input].color_rgba,
                    });
                    self.set_rollback(None);
                } else {
                    self.add_replacing(format!("Transform_{n}"), kind, input, &[input]);
                }
            }
            ToolPanel::CircularPattern {
                input,
                axis,
                center,
                count,
                angle,
            } => self.add_replacing(
                format!("CircularPattern_{n}"),
                FeatureKind::Pattern {
                    input,
                    layout: PatternLayout::Circular {
                        axis_origin: center,
                        axis_dir: AXES[axis].1,
                        count,
                        total_angle_deg: ScalarParam::Literal(angle),
                    },
                    tool_only: self.tool_target(input).is_some(),
                },
                input,
                &[input],
            ),
            ToolPanel::LinearPattern {
                input,
                direction,
                spacing,
                count,
            } => self.add_replacing(
                format!("LinearPattern_{n}"),
                FeatureKind::Pattern {
                    input,
                    layout: PatternLayout::Linear {
                        direction: AXES[direction].1,
                        spacing: ScalarParam::Literal(spacing),
                        count,
                    },
                    tool_only: self.tool_target(input).is_some(),
                },
                input,
                &[input],
            ),
            ToolPanel::Mirror { input, plane, merge } => {
                let includes_original = merge || self.tool_target(input).is_some();
                let replaced: Vec<FeatureId> = if includes_original { vec![input] } else { Vec::new() };
                self.add_replacing(
                    format!("Mirror_{n}"),
                    FeatureKind::Mirror {
                        input,
                        plane: PLANES[plane].1.clone(),
                        merge,
                        tool_only: self.tool_target(input).is_some(),
                    },
                    input,
                    &replaced,
                );
            }
            ToolPanel::Boolean { a, b: Some(b), op } => self.add_replacing(
                format!("Boolean_{n}"),
                FeatureKind::Boolean {
                    op,
                    primary: a,
                    secondary: b,
                },
                a,
                &[a, b],
            ),
            ToolPanel::Boolean { b: None, .. } => {}
            ToolPanel::Shell { input, thickness } => {
                let Some(shape) = self.controller.document.features.get(&input).and_then(|f| f.cached_shape) else {
                    return;
                };
                let faces: Option<Vec<SemanticReference>> = self
                    .open_faces_on(input)
                    .into_iter()
                    .map(|idx| SemanticReference::from_face(input, &self.store, shape, idx).ok())
                    .collect();
                let Some(faces) = faces.filter(|f| !f.is_empty()) else {
                    self.status_logs.push("選んだ面を基準にできませんでした".to_string());
                    return;
                };
                self.add_replacing(
                    format!("Shell_{n}"),
                    FeatureKind::Shell {
                        input,
                        thickness: ScalarParam::Literal(thickness),
                        open_faces: faces,
                    },
                    input,
                    &[input],
                );
            }
            ToolPanel::OffsetPlane { input, face, offset } => {
                self.start_sketch_with(SketchStart::Extrude);
                self.begin_sketch(
                    SketchPlane::OffsetFace {
                        feature: input,
                        face,
                        offset: ScalarParam::Literal(offset),
                    },
                    Some(input),
                );
                self.status_logs.push("オフセット面でスケッチを開始しました".to_string());
            }
            ToolPanel::Draft { input, faces, neutral_index, angle_deg, reversed } => {
                let neutral_face = faces[neutral_index].clone();
                let draft_faces = faces.into_iter().enumerate().filter_map(|(i, f)| (i != neutral_index).then_some(f)).collect();
                self.add_replacing(
                    format!("Draft_{n}"),
                    FeatureKind::Draft { input, faces: draft_faces, neutral_face, angle_deg: ScalarParam::Literal(angle_deg), reversed },
                    input, &[input],
                );
            }
            ToolPanel::Heal { input } => self.add_replacing(
                format!("Heal_{n}"), FeatureKind::Heal { input }, input, &[input],
            ),
        }
    }
}
