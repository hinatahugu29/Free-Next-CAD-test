//! "Select → adjust → confirm" panels for the modify tools on the ribbon: circular and
//! linear patterns, mirror, boolean, and shell.

use super::*;
use cad_core::{BooleanOp, PatternLayout, PatternReferenceLayout, ReferenceAxis};

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
                plane: SketchPlane::XY,
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
            ToolKind::Loft => ToolPanel::Loft { lower: input, upper: None, distance: 20.0 },
            ToolKind::PushPull => {
                let Some((fid, index)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) else { return; };
                ToolPanel::PushPull { input: fid, face, distance: 5.0 }
            }
            ToolKind::SweepCircle => ToolPanel::SweepCircle { radius: 5.0, start: [0.0; 3], path: [0.0, 0.0, 50.0] },
            ToolKind::Counterbore => ToolPanel::Counterbore { input, center: [0.0, 0.0], hole_radius: 3.0, bore_radius: 6.0, bore_depth: 4.0, depth: 20.0 },
            ToolKind::Countersink => ToolPanel::Countersink { input, center: [0.0, 0.0], hole_radius: 3.0, sink_radius: 6.0, sink_depth: 3.0, depth: 20.0 },
            ToolKind::Refine => ToolPanel::Refine { input },
            ToolKind::SolidFromShell => ToolPanel::SolidFromShell { input },
            ToolKind::Sew => ToolPanel::Sew { input, extra_inputs: Vec::new(), tolerance: 0.001 },
            ToolKind::Split => ToolPanel::Split { input, tool: None, plane: Some(SketchPlane::XY) },
            ToolKind::MidPlane => {
                let selected: Vec<_> = self.render_config.selected_faces.iter().copied().take(2).collect();
                if selected.len() != 2 { return; }
                let reference = |(fid, index): (FeatureId, u32)| {
                    let shape = self.controller.document.features.get(&fid)?.cached_shape?;
                    SemanticReference::from_face(fid, &self.store, shape, index).ok().map(|face| (fid, face))
                };
                let (Some((first_feature, first)), Some((second_feature, second))) = (reference(selected[0]), reference(selected[1])) else { return; };
                ToolPanel::MidPlane { first_feature, first, second_feature, second }
            }
            ToolKind::FaceMove => {
                let Some((fid, index)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) else { return; };
                ToolPanel::FaceMove { input: fid, face, displacement: [0.0, 0.0, 10.0] }
            }
            ToolKind::PlanePattern => {
                let Some((fid, index)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) else { return; };
                ToolPanel::PlanePattern { input: fid, base_face: face, spacing: 10.0, count: 3 }
            }
            ToolKind::Hole => {
                let plane = if let Some((fid, index)) = self.render_config.selected_face {
                    if let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) {
                        if let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) {
                            SketchPlane::Face { feature: fid, face }
                        } else {
                            SketchPlane::XY
                        }
                    } else {
                        SketchPlane::XY
                    }
                } else {
                    SketchPlane::XY
                };
                ToolPanel::Hole {
                    input,
                    plane,
                    center: [0.0, 0.0],
                    radius: 3.0,
                    depth: 15.0,
                    through_all: false,
                    hole_type_index: 0,
                    bore_radius: 6.0,
                    bore_depth: 4.0,
                    sink_radius: 6.0,
                    sink_depth: 3.0,
                    thread_radius: 3.0,
                    tap_drill_radius: 2.5,
                    thread_depth: 10.0,
                    has_tip_angle: false,
                    tip_angle_deg: 118.0,
                }
            }
            ToolKind::PatternOnReference => {
                let reference = if let Some((fid, index)) = self.render_config.selected_edge {
                    if let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) {
                        SemanticReference::from_edge(fid, &self.store, shape, index)
                            .map(|edge| ReferenceAxis::Edge { feature: fid, edge })
                            .unwrap_or(ReferenceAxis::Plane(SketchPlane::XY))
                    } else {
                        ReferenceAxis::Plane(SketchPlane::XY)
                    }
                } else if let Some((fid, index)) = self.render_config.selected_face {
                    if let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) {
                        SemanticReference::from_face(fid, &self.store, shape, index)
                            .map(|face| ReferenceAxis::Face { feature: fid, face })
                            .unwrap_or(ReferenceAxis::Plane(SketchPlane::XY))
                    } else {
                        ReferenceAxis::Plane(SketchPlane::XY)
                    }
                } else {
                    ReferenceAxis::Plane(SketchPlane::XY)
                };
                let is_circular = matches!(reference, ReferenceAxis::Face { .. });
                let is_tool = self.tool_target(input).is_some()
                    || matches!(
                        self.controller.document.features.get(&input).map(|f| &f.kind),
                        Some(FeatureKind::Hole { .. })
                    );
                ToolPanel::PatternOnReference {
                    input,
                    reference,
                    is_circular,
                    spacing: 20.0,
                    count: 4,
                    total_angle_deg: 360.0,
                    tool_only: is_tool,
                }
            }
            ToolKind::SurfaceOffset => {
                let Some((fid, index)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) else { return; };
                ToolPanel::SurfaceOffset { input: fid, face, distance: 2.0 }
            }
            ToolKind::TrimSurface => {
                let Some((fid, index)) = self.render_config.selected_face else { return; };
                let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape) else { return; };
                let Ok(face) = SemanticReference::from_face(fid, &self.store, shape, index) else { return; };
                ToolPanel::TrimSurface { input: fid, face, u_min: 0.0, u_max: 1.0, v_min: 0.0, v_max: 1.0 }
            }
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
            ToolPanel::Loft { .. } => "⌁ ロフト",
            ToolPanel::PushPull { .. } => "↕ 面を押出し・引込み",
            ToolPanel::SweepCircle { .. } => "⌒ Sweep（円形・直線）",
            ToolPanel::Counterbore { .. } => "⊙ 座ぐり穴",
            ToolPanel::Countersink { .. } => "▽ 皿穴",
            ToolPanel::Refine { .. } => "◇ 分割を整理",
            ToolPanel::SolidFromShell { .. } => "⬡ 閉じたシェルをソリッド化",
            ToolPanel::Sew { .. } => "⌁ Sew（面・シェルを縫合）",
            ToolPanel::Split { .. } => "✂ 分割（Partition）",
            ToolPanel::MidPlane { .. } => "≡ 中間面からスケッチ",
            ToolPanel::FaceMove { .. } => "↗ 面移動（Face Move）",
            ToolPanel::PlanePattern { .. } => "▤ 基準平面配列（Plane Pattern）",
            ToolPanel::Hole { .. } => "◎ 穴仕様（Hole）",
            ToolPanel::PatternOnReference { .. } => "☍ 参照軸配列（PatternOnReference）",
            ToolPanel::SurfaceOffset { .. } => "⇪ 面オフセット（Surface Offset）",
            ToolPanel::TrimSurface { .. } => "✂ 面トリム（Trim Surface）",
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
                            for (label, candidate) in PLANES.iter() {
                                if ui.selectable_label(*plane == *candidate, *label).clicked() {
                                    *plane = candidate.clone();
                                }
                            }
                            if let Some((fid, face_index)) = self.render_config.selected_face
                                && let Some(shape) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape)
                                && let Ok(reference) = SemanticReference::from_face(fid, &self.store, shape, face_index)
                            {
                                let selected = matches!(plane, SketchPlane::Face { feature, .. } if *feature == fid);
                                if ui.selectable_label(selected, "選択面").clicked() {
                                    *plane = SketchPlane::Face { feature: fid, face: reference };
                                }
                            }
                        });
                        if matches!(plane, SketchPlane::Face { .. }) {
                            ui.small("3Dビューで別の平面をクリックしてから「選択面」を押すと、鏡面を変更できます。");
                        } else {
                            ui.small("基準平面のほか、3Dビューで選んだ平面の面も鏡面にできます。");
                        }
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
                    ToolPanel::Loft { lower, upper, distance } => {
                        ui.label("下側断面と上側断面を選びます（XY平面の押し出しスケッチのみ）");
                        ui.label(format!("下側: {}", self.feature_name(*lower)));
                        ui.horizontal(|ui| {
                            ui.label("上側:");
                            let selected = upper.map(|id| self.feature_name(id)).unwrap_or_else(|| "（選択）".to_string());
                            egui::ComboBox::from_id_salt("loft_upper").selected_text(selected).show_ui(ui, |ui| {
                                for (id, name) in &names { if *id != *lower { ui.selectable_value(upper, Some(*id), name); } }
                            });
                        });
                        ui.horizontal(|ui| { ui.label("断面間距離:"); ui.add(egui::DragValue::new(distance).speed(0.5).suffix(" mm")); });
                    }
                    ToolPanel::PushPull { input, distance, .. } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| { ui.label("距離:"); ui.add(egui::DragValue::new(distance).speed(0.5).suffix(" mm")); });
                        ui.small("プラスは法線方向へ追加、マイナスは引き込みます。平面面のみ対応です。");
                    }
                    ToolPanel::FaceMove { input, displacement, .. } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("移動ベクトル XYZ:");
                            ui.add(egui::DragValue::new(&mut displacement[0]).speed(0.5).prefix("X: ").suffix(" mm"));
                            ui.add(egui::DragValue::new(&mut displacement[1]).speed(0.5).prefix("Y: ").suffix(" mm"));
                            ui.add(egui::DragValue::new(&mut displacement[2]).speed(0.5).prefix("Z: ").suffix(" mm"));
                        });
                        ui.small("法線方向の制約なしに、選択した平面を任意ベクトルへ平行移動します。");
                    }
                    ToolPanel::PlanePattern { input, spacing, count, .. } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("間隔:");
                            ui.add(egui::DragValue::new(spacing).speed(0.5).suffix(" mm"));
                        });
                        ui.horizontal(|ui| {
                            ui.label("個数:");
                            ui.add(egui::DragValue::new(count).range(1..=100));
                        });
                        ui.small("基準面から指定間隔で平行な基準平面配列を生成します。");
                    }
                    ToolPanel::SweepCircle { radius, start, path } => {
                        ui.horizontal(|ui| { ui.label("半径:"); ui.add(egui::DragValue::new(radius).range(0.01..=10000.0).suffix(" mm")); });
                        ui.horizontal(|ui| { ui.label("開始 XYZ:"); for value in start { ui.add(egui::DragValue::new(value).speed(0.5).suffix(" mm")); } });
                        ui.horizontal(|ui| {
                            ui.label("パス XYZ:");
                            for value in path { ui.add(egui::DragValue::new(value).speed(0.5).suffix(" mm")); }
                        });
                        ui.small("原点の円形断面を、指定した一本の直線パスへ沿って掃引します。");
                    }
                    ToolPanel::Counterbore { input, center, hole_radius, bore_radius, bore_depth, depth } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| { ui.label("中心 X / Y:"); for value in center { ui.add(egui::DragValue::new(value).speed(0.5).suffix(" mm")); } });
                        ui.horizontal(|ui| { ui.label("穴半径:"); ui.add(egui::DragValue::new(hole_radius).range(0.01..=1000.0).suffix(" mm")); });
                        ui.horizontal(|ui| { ui.label("座ぐり半径:"); ui.add(egui::DragValue::new(bore_radius).range(0.01..=1000.0).suffix(" mm")); });
                        ui.horizontal(|ui| { ui.label("座ぐり深さ:"); ui.add(egui::DragValue::new(bore_depth).range(0.01..=1000.0).suffix(" mm")); ui.label("全深さ:"); ui.add(egui::DragValue::new(depth).range(0.01..=1000.0).suffix(" mm")); });
                        ui.small("指定したXY位置から +Z へ穴を作ります。各値は作成後にパラメータ化できます。");
                    }
                    ToolPanel::Countersink { input, center, hole_radius, sink_radius, sink_depth, depth } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| { ui.label("中心 X / Y:"); for value in center { ui.add(egui::DragValue::new(value).speed(0.5).suffix(" mm")); } });
                        ui.horizontal(|ui| { ui.label("穴半径:"); ui.add(egui::DragValue::new(hole_radius).range(0.01..=1000.0).suffix(" mm")); });
                        ui.horizontal(|ui| { ui.label("皿穴半径:"); ui.add(egui::DragValue::new(sink_radius).range(0.01..=1000.0).suffix(" mm")); });
                        ui.horizontal(|ui| { ui.label("皿深さ:"); ui.add(egui::DragValue::new(sink_depth).range(0.01..=1000.0).suffix(" mm")); ui.label("全深さ:"); ui.add(egui::DragValue::new(depth).range(0.01..=1000.0).suffix(" mm")); });
                        ui.small("指定したXY位置から +Z へ穴を作ります。各値は作成後にパラメータ化できます。");
                    }
                    ToolPanel::Refine { input } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.small("同一平面・同一曲面に分かれた不要な面・辺を統合します。");
                    }
                    ToolPanel::SolidFromShell { input } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.small("閉じたシェルだけをソリッドへ変換します。開いたシェルはSewで閉じてから実行してください。");
                    }
                    ToolPanel::Sew { input, extra_inputs, tolerance } => {
                        ui.label(format!("対象（主形状）: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| { ui.label("許容差:"); ui.add(egui::DragValue::new(tolerance).range(0.000_001..=1.0).speed(0.0001).suffix(" mm")); });
                        ui.label("追加で縫合する形状:");
                        for (id, name) in &names {
                            if *id != *input {
                                let mut checked = extra_inputs.contains(id);
                                if ui.checkbox(&mut checked, name).changed() {
                                    if checked {
                                        extra_inputs.push(*id);
                                    } else {
                                        extra_inputs.retain(|&x| x != *id);
                                    }
                                }
                            }
                        }
                        ui.small("許容差内で一致する面境界を複数形状間で一括縫合します。");
                    }
                    ToolPanel::Split { input, tool, plane } => {
                        ui.label(format!("分割される形状: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            let use_plane = plane.is_some();
                            if ui.radio(!use_plane, "工具ボディ").clicked() {
                                *plane = None;
                            }
                            if ui.radio(use_plane, "無限平面").clicked() {
                                *tool = None;
                                *plane = Some(SketchPlane::XY);
                            }
                        });
                        if let Some(p) = plane {
                            ui.horizontal(|ui| {
                                ui.label("基準平面:");
                                let mut sel = match p {
                                    SketchPlane::XY => 0,
                                    SketchPlane::XZ => 1,
                                    SketchPlane::YZ => 2,
                                    _ => 0,
                                };
                                egui::ComboBox::from_id_salt("split_plane_combo")
                                    .selected_text(match sel { 0 => "XY平面", 1 => "XZ平面", 2 => "YZ平面", _ => "カスタム" })
                                    .show_ui(ui, |ui| {
                                        if ui.selectable_value(&mut sel, 0, "XY平面").clicked() { *p = SketchPlane::XY; }
                                        if ui.selectable_value(&mut sel, 1, "XZ平面").clicked() { *p = SketchPlane::XZ; }
                                        if ui.selectable_value(&mut sel, 2, "YZ平面").clicked() { *p = SketchPlane::YZ; }
                                    });
                            });
                        } else {
                            egui::ComboBox::from_id_salt("split_tool").selected_text(tool.map(|id| self.feature_name(id)).unwrap_or_else(|| "（分割用ボディを選択）".to_string())).show_ui(ui, |ui| {
                                for (id, name) in &names { if *id != *input { ui.selectable_value(tool, Some(*id), name); } }
                            });
                        }
                        ui.small("工具ボディまたは基準平面で対象形状を分割します。");
                    }
                    ToolPanel::SurfaceOffset { input, distance, .. } => {
                        ui.label(format!("対象: 「{}」の選択面", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("オフセット距離:");
                            ui.add(egui::DragValue::new(distance).speed(0.1).suffix(" mm"));
                        });
                        ui.small("面の法線方向に沿ってオフセット曲面を生成します。");
                    }
                    ToolPanel::TrimSurface { input, u_min, u_max, v_min, v_max, .. } => {
                        ui.label(format!("対象: 「{}」の選択面", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("U範囲:");
                            ui.add(egui::DragValue::new(u_min).range(0.0..=*u_max).speed(0.01).prefix("min: "));
                            ui.add(egui::DragValue::new(u_max).range(*u_min..=1.0).speed(0.01).prefix("max: "));
                        });
                        ui.horizontal(|ui| {
                            ui.label("V範囲:");
                            ui.add(egui::DragValue::new(v_min).range(0.0..=*v_max).speed(0.01).prefix("min: "));
                            ui.add(egui::DragValue::new(v_max).range(*v_min..=1.0).speed(0.01).prefix("max: "));
                        });
                        ui.small("選択面のUVパラメータ境界をトリミングしたサーフェスを生成します。");
                    }
                    ToolPanel::MidPlane { first_feature, second_feature, .. } => {
                        ui.label(format!("面1: 「{}」", self.feature_name(*first_feature)));
                        ui.label(format!("面2: 「{}」", self.feature_name(*second_feature)));
                        ui.small("平行な二面のちょうど中間でスケッチを開始します。");
                    }
                    ToolPanel::Hole {
                        input,
                        plane: _,
                        center,
                        radius,
                        depth,
                        through_all,
                        hole_type_index,
                        bore_radius,
                        bore_depth,
                        sink_radius,
                        sink_depth,
                        thread_radius,
                        tap_drill_radius,
                        thread_depth,
                        has_tip_angle,
                        tip_angle_deg,
                    } => {
                        ui.label(format!("対象: 「{}」", self.feature_name(*input)));
                        ui.horizontal(|ui| {
                            ui.label("種類:");
                            egui::ComboBox::from_id_salt("hole_type_combo")
                                .selected_text(match *hole_type_index {
                                    1 => "座ぐり穴 (Counterbore)",
                                    2 => "皿穴 (Countersink)",
                                    3 => "ねじ下穴 (Tap)",
                                    _ => "単純穴 (Simple)",
                                })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(hole_type_index, 0, "単純穴 (Simple)");
                                    ui.selectable_value(hole_type_index, 1, "座ぐり穴 (Counterbore)");
                                    ui.selectable_value(hole_type_index, 2, "皿穴 (Countersink)");
                                    ui.selectable_value(hole_type_index, 3, "ねじ下穴 (Tap)");
                                });
                        });
                        ui.horizontal(|ui| {
                            ui.label("中心 (U, V):");
                            ui.add(egui::DragValue::new(&mut center[0]).speed(0.5).prefix("U: ").suffix(" mm"));
                            ui.add(egui::DragValue::new(&mut center[1]).speed(0.5).prefix("V: ").suffix(" mm"));
                        });
                        ui.horizontal(|ui| {
                            ui.label("穴半径:");
                            ui.add(egui::DragValue::new(radius).speed(0.1).range(0.1..=1000.0).suffix(" mm"));
                        });
                        ui.horizontal(|ui| {
                            ui.checkbox(through_all, "貫通 (Through All)");
                            if !*through_all {
                                ui.label("深さ:");
                                ui.add(egui::DragValue::new(depth).speed(0.5).range(0.1..=1000.0).suffix(" mm"));
                            }
                        });
                        match *hole_type_index {
                            1 => {
                                ui.horizontal(|ui| {
                                    ui.label("座ぐり半径:");
                                    ui.add(egui::DragValue::new(bore_radius).speed(0.1).range(0.1..=1000.0).suffix(" mm"));
                                    ui.label("深さ:");
                                    ui.add(egui::DragValue::new(bore_depth).speed(0.5).range(0.1..=1000.0).suffix(" mm"));
                                });
                            }
                            2 => {
                                ui.horizontal(|ui| {
                                    ui.label("皿穴半径:");
                                    ui.add(egui::DragValue::new(sink_radius).speed(0.1).range(0.1..=1000.0).suffix(" mm"));
                                    ui.label("深さ:");
                                    ui.add(egui::DragValue::new(sink_depth).speed(0.5).range(0.1..=1000.0).suffix(" mm"));
                                });
                            }
                            3 => {
                                ui.horizontal(|ui| {
                                    ui.label("ねじ呼び半径:");
                                    ui.add(egui::DragValue::new(thread_radius).speed(0.1).range(0.1..=1000.0).suffix(" mm"));
                                    ui.label("下穴半径:");
                                    ui.add(egui::DragValue::new(tap_drill_radius).speed(0.1).range(0.1..=1000.0).suffix(" mm"));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("ねじ深さ:");
                                    ui.add(egui::DragValue::new(thread_depth).speed(0.5).range(0.1..=1000.0).suffix(" mm"));
                                });
                            }
                            _ => {}
                        }
                        if !*through_all {
                            ui.horizontal(|ui| {
                                ui.checkbox(has_tip_angle, "ドリル先端角");
                                if *has_tip_angle {
                                    ui.add(egui::DragValue::new(tip_angle_deg).speed(1.0).range(30.0..=160.0).suffix("°"));
                                }
                            });
                        }
                        ui.small("選択面の法線方向へ穴を切り取ります。各寸法は後からパラメータ編集可能です。");
                    }
                    ToolPanel::PatternOnReference {
                        input,
                        reference,
                        is_circular,
                        spacing,
                        count,
                        total_angle_deg,
                        tool_only,
                    } => {
                        self.describe_pattern_input(ui, *input);
                        ui.horizontal(|ui| {
                            ui.label("参照軸:");
                            let ref_label = match reference {
                                ReferenceAxis::Edge { .. } => "選択エッジ（直線/円弧）",
                                ReferenceAxis::Face { .. } => "選択面（円筒面/法線）",
                                ReferenceAxis::Plane(p) => match p {
                                    SketchPlane::XY => "XY平面法線 (Z軸)",
                                    SketchPlane::XZ => "XZ平面法線 (Y軸)",
                                    SketchPlane::YZ => "YZ平面法線 (X軸)",
                                    _ => "平面法線",
                                },
                            };
                            ui.monospace(ref_label);
                        });
                        ui.horizontal(|ui| {
                            ui.label("配列形式:");
                            ui.radio_value(is_circular, false, "直線");
                            ui.radio_value(is_circular, true, "円形 (回転)");
                        });
                        if *is_circular {
                            ui.horizontal(|ui| {
                                ui.label("個数:");
                                ui.add(egui::DragValue::new(count).range(1..=360));
                                ui.label("合計角度:");
                                ui.add(egui::DragValue::new(total_angle_deg).range(-3600.0..=3600.0).suffix("°"));
                            });
                        } else {
                            ui.horizontal(|ui| {
                                ui.label("個数:");
                                ui.add(egui::DragValue::new(count).range(1..=1000));
                                ui.label("間隔:");
                                ui.add(egui::DragValue::new(spacing).speed(0.5).suffix(" mm"));
                            });
                        }
                        ui.checkbox(tool_only, "ツールのみ複製（穴・ポケット等を配列）");
                        ui.small("選択したエッジの方向または円筒面の回転軸に沿ってパラメトリックに配列します。");
                    }
                }

                ui.separator();
                let ready = match &panel {
                    ToolPanel::Boolean { b, .. } => b.is_some(),
                    ToolPanel::Shell { input, .. } => !self.open_faces_on(*input).is_empty(),
                    ToolPanel::Loft { upper, .. } => upper.is_some(),
                    ToolPanel::Split { tool, plane, .. } => tool.is_some() || plane.is_some(),
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
                        plane,
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
            ToolPanel::Loft { lower, upper: Some(upper), distance } => {
                let sketches = |id| match &self.controller.document.features.get(&id)?.kind {
                    FeatureKind::SketchExtrude { sketch, plane: SketchPlane::XY, .. } => Some(sketch.clone()),
                    _ => None,
                };
                let (Some(lower), Some(upper)) = (sketches(lower), sketches(upper)) else {
                    self.status_logs.push("ロフトにはXY平面上の押し出しスケッチを2つ選んでください".to_string());
                    return;
                };
                self.execute_cmd(CadCommand::AddFeature { name: format!("Loft_{n}"), kind: FeatureKind::Loft { lower, upper, distance: ScalarParam::Literal(distance) }, visible: true, color_rgba: [0.65, 0.55, 0.9, 1.0] });
            }
            ToolPanel::Loft { upper: None, .. } => {}
            ToolPanel::PushPull { input, face, distance } => self.add_replacing(
                format!("PushPull_{n}"), FeatureKind::PushPull { input, face, distance: ScalarParam::Literal(distance) }, input, &[input],
            ),
            ToolPanel::SweepCircle { radius, start, path } => self.execute_cmd(CadCommand::AddFeature {
                name: format!("Sweep_{n}"),
                kind: FeatureKind::SweepCircle { radius: ScalarParam::Literal(radius), start_x: ScalarParam::Literal(start[0]), start_y: ScalarParam::Literal(start[1]), start_z: ScalarParam::Literal(start[2]), dx: ScalarParam::Literal(path[0]), dy: ScalarParam::Literal(path[1]), dz: ScalarParam::Literal(path[2]) },
                visible: true, color_rgba: [0.5, 0.75, 0.9, 1.0],
            }),
            ToolPanel::Counterbore { input, center, hole_radius, bore_radius, bore_depth, depth } => self.add_replacing(
                format!("Counterbore_{n}"), FeatureKind::Counterbore { input, center_x: ScalarParam::Literal(center[0]), center_y: ScalarParam::Literal(center[1]), hole_radius: ScalarParam::Literal(hole_radius), bore_radius: ScalarParam::Literal(bore_radius), bore_depth: ScalarParam::Literal(bore_depth), depth: ScalarParam::Literal(depth) }, input, &[input],
            ),
            ToolPanel::Countersink { input, center, hole_radius, sink_radius, sink_depth, depth } => self.add_replacing(
                format!("Countersink_{n}"), FeatureKind::Countersink { input, center_x: ScalarParam::Literal(center[0]), center_y: ScalarParam::Literal(center[1]), hole_radius: ScalarParam::Literal(hole_radius), sink_radius: ScalarParam::Literal(sink_radius), sink_depth: ScalarParam::Literal(sink_depth), depth: ScalarParam::Literal(depth) }, input, &[input],
            ),
            ToolPanel::Refine { input } => self.add_replacing(
                format!("Refine_{n}"), FeatureKind::Refine { input }, input, &[input],
            ),
            ToolPanel::SolidFromShell { input } => self.add_replacing(
                format!("SolidFromShell_{n}"), FeatureKind::SolidFromShell { input }, input, &[input],
            ),
            ToolPanel::Sew { input, extra_inputs, tolerance } => {
                let mut hides = vec![input];
                hides.extend_from_slice(&extra_inputs);
                self.add_replacing(
                    format!("Sew_{n}"),
                    FeatureKind::Sew { input, extra_inputs, tolerance: ScalarParam::Literal(tolerance) },
                    input,
                    &hides,
                )
            }
            ToolPanel::Split { input, tool, plane } => {
                if let Some(t) = tool {
                    self.add_replacing(
                        format!("Split_{n}"),
                        FeatureKind::Split { input, tool: Some(t), plane: None },
                        input,
                        &[input, t],
                    )
                } else if let Some(p) = plane {
                    self.add_replacing(
                        format!("Split_{n}"),
                        FeatureKind::Split { input, tool: None, plane: Some(p) },
                        input,
                        &[input],
                    )
                }
            }
            ToolPanel::SurfaceOffset { input, face, distance } => self.add_replacing(
                format!("SurfaceOffset_{n}"),
                FeatureKind::SurfaceOffset { input, face, distance: ScalarParam::Literal(distance) },
                input,
                &[input],
            ),
            ToolPanel::TrimSurface { input, face, u_min, u_max, v_min, v_max } => self.add_replacing(
                format!("TrimSurface_{n}"),
                FeatureKind::TrimSurface {
                    input,
                    face,
                    u_min: ScalarParam::Literal(u_min),
                    u_max: ScalarParam::Literal(u_max),
                    v_min: ScalarParam::Literal(v_min),
                    v_max: ScalarParam::Literal(v_max),
                },
                input,
                &[input],
            ),
            ToolPanel::MidPlane { first_feature, first, second_feature, second } => {
                self.start_sketch_with(SketchStart::Extrude);
                self.begin_sketch(SketchPlane::MidFace { first_feature, first, second_feature, second }, None);
            }
            ToolPanel::FaceMove { input, face, displacement } => self.add_replacing(
                format!("FaceMove_{n}"), FeatureKind::FaceMove { input, face, dx: ScalarParam::Literal(displacement[0]), dy: ScalarParam::Literal(displacement[1]), dz: ScalarParam::Literal(displacement[2]) }, input, &[input],
            ),
            ToolPanel::PlanePattern { input, base_face, spacing, count } => self.add_replacing(
                format!("PlanePattern_{n}"), FeatureKind::PlanePattern { input, base_face, spacing: ScalarParam::Literal(spacing), count, reverse: false }, input, &[input],
            ),
            ToolPanel::Hole {
                input,
                plane,
                center,
                radius,
                depth,
                through_all,
                hole_type_index,
                bore_radius,
                bore_depth,
                sink_radius,
                sink_depth,
                thread_radius,
                tap_drill_radius,
                thread_depth,
                has_tip_angle,
                tip_angle_deg,
            } => {
                let hole_type = match hole_type_index {
                    1 => cad_core::HoleType::Counterbore {
                        bore_radius: ScalarParam::Literal(bore_radius),
                        bore_depth: ScalarParam::Literal(bore_depth),
                    },
                    2 => cad_core::HoleType::Countersink {
                        sink_radius: ScalarParam::Literal(sink_radius),
                        sink_depth: ScalarParam::Literal(sink_depth),
                    },
                    3 => cad_core::HoleType::Tap {
                        thread_radius: ScalarParam::Literal(thread_radius),
                        tap_drill_radius: ScalarParam::Literal(tap_drill_radius),
                        thread_depth: ScalarParam::Literal(thread_depth),
                    },
                    _ => cad_core::HoleType::Simple,
                };
                let tip = if has_tip_angle && !through_all {
                    Some(ScalarParam::Literal(tip_angle_deg))
                } else {
                    None
                };
                let mut deps = vec![input];
                for id in plane.features() {
                    if !deps.contains(&id) {
                        deps.push(id);
                    }
                }
                self.add_replacing(
                    format!("Hole_{n}"),
                    FeatureKind::Hole {
                        input,
                        plane,
                        center_x: ScalarParam::Literal(center[0]),
                        center_y: ScalarParam::Literal(center[1]),
                        radius: ScalarParam::Literal(radius),
                        depth: ScalarParam::Literal(depth),
                        through_all,
                        hole_type,
                        tip_angle_deg: tip,
                    },
                    input,
                    &deps,
                );
            }
            ToolPanel::PatternOnReference {
                input,
                reference,
                is_circular,
                spacing,
                count,
                total_angle_deg,
                tool_only,
            } => {
                let layout = if is_circular {
                    PatternReferenceLayout::Circular {
                        count,
                        total_angle_deg: ScalarParam::Literal(total_angle_deg),
                    }
                } else {
                    PatternReferenceLayout::Linear {
                        spacing: ScalarParam::Literal(spacing),
                        count,
                    }
                };
                let mut deps = vec![input];
                match &reference {
                    ReferenceAxis::Edge { feature, .. } | ReferenceAxis::Face { feature, .. } => {
                        if !deps.contains(feature) {
                            deps.push(*feature);
                        }
                    }
                    ReferenceAxis::Plane(p) => {
                        for id in p.features() {
                            if !deps.contains(&id) {
                                deps.push(id);
                            }
                        }
                    }
                }
                self.add_replacing(
                    format!("PatternOnRef_{n}"),
                    FeatureKind::PatternOnReference {
                        input,
                        reference,
                        layout,
                        tool_only,
                    },
                    input,
                    &deps,
                );
            }
        }
    }
}
