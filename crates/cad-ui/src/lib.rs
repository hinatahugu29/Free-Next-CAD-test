//! Desktop CAD GUI for FerroCAD powered by `egui` / `eframe` and `cad-render` (Phase 3 & Phase 4).

use std::path::PathBuf;

use cad_core::{
    CadCommand, Document, DocumentController, FeatureId, FeatureKind, FeatureState, ParameterValue,
    ScalarParam, SemanticReference, Sketch, build_perforated_plate_mvp,
};
use cad_geometry::{ShapeStore, TessellationOptions};
use cad_render::{
    CadRenderer, Camera, DisplayMode, ProjectionMode, StandardView, ViewportRenderConfig,
    pick_document,
};
use egui::{
    Color32, ColorImage, Context, Key, Modifiers, Pos2, Rect, RichText, Sense, TextureHandle,
    TextureOptions, Vec2,
};

/// Installs a Japanese-capable font so UI text (feature names, status logs, parameter
/// labels — all Japanese in this build) renders correctly instead of as tofu boxes.
/// `egui`'s bundled default font has no CJK glyphs, so we load a system font file at
/// startup and register it as a fallback behind the default proportional/monospace fonts.
pub fn install_japanese_font(ctx: &egui::Context) {
    const CANDIDATES: &[&str] = &[
        r"C:\Windows\Fonts\NotoSansJP-Regular.ttf",
        r"C:\Windows\Fonts\meiryo.ttc",
        r"C:\Windows\Fonts\YuGothR.ttc",
        r"C:\Windows\Fonts\msgothic.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc",
    ];

    let Some(bytes) = CANDIDATES
        .iter()
        .find_map(|path| std::fs::read(path).ok())
    else {
        return;
    };

    let mut fonts = egui::FontDefinitions::default();
    fonts
        .font_data
        .insert("jp_font".to_owned(), egui::FontData::from_owned(bytes).into());

    fonts
        .families
        .entry(egui::FontFamily::Proportional)
        .or_default()
        .push("jp_font".to_owned());
    fonts
        .families
        .entry(egui::FontFamily::Monospace)
        .or_default()
        .push("jp_font".to_owned());

    ctx.set_fonts(fonts);
}

/// Main GUI application state for FerroCAD.
pub struct FerroCadApp {
    pub controller: DocumentController,
    pub store: ShapeStore,
    pub tess_options: TessellationOptions,
    pub renderer: Option<CadRenderer>,
    pub camera: Camera,
    pub render_config: ViewportRenderConfig,
    pub viewport_texture: Option<TextureHandle>,
    pub last_viewport_size: [u32; 2],
    pub viewport_dirty: bool,
    pub show_command_palette: bool,
    pub command_filter: String,
    pub status_logs: Vec<String>,
    pub new_param_name: String,
    pub new_param_expr: String,
    pub current_file_path: Option<PathBuf>,
    /// Screen position where the floating on-canvas property editor should appear
    /// (set by double-clicking a face/edge directly in the 3D viewport).
    pub floating_editor_pos: Option<Pos2>,
    /// When true, the central viewport switches from the 3D render to a flat 2D sketch
    /// canvas: clicking places profile points (in millimeters on the sketch plane),
    /// which are later closed into a loop and extruded via `cad_core`'s `Sketch` solver.
    pub sketch_mode: bool,
    pub sketch_points: Vec<[f64; 2]>,
    pub sketch_depth: f64,
}

impl Default for FerroCadApp {
    fn default() -> Self {
        Self::new_with_mvp_demo()
    }
}

impl FerroCadApp {
    /// Initializes the CAD application preloaded with the Milestone M1 Perforated Plate MVP model.
    pub fn new_with_mvp_demo() -> Self {
        let mut store = ShapeStore::new();
        let tess_options = TessellationOptions::default();
        let (mut controller, final_id) = build_perforated_plate_mvp().unwrap_or_else(|_| {
            (
                DocumentController::new(Document::new("Untitled")),
                FeatureId(1),
            )
        });

        let mut logs = Vec::new();
        match controller.document.recompute(&mut store, tess_options) {
            Ok(report) => {
                logs.push(format!(
                    "初期モデル『{}』を計算完了（再計算: {}件, キャッシュ再利用: {}件）",
                    controller.document.name,
                    report.recomputed_features.len(),
                    report.reused_cached_features.len()
                ));
            }
            Err(err) => {
                logs.push(format!("初期計算エラー: {err}"));
            }
        }

        let mut camera = Camera::default();
        if let Some(bbox) = controller.document.visible_bounding_box() {
            camera.fit_bounding_box(&bbox);
        }

        let renderer = CadRenderer::new_headless().ok();
        let render_config = ViewportRenderConfig {
            display_mode: DisplayMode::ShadedWithEdges,
            show_grid: true,
            selected_feature: Some(final_id),
            selected_face: None,
            selected_edge: None,
        };

        Self {
            controller,
            store,
            tess_options,
            renderer,
            camera,
            render_config,
            viewport_texture: None,
            last_viewport_size: [0, 0],
            viewport_dirty: true,
            show_command_palette: false,
            command_filter: String::new(),
            status_logs: logs,
            new_param_name: String::new(),
            new_param_expr: String::new(),
            current_file_path: None,
            floating_editor_pos: None,
            sketch_mode: false,
            sketch_points: Vec::new(),
            sketch_depth: 10.0,
        }
    }

    pub fn recompute_and_refresh(&mut self) {
        match self
            .controller
            .document
            .recompute(&mut self.store, self.tess_options)
        {
            Ok(report) => {
                if !report.failed_features.is_empty() {
                    for (fid, msg) in &report.failed_features {
                        self.status_logs
                            .push(format!("フィーチャー {:?} 計算失敗: {}", fid, msg));
                    }
                } else {
                    self.status_logs.push(format!(
                        "再計算完了: 更新 {}件 / キャッシュ再利用 {}件",
                        report.recomputed_features.len(),
                        report.reused_cached_features.len()
                    ));
                }
            }
            Err(err) => {
                self.status_logs.push(format!("DAG再計算エラー: {err}"));
            }
        }
        self.viewport_dirty = true;
    }

    pub fn execute_cmd(&mut self, cmd: CadCommand) {
        match self.controller.execute(cmd, Some(&mut self.store)) {
            Ok(maybe_fid) => {
                if let Some(fid) = maybe_fid {
                    self.render_config.selected_feature = Some(fid);
                }
                self.recompute_and_refresh();
            }
            Err(err) => {
                self.status_logs.push(format!("コマンド実行エラー: {err}"));
            }
        }
    }

    fn handle_shortcuts(&mut self, ctx: &Context) {
        if ctx.input_mut(|i| i.consume_key(Modifiers::CTRL, Key::K)) {
            self.show_command_palette = !self.show_command_palette;
        }
        if ctx.input_mut(|i| i.consume_key(Modifiers::CTRL, Key::Z))
            && self.controller.undo(&mut self.store).is_ok()
        {
            self.status_logs.push("Undo を実行しました".to_string());
            self.recompute_and_refresh();
        }
        if ctx.input_mut(|i| i.consume_key(Modifiers::CTRL, Key::Y))
            && self.controller.redo(&mut self.store).is_ok()
        {
            self.status_logs.push("Redo を実行しました".to_string());
            self.recompute_and_refresh();
        }
        if !ctx.wants_keyboard_input()
            && ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::F))
            && let Some(bbox) = self.controller.document.visible_bounding_box()
        {
            self.camera.fit_bounding_box(&bbox);
            self.viewport_dirty = true;
        }
    }

    fn render_top_menubar(&mut self, ui: &mut egui::Ui) {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("ファイル (File)", |ui| {
                if ui.button("新規空ドキュメント (New)").clicked() {
                    self.store.clear();
                    self.controller = DocumentController::new(Document::new("新規モデル"));
                    self.render_config.selected_feature = None;
                    self.render_config.selected_face = None;
                    self.render_config.selected_edge = None;
                    self.viewport_dirty = true;
                    ui.close_menu();
                }
                if ui
                    .button("Milestone M1 デモ（穴あきプレート）を再読込")
                    .clicked()
                {
                    *self = Self::new_with_mvp_demo();
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("設計JSONを開く (Open JSON)...").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("FerroCAD JSON", &["json", "fcad"])
                        .pick_file()
                    {
                        match Document::load_json(&path) {
                            Ok(doc) => {
                                self.store.clear();
                                self.controller = DocumentController::new(doc);
                                self.current_file_path = Some(path.clone());
                                self.recompute_and_refresh();
                                if let Some(bbox) = self.controller.document.visible_bounding_box()
                                {
                                    self.camera.fit_bounding_box(&bbox);
                                }
                                self.status_logs
                                    .push(format!("JSON読込完了: {}", path.display()));
                            }
                            Err(e) => self.status_logs.push(format!("読込エラー: {e}")),
                        }
                    }
                    ui.close_menu();
                }
                if ui.button("設計JSONを保存 (Save JSON)...").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("FerroCAD JSON", &["json"])
                        .set_file_name("model.json")
                        .save_file()
                    {
                        match self.controller.document.save_json(&path) {
                            Ok(()) => {
                                self.current_file_path = Some(path.clone());
                                self.status_logs
                                    .push(format!("JSON保存完了: {}", path.display()));
                            }
                            Err(e) => self.status_logs.push(format!("保存エラー: {e}")),
                        }
                    }
                    ui.close_menu();
                }
                ui.separator();
                if ui
                    .button("選択フィーチャーをSTEP出力 (Export STEP)...")
                    .clicked()
                {
                    if let Some(fid) = self.render_config.selected_feature
                        && let Some(feat) = self.controller.document.features.get(&fid)
                        && let Some(shape_id) = feat.cached_shape
                        && let Some(path) = rfd::FileDialog::new()
                            .add_filter("STEP CAD", &["step", "stp"])
                            .set_file_name(format!("{}.step", feat.name))
                            .save_file()
                    {
                        match self.store.export_step(shape_id, &path) {
                            Ok(()) => self
                                .status_logs
                                .push(format!("STEP出力完了: {}", path.display())),
                            Err(e) => self.status_logs.push(format!("STEP出力エラー: {e}")),
                        }
                    }
                    ui.close_menu();
                }
            });

            ui.menu_button("編集 (Edit)", |ui| {
                if ui
                    .add_enabled(
                        self.controller.can_undo(),
                        egui::Button::new("元に戻す (Undo) [Ctrl+Z]"),
                    )
                    .clicked()
                    && self.controller.undo(&mut self.store).is_ok()
                {
                    self.recompute_and_refresh();
                    ui.close_menu();
                }
                if ui
                    .add_enabled(
                        self.controller.can_redo(),
                        egui::Button::new("やり直す (Redo) [Ctrl+Y]"),
                    )
                    .clicked()
                    && self.controller.redo(&mut self.store).is_ok()
                {
                    self.recompute_and_refresh();
                    ui.close_menu();
                }
            });

            ui.menu_button("フィーチャー追加 (Create)", |ui| {
                if ui.button("直方体 (Box)").clicked() {
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("Box_{}", self.controller.document.features.len() + 1),
                        kind: FeatureKind::Box {
                            width: ScalarParam::mm(40.0),
                            height: ScalarParam::mm(40.0),
                            depth: ScalarParam::mm(20.0),
                        },
                        visible: true,
                        color_rgba: [0.35, 0.65, 0.90, 1.0],
                    });
                    ui.close_menu();
                }
                if ui.button("円柱 (Cylinder)").clicked() {
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("Cylinder_{}", self.controller.document.features.len() + 1),
                        kind: FeatureKind::Cylinder {
                            radius: ScalarParam::mm(15.0),
                            height: ScalarParam::mm(40.0),
                        },
                        visible: true,
                        color_rgba: [0.45, 0.78, 0.55, 1.0],
                    });
                    ui.close_menu();
                }
                if ui.button("球 (Sphere)").clicked() {
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("Sphere_{}", self.controller.document.features.len() + 1),
                        kind: FeatureKind::Sphere {
                            radius: ScalarParam::mm(20.0),
                        },
                        visible: true,
                        color_rgba: [0.85, 0.55, 0.35, 1.0],
                    });
                    ui.close_menu();
                }
                if ui.button("円錐 (Cone)").clicked() {
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("Cone_{}", self.controller.document.features.len() + 1),
                        kind: FeatureKind::Cone {
                            bottom_radius: ScalarParam::mm(20.0),
                            top_radius: ScalarParam::mm(5.0),
                            height: ScalarParam::mm(35.0),
                        },
                        visible: true,
                        color_rgba: [0.75, 0.50, 0.85, 1.0],
                    });
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("✏ スケッチから押し出し... (2D描画)").clicked() {
                    self.sketch_mode = true;
                    self.sketch_points.clear();
                    self.sketch_depth = 10.0;
                    ui.close_menu();
                }
            });

            ui.menu_button("表示 (View)", |ui| {
                if ui.button("全体フィット (Fit All) [F]").clicked() {
                    if let Some(bbox) = self.controller.document.visible_bounding_box() {
                        self.camera.fit_bounding_box(&bbox);
                        self.viewport_dirty = true;
                    }
                    ui.close_menu();
                }
                if ui.button("投影切替 (透視 / 平行)").clicked() {
                    self.camera.toggle_projection();
                    self.viewport_dirty = true;
                    ui.close_menu();
                }
                if ui
                    .checkbox(
                        &mut self.render_config.show_grid,
                        "基準グリッド・座標軸表示",
                    )
                    .changed()
                {
                    self.viewport_dirty = true;
                }
            });

            ui.separator();
            if ui
                .button(RichText::new("⌘ コマンドパレット (Ctrl+K)").strong())
                .clicked()
            {
                self.show_command_palette = !self.show_command_palette;
            }

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(format!(
                    "Rev #{} | Undo: {} | Redo: {}",
                    self.controller.document.revision,
                    self.controller.undo_count(),
                    self.controller.redo_count()
                ));
            });
        });
    }

    fn render_left_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("📐 パラメータ一覧 (Parameters)");
        ui.separator();

        let resolved = self
            .controller
            .document
            .resolve_all_parameters()
            .unwrap_or_default();
        let params_snapshot: Vec<_> = self
            .controller
            .document
            .parameters
            .values()
            .cloned()
            .collect();

        let mut pending_param_cmd: Option<CadCommand> = None;

        egui::Grid::new("param_grid")
            .striped(true)
            .num_columns(3)
            .show(ui, |ui| {
                ui.strong("名前");
                ui.strong("値 / 数式");
                ui.strong("評価値");
                ui.end_row();

                for p in params_snapshot {
                    ui.label(&p.name).on_hover_text(&p.description);
                    match p.value {
                        ParameterValue::Number(mut val) => {
                            if ui
                                .add(
                                    egui::DragValue::new(&mut val)
                                        .speed(0.5)
                                        .suffix(format!(" {}", p.unit)),
                                )
                                .changed()
                            {
                                pending_param_cmd = Some(CadCommand::SetParameter {
                                    name: p.name.clone(),
                                    value: ParameterValue::Number(val),
                                    unit: p.unit.clone(),
                                    description: p.description.clone(),
                                });
                            }
                        }
                        ParameterValue::Expression(mut expr) => {
                            let resp = ui.text_edit_singleline(&mut expr);
                            if resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                                pending_param_cmd = Some(CadCommand::SetParameter {
                                    name: p.name.clone(),
                                    value: ParameterValue::Expression(expr),
                                    unit: p.unit.clone(),
                                    description: p.description.clone(),
                                });
                            }
                        }
                    }
                    let eval_val = resolved.get(&p.name).copied().unwrap_or(f64::NAN);
                    ui.label(format!("{eval_val:.2} {}", p.unit));
                    ui.end_row();
                }
            });

        if let Some(cmd) = pending_param_cmd {
            self.execute_cmd(cmd);
        }

        ui.add_space(6.0);
        ui.collapsing("＋ 新規パラメータ追加", |ui| {
            ui.horizontal(|ui| {
                ui.label("名前:");
                ui.text_edit_singleline(&mut self.new_param_name);
            });
            ui.horizontal(|ui| {
                ui.label("値/式:");
                ui.text_edit_singleline(&mut self.new_param_expr);
            });
            if ui.button("追加").clicked() && !self.new_param_name.trim().is_empty() {
                let val = if let Ok(num) = self.new_param_expr.trim().parse::<f64>() {
                    ParameterValue::Number(num)
                } else {
                    ParameterValue::Expression(self.new_param_expr.trim().to_string())
                };
                self.execute_cmd(CadCommand::SetParameter {
                    name: self.new_param_name.trim().to_string(),
                    value: val,
                    unit: "mm".to_string(),
                    description: "ユーザー定義パラメータ".to_string(),
                });
                self.new_param_name.clear();
                self.new_param_expr.clear();
            }
        });

        ui.add_space(12.0);
        // A flat, chronological timeline rather than a nested tree: each feature is
        // listed in the order it was created, so "what happened, in what order" stays
        // readable without digging through nesting (UX concept B).
        ui.heading("🕒 操作タイムライン (作成順)");
        ui.separator();

        let features_list: Vec<_> = self
            .controller
            .document
            .features
            .values()
            .cloned()
            .collect();
        let mut visibility_toggle: Option<(FeatureId, bool)> = None;

        // Dependency highlighting: only computed relative to the current selection, and
        // only drawn as a lightweight tint + arrow note, so the timeline stays flat and
        // readable instead of turning back into a nested tree (UX concept B).
        let selected = self.render_config.selected_feature;
        let direct_upstream: std::collections::HashSet<FeatureId> = selected
            .and_then(|sid| self.controller.document.features.get(&sid))
            .map(|f| f.kind.upstream_features().into_iter().collect())
            .unwrap_or_default();
        let direct_downstream: std::collections::HashSet<FeatureId> = selected
            .map(|sid| {
                features_list
                    .iter()
                    .filter(|f| f.kind.upstream_features().contains(&sid))
                    .map(|f| f.id)
                    .collect()
            })
            .unwrap_or_default();

        for feat in &features_list {
            ui.horizontal(|ui| {
                let mut vis = feat.visible;
                if ui
                    .checkbox(&mut vis, "")
                    .on_hover_text("表示 / 非表示")
                    .changed()
                {
                    visibility_toggle = Some((feat.id, vis));
                }

                let (badge, badge_color) = match &feat.state {
                    FeatureState::Clean => ("✓ Clean", Color32::from_rgb(80, 210, 120)),
                    FeatureState::Dirty => ("⟳ Dirty", Color32::from_rgb(230, 190, 70)),
                    FeatureState::Computing => ("… Computing", Color32::from_rgb(100, 180, 240)),
                    FeatureState::Failed(_) => ("✗ Failed", Color32::from_rgb(240, 85, 85)),
                    FeatureState::Blocked { .. } => ("⊘ Blocked", Color32::from_rgb(235, 140, 60)),
                };

                let is_selected = selected == Some(feat.id);
                let rel_marker = if is_selected {
                    ""
                } else if direct_upstream.contains(&feat.id) {
                    "↑ "
                } else if direct_downstream.contains(&feat.id) {
                    "↓ "
                } else {
                    "  "
                };
                let label_text = format!("{rel_marker}#{} {}", feat.id.0, feat.name);
                let label = if direct_upstream.contains(&feat.id) {
                    RichText::new(label_text).color(Color32::from_rgb(120, 180, 255))
                } else if direct_downstream.contains(&feat.id) {
                    RichText::new(label_text).color(Color32::from_rgb(255, 180, 110))
                } else {
                    RichText::new(label_text)
                };
                if ui.selectable_label(is_selected, label).clicked() {
                    self.render_config.selected_feature = Some(feat.id);
                    self.render_config.selected_face = None;
                    self.render_config.selected_edge = None;
                    self.viewport_dirty = true;
                }
                ui.colored_label(badge_color, badge);
            });
        }
        if selected.is_some() && (!direct_upstream.is_empty() || !direct_downstream.is_empty()) {
            ui.add_space(4.0);
            ui.small("↑ 選択中のフィーチャーが参照している元 / ↓ それを参照している先");
        }

        if let Some((fid, new_vis)) = visibility_toggle {
            self.execute_cmd(CadCommand::UpdateFeature {
                id: fid,
                name: None,
                kind: None,
                visible: Some(new_vis),
                color_rgba: None,
            });
        }
    }

    fn render_right_property_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("🔧 プロパティ (Properties)");
        ui.separator();

        let Some(selected_id) = self.render_config.selected_feature else {
            ui.label("フィーチャーが選択されていません。\nFeature Tree または 3D Viewport をクリックしてください。");
            return;
        };

        let Some(feat) = self.controller.document.features.get(&selected_id).cloned() else {
            return;
        };

        ui.label(
            RichText::new(format!("Feature #{}: {}", feat.id.0, feat.name))
                .strong()
                .size(15.0),
        );

        match &feat.state {
            FeatureState::Clean => {
                ui.colored_label(Color32::from_rgb(80, 210, 120), "状態: Clean（正常）");
            }
            FeatureState::Failed(msg) => {
                ui.colored_label(
                    Color32::from_rgb(245, 90, 90),
                    format!(
                        "状態: 計算失敗 ({msg})\n※3Dビューには最後に正常だった形状を半透明表示中"
                    ),
                );
            }
            FeatureState::Blocked { upstream_failed } => {
                ui.colored_label(
                    Color32::from_rgb(240, 150, 65),
                    format!(
                        "状態: Blocked（上流 Feature #{} の失敗により待機中）",
                        upstream_failed.0
                    ),
                );
            }
            other => {
                ui.label(format!("状態: {other:?}"));
            }
        }

        let deps = feat.all_dependencies();
        if !deps.is_empty() {
            ui.label(format!("依存元ノード: {deps:?}"));
        }

        ui.separator();
        ui.strong("形状寸法・パラメータ編集:");

        let mut updated_kind = feat.kind.clone();
        let mut kind_changed = false;

        match &mut updated_kind {
            FeatureKind::Box {
                width,
                height,
                depth,
            } => {
                kind_changed |= edit_scalar_param(ui, "幅 (width)", width);
                kind_changed |= edit_scalar_param(ui, "高さ (height)", height);
                kind_changed |= edit_scalar_param(ui, "厚み (depth)", depth);
            }
            FeatureKind::Cylinder { radius, height } => {
                kind_changed |= edit_scalar_param(ui, "半径 (radius)", radius);
                kind_changed |= edit_scalar_param(ui, "高さ (height)", height);
            }
            FeatureKind::Sphere { radius } => {
                kind_changed |= edit_scalar_param(ui, "半径 (radius)", radius);
            }
            FeatureKind::Cone {
                bottom_radius,
                top_radius,
                height,
            } => {
                kind_changed |= edit_scalar_param(ui, "底面半径", bottom_radius);
                kind_changed |= edit_scalar_param(ui, "上面半径", top_radius);
                kind_changed |= edit_scalar_param(ui, "高さ", height);
            }
            FeatureKind::Transform {
                dx,
                dy,
                dz,
                rot_angle_deg,
                ..
            } => {
                kind_changed |= edit_scalar_param(ui, "移動 X (dx)", dx);
                kind_changed |= edit_scalar_param(ui, "移動 Y (dy)", dy);
                kind_changed |= edit_scalar_param(ui, "移動 Z (dz)", dz);
                kind_changed |= edit_scalar_param(ui, "回転角 (deg)", rot_angle_deg);
            }
            FeatureKind::LinearPattern {
                spacing_x,
                spacing_y,
                count_x,
                count_y,
                ..
            } => {
                kind_changed |= edit_scalar_param(ui, "ピッチ X", spacing_x);
                kind_changed |= edit_scalar_param(ui, "ピッチ Y", spacing_y);
                kind_changed |= ui
                    .add(
                        egui::DragValue::new(count_x)
                            .range(1..=20)
                            .prefix("個数 X: "),
                    )
                    .changed();
                kind_changed |= ui
                    .add(
                        egui::DragValue::new(count_y)
                            .range(1..=20)
                            .prefix("個数 Y: "),
                    )
                    .changed();
            }
            FeatureKind::Fillet { radius, .. } => {
                kind_changed |= edit_scalar_param(ui, "フィレット半径", radius);
            }
            FeatureKind::Chamfer { distance, .. } => {
                kind_changed |= edit_scalar_param(ui, "面取り距離", distance);
            }
            FeatureKind::Boolean {
                op,
                primary,
                secondary,
            } => {
                ui.label(format!("ブーリアン演算: {op:?}"));
                ui.label(format!(
                    "対象1: Feature #{} / 対象2: Feature #{}",
                    primary.0, secondary.0
                ));
            }
            _ => {}
        }

        if kind_changed {
            self.execute_cmd(CadCommand::UpdateFeature {
                id: feat.id,
                name: None,
                kind: Some(updated_kind),
                visible: None,
                color_rgba: None,
            });
        }

        if let Some(summary) = feat.cached_summary.or(feat.last_valid_summary) {
            ui.separator();
            ui.strong("📊 幾何・トポロジー計測結果:");
            ui.label(format!("体積: {:.2} mm³", summary.volume_mm3));
            ui.label(format!("表面積: {:.2} mm²", summary.surface_area_mm2));
            ui.label(format!(
                "トポロジー: Solid {} / Face {} / Edge {} / Vertex {}",
                summary.solids, summary.faces, summary.edges, summary.vertices
            ));
            let sz = summary.bounds.size();
            ui.label(format!(
                "外形寸法 (BBox): {:.1} × {:.1} × {:.1} mm",
                sz.x, sz.y, sz.z
            ));
        }

        if let Some((fid, face_idx)) = self.render_config.selected_face
            && fid == feat.id
        {
            ui.separator();
            ui.colored_label(
                Color32::from_rgb(250, 200, 60),
                format!("🎯 選択中の面: Face #{face_idx}"),
            );
            if let Some(sid) = feat.cached_shape
                && let Ok(shape_info) = self.store.inspect_shape(sid)
                && let Some(face) = shape_info.faces.iter().find(|f| f.index == face_idx)
            {
                ui.label(format!("面積: {:.2} mm²", face.area_mm2));
                ui.label(format!(
                    "重心: ({:.2}, {:.2}, {:.2})",
                    face.centroid.x, face.centroid.y, face.centroid.z
                ));
                ui.label(format!(
                    "外向き法線: ({:.2}, {:.2}, {:.2})",
                    face.normal.x, face.normal.y, face.normal.z
                ));
            }
        }

        if let Some((fid, edge_idx)) = self.render_config.selected_edge
            && fid == feat.id
        {
            ui.separator();
            ui.colored_label(
                Color32::from_rgb(255, 110, 50),
                format!("📏 選択中の辺: Edge #{edge_idx}"),
            );
            if let Some(sid) = feat.cached_shape
                && let Ok(shape_info) = self.store.inspect_shape(sid)
                && let Some(edge) = shape_info.edges.iter().find(|e| e.index == edge_idx)
            {
                ui.label(format!("長さ: {:.2} mm", edge.length_mm));
                ui.label(format!(
                    "中点: ({:.2}, {:.2}, {:.2})",
                    edge.centroid.x, edge.centroid.y, edge.centroid.z
                ));
                if ui.button("この辺に 1.5mm フィレットを追加").clicked() {
                    // Capture a stable semantic reference for this edge (TNP mitigation):
                    // later edits to upstream dimensions re-identify the edge by geometric
                    // signature instead of trusting this raw index, which can drift.
                    let semantic_refs = SemanticReference::from_edge(feat.id, &self.store, sid, edge_idx)
                        .map(|r| vec![r])
                        .unwrap_or_default();
                    self.execute_cmd(CadCommand::AddFeature {
                        name: format!("Fillet_Edge{edge_idx}"),
                        kind: FeatureKind::Fillet {
                            input: feat.id,
                            radius: ScalarParam::mm(1.5),
                            edge_indices: vec![edge_idx],
                            semantic_refs,
                        },
                        visible: true,
                        color_rgba: feat.color_rgba,
                    });
                }
            }
        }
    }

    /// Flat 2D sketch canvas: click to place profile points on the sketch plane
    /// (millimeters), then close the loop and extrude it into a real feature using
    /// `cad_core`'s `Sketch` (the same constraint-solver-backed profile type the
    /// parametric engine already understands — not a UI-only polygon).
    fn render_sketch_canvas(&mut self, ctx: &Context, ui: &mut egui::Ui) {
        const PX_PER_MM: f32 = 3.0;

        let avail = ui.available_size();
        let (rect, response) =
            ui.allocate_exact_size(avail, Sense::click_and_drag());
        let origin = rect.center();

        let to_screen = |p: [f64; 2]| -> Pos2 {
            Pos2::new(
                origin.x + p[0] as f32 * PX_PER_MM,
                origin.y - p[1] as f32 * PX_PER_MM,
            )
        };
        let to_mm = |p: Pos2| -> [f64; 2] {
            [
                ((p.x - origin.x) / PX_PER_MM) as f64,
                (-(p.y - origin.y) / PX_PER_MM) as f64,
            ]
        };

        let painter = ui.painter();
        painter.rect_filled(rect, 0.0, Color32::from_rgb(24, 27, 33));
        // Light grid every 10mm so placed points can be read at a glance.
        let grid_mm = 10.0;
        let grid_px = grid_mm * PX_PER_MM;
        let mut gx = origin.x % grid_px;
        while gx < rect.width() {
            let x = rect.min.x + gx;
            painter.line_segment(
                [Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)],
                egui::Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 50)),
            );
            gx += grid_px;
        }
        let mut gy = origin.y % grid_px;
        while gy < rect.height() {
            let y = rect.min.y + gy;
            painter.line_segment(
                [Pos2::new(rect.min.x, y), Pos2::new(rect.max.x, y)],
                egui::Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 50)),
            );
            gy += grid_px;
        }
        painter.line_segment(
            [Pos2::new(rect.min.x, origin.y), Pos2::new(rect.max.x, origin.y)],
            egui::Stroke::new(1.5_f32, Color32::from_rgb(90, 60, 60)),
        );
        painter.line_segment(
            [Pos2::new(origin.x, rect.min.y), Pos2::new(origin.x, rect.max.y)],
            egui::Stroke::new(1.5_f32, Color32::from_rgb(60, 90, 60)),
        );

        if response.clicked()
            && let Some(pos) = response.interact_pointer_pos()
        {
            self.sketch_points.push(to_mm(pos));
        }

        for pair in self.sketch_points.windows(2) {
            painter.line_segment(
                [to_screen(pair[0]), to_screen(pair[1])],
                egui::Stroke::new(2.0_f32, Color32::from_rgb(120, 180, 255)),
            );
        }
        if self.sketch_points.len() >= 3
            && let (Some(first), Some(last)) =
                (self.sketch_points.first(), self.sketch_points.last())
        {
            painter.line_segment(
                [to_screen(*last), to_screen(*first)],
                egui::Stroke::new(1.5_f32, Color32::from_rgb(120, 180, 255, ).gamma_multiply(0.5)),
            );
        }
        for p in &self.sketch_points {
            painter.circle_filled(to_screen(*p), 3.5, Color32::from_rgb(250, 200, 60));
        }

        let hud_rect = Rect::from_min_size(rect.min + Vec2::new(12.0, 12.0), Vec2::new(420.0, 90.0));
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(hud_rect), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(22, 26, 34, 225))
                .show(ui, |ui| {
                    ui.label("✏ スケッチモード: クリックして輪郭の頂点を置いてください");
                    ui.label(format!("頂点数: {}  (3点以上で押し出し可能)", self.sketch_points.len()));
                    ui.horizontal(|ui| {
                        ui.label("押し出し量:");
                        ui.add(egui::DragValue::new(&mut self.sketch_depth).speed(0.5).suffix(" mm"));
                    });
                    ui.horizontal(|ui| {
                        if ui
                            .add_enabled(
                                self.sketch_points.len() >= 3,
                                egui::Button::new("✓ 閉じて押し出し"),
                            )
                            .clicked()
                        {
                            self.finish_sketch_extrude();
                        }
                        if ui.button("↶ 最後の点を取消").clicked() {
                            self.sketch_points.pop();
                        }
                        if ui.button("✗ キャンセル (Esc)").clicked() {
                            self.sketch_mode = false;
                            self.sketch_points.clear();
                        }
                    });
                });
        });

        if ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.sketch_mode = false;
            self.sketch_points.clear();
        }
    }

    /// Converts the in-progress 2D sketch polygon into a real `cad_core::Sketch` and
    /// submits it as a `SketchExtrude` feature through the normal command pipeline, so it
    /// participates in the dependency graph, undo/redo, and recompute like any other
    /// feature.
    fn finish_sketch_extrude(&mut self) {
        if self.sketch_points.len() < 3 {
            return;
        }
        let mut sketch = Sketch::new(format!(
            "Sketch_{}",
            self.controller.document.features.len() + 1
        ));
        let point_ids: Vec<_> = self
            .sketch_points
            .iter()
            .map(|[x, y]| sketch.add_point(*x, *y))
            .collect();
        sketch.add_polyline(point_ids, true);

        self.execute_cmd(CadCommand::AddFeature {
            name: format!("SketchExtrude_{}", self.controller.document.features.len() + 1),
            kind: FeatureKind::SketchExtrude {
                sketch,
                depth: ScalarParam::mm(self.sketch_depth.max(0.1)),
            },
            visible: true,
            color_rgba: [0.55, 0.75, 0.95, 1.0],
        });

        self.sketch_mode = false;
        self.sketch_points.clear();
    }

    fn render_central_viewport(&mut self, ctx: &Context, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let width = (avail.x.max(64.0) as u32).min(2048);
        let height = (avail.y.max(64.0) as u32).min(2048);

        if self.last_viewport_size != [width, height] {
            self.last_viewport_size = [width, height];
            self.viewport_dirty = true;
        }

        if self.viewport_dirty {
            if let Some(ref renderer) = self.renderer
                && let Ok(rgba) = renderer.render_to_rgba(
                    &self.controller.document,
                    &self.camera,
                    &self.render_config,
                    width,
                    height,
                )
            {
                let color_img =
                    ColorImage::from_rgba_unmultiplied([width as usize, height as usize], &rgba);
                match &mut self.viewport_texture {
                    Some(tex) => tex.set(color_img, TextureOptions::LINEAR),
                    None => {
                        self.viewport_texture = Some(ctx.load_texture(
                            "cad_viewport_tex",
                            color_img,
                            TextureOptions::LINEAR,
                        ));
                    }
                }
            }
            self.viewport_dirty = false;
        }

        let (rect, response) = ui.allocate_exact_size(
            Vec2::new(width as f32, height as f32),
            Sense::click_and_drag(),
        );

        if let Some(ref tex) = self.viewport_texture {
            ui.painter().image(
                tex.id(),
                rect,
                Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                Color32::WHITE,
            );
        } else {
            ui.painter()
                .rect_filled(rect, 0.0, Color32::from_rgb(28, 33, 41));
        }

        // Mouse camera manipulation (Orbit, Pan, Zoom)
        if response.dragged() {
            let delta = response.drag_delta();
            let shift = ctx.input(|i| i.modifiers.shift);
            if response.dragged_by(egui::PointerButton::Secondary)
                || (response.dragged_by(egui::PointerButton::Primary) && shift)
            {
                self.camera.pan(delta.x, delta.y);
                self.viewport_dirty = true;
            } else if response.dragged_by(egui::PointerButton::Primary)
                || response.dragged_by(egui::PointerButton::Middle)
            {
                self.camera.orbit(-delta.x * 0.008, delta.y * 0.008);
                self.viewport_dirty = true;
            }
        }

        if response.hovered() {
            let scroll_y = ctx.input(|i| i.smooth_scroll_delta.y);
            if scroll_y.abs() > 0.1 {
                let factor = (1.0 - scroll_y * 0.0018).clamp(0.75, 1.35);
                self.camera.zoom(factor);
                self.viewport_dirty = true;
            }
        }

        // 3D Raycast Picking: single-click selects, double-click also opens
        // a floating on-canvas property editor right at the clicked point
        // (no need to go hunting through a side panel).
        if (response.clicked() || response.double_clicked())
            && let Some(pos) = response.interact_pointer_pos()
        {
            let uv_x = ((pos.x - rect.min.x) / rect.width()).clamp(0.0, 1.0);
            let uv_y = ((pos.y - rect.min.y) / rect.height()).clamp(0.0, 1.0);
            let aspect = rect.width() / rect.height().max(1.0);

            if let Some(hit) =
                pick_document(&self.controller.document, &self.camera, uv_x, uv_y, aspect)
            {
                self.render_config.selected_feature = Some(hit.feature_id);
                self.render_config.selected_face = Some((hit.feature_id, hit.face_index));
                self.render_config.selected_edge =
                    hit.nearest_edge_index.map(|eid| (hit.feature_id, eid));
                self.status_logs.push(format!(
                    "「{}」の面を選択しました",
                    self.controller
                        .document
                        .features
                        .get(&hit.feature_id)
                        .map(|f| f.name.as_str())
                        .unwrap_or("形状")
                ));
                if response.double_clicked() {
                    self.floating_editor_pos = Some(pos);
                }
            } else {
                self.render_config.selected_face = None;
                self.render_config.selected_edge = None;
                self.floating_editor_pos = None;
            }
            self.viewport_dirty = true;
        }

        self.render_floating_editor(ctx);

        // Top-Left HUD Overlay (View & Display Quick Controls)
        let hud_rect =
            Rect::from_min_size(rect.min + Vec2::new(12.0, 12.0), Vec2::new(560.0, 36.0));
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(hud_rect), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(22, 26, 34, 215))
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        for (label, view) in [
                            ("Iso", StandardView::Isometric),
                            ("Top", StandardView::Top),
                            ("Front", StandardView::Front),
                            ("Right", StandardView::Right),
                        ] {
                            if ui.small_button(label).clicked() {
                                self.camera.set_standard_view(view);
                                self.viewport_dirty = true;
                            }
                        }
                        if ui.small_button("Fit [F]").clicked()
                            && let Some(bbox) = self.controller.document.visible_bounding_box()
                        {
                            self.camera.fit_bounding_box(&bbox);
                            self.viewport_dirty = true;
                        }
                        ui.separator();
                        let proj_label = match self.camera.projection {
                            ProjectionMode::Perspective { .. } => "透視投影 (Persp)",
                            ProjectionMode::Orthographic { .. } => "平行投影 (Ortho)",
                        };
                        if ui.small_button(proj_label).clicked() {
                            self.camera.toggle_projection();
                            self.viewport_dirty = true;
                        }
                        ui.separator();
                        for (label, mode) in [
                            ("面+辺", DisplayMode::ShadedWithEdges),
                            ("面のみ", DisplayMode::Shaded),
                            ("線画", DisplayMode::Wireframe),
                        ] {
                            if ui
                                .selectable_label(self.render_config.display_mode == mode, label)
                                .clicked()
                            {
                                self.render_config.display_mode = mode;
                                self.viewport_dirty = true;
                            }
                        }
                    });
                });
        });
    }

    /// Floating property editor that appears directly at the point the user
    /// double-clicked in the 3D viewport, so dimensions can be changed without
    /// hunting through the side panel (UX concept C).
    fn render_floating_editor(&mut self, ctx: &Context) {
        let Some(pos) = self.floating_editor_pos else {
            return;
        };
        let Some(fid) = self.render_config.selected_feature else {
            self.floating_editor_pos = None;
            return;
        };
        let Some(feat) = self.controller.document.features.get(&fid).cloned() else {
            self.floating_editor_pos = None;
            return;
        };

        let mut still_open = true;
        let mut updated_kind = feat.kind.clone();
        let mut kind_changed = false;

        egui::Area::new(egui::Id::new("floating_property_editor"))
            .fixed_pos(pos + Vec2::new(14.0, 14.0))
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_min_width(220.0);
                    ui.horizontal(|ui| {
                        ui.strong(format!("{}", feat.name));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("×").clicked() {
                                still_open = false;
                            }
                        });
                    });
                    ui.separator();
                    match &mut updated_kind {
                        FeatureKind::Box {
                            width,
                            height,
                            depth,
                        } => {
                            kind_changed |= edit_scalar_param(ui, "幅", width);
                            kind_changed |= edit_scalar_param(ui, "高さ", height);
                            kind_changed |= edit_scalar_param(ui, "厚み", depth);
                        }
                        FeatureKind::Cylinder { radius, height } => {
                            kind_changed |= edit_scalar_param(ui, "半径", radius);
                            kind_changed |= edit_scalar_param(ui, "高さ", height);
                        }
                        FeatureKind::Sphere { radius } => {
                            kind_changed |= edit_scalar_param(ui, "半径", radius);
                        }
                        FeatureKind::Fillet { radius, .. } => {
                            kind_changed |= edit_scalar_param(ui, "フィレット半径", radius);
                        }
                        FeatureKind::Chamfer { distance, .. } => {
                            kind_changed |= edit_scalar_param(ui, "面取り距離", distance);
                        }
                        _ => {
                            ui.label("この形状はここでは編集できません。\n右側のプロパティパネルをご利用ください。");
                        }
                    }
                    ui.add_space(4.0);
                    ui.small("ダブルクリックした場所でそのまま編集できます");
                });
            });

        if kind_changed {
            self.execute_cmd(CadCommand::UpdateFeature {
                id: feat.id,
                name: None,
                kind: Some(updated_kind),
                visible: None,
                color_rgba: None,
            });
        }

        if !still_open || ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.floating_editor_pos = None;
        }
    }
}

fn edit_scalar_param(ui: &mut egui::Ui, label: &str, param: &mut ScalarParam) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        match param {
            ScalarParam::Literal(val) => {
                if ui
                    .add(egui::DragValue::new(val).speed(0.5).suffix(" mm"))
                    .changed()
                {
                    changed = true;
                }
            }
            ScalarParam::Expr(expr) => {
                let resp = ui.text_edit_singleline(expr);
                if resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                    changed = true;
                }
            }
        }
    });
    changed
}

impl eframe::App for FerroCadApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        self.handle_shortcuts(ctx);

        egui::TopBottomPanel::top("top_menubar").show(ctx, |ui| {
            self.render_top_menubar(ui);
        });

        egui::TopBottomPanel::bottom("bottom_status_panel")
            .resizable(true)
            .default_height(85.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.strong("📋 ステータス・操作履歴ログ (Logs & Command History)");
                    if ui.small_button("クリア").clicked() {
                        self.status_logs.clear();
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for log in &self.status_logs {
                            ui.label(log);
                        }
                    });
            });

        egui::SidePanel::left("left_tree_panel")
            .resizable(true)
            .default_width(310.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.render_left_panel(ui);
                });
            });

        egui::SidePanel::right("right_property_panel")
            .resizable(true)
            .default_width(300.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.render_right_property_panel(ui);
                });
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ctx, |ui| {
                if self.sketch_mode {
                    self.render_sketch_canvas(ctx, ui);
                } else {
                    self.render_central_viewport(ctx, ui);
                }
            });

        if self.show_command_palette {
            egui::Window::new("⌘ 統合コマンドパレット")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_TOP, Vec2::new(0.0, 60.0))
                .show(ctx, |ui| {
                    ui.text_edit_singleline(&mut self.command_filter);
                    ui.separator();
                    if ui.button("直方体フィーチャーを追加 (Add Box)").clicked() {
                        self.execute_cmd(CadCommand::AddFeature {
                            name: "Box".to_string(),
                            kind: FeatureKind::Box {
                                width: ScalarParam::mm(50.0),
                                height: ScalarParam::mm(50.0),
                                depth: ScalarParam::mm(20.0),
                            },
                            visible: true,
                            color_rgba: [0.35, 0.65, 0.90, 1.0],
                        });
                        self.show_command_palette = false;
                    }
                    if ui.button("円柱フィーチャーを追加 (Add Cylinder)").clicked() {
                        self.execute_cmd(CadCommand::AddFeature {
                            name: "Cylinder".to_string(),
                            kind: FeatureKind::Cylinder {
                                radius: ScalarParam::mm(15.0),
                                height: ScalarParam::mm(40.0),
                            },
                            visible: true,
                            color_rgba: [0.45, 0.78, 0.55, 1.0],
                        });
                        self.show_command_palette = false;
                    }
                    if ui.button("全体フィット (Fit All)").clicked() {
                        if let Some(bbox) = self.controller.document.visible_bounding_box() {
                            self.camera.fit_bounding_box(&bbox);
                            self.viewport_dirty = true;
                        }
                        self.show_command_palette = false;
                    }
                    if ui.button("Milestone M1 穴あきプレートを再生成").clicked() {
                        *self = Self::new_with_mvp_demo();
                        self.show_command_palette = false;
                    }
                });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase3_and_4_ui_state_and_command_execution() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        assert_eq!(app.controller.document.features.len(), 5);
        assert!(app.render_config.selected_feature.is_some());

        // Modify width parameter through UI command dispatch
        app.execute_cmd(CadCommand::SetParameter {
            name: "width".to_string(),
            value: ParameterValue::Number(120.0),
            unit: "mm".to_string(),
            description: "GUIから幅を120mmへ変更".to_string(),
        });
        assert!(app.viewport_dirty);
        let final_id = FeatureId(5);
        let vol = app.controller.document.features[&final_id]
            .cached_summary
            .unwrap()
            .volume_mm3;
        let expected_vol = 120.0 * 60.0 * 10.0 - 4.0 * std::f64::consts::PI * 3.3 * 3.3 * 10.0;
        assert!((vol - expected_vol).abs() < 1.0);
    }
}
