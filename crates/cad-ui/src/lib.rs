//! Desktop CAD GUI for FerroCAD powered by `egui` / `eframe` and `cad-render` (Phase 3 & Phase 4).

use std::path::PathBuf;

use cad_core::{
    CadCommand, CoreError, Document, DocumentController, ExtrudeMode, FeatureId, FeatureKind,
    FeatureState, ParameterValue, PlaneFrame, ScalarParam, SemanticReference, Sketch, SketchPlane,
    build_perforated_plate_mvp,
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

/// Tabs of the Office-style ribbon toolbar at the top of the window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RibbonTab {
    #[default]
    Solid,
    View,
}

/// One ribbon button: a large icon glyph above a short label, fixed width so groups
/// line up like a desktop CAD ribbon.
fn ribbon_button(ui: &mut egui::Ui, icon: &str, label: &str, enabled: bool, hint: &str) -> bool {
    let text = RichText::new(format!("{icon}\n{label}")).size(12.0);
    ui.add_enabled(
        enabled,
        egui::Button::new(text).min_size(Vec2::new(58.0, 48.0)).frame(false),
    )
    .on_hover_text(hint)
    .on_disabled_hover_text(hint)
    .clicked()
}

/// A titled ribbon group (e.g. "作成", "修正"): buttons in a row, group name below.
fn ribbon_group(ui: &mut egui::Ui, title: &str, add_contents: impl FnOnce(&mut egui::Ui)) {
    ui.vertical(|ui| {
        ui.horizontal(add_contents);
        ui.vertical_centered(|ui| {
            ui.label(RichText::new(title).size(11.0).weak());
        });
    });
    ui.separator();
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
    /// Snap new sketch points to the sketch's own vertices (to close loops exactly),
    /// to the 10mm grid, and to the midpoints of edges belonging to other already-built
    /// features (projected onto the sketch's XY plane) — a lightweight version of the
    /// object-snap found in most CAD sketchers.
    pub sketch_snap_enabled: bool,
    pub ribbon_tab: RibbonTab,
    /// Waiting for the user to choose a sketch plane (base plane button or a face click).
    pub sketch_plane_picking: bool,
    /// Face under the cursor while picking a sketch plane, highlighted in the viewport.
    pub hover_face: Option<(FeatureId, u32)>,
    pub sketch_plane: SketchPlane,
    pub sketch_frame: PlaneFrame,
    pub sketch_op: SketchOp,
    /// Body that Join/Cut combines with (the picked face's feature, or the selection).
    pub sketch_target: Option<FeatureId>,
    pub sketch_reversed: bool,
    pub sketch_symmetric: bool,
    pub rename_target: Option<(FeatureId, String)>,
    /// Sketch canvas view: centre (sketch mm) and zoom (px per mm). `None` = fit on next frame.
    pub sketch_view: Option<([f64; 2], f32)>,
}

/// Initial sketch view framing the existing geometry (and any placed points).
fn fit_sketch_view(segments: &[[[f64; 2]; 2]], points: &[[f64; 2]], rect: Rect) -> ([f64; 2], f32) {
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    for p in segments.iter().flatten().chain(points) {
        for i in 0..2 {
            min[i] = min[i].min(p[i]);
            max[i] = max[i].max(p[i]);
        }
    }
    if !min[0].is_finite() {
        return ([0.0, 0.0], 3.0);
    }
    let extent = (max[0] - min[0]).max(max[1] - min[1]).max(10.0) as f32;
    let px_per_mm = (rect.width().min(rect.height()) * 0.7 / extent).clamp(0.3, 30.0);
    ([(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5], px_per_mm)
}

/// Action chosen from a timeline chip's right-click menu.
enum ChipAction {
    Rename(FeatureId),
    ToggleSuppress(FeatureId),
    Delete(FeatureId),
}

/// How the finished sketch is turned into geometry (UI-side choice for `ExtrudeMode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchOp {
    NewBody,
    Join,
    Cut,
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
            sketch_snap_enabled: true,
            ribbon_tab: RibbonTab::default(),
            sketch_plane_picking: false,
            hover_face: None,
            sketch_plane: SketchPlane::XY,
            sketch_frame: PlaneFrame::from_point_normal(glam::DVec3::ZERO, glam::DVec3::Z),
            sketch_op: SketchOp::NewBody,
            sketch_target: None,
            sketch_reversed: false,
            sketch_symmetric: false,
            rename_target: None,
            sketch_view: None,
        }
    }

    fn add_primitive(&mut self, base_name: &str, kind: FeatureKind, color_rgba: [f32; 4]) {
        self.execute_cmd(CadCommand::AddFeature {
            name: format!("{base_name}_{}", self.controller.document.features.len() + 1),
            kind,
            visible: true,
            color_rgba,
        });
    }

    /// First step of a sketch: ask for a plane (base plane button or face click).
    fn start_sketch(&mut self) {
        self.sketch_plane_picking = true;
        self.hover_face = None;
        self.floating_editor_pos = None;
    }

    fn cancel_sketch(&mut self) {
        self.sketch_plane_picking = false;
        self.sketch_mode = false;
        self.sketch_points.clear();
        if self.hover_face.take().is_some() {
            self.render_config.selected_face = None;
            self.viewport_dirty = true;
        }
    }

    /// Opens the 2D canvas on `plane`. Join/Cut default to `target`: the picked face's
    /// body, or for a base plane the currently selected feature.
    fn begin_sketch(&mut self, plane: SketchPlane, target: Option<FeatureId>) {
        let frame = match self.controller.document.sketch_plane_frame(&plane, &self.store) {
            Ok(frame) => frame,
            Err(err) => {
                self.status_logs
                    .push(format!("この平面にはスケッチを開始できませんでした: {err}"));
                return;
            }
        };
        self.sketch_plane_picking = false;
        self.hover_face = None;
        self.render_config.selected_face = None;
        self.sketch_plane = plane;
        self.sketch_frame = frame;
        self.sketch_target = target;
        // Sketching on a face usually means adding to that body; a base-plane sketch
        // usually starts a new one.
        self.set_sketch_op(if target.is_some() && matches!(self.sketch_plane, SketchPlane::Face { .. }) {
            SketchOp::Join
        } else {
            SketchOp::NewBody
        });
        self.sketch_symmetric = false;
        self.sketch_mode = true;
        self.sketch_points.clear();
        self.sketch_depth = 10.0;
        self.sketch_view = None;
        self.viewport_dirty = true;
    }

    /// Cut defaults to extruding into the material (against the face's outward normal).
    fn set_sketch_op(&mut self, op: SketchOp) {
        self.sketch_op = op;
        self.sketch_reversed = op == SketchOp::Cut;
    }

    fn begin_sketch_on_face(&mut self, fid: FeatureId, face_index: u32) {
        let Some(shape_id) = self.controller.document.features.get(&fid).and_then(|f| f.cached_shape)
        else {
            return;
        };
        match SemanticReference::from_face(fid, &self.store, shape_id, face_index) {
            Ok(face) => self.begin_sketch(SketchPlane::Face { feature: fid, face }, Some(fid)),
            Err(err) => self
                .status_logs
                .push(format!("この面を基準にできませんでした: {err:?}")),
        }
    }

    fn feature_name(&self, id: FeatureId) -> String {
        self.controller
            .document
            .features
            .get(&id)
            .map(|f| f.name.clone())
            .unwrap_or_else(|| format!("#{}", id.0))
    }

    /// Deletes a feature; if others depend on it, explains which ones instead.
    fn delete_feature(&mut self, id: FeatureId) {
        let name = self.feature_name(id);
        match self
            .controller
            .execute(CadCommand::RemoveFeature { id }, Some(&mut self.store))
        {
            Ok(_) => {
                if self.render_config.selected_feature == Some(id) {
                    self.render_config.selected_feature = None;
                    self.render_config.selected_face = None;
                    self.render_config.selected_edge = None;
                }
                self.status_logs.push(format!("「{name}」を削除しました（Ctrl+Zで元に戻せます）"));
                self.recompute_and_refresh();
            }
            Err(CoreError::FeatureInUse { dependents, .. }) => {
                let users: Vec<String> = dependents
                    .iter()
                    .map(|d| format!("「{}」", self.feature_name(*d)))
                    .collect();
                self.status_logs.push(format!(
                    "「{name}」は {} から使われているため削除できません。先にそちらを削除するか、一時的に止めたい場合は「抑制」を使ってください",
                    users.join("、")
                ));
            }
            Err(err) => self.status_logs.push(format!("削除できませんでした: {err}")),
        }
    }

    fn toggle_suppressed(&mut self, id: FeatureId) {
        let Some(now) = self.controller.document.features.get(&id).map(|f| f.suppressed) else {
            return;
        };
        let name = self.feature_name(id);
        self.execute_cmd(CadCommand::SetSuppressed {
            id,
            suppressed: !now,
        });
        self.status_logs.push(if now {
            format!("「{name}」の抑制を解除しました")
        } else {
            format!("「{name}」を抑制しました（形状から一時的に外れます）")
        });
    }

    /// Adds a fillet or chamfer on the currently selected edge, capturing a semantic
    /// reference so the edge is re-identified (not trusted by raw index) after upstream
    /// edits. Returns false when no edge is selected.
    fn add_edge_modifier_on_selection(&mut self, chamfer: bool) -> bool {
        let Some((fid, edge_idx)) = self.render_config.selected_edge else {
            return false;
        };
        let Some(feat) = self.controller.document.features.get(&fid).cloned() else {
            return false;
        };
        let semantic_refs = feat
            .cached_shape
            .and_then(|sid| SemanticReference::from_edge(fid, &self.store, sid, edge_idx).ok())
            .map(|r| vec![r])
            .unwrap_or_default();
        let kind = if chamfer {
            FeatureKind::Chamfer {
                input: fid,
                distance: ScalarParam::mm(1.0),
                edge_indices: vec![edge_idx],
                semantic_refs,
            }
        } else {
            FeatureKind::Fillet {
                input: fid,
                radius: ScalarParam::mm(1.5),
                edge_indices: vec![edge_idx],
                semantic_refs,
            }
        };
        let base = if chamfer { "Chamfer" } else { "Fillet" };
        self.execute_cmd(CadCommand::AddFeature {
            name: format!("{base}_Edge{edge_idx}"),
            kind,
            visible: true,
            color_rgba: feat.color_rgba,
        });
        true
    }

    fn fit_all(&mut self) {
        if let Some(bbox) = self.controller.document.visible_bounding_box() {
            self.camera.fit_bounding_box(&bbox);
            self.viewport_dirty = true;
        }
    }

    fn render_ribbon(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            for (tab, label) in [(RibbonTab::Solid, "ソリッド"), (RibbonTab::View, "表示")] {
                if ui.selectable_label(self.ribbon_tab == tab, RichText::new(label).size(13.0)).clicked() {
                    self.ribbon_tab = tab;
                }
            }
        });
        ui.separator();

        let edge_selected = self.render_config.selected_edge.is_some();
        ui.horizontal(|ui| match self.ribbon_tab {
            RibbonTab::Solid => {
                ribbon_group(ui, "スケッチ", |ui| {
                    if ribbon_button(ui, "✏", "スケッチ", true, "2Dで輪郭を描いて押し出し") {
                        self.start_sketch();
                    }
                });
                ribbon_group(ui, "作成", |ui| {
                    if ribbon_button(ui, "■", "直方体", true, "直方体を追加") {
                        self.add_primitive(
                            "Box",
                            FeatureKind::Box {
                                width: ScalarParam::mm(40.0),
                                height: ScalarParam::mm(40.0),
                                depth: ScalarParam::mm(20.0),
                            },
                            [0.35, 0.65, 0.90, 1.0],
                        );
                    }
                    if ribbon_button(ui, "◉", "円柱", true, "円柱を追加") {
                        self.add_primitive(
                            "Cylinder",
                            FeatureKind::Cylinder {
                                radius: ScalarParam::mm(15.0),
                                height: ScalarParam::mm(40.0),
                            },
                            [0.45, 0.78, 0.55, 1.0],
                        );
                    }
                    if ribbon_button(ui, "●", "球", true, "球を追加") {
                        self.add_primitive(
                            "Sphere",
                            FeatureKind::Sphere {
                                radius: ScalarParam::mm(20.0),
                            },
                            [0.85, 0.55, 0.35, 1.0],
                        );
                    }
                    if ribbon_button(ui, "▲", "円錐", true, "円錐を追加") {
                        self.add_primitive(
                            "Cone",
                            FeatureKind::Cone {
                                bottom_radius: ScalarParam::mm(20.0),
                                top_radius: ScalarParam::mm(5.0),
                                height: ScalarParam::mm(35.0),
                            },
                            [0.75, 0.50, 0.85, 1.0],
                        );
                    }
                });
                ribbon_group(ui, "修正", |ui| {
                    let hint = if edge_selected {
                        "選択中の辺に適用"
                    } else {
                        "先に3Dビューで辺をクリックして選択してください"
                    };
                    if ribbon_button(ui, "◜", "フィレット", edge_selected, hint) {
                        self.add_edge_modifier_on_selection(false);
                    }
                    if ribbon_button(ui, "◸", "面取り", edge_selected, hint) {
                        self.add_edge_modifier_on_selection(true);
                    }
                });
            }
            RibbonTab::View => {
                ribbon_group(ui, "視点", |ui| {
                    for (icon, label, view) in [
                        ("◆", "等角", StandardView::Isometric),
                        ("▔", "上面", StandardView::Top),
                        ("▯", "正面", StandardView::Front),
                        ("▷", "右側面", StandardView::Right),
                    ] {
                        if ribbon_button(ui, icon, label, true, label) {
                            self.camera.set_standard_view(view);
                            self.viewport_dirty = true;
                        }
                    }
                    if ribbon_button(ui, "⛶", "全体表示", true, "全体が収まるように表示 [F]") {
                        self.fit_all();
                    }
                });
                ribbon_group(ui, "表示方法", |ui| {
                    for (icon, label, mode) in [
                        ("▣", "面+辺", DisplayMode::ShadedWithEdges),
                        ("■", "面のみ", DisplayMode::Shaded),
                        ("□", "線画", DisplayMode::Wireframe),
                    ] {
                        if ribbon_button(ui, icon, label, self.render_config.display_mode != mode, label) {
                            self.render_config.display_mode = mode;
                            self.viewport_dirty = true;
                        }
                    }
                    let proj = match self.camera.projection {
                        ProjectionMode::Perspective { .. } => "透視",
                        ProjectionMode::Orthographic { .. } => "平行",
                    };
                    if ribbon_button(ui, "◫", proj, true, "透視投影 / 平行投影を切り替え") {
                        self.camera.toggle_projection();
                        self.viewport_dirty = true;
                    }
                    if ribbon_button(ui, "▦", "グリッド", true, "グリッドと座標軸の表示切替") {
                        self.render_config.show_grid = !self.render_config.show_grid;
                        self.viewport_dirty = true;
                    }
                });
            }
        });
    }

    /// Existing visible geometry projected onto the current sketch plane, taken from each
    /// feature's tessellated B-Rep edge polylines: returns the outline segments to draw
    /// as a reference underlay, plus snap targets at every edge's true endpoints (B-Rep
    /// vertices) and midpoints.
    fn sketch_reference_geometry(&self) -> (Vec<[[f64; 2]; 2]>, Vec<[f64; 2]>) {
        let frame = self.sketch_frame;
        let local = |p: [f32; 3]| {
            frame.to_local(glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64))
        };
        let mut segments = Vec::new();
        let mut snaps = Vec::new();
        for feat in self.controller.document.features.values() {
            if !feat.visible {
                continue;
            }
            let Some(mesh) = feat.cached_mesh.as_ref() else {
                continue;
            };
            let mut edges: std::collections::BTreeMap<u32, Vec<[[f64; 2]; 2]>> =
                std::collections::BTreeMap::new();
            for (seg, &eid) in mesh.edge_segments.iter().zip(&mesh.edge_segment_ids) {
                let s = [local(seg[0]), local(seg[1])];
                segments.push(s);
                edges.entry(eid).or_default().push(s);
            }
            for segs in edges.values() {
                let (Some(first), Some(last)) = (segs.first(), segs.last()) else {
                    continue;
                };
                snaps.push(first[0]);
                snaps.push(last[1]);
                if let Some(mid) = segs.get(segs.len() / 2) {
                    snaps.push(if segs.len() % 2 == 0 {
                        mid[0]
                    } else {
                        [(mid[0][0] + mid[1][0]) * 0.5, (mid[0][1] + mid[1][1]) * 0.5]
                    });
                }
            }
        }
        (segments, snaps)
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

    }

    /// Horizontal, chronological timeline along the bottom of the window (like the
    /// history bar in Fusion-style CAD): one chip per feature in creation order, flat
    /// rather than a nested tree (UX concept B).
    fn render_bottom_timeline(&mut self, ui: &mut egui::Ui) {
        let features_list: Vec<_> = self
            .controller
            .document
            .features
            .values()
            .cloned()
            .collect();
        let mut visibility_toggle: Option<(FeatureId, bool)> = None;
        let mut chip_action: Option<ChipAction> = None;

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

        ui.horizontal(|ui| {
            ui.strong("🕒 タイムライン");
            if selected.is_some() && (!direct_upstream.is_empty() || !direct_downstream.is_empty()) {
                ui.small("  ← 青: 選択中が参照している元 / 橙: それを参照している先 →");
            }
        });
        egui::ScrollArea::horizontal().show(ui, |ui| {
            ui.horizontal(|ui| {
                for (i, feat) in features_list.iter().enumerate() {
                    if i > 0 {
                        ui.label(RichText::new("›").weak());
                    }
                    let (badge, badge_color) = match &feat.state {
                        FeatureState::Clean => ("✓", Color32::from_rgb(80, 210, 120)),
                        FeatureState::Dirty => ("⟳", Color32::from_rgb(230, 190, 70)),
                        FeatureState::Computing => ("…", Color32::from_rgb(100, 180, 240)),
                        FeatureState::Failed(_) => ("✗", Color32::from_rgb(240, 85, 85)),
                        FeatureState::Blocked { .. } => ("⊘", Color32::from_rgb(235, 140, 60)),
                    };
                    let is_selected = selected == Some(feat.id);
                    let stroke_color = if is_selected {
                        Color32::from_rgb(250, 200, 60)
                    } else if direct_upstream.contains(&feat.id) {
                        Color32::from_rgb(120, 180, 255)
                    } else if direct_downstream.contains(&feat.id) {
                        Color32::from_rgb(255, 180, 110)
                    } else {
                        Color32::from_rgb(60, 66, 78)
                    };
                    egui::Frame::group(ui.style())
                        .stroke(egui::Stroke::new(1.5_f32, stroke_color))
                        .inner_margin(egui::Margin::symmetric(6, 3))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                let mut vis = feat.visible;
                                if ui.checkbox(&mut vis, "").on_hover_text("表示 / 非表示").changed() {
                                    visibility_toggle = Some((feat.id, vis));
                                }
                                if feat.suppressed {
                                    ui.colored_label(Color32::from_rgb(150, 150, 160), "⏸");
                                } else {
                                    ui.colored_label(badge_color, badge);
                                }
                                let name = if feat.suppressed {
                                    RichText::new(&feat.name).strikethrough().weak()
                                } else {
                                    RichText::new(&feat.name)
                                };
                                let resp = ui.selectable_label(is_selected, name);
                                let resp = if let FeatureState::Failed(msg) = &feat.state {
                                    resp.on_hover_text(msg)
                                } else if feat.suppressed {
                                    resp.on_hover_text("抑制中（形状から一時的に外れています）")
                                } else {
                                    resp.on_hover_text("右クリック: 名前変更・抑制・削除")
                                };
                                if resp.clicked() {
                                    self.render_config.selected_feature = Some(feat.id);
                                    self.render_config.selected_face = None;
                                    self.render_config.selected_edge = None;
                                    self.viewport_dirty = true;
                                }
                                resp.context_menu(|ui| {
                                    if ui.button("✎ 名前を変更…").clicked() {
                                        chip_action = Some(ChipAction::Rename(feat.id));
                                        ui.close_menu();
                                    }
                                    let label = if feat.suppressed { "▶ 抑制を解除" } else { "⏸ 抑制（一時的に外す）" };
                                    if ui.button(label).clicked() {
                                        chip_action = Some(ChipAction::ToggleSuppress(feat.id));
                                        ui.close_menu();
                                    }
                                    ui.separator();
                                    if ui.button("🗑 削除").clicked() {
                                        chip_action = Some(ChipAction::Delete(feat.id));
                                        ui.close_menu();
                                    }
                                });
                            });
                        });
                }
            });
        });

        match chip_action {
            Some(ChipAction::Rename(id)) => {
                self.rename_target = Some((id, self.feature_name(id)));
            }
            Some(ChipAction::ToggleSuppress(id)) => self.toggle_suppressed(id),
            Some(ChipAction::Delete(id)) => self.delete_feature(id),
            None => {}
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
                let upstream_name = self.feature_name(*upstream_failed);
                let suppressed = self
                    .controller
                    .document
                    .features
                    .get(upstream_failed)
                    .is_some_and(|f| f.suppressed);
                let msg = if suppressed {
                    format!("状態: 計算待ち（元になる「{upstream_name}」が抑制中のため）")
                } else {
                    format!("状態: 計算待ち（元になる「{upstream_name}」の計算に失敗しているため）")
                };
                ui.colored_label(Color32::from_rgb(240, 150, 65), msg);
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
            FeatureKind::SketchExtrude { depth, .. } | FeatureKind::ExtrudePolygon { depth, .. } => {
                kind_changed |= edit_scalar_param(ui, "押し出し量", depth);
            }
            FeatureKind::ExtrudeCircle { radius, depth } => {
                kind_changed |= edit_scalar_param(ui, "半径", radius);
                kind_changed |= edit_scalar_param(ui, "押し出し量", depth);
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
                    self.add_edge_modifier_on_selection(false);
                }
            }
        }
    }

    /// Flat 2D sketch canvas: click to place profile points on the sketch plane
    /// (millimeters), then close the loop and extrude it into a real feature using
    /// `cad_core`'s `Sketch` (the same constraint-solver-backed profile type the
    /// parametric engine already understands — not a UI-only polygon).
    fn render_sketch_canvas(&mut self, ctx: &Context, ui: &mut egui::Ui) {
        let avail = ui.available_size();
        let (rect, response) =
            ui.allocate_exact_size(avail, Sense::click_and_drag());
        let (reference_segments, reference_snaps) = self.sketch_reference_geometry();

        // View: fitted to the existing geometry on first show, then wheel to zoom and
        // right-drag to pan.
        let (mut center, mut px_per_mm) = *self.sketch_view.get_or_insert_with(|| {
            fit_sketch_view(&reference_segments, &self.sketch_points, rect)
        });
        if response.hovered() {
            let scroll_y = ctx.input(|i| i.smooth_scroll_delta.y);
            if scroll_y.abs() > 0.1 {
                px_per_mm = (px_per_mm * (1.0 + scroll_y * 0.0018).clamp(0.75, 1.35)).clamp(0.1, 60.0);
            }
        }
        if response.dragged_by(egui::PointerButton::Secondary) {
            let d = response.drag_delta();
            center[0] -= (d.x / px_per_mm) as f64;
            center[1] += (d.y / px_per_mm) as f64;
        }
        self.sketch_view = Some((center, px_per_mm));
        let origin = rect.center()
            - Vec2::new(center[0] as f32 * px_per_mm, -(center[1] as f32) * px_per_mm);

        let to_screen = |p: [f64; 2]| -> Pos2 {
            Pos2::new(
                origin.x + p[0] as f32 * px_per_mm,
                origin.y - p[1] as f32 * px_per_mm,
            )
        };
        let to_mm = |p: Pos2| -> [f64; 2] {
            [
                ((p.x - origin.x) / px_per_mm) as f64,
                (-(p.y - origin.y) / px_per_mm) as f64,
            ]
        };

        let painter = ui.painter();
        painter.rect_filled(rect, 0.0, Color32::from_rgb(24, 27, 33));
        // Grid spacing adapts to zoom so lines stay readable (and grid snap follows it).
        let grid_mm: f32 = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0]
            .into_iter()
            .find(|g| g * px_per_mm >= 15.0)
            .unwrap_or(1000.0);
        let grid_px = grid_mm * px_per_mm;
        let mut gx = (origin.x - rect.min.x).rem_euclid(grid_px);
        while gx < rect.width() {
            let x = rect.min.x + gx;
            painter.line_segment(
                [Pos2::new(x, rect.min.y), Pos2::new(x, rect.max.y)],
                egui::Stroke::new(1.0_f32, Color32::from_rgb(38, 42, 50)),
            );
            gx += grid_px;
        }
        let mut gy = (origin.y - rect.min.y).rem_euclid(grid_px);
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

        // Existing geometry is always drawn as a faint top-view outline so the user can
        // see what they are sketching against; snap targets (edge endpoints and
        // midpoints) are marked only while snapping is on.
        const SNAP_RADIUS_PX: f32 = 9.0;
        let outline = egui::Stroke::new(1.2_f32, Color32::from_rgb(110, 125, 145));
        for seg in &reference_segments {
            painter.line_segment([to_screen(seg[0]), to_screen(seg[1])], outline);
        }
        let existing_geometry_snaps = if self.sketch_snap_enabled {
            reference_snaps
        } else {
            Vec::new()
        };
        for p in &existing_geometry_snaps {
            let s = to_screen(*p);
            if rect.contains(s) {
                painter.circle_stroke(s, 3.0, egui::Stroke::new(1.2_f32, Color32::from_rgb(90, 230, 210)));
            }
        }

        if response.clicked()
            && let Some(pos) = response.interact_pointer_pos()
        {
            let mut snapped = to_mm(pos);
            if self.sketch_snap_enabled {
                let mut best: Option<(f32, [f64; 2])> = None;
                let mut consider = |cand: [f64; 2]| {
                    let d = to_screen(cand).distance(pos);
                    if d <= SNAP_RADIUS_PX && best.is_none_or(|(bd, _)| d < bd) {
                        best = Some((d, cand));
                    }
                };
                for p in &self.sketch_points {
                    consider(*p);
                }
                for p in &existing_geometry_snaps {
                    consider(*p);
                }
                let grid_x = (snapped[0] / grid_mm as f64).round() * grid_mm as f64;
                let grid_y = (snapped[1] / grid_mm as f64).round() * grid_mm as f64;
                consider([grid_x, grid_y]);
                if let Some((_, p)) = best {
                    snapped = p;
                }
            }
            self.sketch_points.push(snapped);
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

        let hud_rect = Rect::from_min_size(rect.min + Vec2::new(12.0, 12.0), Vec2::new(440.0, 240.0));
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(hud_rect), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(22, 26, 34, 225))
                .show(ui, |ui| {
                    let plane_label = match &self.sketch_plane {
                        SketchPlane::XY => "XY平面".to_string(),
                        SketchPlane::XZ => "XZ平面".to_string(),
                        SketchPlane::YZ => "YZ平面".to_string(),
                        SketchPlane::Face { feature, .. } => {
                            format!("「{}」の面", self.feature_name(*feature))
                        }
                    };
                    ui.label(format!("✏ スケッチ中: {plane_label}"));
                    ui.small("クリックで頂点を配置 / ホイールで拡大縮小 / 右ドラッグで移動");
                    ui.label(format!("頂点数: {}  (3点以上で押し出し可能)", self.sketch_points.len()));
                    ui.checkbox(
                        &mut self.sketch_snap_enabled,
                        "スナップ (自分の頂点 / 10mmグリッド / 既存形状の角・辺の中点 ○)",
                    );

                    // Inline dimensioning for the most recently drawn segment: typing an
                    // exact length moves the last point along the segment's direction to
                    // match it (the segment's endpoints are what gets written into the
                    // final Sketch, so this is a real, baked-in length constraint).
                    if self.sketch_points.len() >= 2 {
                        let n = self.sketch_points.len();
                        let [ax, ay] = self.sketch_points[n - 2];
                        let [bx, by] = self.sketch_points[n - 1];
                        let (dx, dy) = (bx - ax, by - ay);
                        let mut len = (dx * dx + dy * dy).sqrt();
                        ui.horizontal(|ui| {
                            ui.label("最後の辺の長さ (拘束):");
                            if ui
                                .add(egui::DragValue::new(&mut len).speed(0.5).suffix(" mm").range(0.01..=100000.0))
                                .changed()
                                && (dx.abs() > 1e-9 || dy.abs() > 1e-9)
                            {
                                let seg_len = (dx * dx + dy * dy).sqrt().max(1e-9);
                                let (ux, uy) = (dx / seg_len, dy / seg_len);
                                self.sketch_points[n - 1] = [ax + ux * len, ay + uy * len];
                            }
                        });
                    }

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label("押し出し:");
                        let has_target = self.sketch_target.is_some();
                        let target_hint = match self.sketch_target {
                            Some(t) => format!("「{}」と組み合わせます", self.feature_name(t)),
                            None => "結合・切り取りは、面の上でスケッチするか、先に対象を選択してから使えます".to_string(),
                        };
                        for (op, label) in [
                            (SketchOp::NewBody, "新規"),
                            (SketchOp::Join, "結合"),
                            (SketchOp::Cut, "切り取り"),
                        ] {
                            let enabled = op == SketchOp::NewBody || has_target;
                            if ui
                                .add_enabled(enabled, egui::SelectableLabel::new(self.sketch_op == op, label))
                                .on_hover_text(&target_hint)
                                .on_disabled_hover_text(&target_hint)
                                .clicked()
                            {
                                self.set_sketch_op(op);
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("量:");
                        ui.add(egui::DragValue::new(&mut self.sketch_depth).speed(0.5).suffix(" mm").range(0.1..=100000.0));
                        ui.checkbox(&mut self.sketch_reversed, "方向を反転")
                            .on_hover_text("切り取りでは、既定で面の内側（材料側）へ向かいます");
                        ui.checkbox(&mut self.sketch_symmetric, "両側");
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
                            self.cancel_sketch();
                        }
                    });
                });
        });

        if ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.cancel_sketch();
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

        let (mode, base_name) = match (self.sketch_op, self.sketch_target) {
            (SketchOp::Join, Some(target)) => (ExtrudeMode::Join { target }, "Join"),
            (SketchOp::Cut, Some(target)) => (ExtrudeMode::Cut { target }, "Cut"),
            _ => (ExtrudeMode::NewBody, "Extrude"),
        };
        let color_rgba = self
            .sketch_target
            .filter(|_| mode != ExtrudeMode::NewBody)
            .and_then(|t| self.controller.document.features.get(&t))
            .map(|f| f.color_rgba)
            .unwrap_or([0.55, 0.75, 0.95, 1.0]);

        let before = self.controller.document.features.len();
        self.execute_cmd(CadCommand::AddFeature {
            name: format!("{base_name}_{}", before + 1),
            kind: FeatureKind::SketchExtrude {
                sketch,
                depth: ScalarParam::mm(self.sketch_depth.max(0.1)),
                plane: self.sketch_plane.clone(),
                mode,
                reversed: self.sketch_reversed,
                symmetric: self.sketch_symmetric,
            },
            visible: true,
            color_rgba,
        });

        // The combined result replaces the target body on screen.
        if let (Some(target), true) = (
            self.sketch_target.filter(|_| mode != ExtrudeMode::NewBody),
            self.controller.document.features.len() > before,
        ) {
            let new_feature = self.render_config.selected_feature;
            self.execute_cmd(CadCommand::UpdateFeature {
                id: target,
                name: None,
                kind: None,
                visible: Some(false),
                color_rgba: None,
            });
            self.render_config.selected_feature = new_feature;
        }

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

        let aspect = rect.width() / rect.height().max(1.0);
        let uv = |pos: Pos2| {
            (
                ((pos.x - rect.min.x) / rect.width()).clamp(0.0, 1.0),
                ((pos.y - rect.min.y) / rect.height()).clamp(0.0, 1.0),
            )
        };

        // Choosing a sketch plane: the face under the cursor is highlighted, a click
        // starts the sketch on it.
        if self.sketch_plane_picking {
            let hovered = response.hover_pos().and_then(|pos| {
                let (u, v) = uv(pos);
                pick_document(&self.controller.document, &self.camera, u, v, aspect)
                    .map(|hit| (hit.feature_id, hit.face_index))
            });
            if hovered != self.hover_face {
                self.hover_face = hovered;
                self.render_config.selected_face = hovered;
                self.viewport_dirty = true;
                ctx.request_repaint();
            }
            if response.clicked()
                && let Some((fid, face_index)) = hovered
            {
                self.begin_sketch_on_face(fid, face_index);
            }
            self.render_plane_picker(ui, rect);
            if ctx.input(|i| i.key_pressed(Key::Escape)) {
                self.cancel_sketch();
            }
        }

        // 3D Raycast Picking: single-click selects, double-click also opens
        // a floating on-canvas property editor right at the clicked point
        // (no need to go hunting through a side panel).
        if !self.sketch_plane_picking
            && !self.sketch_mode
            && (response.clicked() || response.double_clicked())
            && let Some(pos) = response.interact_pointer_pos()
        {
            let (uv_x, uv_y) = uv(pos);
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

    /// Prompt shown while choosing a sketch plane: base-plane buttons, or click a face.
    fn render_plane_picker(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let panel = Rect::from_center_size(
            Pos2::new(rect.center().x, rect.min.y + 100.0),
            Vec2::new(460.0, 120.0),
        );
        let mut choice: Option<SketchPlane> = None;
        let mut cancel = false;
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(panel), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(22, 26, 34, 235))
                .show(ui, |ui| {
                    ui.strong("✏ スケッチする平面を選んでください");
                    ui.label("形状の平らな面をクリックすると、その面の上に描けます（カーソルの下の面が光ります）");
                    ui.horizontal(|ui| {
                        ui.label("基準平面:");
                        for (label, plane) in [
                            ("XY (上から)", SketchPlane::XY),
                            ("XZ (正面)", SketchPlane::XZ),
                            ("YZ (右から)", SketchPlane::YZ),
                        ] {
                            if ui.button(label).clicked() {
                                choice = Some(plane);
                            }
                        }
                        if ui.button("キャンセル (Esc)").clicked() {
                            cancel = true;
                        }
                    });
                });
        });
        if let Some(plane) = choice {
            let target = self.render_config.selected_feature;
            self.begin_sketch(plane, target);
        } else if cancel {
            self.cancel_sketch();
        }
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
                        FeatureKind::SketchExtrude { depth, .. }
                        | FeatureKind::ExtrudePolygon { depth, .. } => {
                            kind_changed |= edit_scalar_param(ui, "押し出し量", depth);
                        }
                        FeatureKind::ExtrudeCircle { radius, depth } => {
                            kind_changed |= edit_scalar_param(ui, "半径", radius);
                            kind_changed |= edit_scalar_param(ui, "押し出し量", depth);
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

        egui::TopBottomPanel::top("ribbon").show(ctx, |ui| {
            ui.add_enabled_ui(!self.sketch_mode, |ui| self.render_ribbon(ui));
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

        egui::TopBottomPanel::bottom("bottom_timeline")
            .resizable(false)
            .show(ctx, |ui| {
                self.render_bottom_timeline(ui);
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

        if let Some((id, mut name)) = self.rename_target.take() {
            let mut keep_open = true;
            let mut commit = false;
            egui::Window::new("名前を変更")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .show(ctx, |ui| {
                    let resp = ui.text_edit_singleline(&mut name);
                    resp.request_focus();
                    if resp.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                        commit = true;
                    }
                    ui.horizontal(|ui| {
                        if ui.button("変更").clicked() {
                            commit = true;
                        }
                        if ui.button("キャンセル").clicked() || ui.input(|i| i.key_pressed(Key::Escape)) {
                            keep_open = false;
                        }
                    });
                });
            let trimmed = name.trim().to_string();
            if commit && !trimmed.is_empty() {
                self.execute_cmd(CadCommand::UpdateFeature {
                    id,
                    name: Some(trimmed),
                    kind: None,
                    visible: None,
                    color_rgba: None,
                });
            } else if keep_open && !commit {
                self.rename_target = Some((id, name));
            }
        }

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

    /// Drives the M2 flow through the app: pick the plate's top face, draw a 10x10
    /// square, choose 切り取り, finish. The cut becomes the visible body, the plate is
    /// hidden, and deleting the plate is refused with an explanation.
    #[test]
    fn m2_sketch_on_top_face_and_cut_through() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let plate = FeatureId(5);
        let shape = app.controller.document.features[&plate].cached_shape.unwrap();
        let top = app
            .store
            .resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99)
            .unwrap();
        let before_vol = app.controller.document.features[&plate]
            .cached_summary
            .unwrap()
            .volume_mm3;

        app.start_sketch();
        assert!(app.sketch_plane_picking);
        app.begin_sketch_on_face(plate, top.index);
        assert!(app.sketch_mode && !app.sketch_plane_picking);
        assert_eq!(app.sketch_target, Some(plate));
        // On a top face, sketch coordinates coincide with world XY.
        assert!(app.sketch_frame.origin.z > 9.99 && app.sketch_frame.normal.z > 0.99);

        app.sketch_points = vec![[40.0, 25.0], [50.0, 25.0], [50.0, 35.0], [40.0, 35.0]];
        app.set_sketch_op(SketchOp::Cut);
        assert!(app.sketch_reversed, "cut should default to going into the material");
        app.finish_sketch_extrude();

        let (&cut_id, cut) = app.controller.document.features.iter().next_back().unwrap();
        assert!(cut.visible && !app.controller.document.features[&plate].visible);
        assert_eq!(app.render_config.selected_feature, Some(cut_id));
        let vol = cut.cached_summary.expect("cut computed").volume_mm3;
        assert!((vol - (before_vol - 1000.0)).abs() < 1e-3, "vol={vol}");

        let features_before = app.controller.document.features.len();
        app.delete_feature(plate);
        assert_eq!(app.controller.document.features.len(), features_before);
        assert!(app.status_logs.last().unwrap().contains("使われているため削除できません"));
    }
}
