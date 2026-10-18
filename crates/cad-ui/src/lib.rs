//! Desktop CAD GUI for FerroCAD powered by `egui` / `eframe` and `cad-render` (Phase 3 & Phase 4).

mod explain;
mod files;
mod measure;
mod rebind;
mod selection;

pub use files::default_app_dir;
mod sketch_editor;
mod tools;

use std::path::PathBuf;

use cad_core::{
    CadCommand, CoreError, Document, DocumentController, ExtrudeMode, FeatureId, FeatureKind,
    FeatureState, ParameterValue, PatternLayout, PlaneFrame, ScalarParam, SemanticReference, Sketch, SketchPlane,
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
    Tools,
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
        let row = ui.horizontal(add_contents).response.rect;
        // Centre the title under this group's buttons only (a centred layout would
        // otherwise stretch to the full ribbon width and push later groups off-screen).
        ui.allocate_ui_with_layout(
            Vec2::new(row.width(), 14.0),
            egui::Layout::top_down(egui::Align::Center),
            |ui| ui.label(RichText::new(title).size(11.0).weak()),
        );
    });
    ui.separator();
}

/// Named, user-facing scalar inputs of a feature. This is deliberately separate from
/// `referenced_parameters`: the UI needs to explain *which dimension* uses a name.
fn feature_parameter_slots(kind: &FeatureKind) -> Vec<(&'static str, &ScalarParam)> {
    match kind {
        FeatureKind::Box { width, height, depth } => {
            vec![("幅", width), ("高さ", height), ("厚み", depth)]
        }
        FeatureKind::Cylinder { radius, height } => vec![("半径", radius), ("高さ", height)],
        FeatureKind::Sphere { radius } => vec![("半径", radius)],
        FeatureKind::Cone { bottom_radius, top_radius, height } => {
            vec![("底面半径", bottom_radius), ("上面半径", top_radius), ("高さ", height)]
        }
        FeatureKind::SketchExtrude { depth, .. } | FeatureKind::ExtrudePolygon { depth, .. } => {
            vec![("押し出し量", depth)]
        }
        FeatureKind::SketchRevolve { angle_deg, .. } | FeatureKind::RevolvePolygon { angle_deg, .. } => {
            vec![("回転角", angle_deg)]
        }
        FeatureKind::Loft { distance, .. } => vec![("断面間距離", distance)],
        FeatureKind::ExtrudeCircle { radius, depth } => vec![("半径", radius), ("押し出し量", depth)],
        FeatureKind::Transform { dx, dy, dz, rot_angle_deg, .. } => vec![
            ("移動 X", dx),
            ("移動 Y", dy),
            ("移動 Z", dz),
            ("回転角", rot_angle_deg),
        ],
        FeatureKind::Fillet { radius, .. } => vec![("フィレット半径", radius)],
        FeatureKind::Chamfer { distance, .. } => vec![("面取り距離", distance)],
        FeatureKind::Draft { angle_deg, .. } => vec![("ドラフト角", angle_deg)],
        FeatureKind::LinearPattern { spacing_x, spacing_y, spacing_z, .. } => {
            vec![("ピッチ X", spacing_x), ("ピッチ Y", spacing_y), ("ピッチ Z", spacing_z)]
        }
        FeatureKind::Pattern { layout, .. } => match layout {
            PatternLayout::Circular { total_angle_deg, .. } => vec![("全体の角度", total_angle_deg)],
            PatternLayout::Linear { spacing, .. } => vec![("間隔", spacing)],
        },
        FeatureKind::Shell { thickness, .. } => vec![("壁の厚さ", thickness)],
        FeatureKind::Mirror { .. }
        | FeatureKind::Boolean { .. }
        | FeatureKind::Heal { .. }
        | FeatureKind::ImportedBody { .. } => Vec::new(),
    }
}

fn scalar_expression(param: &ScalarParam) -> Option<&str> {
    match param {
        ScalarParam::Expr(expr) => Some(expr),
        ScalarParam::Literal(_) => None,
    }
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
    pub new_param_unit: String,
    pub new_param_description: String,
    pub current_file_path: Option<PathBuf>,
    /// Screen position where the floating on-canvas property editor should appear
    /// (set by double-clicking a face/edge directly in the 3D viewport).
    pub floating_editor_pos: Option<Pos2>,
    /// When true, the central viewport switches from the 3D render to the 2D sketch editor,
    /// which edits `sketch` (a constraint-solved `cad_core::Sketch`) directly.
    pub sketch_mode: bool,
    pub sketch: Sketch,
    pub sketch_tool: SketchTool,
    /// Clicks collected by the active drawing tool (position + existing point snapped to).
    pub sketch_pending: Vec<([f64; 2], Option<cad_core::PointId>)>,
    pub sketch_selection: Vec<SketchSel>,
    pub sketch_report: Option<cad_core::SketchSolveReport>,
    pub sketch_drag: Option<cad_core::PointId>,
    /// Dimension being edited in place: constraint index and its value.
    pub sketch_dim_edit: Option<(usize, f64)>,
    /// Feature whose sketch is being re-edited (`None` = creating a new feature).
    pub sketch_editing: Option<FeatureId>,
    pub sketch_depth: f64,
    /// Depth expression kept verbatim when re-editing a feature whose depth is a formula.
    pub sketch_depth_expr: Option<String>,
    /// Snap new sketch points to the sketch's own points, the grid, and the corners and
    /// edge midpoints of existing geometry projected onto the sketch plane.
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
    /// Finish the sketch as a revolve instead of an extrude.
    pub sketch_revolve: bool,
    pub sketch_axis: cad_core::SketchAxis,
    pub sketch_angle: f64,
    pub sketch_through_all: bool,
    /// Open "select → adjust → confirm" panel for a modify tool.
    pub tool_panel: Option<ToolPanel>,
    pub sketch_start: SketchStart,
    /// Re-picking a feature's face/edge reference in the viewport.
    pub rebind: Option<rebind::RebindState>,
    /// Folder for config, recent files and autosave (`None` = persistence off, as in tests).
    pub app_dir: Option<PathBuf>,
    pub recent_files: Vec<PathBuf>,
    /// Document revision at the last save (or load); differs when there are unsaved changes.
    pub saved_revision: u64,
    pub autosaved_revision: u64,
    pub last_autosave: Option<std::time::Instant>,
    /// Autosaved work found at startup, with the file it belonged to.
    pub recovery_offer: Option<Option<PathBuf>>,
    pub pending_action: Option<files::PendingAction>,
    pub allow_close: bool,
    pub measure: Option<measure::MeasureState>,
    /// A delete waiting for confirmation: the feature and everything built on it.
    pub delete_confirm: Option<(FeatureId, Vec<FeatureId>)>,
    last_title: String,
}

/// Which modify tool a ribbon button opens.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToolKind {
    Transform,
    CircularPattern,
    LinearPattern,
    Mirror,
    Boolean,
    Shell,
    /// Start a sketch on a plane parallel to the currently selected face.
    OffsetPlane,
    Draft,
    Heal,
    Loft,
}

/// What the sketch being started will become.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchStart {
    Extrude,
    Revolve,
    Hole,
}

/// State of an open modify-tool panel.
#[derive(Debug, Clone, PartialEq)]
pub enum ToolPanel {
    Transform {
        input: FeatureId,
        /// When set, the transform is inserted immediately after this feature so an
        /// existing Boolean can keep using the moved tool instead of moving its result.
        insert_after: Option<FeatureId>,
        translation: [f64; 3],
        rotation_axis: usize,
        rotation_angle_deg: f64,
    },
    CircularPattern {
        input: FeatureId,
        axis: usize,
        center: [f64; 3],
        count: u32,
        angle: f64,
    },
    LinearPattern {
        input: FeatureId,
        direction: usize,
        spacing: f64,
        count: u32,
    },
    Mirror {
        input: FeatureId,
        plane: usize,
        merge: bool,
    },
    Boolean {
        a: FeatureId,
        b: Option<FeatureId>,
        op: cad_core::BooleanOp,
    },
    Shell {
        input: FeatureId,
        thickness: f64,
    },
    OffsetPlane {
        input: FeatureId,
        face: SemanticReference,
        offset: f64,
    },
    Draft {
        input: FeatureId,
        faces: Vec<SemanticReference>,
        neutral_index: usize,
        angle_deg: f64,
        reversed: bool,
    },
    Heal { input: FeatureId },
    Loft { lower: FeatureId, upper: Option<FeatureId>, distance: f64 },
}

/// Action chosen from a timeline chip's right-click menu.
enum ChipAction {
    Rebind(FeatureId),
    RollbackTo(FeatureId),
    EditSketch(FeatureId),
    Rename(FeatureId),
    ToggleSuppress(FeatureId),
    Delete(FeatureId),
}

/// Active tool in the sketch editor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchTool {
    Select,
    Line,
    Rect,
    Circle,
    Arc,
}

/// Something selected in the sketch editor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SketchSel {
    Point(cad_core::PointId),
    Segment(cad_core::PointId, cad_core::PointId),
    Curve(cad_core::EntityId),
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
            ..Default::default()
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
            new_param_unit: "mm".to_string(),
            new_param_description: String::new(),
            current_file_path: None,
            floating_editor_pos: None,
            sketch_mode: false,
            sketch: Sketch::new("Sketch"),
            sketch_tool: SketchTool::Line,
            sketch_pending: Vec::new(),
            sketch_selection: Vec::new(),
            sketch_report: None,
            sketch_drag: None,
            sketch_dim_edit: None,
            sketch_editing: None,
            sketch_depth: 10.0,
            sketch_depth_expr: None,
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
            sketch_revolve: false,
            sketch_axis: cad_core::SketchAxis::Y,
            sketch_angle: 360.0,
            sketch_through_all: false,
            tool_panel: None,
            sketch_start: SketchStart::Extrude,
            rebind: None,
            app_dir: None,
            recent_files: Vec::new(),
            saved_revision: 0,
            autosaved_revision: 0,
            last_autosave: None,
            recovery_offer: None,
            pending_action: None,
            allow_close: false,
            measure: None,
            delete_confirm: None,
            last_title: String::new(),
        }
        .mark_saved()
    }

    fn mark_saved(mut self) -> Self {
        self.saved_revision = self.controller.document.revision;
        self.autosaved_revision = self.saved_revision;
        self
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
    fn start_sketch_with(&mut self, kind: SketchStart) {
        self.sketch_start = kind;
        self.sketch_plane_picking = true;
        self.hover_face = None;
        self.floating_editor_pos = None;
        self.tool_panel = None;
    }

    fn cancel_sketch(&mut self) {
        self.sketch_plane_picking = false;
        self.sketch_mode = false;
        self.viewport_dirty = true;
        self.sketch_editing = None;
        self.sketch_pending.clear();
        self.sketch_selection.clear();
        if self.hover_face.take().is_some() {
            self.render_config.selected_face = None;
            self.render_config.selected_faces.clear();
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
        self.render_config.selected_faces.clear();
        self.sketch_plane = plane;
        self.sketch_frame = frame;
        self.sketch_target = target;
        // Sketching on a face usually means adding to that body; a base-plane sketch
        // usually starts a new one.
        self.set_sketch_op(if target.is_some()
            && matches!(self.sketch_plane, SketchPlane::Face { .. } | SketchPlane::OffsetFace { .. })
        {
            SketchOp::Join
        } else {
            SketchOp::NewBody
        });
        self.sketch_symmetric = false;
        self.sketch_depth = 10.0;
        self.sketch_depth_expr = None;
        self.sketch_editing = None;
        self.sketch_revolve = self.sketch_start == SketchStart::Revolve;
        self.sketch_axis = cad_core::SketchAxis::Y;
        self.sketch_angle = 360.0;
        self.sketch_through_all = false;
        let name = format!("Sketch_{}", self.controller.document.features.len() + 1);
        self.open_sketch_editor(Sketch::new(name));
        if self.sketch_start == SketchStart::Hole {
            self.sketch_tool = SketchTool::Circle;
            if self.sketch_target.is_some() {
                self.set_sketch_op(SketchOp::Cut);
                self.sketch_through_all = true;
            }
        }
    }

    /// Shows the 2D editor for `sketch` on the already-set plane/frame.
    fn open_sketch_editor(&mut self, sketch: Sketch) {
        self.sketch = sketch;
        self.sketch_report = Some(self.sketch.clone().solve(60, 1e-9));
        self.sketch_tool = if self.sketch.entities.is_empty() {
            SketchTool::Line
        } else {
            SketchTool::Select
        };
        self.sketch_pending.clear();
        self.sketch_selection.clear();
        self.sketch_drag = None;
        self.sketch_dim_edit = None;
        self.sketch_mode = true;
        self.sketch_view = None;
        self.viewport_dirty = true;
    }

    /// Re-opens an existing sketch-based feature in the editor; finishing updates that
    /// feature in place so everything downstream recomputes.
    fn edit_sketch_feature(&mut self, id: FeatureId) {
        let Some(feat) = self.controller.document.features.get(&id).cloned() else {
            return;
        };
        let (sketch, plane, mode) = match feat.kind {
            FeatureKind::SketchExtrude {
                sketch,
                depth,
                plane,
                mode,
                reversed,
                symmetric,
                through_all,
            } => {
                self.sketch_revolve = false;
                self.sketch_reversed = reversed;
                self.sketch_symmetric = symmetric;
                self.sketch_through_all = through_all;
                match depth {
                    ScalarParam::Literal(v) => {
                        self.sketch_depth = v;
                        self.sketch_depth_expr = None;
                    }
                    ScalarParam::Expr(e) => self.sketch_depth_expr = Some(e),
                }
                (sketch, plane, mode)
            }
            FeatureKind::SketchRevolve {
                sketch,
                plane,
                axis,
                angle_deg,
                mode,
            } => {
                self.sketch_revolve = true;
                self.sketch_axis = axis;
                self.sketch_angle = match angle_deg {
                    ScalarParam::Literal(v) => v,
                    ScalarParam::Expr(_) => 360.0,
                };
                self.sketch_depth_expr = None;
                (sketch, plane, mode)
            }
            _ => {
                self.status_logs
                    .push(format!("「{}」はスケッチから作られていないため、スケッチ編集はできません", feat.name));
                return;
            }
        };
        let frame = match self.controller.document.sketch_plane_frame(&plane, &self.store) {
            Ok(frame) => frame,
            Err(err) => {
                self.status_logs
                    .push(format!("スケッチ平面を特定できないため編集を開けませんでした: {err}"));
                return;
            }
        };
        self.sketch_plane_picking = false;
        self.floating_editor_pos = None;
        self.sketch_plane = plane;
        self.sketch_frame = frame;
        let (op, target) = match mode {
            ExtrudeMode::NewBody => (SketchOp::NewBody, None),
            ExtrudeMode::Join { target } => (SketchOp::Join, Some(target)),
            ExtrudeMode::Cut { target } => (SketchOp::Cut, Some(target)),
        };
        self.sketch_target = target;
        self.sketch_op = op;
        self.open_sketch_editor(sketch);
        self.sketch_editing = Some(id);
    }

    /// Sets the result type and a sensible default direction: a cut heads toward the
    /// target body, a join heads away from it (works for faces and base planes alike).
    fn set_sketch_op(&mut self, op: SketchOp) {
        self.sketch_op = op;
        let body_side = self
            .sketch_target
            .and_then(|t| self.controller.document.features.get(&t))
            .and_then(|f| f.cached_summary.or(f.last_valid_summary))
            .map(|s| (s.bounds.center() - self.sketch_frame.origin).dot(self.sketch_frame.normal));
        self.sketch_reversed = match (op, body_side) {
            (SketchOp::Cut, Some(side)) => side < 0.0,
            (SketchOp::Join, Some(side)) => side > 0.0,
            _ => false,
        };
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
                    self.render_config.selected_faces.clear();
                    self.render_config.selected_edge = None;
                    self.render_config.selected_edges.clear();
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
        let (fid, edge_indices) = match self.edges_on_one_body() {
            Ok(sel) => sel,
            Err(reason) => {
                self.status_logs.push(reason.to_string());
                return false;
            }
        };
        let Some(feat) = self.controller.document.features.get(&fid).cloned() else {
            return false;
        };
        // One semantic reference per edge, so each can be re-bound on its own later.
        let semantic_refs: Vec<SemanticReference> = match feat.cached_shape {
            Some(sid) => edge_indices
                .iter()
                .filter_map(|&e| SemanticReference::from_edge(fid, &self.store, sid, e).ok())
                .collect(),
            None => Vec::new(),
        };
        let semantic_refs = if semantic_refs.len() == edge_indices.len() {
            semantic_refs
        } else {
            Vec::new()
        };
        let kind = if chamfer {
            FeatureKind::Chamfer {
                input: fid,
                distance: ScalarParam::mm(1.0),
                edge_indices: edge_indices.clone(),
                semantic_refs,
            }
        } else {
            FeatureKind::Fillet {
                input: fid,
                radius: ScalarParam::mm(1.5),
                edge_indices: edge_indices.clone(),
                semantic_refs,
            }
        };
        let base = if chamfer { "Chamfer" } else { "Fillet" };
        let name = if edge_indices.len() == 1 {
            format!("{base}_Edge{}", edge_indices[0])
        } else {
            format!("{base}_{}Edges", edge_indices.len())
        };
        self.execute_cmd(CadCommand::AddFeature {
            name,
            kind,
            visible: true,
            color_rgba: feat.color_rgba,
        });
        // The modified body is what the user now looks at.
        if let Some(new_id) = self.render_config.selected_feature.filter(|&n| n != fid) {
            self.execute_cmd(CadCommand::UpdateFeature {
                id: fid,
                name: None,
                kind: None,
                visible: Some(false),
                color_rgba: None,
            });
            self.render_config.selected_feature = Some(new_id);
        }
        self.clear_selection();
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
            for (tab, label) in [
                (RibbonTab::Solid, "ソリッド"),
                (RibbonTab::View, "表示"),
                (RibbonTab::Tools, "ツール"),
            ] {
                if ui.selectable_label(self.ribbon_tab == tab, RichText::new(label).size(13.0)).clicked() {
                    self.ribbon_tab = tab;
                }
            }
        });
        ui.separator();

        let edge_selection = self.edges_on_one_body();
        let edge_selected = edge_selection.is_ok();
        let edge_hint = match &edge_selection {
            Ok((_, e)) if e.len() == 1 => "選択中の辺に適用します（Shift+クリックで辺を追加）".to_string(),
            Ok((_, e)) => format!("選択中の{}本の辺にまとめて適用します", e.len()),
            Err(reason) => reason.to_string(),
        };
        let feature_selected = self.render_config.selected_feature.is_some();
        let need = "先にタイムラインか3Dビューで対象を選択してください";
        ui.horizontal(|ui| match self.ribbon_tab {
            RibbonTab::Solid => {
                ribbon_group(ui, "スケッチ", |ui| {
                    if ribbon_button(ui, "✏", "押し出し", true, "平面を選んで輪郭を描き、押し出します") {
                        self.start_sketch_with(SketchStart::Extrude);
                    }
                    if ribbon_button(ui, "⟳", "回転", true, "平面を選んで輪郭を描き、軸の周りに回転させます") {
                        self.start_sketch_with(SketchStart::Revolve);
                    }
                    if ribbon_button(ui, "○", "穴", true, "面をクリックして円を描くと、その位置に貫通穴を開けます") {
                        self.start_sketch_with(SketchStart::Hole);
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
                    if ribbon_button(ui, "↔", "移動", feature_selected, if feature_selected { "選択中の形状を XYZ 方向へ移動・回転します" } else { need }) {
                        self.open_tool_panel(ToolKind::Transform);
                    }
                    if ribbon_button(ui, "⌒", "フィレット", edge_selected, &edge_hint) {
                        self.add_edge_modifier_on_selection(false);
                    }
                    if ribbon_button(ui, "◣", "面取り", edge_selected, &edge_hint) {
                        self.add_edge_modifier_on_selection(true);
                    }
                    let face_selected = !self.render_config.selected_faces.is_empty();
                    if ribbon_button(
                        ui,
                        "□",
                        "シェル",
                        face_selected,
                        if face_selected {
                            "選択中の面を開口にして、内側をくり抜きます（Shift+クリックで面を追加）"
                        } else {
                            "先に3Dビューで開口にしたい面をクリックしてください"
                        },
                    ) {
                        self.open_tool_panel(ToolKind::Shell);
                    }
                    if ribbon_button(
                        ui,
                        "▱",
                        "オフセット面",
                        face_selected,
                        if face_selected {
                            "選択面から平行な距離指定面を作り、そこからスケッチを始めます"
                        } else {
                            "先に3Dビューで基準となる平らな面をクリックしてください"
                        },
                    ) {
                        self.open_tool_panel(ToolKind::OffsetPlane);
                    }
                    if ribbon_button(
                        ui,
                        "◢",
                        "ドラフト",
                        self.render_config.selected_faces.len() >= 2,
                        "同じボディの面を2つ以上選び、固定面と傾ける面を指定します",
                    ) {
                        self.open_tool_panel(ToolKind::Draft);
                    }
                    if ribbon_button(ui, "✚", "形状を修復", feature_selected, "選択中の形状を検証し、OCCT Shape Healingを適用します") {
                        self.open_tool_panel(ToolKind::Heal);
                    }
                    if ribbon_button(ui, "⌁", "ロフト", feature_selected, "2つのXYスケッチ断面の間に遷移形状を作ります") {
                        self.open_tool_panel(ToolKind::Loft);
                    }
                });
                ribbon_group(ui, "パターン・組み合わせ", |ui| {
                    if ribbon_button(ui, "⟲", "円形", feature_selected, if feature_selected { "選択中の形状（穴なら穴）を円周上に並べます" } else { need }) {
                        self.open_tool_panel(ToolKind::CircularPattern);
                    }
                    if ribbon_button(ui, "…", "直線", feature_selected, if feature_selected { "選択中の形状（穴なら穴）を一直線に並べます" } else { need }) {
                        self.open_tool_panel(ToolKind::LinearPattern);
                    }
                    if ribbon_button(ui, "∥", "ミラー", feature_selected, if feature_selected { "選択中の形状を平面で鏡映します" } else { need }) {
                        self.open_tool_panel(ToolKind::Mirror);
                    }
                    if ribbon_button(ui, "◆", "ブーリアン", feature_selected, if feature_selected { "2つのボディを足す・引く・重なりを取る" } else { need }) {
                        self.open_tool_panel(ToolKind::Boolean);
                    }
                });
            }
            RibbonTab::Tools => {
                ribbon_group(ui, "読み込み・書き出し", |ui| {
                    if ribbon_button(ui, "▽", "STEP取込", true, "STEPファイルを固定の形状として取り込みます") {
                        self.import_step_dialog();
                    }
                    let selected = self.render_config.selected_feature.is_some();
                    if ribbon_button(
                        ui,
                        "△",
                        "STEP書出",
                        selected,
                        if selected { "選択中のボディをSTEPで書き出します" } else { "先に書き出すボディを選択してください" },
                    ) {
                        self.export_step_dialog();
                    }
                    if ribbon_button(ui, "▲", "STL書出", true, "表示中のボディを3Dプリンタ用のSTLで書き出します") {
                        self.export_stl_dialog();
                    }
                });
                ribbon_group(ui, "検査", |ui| {
                    if ribbon_button(ui, "⊿", "測定", true, "面積・距離・角度を測ります") {
                        self.start_measure();
                    }
                });
            }
            RibbonTab::View => {
                ribbon_group(ui, "視点", |ui| {
                    for (icon, label, view) in [
                        ("◆", "等角", StandardView::Isometric),
                        ("▔", "上面", StandardView::Top),
                        ("□", "正面", StandardView::Front),
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
                        ("▱", "線画", DisplayMode::Wireframe),
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
                    if ribbon_button(ui, "◇", proj, true, "透視投影 / 平行投影を切り替え") {
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

    pub fn recompute_and_refresh(&mut self) {
        match self
            .controller
            .document
            .recompute(&mut self.store, self.tess_options)
        {
            Ok(report) => {
                if !report.failed_features.is_empty() {
                    for (fid, msg) in &report.failed_features {
                        let doc = &self.controller.document;
                        let line = match doc.features.get(fid).and_then(|f| f.failure.as_ref()) {
                            Some(err) => {
                                let e = explain::explain(err, doc);
                                format!("「{}」を計算できません: {} → {}", self.feature_name(*fid), e.summary, e.action)
                            }
                            None => format!("「{}」を計算できません: {msg}", self.feature_name(*fid)),
                        };
                        self.status_logs.push(line);
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
        let idle = !ctx.wants_keyboard_input()
            && !self.sketch_mode
            && !self.sketch_plane_picking
            && self.rebind.is_none()
            && self.measure.is_none()
            && self.tool_panel.is_none()
            && self.delete_confirm.is_none()
            && self.pending_action.is_none()
            && self.floating_editor_pos.is_none();
        if idle && ctx.input(|i| i.key_pressed(Key::Delete))
            && let Some(id) = self.render_config.selected_feature
        {
            self.request_delete(id);
        }
        if idle && ctx.input(|i| i.key_pressed(Key::Escape)) {
            self.clear_selection();
        }
        if ctx.input_mut(|i| i.consume_key(Modifiers::CTRL, Key::S)) {
            self.save_document(None);
        }
        if ctx.input_mut(|i| i.consume_key(Modifiers::CTRL, Key::O)) {
            self.open_dialog();
        }
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
                if ui.button("新規 (New)").clicked() {
                    self.request(files::PendingAction::New);
                    ui.close_menu();
                }
                if ui.button("開く… (Ctrl+O)").clicked() {
                    ui.close_menu();
                    self.open_dialog();
                }
                ui.menu_button("最近使ったファイル", |ui| {
                    if self.recent_files.is_empty() {
                        ui.weak("（まだありません）");
                    }
                    for path in self.recent_files.clone() {
                        let label = path
                            .file_name()
                            .map(|n| n.to_string_lossy().into_owned())
                            .unwrap_or_default();
                        if ui.button(label).on_hover_text(path.display().to_string()).clicked() {
                            self.request(files::PendingAction::Open(path));
                            ui.close_menu();
                        }
                    }
                });
                ui.separator();
                if ui.button("保存 (Ctrl+S)").clicked() {
                    ui.close_menu();
                    self.save_document(None);
                }
                if ui.button("名前を付けて保存…").clicked() {
                    ui.close_menu();
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("FerroCAD", &["fcad"])
                        .set_file_name(format!("{}.fcad", self.controller.document.name))
                        .save_file()
                    {
                        self.save_document(Some(path));
                    }
                }
                ui.separator();
                if ui.button("STEPを取り込む…").clicked() {
                    ui.close_menu();
                    self.import_step_dialog();
                }
                if ui.button("STLで書き出す…（3Dプリンタ用）").clicked() {
                    ui.close_menu();
                    self.export_stl_dialog();
                }
                if ui.button("選択中のボディをSTEPで書き出す…").clicked() {
                    ui.close_menu();
                    self.export_step_dialog();
                }
                ui.separator();
                if ui.button("デモ（穴あきプレート）を開く").clicked() {
                    self.request(files::PendingAction::Demo);
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
        let parameter_uses: std::collections::BTreeMap<String, Vec<String>> = self
            .controller
            .document
            .features
            .values()
            .flat_map(|feature| {
                feature_parameter_slots(&feature.kind)
                    .into_iter()
                    .flat_map(move |(label, scalar)| {
                        scalar_expression(scalar)
                            .into_iter()
                            .flat_map(move |expr| {
                                scalar
                                    .referenced_parameters()
                                    .into_iter()
                                    .map(move |name| {
                                        (name, format!("{} → {}（{}）", feature.name, label, expr))
                                    })
                            })
                    })
            })
            .fold(std::collections::BTreeMap::new(), |mut uses, (name, slot)| {
                uses.entry(name).or_default().push(slot);
                uses
            });
        let parameter_dependents: std::collections::BTreeMap<String, Vec<String>> = self
            .controller
            .document
            .parameters
            .values()
            .flat_map(|parameter| {
                parameter
                    .value
                    .referenced_parameters()
                    .into_iter()
                    .map(move |name| (name, parameter.name.clone()))
            })
            .fold(std::collections::BTreeMap::new(), |mut dependents, (name, dependent)| {
                dependents.entry(name).or_default().push(dependent);
                dependents
            });

        let mut pending_param_cmd: Option<CadCommand> = None;
        let mut remove_parameter: Option<String> = None;

        egui::Grid::new("param_grid")
            .striped(true)
            .num_columns(3)
            .show(ui, |ui| {
                ui.strong("名前");
                ui.strong("値 / 数式");
                ui.strong("評価値 / 使用先");
                ui.end_row();

                for p in params_snapshot {
                    let feature_uses = parameter_uses.get(&p.name).cloned().unwrap_or_default();
                    let expression_dependents = parameter_dependents.get(&p.name).cloned().unwrap_or_default();
                    let can_remove = feature_uses.is_empty() && expression_dependents.is_empty();
                    ui.horizontal(|ui| {
                        ui.label(&p.name).on_hover_text(&p.description);
                        let hint = if can_remove {
                            "この未使用パラメータを削除します"
                        } else {
                            "使用中のため削除できません。使用しているフィーチャーや数式を先に変更してください"
                        };
                        if ui
                            .add_enabled(can_remove, egui::Button::new("×").small())
                            .on_hover_text(hint)
                            .clicked()
                        {
                            remove_parameter = Some(p.name.clone());
                        }
                    });
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
                    ui.vertical(|ui| {
                        ui.label(format!("{eval_val:.2} {}", p.unit));
                        match parameter_uses.get(&p.name) {
                            Some(slots) => ui.small(format!("使用先: {}", slots.join("、"))),
                            None => ui.small("未使用"),
                        };
                        if let Some(dependents) = parameter_dependents.get(&p.name) {
                            ui.small(format!("数式で使用: {}", dependents.join("、")));
                        }
                    });
                    ui.end_row();
                }
            });

        if let Some(cmd) = pending_param_cmd {
            self.execute_cmd(cmd);
        }
        if let Some(name) = remove_parameter {
            self.execute_cmd(CadCommand::RemoveParameter { name });
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
            ui.horizontal(|ui| {
                ui.label("単位:");
                ui.text_edit_singleline(&mut self.new_param_unit);
            });
            ui.horizontal(|ui| {
                ui.label("説明:");
                ui.text_edit_singleline(&mut self.new_param_description);
            });
            ui.small("例: plate_width / 100 / mm、または hole_offset / plate_width * 0.15 / mm");
            if ui.button("追加").clicked() && !self.new_param_name.trim().is_empty() {
                let val = if let Ok(num) = self.new_param_expr.trim().parse::<f64>() {
                    ParameterValue::Number(num)
                } else {
                    ParameterValue::Expression(self.new_param_expr.trim().to_string())
                };
                self.execute_cmd(CadCommand::SetParameter {
                    name: self.new_param_name.trim().to_string(),
                    value: val,
                    unit: if self.new_param_unit.trim().is_empty() {
                        "mm".to_string()
                    } else {
                        self.new_param_unit.trim().to_string()
                    },
                    description: if self.new_param_description.trim().is_empty() {
                        "ユーザー定義パラメータ".to_string()
                    } else {
                        self.new_param_description.trim().to_string()
                    },
                });
                self.new_param_name.clear();
                self.new_param_expr.clear();
                self.new_param_unit = "mm".to_string();
                self.new_param_description.clear();
            }
        });

    }

    /// Horizontal, chronological timeline along the bottom of the window (like the
    /// history bar in Fusion-style CAD): one chip per feature in creation order, flat
    /// rather than a nested tree (UX concept B).
    fn render_bottom_timeline(&mut self, ui: &mut egui::Ui) {
        let doc = &self.controller.document;
        let features_list: Vec<_> = doc
            .timeline()
            .into_iter()
            .filter_map(|id| doc.features.get(&id).cloned())
            .collect();
        let rollback = doc.rollback;
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
        // Everything downstream (transitively): what a change to the selection affects.
        let direct_downstream: std::collections::HashSet<FeatureId> = selected
            .map(|sid| self.controller.document.downstream_of(sid).into_iter().collect())
            .unwrap_or_default();

        let mut roll_forward = false;
        ui.horizontal(|ui| {
            ui.strong("🕒 タイムライン");
            if rollback.is_some() {
                ui.colored_label(
                    Color32::from_rgb(240, 190, 80),
                    "巻き戻し中：薄い部分は一時的に外れています。新しい操作は ▌ の位置に入ります",
                );
                if ui.button("▶ 最後まで進める").clicked() {
                    roll_forward = true;
                }
            } else if selected.is_some() && (!direct_upstream.is_empty() || !direct_downstream.is_empty()) {
                ui.small("  ← 青: 選択中が参照している元 / 橙: 変更すると影響を受ける先 →");
            }
        });
        egui::ScrollArea::horizontal().show(ui, |ui| {
            ui.horizontal(|ui| {
                let mut past_marker = false;
                for (i, feat) in features_list.iter().enumerate() {
                    if i > 0 {
                        ui.label(RichText::new("›").weak());
                    }
                    let rolled = past_marker;
                    if rolled {
                        ui.set_opacity(0.4);
                    }
                    let (badge, badge_color) = match &feat.state {
                        FeatureState::Clean => ("✓", Color32::from_rgb(80, 210, 120)),
                        FeatureState::Dirty => ("⟳", Color32::from_rgb(230, 190, 70)),
                        FeatureState::Computing => ("…", Color32::from_rgb(100, 180, 240)),
                        FeatureState::Failed(_) => ("×", Color32::from_rgb(240, 85, 85)),
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
                                let is_sketch = matches!(feat.kind, FeatureKind::SketchExtrude { .. } | FeatureKind::SketchRevolve { .. });
                                let resp = ui.selectable_label(is_selected, name);
                                let resp = if rolled {
                                    resp.on_hover_text("巻き戻し中（計算から一時的に外れています）")
                                } else if let FeatureState::Failed(msg) = &feat.state {
                                    let text = match &feat.failure {
                                        Some(err) => {
                                            let e = explain::explain(err, &self.controller.document);
                                            format!("{}
→ {}", e.summary, e.action)
                                        }
                                        None => msg.clone(),
                                    };
                                    resp.on_hover_text(text)
                                } else if feat.suppressed {
                                    resp.on_hover_text("抑制中（形状から一時的に外れています）")
                                } else if is_sketch {
                                    resp.on_hover_text("ダブルクリック: スケッチを編集 / 右クリック: メニュー")
                                } else {
                                    resp.on_hover_text("右クリック: 名前変更・抑制・削除")
                                };
                                if resp.clicked() {
                                    self.render_config.selected_feature = Some(feat.id);
                                    self.render_config.selected_face = None;
                                    self.render_config.selected_faces.clear();
                                    self.render_config.selected_edge = None;
                                    self.render_config.selected_edges.clear();
                                    self.viewport_dirty = true;
                                }
                                if resp.double_clicked() && is_sketch {
                                    chip_action = Some(ChipAction::EditSketch(feat.id));
                                }
                                resp.context_menu(|ui| {
                                    if is_sketch && ui.button("✏ スケッチを編集").clicked() {
                                        chip_action = Some(ChipAction::EditSketch(feat.id));
                                        ui.close_menu();
                                    }
                                    if ui.button("✏ 名前を変更…").clicked() {
                                        chip_action = Some(ChipAction::Rename(feat.id));
                                        ui.close_menu();
                                    }
                                    let label = if feat.suppressed { "▶ 抑制を解除" } else { "⏸ 抑制（一時的に外す）" };
                                    if ui.button(label).clicked() {
                                        chip_action = Some(ChipAction::ToggleSuppress(feat.id));
                                        ui.close_menu();
                                    }
                                    if !feat.kind.reference_slots().is_empty()
                                        && ui.button("🎯 参照を選び直す…").clicked()
                                    {
                                        chip_action = Some(ChipAction::Rebind(feat.id));
                                        ui.close_menu();
                                    }
                                    if ui
                                        .button("⏮ ここまで戻す")
                                        .on_hover_text("これより後の操作を一時的に外し、ここに新しい操作を差し込めるようにします")
                                        .clicked()
                                    {
                                        chip_action = Some(ChipAction::RollbackTo(feat.id));
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
                    if rolled {
                        ui.set_opacity(1.0);
                    }
                    if rollback == Some(feat.id) {
                        ui.label(RichText::new("▌").size(20.0).color(Color32::from_rgb(240, 190, 80)))
                            .on_hover_text("巻き戻し位置: 新しい操作はここに入ります");
                        past_marker = true;
                    }
                }
            });
        });

        if roll_forward {
            self.set_rollback(None);
        }
        match chip_action {
            Some(ChipAction::Rebind(id)) => self.start_rebind(id, None),
            Some(ChipAction::RollbackTo(id)) => self.set_rollback(Some(id)),
            Some(ChipAction::EditSketch(id)) => self.edit_sketch_feature(id),
            Some(ChipAction::Rename(id)) => {
                self.rename_target = Some((id, self.feature_name(id)));
            }
            Some(ChipAction::ToggleSuppress(id)) => self.toggle_suppressed(id),
            Some(ChipAction::Delete(id)) => self.request_delete(id),
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
                egui::Frame::group(ui.style())
                    .fill(Color32::from_rgba_unmultiplied(80, 24, 24, 160))
                    .show(ui, |ui| {
                        ui.colored_label(Color32::from_rgb(245, 110, 110), "× 計算できませんでした");
                        match &feat.failure {
                            Some(err) => {
                                let e = explain::explain(err, &self.controller.document);
                                ui.label(&e.summary);
                                if !e.action.is_empty() {
                                    ui.label(RichText::new(format!("→ {}", e.action)).strong());
                                }
                                ui.collapsing("詳細（技術情報）", |ui| {
                                    ui.small(msg);
                                });
                            }
                            None => {
                                ui.label(msg);
                            }
                        }
                    });
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
            _ if self.controller.document.is_rolled_back(feat.id) => {
                ui.colored_label(
                    Color32::from_rgb(240, 190, 80),
                    "状態: 巻き戻し中（計算から一時的に外れています）",
                );
            }
            other => {
                ui.label(format!("状態: {other:?}"));
            }
        }

        let names = |ids: &[FeatureId]| {
            ids.iter()
                .map(|d| format!("「{}」", self.feature_name(*d)))
                .collect::<Vec<_>>()
                .join("、")
        };
        let deps = feat.all_dependencies();
        if !deps.is_empty() {
            ui.label(format!("元にしている: {}", names(&deps)));
        }
        let affected = self.controller.document.downstream_of(feat.id);
        if !affected.is_empty() {
            ui.label(format!("変更すると影響: {}（{}個）", names(&affected), affected.len()))
                .on_hover_text("これらはタイムラインで橙色の枠になっています");
        }

        // Face/edge references this feature holds, with whether each still resolves.
        let slots = feat.kind.reference_slots();
        if !slots.is_empty() {
            ui.separator();
            ui.strong("参照（形状への結び付き）:");
            let mut rebind_role = None;
            for slot in &slots {
                let status = self.controller.document.check_reference(slot, &self.store);
                ui.horizontal(|ui| {
                    let label = rebind::role_label(slot.role);
                    match status {
                        Ok(()) => ui.colored_label(Color32::from_rgb(80, 210, 120), format!("✓ {label}")),
                        Err(_) => ui.colored_label(
                            Color32::from_rgb(245, 110, 110),
                            format!("× {label}（見つかりません）"),
                        ),
                    }
                    .on_hover_text(format!("「{}」の上の参照", self.feature_name(slot.upstream)));
                    if ui.small_button("選び直す").clicked() {
                        rebind_role = Some(slot.role);
                    }
                });
            }
            if let Some(role) = rebind_role {
                self.start_rebind(feat.id, Some(role));
            }
        }

        ui.separator();
        ui.strong("形状寸法・パラメータ編集:");
        let bindings: Vec<_> = feature_parameter_slots(&feat.kind)
            .into_iter()
            .filter_map(|(label, scalar)| scalar_expression(scalar).map(|expr| format!("{label} = {expr}")))
            .collect();
        if bindings.is_empty() {
            ui.small("数値欄の fx を押すと、パラメータ名や数式で寸法を指定できます");
        } else {
            ui.small(format!("このフィーチャーのパラメータ指定: {}", bindings.join(" / ")));
        }

        let mut updated_kind = feat.kind.clone();
        let mut kind_changed = false;
        let mut captured_parameter: Option<(String, f64)> = None;

        match &mut updated_kind {
            FeatureKind::Box {
                width,
                height,
                depth,
            } => {
                kind_changed |= edit_scalar_param_capture(ui, "幅 (width)", width, format!("f{}_width", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "高さ (height)", height, format!("f{}_height", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "厚み (depth)", depth, format!("f{}_depth", feat.id.0), &mut captured_parameter);
            }
            FeatureKind::Cylinder { radius, height } => {
                kind_changed |= edit_scalar_param_capture(ui, "半径 (radius)", radius, format!("f{}_radius", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "高さ (height)", height, format!("f{}_height", feat.id.0), &mut captured_parameter);
            }
            FeatureKind::Sphere { radius } => {
                kind_changed |= edit_scalar_param_capture(ui, "半径 (radius)", radius, format!("f{}_radius", feat.id.0), &mut captured_parameter);
            }
            FeatureKind::Cone {
                bottom_radius,
                top_radius,
                height,
            } => {
                kind_changed |= edit_scalar_param_capture(ui, "底面半径", bottom_radius, format!("f{}_bottom_radius", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "上面半径", top_radius, format!("f{}_top_radius", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "高さ", height, format!("f{}_height", feat.id.0), &mut captured_parameter);
            }
            FeatureKind::Transform {
                dx,
                dy,
                dz,
                rot_angle_deg,
                ..
            } => {
                kind_changed |= edit_scalar_param_capture(ui, "移動 X (dx)", dx, format!("f{}_move_x", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "移動 Y (dy)", dy, format!("f{}_move_y", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "移動 Z (dz)", dz, format!("f{}_move_z", feat.id.0), &mut captured_parameter);
                kind_changed |= edit_scalar_param_capture(ui, "回転角 (deg)", rot_angle_deg, format!("f{}_rotation_deg", feat.id.0), &mut captured_parameter);
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
            k @ (FeatureKind::SketchRevolve { .. }
            | FeatureKind::Pattern { .. }
            | FeatureKind::Mirror { .. }
            | FeatureKind::Shell { .. }) => {
                kind_changed |= edit_operation_params(ui, k);
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
                if ui
                    .button("対象2（相手ボディ）を移動")
                    .on_hover_text("ブーリアン結果全体ではなく、相手ボディだけの移動を履歴へ差し込みます")
                    .clicked()
                {
                    self.open_boolean_tool_transform(feat.id, *secondary);
                }
            }
            _ => {}
        }

        if let Some((name, value)) = captured_parameter {
            self.execute_cmd(CadCommand::SetParameter {
                description: format!("{} の値を取り込んだパラメータ", feat.name),
                name,
                value: ParameterValue::Number(value),
                unit: "mm".to_string(),
            });
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

        let edge_count = self.render_config.selected_edges.len();
        if edge_count > 0 {
            ui.separator();
            ui.colored_label(
                Color32::from_rgb(255, 110, 50),
                format!(
                    "📏 選択中の辺: {edge_count}本（合計 {:.2} mm）",
                    self.selected_edges_length()
                ),
            );
            ui.small("Shift+クリックで辺を追加・解除、Escで選択解除");
            match self.edges_on_one_body() {
                Ok(_) => {
                    ui.horizontal(|ui| {
                        if ui.button(format!("⌒ {edge_count}本にフィレット")).clicked() {
                            self.add_edge_modifier_on_selection(false);
                        }
                        if ui.button(format!("◣ {edge_count}本を面取り")).clicked() {
                            self.add_edge_modifier_on_selection(true);
                        }
                    });
                }
                Err(reason) => {
                    ui.colored_label(Color32::from_rgb(240, 150, 65), reason);
                }
            }
        }
        let face_count = self.render_config.selected_faces.len();
        if face_count > 1 {
            ui.small(format!("選択中の面: {face_count}面"));
        }

        ui.separator();
        if ui
            .button("🗑 このフィーチャーを削除 (Delete)")
            .on_hover_text("使っている後の操作がある場合は、まとめて削除するか確認します")
            .clicked()
        {
            self.request_delete(feat.id);
        }
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
            let view_doc = self.view_document();
            if let Some(ref renderer) = self.renderer
                && let Ok(rgba) = renderer.render_to_rgba(
                    &view_doc,
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

        // Sketching happens on its plane over the live 3D view.
        if self.sketch_mode {
            self.render_sketch_overlay(ctx, ui, rect, &response);
        }

        // Mouse camera manipulation (Orbit, Pan, Zoom); dragging a sketch point wins.
        if response.dragged() && self.sketch_drag.is_none() {
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
                pick_document(&self.view_document(), &self.camera, u, v, aspect)
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

        // Measuring: clicks pick faces/points; results are drawn over the view.
        if self.measure.is_some() {
            if response.clicked()
                && let Some(pos) = response.interact_pointer_pos()
            {
                let (u, v) = uv(pos);
                if let Some(hit) = pick_document(&self.view_document(), &self.camera, u, v, aspect) {
                    let p = hit.hit_point;
                    self.measure_pick(hit.feature_id, hit.face_index, glam::DVec3::new(p.x as f64, p.y as f64, p.z as f64));
                }
            }
            self.draw_measure_overlay(ui, rect);
        }

        // Re-binding a reference: a click on the shown body picks the new face/edge.
        if self.rebind.is_some() {
            if response.clicked()
                && let Some(pos) = response.interact_pointer_pos()
            {
                let (u, v) = uv(pos);
                let hit = pick_document(&self.view_document(), &self.camera, u, v, aspect);
                if let Some(hit) = hit {
                    self.apply_rebind_pick(hit.feature_id, hit.face_index, hit.nearest_edge_index);
                }
            }
            self.render_rebind_banner(ui, rect);
        }

        // 3D Raycast Picking: single-click selects, double-click also opens
        // a floating on-canvas property editor right at the clicked point
        // (no need to go hunting through a side panel).
        if !self.sketch_plane_picking
            && !self.sketch_mode
            && self.rebind.is_none()
            && self.measure.is_none()
            && (response.clicked() || response.double_clicked())
            && let Some(pos) = response.interact_pointer_pos()
        {
            let (uv_x, uv_y) = uv(pos);
            let additive = ctx.input(|i| i.modifiers.shift || i.modifiers.ctrl);
            if let Some(hit) =
                pick_document(&self.view_document(), &self.camera, uv_x, uv_y, aspect)
            {
                // The second click of a double-click opens the editor; it must not
                // toggle the selection back off.
                if response.double_clicked() {
                    self.floating_editor_pos = Some(pos);
                } else {
                    self.select_pick(hit.feature_id, hit.face_index, hit.nearest_edge_index, additive);
                }
            } else if !additive {
                self.clear_selection();
                self.floating_editor_pos = None;
            }
            self.viewport_dirty = true;
        }

        // Right-click on a body: select it (unless it is already part of the selection)
        // and offer the actions that apply to it.
        if !self.sketch_plane_picking
            && !self.sketch_mode
            && self.rebind.is_none()
            && self.measure.is_none()
        {
            if response.secondary_clicked()
                && let Some(pos) = response.interact_pointer_pos()
            {
                let (u, v) = uv(pos);
                match pick_document(&self.view_document(), &self.camera, u, v, aspect) {
                    Some(hit) if self.render_config.selected_feature != Some(hit.feature_id) => {
                        self.select_pick(hit.feature_id, hit.face_index, hit.nearest_edge_index, false);
                    }
                    Some(_) => {}
                    None => self.clear_selection(),
                }
                self.viewport_dirty = true;
            }
            response.context_menu(|ui| self.render_selection_menu(ui));
        }

        self.render_floating_editor(ctx);

        // The sketch HUD takes this corner while sketching.
        if self.sketch_mode {
            return;
        }

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
            Pos2::new(rect.center().x, rect.min.y + 110.0),
            Vec2::new(rect.width().min(600.0) - 24.0, 120.0),
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
                        k @ (FeatureKind::SketchRevolve { .. }
                        | FeatureKind::Pattern { .. }
                        | FeatureKind::Mirror { .. }
                        | FeatureKind::Shell { .. }) => {
                            kind_changed |= edit_operation_params(ui, k);
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
    edit_scalar_param_unit(ui, label, param, " mm")
}

/// Property-panel variant that can turn the current literal into a named document
/// parameter in one click. `captured` is applied by the caller together with the
/// feature update, keeping undo/recompute history coherent.
fn edit_scalar_param_capture(
    ui: &mut egui::Ui,
    label: &str,
    param: &mut ScalarParam,
    parameter_name: String,
    captured: &mut Option<(String, f64)>,
) -> bool {
    let mut changed = false;
    let mut convert_to_expression = None;
    let mut capture_value = None;
    let mut revert_to_literal = None;
    ui.horizontal(|ui| {
        ui.label(label);
        match param {
            ScalarParam::Literal(value) => {
                changed |= ui
                    .add(egui::DragValue::new(value).speed(0.5).suffix(" mm"))
                    .changed();
                if ui.small_button("fx").on_hover_text("数式入力へ切り替え").clicked() {
                    convert_to_expression = Some(value.to_string());
                }
                if ui
                    .small_button("＋変数")
                    .on_hover_text("現在値でパラメータを作成し、この寸法にすぐ結び付けます")
                    .clicked()
                {
                    capture_value = Some(*value);
                }
            }
            ScalarParam::Expr(expr) => {
                let collapse_id = ui.make_persistent_id(("collapse_formula", label));
                let collapsed = ui.data(|data| data.get_temp::<bool>(collapse_id).unwrap_or(false));
                if collapsed {
                    if ui
                        .small_button(format!("ƒx {expr}"))
                        .on_hover_text("クリックして数式を編集します")
                        .clicked()
                    {
                        ui.data_mut(|data| data.insert_temp(collapse_id, false));
                    }
                } else {
                    let resp = ui.text_edit_singleline(expr).on_hover_text("例: plate_width * 0.15");
                    // The editable feature clone is rebuilt every frame. Commit every edit so
                    // partially typed expressions do not revert before they can be completed.
                    changed |= resp.changed();
                    let escape = resp.has_focus() && ui.input(|input| input.key_pressed(Key::Escape));
                    if escape {
                        // `fx` starts with the previous literal as its expression. Esc is
                        // cancellation in that case, so restore the compact number widget.
                        if let Ok(value) = expr.trim().parse::<f64>() {
                            revert_to_literal = Some(value);
                        } else {
                            ui.data_mut(|data| data.insert_temp(collapse_id, true));
                        }
                        ui.memory_mut(|memory| memory.surrender_focus(resp.id));
                    } else if ui.small_button("▴").on_hover_text("数式欄を畳む").clicked() {
                        ui.data_mut(|data| data.insert_temp(collapse_id, true));
                        ui.memory_mut(|memory| memory.surrender_focus(resp.id));
                    }
                }
                ui.label("fx").on_hover_text("数式またはパラメータ名で駆動中");
            }
        }
    });
    if let Some(value) = revert_to_literal {
        *param = ScalarParam::Literal(value);
        changed = true;
    } else if let Some(value) = capture_value {
        *param = ScalarParam::Expr(parameter_name.clone());
        *captured = Some((parameter_name, value));
        changed = true;
    } else if let Some(expr) = convert_to_expression {
        *param = ScalarParam::Expr(expr);
        changed = true;
    }
    changed
}

fn edit_scalar_param_unit(ui: &mut egui::Ui, label: &str, param: &mut ScalarParam, unit: &str) -> bool {
    let mut changed = false;
    let mut convert_to_expression = None;
    let mut revert_to_literal = None;
    ui.horizontal(|ui| {
        ui.label(label);
        match param {
            ScalarParam::Literal(val) => {
                if ui
                    .add(egui::DragValue::new(val).speed(0.5).suffix(unit))
                    .changed()
                {
                    changed = true;
                }
                if ui
                    .small_button("fx")
                    .on_hover_text("数値をパラメータ名または数式で指定する入力欄に切り替えます")
                    .clicked()
                {
                    convert_to_expression = Some(val.to_string());
                }
            }
            ScalarParam::Expr(expr) => {
                let collapse_id = ui.make_persistent_id(("collapse_formula", label));
                let collapsed = ui.data(|data| data.get_temp::<bool>(collapse_id).unwrap_or(false));
                if collapsed {
                    if ui
                        .small_button(format!("ƒx {expr}"))
                        .on_hover_text("クリックして数式を編集します")
                        .clicked()
                    {
                        ui.data_mut(|data| data.insert_temp(collapse_id, false));
                    }
                } else {
                    let resp = ui
                        .text_edit_singleline(expr)
                        .on_hover_text("例: plate_width、plate_width * 0.15、offset_x + 10");
                    changed |= resp.changed();
                    let escape = resp.has_focus() && ui.input(|input| input.key_pressed(Key::Escape));
                    if escape {
                        if let Ok(value) = expr.trim().parse::<f64>() {
                            revert_to_literal = Some(value);
                        } else {
                            ui.data_mut(|data| data.insert_temp(collapse_id, true));
                        }
                        ui.memory_mut(|memory| memory.surrender_focus(resp.id));
                    } else if ui.small_button("▴").on_hover_text("数式欄を畳む").clicked() {
                        ui.data_mut(|data| data.insert_temp(collapse_id, true));
                        ui.memory_mut(|memory| memory.surrender_focus(resp.id));
                    }
                }
                ui.label("fx").on_hover_text("数式またはパラメータ名で駆動中");
            }
        }
    });
    if let Some(value) = revert_to_literal {
        *param = ScalarParam::Literal(value);
        changed = true;
    } else if let Some(expr) = convert_to_expression {
        *param = ScalarParam::Expr(expr);
        changed = true;
    }
    changed
}

/// Editors for revolve / pattern / mirror / shell parameters (shared by the property
/// panel and the floating editor).
fn edit_operation_params(ui: &mut egui::Ui, kind: &mut FeatureKind) -> bool {
    let mut changed = false;
    match kind {
        FeatureKind::SketchRevolve { angle_deg, .. } => {
            changed |= edit_scalar_param_unit(ui, "回転角度", angle_deg, "°");
        }
        FeatureKind::Pattern { layout, .. } => match layout {
            cad_core::PatternLayout::Circular {
                count,
                total_angle_deg,
                ..
            } => {
                ui.horizontal(|ui| {
                    ui.label("個数");
                    changed |= ui.add(egui::DragValue::new(count).range(2..=360)).changed();
                });
                changed |= edit_scalar_param_unit(ui, "全体の角度", total_angle_deg, "°");
            }
            cad_core::PatternLayout::Linear { count, spacing, .. } => {
                ui.horizontal(|ui| {
                    ui.label("個数");
                    changed |= ui.add(egui::DragValue::new(count).range(2..=200)).changed();
                });
                changed |= edit_scalar_param(ui, "間隔", spacing);
            }
        },
        FeatureKind::Mirror { merge, .. } => {
            changed |= ui.checkbox(merge, "元の形状と結合する").changed();
        }
        FeatureKind::Shell { thickness, .. } => {
            changed |= edit_scalar_param(ui, "壁の厚さ", thickness);
        }
        _ => {}
    }
    changed
}

impl eframe::App for FerroCadApp {
    /// A normal exit leaves no autosave behind (so the next start offers no recovery);
    /// if the process dies instead, the autosave survives and is offered next time.
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        if !self.has_unsaved_changes() || self.allow_close {
            self.clear_autosave();
        }
    }

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

        // While sketching, the side panels give their space to the canvas.
        egui::SidePanel::left("left_tree_panel")
            .resizable(true)
            .default_width(310.0)
            .show_animated(ctx, !self.sketch_mode, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.render_left_panel(ui);
                });
            });

        egui::SidePanel::right("right_property_panel")
            .resizable(true)
            .default_width(300.0)
            .show_animated(ctx, !self.sketch_mode, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.render_right_property_panel(ui);
                });
            });

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ctx, |ui| {
                self.render_central_viewport(ctx, ui);
            });

        if !self.sketch_mode {
            self.render_tool_panel(ctx);
            self.render_measure_panel(ctx);
        }
        self.render_file_dialogs(ctx);
        self.render_delete_confirm(ctx);

        // Autosave, window title, and asking before closing with unsaved work.
        self.tick_autosave(std::time::Instant::now());
        let title = self.window_title();
        if title != self.last_title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.last_title = title;
        }
        if ctx.input(|i| i.viewport().close_requested()) && !self.allow_close && self.has_unsaved_changes() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.pending_action = Some(files::PendingAction::Exit);
        }
        if self.allow_close {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

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
                        self.show_command_palette = false;
                        self.request(files::PendingAction::Demo);
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

    #[test]
    fn transform_tool_moves_a_selected_body_and_replaces_its_display() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let input = app.render_config.selected_feature.unwrap();
        let before = app.controller.document.features[&input]
            .cached_summary
            .unwrap()
            .bounds;

        app.open_tool_panel(ToolKind::Transform);
        let mut panel = app.tool_panel.take().unwrap();
        match &mut panel {
            ToolPanel::Transform { translation, .. } => *translation = [25.0, -10.0, 5.0],
            other => panic!("unexpected panel {other:?}"),
        }
        app.confirm_tool(panel);

        let moved = app.render_config.selected_feature.unwrap();
        let after = app.controller.document.features[&moved]
            .cached_summary
            .unwrap()
            .bounds;
        assert_eq!(after.min - before.min, glam::DVec3::new(25.0, -10.0, 5.0));
        assert!(!app.controller.document.features[&input].visible);
        assert!(app.controller.document.features[&moved].visible);
    }

    #[test]
    fn parameter_usage_labels_identify_the_feature_dimension() {
        let kind = FeatureKind::Transform {
            input: FeatureId(1),
            dx: ScalarParam::expr("offset_x"),
            dy: ScalarParam::mm(0.0),
            dz: ScalarParam::mm(0.0),
            rot_origin: [0.0; 3],
            rot_axis: [0.0, 0.0, 1.0],
            rot_angle_deg: ScalarParam::expr("rotation"),
        };
        let slots = feature_parameter_slots(&kind);
        assert_eq!(slots.len(), 4);
        assert_eq!(slots[0].0, "移動 X");
        assert_eq!(scalar_expression(slots[0].1), Some("offset_x"));
        assert_eq!(slots[3].0, "回転角");
        assert_eq!(scalar_expression(slots[3].1), Some("rotation"));
    }

    #[test]
    fn boolean_tool_move_inserts_transform_before_boolean() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let boolean = FeatureId(5);
        let tool = match &app.controller.document.features[&boolean].kind {
            FeatureKind::Boolean { secondary, .. } => *secondary,
            other => panic!("expected Boolean, got {other:?}"),
        };

        app.open_boolean_tool_transform(boolean, tool);
        let mut panel = app.tool_panel.take().unwrap();
        match &mut panel {
            ToolPanel::Transform { translation, .. } => *translation = [12.0, 0.0, 0.0],
            other => panic!("unexpected panel {other:?}"),
        }
        app.confirm_tool(panel);

        let moved_tool = match &app.controller.document.features[&boolean].kind {
            FeatureKind::Boolean { secondary, .. } => *secondary,
            other => panic!("expected Boolean, got {other:?}"),
        };
        assert_ne!(moved_tool, tool);
        assert!(matches!(
            app.controller.document.features[&moved_tool].kind,
            FeatureKind::Transform { input, .. } if input == tool
        ));
        let timeline = app.controller.document.timeline();
        let pos = |id| timeline.iter().position(|&item| item == id).unwrap();
        assert!(pos(tool) < pos(moved_tool) && pos(moved_tool) < pos(boolean));
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

        app.start_sketch_with(SketchStart::Extrude);
        assert!(app.sketch_plane_picking);
        app.begin_sketch_on_face(plate, top.index);
        assert!(app.sketch_mode && !app.sketch_plane_picking);
        assert_eq!(app.sketch_target, Some(plate));
        // On a top face, sketch coordinates coincide with world XY.
        assert!(app.sketch_frame.origin.z > 9.99 && app.sketch_frame.normal.z > 0.99);

        app.sketch.add_rectangle(40.0, 25.0, 50.0, 35.0);
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

    #[test]
    fn m7_offset_plane_starts_a_following_face_sketch() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let plate = FeatureId(5);
        let shape = app.controller.document.features[&plate].cached_shape.unwrap();
        let top = app
            .store
            .resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99)
            .unwrap();
        app.select_pick(plate, top.index, None, false);
        app.open_tool_panel(ToolKind::OffsetPlane);
        let mut panel = app.tool_panel.take().expect("offset panel opens for a selected face");
        if let ToolPanel::OffsetPlane { offset, .. } = &mut panel {
            *offset = 12.5;
        } else {
            panic!("expected offset-plane panel");
        }
        app.confirm_tool(panel);

        assert!(app.sketch_mode);
        assert_eq!(app.sketch_target, Some(plate));
        assert!(matches!(app.sketch_plane, SketchPlane::OffsetFace { .. }));
        assert!(app.sketch_frame.normal.z > 0.99);
        assert!((app.sketch_frame.origin.z - (top.centroid.z + 12.5)).abs() < 1e-6);
    }

    /// Renders the real UI offscreen (egui_kittest + wgpu) to PNGs under
    /// `build/ui-snapshots/` so layout can be inspected without a window. Needs a GPU
    /// adapter, so it is opt-in: `cargo test -p cad-ui -- --ignored ui_snapshots`.
    #[test]
    #[ignore]
    fn ui_snapshots() {
        use cad_core::SketchConstraint;
        use egui_kittest::Harness;

        let out = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../build/ui-snapshots");
        std::fs::create_dir_all(&out).unwrap();
        let shoot = |name: &str, setup: &dyn Fn(&mut FerroCadApp)| {
            let mut app = FerroCadApp::new_with_mvp_demo();
            setup(&mut app);
            let mut harness = Harness::builder()
                .with_size(egui::vec2(1360.0, 840.0))
                .build_eframe(|cc| {
                    install_japanese_font(&cc.egui_ctx);
                    app
                });
            harness.run_steps(6);
            harness.render().unwrap().save(out.join(name)).unwrap();
        };

        shoot("main.png", &|_| {});
        shoot("plane_picker.png", &|app| app.start_sketch_with(SketchStart::Extrude));
        shoot("sketch_editor.png", &|app| {
            let plate = FeatureId(5);
            let shape = app.controller.document.features[&plate].cached_shape.unwrap();
            let top = app
                .store
                .resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99)
                .unwrap();
            app.start_sketch_with(SketchStart::Extrude);
            app.begin_sketch_on_face(plate, top.index);
            let (_, [p0, p1, _, p3]) = app.sketch.add_rectangle(32.0, 18.0, 68.0, 42.0);
            let c = app.sketch.add_point(50.0, 30.0);
            let circle = app.sketch.add_circle(c, 6.0);
            app.try_add_constraint(SketchConstraint::Distance { p1: p0, p2: p1, distance: 36.0 }, false);
            app.try_add_constraint(SketchConstraint::Distance { p1: p0, p2: p3, distance: 24.0 }, false);
            app.try_add_constraint(SketchConstraint::Radius { entity: circle, radius: 6.0 }, false);
            app.sketch_tool = SketchTool::Select;
            app.sketch_selection = vec![SketchSel::Segment(p0, p1)];
            app.set_sketch_op(SketchOp::Cut);
        });

        // M4: a flange with a ring of bolt holes, built through the app's own tools.
        let build_flange = |app: &mut FerroCadApp| -> FeatureId {
            for id in app.controller.document.features.keys().copied().collect::<Vec<_>>() {
                app.execute_cmd(CadCommand::UpdateFeature {
                    id,
                    name: None,
                    kind: None,
                    visible: Some(false),
                    color_rgba: None,
                });
            }
            app.start_sketch_with(SketchStart::Revolve);
            app.begin_sketch(SketchPlane::XZ, None);
            let pts: Vec<_> = [[10.0, 0.0], [50.0, 0.0], [50.0, 8.0], [20.0, 8.0], [20.0, 25.0], [10.0, 25.0]]
                .iter()
                .map(|p| app.sketch.add_point(p[0], p[1]))
                .collect();
            app.sketch.add_polyline(pts, true);
            app.finish_sketch_extrude();
            let flange = *app.controller.document.features.keys().next_back().unwrap();
            app.sketch_target = Some(flange);
            app.start_sketch_with(SketchStart::Hole);
            app.begin_sketch(SketchPlane::XY, Some(flange));
            app.set_sketch_op(SketchOp::Cut);
            assert!(!app.sketch_reversed, "the flange is above XY, so the cut must head +Z");
            app.sketch_through_all = true;
            let c = app.sketch.add_point(35.0, 0.0);
            app.sketch.add_circle(c, 4.0);
            app.finish_sketch_extrude();
            *app.controller.document.features.keys().next_back().unwrap()
        };
        shoot("m4_flange.png", &|app| {
            let hole = build_flange(app);
            app.render_config.selected_feature = Some(hole);
            app.open_tool_panel(ToolKind::CircularPattern);
            let panel = app.tool_panel.take().unwrap();
            app.confirm_tool(panel);
            app.render_config.selected_feature = None;
            app.fit_all();
        });
        shoot("m4_pattern_panel.png", &|app| {
            let hole = build_flange(app);
            app.render_config.selected_feature = Some(hole);
            app.fit_all();
            app.open_tool_panel(ToolKind::CircularPattern);
        });
        // M5: a hole whose plane reference broke (explanation + red chip), the re-bind
        // mode, and a rolled-back timeline.
        let broken_hole = |app: &mut FerroCadApp| -> FeatureId {
            let plate = FeatureId(5);
            let shape = app.controller.document.features[&plate].cached_shape.unwrap();
            let top = app.store.resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99).unwrap();
            app.start_sketch_with(SketchStart::Hole);
            app.begin_sketch_on_face(plate, top.index);
            let c = app.sketch.add_point(50.0, 30.0);
            app.sketch.add_circle(c, 5.0);
            app.finish_sketch_extrude();
            let hole = *app.controller.document.features.keys().next_back().unwrap();
            let mut kind = app.controller.document.features[&hole].kind.clone();
            let mut bogus = kind.reference_slots()[0].reference.clone();
            bogus.expected_normal = Some(glam::DVec3::new(1.0, 1.0, 1.0).normalize());
            kind.replace_reference(cad_core::RefRole::Plane, bogus);
            app.execute_cmd(CadCommand::UpdateFeature { id: hole, name: None, kind: Some(kind), visible: None, color_rgba: None });
            app.render_config.selected_feature = Some(hole);
            hole
        };
        shoot("m5_failure.png", &|app| {
            broken_hole(app);
        });
        shoot("m5_rebind.png", &|app| {
            let hole = broken_hole(app);
            app.start_rebind(hole, None);
        });
        shoot("m5_rollback.png", &|app| {
            app.set_rollback(Some(FeatureId(3)));
            app.render_config.selected_feature = Some(FeatureId(1));
        });
        shoot("m6_measure.png", &|app| {
            app.ribbon_tab = RibbonTab::Tools;
            let shape = app.controller.document.features[&FeatureId(5)].cached_shape.unwrap();
            let top = app.store.resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99).unwrap();
            let side = app.store.resolve_face_by_signature(shape, glam::DVec3::X, Some(glam::DVec3::new(100.0, 30.0, 5.0)), 0.99).unwrap();
            app.start_measure();
            app.measure_pick(FeatureId(5), top.index, glam::DVec3::new(30.0, 30.0, 10.0));
            app.measure_pick(FeatureId(5), side.index, glam::DVec3::new(100.0, 30.0, 5.0));
        });
        shoot("m6_recovery.png", &|app| {
            app.recovery_offer = Some(Some(std::path::PathBuf::from(r"C:/work/bracket.fcad")));
        });
        shoot("m6_unsaved.png", &|app| {
            app.pending_action = Some(files::PendingAction::New);
        });
        shoot("multi_edges.png", &|app| {
            let plate = FeatureId(5);
            let shape = app.controller.document.features[&plate].cached_shape.unwrap();
            let info = app.store.inspect_shape(shape).unwrap();
            let top = app.store.resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99).unwrap();
            let top_edges: Vec<u32> = info
                .edges
                .iter()
                .filter(|e| (e.centroid.z - 10.0).abs() < 1e-6 && e.length_mm > 50.0)
                .map(|e| e.index)
                .collect();
            for (i, e) in top_edges.iter().enumerate() {
                app.select_pick(plate, top.index, Some(*e), i > 0);
            }
        });
        shoot("delete_confirm.png", &|app| {
            app.request_delete(FeatureId(1));
        });
        shoot("m4_revolve_sketch.png", &|app| {
            app.start_sketch_with(SketchStart::Revolve);
            app.begin_sketch(SketchPlane::XZ, None);
            let pts: Vec<_> = [[10.0, 0.0], [50.0, 0.0], [50.0, 8.0], [20.0, 8.0], [20.0, 25.0], [10.0, 25.0]]
                .iter()
                .map(|p| app.sketch.add_point(p[0], p[1]))
                .collect();
            app.sketch.add_polyline(pts, true);
            app.resolve_sketch();
            app.sketch_tool = SketchTool::Select;
        });
    }

    /// M4 flow through the app: revolve a flange on the XZ plane, drill one hole with
    /// the 穴 tool, ring it with a circular pattern panel, then mirror a block and shell
    /// a box via their panels.
    #[test]
    fn m4_flange_via_tools() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let vol = |app: &FerroCadApp, id: FeatureId| {
            app.controller.document.features[&id].cached_summary.expect("computed").volume_mm3
        };
        let last = |app: &FerroCadApp| *app.controller.document.features.keys().next_back().unwrap();

        // Revolve: L-profile on XZ about the sketch's vertical axis (world Z).
        app.start_sketch_with(SketchStart::Revolve);
        app.begin_sketch(SketchPlane::XZ, None);
        assert!(app.sketch_revolve);
        let pts: Vec<_> = [[10.0, 0.0], [50.0, 0.0], [50.0, 8.0], [20.0, 8.0], [20.0, 25.0], [10.0, 25.0]]
            .iter()
            .map(|p| app.sketch.add_point(p[0], p[1]))
            .collect();
        app.sketch.add_polyline(pts, true);
        app.finish_sketch_extrude();
        let flange = last(&app);
        let flange_vol = 24_300.0 * std::f64::consts::PI;
        assert!((vol(&app, flange) - flange_vol).abs() < 1.0);

        // 穴 tool: the face picked is the flange's bottom; through-all cut by default.
        let shape = app.controller.document.features[&flange].cached_shape.unwrap();
        let bottom = app
            .store
            .resolve_face_by_signature(shape, glam::DVec3::NEG_Z, None, 0.99)
            .unwrap();
        app.start_sketch_with(SketchStart::Hole);
        app.begin_sketch_on_face(flange, bottom.index);
        assert_eq!(app.sketch_tool, SketchTool::Circle);
        assert_eq!(app.sketch_op, SketchOp::Cut);
        assert!(app.sketch_through_all);
        // Bottom face is viewed from below, so sketch X runs along world X: (35, 0).
        let c = app.sketch.add_point(35.0, 0.0);
        app.sketch.add_circle(c, 4.0);
        app.finish_sketch_extrude();
        let hole = last(&app);
        let hole_area = std::f64::consts::PI * 16.0;
        assert!((vol(&app, hole) - (flange_vol - hole_area * 8.0)).abs() < 1.0, "{}", vol(&app, hole));

        // Circular pattern panel: defaults to the target's centre, axis Z, 6 x 360°.
        app.render_config.selected_feature = Some(hole);
        app.open_tool_panel(ToolKind::CircularPattern);
        let panel = app.tool_panel.take().unwrap();
        match &panel {
            ToolPanel::CircularPattern { axis, center, count, .. } => {
                assert_eq!((*axis, *count), (2, 6));
                assert!(center[0].abs() < 1e-6 && center[1].abs() < 1e-6);
            }
            other => panic!("unexpected panel {other:?}"),
        }
        app.confirm_tool(panel);
        let ring = last(&app);
        assert!((vol(&app, ring) - (flange_vol - 6.0 * hole_area * 8.0)).abs() < 1.0, "{}", vol(&app, ring));
        assert!(!app.controller.document.features[&hole].visible);
        assert!(app.controller.document.features[&ring].visible);
    }

    /// M5 flow: a hole whose sketch-plane reference is lost shows a plain explanation
    /// and is repaired by re-binding (clicking the top face again); then rolling back to
    /// the plate, inserting a second hole and rolling forward keeps the timeline order.
    #[test]
    fn m5_explain_rebind_and_rollback() {
        use cad_core::RefRole;

        let mut app = FerroCadApp::new_with_mvp_demo();
        let plate = FeatureId(5);
        let shape = app.controller.document.features[&plate].cached_shape.unwrap();
        let top = app
            .store
            .resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99)
            .unwrap();
        app.start_sketch_with(SketchStart::Hole);
        app.begin_sketch_on_face(plate, top.index);
        let c = app.sketch.add_point(50.0, 30.0);
        app.sketch.add_circle(c, 5.0);
        app.finish_sketch_extrude();
        let hole = *app.controller.document.features.keys().next_back().unwrap();
        assert_eq!(app.controller.document.features[&hole].state, FeatureState::Clean);

        // Break the plane reference (as an upstream edit could) and recompute.
        let mut kind = app.controller.document.features[&hole].kind.clone();
        let mut bogus = kind.reference_slots()[0].reference.clone();
        bogus.expected_normal = Some(glam::DVec3::new(1.0, 1.0, 1.0).normalize());
        bogus.normal_cos_threshold = 0.99;
        assert!(kind.replace_reference(RefRole::Plane, bogus));
        app.execute_cmd(CadCommand::UpdateFeature { id: hole, name: None, kind: Some(kind), visible: None, color_rgba: None });
        let feat = &app.controller.document.features[&hole];
        let e = explain::explain(feat.failure.as_ref().expect("structured failure"), &app.controller.document);
        assert!(e.summary.contains("見つからなく"), "{}", e.summary);
        assert!(e.action.contains("選び直す"), "{}", e.action);
        assert!(app.status_logs.last().unwrap().contains("計算できません"));

        // Re-bind: the viewport shows only the plate; clicking its top face repairs it.
        app.start_rebind(hole, None);
        let state = app.rebind.expect("rebind active");
        assert_eq!((state.role, state.upstream), (RefRole::Plane, plate));
        let view = app.view_document();
        assert!(view.features.values().all(|f| f.visible == (f.id == plate)));
        drop(view);
        app.apply_rebind_pick(plate, top.index, None);
        assert!(app.rebind.is_none());
        assert_eq!(app.controller.document.features[&hole].state, FeatureState::Clean);
        assert!(app.status_logs.last().unwrap().contains("選び直しました"));

        // Roll back to the plate, insert a second hole there, roll forward.
        app.set_rollback(Some(plate));
        assert!(app.controller.document.is_rolled_back(hole));
        assert!(app.controller.document.features[&hole].cached_mesh.is_none());
        app.start_sketch_with(SketchStart::Hole);
        app.begin_sketch_on_face(plate, top.index);
        let c = app.sketch.add_point(70.0, 30.0);
        app.sketch.add_circle(c, 5.0);
        app.finish_sketch_extrude();
        let inserted = app.controller.document.rollback.expect("marker on inserted feature");
        let tl = app.controller.document.timeline();
        let pos = |id| tl.iter().position(|&f| f == id).unwrap();
        assert!(pos(plate) < pos(inserted) && pos(inserted) < pos(hole));
        app.set_rollback(None);
        let doc = &app.controller.document;
        assert_eq!(doc.features[&hole].state, FeatureState::Clean);
        // The later hole was re-wired onto the inserted one, so it carries both holes.
        let area = std::f64::consts::PI * 25.0;
        let plate_vol = doc.features[&plate].cached_summary.unwrap().volume_mm3;
        let vol = doc.features[&hole].cached_summary.unwrap().volume_mm3;
        assert!((vol - (plate_vol - 2.0 * area * 10.0)).abs() < 1.0, "{vol} vs {plate_vol}");
    }

    /// M6: autosave survives a crash and is offered/restored on the next start; saving
    /// clears it and records a recent file; STL/STEP go through the app; measuring two
    /// parallel faces reports their gap; unsaved work is guarded.
    #[test]
    fn m6_autosave_recovery_files_and_measure() {
        let dir = std::env::temp_dir().join(format!("ferrocad_ui_m6_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);

        let mut app = FerroCadApp::new_with_mvp_demo().with_persistence(dir.clone());
        assert!(app.recovery_offer.is_none());
        assert!(!app.has_unsaved_changes());
        app.add_primitive(
            "Box",
            FeatureKind::Box {
                width: ScalarParam::mm(10.0),
                height: ScalarParam::mm(10.0),
                depth: ScalarParam::mm(10.0),
            },
            [1.0; 4],
        );
        assert!(app.has_unsaved_changes());
        app.tick_autosave(std::time::Instant::now());
        assert!(app.autosave_path().unwrap().exists());
        let expected = app.controller.document.features.len();
        drop(app); // the process "dies": no on_exit, autosave stays

        let mut app = FerroCadApp::new_with_mvp_demo().with_persistence(dir.clone());
        assert_eq!(app.recovery_offer, Some(None), "work from the dead session is offered");
        app.recover();
        assert_eq!(app.controller.document.features.len(), expected);
        assert!(app.has_unsaved_changes(), "recovered work still needs saving");

        let saved = dir.join("part.fcad");
        assert!(app.save_document(Some(saved.clone())));
        assert!(!app.autosave_path().unwrap().exists(), "saving clears the autosave");
        assert!(app.window_title().starts_with("part.fcad"));

        // STL of what is visible, and a STEP round trip as an imported body.
        let stl = dir.join("part.stl");
        assert!(app.export_visible_stl(&stl));
        let bytes = std::fs::read(&stl).unwrap();
        let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        assert!(n > 0 && bytes.len() == 84 + 50 * n);
        let step = dir.join("plate.step");
        let plate_shape = app.controller.document.features[&FeatureId(5)].cached_shape.unwrap();
        app.store.export_step(plate_shape, &step).unwrap();
        assert!(app.import_step_file(&step));
        let imported = *app.controller.document.features.keys().next_back().unwrap();
        assert!(matches!(app.controller.document.features[&imported].kind, FeatureKind::ImportedBody { .. }));
        assert_eq!(app.controller.document.features[&imported].state, FeatureState::Clean);

        // Measure the plate's top and bottom faces: parallel, 10mm apart.
        let shape = app.controller.document.features[&FeatureId(5)].cached_shape.unwrap();
        let top = app.store.resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99).unwrap();
        let bottom = app.store.resolve_face_by_signature(shape, glam::DVec3::NEG_Z, None, 0.99).unwrap();
        app.start_measure();
        app.measure_pick(FeatureId(5), top.index, glam::DVec3::new(50.0, 30.0, 10.0));
        app.measure_pick(FeatureId(5), bottom.index, glam::DVec3::new(50.0, 30.0, 0.0));
        let rows = app.measure_results();
        let find = |label: &str| rows.iter().find(|(l, _)| l == label).map(|(_, v)| v.clone()).unwrap();
        assert_eq!(find("点と点の距離"), "10.000 mm");
        assert_eq!(find("面と面の角度"), "0.00°");
        assert_eq!(find("面と面の距離（平行）"), "10.000 mm");

        // Unsaved changes are guarded before a destructive action.
        assert!(app.has_unsaved_changes());
        app.request(files::PendingAction::New);
        assert_eq!(app.pending_action, Some(files::PendingAction::New));

        // A clean start afterwards: recent file remembered, nothing to recover.
        app.clear_autosave();
        let app2 = FerroCadApp::new_with_mvp_demo().with_persistence(dir.clone());
        assert_eq!(app2.recent_files.first(), Some(&saved));
        assert!(app2.recovery_offer.is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// Multi-selection: Shift+click toggles edges; a fillet takes all selected edges of
    /// one body (mixed bodies are refused); deleting a used body asks and then removes
    /// it with its dependents in one undo step; deleting a fillet shows its body again;
    /// a shell opens every selected face.
    #[test]
    fn multi_select_fillet_and_cascade_delete() {
        let mut app = FerroCadApp::new_with_mvp_demo();
        let plate = FeatureId(5);
        let shape = app.controller.document.features[&plate].cached_shape.unwrap();
        let info = app.store.inspect_shape(shape).unwrap();
        let top = app.store.resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99).unwrap();
        // The two long top edges of the plate (100 mm, at z = 10).
        let long_top: Vec<u32> = info
            .edges
            .iter()
            .filter(|e| (e.length_mm - 100.0).abs() < 1e-6 && (e.centroid.z - 10.0).abs() < 1e-6)
            .map(|e| e.index)
            .collect();
        assert_eq!(long_top.len(), 2);

        app.select_pick(plate, top.index, Some(long_top[0]), false);
        app.select_pick(plate, top.index, Some(long_top[1]), true);
        assert_eq!(app.render_config.selected_edges.len(), 2);
        app.select_pick(plate, top.index, Some(long_top[1]), true);
        assert_eq!(app.render_config.selected_edges.len(), 1, "Shift+click again removes it");
        app.select_pick(plate, top.index, Some(long_top[1]), true);
        assert!(app.render_config.is_edge_selected((plate, long_top[0])));

        let plate_vol = app.controller.document.features[&plate].cached_summary.unwrap().volume_mm3;
        assert!(app.add_edge_modifier_on_selection(false));
        let fillet = *app.controller.document.features.keys().next_back().unwrap();
        let f = &app.controller.document.features[&fillet];
        assert_eq!(f.state, FeatureState::Clean);
        assert_eq!(f.kind.reference_slots().len(), 2, "one reference per edge");
        assert!(f.cached_summary.unwrap().volume_mm3 < plate_vol);
        assert!(!app.controller.document.features[&plate].visible, "the filleted body replaces it");
        assert!(app.render_config.selected_edges.is_empty());

        // Edges from two different bodies cannot be filleted together.
        app.add_primitive(
            "Box",
            FeatureKind::Box {
                width: ScalarParam::mm(40.0),
                height: ScalarParam::mm(30.0),
                depth: ScalarParam::mm(20.0),
            },
            [1.0; 4],
        );
        let boxed = *app.controller.document.features.keys().next_back().unwrap();
        app.select_pick(plate, top.index, Some(long_top[0]), false);
        app.select_pick(boxed, 0, Some(0), true);
        assert!(app.edges_on_one_body().unwrap_err().contains("別々のボディ"));

        // Shell the box with two open faces (top and +X side), Shift-selected.
        let bshape = app.controller.document.features[&boxed].cached_shape.unwrap();
        let btop = app.store.resolve_face_by_signature(bshape, glam::DVec3::Z, None, 0.99).unwrap();
        let bside = app.store.resolve_face_by_signature(bshape, glam::DVec3::X, None, 0.99).unwrap();
        app.select_pick(boxed, btop.index, None, false);
        app.select_pick(boxed, bside.index, None, true);
        app.open_tool_panel(ToolKind::Shell);
        let panel = app.tool_panel.take().unwrap();
        app.confirm_tool(panel);
        let shell = *app.controller.document.features.keys().next_back().unwrap();
        let s = &app.controller.document.features[&shell];
        assert_eq!(s.state, FeatureState::Clean, "{:?}", s.failure);
        assert_eq!(s.kind.reference_slots().len(), 2);

        // Deleting the fillet alone: the plate is shown again.
        app.request_delete(fillet);
        assert!(app.delete_confirm.is_none());
        assert!(!app.controller.document.features.contains_key(&fillet));
        assert!(app.controller.document.features[&plate].visible);

        // Deleting the plate (used by nothing now) vs. a used base: ask, then cascade.
        let base = FeatureId(1);
        let before = app.controller.document.features.len();
        app.request_delete(base);
        let (id, deps) = app.delete_confirm.clone().expect("asks before deleting dependents");
        assert_eq!(id, base);
        assert!(deps.contains(&plate));
        app.delete_confirm = None;
        app.delete_with_dependents(base);
        assert!(!app.controller.document.features.contains_key(&base));
        assert!(!app.controller.document.features.contains_key(&plate));
        app.controller.undo(&mut app.store).unwrap();
        app.recompute_and_refresh();
        assert_eq!(app.controller.document.features.len(), before, "one Ctrl+Z restores them all");
    }

    /// M3 flow: draw a rectangle with a circle inside, constrain it fully with
    /// dimensions, extrude it, add a downstream cut on its top, then re-edit the base
    /// sketch's width — the cut recomputes on the wider plate. A conflicting constraint
    /// is refused with a plain-language message.
    #[test]
    fn m3_constrain_extrude_and_reedit_sketch() {
        use cad_core::{SketchConstraint, SketchStatus};

        let mut app = FerroCadApp::new_with_mvp_demo();
        app.start_sketch_with(SketchStart::Extrude);
        app.begin_sketch(SketchPlane::XY, None);
        let (_, [p0, p1, p2, p3]) = app.sketch.add_rectangle(200.0, 0.0, 280.0, 50.0);
        let c = app.sketch.add_point(240.0, 25.0);
        let circle = app.sketch.add_circle(c, 8.0);
        for constraint in [
            SketchConstraint::Fixed { point: p0, x: 200.0, y: 0.0 },
            SketchConstraint::Distance { p1: p0, p2: p1, distance: 80.0 },
            SketchConstraint::Distance { p1: p0, p2: p3, distance: 50.0 },
            SketchConstraint::Fixed { point: c, x: 240.0, y: 25.0 },
            SketchConstraint::Radius { entity: circle, radius: 10.0 },
        ] {
            assert!(app.try_add_constraint(constraint, false));
        }
        assert_eq!(
            app.sketch_report.as_ref().unwrap().status,
            SketchStatus::FullyConstrained
        );

        // Redundant (already horizontal) and conflicting (a 60mm side that is fixed at
        // 80mm) constraints are both refused, with an explanation.
        assert!(!app.try_add_constraint(SketchConstraint::Horizontal { p1: p0, p2: p1 }, false));
        assert!(app.status_logs.last().unwrap().contains("すでに他の拘束で決まっています"));
        assert!(!app.try_add_constraint(SketchConstraint::Distance { p1: p2, p2: p3, distance: 60.0 }, false));
        assert!(app.status_logs.last().unwrap().contains("矛盾"));

        app.sketch_depth = 10.0;
        app.finish_sketch_extrude();
        let (&base, _) = app.controller.document.features.iter().next_back().unwrap();
        let hole = std::f64::consts::PI * 100.0;
        let vol = |app: &FerroCadApp, id: FeatureId| {
            app.controller.document.features[&id].cached_summary.unwrap().volume_mm3
        };
        assert!((vol(&app, base) - (4000.0 - hole) * 10.0).abs() < 1.0);

        // Downstream: cut a 10x10 pocket 4mm deep from the top face.
        let shape = app.controller.document.features[&base].cached_shape.unwrap();
        let top = app
            .store
            .resolve_face_by_signature(shape, glam::DVec3::Z, None, 0.99)
            .unwrap();
        app.start_sketch_with(SketchStart::Extrude);
        app.begin_sketch_on_face(base, top.index);
        app.sketch.add_rectangle(205.0, 5.0, 215.0, 15.0);
        app.set_sketch_op(SketchOp::Cut);
        app.sketch_depth = 4.0;
        app.finish_sketch_extrude();
        let (&pocket, _) = app.controller.document.features.iter().next_back().unwrap();
        assert!((vol(&app, pocket) - ((4000.0 - hole) * 10.0 - 400.0)).abs() < 1.0);

        // Re-edit the base sketch: widen the plate from 80 to 100.
        app.edit_sketch_feature(base);
        assert!(app.sketch_mode && app.sketch_editing == Some(base));
        let width_dim = app
            .sketch
            .constraints
            .iter()
            .position(|c| matches!(c, SketchConstraint::Distance { distance, .. } if (*distance - 80.0).abs() < 1e-9))
            .unwrap();
        assert!(app.try_set_dimension(width_dim, 100.0));
        let features_before = app.controller.document.features.len();
        app.finish_sketch_extrude();
        assert_eq!(app.controller.document.features.len(), features_before, "re-edit must not add a feature");
        assert!(app.controller.document.features[&pocket].state == FeatureState::Clean);
        assert!((vol(&app, pocket) - ((5000.0 - hole) * 10.0 - 400.0)).abs() < 1.0);
    }
}
