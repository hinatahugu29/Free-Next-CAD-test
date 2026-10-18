//! Saving and opening `.fcad` documents, recent files, autosave with crash recovery,
//! and STEP / STL import and export.

use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use super::*;

const AUTOSAVE_INTERVAL: Duration = Duration::from_secs(20);
const MAX_RECENT: usize = 8;

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct AppConfig {
    #[serde(default)]
    recent: Vec<PathBuf>,
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
struct AutosaveMeta {
    /// The file the autosaved work belonged to, if it had been saved before.
    source: Option<PathBuf>,
}

/// Something that would discard unsaved work, waiting for the user's choice.
#[derive(Debug, Clone, PartialEq)]
pub enum PendingAction {
    New,
    Demo,
    Open(PathBuf),
    Exit,
}

/// Per-user application folder (`%APPDATA%\FerroCAD`), falling back to the temp folder.
pub fn default_app_dir() -> PathBuf {
    std::env::var_os("APPDATA")
        .map(PathBuf::from)
        .unwrap_or_else(std::env::temp_dir)
        .join("FerroCAD")
}

fn file_label(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

impl FerroCadApp {
    /// Enables config, recent files and autosave in `dir`, and checks for work left over
    /// from a session that did not exit normally.
    pub fn with_persistence(mut self, dir: PathBuf) -> Self {
        let _ = std::fs::create_dir_all(&dir);
        self.app_dir = Some(dir);
        self.load_config();
        self.check_recovery();
        self
    }

    fn config_path(&self) -> Option<PathBuf> {
        self.app_dir.as_ref().map(|d| d.join("config.json"))
    }

    pub(crate) fn autosave_path(&self) -> Option<PathBuf> {
        self.app_dir.as_ref().map(|d| d.join("autosave.fcad"))
    }

    fn autosave_meta_path(&self) -> Option<PathBuf> {
        self.app_dir.as_ref().map(|d| d.join("autosave.meta.json"))
    }

    fn load_config(&mut self) {
        let Some(path) = self.config_path() else {
            return;
        };
        if let Ok(text) = std::fs::read_to_string(path)
            && let Ok(cfg) = serde_json::from_str::<AppConfig>(&text)
        {
            self.recent_files = cfg.recent;
        }
    }

    fn save_config(&self) {
        let Some(path) = self.config_path() else {
            return;
        };
        let cfg = AppConfig {
            recent: self.recent_files.clone(),
        };
        if let Ok(text) = serde_json::to_string_pretty(&cfg) {
            let _ = std::fs::write(path, text);
        }
    }

    fn add_recent(&mut self, path: &Path) {
        self.recent_files.retain(|p| p != path);
        self.recent_files.insert(0, path.to_path_buf());
        self.recent_files.truncate(MAX_RECENT);
        self.save_config();
    }

    pub(crate) fn has_unsaved_changes(&self) -> bool {
        self.controller.document.revision != self.saved_revision
    }

    /// Saves to `path`, or to the current file, or asks for a file name. Returns true
    /// when the document was written.
    pub(crate) fn save_document(&mut self, path: Option<PathBuf>) -> bool {
        let path = match path.or_else(|| self.current_file_path.clone()) {
            Some(p) => p,
            None => match rfd::FileDialog::new()
                .add_filter("FerroCAD", &["fcad"])
                .set_file_name(format!("{}.fcad", self.controller.document.name))
                .save_file()
            {
                Some(p) => p,
                None => return false,
            },
        };
        match self.controller.document.save_json(&path) {
            Ok(()) => {
                self.status_logs
                    .push(format!("保存しました: {}", path.display()));
                self.saved_revision = self.controller.document.revision;
                self.current_file_path = Some(path.clone());
                self.add_recent(&path);
                self.clear_autosave();
                true
            }
            Err(e) => {
                self.status_logs.push(format!("保存できませんでした: {e}"));
                false
            }
        }
    }

    /// Loads a document, replacing the current one (callers check for unsaved work).
    pub(crate) fn open_document(&mut self, path: PathBuf) {
        match Document::load_json(&path) {
            Ok(doc) => {
                self.replace_document(doc);
                self.current_file_path = Some(path.clone());
                self.saved_revision = self.controller.document.revision;
                self.add_recent(&path);
                self.status_logs
                    .push(format!("開きました: {}", path.display()));
            }
            Err(e) => {
                self.status_logs.push(format!(
                    "「{}」を開けませんでした（FerroCADのファイルではないか、壊れています）: {e}",
                    file_label(&path)
                ));
                self.recent_files.retain(|p| p != &path);
                self.save_config();
            }
        }
    }

    pub(crate) fn replace_document(&mut self, doc: Document) {
        self.store.clear();
        self.controller = DocumentController::new(doc);
        self.render_config.selected_feature = None;
        self.render_config.selected_face = None;
        self.render_config.selected_faces.clear();
        self.render_config.selected_edge = None;
        self.render_config.selected_edges.clear();
        self.tool_panel = None;
        self.rebind = None;
        self.measure = None;
        self.cancel_sketch();
        self.recompute_and_refresh();
        self.fit_all();
    }

    /// Runs `action`, first asking what to do with unsaved changes if there are any.
    pub(crate) fn request(&mut self, action: PendingAction) {
        if self.has_unsaved_changes() {
            self.pending_action = Some(action);
        } else {
            self.perform(action);
        }
    }

    fn perform(&mut self, action: PendingAction) {
        match action {
            PendingAction::New => {
                self.replace_document(Document::new("新規モデル"));
                self.current_file_path = None;
                self.saved_revision = self.controller.document.revision;
            }
            PendingAction::Demo => {
                let dir = self.app_dir.clone();
                let recent = std::mem::take(&mut self.recent_files);
                *self = Self::new_with_mvp_demo();
                self.app_dir = dir;
                self.recent_files = recent;
            }
            PendingAction::Open(path) => self.open_document(path),
            PendingAction::Exit => {
                self.allow_close = true;
                self.clear_autosave();
            }
        }
    }

    /// Writes the autosave every few seconds while there are unsaved changes.
    pub(crate) fn tick_autosave(&mut self, now: Instant) {
        if self.app_dir.is_none()
            || !self.has_unsaved_changes()
            || self.controller.document.revision == self.autosaved_revision
        {
            return;
        }
        if self.last_autosave.is_some_and(|t| now.duration_since(t) < AUTOSAVE_INTERVAL) {
            return;
        }
        self.last_autosave = Some(now);
        if self.write_autosave().is_ok() {
            self.autosaved_revision = self.controller.document.revision;
        }
    }

    /// Writes the current document to the autosave slot (atomically, via a temp file).
    pub(crate) fn write_autosave(&mut self) -> std::io::Result<()> {
        let (Some(path), Some(meta_path)) = (self.autosave_path(), self.autosave_meta_path()) else {
            return Ok(());
        };
        let json = self
            .controller
            .document
            .to_json_pretty()
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        let tmp = path.with_extension("fcad.tmp");
        std::fs::write(&tmp, json)?;
        std::fs::rename(&tmp, &path)?;
        let meta = AutosaveMeta {
            source: self.current_file_path.clone(),
        };
        std::fs::write(meta_path, serde_json::to_string(&meta).unwrap_or_default())?;
        Ok(())
    }

    pub(crate) fn clear_autosave(&self) {
        if let Some(p) = self.autosave_path() {
            let _ = std::fs::remove_file(p);
        }
        if let Some(p) = self.autosave_meta_path() {
            let _ = std::fs::remove_file(p);
        }
    }

    fn check_recovery(&mut self) {
        let Some(path) = self.autosave_path() else {
            return;
        };
        if !path.exists() {
            return;
        }
        let source = self
            .autosave_meta_path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|t| serde_json::from_str::<AutosaveMeta>(&t).ok())
            .and_then(|m| m.source);
        self.recovery_offer = Some(source);
    }

    /// Restores the autosaved work offered at startup.
    pub(crate) fn recover(&mut self) {
        let Some(source) = self.recovery_offer.take() else {
            return;
        };
        let Some(path) = self.autosave_path() else {
            return;
        };
        match Document::load_json(&path) {
            Ok(doc) => {
                self.replace_document(doc);
                self.current_file_path = source;
                // Recovered work has not been saved to its file yet.
                self.saved_revision = u64::MAX;
                self.status_logs
                    .push("前回の作業を復旧しました。必要なら保存してください（Ctrl+S）".to_string());
            }
            Err(e) => self
                .status_logs
                .push(format!("自動保存ファイルを読み込めませんでした: {e}")),
        }
    }

    pub(crate) fn discard_recovery(&mut self) {
        self.recovery_offer = None;
        self.clear_autosave();
    }

    /// Visible, computed bodies (what the user sees) for export.
    pub(crate) fn visible_bodies(&self) -> Vec<FeatureId> {
        let view = self.view_document();
        view.timeline()
            .into_iter()
            .filter(|id| {
                view.features
                    .get(id)
                    .is_some_and(|f| f.visible && f.cached_shape.is_some())
            })
            .collect()
    }

    /// Writes the visible bodies as STL, finely tessellated for printing.
    pub(crate) fn export_visible_stl(&mut self, path: &Path) -> bool {
        let bodies = self.visible_bodies();
        if bodies.is_empty() {
            self.status_logs
                .push("書き出す形状がありません（表示中のボディが対象です）".to_string());
            return false;
        }
        let fine = TessellationOptions {
            linear_deflection_mm: 0.02,
            angular_deflection_rad: 0.1,
        };
        match self
            .controller
            .document
            .export_stl(&self.store, &bodies, path, fine)
        {
            Ok(tris) => {
                self.status_logs.push(format!(
                    "STLを書き出しました: {}（ボディ {} 個 / 三角形 {} 枚）",
                    path.display(),
                    bodies.len(),
                    tris
                ));
                true
            }
            Err(e) => {
                self.status_logs.push(format!("STLを書き出せませんでした: {e}"));
                false
            }
        }
    }

    pub(crate) fn export_stl_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STL（3Dプリンタ用）", &["stl"])
            .set_file_name(format!("{}.stl", self.controller.document.name))
            .save_file()
        {
            self.export_visible_stl(&path);
        }
    }

    pub(crate) fn export_step_dialog(&mut self) {
        let Some(fid) = self.render_config.selected_feature else {
            self.status_logs
                .push("STEPで書き出すボディを選択してください".to_string());
            return;
        };
        let Some((name, shape)) = self
            .controller
            .document
            .features
            .get(&fid)
            .and_then(|f| f.cached_shape.map(|s| (f.name.clone(), s)))
        else {
            self.status_logs
                .push("選択中のフィーチャーには書き出せる形状がありません".to_string());
            return;
        };
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STEP", &["step", "stp"])
            .set_file_name(format!("{name}.step"))
            .save_file()
        {
            match self.store.export_step(shape, &path) {
                Ok(()) => self
                    .status_logs
                    .push(format!("STEPを書き出しました: {}", path.display())),
                Err(e) => self.status_logs.push(format!("STEPを書き出せませんでした: {e}")),
            }
        }
    }

    /// Adds a STEP file as a fixed body (its text is stored in the document).
    pub(crate) fn import_step_file(&mut self, path: &Path) -> bool {
        let text = match std::fs::read_to_string(path) {
            Ok(t) => t,
            Err(e) => {
                self.status_logs
                    .push(format!("ファイルを読めませんでした: {e}"));
                return false;
            }
        };
        let label = file_label(path);
        let before = self.controller.document.features.len();
        self.execute_cmd(CadCommand::AddFeature {
            name: label.trim_end_matches(".step").trim_end_matches(".stp").to_string(),
            kind: FeatureKind::ImportedBody {
                source_name: label.clone(),
                step_text: text,
            },
            visible: true,
            color_rgba: [0.75, 0.75, 0.78, 1.0],
        });
        let added = self.controller.document.features.len() > before;
        if added {
            self.fit_all();
            self.status_logs.push(format!(
                "「{label}」を取り込みました（寸法は変えられない固定の形状です。穴あけ・切り取り等は可能）"
            ));
        }
        added
    }

    pub(crate) fn import_step_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("STEP", &["step", "stp"])
            .pick_file()
        {
            self.import_step_file(&path);
        }
    }

    pub(crate) fn open_dialog(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("FerroCAD", &["fcad", "json"])
            .pick_file()
        {
            self.request(PendingAction::Open(path));
        }
    }

    pub(crate) fn window_title(&self) -> String {
        let name = self
            .current_file_path
            .as_deref()
            .map(file_label)
            .unwrap_or_else(|| "（未保存）".to_string());
        let dirty = if self.has_unsaved_changes() { " *" } else { "" };
        format!("{name}{dirty} — FerroCAD")
    }

    /// Modal dialogs: crash recovery at startup, and "save changes?" before discarding.
    pub(crate) fn render_file_dialogs(&mut self, ctx: &Context) {
        if let Some(source) = self.recovery_offer.clone() {
            let mut choice = None;
            egui::Window::new("前回の作業の復旧")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.label("前回は正常に終了しませんでした。自動保存された作業があります。");
                    if let Some(src) = &source {
                        ui.label(format!("元のファイル: {}", src.display()));
                    }
                    ui.horizontal(|ui| {
                        if ui.button("復旧する").clicked() {
                            choice = Some(true);
                        }
                        if ui.button("破棄する").clicked() {
                            choice = Some(false);
                        }
                    });
                });
            match choice {
                Some(true) => self.recover(),
                Some(false) => self.discard_recovery(),
                None => {}
            }
            return;
        }

        if let Some(action) = self.pending_action.clone() {
            let mut choice: Option<u8> = None;
            egui::Window::new("保存されていない変更があります")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
                .show(ctx, |ui| {
                    ui.label("変更を保存しますか？保存しないと、この変更は失われます。");
                    ui.horizontal(|ui| {
                        if ui.button("保存する").clicked() {
                            choice = Some(0);
                        }
                        if ui.button("保存しない").clicked() {
                            choice = Some(1);
                        }
                        if ui.button("キャンセル").clicked() {
                            choice = Some(2);
                        }
                    });
                });
            match choice {
                Some(0) => {
                    self.pending_action = None;
                    if self.save_document(None) {
                        self.perform(action);
                    }
                }
                Some(1) => {
                    self.pending_action = None;
                    self.perform(action);
                }
                Some(_) => self.pending_action = None,
                None => {}
            }
        }
    }
}
