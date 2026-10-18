//! 2D sketch editor: drawing tools, constraints and dimensions on a `cad_core::Sketch`,
//! solved live, then extruded (or written back into the feature being re-edited).

use super::*;
use cad_core::{
    EntityId, PointId, SketchConstraint, SketchEntity, SketchStatus, constraint_entity,
    constraint_points,
};

const HIT_PX: f32 = 8.0;
const COLOR_FREE: Color32 = Color32::from_rgb(120, 180, 255);
const COLOR_FULL: Color32 = Color32::from_rgb(90, 210, 120);
const COLOR_BAD: Color32 = Color32::from_rgb(240, 90, 90);
const COLOR_SELECTED: Color32 = Color32::from_rgb(250, 200, 60);
const COLOR_HOVER: Color32 = Color32::from_rgb(235, 235, 245);

/// Maps between the sketch plane and the 3D viewport through the camera, so the
/// sketch is drawn on (and clicked onto) its plane from whatever angle it is viewed.
#[derive(Clone, Copy)]
struct View {
    rect: Rect,
    vp: glam::Mat4,
    /// Inverse in double precision: un-projecting through the far plane in f32 loses
    /// hundredths of a millimetre.
    inv_vp: glam::DMat4,
    frame: PlaneFrame,
}

impl View {
    fn new(camera: &Camera, rect: Rect, frame: PlaneFrame) -> Self {
        let vp = camera.view_proj_matrix(rect.width() / rect.height().max(1.0));
        Self {
            rect,
            vp,
            inv_vp: vp.as_dmat4().inverse(),
            frame,
        }
    }

    fn world_to_screen(self, w: glam::DVec3) -> Pos2 {
        let clip = self.vp * glam::Vec4::new(w.x as f32, w.y as f32, w.z as f32, 1.0);
        let ndc = clip.truncate() / clip.w.max(1e-4);
        Pos2::new(
            self.rect.min.x + (ndc.x * 0.5 + 0.5) * self.rect.width(),
            self.rect.min.y + (0.5 - ndc.y * 0.5) * self.rect.height(),
        )
    }

    fn to_screen(self, p: [f64; 2]) -> Pos2 {
        self.world_to_screen(self.frame.to_world(p[0], p[1]))
    }

    /// Where the view ray under `pos` meets the sketch plane, in sketch coordinates.
    /// `None` when the ray runs (almost) parallel to the plane or meets it behind the eye.
    fn to_mm(self, pos: Pos2) -> Option<[f64; 2]> {
        let ndc_x = ((pos.x - self.rect.min.x) / self.rect.width()) as f64 * 2.0 - 1.0;
        let ndc_y = 1.0 - ((pos.y - self.rect.min.y) / self.rect.height()) as f64 * 2.0;
        let unproject = |z: f64| {
            let v = self.inv_vp * glam::DVec4::new(ndc_x, ndc_y, z, 1.0);
            v.truncate() / v.w
        };
        let near = unproject(0.0);
        let dir = (unproject(1.0) - near).normalize_or_zero();
        let denom = dir.dot(self.frame.normal);
        if denom.abs() < 1e-6 {
            return None;
        }
        let t = (self.frame.origin - near).dot(self.frame.normal) / denom;
        if t < 0.0 {
            return None;
        }
        Some(self.frame.to_local(near + dir * t))
    }

    /// Screen pixels per millimetre on the plane around `at` (averaged over both axes).
    fn px_per_mm(self, at: [f64; 2]) -> f32 {
        let o = self.to_screen(at);
        let x = self.to_screen([at[0] + 1.0, at[1]]).distance(o);
        let y = self.to_screen([at[0], at[1] + 1.0]).distance(o);
        ((x + y) * 0.5).max(1e-4)
    }

    /// A circle or arc on the plane, as a screen-space polyline.
    fn arc(self, c: [f64; 2], r: f64, a0: f64, sweep: f64, n: usize) -> Vec<Pos2> {
        (0..=n)
            .map(|i| {
                let t = a0 + sweep * i as f64 / n as f64;
                self.to_screen([c[0] + r * t.cos(), c[1] + r * t.sin()])
            })
            .collect()
    }
}

fn polyline_dist(p: Pos2, pts: &[Pos2]) -> f32 {
    pts.windows(2)
        .map(|w| seg_dist(p, w[0], w[1]))
        .fold(f32::INFINITY, f32::min)
}

fn seg_dist(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-6)).clamp(0.0, 1.0);
    (a + ab * t).distance(p)
}

fn dist_mm(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}


/// Short Japanese description of a constraint, for lists and messages.
pub(crate) fn describe_constraint(c: &SketchConstraint) -> String {
    match c {
        SketchConstraint::Fixed { .. } => "固定".into(),
        SketchConstraint::Coincident { .. } => "一致".into(),
        SketchConstraint::Horizontal { .. } => "水平".into(),
        SketchConstraint::Vertical { .. } => "垂直".into(),
        SketchConstraint::Parallel { .. } => "平行".into(),
        SketchConstraint::Perpendicular { .. } => "直交".into(),
        SketchConstraint::Tangent { .. } => "接線".into(),
        SketchConstraint::Distance { distance, .. } => format!("距離 {distance:.2} mm"),
        SketchConstraint::Radius { radius, .. } => format!("半径 R{radius:.2} mm"),
        SketchConstraint::Equal { .. } => "等長".into(),
    }
}

impl FerroCadApp {
    fn sketch_pos(&self, p: PointId) -> [f64; 2] {
        self.sketch
            .points
            .get(&p)
            .map(|q| [q.x, q.y])
            .unwrap_or([0.0, 0.0])
    }

    /// Snap targets from existing geometry that lies on the sketch plane (excluding the
    /// feature being re-edited): edge endpoints and midpoints, in sketch coordinates.
    fn sketch_reference_snaps(&self) -> Vec<[f64; 2]> {
        let frame = self.sketch_frame;
        let on_plane = |p: [f32; 3]| {
            let w = glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
            ((w - frame.origin).dot(frame.normal).abs() < 1e-3).then(|| frame.to_local(w))
        };
        let mut snaps = Vec::new();
        for feat in self.controller.document.features.values() {
            if !feat.visible || Some(feat.id) == self.sketch_editing {
                continue;
            }
            let Some(mesh) = feat.cached_mesh.as_ref() else {
                continue;
            };
            let mut edges: std::collections::BTreeMap<u32, Vec<[[f64; 2]; 2]>> =
                std::collections::BTreeMap::new();
            for (seg, &eid) in mesh.edge_segments.iter().zip(&mesh.edge_segment_ids) {
                if let (Some(a), Some(b)) = (on_plane(seg[0]), on_plane(seg[1])) {
                    edges.entry(eid).or_default().push([a, b]);
                }
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
        snaps
    }

    /// Solves the sketch in place and records the result.
    pub(crate) fn resolve_sketch(&mut self) {
        self.sketch_report = Some(self.sketch.solve(80, 1e-10));
    }

    /// Adds a constraint only if the sketch stays solvable and not over-constrained;
    /// otherwise explains why (redundant vs. conflicting, with the related constraints).
    pub(crate) fn try_add_constraint(&mut self, c: SketchConstraint, quiet: bool) -> bool {
        let mut trial = self.sketch.clone();
        trial.add_constraint(c.clone());
        let report = trial.solve(80, 1e-10);
        let rejected = matches!(
            report.status,
            SketchStatus::OverConstrained { .. } | SketchStatus::SolverFailed { .. }
        );
        if !rejected {
            self.sketch = trial;
            self.sketch_report = Some(report);
            return true;
        }
        if !quiet {
            let what = describe_constraint(&c);
            if report.max_residual < 1e-6 {
                self.status_logs.push(format!(
                    "「{what}」は追加しませんでした: その形はすでに他の拘束で決まっています"
                ));
            } else {
                let pts = constraint_points(&c);
                let ent = constraint_entity(&c);
                let related: Vec<String> = self
                    .sketch
                    .constraints
                    .iter()
                    .filter(|o| {
                        constraint_points(o).iter().any(|p| pts.contains(p))
                            || (ent.is_some() && constraint_entity(o) == ent)
                    })
                    .map(|o| format!("「{}」", describe_constraint(o)))
                    .collect();
                let hint = if related.is_empty() {
                    String::new()
                } else {
                    format!(" 関係する拘束: {}。不要なものを拘束一覧から外してください", related.join("、"))
                };
                self.status_logs.push(format!(
                    "「{what}」は追加しませんでした: 既存の拘束と矛盾します。{hint}"
                ));
            }
        }
        false
    }

    /// Changes a dimension's value if the sketch can still be solved with it.
    pub(crate) fn try_set_dimension(&mut self, index: usize, value: f64) -> bool {
        let mut trial = self.sketch.clone();
        match trial.constraints.get_mut(index) {
            Some(SketchConstraint::Distance { distance, .. }) => *distance = value,
            Some(SketchConstraint::Radius { radius, .. }) => *radius = value,
            _ => return false,
        }
        let report = trial.solve(120, 1e-10);
        if matches!(
            report.status,
            SketchStatus::OverConstrained { .. } | SketchStatus::SolverFailed { .. }
        ) {
            self.status_logs.push(format!(
                "寸法を {value:.2} mm にすると形が成り立たないため、元に戻しました"
            ));
            return false;
        }
        self.sketch = trial;
        self.sketch_report = Some(report);
        true
    }

    fn point_for(&mut self, click: ([f64; 2], Option<PointId>)) -> PointId {
        click
            .1
            .unwrap_or_else(|| self.sketch.add_point(click.0[0], click.0[1]))
    }

    /// Adds a line and, if it was drawn nearly horizontal/vertical, the matching constraint.
    fn add_sketch_line(&mut self, a: PointId, b: PointId) {
        self.sketch.add_line(a, b);
        let (pa, pb) = (self.sketch_pos(a), self.sketch_pos(b));
        let (dx, dy) = ((pb[0] - pa[0]).abs(), (pb[1] - pa[1]).abs());
        let tol = 2.0_f64.to_radians().tan();
        if dy <= dx * tol {
            self.try_add_constraint(SketchConstraint::Horizontal { p1: a, p2: b }, true);
        } else if dx <= dy * tol {
            self.try_add_constraint(SketchConstraint::Vertical { p1: a, p2: b }, true);
        }
        self.resolve_sketch();
    }

    fn hit_test(&self, view: View, pos: Pos2) -> Option<SketchSel> {
        let mut best: Option<(f32, SketchSel)> = None;
        let mut consider = |d: f32, sel: SketchSel, bias: f32| {
            let d = d - bias;
            if d <= HIT_PX && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, sel));
            }
        };
        for (&pid, p) in &self.sketch.points {
            consider(view.to_screen([p.x, p.y]).distance(pos), SketchSel::Point(pid), 3.0);
        }
        for (a, b) in self.sketch.segments() {
            let d = seg_dist(pos, view.to_screen(self.sketch_pos(a)), view.to_screen(self.sketch_pos(b)));
            consider(d, SketchSel::Segment(a, b), 0.0);
        }
        for ent in self.sketch.entities.values() {
            match ent {
                SketchEntity::Circle { id, center, radius } => {
                    let pts = view.arc(self.sketch_pos(*center), *radius, 0.0, std::f64::consts::TAU, 64);
                    consider(polyline_dist(pos, &pts), SketchSel::Curve(*id), 0.0);
                }
                SketchEntity::Arc {
                    id,
                    center,
                    start,
                    end,
                    radius,
                } => {
                    let c = self.sketch_pos(*center);
                    let (s, e) = (self.sketch_pos(*start), self.sketch_pos(*end));
                    let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                    let sweep = ((e[1] - c[1]).atan2(e[0] - c[0]) - a0).rem_euclid(std::f64::consts::TAU);
                    let pts = view.arc(c, *radius, a0, sweep, 48);
                    consider(polyline_dist(pos, &pts), SketchSel::Curve(*id), 0.0);
                }
                _ => {}
            }
        }
        best.map(|(_, s)| s)
    }

    /// Snap for drawing tools: sketch points (reused), existing-geometry snaps, grid.
    fn snap_click(
        &self,
        view: View,
        pos: Pos2,
        reference_snaps: &[[f64; 2]],
        grid_mm: f64,
    ) -> Option<([f64; 2], Option<PointId>)> {
        let raw = view.to_mm(pos)?;
        if !self.sketch_snap_enabled {
            return Some((raw, None));
        }
        let mut best: Option<(f32, [f64; 2], Option<PointId>)> = None;
        let mut consider = |cand: [f64; 2], pid: Option<PointId>, bias: f32| {
            let d = view.to_screen(cand).distance(pos) - bias;
            if d <= HIT_PX && best.is_none_or(|(bd, _, _)| d < bd) {
                best = Some((d, cand, pid));
            }
        };
        for (&pid, p) in &self.sketch.points {
            consider([p.x, p.y], Some(pid), 2.0);
        }
        for p in reference_snaps {
            consider(*p, None, 1.0);
        }
        consider(
            [(raw[0] / grid_mm).round() * grid_mm, (raw[1] / grid_mm).round() * grid_mm],
            None,
            0.0,
        );
        Some(best.map(|(_, p, pid)| (p, pid)).unwrap_or((raw, None)))
    }

    fn delete_sketch_selection(&mut self) {
        let mut doomed: Vec<EntityId> = Vec::new();
        for sel in &self.sketch_selection {
            for (&eid, ent) in &self.sketch.entities {
                let pts = cad_core::entity_points(ent);
                let hit = match *sel {
                    SketchSel::Curve(id) => id == eid,
                    SketchSel::Point(p) => pts.contains(&p),
                    SketchSel::Segment(a, b) => pts.contains(&a) && pts.contains(&b),
                };
                if hit && !doomed.contains(&eid) {
                    doomed.push(eid);
                }
            }
        }
        for eid in doomed {
            self.sketch.remove_entity(eid);
        }
        self.sketch_selection.clear();
        self.resolve_sketch();
    }

    /// Draws and edits the sketch on its plane, over the live 3D view: the camera can be
    /// orbited / zoomed / panned while sketching, and clicks land where the view ray
    /// meets the plane.
    pub(crate) fn render_sketch_overlay(
        &mut self,
        ctx: &Context,
        ui: &mut egui::Ui,
        rect: Rect,
        response: &egui::Response,
    ) {
        let view = View::new(&self.camera, rect, self.sketch_frame);
        let reference_snaps = if self.sketch_snap_enabled {
            self.sketch_reference_snaps()
        } else {
            Vec::new()
        };
        let painter = ui.painter_at(rect);
        // Sketch strokes get a dark outline so they stay visible over the shaded model.
        let halo = Color32::from_rgba_unmultiplied(8, 12, 20, 210);
        let stroke_line = |pts: Vec<Pos2>, color: Color32| {
            painter.add(egui::Shape::line(pts.clone(), egui::Stroke::new(5.0_f32, halo)));
            painter.add(egui::Shape::line(pts, egui::Stroke::new(2.2_f32, color)));
        };

        // Grid on the plane around the point under the view centre, spaced so lines stay
        // readable at the current zoom; the grid also sets the snap step.
        let focus = view.to_mm(rect.center()).unwrap_or([0.0, 0.0]);
        let ppm = view.px_per_mm(focus);
        let grid_mm: f64 = [1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0]
            .into_iter()
            .find(|g| *g as f32 * ppm >= 15.0)
            .unwrap_or(1000.0);
        let half_lines = (((self.camera.distance as f64) * 0.6 / grid_mm).ceil() as i64).clamp(4, 30);
        let reach = half_lines as f64 * grid_mm;
        let cx = (focus[0] / grid_mm).round() * grid_mm;
        let cy = (focus[1] / grid_mm).round() * grid_mm;
        let grid_stroke = egui::Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(170, 190, 220, 22));
        for i in -half_lines..=half_lines {
            let o = i as f64 * grid_mm;
            painter.line_segment(
                [view.to_screen([cx + o, cy - reach]), view.to_screen([cx + o, cy + reach])],
                grid_stroke,
            );
            painter.line_segment(
                [view.to_screen([cx - reach, cy + o]), view.to_screen([cx + reach, cy + o])],
                grid_stroke,
            );
        }
        let axis_reach = reach + focus[0].abs().max(focus[1].abs());
        painter.line_segment(
            [view.to_screen([-axis_reach, 0.0]), view.to_screen([axis_reach, 0.0])],
            egui::Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(230, 90, 90, 150)),
        );
        painter.line_segment(
            [view.to_screen([0.0, -axis_reach]), view.to_screen([0.0, axis_reach])],
            egui::Stroke::new(1.5_f32, Color32::from_rgba_unmultiplied(90, 220, 110, 150)),
        );
        for p in &reference_snaps {
            painter.circle_stroke(
                view.to_screen(*p),
                3.0,
                egui::Stroke::new(1.2_f32, Color32::from_rgb(90, 230, 210)),
            );
        }

        let hover_pos = response.hover_pos();
        let hovered = if self.sketch_tool == SketchTool::Select {
            hover_pos.and_then(|p| self.hit_test(view, p))
        } else {
            None
        };
        let snap_preview = hover_pos.and_then(|p| self.snap_click(view, p, &reference_snaps, grid_mm));

        self.handle_sketch_input(ctx, response, view, hovered, &reference_snaps, grid_mm);

        // Geometry, coloured by constraint state.
        let status_color = match self.sketch_report.as_ref().map(|r| &r.status) {
            Some(SketchStatus::FullyConstrained) => COLOR_FULL,
            Some(SketchStatus::OverConstrained { .. } | SketchStatus::SolverFailed { .. }) => COLOR_BAD,
            _ => COLOR_FREE,
        };
        let color_for = |sel: SketchSel| {
            if self.sketch_selection.contains(&sel) {
                COLOR_SELECTED
            } else if hovered == Some(sel) {
                COLOR_HOVER
            } else {
                status_color
            }
        };
        for (a, b) in self.sketch.segments() {
            stroke_line(
                vec![view.to_screen(self.sketch_pos(a)), view.to_screen(self.sketch_pos(b))],
                color_for(SketchSel::Segment(a, b)),
            );
        }
        for ent in self.sketch.entities.values() {
            match ent {
                SketchEntity::Circle { id, center, radius } => {
                    let pts = view.arc(self.sketch_pos(*center), *radius, 0.0, std::f64::consts::TAU, 64);
                    stroke_line(pts, color_for(SketchSel::Curve(*id)));
                }
                SketchEntity::Arc {
                    id,
                    center,
                    start,
                    end,
                    radius,
                } => {
                    let c = self.sketch_pos(*center);
                    let (s, e) = (self.sketch_pos(*start), self.sketch_pos(*end));
                    let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                    let sweep = ((e[1] - c[1]).atan2(e[0] - c[0]) - a0).rem_euclid(std::f64::consts::TAU);
                    stroke_line(view.arc(c, *radius, a0, sweep, 48), color_for(SketchSel::Curve(*id)));
                }
                _ => {}
            }
        }
        let fixed: Vec<PointId> = self
            .sketch
            .constraints
            .iter()
            .filter_map(|c| match c {
                SketchConstraint::Fixed { point, .. } => Some(*point),
                _ => None,
            })
            .collect();
        for &pid in self.sketch.points.keys() {
            let s = view.to_screen(self.sketch_pos(pid));
            let col = color_for(SketchSel::Point(pid));
            if fixed.contains(&pid) {
                painter.rect_filled(Rect::from_center_size(s, Vec2::splat(10.0)), 1.0, halo);
                painter.rect_filled(Rect::from_center_size(s, Vec2::splat(7.0)), 1.0, col);
            } else {
                painter.circle_filled(s, 4.8, halo);
                painter.circle_filled(s, 3.2, col);
            }
        }
        self.draw_constraint_glyphs(&painter, view);

        // Revolve axis, extended across the plane.
        if self.sketch_revolve {
            let (a, d) = match self.sketch_axis {
                cad_core::SketchAxis::Y => ([0.0, 0.0], [0.0, 1.0]),
                cad_core::SketchAxis::X => ([0.0, 0.0], [1.0, 0.0]),
                cad_core::SketchAxis::Line { a, b } => {
                    let (pa, pb) = (self.sketch_pos(a), self.sketch_pos(b));
                    let len = dist_mm(pa, pb).max(1e-9);
                    (pa, [(pb[0] - pa[0]) / len, (pb[1] - pa[1]) / len])
                }
            };
            let p0 = view.to_screen([a[0] - d[0] * axis_reach, a[1] - d[1] * axis_reach]);
            let p1 = view.to_screen([a[0] + d[0] * axis_reach, a[1] + d[1] * axis_reach]);
            painter.extend(egui::Shape::dashed_line(
                &[p0, p1],
                egui::Stroke::new(1.5_f32, Color32::from_rgb(200, 120, 255)),
                10.0,
                6.0,
            ));
        }

        // Preview of the shape the active tool will create (drawn on the plane).
        if let Some((cur, _)) = snap_preview {
            let preview = egui::Stroke::new(1.5_f32, COLOR_FREE.gamma_multiply(0.7));
            let cs = view.to_screen(cur);
            painter.circle_stroke(cs, 4.0, egui::Stroke::new(1.0_f32, COLOR_HOVER));
            match (self.sketch_tool, self.sketch_pending.as_slice()) {
                (SketchTool::Line, [.., (p, _)]) => {
                    painter.line_segment([view.to_screen(*p), cs], preview);
                }
                (SketchTool::Rect, [(p, _)]) => {
                    let corners = [*p, [cur[0], p[1]], cur, [p[0], cur[1]], *p];
                    painter.add(egui::Shape::line(
                        corners.iter().map(|c| view.to_screen(*c)).collect(),
                        preview,
                    ));
                }
                (SketchTool::Circle, [(p, _)]) => {
                    painter.add(egui::Shape::line(
                        view.arc(*p, dist_mm(*p, cur), 0.0, std::f64::consts::TAU, 64),
                        preview,
                    ));
                }
                (SketchTool::Arc, [(c, _)]) => {
                    painter.line_segment([view.to_screen(*c), cs], preview);
                }
                (SketchTool::Arc, [(c, _), (s, _)]) => {
                    let r = dist_mm(*c, *s);
                    let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                    let sweep = ((cur[1] - c[1]).atan2(cur[0] - c[0]) - a0).rem_euclid(std::f64::consts::TAU);
                    painter.add(egui::Shape::line(view.arc(*c, r, a0, sweep, 32), preview));
                }
                _ => {}
            }
        }

        self.draw_dimensions(ui, view);
        self.render_sketch_hud(ui, rect);
    }

    /// Turns the camera to look straight at the sketch plane (only on request).
    pub(crate) fn face_sketch_plane(&mut self) {
        let f = self.sketch_frame;
        let center = if self.sketch.points.is_empty() {
            let t = self.camera.target;
            let t = glam::DVec3::new(t.x as f64, t.y as f64, t.z as f64);
            let local = f.to_local(t);
            f.to_world(local[0], local[1])
        } else {
            let n = self.sketch.points.len() as f64;
            let (sx, sy) = self.sketch.points.values().fold((0.0, 0.0), |(x, y), p| (x + p.x, y + p.y));
            f.to_world(sx / n, sy / n)
        };
        self.camera.look_at_plane(
            glam::Vec3::new(center.x as f32, center.y as f32, center.z as f32),
            glam::Vec3::new(f.normal.x as f32, f.normal.y as f32, f.normal.z as f32),
        );
        self.viewport_dirty = true;
    }

    /// How squarely the camera faces the sketch plane (1 = head-on, 0 = edge-on).
    pub(crate) fn sketch_facing(&self) -> f64 {
        let fwd = self.camera.forward_dir();
        glam::DVec3::new(fwd.x as f64, fwd.y as f64, fwd.z as f64)
            .dot(self.sketch_frame.normal)
            .abs()
    }

    fn handle_sketch_input(
        &mut self,
        ctx: &Context,
        response: &egui::Response,
        view: View,
        hovered: Option<SketchSel>,
        reference_snaps: &[[f64; 2]],
        grid_mm: f64,
    ) {
        let typing = ctx.wants_keyboard_input();
        if !typing {
            for (key, tool) in [
                (Key::L, SketchTool::Line),
                (Key::R, SketchTool::Rect),
                (Key::C, SketchTool::Circle),
                (Key::A, SketchTool::Arc),
                (Key::S, SketchTool::Select),
            ] {
                if ctx.input(|i| i.key_pressed(key)) {
                    self.sketch_tool = tool;
                    self.sketch_pending.clear();
                }
            }
            if ctx.input(|i| i.key_pressed(Key::Escape)) {
                if !self.sketch_pending.is_empty() {
                    self.sketch_pending.clear();
                } else if !self.sketch_selection.is_empty() {
                    self.sketch_selection.clear();
                } else {
                    self.sketch_tool = SketchTool::Select;
                }
            }
            if ctx.input(|i| i.key_pressed(Key::Delete) || i.key_pressed(Key::Backspace))
                && !self.sketch_selection.is_empty()
            {
                self.delete_sketch_selection();
            }
        }

        if response.secondary_clicked() {
            self.sketch_pending.clear();
        }

        if self.sketch_tool == SketchTool::Select {
            if response.drag_started_by(egui::PointerButton::Primary)
                && let Some(SketchSel::Point(p)) = hovered
            {
                self.sketch_drag = Some(p);
            }
            if let (Some(p), Some(pos)) = (self.sketch_drag, response.interact_pointer_pos())
                && response.dragged_by(egui::PointerButton::Primary)
            {
                if let Some(m) = view.to_mm(pos)
                    && let Some(pt) = self.sketch.points.get_mut(&p)
                {
                    pt.x = m[0];
                    pt.y = m[1];
                    self.resolve_sketch();
                }
            }
            if response.drag_stopped() {
                self.sketch_drag = None;
            }
            if response.clicked() {
                let shift = ctx.input(|i| i.modifiers.shift);
                match hovered {
                    Some(sel) if shift => {
                        if let Some(i) = self.sketch_selection.iter().position(|s| *s == sel) {
                            self.sketch_selection.remove(i);
                        } else {
                            self.sketch_selection.push(sel);
                        }
                    }
                    Some(sel) => self.sketch_selection = vec![sel],
                    None => self.sketch_selection.clear(),
                }
            }
            return;
        }

        if !response.clicked() {
            return;
        }
        let Some(pos) = response.interact_pointer_pos() else {
            return;
        };
        let Some(click) = self.snap_click(view, pos, reference_snaps, grid_mm) else {
            return;
        };
        match self.sketch_tool {
            SketchTool::Line => {
                if let Some(&prev) = self.sketch_pending.last() {
                    if dist_mm(prev.0, click.0) < 1e-6 {
                        return;
                    }
                    let a = self.point_for(prev);
                    let b = self.point_for(click);
                    self.add_sketch_line(a, b);
                    // Clicking the chain's first point closes the loop and ends the chain.
                    let closes = self.sketch_pending.first().and_then(|f| f.1) == Some(b)
                        && self.sketch_pending.len() > 1;
                    if closes {
                        self.sketch_pending.clear();
                    } else {
                        if self.sketch_pending.len() == 1 && self.sketch_pending[0].1.is_none() {
                            self.sketch_pending[0].1 = Some(a);
                        }
                        let pos_b = self.sketch_pos(b);
                        self.sketch_pending.push((pos_b, Some(b)));
                    }
                } else {
                    self.sketch_pending.push(click);
                }
            }
            SketchTool::Rect => {
                if let Some(&first) = self.sketch_pending.first() {
                    let (a, b) = (first.0, click.0);
                    if (a[0] - b[0]).abs() < 1e-6 || (a[1] - b[1]).abs() < 1e-6 {
                        return;
                    }
                    let (x0, x1) = (a[0].min(b[0]), a[0].max(b[0]));
                    let (y0, y1) = (a[1].min(b[1]), a[1].max(b[1]));
                    let (_, corners) = self.sketch.add_rectangle(x0, y0, x1, y1);
                    // Tie the rectangle to points it was snapped onto.
                    for (p, existing) in [first, click] {
                        if let Some(e) = existing
                            && let Some(&corner) = corners
                                .iter()
                                .find(|c| dist_mm(self.sketch_pos(**c), p) < 1e-6)
                        {
                            self.try_add_constraint(SketchConstraint::Coincident { p1: e, p2: corner }, true);
                        }
                    }
                    self.resolve_sketch();
                    self.sketch_pending.clear();
                } else {
                    self.sketch_pending.push(click);
                }
            }
            SketchTool::Circle => {
                if let Some(&c) = self.sketch_pending.first() {
                    let r = dist_mm(c.0, click.0);
                    if r < 1e-6 {
                        return;
                    }
                    let center = self.point_for(c);
                    self.sketch.add_circle(center, r);
                    self.resolve_sketch();
                    self.sketch_pending.clear();
                } else {
                    self.sketch_pending.push(click);
                }
            }
            SketchTool::Arc => {
                if self.sketch_pending.len() < 2 {
                    if self.sketch_pending.len() == 1 && dist_mm(self.sketch_pending[0].0, click.0) < 1e-6 {
                        return;
                    }
                    self.sketch_pending.push(click);
                } else {
                    let (c, s) = (self.sketch_pending[0], self.sketch_pending[1]);
                    let r = dist_mm(c.0, s.0);
                    let ang = (click.0[1] - c.0[1]).atan2(click.0[0] - c.0[0]);
                    // The end lands on the arc's circle unless it snapped onto a point.
                    let end_click = match click.1 {
                        Some(_) => click,
                        None => ([c.0[0] + r * ang.cos(), c.0[1] + r * ang.sin()], None),
                    };
                    let center = self.point_for(c);
                    let start = self.point_for(s);
                    let end = self.point_for(end_click);
                    self.sketch.add_arc(center, start, end, r);
                    self.resolve_sketch();
                    self.sketch_pending.clear();
                }
            }
            SketchTool::Select => {}
        }
    }

    /// Small glyphs marking geometric constraints next to the geometry they act on.
    fn draw_constraint_glyphs(&self, painter: &egui::Painter, view: View) {
        let font = egui::FontId::proportional(11.0);
        let col = Color32::from_rgb(200, 170, 255);
        let mid = |a: PointId, b: PointId| {
            let (pa, pb) = (view.to_screen(self.sketch_pos(a)), view.to_screen(self.sketch_pos(b)));
            let m = pa + (pb - pa) * 0.5;
            let n = (pb - pa).normalized().rot90();
            m - n * 10.0
        };
        for c in &self.sketch.constraints {
            let marks: Vec<(Pos2, &str)> = match *c {
                SketchConstraint::Horizontal { p1, p2 } => vec![(mid(p1, p2), "H")],
                SketchConstraint::Vertical { p1, p2 } => vec![(mid(p1, p2), "V")],
                SketchConstraint::Parallel { a1, a2, b1, b2 } => vec![(mid(a1, a2), "∥"), (mid(b1, b2), "∥")],
                SketchConstraint::Perpendicular { a1, a2, b1, b2 } => {
                    vec![(mid(a1, a2), "⊥"), (mid(b1, b2), "⊥")]
                }
                SketchConstraint::Equal { a1, a2, b1, b2 } => vec![(mid(a1, a2), "="), (mid(b1, b2), "=")],
                SketchConstraint::Tangent { l1, l2, .. } => vec![(mid(l1, l2), "T")],
                _ => Vec::new(),
            };
            for (p, s) in marks {
                painter.text(p, egui::Align2::CENTER_CENTER, s, font.clone(), col);
            }
        }
    }

    /// Distance/radius dimensions drawn as labels on the canvas; click one to edit its
    /// value in place, right-click to remove it.
    fn draw_dimensions(&mut self, ui: &mut egui::Ui, view: View) {
        let mut set: Option<(usize, f64)> = None;
        let mut remove: Option<usize> = None;
        let mut start_edit: Option<(usize, f64)> = None;
        let mut stop_edit = false;
        let dims: Vec<(usize, Pos2, f64, bool)> = self
            .sketch
            .constraints
            .iter()
            .enumerate()
            .filter_map(|(i, c)| match *c {
                SketchConstraint::Distance { p1, p2, distance } => {
                    let (a, b) = (view.to_screen(self.sketch_pos(p1)), view.to_screen(self.sketch_pos(p2)));
                    let n = (b - a).normalized().rot90();
                    Some((i, a + (b - a) * 0.5 + n * 16.0, distance, false))
                }
                SketchConstraint::Radius { entity, radius } => {
                    let center = match self.sketch.entities.get(&entity)? {
                        SketchEntity::Circle { center, .. } | SketchEntity::Arc { center, .. } => *center,
                        _ => return None,
                    };
                    let c = self.sketch_pos(center);
                    let k = std::f64::consts::FRAC_1_SQRT_2 * radius;
                    Some((i, view.to_screen([c[0] + k, c[1] + k]), radius, true))
                }
                _ => None,
            })
            .collect();

        for (i, pos, value, is_radius) in dims {
            let rect = Rect::from_center_size(pos, Vec2::new(70.0, 20.0));
            if let Some((ei, mut v)) = self.sketch_dim_edit.filter(|(ei, _)| *ei == i) {
                let resp = ui.put(
                    rect,
                    egui::DragValue::new(&mut v).speed(0.1).range(0.01..=100000.0).suffix(" mm"),
                );
                resp.request_focus();
                self.sketch_dim_edit = Some((ei, v));
                if resp.lost_focus() || ui.input(|i| i.key_pressed(Key::Enter)) {
                    set = Some((i, v));
                    stop_edit = true;
                }
            } else {
                let text = if is_radius {
                    format!("R{value:.2}")
                } else {
                    format!("{value:.2}")
                };
                let resp = ui
                    .put(
                        rect,
                        egui::Button::new(RichText::new(text).size(12.0).color(Color32::from_rgb(255, 225, 140)))
                            .fill(Color32::from_rgba_unmultiplied(30, 34, 44, 220)),
                    )
                    .on_hover_text("クリックで寸法を変更 / 右クリックで削除");
                if resp.clicked() {
                    start_edit = Some((i, value));
                }
                if resp.secondary_clicked() {
                    remove = Some(i);
                }
            }
        }

        if let Some((i, v)) = set {
            self.try_set_dimension(i, v);
        }
        if stop_edit {
            self.sketch_dim_edit = None;
        }
        if let Some(edit) = start_edit {
            self.sketch_dim_edit = Some(edit);
        }
        if let Some(i) = remove {
            self.sketch.remove_constraint(i);
            self.sketch_dim_edit = None;
            self.resolve_sketch();
        }
    }

    /// Constraints applicable to the current selection, as (label, constraint).
    fn applicable_constraints(&self) -> Vec<(&'static str, SketchConstraint)> {
        let sel = &self.sketch_selection;
        let points: Vec<PointId> = sel
            .iter()
            .filter_map(|s| match s {
                SketchSel::Point(p) => Some(*p),
                _ => None,
            })
            .collect();
        let segs: Vec<(PointId, PointId)> = sel
            .iter()
            .filter_map(|s| match s {
                SketchSel::Segment(a, b) => Some((*a, *b)),
                _ => None,
            })
            .collect();
        let curves: Vec<EntityId> = sel
            .iter()
            .filter_map(|s| match s {
                SketchSel::Curve(e) => Some(*e),
                _ => None,
            })
            .collect();
        let mut out = Vec::new();
        let len = |a: PointId, b: PointId| dist_mm(self.sketch_pos(a), self.sketch_pos(b));
        match (points.as_slice(), segs.as_slice(), curves.as_slice()) {
            ([], [(a, b)], []) | ([a, b], [], []) => {
                out.push(("水平", SketchConstraint::Horizontal { p1: *a, p2: *b }));
                out.push(("垂直", SketchConstraint::Vertical { p1: *a, p2: *b }));
                if !points.is_empty() {
                    out.push(("一致", SketchConstraint::Coincident { p1: *a, p2: *b }));
                }
                out.push((
                    if points.is_empty() { "長さ" } else { "距離" },
                    SketchConstraint::Distance {
                        p1: *a,
                        p2: *b,
                        distance: len(*a, *b),
                    },
                ));
            }
            ([], [(a1, a2), (b1, b2)], []) => {
                out.push(("平行", SketchConstraint::Parallel { a1: *a1, a2: *a2, b1: *b1, b2: *b2 }));
                out.push(("直交", SketchConstraint::Perpendicular { a1: *a1, a2: *a2, b1: *b1, b2: *b2 }));
                out.push(("等長", SketchConstraint::Equal { a1: *a1, a2: *a2, b1: *b1, b2: *b2 }));
            }
            ([], [], [e]) => {
                if let Some(SketchEntity::Circle { radius, .. } | SketchEntity::Arc { radius, .. }) =
                    self.sketch.entities.get(e)
                {
                    out.push(("半径", SketchConstraint::Radius { entity: *e, radius: *radius }));
                }
            }
            ([], [(l1, l2)], [e]) => {
                if let Some(SketchEntity::Circle { center, .. } | SketchEntity::Arc { center, .. }) =
                    self.sketch.entities.get(e)
                {
                    out.push((
                        "接線",
                        SketchConstraint::Tangent { l1: *l1, l2: *l2, center: *center, circle: *e },
                    ));
                }
            }
            _ => {}
        }
        if !points.is_empty() && segs.is_empty() && curves.is_empty() && points.len() == 1 {
            let p = self.sketch_pos(points[0]);
            out.push(("固定", SketchConstraint::Fixed { point: points[0], x: p[0], y: p[1] }));
        }
        out
    }

    fn render_sketch_hud(&mut self, ui: &mut egui::Ui, rect: Rect) {
        let hud_rect = Rect::from_min_size(
            rect.min + Vec2::new(12.0, 12.0),
            Vec2::new(470.0, (rect.height() - 24.0).max(200.0)),
        );
        let mut finish = false;
        let mut cancel = false;
        ui.allocate_new_ui(egui::UiBuilder::new().max_rect(hud_rect), |ui| {
            egui::Frame::window(ui.style())
                .fill(Color32::from_rgba_unmultiplied(22, 26, 34, 230))
                .show(ui, |ui| {
                    let plane_label = match &self.sketch_plane {
                        SketchPlane::XY => "XY平面".to_string(),
                        SketchPlane::XZ => "XZ平面".to_string(),
                        SketchPlane::YZ => "YZ平面".to_string(),
                        SketchPlane::Face { feature, .. } => format!("「{}」の面", self.feature_name(*feature)),
                        SketchPlane::OffsetFace { feature, offset, .. } => {
                            let distance = match offset {
                                ScalarParam::Literal(value) => format!("{value:.2} mm"),
                                ScalarParam::Expr(expression) => expression.clone(),
                            };
                            format!("「{}」の面から {}", self.feature_name(*feature), distance)
                        }
                    };
                    let editing = self
                        .sketch_editing
                        .map(|id| format!("（「{}」を編集中）", self.feature_name(id)))
                        .unwrap_or_default();
                    ui.label(RichText::new(format!("✏ スケッチ: {plane_label}{editing}")).strong());

                    ui.horizontal(|ui| {
                        for (tool, label, key) in [
                            (SketchTool::Select, "↖ 選択", "S"),
                            (SketchTool::Line, "／ 線", "L"),
                            (SketchTool::Rect, "□ 矩形", "R"),
                            (SketchTool::Circle, "○ 円", "C"),
                            (SketchTool::Arc, "⌒ 円弧", "A"),
                        ] {
                            if ui
                                .selectable_label(self.sketch_tool == tool, label)
                                .on_hover_text(format!("ショートカット: {key}"))
                                .clicked()
                            {
                                self.sketch_tool = tool;
                                self.sketch_pending.clear();
                            }
                        }
                    });
                    let hint = match (self.sketch_tool, self.sketch_pending.len()) {
                        (SketchTool::Select, _) => {
                            "点・線・円をクリックで選択（Shiftで追加）、点はドラッグで移動、Deleteで削除"
                        }
                        (SketchTool::Line, 0) => "始点をクリック",
                        (SketchTool::Line, _) => "次の点をクリック（始点をクリックで閉じる / 右クリック・Escで終了）",
                        (SketchTool::Rect, 0) => "1つ目の角をクリック",
                        (SketchTool::Rect, _) => "対角の角をクリック",
                        (SketchTool::Circle, 0) => "中心をクリック",
                        (SketchTool::Circle, _) => "円周上の点をクリック",
                        (SketchTool::Arc, 0) => "中心をクリック",
                        (SketchTool::Arc, 1) => "始点をクリック",
                        (SketchTool::Arc, _) => "終点をクリック（反時計回り）",
                    };
                    ui.small(hint);
                    ui.horizontal(|ui| {
                        ui.small("左ドラッグ: 回転 / 右ドラッグ: 移動 / ホイール: 拡大縮小");
                        if ui
                            .small_button("面を正面に向ける")
                            .on_hover_text("スケッチ面を真正面から見る向きにカメラを回します")
                            .clicked()
                        {
                            self.face_sketch_plane();
                        }
                    });
                    if self.sketch_facing() < 0.2 {
                        ui.colored_label(
                            Color32::from_rgb(240, 170, 70),
                            "面をほぼ真横から見ているため、クリック位置がずれやすくなっています。回転するか［面を正面に向ける］を押してください",
                        );
                    }

                    ui.separator();
                    let options = self.applicable_constraints();
                    ui.horizontal_wrapped(|ui| {
                        ui.label("拘束:");
                        if options.is_empty() {
                            ui.weak("（選択ツールで線・点・円を選ぶと使えるものが出ます）");
                        }
                        for (label, c) in options {
                            if ui.button(label).clicked() && self.try_add_constraint(c, false) {
                                self.sketch_selection.clear();
                            }
                        }
                    });

                    if self.sketch.points.is_empty() {
                        ui.weak("まだ図形がありません。上のツールで描き始めてください");
                    } else if let Some(report) = &self.sketch_report {
                        let (text, color) = match report.status {
                            SketchStatus::FullyConstrained => {
                                ("✓ 完全拘束（形と位置がすべて決まっています）".to_string(), COLOR_FULL)
                            }
                            SketchStatus::UnderConstrained { remaining_dof } => (
                                format!("あと {remaining_dof} 自由度（青い部分はまだ動かせます。寸法や「固定」で決められます）"),
                                COLOR_FREE,
                            ),
                            SketchStatus::OverConstrained { .. } => (
                                "拘束が多すぎます。拘束一覧から不要なものを外してください".to_string(),
                                COLOR_BAD,
                            ),
                            SketchStatus::SolverFailed { .. } => (
                                "拘束を満たす形が見つかりません。最後に追加した拘束を外してください".to_string(),
                                COLOR_BAD,
                            ),
                        };
                        ui.colored_label(color, text);
                    }

                    let mut remove: Option<usize> = None;
                    egui::CollapsingHeader::new(format!("拘束一覧（{}）", self.sketch.constraints.len()))
                        .id_salt("sketch_constraint_list")
                        .show(ui, |ui| {
                            egui::ScrollArea::vertical().max_height(140.0).show(ui, |ui| {
                                for (i, c) in self.sketch.constraints.iter().enumerate() {
                                    ui.horizontal(|ui| {
                                        if ui.small_button("×").on_hover_text("この拘束を外す").clicked() {
                                            remove = Some(i);
                                        }
                                        ui.label(describe_constraint(c));
                                    });
                                }
                            });
                        });
                    if let Some(i) = remove {
                        self.sketch.remove_constraint(i);
                        self.sketch_dim_edit = None;
                        self.resolve_sketch();
                    }
                    ui.checkbox(
                        &mut self.sketch_snap_enabled,
                        "スナップ（スケッチの点 / グリッド / 既存形状の角・辺の中点 ○）",
                    );

                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label("立体化:");
                        let locked = self.sketch_editing.is_some();
                        for (revolve, label) in [(false, "押し出し"), (true, "回転")] {
                            if ui
                                .add_enabled(!locked, egui::SelectableLabel::new(self.sketch_revolve == revolve, label))
                                .on_disabled_hover_text("編集中のフィーチャーの種類は変更できません")
                                .clicked()
                            {
                                self.sketch_revolve = revolve;
                            }
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("結果:");
                        let has_target = self.sketch_target.is_some();
                        let target_hint = match self.sketch_target {
                            Some(t) => format!("「{}」と組み合わせます", self.feature_name(t)),
                            None => {
                                "結合・切り取りは、面の上でスケッチするか、先に対象を選択してから使えます".to_string()
                            }
                        };
                        let locked = self.sketch_editing.is_some();
                        for (op, label) in [
                            (SketchOp::NewBody, "新規"),
                            (SketchOp::Join, "結合"),
                            (SketchOp::Cut, "切り取り"),
                        ] {
                            let enabled = !locked && (op == SketchOp::NewBody || has_target);
                            if ui
                                .add_enabled(enabled, egui::SelectableLabel::new(self.sketch_op == op, label))
                                .on_hover_text(&target_hint)
                                .on_disabled_hover_text(if locked {
                                    "編集中のフィーチャーの種類は変更できません"
                                } else {
                                    target_hint.as_str()
                                })
                                .clicked()
                            {
                                self.set_sketch_op(op);
                            }
                        }
                    });
                    if self.sketch_revolve {
                        let selected_line = self.sketch_selection.iter().find_map(|s| match s {
                            SketchSel::Segment(a, b) => Some((*a, *b)),
                            _ => None,
                        });
                        ui.horizontal(|ui| {
                            ui.label("軸:");
                            if ui
                                .selectable_label(self.sketch_axis == cad_core::SketchAxis::Y, "縦軸 (Y)")
                                .clicked()
                            {
                                self.sketch_axis = cad_core::SketchAxis::Y;
                            }
                            if ui
                                .selectable_label(self.sketch_axis == cad_core::SketchAxis::X, "横軸 (X)")
                                .clicked()
                            {
                                self.sketch_axis = cad_core::SketchAxis::X;
                            }
                            let is_line = matches!(self.sketch_axis, cad_core::SketchAxis::Line { .. });
                            if ui
                                .add_enabled(
                                    selected_line.is_some() || is_line,
                                    egui::SelectableLabel::new(is_line, "選択した線"),
                                )
                                .on_hover_text("選択ツールで線を1本選んでから押すと、その線を軸にします")
                                .on_disabled_hover_text("選択ツールで軸にしたい線を1本選んでください")
                                .clicked()
                                && let Some((a, b)) = selected_line
                            {
                                self.sketch_axis = cad_core::SketchAxis::Line { a, b };
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("角度:");
                            ui.add(
                                egui::DragValue::new(&mut self.sketch_angle)
                                    .speed(1.0)
                                    .suffix("°")
                                    .range(1.0..=360.0),
                            );
                            ui.small("輪郭は軸の片側に描いてください（紫の破線が軸）");
                        });
                    } else {
                        ui.horizontal(|ui| {
                            ui.label("量:");
                            let through = self.sketch_through_all && self.sketch_op != SketchOp::NewBody;
                            if through {
                                ui.weak("（貫通）");
                            } else if let Some(expr) = &self.sketch_depth_expr {
                                ui.label(format!("式 {expr}")).on_hover_text("式で指定された量はそのまま保持されます");
                            } else {
                                ui.add(
                                    egui::DragValue::new(&mut self.sketch_depth)
                                        .speed(0.5)
                                        .suffix(" mm")
                                        .range(0.1..=100000.0),
                                );
                            }
                            ui.add_enabled(
                                self.sketch_op != SketchOp::NewBody,
                                egui::Checkbox::new(&mut self.sketch_through_all, "貫通"),
                            )
                            .on_hover_text("対象のボディを突き抜けるまで押し出します")
                            .on_disabled_hover_text("結合・切り取りのときに使えます");
                            ui.checkbox(&mut self.sketch_reversed, "方向を反転")
                                .on_hover_text("切り取りでは、既定で面の内側（材料側）へ向かいます");
                            ui.checkbox(&mut self.sketch_symmetric, "両側");
                        });
                    }

                    let loops = self.sketch.extract_closed_profiles().len();
                    ui.horizontal(|ui| {
                        let label = if self.sketch_editing.is_some() {
                            "✓ 更新して閉じる"
                        } else if self.sketch_revolve {
                            "✓ 回転して閉じる"
                        } else {
                            "✓ 押し出して閉じる"
                        };
                        let btn = ui.add_enabled(loops > 0, egui::Button::new(label));
                        let btn = if loops == 0 {
                            btn.on_disabled_hover_text("閉じた輪郭がまだありません（線を始点まで戻すか、矩形・円を描いてください）")
                        } else {
                            btn.on_hover_text(format!("閉じた輪郭: {loops} 個（内側の輪郭は穴になります）"))
                        };
                        if btn.clicked() {
                            finish = true;
                        }
                        if ui.button("× キャンセル").clicked() {
                            cancel = true;
                        }
                    });
                });
        });
        if finish {
            self.finish_sketch_extrude();
        } else if cancel {
            self.cancel_sketch();
        }
    }

    /// Writes the sketch into the document: a new SketchExtrude feature, or an update of
    /// the feature being re-edited (which recomputes everything downstream of it).
    pub(crate) fn finish_sketch_extrude(&mut self) {
        if self.sketch.extract_closed_profiles().is_empty() {
            self.status_logs
                .push("閉じた輪郭がないため押し出せません".to_string());
            return;
        }
        let depth = match &self.sketch_depth_expr {
            Some(expr) => ScalarParam::Expr(expr.clone()),
            None => ScalarParam::mm(self.sketch_depth.max(0.1)),
        };
        let (mode, base_name) = match (self.sketch_op, self.sketch_target) {
            (SketchOp::Join, Some(target)) => (ExtrudeMode::Join { target }, "Join"),
            (SketchOp::Cut, Some(target)) => (ExtrudeMode::Cut { target }, "Cut"),
            _ => (ExtrudeMode::NewBody, "Extrude"),
        };
        let kind = if self.sketch_revolve {
            FeatureKind::SketchRevolve {
                sketch: self.sketch.clone(),
                plane: self.sketch_plane.clone(),
                axis: self.sketch_axis,
                angle_deg: ScalarParam::Literal(self.sketch_angle.clamp(1.0, 360.0)),
                mode,
            }
        } else {
            FeatureKind::SketchExtrude {
                sketch: self.sketch.clone(),
                depth,
                plane: self.sketch_plane.clone(),
                mode,
                reversed: self.sketch_reversed,
                symmetric: self.sketch_symmetric,
                through_all: self.sketch_through_all && mode != ExtrudeMode::NewBody,
            }
        };
        let base_name = if self.sketch_revolve {
            match mode {
                ExtrudeMode::NewBody => "Revolve",
                ExtrudeMode::Join { .. } => "RevolveJoin",
                ExtrudeMode::Cut { .. } => "RevolveCut",
            }
        } else {
            base_name
        };

        if let Some(id) = self.sketch_editing.take() {
            let name = self.feature_name(id);
            self.execute_cmd(CadCommand::UpdateFeature {
                id,
                name: None,
                kind: Some(kind),
                visible: None,
                color_rgba: None,
            });
            self.status_logs
                .push(format!("「{name}」のスケッチを更新しました（後続のフィーチャーも再計算されます）"));
        } else {
            let color_rgba = self
                .sketch_target
                .filter(|_| mode != ExtrudeMode::NewBody)
                .and_then(|t| self.controller.document.features.get(&t))
                .map(|f| f.color_rgba)
                .unwrap_or([0.55, 0.75, 0.95, 1.0]);
            let before = self.controller.document.features.len();
            self.execute_cmd(CadCommand::AddFeature {
                name: format!("{base_name}_{}", before + 1),
                kind,
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
        }

        self.sketch_mode = false;
        self.sketch_pending.clear();
        self.sketch_selection.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// From an oblique camera, a point on the sketch plane projected to the screen and
    /// clicked there maps back to the same sketch coordinates (top face of a plate, and
    /// a tilted plane).
    #[test]
    fn oblique_view_round_trips_between_screen_and_plane() {
        let mut camera = Camera::default();
        camera.target = glam::Vec3::new(50.0, 30.0, 10.0);
        camera.distance = 200.0;
        camera.yaw_rad = -0.9;
        camera.pitch_rad = 0.45;
        let rect = Rect::from_min_size(Pos2::new(300.0, 120.0), Vec2::new(1000.0, 700.0));
        for frame in [
            PlaneFrame::from_point_normal(glam::DVec3::new(0.0, 0.0, 10.0), glam::DVec3::Z),
            PlaneFrame::from_point_normal(glam::DVec3::new(20.0, 10.0, 5.0), glam::DVec3::new(0.3, -0.5, 0.8)),
        ] {
            let view = View::new(&camera, rect, frame);
            for p in [[0.0, 0.0], [37.5, 12.25], [-20.0, 44.0], [90.0, -15.0]] {
                let screen = view.to_screen(p);
                let back = view.to_mm(screen).expect("plane is in front of the camera");
                assert!(dist_mm(p, back) < 1e-3, "{p:?} -> {screen:?} -> {back:?}");
            }
        }

        // Looking along the plane (edge-on) gives no usable intersection.
        let mut edge_on = camera.clone();
        edge_on.pitch_rad = 0.0;
        let flat = PlaneFrame::from_point_normal(glam::DVec3::new(0.0, 0.0, 10.0), glam::DVec3::Z);
        let target_screen = View::new(&edge_on, rect, flat).world_to_screen(glam::DVec3::new(50.0, 30.0, 10.0));
        let _ = View::new(&edge_on, rect, flat).to_mm(target_screen);
    }
}
