//! 2D Parametric Sketcher with Geometric Constraint Solver (Phase 5A).
//!
//! Supports basic 2D entities (`Point`, `Line`, `Circle`, `Arc`, `Rectangle`, `Polyline`),
//! geometric constraints (`Coincident`, `Fixed`, `Horizontal`, `Vertical`, `Parallel`,
//! `Perpendicular`, `Tangent`, `Distance`, `Radius`, `Equal`), iterative Gauss-Newton
//! solver, Degrees-of-Freedom (DOF) analysis, and closed profile extraction for `Extrude`.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

/// Identifier for a 2D point in a [`Sketch`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct PointId(pub u32);

/// Identifier for a 2D curve entity in a [`Sketch`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct EntityId(pub u32);

/// A 2D point in sketch coordinates $(x, y)$ in millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SketchPoint {
    pub id: PointId,
    pub x: f64,
    pub y: f64,
}

/// 2D geometric entity in a [`Sketch`] (Phase 5A).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SketchEntity {
    Line {
        id: EntityId,
        start: PointId,
        end: PointId,
    },
    Circle {
        id: EntityId,
        center: PointId,
        radius: f64,
    },
    Arc {
        id: EntityId,
        center: PointId,
        start: PointId,
        end: PointId,
        radius: f64,
    },
    Rectangle {
        id: EntityId,
        /// Corner points in counter-clockwise order: `[bottom_left, bottom_right, top_right, top_left]`.
        corners: [PointId; 4],
    },
    Polyline {
        id: EntityId,
        points: Vec<PointId>,
        closed: bool,
    },
}

/// Geometric and dimensional constraints supported by the 2D Sketcher (Phase 5A).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SketchConstraint {
    /// Fixes a point at exact $(x, y)$ coordinates.
    Fixed { point: PointId, x: f64, y: f64 },
    /// Forces two points to occupy the same position.
    Coincident { p1: PointId, p2: PointId },
    /// Forces two points (or a line's endpoints) to have the same $y$ coordinate.
    Horizontal { p1: PointId, p2: PointId },
    /// Forces two points (or a line's endpoints) to have the same $x$ coordinate.
    Vertical { p1: PointId, p2: PointId },
    /// Forces line `(a1, a2)` and line `(b1, b2)` to be parallel.
    Parallel {
        a1: PointId,
        a2: PointId,
        b1: PointId,
        b2: PointId,
    },
    /// Forces line `(a1, a2)` and line `(b1, b2)` to be perpendicular.
    Perpendicular {
        a1: PointId,
        a2: PointId,
        b1: PointId,
        b2: PointId,
    },
    /// Forces a line `(l1, l2)` to be tangent to a circle with `center` and `radius_entity`.
    Tangent {
        l1: PointId,
        l2: PointId,
        center: PointId,
        circle: EntityId,
    },
    /// Constrains the Euclidean distance between `p1` and `p2` to `distance`.
    Distance {
        p1: PointId,
        p2: PointId,
        distance: f64,
    },
    /// Constrains the radius of a `Circle` or `Arc` entity.
    Radius { entity: EntityId, radius: f64 },
    /// Constrains two segments `(a1, a2)` and `(b1, b2)` to have equal length.
    Equal {
        a1: PointId,
        a2: PointId,
        b1: PointId,
        b2: PointId,
    },
}

/// Overall constraint state of a [`Sketch`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum SketchStatus {
    FullyConstrained,
    UnderConstrained { remaining_dof: usize },
    OverConstrained { conflicting_equations: usize },
    SolverFailed { residual_norm: String },
}

/// Result returned after solving a [`Sketch`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SketchSolveReport {
    pub status: SketchStatus,
    pub iterations: usize,
    pub max_residual: f64,
    pub total_variables: usize,
    pub total_scalar_equations: usize,
    pub remaining_dof: usize,
}

/// A 2D parametric sketch consisting of points, curves, and geometric constraints.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Sketch {
    pub name: String,
    next_point_id: u32,
    next_entity_id: u32,
    pub points: BTreeMap<PointId, SketchPoint>,
    pub entities: BTreeMap<EntityId, SketchEntity>,
    pub constraints: Vec<SketchConstraint>,
}

impl Sketch {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            next_point_id: 1,
            next_entity_id: 1,
            points: BTreeMap::new(),
            entities: BTreeMap::new(),
            constraints: Vec::new(),
        }
    }

    pub fn add_point(&mut self, x: f64, y: f64) -> PointId {
        let id = PointId(self.next_point_id);
        self.next_point_id += 1;
        self.points.insert(id, SketchPoint { id, x, y });
        id
    }

    pub fn add_line(&mut self, start: PointId, end: PointId) -> EntityId {
        let id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        self.entities
            .insert(id, SketchEntity::Line { id, start, end });
        id
    }

    pub fn add_circle(&mut self, center: PointId, radius: f64) -> EntityId {
        let id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        self.entities.insert(
            id,
            SketchEntity::Circle {
                id,
                center,
                radius: radius.max(0.1),
            },
        );
        id
    }

    pub fn add_arc(
        &mut self,
        center: PointId,
        start: PointId,
        end: PointId,
        radius: f64,
    ) -> EntityId {
        let id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        self.entities.insert(
            id,
            SketchEntity::Arc {
                id,
                center,
                start,
                end,
                radius: radius.max(0.1),
            },
        );
        id
    }

    /// Adds a rectangle with initial lower-left `(x0, y0)` and upper-right `(x1, y1)`,
    /// automatically registering `Horizontal` and `Vertical` constraints on its four sides.
    pub fn add_rectangle(
        &mut self,
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
    ) -> (EntityId, [PointId; 4]) {
        let p0 = self.add_point(x0, y0);
        let p1 = self.add_point(x1, y0);
        let p2 = self.add_point(x1, y1);
        let p3 = self.add_point(x0, y1);
        let id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        let corners = [p0, p1, p2, p3];
        self.entities
            .insert(id, SketchEntity::Rectangle { id, corners });
        self.add_constraint(SketchConstraint::Horizontal { p1: p0, p2: p1 });
        self.add_constraint(SketchConstraint::Horizontal { p1: p3, p2 });
        self.add_constraint(SketchConstraint::Vertical { p1: p0, p2: p3 });
        self.add_constraint(SketchConstraint::Vertical { p1, p2 });
        (id, corners)
    }

    pub fn add_polyline(&mut self, points: Vec<PointId>, closed: bool) -> EntityId {
        let id = EntityId(self.next_entity_id);
        self.next_entity_id += 1;
        self.entities
            .insert(id, SketchEntity::Polyline { id, points, closed });
        id
    }

    pub fn add_constraint(&mut self, constraint: SketchConstraint) {
        self.constraints.push(constraint);
    }

    /// Solves the geometric constraints using damped Gauss-Newton (Levenberg-Marquardt) iteration
    /// and computes the remaining Degrees of Freedom (DOF).
    pub fn solve(&mut self, max_iterations: usize, tolerance: f64) -> SketchSolveReport {
        // Pack variables: 2 per point (x, y) + 1 per Circle/Arc radius
        let point_ids: Vec<PointId> = self.points.keys().copied().collect();
        let mut point_var_offset = BTreeMap::new();
        let mut vars = Vec::new();

        for pid in &point_ids {
            point_var_offset.insert(*pid, vars.len());
            let pt = self.points[pid];
            vars.push(pt.x);
            vars.push(pt.y);
        }

        let mut radius_var_offset = BTreeMap::new();
        for (&eid, ent) in &self.entities {
            match ent {
                SketchEntity::Circle { radius, .. } | SketchEntity::Arc { radius, .. } => {
                    radius_var_offset.insert(eid, vars.len());
                    vars.push(*radius);
                }
                _ => {}
            }
        }

        let n_vars = vars.len();
        let mut iter = 0;

        for step in 0..max_iterations {
            iter = step + 1;
            let residuals = self.evaluate_residuals(&vars, &point_var_offset, &radius_var_offset);
            if residuals.is_empty() {
                break;
            }
            let step_max_res = residuals.iter().fold(0.0_f64, |acc, &r| acc.max(r.abs()));
            if step_max_res <= tolerance {
                break;
            }

            let m = residuals.len();
            // Numerical Jacobian J (m x n_vars)
            let eps = 1e-7;
            let mut jac = vec![vec![0.0; n_vars]; m];
            for j in 0..n_vars {
                let orig = vars[j];
                vars[j] = orig + eps;
                let r_plus = self.evaluate_residuals(&vars, &point_var_offset, &radius_var_offset);
                vars[j] = orig;
                for i in 0..m {
                    jac[i][j] = (r_plus[i] - residuals[i]) / eps;
                }
            }

            // Form normal equations: (J^T J + lambda I) delta = -J^T r
            let lambda = 1e-4;
            let mut a = vec![vec![0.0; n_vars]; n_vars];
            let mut b = vec![0.0; n_vars];
            for i in 0..n_vars {
                a[i][i] = lambda;
                for k in 0..m {
                    b[i] -= jac[k][i] * residuals[k];
                    for j in 0..n_vars {
                        a[i][j] += jac[k][i] * jac[k][j];
                    }
                }
            }

            if let Some(delta) = solve_linear_system(a, b) {
                for j in 0..n_vars {
                    vars[j] += delta[j];
                }
            } else {
                break;
            }
        }

        // Write solved variables back to points & entities
        for pid in &point_ids {
            let idx = point_var_offset[pid];
            if let Some(pt) = self.points.get_mut(pid) {
                pt.x = vars[idx];
                pt.y = vars[idx + 1];
            }
        }
        for (&eid, &idx) in &radius_var_offset {
            if let Some(SketchEntity::Circle { radius, .. } | SketchEntity::Arc { radius, .. }) =
                self.entities.get_mut(&eid)
            {
                *radius = vars[idx].abs().max(1e-3);
            }
        }

        let final_residuals = self.evaluate_residuals(&vars, &point_var_offset, &radius_var_offset);
        let max_res = final_residuals
            .iter()
            .fold(0.0_f64, |acc, &r| acc.max(r.abs()));
        let n_eqs = final_residuals.len();

        let status = if max_res > 1e-3 {
            if n_eqs > n_vars {
                SketchStatus::OverConstrained {
                    conflicting_equations: n_eqs - n_vars,
                }
            } else {
                SketchStatus::SolverFailed {
                    residual_norm: format!("{max_res:.4e}"),
                }
            }
        } else if n_eqs > n_vars {
            SketchStatus::OverConstrained {
                conflicting_equations: n_eqs - n_vars,
            }
        } else if n_vars > n_eqs {
            SketchStatus::UnderConstrained {
                remaining_dof: n_vars - n_eqs,
            }
        } else {
            SketchStatus::FullyConstrained
        };

        SketchSolveReport {
            status,
            iterations: iter,
            max_residual: max_res,
            total_variables: n_vars,
            total_scalar_equations: n_eqs,
            remaining_dof: n_vars.saturating_sub(n_eqs),
        }
    }

    fn evaluate_residuals(
        &self,
        vars: &[f64],
        pt_map: &BTreeMap<PointId, usize>,
        rad_map: &BTreeMap<EntityId, usize>,
    ) -> Vec<f64> {
        let get_pt = |pid: PointId| -> (f64, f64) {
            let idx = pt_map[&pid];
            (vars[idx], vars[idx + 1])
        };

        let mut res = Vec::new();
        for c in &self.constraints {
            match *c {
                SketchConstraint::Fixed { point, x, y } => {
                    let (px, py) = get_pt(point);
                    res.push(px - x);
                    res.push(py - y);
                }
                SketchConstraint::Coincident { p1, p2 } => {
                    let (x1, y1) = get_pt(p1);
                    let (x2, y2) = get_pt(p2);
                    res.push(x1 - x2);
                    res.push(y1 - y2);
                }
                SketchConstraint::Horizontal { p1, p2 } => {
                    let (_, y1) = get_pt(p1);
                    let (_, y2) = get_pt(p2);
                    res.push(y1 - y2);
                }
                SketchConstraint::Vertical { p1, p2 } => {
                    let (x1, _) = get_pt(p1);
                    let (x2, _) = get_pt(p2);
                    res.push(x1 - x2);
                }
                SketchConstraint::Parallel { a1, a2, b1, b2 } => {
                    let (ax1, ay1) = get_pt(a1);
                    let (ax2, ay2) = get_pt(a2);
                    let (bx1, by1) = get_pt(b1);
                    let (bx2, by2) = get_pt(b2);
                    let dax = ax2 - ax1;
                    let day = ay2 - ay1;
                    let dbx = bx2 - bx1;
                    let dby = by2 - by1;
                    res.push(dax * dby - day * dbx);
                }
                SketchConstraint::Perpendicular { a1, a2, b1, b2 } => {
                    let (ax1, ay1) = get_pt(a1);
                    let (ax2, ay2) = get_pt(a2);
                    let (bx1, by1) = get_pt(b1);
                    let (bx2, by2) = get_pt(b2);
                    let dax = ax2 - ax1;
                    let day = ay2 - ay1;
                    let dbx = bx2 - bx1;
                    let dby = by2 - by1;
                    res.push(dax * dbx + day * dby);
                }
                SketchConstraint::Tangent {
                    l1,
                    l2,
                    center,
                    circle,
                } => {
                    let (x1, y1) = get_pt(l1);
                    let (x2, y2) = get_pt(l2);
                    let (cx, cy) = get_pt(center);
                    let r = vars[rad_map[&circle]];
                    let dx = x2 - x1;
                    let dy = y2 - y1;
                    let len = (dx * dx + dy * dy).sqrt().max(1e-9);
                    let dist = ((dy * cx - dx * cy + x2 * y1 - y2 * x1).abs()) / len;
                    res.push(dist - r);
                }
                SketchConstraint::Distance { p1, p2, distance } => {
                    let (x1, y1) = get_pt(p1);
                    let (x2, y2) = get_pt(p2);
                    let d = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
                    res.push(d - distance);
                }
                SketchConstraint::Radius { entity, radius } => {
                    if let Some(&idx) = rad_map.get(&entity) {
                        res.push(vars[idx] - radius);
                    }
                }
                SketchConstraint::Equal { a1, a2, b1, b2 } => {
                    let (ax1, ay1) = get_pt(a1);
                    let (ax2, ay2) = get_pt(a2);
                    let (bx1, by1) = get_pt(b1);
                    let (bx2, by2) = get_pt(b2);
                    let la = ((ax2 - ax1).powi(2) + (ay2 - ay1).powi(2)).sqrt();
                    let lb = ((bx2 - bx1).powi(2) + (by2 - by1).powi(2)).sqrt();
                    res.push(la - lb);
                }
            }
        }
        // Implicit: an arc's endpoints lie on its circle.
        for ent in self.entities.values() {
            if let SketchEntity::Arc {
                id,
                center,
                start,
                end,
                ..
            } = ent
                && let Some(&ri) = rad_map.get(id)
            {
                let (cx, cy) = get_pt(*center);
                for p in [*start, *end] {
                    let (x, y) = get_pt(p);
                    res.push((x - cx).hypot(y - cy) - vars[ri]);
                }
            }
        }
        res
    }

    /// Removes the constraint at `index` (no-op if out of range).
    pub fn remove_constraint(&mut self, index: usize) {
        if index < self.constraints.len() {
            self.constraints.remove(index);
        }
    }

    /// Removes an entity, then any points no longer used by an entity together with the
    /// constraints that reference them.
    pub fn remove_entity(&mut self, id: EntityId) {
        if self.entities.remove(&id).is_none() {
            return;
        }
        let used: std::collections::BTreeSet<PointId> =
            self.entities.values().flat_map(entity_points).collect();
        let orphans: Vec<PointId> = self
            .points
            .keys()
            .copied()
            .filter(|p| !used.contains(p))
            .collect();
        for p in &orphans {
            self.points.remove(p);
        }
        self.constraints.retain(|c| {
            constraint_points(c).iter().all(|p| !orphans.contains(p))
                && constraint_entity(c).is_none_or(|e| e != id)
        });
    }

    /// Straight segments of the sketch as point pairs (lines, rectangle sides, polyline edges).
    pub fn segments(&self) -> Vec<(PointId, PointId)> {
        let mut out = Vec::new();
        for ent in self.entities.values() {
            match ent {
                SketchEntity::Line { start, end, .. } => out.push((*start, *end)),
                SketchEntity::Rectangle { corners, .. } => {
                    for i in 0..4 {
                        out.push((corners[i], corners[(i + 1) % 4]));
                    }
                }
                SketchEntity::Polyline { points, closed, .. } => {
                    for w in points.windows(2) {
                        out.push((w[0], w[1]));
                    }
                    if *closed && points.len() > 2 {
                        out.push((points[points.len() - 1], points[0]));
                    }
                }
                _ => {}
            }
        }
        out
    }

    /// All closed loops of the sketch as polygons (circles and arcs discretised): each
    /// rectangle, closed polyline and circle, plus loops chained from lines, arcs and
    /// open polylines whose endpoints meet (shared point or same position).
    pub fn extract_closed_profiles(&self) -> Vec<Vec<[f64; 2]>> {
        const SEGMENTS: usize = 48;
        const TOL: f64 = 1e-4;
        let pos = |p: &PointId| self.points.get(p).map(|q| [q.x, q.y]);
        let mut loops = Vec::new();
        let mut open: Vec<Vec<[f64; 2]>> = Vec::new();

        for ent in self.entities.values() {
            match ent {
                SketchEntity::Rectangle { corners, .. } => {
                    if let Some(pts) = corners.iter().map(pos).collect::<Option<Vec<_>>>() {
                        loops.push(pts);
                    }
                }
                SketchEntity::Polyline { points, closed, .. } => {
                    if let Some(pts) = points.iter().map(pos).collect::<Option<Vec<_>>>() {
                        if *closed && pts.len() >= 3 {
                            loops.push(pts);
                        } else if pts.len() >= 2 {
                            open.push(pts);
                        }
                    }
                }
                SketchEntity::Circle { center, radius, .. } => {
                    if let Some([cx, cy]) = pos(center) {
                        loops.push(
                            (0..SEGMENTS)
                                .map(|i| {
                                    let t = i as f64 * std::f64::consts::TAU / SEGMENTS as f64;
                                    [cx + radius * t.cos(), cy + radius * t.sin()]
                                })
                                .collect(),
                        );
                    }
                }
                SketchEntity::Line { start, end, .. } => {
                    if let (Some(a), Some(b)) = (pos(start), pos(end)) {
                        open.push(vec![a, b]);
                    }
                }
                SketchEntity::Arc {
                    center,
                    start,
                    end,
                    radius,
                    ..
                } => {
                    if let (Some([cx, cy]), Some(a), Some(b)) = (pos(center), pos(start), pos(end)) {
                        let a0 = (a[1] - cy).atan2(a[0] - cx);
                        let mut sweep = (b[1] - cy).atan2(b[0] - cx) - a0;
                        if sweep <= 0.0 {
                            sweep += std::f64::consts::TAU;
                        }
                        let n = ((sweep / std::f64::consts::TAU) * SEGMENTS as f64).ceil().max(2.0) as usize;
                        let mut pts = vec![a];
                        for i in 1..n {
                            let t = a0 + sweep * i as f64 / n as f64;
                            pts.push([cx + radius * t.cos(), cy + radius * t.sin()]);
                        }
                        pts.push(b);
                        open.push(pts);
                    }
                }
            }
        }

        let close = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) < TOL;
        while let Some(mut chain) = open.pop() {
            loop {
                let (first, last) = (chain[0], chain[chain.len() - 1]);
                if chain.len() >= 4 && close(first, last) {
                    chain.pop();
                    loops.push(chain);
                    break;
                }
                let Some(i) = open
                    .iter()
                    .position(|e| close(e[0], last) || close(e[e.len() - 1], last))
                else {
                    break;
                };
                let mut next = open.swap_remove(i);
                if !close(next[0], last) {
                    next.reverse();
                }
                chain.extend_from_slice(&next[1..]);
            }
        }
        loops
    }

    /// Extracts a closed 2D polygon boundary `Vec<[f64; 2]>` from the sketch for `Extrude`.
    pub fn extract_closed_profile(&self) -> Option<Vec<[f64; 2]>> {
        // 1. Check Rectangle or closed Polyline first
        for ent in self.entities.values() {
            match ent {
                SketchEntity::Rectangle { corners, .. } => {
                    let mut pts = Vec::with_capacity(4);
                    for pid in corners {
                        let p = self.points.get(pid)?;
                        pts.push([p.x, p.y]);
                    }
                    return Some(pts);
                }
                SketchEntity::Polyline {
                    points,
                    closed: true,
                    ..
                } if points.len() >= 3 => {
                    let mut pts = Vec::with_capacity(points.len());
                    for pid in points {
                        let p = self.points.get(pid)?;
                        pts.push([p.x, p.y]);
                    }
                    return Some(pts);
                }
                SketchEntity::Circle { center, radius, .. } => {
                    let c = self.points.get(center)?;
                    let segments = 32;
                    let mut pts = Vec::with_capacity(segments);
                    for i in 0..segments {
                        let theta = (i as f64) * 2.0 * std::f64::consts::PI / (segments as f64);
                        pts.push([c.x + radius * theta.cos(), c.y + radius * theta.sin()]);
                    }
                    return Some(pts);
                }
                _ => {}
            }
        }

        // 2. Otherwise chain connected Line entities into a closed loop
        let mut lines: Vec<(PointId, PointId)> = self
            .entities
            .values()
            .filter_map(|e| match e {
                SketchEntity::Line { start, end, .. } => Some((*start, *end)),
                _ => None,
            })
            .collect();

        if lines.len() < 3 {
            return None;
        }

        let (first_start, mut curr_end) = lines.remove(0);
        let mut ordered_pids = vec![first_start, curr_end];

        while !lines.is_empty() {
            let curr_pt = self.points.get(&curr_end)?;
            let next_idx = lines.iter().position(|&(s, e)| {
                let ps = &self.points[&s];
                let pe = &self.points[&e];
                ((ps.x - curr_pt.x).hypot(ps.y - curr_pt.y) < 1e-3)
                    || ((pe.x - curr_pt.x).hypot(pe.y - curr_pt.y) < 1e-3)
            })?;

            let (s, e) = lines.remove(next_idx);
            let ps = &self.points[&s];
            let next_pid = if (ps.x - curr_pt.x).hypot(ps.y - curr_pt.y) < 1e-3 {
                e
            } else {
                s
            };
            curr_end = next_pid;
            ordered_pids.push(curr_end);
        }

        // Verify loop closure
        let p_first = self.points.get(&first_start)?;
        let p_last = self.points.get(&curr_end)?;
        if (p_first.x - p_last.x).hypot(p_first.y - p_last.y) < 1e-3 {
            ordered_pids.pop();
        }

        if ordered_pids.len() >= 3 {
            Some(
                ordered_pids
                    .into_iter()
                    .map(|pid| {
                        let p = &self.points[&pid];
                        [p.x, p.y]
                    })
                    .collect(),
            )
        } else {
            None
        }
    }
}

/// Points an entity is defined by.
pub fn entity_points(ent: &SketchEntity) -> Vec<PointId> {
    match ent {
        SketchEntity::Line { start, end, .. } => vec![*start, *end],
        SketchEntity::Circle { center, .. } => vec![*center],
        SketchEntity::Arc {
            center, start, end, ..
        } => vec![*center, *start, *end],
        SketchEntity::Rectangle { corners, .. } => corners.to_vec(),
        SketchEntity::Polyline { points, .. } => points.clone(),
    }
}

/// Points a constraint refers to.
pub fn constraint_points(c: &SketchConstraint) -> Vec<PointId> {
    match *c {
        SketchConstraint::Fixed { point, .. } => vec![point],
        SketchConstraint::Coincident { p1, p2 }
        | SketchConstraint::Horizontal { p1, p2 }
        | SketchConstraint::Vertical { p1, p2 }
        | SketchConstraint::Distance { p1, p2, .. } => vec![p1, p2],
        SketchConstraint::Parallel { a1, a2, b1, b2 }
        | SketchConstraint::Perpendicular { a1, a2, b1, b2 }
        | SketchConstraint::Equal { a1, a2, b1, b2 } => vec![a1, a2, b1, b2],
        SketchConstraint::Tangent { l1, l2, center, .. } => vec![l1, l2, center],
        SketchConstraint::Radius { .. } => Vec::new(),
    }
}

/// Entity a constraint refers to directly (radius / tangent circle), if any.
pub fn constraint_entity(c: &SketchConstraint) -> Option<EntityId> {
    match *c {
        SketchConstraint::Radius { entity, .. } => Some(entity),
        SketchConstraint::Tangent { circle, .. } => Some(circle),
        _ => None,
    }
}

fn solve_linear_system(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for i in 0..n {
        let mut max_row = i;
        let mut max_val = a[i][i].abs();
        for (r, row) in a.iter().enumerate().take(n).skip(i + 1) {
            if row[i].abs() > max_val {
                max_val = row[i].abs();
                max_row = r;
            }
        }
        if max_val < 1e-13 {
            return None;
        }
        a.swap(i, max_row);
        b.swap(i, max_row);

        let pivot = a[i][i];
        let pivot_row = a[i].clone();
        for r in (i + 1)..n {
            let factor = a[r][i] / pivot;
            b[r] -= factor * b[i];
            for (cell, &piv_val) in a[r].iter_mut().zip(pivot_row.iter()).take(n).skip(i) {
                *cell -= factor * piv_val;
            }
        }
    }

    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i][j] * x[j];
        }
        x[i] = sum / a[i][i];
    }
    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase5a_sketch_constraints_and_dof() {
        let mut sketch = Sketch::new("ConstrainedBracketProfile");
        // Create a rectangle with rough initial coordinates
        let (_rect_id, [p0, p1, p2, p3]) = sketch.add_rectangle(2.0, -1.0, 45.0, 28.0);

        // Fix bottom-left corner at (0, 0) -> 2 equations
        sketch.add_constraint(SketchConstraint::Fixed {
            point: p0,
            x: 0.0,
            y: 0.0,
        });
        // Constrain width = 80.0 mm and height = 50.0 mm -> 2 equations
        // Total: 4 points = 8 variables; 4 H/V + 2 Fixed + 2 Distance = 8 equations -> FullyConstrained (0 DOF)!
        sketch.add_constraint(SketchConstraint::Distance {
            p1: p0,
            p2: p1,
            distance: 80.0,
        });
        sketch.add_constraint(SketchConstraint::Distance {
            p1: p0,
            p2: p3,
            distance: 50.0,
        });

        let report = sketch.solve(50, 1e-9);
        assert_eq!(report.status, SketchStatus::FullyConstrained);
        assert_eq!(report.remaining_dof, 0);

        let profile = sketch.extract_closed_profile().expect("closed profile");
        assert_eq!(profile.len(), 4);
        assert!((profile[0][0] - 0.0).abs() < 1e-5);
        assert!((profile[1][0] - 80.0).abs() < 1e-5);
        assert!((profile[2][0] - 80.0).abs() < 1e-5);
        assert!((profile[2][1] - 50.0).abs() < 1e-5);

        // Add conflicting Distance constraint -> OverConstrained detected!
        sketch.add_constraint(SketchConstraint::Distance {
            p1: p3,
            p2,
            distance: 100.0,
        });
        let over_report = sketch.solve(50, 1e-9);
        assert_eq!(
            over_report.status,
            SketchStatus::OverConstrained {
                conflicting_equations: 1
            }
        );
    }
}
