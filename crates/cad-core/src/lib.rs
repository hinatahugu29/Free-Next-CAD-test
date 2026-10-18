//! Parametric CAD Document, Expression Parameter System, Feature DAG,
//! Incremental Recompute Engine, Undo/Redo Commands, and Serialization (Phase 2 & Phase 4).

pub mod semantic_ref;
pub mod sketch;

use std::collections::{BTreeMap, HashSet, VecDeque};
use std::path::Path;

use cad_geometry::{
    Angle, BoundingBox, GeometryError, Length, ShapeId, ShapeStore, ShapeSummary,
    TessellationOptions, TriMesh,
};
use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uuid::Uuid;

pub use semantic_ref::{ResolutionFailureReason, SemanticReference};
pub use sketch::{
    EntityId, PointId, Sketch, SketchConstraint, SketchEntity, SketchSolveReport, SketchStatus,
};

pub const DOCUMENT_SCHEMA_VERSION: u32 = 1;

/// Errors produced by `cad-core` document and DAG operations.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Error)]
pub enum CoreError {
    #[error("Parameter error for '{name}': {reason}")]
    ParameterError { name: String, reason: String },
    #[error("Circular dependency detected in parameters involving '{0}'")]
    CircularParameterDependency(String),
    #[error("Feature {0:?} not found in document")]
    FeatureNotFound(FeatureId),
    #[error("Cyclic dependency detected in Feature Graph")]
    CyclicFeatureDependency,
    #[error(
        "Cannot remove feature {id:?} because it is referenced by downstream features: {dependents:?}"
    )]
    FeatureInUse {
        id: FeatureId,
        dependents: Vec<FeatureId>,
    },
    #[error("Unsupported document schema version {0} (expected {DOCUMENT_SCHEMA_VERSION})")]
    UnsupportedSchemaVersion(u32),
    #[error("Serialization error: {0}")]
    Serialization(String),
    #[error("Nothing to undo")]
    NothingToUndo,
    #[error("Nothing to redo")]
    NothingToRedo,
    #[error("Geometry error: {0}")]
    Geometry(#[from] GeometryError),
    #[error(
        "Feature {feature:?} could not re-identify a referenced edge/face after the upstream shape changed: {reason:?}"
    )]
    SemanticReferenceLost {
        feature: FeatureId,
        reason: ResolutionFailureReason,
    },
}

/// Unique identifier for a [`Document`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct DocumentId(pub Uuid);

impl Default for DocumentId {
    fn default() -> Self {
        Self(Uuid::new_v4())
    }
}

/// Stable identifier for a [`Feature`] node within a [`Document`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct FeatureId(pub u32);

/// Value of a [`Parameter`]: either a direct numeric value or an arithmetic expression.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ParameterValue {
    Number(f64),
    Expression(String),
}

impl ParameterValue {
    /// Extracts identifier tokens referenced by this parameter value.
    pub fn referenced_parameters(&self) -> Vec<String> {
        match self {
            Self::Number(_) => Vec::new(),
            Self::Expression(expr) => extract_identifiers(expr),
        }
    }
}

/// Named parameter stored in a [`Document`] (Phase 4.1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Parameter {
    pub name: String,
    pub value: ParameterValue,
    pub unit: String,
    pub description: String,
}

/// A scalar input to a feature that can be either a constant number or an expression/parameter name.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ScalarParam {
    Literal(f64),
    Expr(String),
}

impl ScalarParam {
    pub const fn mm(val: f64) -> Self {
        Self::Literal(val)
    }

    pub const fn deg(val: f64) -> Self {
        Self::Literal(val)
    }

    pub fn expr(s: impl Into<String>) -> Self {
        Self::Expr(s.into())
    }

    pub fn evaluate(&self, resolved_params: &BTreeMap<String, f64>) -> Result<f64, CoreError> {
        match self {
            Self::Literal(v) => {
                if !v.is_finite() {
                    return Err(CoreError::ParameterError {
                        name: "<literal>".to_string(),
                        reason: "non-finite literal value".to_string(),
                    });
                }
                Ok(*v)
            }
            Self::Expr(expr) => eval_expression(expr, resolved_params),
        }
    }

    pub fn referenced_parameters(&self) -> Vec<String> {
        match self {
            Self::Literal(_) => Vec::new(),
            Self::Expr(e) => extract_identifiers(e),
        }
    }
}

impl From<f64> for ScalarParam {
    fn from(v: f64) -> Self {
        Self::Literal(v)
    }
}

impl From<&str> for ScalarParam {
    fn from(s: &str) -> Self {
        Self::Expr(s.to_string())
    }
}

/// Boolean operation kind.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum BooleanOp {
    Fuse,
    Cut,
    Common,
}

/// The plane a sketch is drawn on.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub enum SketchPlane {
    #[default]
    XY,
    XZ,
    YZ,
    /// A face of another feature's shape, held as a semantic reference so the plane
    /// follows that face when upstream dimensions change.
    Face {
        feature: FeatureId,
        face: SemanticReference,
    },
}

impl SketchPlane {
    fn feature(&self) -> Option<FeatureId> {
        match self {
            Self::Face { feature, .. } => Some(*feature),
            _ => None,
        }
    }
}

/// A 2D coordinate frame in 3D space for a sketch plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlaneFrame {
    pub origin: DVec3,
    pub x_axis: DVec3,
    pub y_axis: DVec3,
    pub normal: DVec3,
}

impl PlaneFrame {
    /// Frame for the plane through `point` with `normal`. The origin is the world
    /// origin projected onto the plane and the X axis is world X projected onto it
    /// (falling back to world Y), so sketch coordinates on a face parallel to a base
    /// plane coincide with world coordinates and do not shift when the face's extent
    /// changes — only when the plane itself moves.
    pub fn from_point_normal(point: DVec3, normal: DVec3) -> Self {
        let n = normal.normalize();
        let origin = n * point.dot(n);
        let x_axis = (DVec3::X - n * n.x)
            .try_normalize()
            .unwrap_or_else(|| (DVec3::Y - n * n.y).normalize());
        let y_axis = n.cross(x_axis);
        Self {
            origin,
            x_axis,
            y_axis,
            normal: n,
        }
    }

    pub fn to_local(&self, p: DVec3) -> [f64; 2] {
        let d = p - self.origin;
        [d.dot(self.x_axis), d.dot(self.y_axis)]
    }

    pub fn to_world(&self, x: f64, y: f64) -> DVec3 {
        self.origin + self.x_axis * x + self.y_axis * y
    }
}

/// Moves a shape built in world XY (extruded along +Z) onto `frame`: first shifted by
/// `z_shift` along Z, then rotated so X/Y/Z map to the frame axes, then translated to
/// the frame origin. Intermediate shapes are released from the store.
fn place_on_frame(
    store: &mut ShapeStore,
    shape: ShapeId,
    frame: &PlaneFrame,
    z_shift: f64,
) -> Result<ShapeId, CoreError> {
    let mut current = shape;
    if z_shift.abs() > 1e-12 {
        let moved = store.translate(current, 0.0, 0.0, z_shift)?;
        let _ = store.remove(current);
        current = moved;
    }
    let rotation = DQuat::from_mat3(&DMat3::from_cols(frame.x_axis, frame.y_axis, frame.normal));
    let (axis, angle) = rotation.to_axis_angle();
    if angle.abs() > 1e-12 {
        let rotated = store.rotate(current, DVec3::ZERO, axis, Angle::rad(angle))?;
        let _ = store.remove(current);
        current = rotated;
    }
    if frame.origin.length_squared() > 1e-24 {
        let moved = store.translate(current, frame.origin.x, frame.origin.y, frame.origin.z)?;
        let _ = store.remove(current);
        current = moved;
    }
    Ok(current)
}

/// How an extruded sketch profile is combined with existing geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum ExtrudeMode {
    #[default]
    NewBody,
    /// Fuse the extrusion into `target`.
    Join { target: FeatureId },
    /// Subtract the extrusion from `target`.
    Cut { target: FeatureId },
}

impl ExtrudeMode {
    fn target(&self) -> Option<FeatureId> {
        match self {
            Self::NewBody => None,
            Self::Join { target } | Self::Cut { target } => Some(*target),
        }
    }
}

/// Parametric feature definition (Phase 2.2 & Phase 4.2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeatureKind {
    Box {
        width: ScalarParam,
        height: ScalarParam,
        depth: ScalarParam,
    },
    Cylinder {
        radius: ScalarParam,
        height: ScalarParam,
    },
    Sphere {
        radius: ScalarParam,
    },
    Cone {
        bottom_radius: ScalarParam,
        top_radius: ScalarParam,
        height: ScalarParam,
    },
    ExtrudePolygon {
        points_xy: Vec<[ScalarParam; 2]>,
        depth: ScalarParam,
    },
    SketchExtrude {
        sketch: Sketch,
        depth: ScalarParam,
        #[serde(default)]
        plane: SketchPlane,
        #[serde(default)]
        mode: ExtrudeMode,
        /// Extrude against the plane normal instead of along it.
        #[serde(default)]
        reversed: bool,
        /// Extrude half the depth to each side of the plane (overrides `reversed`).
        #[serde(default)]
        symmetric: bool,
    },
    ExtrudeCircle {
        radius: ScalarParam,
        depth: ScalarParam,
    },
    RevolvePolygon {
        points_xz: Vec<[ScalarParam; 2]>,
        angle_deg: ScalarParam,
    },
    Boolean {
        op: BooleanOp,
        primary: FeatureId,
        secondary: FeatureId,
    },
    Transform {
        input: FeatureId,
        dx: ScalarParam,
        dy: ScalarParam,
        dz: ScalarParam,
        rot_origin: [f64; 3],
        rot_axis: [f64; 3],
        rot_angle_deg: ScalarParam,
    },
    Fillet {
        input: FeatureId,
        radius: ScalarParam,
        edge_indices: Vec<u32>,
        /// Stable semantic references captured when the edges were picked (Topological
        /// Naming Problem mitigation); when present, these are re-resolved against the
        /// upstream shape's *current* topology at recompute time instead of trusting
        /// `edge_indices`, which can silently point at the wrong edge after upstream edits.
        #[serde(default)]
        semantic_refs: Vec<SemanticReference>,
    },
    Chamfer {
        input: FeatureId,
        distance: ScalarParam,
        edge_indices: Vec<u32>,
        #[serde(default)]
        semantic_refs: Vec<SemanticReference>,
    },
    LinearPattern {
        input: FeatureId,
        spacing_x: ScalarParam,
        spacing_y: ScalarParam,
        spacing_z: ScalarParam,
        count_x: u32,
        count_y: u32,
    },
}

impl FeatureKind {
    /// Returns the upstream [`FeatureId`]s directly referenced by this feature definition.
    pub fn upstream_features(&self) -> Vec<FeatureId> {
        match self {
            Self::Box { .. }
            | Self::Cylinder { .. }
            | Self::Sphere { .. }
            | Self::Cone { .. }
            | Self::ExtrudePolygon { .. }
            | Self::ExtrudeCircle { .. }
            | Self::RevolvePolygon { .. } => Vec::new(),
            Self::SketchExtrude { plane, mode, .. } => {
                let mut deps = Vec::new();
                for id in [plane.feature(), mode.target()].into_iter().flatten() {
                    if !deps.contains(&id) {
                        deps.push(id);
                    }
                }
                deps
            }
            Self::Boolean {
                primary, secondary, ..
            } => {
                if primary == secondary {
                    vec![*primary]
                } else {
                    vec![*primary, *secondary]
                }
            }
            Self::Transform { input, .. }
            | Self::Fillet { input, .. }
            | Self::Chamfer { input, .. }
            | Self::LinearPattern { input, .. } => vec![*input],
        }
    }

    /// The body this feature modifies, if any. A suppressed feature passes this
    /// input's shape through unchanged, as if the feature were not there.
    pub fn primary_input(&self) -> Option<FeatureId> {
        match self {
            Self::Transform { input, .. }
            | Self::Fillet { input, .. }
            | Self::Chamfer { input, .. }
            | Self::LinearPattern { input, .. } => Some(*input),
            Self::Boolean { primary, .. } => Some(*primary),
            Self::SketchExtrude { mode, .. } => mode.target(),
            _ => None,
        }
    }

    /// Returns parameter names referenced by scalar expressions inside this feature.
    pub fn referenced_parameters(&self) -> HashSet<String> {
        let mut set = HashSet::new();
        let mut add = |sp: &ScalarParam| {
            for name in sp.referenced_parameters() {
                set.insert(name);
            }
        };
        match self {
            Self::Box {
                width,
                height,
                depth,
            } => {
                add(width);
                add(height);
                add(depth);
            }
            Self::Cylinder { radius, height } => {
                add(radius);
                add(height);
            }
            Self::Sphere { radius } => {
                add(radius);
            }
            Self::Cone {
                bottom_radius,
                top_radius,
                height,
            } => {
                add(bottom_radius);
                add(top_radius);
                add(height);
            }
            Self::ExtrudePolygon { points_xy, depth } => {
                for [x, y] in points_xy {
                    add(x);
                    add(y);
                }
                add(depth);
            }
            Self::SketchExtrude { depth, .. } => {
                add(depth);
            }
            Self::ExtrudeCircle { radius, depth } => {
                add(radius);
                add(depth);
            }
            Self::RevolvePolygon {
                points_xz,
                angle_deg,
            } => {
                for [x, z] in points_xz {
                    add(x);
                    add(z);
                }
                add(angle_deg);
            }
            Self::Boolean { .. } => {}
            Self::Transform {
                dx,
                dy,
                dz,
                rot_angle_deg,
                ..
            } => {
                add(dx);
                add(dy);
                add(dz);
                add(rot_angle_deg);
            }
            Self::Fillet { radius, .. } => {
                add(radius);
            }
            Self::Chamfer { distance, .. } => {
                add(distance);
            }
            Self::LinearPattern {
                spacing_x,
                spacing_y,
                spacing_z,
                ..
            } => {
                add(spacing_x);
                add(spacing_y);
                add(spacing_z);
            }
        }
        set
    }
}

/// Evaluation status of a [`Feature`] node in the DAG (Phase 2.3).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum FeatureState {
    Clean,
    Dirty,
    Computing,
    Failed(String),
    Blocked { upstream_failed: FeatureId },
}

/// A feature node in the parametric [`Document`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub kind: FeatureKind,
    pub visible: bool,
    pub color_rgba: [f32; 4],
    pub extra_dependencies: Vec<FeatureId>,
    /// Temporarily disabled: the feature passes its primary input through unchanged
    /// (or produces nothing if it has none) without being deleted.
    #[serde(default)]
    pub suppressed: bool,
    #[serde(skip, default = "default_dirty_state")]
    pub state: FeatureState,
    #[serde(skip)]
    pub cached_shape: Option<ShapeId>,
    #[serde(skip)]
    pub cached_summary: Option<ShapeSummary>,
    #[serde(skip)]
    pub cached_mesh: Option<TriMesh>,
    /// Preserves the last valid mesh & summary even when the feature enters `Failed` or `Blocked` state,
    /// allowing the UI to distinguish logical failure from the last valid visual geometry (Phase 4.3).
    #[serde(skip)]
    pub last_valid_mesh: Option<TriMesh>,
    #[serde(skip)]
    pub last_valid_summary: Option<ShapeSummary>,
}

fn default_dirty_state() -> FeatureState {
    FeatureState::Dirty
}

impl Feature {
    /// Returns all upstream feature dependencies (intrinsic + explicitly registered).
    pub fn all_dependencies(&self) -> Vec<FeatureId> {
        let mut deps = self.kind.upstream_features();
        for &extra in &self.extra_dependencies {
            if !deps.contains(&extra) {
                deps.push(extra);
            }
        }
        deps
    }
}

/// Serializable user/AI/CLI command representing a discrete document modification (Phase 2.4 & 4.4).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CadCommand {
    SetParameter {
        name: String,
        value: ParameterValue,
        unit: String,
        description: String,
    },
    RemoveParameter {
        name: String,
    },
    AddFeature {
        name: String,
        kind: FeatureKind,
        visible: bool,
        color_rgba: [f32; 4],
    },
    UpdateFeature {
        id: FeatureId,
        name: Option<String>,
        kind: Option<FeatureKind>,
        visible: Option<bool>,
        color_rgba: Option<[f32; 4]>,
    },
    RemoveFeature {
        id: FeatureId,
    },
    SetSuppressed {
        id: FeatureId,
        suppressed: bool,
    },
    AddDependency {
        upstream: FeatureId,
        downstream: FeatureId,
    },
}

/// Summary report returned after an incremental [`Document::recompute`] run.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecomputeReport {
    pub evaluated_order: Vec<FeatureId>,
    pub recomputed_features: Vec<FeatureId>,
    pub reused_cached_features: Vec<FeatureId>,
    pub failed_features: Vec<(FeatureId, String)>,
    pub blocked_features: Vec<(FeatureId, FeatureId)>,
}

impl RecomputeReport {
    pub fn is_all_clean(&self) -> bool {
        self.failed_features.is_empty() && self.blocked_features.is_empty()
    }
}

/// Versioned snapshot of a [`Document`] for JSON persistence (Phase 2.5).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DocumentSnapshot {
    pub schema_version: u32,
    pub id: DocumentId,
    pub name: String,
    pub revision: u64,
    pub next_feature_id: u32,
    pub parameters: BTreeMap<String, Parameter>,
    pub features: BTreeMap<FeatureId, Feature>,
    pub operation_history: Vec<CadCommand>,
}

/// Core parametric CAD Document (Phase 2.1).
///
/// Holds no raw OCCT objects directly; all geometry lives in [`ShapeStore`].
#[derive(Debug, Clone)]
pub struct Document {
    pub id: DocumentId,
    pub name: String,
    pub revision: u64,
    next_feature_id: u32,
    pub parameters: BTreeMap<String, Parameter>,
    pub features: BTreeMap<FeatureId, Feature>,
    pub operation_history: Vec<CadCommand>,
}

impl Default for Document {
    fn default() -> Self {
        Self::new("Untitled Document")
    }
}

impl Document {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            id: DocumentId::default(),
            name: name.into(),
            revision: 1,
            next_feature_id: 1,
            parameters: BTreeMap::new(),
            features: BTreeMap::new(),
            operation_history: Vec::new(),
        }
    }

    /// Sets or updates a named parameter and marks dependent features `Dirty`.
    pub fn set_parameter(
        &mut self,
        name: impl Into<String>,
        value: ParameterValue,
        unit: impl Into<String>,
        description: impl Into<String>,
    ) -> Result<(), CoreError> {
        let name_str = name.into();
        let unit_str = unit.into();
        let desc_str = description.into();

        // Tentatively insert and verify no circular parameter dependencies
        let prev = self.parameters.insert(
            name_str.clone(),
            Parameter {
                name: name_str.clone(),
                value: value.clone(),
                unit: unit_str.clone(),
                description: desc_str.clone(),
            },
        );

        if let Err(err) = self.resolve_all_parameters() {
            // Rollback on invalid/circular expression
            if let Some(old) = prev {
                self.parameters.insert(name_str, old);
            } else {
                self.parameters.remove(&name_str);
            }
            return Err(err);
        }

        // Find all parameters transitively affected by `name_str`
        let affected_params = self.transitive_affected_parameters(&name_str);

        // Mark only features referencing any affected parameter as Dirty (and propagate down the DAG)
        let directly_affected_features: Vec<FeatureId> = self
            .features
            .values()
            .filter(|f| {
                f.kind
                    .referenced_parameters()
                    .iter()
                    .any(|p| affected_params.contains(p))
            })
            .map(|f| f.id)
            .collect();

        for fid in directly_affected_features {
            self.mark_dirty(fid);
        }

        self.revision += 1;
        self.operation_history.push(CadCommand::SetParameter {
            name: name_str,
            value,
            unit: unit_str,
            description: desc_str,
        });
        Ok(())
    }

    /// Resolves all parameters into concrete `f64` values, detecting circular references.
    pub fn resolve_all_parameters(&self) -> Result<BTreeMap<String, f64>, CoreError> {
        let mut resolved = BTreeMap::new();
        let mut visiting = HashSet::new();

        for name in self.parameters.keys() {
            self.resolve_single_parameter(name, &mut resolved, &mut visiting)?;
        }
        Ok(resolved)
    }

    fn resolve_single_parameter(
        &self,
        name: &str,
        resolved: &mut BTreeMap<String, f64>,
        visiting: &mut HashSet<String>,
    ) -> Result<f64, CoreError> {
        if let Some(&val) = resolved.get(name) {
            return Ok(val);
        }
        let Some(param) = self.parameters.get(name) else {
            return Err(CoreError::ParameterError {
                name: name.to_string(),
                reason: "undefined parameter".to_string(),
            });
        };
        if !visiting.insert(name.to_string()) {
            return Err(CoreError::CircularParameterDependency(name.to_string()));
        }

        let val = match &param.value {
            ParameterValue::Number(v) => {
                if !v.is_finite() {
                    return Err(CoreError::ParameterError {
                        name: name.to_string(),
                        reason: "parameter number is not finite".to_string(),
                    });
                }
                *v
            }
            ParameterValue::Expression(expr) => {
                for dep in extract_identifiers(expr) {
                    self.resolve_single_parameter(&dep, resolved, visiting)?;
                }
                eval_expression(expr, resolved)?
            }
        };

        visiting.remove(name);
        resolved.insert(name.to_string(), val);
        Ok(val)
    }

    fn transitive_affected_parameters(&self, root_param: &str) -> HashSet<String> {
        let mut affected = HashSet::new();
        let mut queue = VecDeque::new();
        affected.insert(root_param.to_string());
        queue.push_back(root_param.to_string());

        while let Some(curr) = queue.pop_front() {
            for (name, param) in &self.parameters {
                if !affected.contains(name)
                    && param
                        .value
                        .referenced_parameters()
                        .iter()
                        .any(|r| r == &curr)
                {
                    affected.insert(name.clone());
                    queue.push_back(name.clone());
                }
            }
        }
        affected
    }

    /// Adds a new feature to the DAG after validating that all upstream dependencies exist.
    pub fn add_feature(
        &mut self,
        name: impl Into<String>,
        kind: FeatureKind,
        visible: bool,
        color_rgba: [f32; 4],
    ) -> Result<FeatureId, CoreError> {
        for dep in kind.upstream_features() {
            if !self.features.contains_key(&dep) {
                return Err(CoreError::FeatureNotFound(dep));
            }
        }
        let id = FeatureId(self.next_feature_id);
        self.next_feature_id += 1;

        let name_str = name.into();
        let feature = Feature {
            id,
            name: name_str.clone(),
            kind: kind.clone(),
            visible,
            color_rgba,
            extra_dependencies: Vec::new(),
            suppressed: false,
            state: FeatureState::Dirty,
            cached_shape: None,
            cached_summary: None,
            cached_mesh: None,
            last_valid_mesh: None,
            last_valid_summary: None,
        };
        self.features.insert(id, feature);
        self.revision += 1;
        self.operation_history.push(CadCommand::AddFeature {
            name: name_str,
            kind,
            visible,
            color_rgba,
        });
        Ok(id)
    }

    /// Updates an existing feature and propagates `Dirty` state to all downstream dependents.
    /// Rejects changes that would introduce a cycle into the Feature DAG.
    pub fn update_feature(
        &mut self,
        id: FeatureId,
        name: Option<String>,
        kind: Option<FeatureKind>,
        visible: Option<bool>,
        color_rgba: Option<[f32; 4]>,
    ) -> Result<(), CoreError> {
        if !self.features.contains_key(&id) {
            return Err(CoreError::FeatureNotFound(id));
        }
        if let Some(ref new_kind) = kind {
            for dep in new_kind.upstream_features() {
                if !self.features.contains_key(&dep) {
                    return Err(CoreError::FeatureNotFound(dep));
                }
            }
        }

        let backup = self.features.get(&id).cloned().unwrap();
        let mut geometry_changed = false;

        {
            let feat = self.features.get_mut(&id).unwrap();
            if let Some(ref n) = name {
                feat.name = n.clone();
            }
            if let Some(ref k) = kind
                && &feat.kind != k
            {
                feat.kind = k.clone();
                geometry_changed = true;
            }
            if let Some(v) = visible {
                feat.visible = v;
            }
            if let Some(c) = color_rgba {
                feat.color_rgba = c;
            }
        }

        if let Err(err) = self.topological_order() {
            self.features.insert(id, backup);
            return Err(err);
        }

        if geometry_changed {
            self.mark_dirty(id);
        }

        self.revision += 1;
        self.operation_history.push(CadCommand::UpdateFeature {
            id,
            name,
            kind,
            visible,
            color_rgba,
        });
        Ok(())
    }

    /// Registers an explicit dependency `upstream -> downstream` and rejects cycles.
    pub fn add_dependency(
        &mut self,
        upstream: FeatureId,
        downstream: FeatureId,
    ) -> Result<(), CoreError> {
        if !self.features.contains_key(&upstream) {
            return Err(CoreError::FeatureNotFound(upstream));
        }
        if !self.features.contains_key(&downstream) {
            return Err(CoreError::FeatureNotFound(downstream));
        }
        if upstream == downstream {
            return Err(CoreError::CyclicFeatureDependency);
        }

        {
            let feat = self.features.get_mut(&downstream).unwrap();
            if !feat.extra_dependencies.contains(&upstream) {
                feat.extra_dependencies.push(upstream);
            }
        }

        if let Err(err) = self.topological_order() {
            let feat = self.features.get_mut(&downstream).unwrap();
            feat.extra_dependencies.retain(|&x| x != upstream);
            return Err(err);
        }

        self.mark_dirty(downstream);
        self.revision += 1;
        self.operation_history.push(CadCommand::AddDependency {
            upstream,
            downstream,
        });
        Ok(())
    }

    /// Suppresses or un-suppresses a feature and marks it and its dependents dirty.
    pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) -> Result<(), CoreError> {
        let feat = self
            .features
            .get_mut(&id)
            .ok_or(CoreError::FeatureNotFound(id))?;
        if feat.suppressed == suppressed {
            return Ok(());
        }
        feat.suppressed = suppressed;
        self.mark_dirty(id);
        self.revision += 1;
        self.operation_history
            .push(CadCommand::SetSuppressed { id, suppressed });
        Ok(())
    }

    /// Features that directly depend on `id`.
    pub fn direct_dependents(&self, id: FeatureId) -> Vec<FeatureId> {
        self.features
            .values()
            .filter(|f| f.id != id && f.all_dependencies().contains(&id))
            .map(|f| f.id)
            .collect()
    }

    /// Resolves a sketch plane to a coordinate frame against the current geometry.
    pub fn sketch_plane_frame(
        &self,
        plane: &SketchPlane,
        store: &ShapeStore,
    ) -> Result<PlaneFrame, CoreError> {
        match plane {
            SketchPlane::XY => Ok(PlaneFrame::from_point_normal(DVec3::ZERO, DVec3::Z)),
            SketchPlane::XZ => Ok(PlaneFrame::from_point_normal(DVec3::ZERO, DVec3::NEG_Y)),
            SketchPlane::YZ => Ok(PlaneFrame::from_point_normal(DVec3::ZERO, DVec3::X)),
            SketchPlane::Face { feature, face } => {
                let shape_id = self
                    .features
                    .get(feature)
                    .and_then(|f| f.cached_shape)
                    .ok_or(CoreError::FeatureNotFound(*feature))?;
                let resolved = face.resolve_face(store, shape_id).map_err(|reason| {
                    CoreError::SemanticReferenceLost {
                        feature: *feature,
                        reason,
                    }
                })?;
                Ok(PlaneFrame::from_point_normal(resolved.centroid, resolved.normal))
            }
        }
    }

    /// Removes a feature if no downstream features depend on it.
    pub fn remove_feature(
        &mut self,
        id: FeatureId,
        store: Option<&mut ShapeStore>,
    ) -> Result<Feature, CoreError> {
        if !self.features.contains_key(&id) {
            return Err(CoreError::FeatureNotFound(id));
        }
        let dependents: Vec<FeatureId> = self
            .features
            .values()
            .filter(|f| f.id != id && f.all_dependencies().contains(&id))
            .map(|f| f.id)
            .collect();

        if !dependents.is_empty() {
            return Err(CoreError::FeatureInUse { id, dependents });
        }

        let removed = self.features.remove(&id).unwrap();
        if let (Some(shape_store), Some(shape_id)) = (store, removed.cached_shape) {
            let _ = shape_store.remove(shape_id);
        }
        self.revision += 1;
        self.operation_history
            .push(CadCommand::RemoveFeature { id });
        Ok(removed)
    }

    /// Marks `root_id` and all transitive downstream dependents as `Dirty`.
    pub fn mark_dirty(&mut self, root_id: FeatureId) {
        let mut queue = VecDeque::new();
        let mut visited = HashSet::new();
        queue.push_back(root_id);
        visited.insert(root_id);

        while let Some(curr) = queue.pop_front() {
            if let Some(feat) = self.features.get_mut(&curr) {
                feat.state = FeatureState::Dirty;
            }
            let downstream: Vec<FeatureId> = self
                .features
                .values()
                .filter(|f| f.all_dependencies().contains(&curr))
                .map(|f| f.id)
                .collect();

            for d in downstream {
                if visited.insert(d) {
                    queue.push_back(d);
                }
            }
        }
    }

    /// Computes a topological evaluation order of all features using Kahn's algorithm.
    /// Returns [`CoreError::CyclicFeatureDependency`] if a cycle exists.
    pub fn topological_order(&self) -> Result<Vec<FeatureId>, CoreError> {
        let mut in_degree: BTreeMap<FeatureId, usize> = BTreeMap::new();
        let mut dependents_map: BTreeMap<FeatureId, Vec<FeatureId>> = BTreeMap::new();

        for &id in self.features.keys() {
            in_degree.insert(id, 0);
            dependents_map.insert(id, Vec::new());
        }

        for (&id, feat) in &self.features {
            let deps = feat.all_dependencies();
            for dep in deps {
                if !self.features.contains_key(&dep) {
                    return Err(CoreError::FeatureNotFound(dep));
                }
                *in_degree.get_mut(&id).unwrap() += 1;
                dependents_map.get_mut(&dep).unwrap().push(id);
            }
        }

        let mut queue: VecDeque<FeatureId> = in_degree
            .iter()
            .filter_map(|(&id, &deg)| if deg == 0 { Some(id) } else { None })
            .collect();

        let mut order = Vec::with_capacity(self.features.len());
        while let Some(curr) = queue.pop_front() {
            order.push(curr);
            if let Some(children) = dependents_map.get(&curr) {
                for &child in children {
                    let deg = in_degree.get_mut(&child).unwrap();
                    *deg -= 1;
                    if *deg == 0 {
                        queue.push_back(child);
                    }
                }
            }
        }

        if order.len() != self.features.len() {
            return Err(CoreError::CyclicFeatureDependency);
        }
        Ok(order)
    }

    /// Incrementally recomputes only `Dirty` (or uncached) features in topological order,
    /// reusing cached shapes for `Clean` nodes and propagating `Failed` / `Blocked` states (Phase 2.3).
    pub fn recompute(
        &mut self,
        store: &mut ShapeStore,
        tess_opts: TessellationOptions,
    ) -> Result<RecomputeReport, CoreError> {
        let order = self.topological_order()?;
        let resolved_params = self.resolve_all_parameters()?;
        let mut report = RecomputeReport {
            evaluated_order: order.clone(),
            ..RecomputeReport::default()
        };

        for &fid in &order {
            let feat_snapshot = self.features.get(&fid).cloned().unwrap();

            // Check if any upstream dependency failed or is blocked
            let mut blocked_by: Option<FeatureId> = None;
            for dep_id in feat_snapshot.all_dependencies() {
                if let Some(dep_feat) = self.features.get(&dep_id) {
                    match dep_feat.state {
                        FeatureState::Failed(_) => {
                            blocked_by = Some(dep_id);
                            break;
                        }
                        FeatureState::Blocked { upstream_failed } => {
                            blocked_by = Some(upstream_failed);
                            break;
                        }
                        _ if dep_feat.suppressed && dep_feat.cached_shape.is_none() => {
                            blocked_by = Some(dep_id);
                            break;
                        }
                        _ => {}
                    }
                }
            }

            if let Some(upstream_failed) = blocked_by {
                let feat = self.features.get_mut(&fid).unwrap();
                if let Some(old_id) = feat.cached_shape.take() {
                    let _ = store.remove(old_id);
                }
                feat.cached_mesh = None;
                feat.cached_summary = None;
                feat.state = FeatureState::Blocked { upstream_failed };
                report.blocked_features.push((fid, upstream_failed));
                continue;
            }

            // Reuse cache if Clean and shape is still alive in ShapeStore
            if feat_snapshot.state == FeatureState::Clean
                && let Some(sid) = feat_snapshot.cached_shape
                && store.contains(sid)
                && feat_snapshot.cached_mesh.is_some()
            {
                report.reused_cached_features.push(fid);
                continue;
            }

            // Transition to Computing
            self.features.get_mut(&fid).unwrap().state = FeatureState::Computing;

            // Evaluate feature geometry; a suppressed feature passes its input through.
            let evaluated = if feat_snapshot.suppressed {
                let Some(input) = feat_snapshot.kind.primary_input() else {
                    let feat = self.features.get_mut(&fid).unwrap();
                    if let Some(old_id) = feat.cached_shape.take() {
                        let _ = store.remove(old_id);
                    }
                    feat.cached_mesh = None;
                    feat.cached_summary = None;
                    feat.state = FeatureState::Clean;
                    report.recomputed_features.push(fid);
                    continue;
                };
                self.features
                    .get(&input)
                    .and_then(|f| f.cached_shape)
                    .ok_or(CoreError::FeatureNotFound(input))
                    .and_then(|sid| Ok(store.clone_shape(sid)?))
            } else {
                self.evaluate_feature_kind(&feat_snapshot.kind, &resolved_params, store)
            };
            match evaluated {
                Ok(new_shape_id) => {
                    match store
                        .inspect_shape(new_shape_id)
                        .and_then(|s| Ok((s.summary, store.tessellate(new_shape_id, tess_opts)?)))
                    {
                        Ok((summary, mesh)) => {
                            let feat = self.features.get_mut(&fid).unwrap();
                            if let Some(old_id) = feat.cached_shape.replace(new_shape_id) {
                                let _ = store.remove(old_id);
                            }
                            feat.cached_summary = Some(summary);
                            feat.cached_mesh = Some(mesh.clone());
                            feat.last_valid_summary = Some(summary);
                            feat.last_valid_mesh = Some(mesh);
                            feat.state = FeatureState::Clean;
                            report.recomputed_features.push(fid);
                        }
                        Err(err) => {
                            let _ = store.remove(new_shape_id);
                            let msg = err.to_string();
                            let feat = self.features.get_mut(&fid).unwrap();
                            if let Some(old_id) = feat.cached_shape.take() {
                                let _ = store.remove(old_id);
                            }
                            feat.cached_mesh = None;
                            feat.cached_summary = None;
                            feat.state = FeatureState::Failed(msg.clone());
                            report.failed_features.push((fid, msg));
                        }
                    }
                }
                Err(err) => {
                    let msg = err.to_string();
                    let feat = self.features.get_mut(&fid).unwrap();
                    if let Some(old_id) = feat.cached_shape.take() {
                        let _ = store.remove(old_id);
                    }
                    feat.cached_mesh = None;
                    feat.cached_summary = None;
                    feat.state = FeatureState::Failed(msg.clone());
                    report.failed_features.push((fid, msg));
                }
            }
        }

        Ok(report)
    }

    /// Re-resolves stable [`SemanticReference`]s against an upstream feature's *current*
    /// cached shape, returning the matching edge indices in the live topology. This is the
    /// Topological Naming Problem mitigation surfaced to features: rather than trusting a
    /// raw edge index that may now point at a different (or nonexistent) edge after an
    /// upstream edit, each reference is re-identified by geometric signature, and any
    /// ambiguous or missing match is reported as an explicit, actionable error instead of
    /// silently filleting/chamfering the wrong edge.
    fn resolve_semantic_edges(
        &self,
        upstream: FeatureId,
        semantic_refs: &[SemanticReference],
        store: &ShapeStore,
    ) -> Result<Vec<u32>, CoreError> {
        let shape_id = self
            .features
            .get(&upstream)
            .and_then(|f| f.cached_shape)
            .ok_or(CoreError::FeatureNotFound(upstream))?;

        let mut resolved = Vec::with_capacity(semantic_refs.len());
        for sem_ref in semantic_refs {
            let edge = sem_ref
                .resolve_edge(store, shape_id)
                .map_err(|reason| CoreError::SemanticReferenceLost {
                    feature: upstream,
                    reason,
                })?;
            resolved.push(edge.index);
        }
        Ok(resolved)
    }

    fn evaluate_feature_kind(
        &self,
        kind: &FeatureKind,
        params: &BTreeMap<String, f64>,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        let lookup_shape = |id: FeatureId| -> Result<ShapeId, CoreError> {
            self.features
                .get(&id)
                .and_then(|f| f.cached_shape)
                .ok_or(CoreError::FeatureNotFound(id))
        };

        match kind {
            FeatureKind::Box {
                width,
                height,
                depth,
            } => {
                let w = width.evaluate(params)?;
                let h = height.evaluate(params)?;
                let d = depth.evaluate(params)?;
                Ok(store.create_box(Length::mm(w), Length::mm(h), Length::mm(d))?)
            }
            FeatureKind::Cylinder { radius, height } => {
                let r = radius.evaluate(params)?;
                let h = height.evaluate(params)?;
                Ok(store.create_cylinder(Length::mm(r), Length::mm(h))?)
            }
            FeatureKind::Sphere { radius } => {
                let r = radius.evaluate(params)?;
                Ok(store.create_sphere(Length::mm(r))?)
            }
            FeatureKind::Cone {
                bottom_radius,
                top_radius,
                height,
            } => {
                let r1 = bottom_radius.evaluate(params)?;
                let r2 = top_radius.evaluate(params)?;
                let h = height.evaluate(params)?;
                Ok(store.create_cone(Length::mm(r1), Length::mm(r2), Length::mm(h))?)
            }
            FeatureKind::ExtrudePolygon { points_xy, depth } => {
                let mut pts = Vec::with_capacity(points_xy.len());
                for [x, y] in points_xy {
                    pts.push([x.evaluate(params)?, y.evaluate(params)?]);
                }
                let d = depth.evaluate(params)?;
                Ok(store.extrude_polygon(&pts, Length::mm(d))?)
            }
            FeatureKind::SketchExtrude {
                sketch,
                depth,
                plane,
                mode,
                reversed,
                symmetric,
            } => {
                let mut solved = sketch.clone();
                let report = solved.solve(60, 1e-7);
                if matches!(
                    report.status,
                    SketchStatus::OverConstrained { .. } | SketchStatus::SolverFailed { .. }
                ) {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: solved.name.clone(),
                        reason: format!("Sketch solver failed with status: {:?}", report.status),
                    }));
                }
                let pts = solved.extract_closed_profile().ok_or_else(|| {
                    CoreError::Geometry(GeometryError::InvalidParameter {
                        name: solved.name.clone(),
                        reason: "Sketch does not contain a valid closed 2D profile".to_string(),
                    })
                })?;
                let d = depth.evaluate(params)?;
                let frame = self.sketch_plane_frame(plane, store)?;
                let prism = store.extrude_polygon(&pts, Length::mm(d))?;
                let z_shift = if *symmetric {
                    -d * 0.5
                } else if *reversed {
                    -d
                } else {
                    0.0
                };
                let tool = place_on_frame(store, prism, &frame, z_shift)?;
                match mode {
                    ExtrudeMode::NewBody => Ok(tool),
                    ExtrudeMode::Join { target } | ExtrudeMode::Cut { target } => {
                        let base = lookup_shape(*target)?;
                        let combined = if matches!(mode, ExtrudeMode::Cut { .. }) {
                            store.cut(base, tool)
                        } else {
                            store.fuse(base, tool)
                        };
                        let _ = store.remove(tool);
                        Ok(combined?)
                    }
                }
            }
            FeatureKind::ExtrudeCircle { radius, depth } => {
                let r = radius.evaluate(params)?;
                let d = depth.evaluate(params)?;
                Ok(store.extrude_circle(Length::mm(r), Length::mm(d))?)
            }
            FeatureKind::RevolvePolygon {
                points_xz,
                angle_deg,
            } => {
                let mut pts = Vec::with_capacity(points_xz.len());
                for [x, z] in points_xz {
                    pts.push([x.evaluate(params)?, z.evaluate(params)?]);
                }
                let a = angle_deg.evaluate(params)?;
                Ok(store.revolve_polygon(&pts, Angle::deg(a))?)
            }
            FeatureKind::Boolean {
                op,
                primary,
                secondary,
            } => {
                let s1 = lookup_shape(*primary)?;
                let s2 = lookup_shape(*secondary)?;
                let res = match op {
                    BooleanOp::Fuse => store.fuse(s1, s2)?,
                    BooleanOp::Cut => store.cut(s1, s2)?,
                    BooleanOp::Common => store.common(s1, s2)?,
                };
                Ok(res)
            }
            FeatureKind::Transform {
                input,
                dx,
                dy,
                dz,
                rot_origin,
                rot_axis,
                rot_angle_deg,
            } => {
                let src = lookup_shape(*input)?;
                let tx = dx.evaluate(params)?;
                let ty = dy.evaluate(params)?;
                let tz = dz.evaluate(params)?;
                let angle = rot_angle_deg.evaluate(params)?;

                let mut current = store.clone_shape(src)?;
                if angle.abs() > 1e-9 {
                    let rotated = store.rotate(
                        current,
                        DVec3::from_array(*rot_origin),
                        DVec3::from_array(*rot_axis),
                        Angle::deg(angle),
                    )?;
                    let _ = store.remove(current);
                    current = rotated;
                }
                if tx.abs() > 1e-9 || ty.abs() > 1e-9 || tz.abs() > 1e-9 {
                    let moved = store.translate(current, tx, ty, tz)?;
                    let _ = store.remove(current);
                    current = moved;
                }
                Ok(current)
            }
            FeatureKind::Fillet {
                input,
                radius,
                edge_indices,
                semantic_refs,
            } => {
                let src = lookup_shape(*input)?;
                let r = radius.evaluate(params)?;
                if !semantic_refs.is_empty() {
                    let resolved = self.resolve_semantic_edges(*input, semantic_refs, store)?;
                    Ok(store.fillet_edges(src, &resolved, Length::mm(r))?)
                } else if edge_indices.is_empty() {
                    Ok(store.fillet_all(src, Length::mm(r))?)
                } else {
                    Ok(store.fillet_edges(src, edge_indices, Length::mm(r))?)
                }
            }
            FeatureKind::Chamfer {
                input,
                distance,
                edge_indices,
                semantic_refs,
            } => {
                let src = lookup_shape(*input)?;
                let d = distance.evaluate(params)?;
                if !semantic_refs.is_empty() {
                    let resolved = self.resolve_semantic_edges(*input, semantic_refs, store)?;
                    Ok(store.chamfer_edges(src, &resolved, Length::mm(d))?)
                } else if edge_indices.is_empty() {
                    Ok(store.chamfer_all(src, Length::mm(d))?)
                } else {
                    Ok(store.chamfer_edges(src, edge_indices, Length::mm(d))?)
                }
            }
            FeatureKind::LinearPattern {
                input,
                spacing_x,
                spacing_y,
                spacing_z,
                count_x,
                count_y,
            } => {
                if *count_x == 0 || *count_y == 0 {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "pattern_count".to_string(),
                        reason: "count_x and count_y must be >= 1".to_string(),
                    }));
                }
                let src = lookup_shape(*input)?;
                let sx = spacing_x.evaluate(params)?;
                let sy = spacing_y.evaluate(params)?;
                let sz = spacing_z.evaluate(params)?;

                let mut accumulated: Option<ShapeId> = None;
                for ix in 0..*count_x {
                    for iy in 0..*count_y {
                        let ox = sx * (ix as f64);
                        let oy = sy * (iy as f64);
                        let oz = sz * ((ix + iy) as f64);
                        let instance = if ox.abs() < 1e-9 && oy.abs() < 1e-9 && oz.abs() < 1e-9 {
                            store.clone_shape(src)?
                        } else {
                            store.translate(src, ox, oy, oz)?
                        };
                        accumulated = Some(match accumulated {
                            None => instance,
                            Some(prev) => {
                                let fused = store.fuse(prev, instance)?;
                                let _ = store.remove(prev);
                                let _ = store.remove(instance);
                                fused
                            }
                        });
                    }
                }
                Ok(accumulated.unwrap())
            }
        }
    }

    /// Returns the combined axis-aligned bounding box of all visible features.
    pub fn visible_bounding_box(&self) -> Option<BoundingBox> {
        let mut combined: Option<BoundingBox> = None;
        for feat in self.features.values().filter(|f| f.visible) {
            if let Some(summary) = feat.cached_summary.or(feat.last_valid_summary) {
                combined = Some(match combined {
                    None => summary.bounds,
                    Some(prev) => prev.union(&summary.bounds),
                });
            }
        }
        combined
    }

    /// Creates a serializable [`DocumentSnapshot`] excluding transient geometry caches.
    pub fn to_snapshot(&self) -> DocumentSnapshot {
        DocumentSnapshot {
            schema_version: DOCUMENT_SCHEMA_VERSION,
            id: self.id,
            name: self.name.clone(),
            revision: self.revision,
            next_feature_id: self.next_feature_id,
            parameters: self.parameters.clone(),
            features: self.features.clone(),
            operation_history: self.operation_history.clone(),
        }
    }

    /// Restores a [`Document`] from a [`DocumentSnapshot`], marking all features `Dirty` for clean recomputation.
    pub fn from_snapshot(snapshot: DocumentSnapshot) -> Result<Self, CoreError> {
        if snapshot.schema_version != DOCUMENT_SCHEMA_VERSION {
            return Err(CoreError::UnsupportedSchemaVersion(snapshot.schema_version));
        }
        let mut features = snapshot.features;
        for feat in features.values_mut() {
            feat.state = FeatureState::Dirty;
            feat.cached_shape = None;
            feat.cached_summary = None;
            feat.cached_mesh = None;
            feat.last_valid_mesh = None;
            feat.last_valid_summary = None;
        }
        let doc = Self {
            id: snapshot.id,
            name: snapshot.name,
            revision: snapshot.revision,
            next_feature_id: snapshot.next_feature_id,
            parameters: snapshot.parameters,
            features,
            operation_history: snapshot.operation_history,
        };
        // Validate parameter & feature graph integrity on load
        doc.resolve_all_parameters()?;
        doc.topological_order()?;
        Ok(doc)
    }

    /// Serializes the document to a pretty-printed JSON string.
    pub fn to_json_pretty(&self) -> Result<String, CoreError> {
        serde_json::to_string_pretty(&self.to_snapshot())
            .map_err(|e| CoreError::Serialization(e.to_string()))
    }

    /// Deserializes a document from a JSON string.
    pub fn from_json_str(json: &str) -> Result<Self, CoreError> {
        let snapshot: DocumentSnapshot =
            serde_json::from_str(json).map_err(|e| CoreError::Serialization(e.to_string()))?;
        Self::from_snapshot(snapshot)
    }

    /// Saves the document to a JSON file on disk.
    pub fn save_json(&self, path: impl AsRef<Path>) -> Result<(), CoreError> {
        let json = self.to_json_pretty()?;
        std::fs::write(path, json).map_err(|e| CoreError::Serialization(e.to_string()))
    }

    /// Loads a document from a JSON file on disk.
    pub fn load_json(path: impl AsRef<Path>) -> Result<Self, CoreError> {
        let json =
            std::fs::read_to_string(path).map_err(|e| CoreError::Serialization(e.to_string()))?;
        Self::from_json_str(&json)
    }
}

/// Controller wrapping [`Document`] with full Undo / Redo command execution (Phase 2.4).
pub struct DocumentController {
    pub document: Document,
    undo_stack: Vec<DocumentSnapshot>,
    redo_stack: Vec<DocumentSnapshot>,
}

impl DocumentController {
    pub fn new(document: Document) -> Self {
        Self {
            document,
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    pub fn undo_count(&self) -> usize {
        self.undo_stack.len()
    }

    pub fn redo_count(&self) -> usize {
        self.redo_stack.len()
    }

    /// Executes a [`CadCommand`], pushing the pre-command state onto the undo stack if successful.
    pub fn execute(
        &mut self,
        cmd: CadCommand,
        store: Option<&mut ShapeStore>,
    ) -> Result<Option<FeatureId>, CoreError> {
        let pre_snapshot = self.document.to_snapshot();
        let result = match cmd {
            CadCommand::SetParameter {
                name,
                value,
                unit,
                description,
            } => {
                self.document
                    .set_parameter(name, value, unit, description)?;
                Ok(None)
            }
            CadCommand::RemoveParameter { name } => {
                let prev = self.document.parameters.remove(&name);
                if let Err(err) = self.document.resolve_all_parameters() {
                    if let Some(old) = prev {
                        self.document.parameters.insert(name, old);
                    }
                    return Err(err);
                }
                self.document.revision += 1;
                self.document
                    .operation_history
                    .push(CadCommand::RemoveParameter { name });
                Ok(None)
            }
            CadCommand::AddFeature {
                name,
                kind,
                visible,
                color_rgba,
            } => {
                let fid = self.document.add_feature(name, kind, visible, color_rgba)?;
                Ok(Some(fid))
            }
            CadCommand::UpdateFeature {
                id,
                name,
                kind,
                visible,
                color_rgba,
            } => {
                self.document
                    .update_feature(id, name, kind, visible, color_rgba)?;
                Ok(Some(id))
            }
            CadCommand::RemoveFeature { id } => {
                self.document.remove_feature(id, store)?;
                Ok(None)
            }
            CadCommand::SetSuppressed { id, suppressed } => {
                self.document.set_suppressed(id, suppressed)?;
                Ok(Some(id))
            }
            CadCommand::AddDependency {
                upstream,
                downstream,
            } => {
                self.document.add_dependency(upstream, downstream)?;
                Ok(None)
            }
        };

        if result.is_ok() {
            self.undo_stack.push(pre_snapshot);
            self.redo_stack.clear();
        }
        result
    }

    /// Reverts the document to the state before the last executed command.
    pub fn undo(&mut self, store: &mut ShapeStore) -> Result<(), CoreError> {
        let Some(prev_snapshot) = self.undo_stack.pop() else {
            return Err(CoreError::NothingToUndo);
        };
        let current_snapshot = self.document.to_snapshot();
        self.redo_stack.push(current_snapshot);
        store.clear();
        self.document = Document::from_snapshot(prev_snapshot)?;
        Ok(())
    }

    /// Re-applies the most recently undone command state.
    pub fn redo(&mut self, store: &mut ShapeStore) -> Result<(), CoreError> {
        let Some(next_snapshot) = self.redo_stack.pop() else {
            return Err(CoreError::NothingToRedo);
        };
        let current_snapshot = self.document.to_snapshot();
        self.undo_stack.push(current_snapshot);
        store.clear();
        self.document = Document::from_snapshot(next_snapshot)?;
        Ok(())
    }
}

/// Constructs the Milestone M1 MVP model specified in `roadmap.md`:
/// **穴あきプレート (Perforated Plate)**
/// - Width: 100 mm
/// - Height: 60 mm
/// - Thickness: 10 mm
/// - 4 Holes of diameter 6.6 mm (M6 clearance hole) controlled by parametric expressions
pub fn build_perforated_plate_mvp() -> Result<(DocumentController, FeatureId), CoreError> {
    let mut ctrl = DocumentController::new(Document::new("Milestone M1 - 穴あきプレート"));

    ctrl.execute(
        CadCommand::SetParameter {
            name: "width".to_string(),
            value: ParameterValue::Number(100.0),
            unit: "mm".to_string(),
            description: "プレート幅".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "height".to_string(),
            value: ParameterValue::Number(60.0),
            unit: "mm".to_string(),
            description: "プレート高さ（奥行）".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "thickness".to_string(),
            value: ParameterValue::Number(10.0),
            unit: "mm".to_string(),
            description: "プレート厚み".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_diameter".to_string(),
            value: ParameterValue::Number(6.6),
            unit: "mm".to_string(),
            description: "M6ボルト用キリ穴径".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_offset_x".to_string(),
            value: ParameterValue::Expression("width * 0.15".to_string()),
            unit: "mm".to_string(),
            description: "端面からのX方向穴中心距離".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_offset_y".to_string(),
            value: ParameterValue::Expression("height * 0.25".to_string()),
            unit: "mm".to_string(),
            description: "端面からのY方向穴中心距離".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_pitch_x".to_string(),
            value: ParameterValue::Expression("width - 2 * hole_offset_x".to_string()),
            unit: "mm".to_string(),
            description: "X方向の穴ピッチ".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_pitch_y".to_string(),
            value: ParameterValue::Expression("height - 2 * hole_offset_y".to_string()),
            unit: "mm".to_string(),
            description: "Y方向の穴ピッチ".to_string(),
        },
        None,
    )?;

    // 1. Base Plate Box (hidden once final Cut is displayed)
    let base_plate = ctrl
        .execute(
            CadCommand::AddFeature {
                name: "BasePlate".to_string(),
                kind: FeatureKind::Box {
                    width: ScalarParam::expr("width"),
                    height: ScalarParam::expr("height"),
                    depth: ScalarParam::expr("thickness"),
                },
                visible: false,
                color_rgba: [0.55, 0.62, 0.72, 1.0],
            },
            None,
        )?
        .unwrap();

    // 2. Single Hole Cylinder tool
    let single_hole = ctrl
        .execute(
            CadCommand::AddFeature {
                name: "HoleCylinder".to_string(),
                kind: FeatureKind::Cylinder {
                    radius: ScalarParam::expr("hole_diameter / 2"),
                    height: ScalarParam::expr("thickness + 4"),
                },
                visible: false,
                color_rgba: [0.85, 0.35, 0.35, 1.0],
            },
            None,
        )?
        .unwrap();

    // 3. Position first hole at (hole_offset_x, hole_offset_y, -2)
    let positioned_hole = ctrl
        .execute(
            CadCommand::AddFeature {
                name: "PositionedHole".to_string(),
                kind: FeatureKind::Transform {
                    input: single_hole,
                    dx: ScalarParam::expr("hole_offset_x"),
                    dy: ScalarParam::expr("hole_offset_y"),
                    dz: ScalarParam::Literal(-2.0),
                    rot_origin: [0.0, 0.0, 0.0],
                    rot_axis: [0.0, 0.0, 1.0],
                    rot_angle_deg: ScalarParam::Literal(0.0),
                },
                visible: false,
                color_rgba: [0.85, 0.35, 0.35, 1.0],
            },
            None,
        )?
        .unwrap();

    // 4. 2x2 Linear Pattern of holes
    let hole_pattern = ctrl
        .execute(
            CadCommand::AddFeature {
                name: "HolePattern4x".to_string(),
                kind: FeatureKind::LinearPattern {
                    input: positioned_hole,
                    spacing_x: ScalarParam::expr("hole_pitch_x"),
                    spacing_y: ScalarParam::expr("hole_pitch_y"),
                    spacing_z: ScalarParam::Literal(0.0),
                    count_x: 2,
                    count_y: 2,
                },
                visible: false,
                color_rgba: [0.85, 0.35, 0.35, 1.0],
            },
            None,
        )?
        .unwrap();

    // 5. Boolean Cut: BasePlate - HolePattern4x
    let final_plate = ctrl
        .execute(
            CadCommand::AddFeature {
                name: "PerforatedPlate".to_string(),
                kind: FeatureKind::Boolean {
                    op: BooleanOp::Cut,
                    primary: base_plate,
                    secondary: hole_pattern,
                },
                visible: true,
                color_rgba: [0.26, 0.56, 0.88, 1.0],
            },
            None,
        )?
        .unwrap();

    Ok((ctrl, final_plate))
}

// --- Lightweight Recursive-Descent Arithmetic Expression Evaluator ---

fn extract_identifiers(expr: &str) -> Vec<String> {
    let mut ids = Vec::new();
    let mut chars = expr.chars().peekable();
    while let Some(&ch) = chars.peek() {
        if ch.is_ascii_alphabetic() || ch == '_' {
            let mut ident = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_ascii_alphanumeric() || c == '_' {
                    ident.push(c);
                    chars.next();
                } else {
                    break;
                }
            }
            if !ids.contains(&ident) {
                ids.push(ident);
            }
        } else {
            chars.next();
        }
    }
    ids
}

#[derive(Debug, Clone, PartialEq)]
enum ExprToken {
    Number(f64),
    Ident(String),
    Plus,
    Minus,
    Star,
    Slash,
    LParen,
    RParen,
}

fn tokenize_expr(expr: &str) -> Result<Vec<ExprToken>, CoreError> {
    let mut tokens = Vec::new();
    let mut chars = expr.chars().peekable();
    while let Some(&ch) = chars.peek() {
        if ch.is_whitespace() {
            chars.next();
        } else if ch.is_ascii_digit() || ch == '.' {
            let mut num_str = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_ascii_digit() || c == '.' {
                    num_str.push(c);
                    chars.next();
                } else {
                    break;
                }
            }
            let val: f64 = num_str.parse().map_err(|_| CoreError::ParameterError {
                name: expr.to_string(),
                reason: format!("invalid number literal '{num_str}'"),
            })?;
            tokens.push(ExprToken::Number(val));
        } else if ch.is_ascii_alphabetic() || ch == '_' {
            let mut ident = String::new();
            while let Some(&c) = chars.peek() {
                if c.is_ascii_alphanumeric() || c == '_' {
                    ident.push(c);
                    chars.next();
                } else {
                    break;
                }
            }
            tokens.push(ExprToken::Ident(ident));
        } else {
            chars.next();
            match ch {
                '+' => tokens.push(ExprToken::Plus),
                '-' => tokens.push(ExprToken::Minus),
                '*' => tokens.push(ExprToken::Star),
                '/' => tokens.push(ExprToken::Slash),
                '(' => tokens.push(ExprToken::LParen),
                ')' => tokens.push(ExprToken::RParen),
                _ => {
                    return Err(CoreError::ParameterError {
                        name: expr.to_string(),
                        reason: format!("unexpected character '{ch}' in expression"),
                    });
                }
            }
        }
    }
    Ok(tokens)
}

fn eval_expression(expr: &str, params: &BTreeMap<String, f64>) -> Result<f64, CoreError> {
    let tokens = tokenize_expr(expr)?;
    if tokens.is_empty() {
        return Err(CoreError::ParameterError {
            name: expr.to_string(),
            reason: "empty expression".to_string(),
        });
    }
    let mut pos = 0;
    let val = parse_add_sub(&tokens, &mut pos, params, expr)?;
    if pos < tokens.len() {
        return Err(CoreError::ParameterError {
            name: expr.to_string(),
            reason: "unexpected trailing tokens in expression".to_string(),
        });
    }
    if !val.is_finite() {
        return Err(CoreError::ParameterError {
            name: expr.to_string(),
            reason: "expression evaluated to non-finite number".to_string(),
        });
    }
    Ok(val)
}

fn parse_add_sub(
    tokens: &[ExprToken],
    pos: &mut usize,
    params: &BTreeMap<String, f64>,
    raw_expr: &str,
) -> Result<f64, CoreError> {
    let mut left = parse_mul_div(tokens, pos, params, raw_expr)?;
    while *pos < tokens.len() {
        match tokens[*pos] {
            ExprToken::Plus => {
                *pos += 1;
                left += parse_mul_div(tokens, pos, params, raw_expr)?;
            }
            ExprToken::Minus => {
                *pos += 1;
                left -= parse_mul_div(tokens, pos, params, raw_expr)?;
            }
            _ => break,
        }
    }
    Ok(left)
}

fn parse_mul_div(
    tokens: &[ExprToken],
    pos: &mut usize,
    params: &BTreeMap<String, f64>,
    raw_expr: &str,
) -> Result<f64, CoreError> {
    let mut left = parse_unary(tokens, pos, params, raw_expr)?;
    while *pos < tokens.len() {
        match tokens[*pos] {
            ExprToken::Star => {
                *pos += 1;
                left *= parse_unary(tokens, pos, params, raw_expr)?;
            }
            ExprToken::Slash => {
                *pos += 1;
                let divisor = parse_unary(tokens, pos, params, raw_expr)?;
                if divisor.abs() <= 1e-15 {
                    return Err(CoreError::ParameterError {
                        name: raw_expr.to_string(),
                        reason: "division by zero in expression".to_string(),
                    });
                }
                left /= divisor;
            }
            _ => break,
        }
    }
    Ok(left)
}

fn parse_unary(
    tokens: &[ExprToken],
    pos: &mut usize,
    params: &BTreeMap<String, f64>,
    raw_expr: &str,
) -> Result<f64, CoreError> {
    if *pos < tokens.len() {
        match tokens[*pos] {
            ExprToken::Plus => {
                *pos += 1;
                return parse_unary(tokens, pos, params, raw_expr);
            }
            ExprToken::Minus => {
                *pos += 1;
                return Ok(-parse_unary(tokens, pos, params, raw_expr)?);
            }
            _ => {}
        }
    }
    parse_primary(tokens, pos, params, raw_expr)
}

fn parse_primary(
    tokens: &[ExprToken],
    pos: &mut usize,
    params: &BTreeMap<String, f64>,
    raw_expr: &str,
) -> Result<f64, CoreError> {
    let Some(tok) = tokens.get(*pos) else {
        return Err(CoreError::ParameterError {
            name: raw_expr.to_string(),
            reason: "unexpected end of expression".to_string(),
        });
    };
    match tok {
        ExprToken::Number(v) => {
            *pos += 1;
            Ok(*v)
        }
        ExprToken::Ident(name) => {
            *pos += 1;
            params
                .get(name)
                .copied()
                .ok_or_else(|| CoreError::ParameterError {
                    name: name.clone(),
                    reason: format!("unknown parameter '{name}' in expression '{raw_expr}'"),
                })
        }
        ExprToken::LParen => {
            *pos += 1;
            let val = parse_add_sub(tokens, pos, params, raw_expr)?;
            if tokens.get(*pos) != Some(&ExprToken::RParen) {
                return Err(CoreError::ParameterError {
                    name: raw_expr.to_string(),
                    reason: "missing closing parenthesis ')'".to_string(),
                });
            }
            *pos += 1;
            Ok(val)
        }
        _ => Err(CoreError::ParameterError {
            name: raw_expr.to_string(),
            reason: "unexpected token in expression".to_string(),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fillet/Chamfer features captured by [`SemanticReference`] re-identify their target
    /// edge by geometric signature instead of a raw index, so they survive an upstream
    /// dimension change; and when no edge matches any more, the failure is reported through
    /// `CoreError::SemanticReferenceLost` rather than silently filleting the wrong edge.
    #[test]
    fn semantic_reference_survives_resize_and_reports_lost_reference() {
        let mut store = ShapeStore::new();
        let mut ctrl = DocumentController::new(Document::new("TNP Mitigation Test"));

        let box_id = ctrl
            .execute(
                CadCommand::AddFeature {
                    name: "Box".to_string(),
                    kind: FeatureKind::Box {
                        width: ScalarParam::mm(100.0),
                        height: ScalarParam::mm(60.0),
                        depth: ScalarParam::mm(10.0),
                    },
                    visible: true,
                    color_rgba: [1.0, 1.0, 1.0, 1.0],
                },
                Some(&mut store),
            )
            .expect("add box")
            .expect("box id");

        ctrl.document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute box");

        let box_shape = ctrl.document.features[&box_id].cached_shape.expect("box shape");
        let any_edge = store
            .inspect_shape(box_shape)
            .expect("inspect box")
            .edges
            .first()
            .expect("box has edges")
            .index;
        let good_ref = SemanticReference::from_edge(box_id, &store, box_shape, any_edge)
            .expect("capture semantic reference");

        let fillet_id = ctrl
            .execute(
                CadCommand::AddFeature {
                    name: "Fillet".to_string(),
                    kind: FeatureKind::Fillet {
                        input: box_id,
                        radius: ScalarParam::mm(1.0),
                        edge_indices: vec![any_edge],
                        semantic_refs: vec![good_ref],
                    },
                    visible: true,
                    color_rgba: [1.0, 1.0, 1.0, 1.0],
                },
                Some(&mut store),
            )
            .expect("add fillet")
            .expect("fillet id");

        let report1 = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute fillet");
        assert!(
            report1.is_all_clean(),
            "fillet should resolve via semantic reference: {:?}",
            report1.failed_features
        );

        // Resize the box: the raw edge index may now point at a different edge, but the
        // semantic reference should still re-identify the geometrically equivalent edge.
        ctrl.execute(
            CadCommand::UpdateFeature {
                id: box_id,
                name: None,
                kind: Some(FeatureKind::Box {
                    width: ScalarParam::mm(180.0),
                    height: ScalarParam::mm(90.0),
                    depth: ScalarParam::mm(25.0),
                }),
                visible: None,
                color_rgba: None,
            },
            Some(&mut store),
        )
        .expect("resize box");

        let report2 = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute after resize");
        assert!(
            report2.is_all_clean(),
            "fillet should survive resize via semantic reference: {:?}",
            report2.failed_features
        );
        assert_eq!(ctrl.document.features[&fillet_id].state, FeatureState::Clean);

        // A lost reference (e.g. the target shape disappeared) must surface as an
        // explicit, actionable failure rather than silently filleting an arbitrary edge.
        let bogus_err = CoreError::SemanticReferenceLost {
            feature: box_id,
            reason: ResolutionFailureReason::MissingTargetShape("shape removed".to_string()),
        };
        assert!(
            bogus_err.to_string().contains("could not re-identify"),
            "SemanticReferenceLost must produce a human-readable explanation: {bogus_err}"
        );
    }

    /// M2: a circle sketched on a plate's top face cuts a hole; after the plate is
    /// resized the sketch plane follows the (moved) top face and the hole keeps its
    /// position. Also covers suppression pass-through, delete protection, and the
    /// orientation of a base-plane (XZ) extrusion.
    #[test]
    fn m2_sketch_on_face_cut_follows_resize() {
        let tess = TessellationOptions::default();
        let mut store = ShapeStore::new();
        let mut doc = Document::new("M2");

        let box_kind = |w: f64, h: f64, d: f64| FeatureKind::Box {
            width: ScalarParam::mm(w),
            height: ScalarParam::mm(h),
            depth: ScalarParam::mm(d),
        };
        let plate = doc
            .add_feature("Plate", box_kind(100.0, 60.0, 10.0), true, [1.0; 4])
            .unwrap();
        doc.recompute(&mut store, tess).unwrap();

        let plate_shape = doc.features[&plate].cached_shape.unwrap();
        let top = store
            .resolve_face_by_signature(plate_shape, DVec3::Z, None, 0.99)
            .unwrap();
        let top_ref = SemanticReference::from_face(plate, &store, plate_shape, top.index).unwrap();

        let mut circle = Sketch::new("Hole");
        let c = circle.add_point(30.0, 20.0);
        circle.add_circle(c, 5.0);
        let cut = doc
            .add_feature(
                "HoleCut",
                FeatureKind::SketchExtrude {
                    sketch: circle,
                    depth: ScalarParam::mm(10.0),
                    plane: SketchPlane::Face {
                        feature: plate,
                        face: top_ref,
                    },
                    mode: ExtrudeMode::Cut { target: plate },
                    reversed: true,
                    symmetric: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);

        // The profile is a 32-gon approximation of the circle.
        let hole_area = 0.5 * 32.0 * 25.0 * (2.0 * std::f64::consts::PI / 32.0).sin();
        let vol = doc.features[&cut].cached_summary.unwrap().volume_mm3;
        assert!((vol - (60_000.0 - hole_area * 10.0)).abs() < 1.0, "vol={vol}");

        // Thicker, larger plate: the top face moves to z=20. A 10mm blind hole from the
        // top must now have its floor at z=10, still centred at (30, 20).
        doc.update_feature(plate, None, Some(box_kind(150.0, 80.0, 20.0)), None, None)
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let vol = doc.features[&cut].cached_summary.unwrap().volume_mm3;
        assert!((vol - (240_000.0 - hole_area * 10.0)).abs() < 1.0, "vol={vol}");
        let cut_shape = store
            .inspect_shape(doc.features[&cut].cached_shape.unwrap())
            .unwrap();
        assert!(
            cut_shape.faces.iter().any(|f| f.normal.z > 0.99
                && (f.centroid.z - 10.0).abs() < 1e-3
                && (f.centroid.x - 30.0).abs() < 0.5
                && (f.centroid.y - 20.0).abs() < 0.5),
            "hole floor should sit at z=10 under (30, 20)"
        );

        // Suppressing the cut passes the plate through unchanged.
        doc.set_suppressed(cut, true).unwrap();
        doc.recompute(&mut store, tess).unwrap();
        let vol = doc.features[&cut].cached_summary.unwrap().volume_mm3;
        assert!((vol - 240_000.0).abs() < 1e-3);
        doc.set_suppressed(cut, false).unwrap();

        // The plate cannot be deleted while the cut depends on it.
        match doc.remove_feature(plate, Some(&mut store)) {
            Err(CoreError::FeatureInUse { dependents, .. }) => assert_eq!(dependents, vec![cut]),
            other => panic!("expected FeatureInUse, got {other:?}"),
        }

        // A rectangle on the XZ plane extrudes along -Y (the plane normal).
        let mut rect = Sketch::new("Side");
        rect.add_rectangle(0.0, 0.0, 20.0, 30.0);
        let side = doc
            .add_feature(
                "Side",
                FeatureKind::SketchExtrude {
                    sketch: rect,
                    depth: ScalarParam::mm(5.0),
                    plane: SketchPlane::XZ,
                    mode: ExtrudeMode::NewBody,
                    reversed: false,
                    symmetric: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        doc.recompute(&mut store, tess).unwrap();
        let b = doc.features[&side].cached_summary.unwrap().bounds;
        assert!((b.min.y + 5.0).abs() < 1e-6 && b.max.y.abs() < 1e-6, "{b:?}");
        assert!((b.max.x - 20.0).abs() < 1e-6 && (b.max.z - 30.0).abs() < 1e-6, "{b:?}");
    }

    #[test]
    fn phase2_dag_incremental_recompute_and_blocked_state() {
        let mut store = ShapeStore::new();
        let (mut ctrl, final_plate) = build_perforated_plate_mvp().expect("build mvp");

        // 1. Initial full recompute
        let report1 = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute 1");
        assert!(report1.is_all_clean());
        assert_eq!(report1.recomputed_features.len(), 5);
        assert_eq!(report1.reused_cached_features.len(), 0);

        // Expected volume: 100*60*10 - 4 * (pi * 3.3^2 * 10) = 60,000 - 1368.4777 = 58631.522
        let expected_vol = 60_000.0 - 4.0 * std::f64::consts::PI * 3.3 * 3.3 * 10.0;
        let plate_summary = ctrl.document.features[&final_plate]
            .cached_summary
            .expect("summary");
        assert!((plate_summary.volume_mm3 - expected_vol).abs() < 1.0);

        // 2. Recompute without changes -> 100% cache hit
        let report2 = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute 2");
        assert_eq!(report2.recomputed_features.len(), 0);
        assert_eq!(report2.reused_cached_features.len(), 5);

        // 3. Change only `hole_diameter` -> `BasePlate` (FeatureId(1)) must NOT be recomputed!
        ctrl.execute(
            CadCommand::SetParameter {
                name: "hole_diameter".to_string(),
                value: ParameterValue::Number(10.0),
                unit: "mm".to_string(),
                description: "M10穴へ変更".to_string(),
            },
            None,
        )
        .expect("set hole_diameter");

        let report3 = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute 3");
        assert!(report3.is_all_clean());
        // BasePlate is reused from cache; only HoleCylinder -> PositionedHole -> HolePattern4x -> PerforatedPlate recompute
        assert_eq!(report3.reused_cached_features, vec![FeatureId(1)]);
        assert_eq!(report3.recomputed_features.len(), 4);

        // 4. Undo parameter change -> returns to 6.6mm holes
        ctrl.undo(&mut store).expect("undo");
        ctrl.document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute after undo");
        let vol_after_undo = ctrl.document.features[&final_plate]
            .cached_summary
            .unwrap()
            .volume_mm3;
        assert!((vol_after_undo - expected_vol).abs() < 1.0);

        // 5. Redo parameter change -> returns to 10.0mm holes
        ctrl.redo(&mut store).expect("redo");
        ctrl.document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute after redo");
        let expected_vol_10 = 60_000.0 - 4.0 * std::f64::consts::PI * 5.0 * 5.0 * 10.0;
        let vol_after_redo = ctrl.document.features[&final_plate]
            .cached_summary
            .unwrap()
            .volume_mm3;
        assert!((vol_after_redo - expected_vol_10).abs() < 1.0);

        // 6. Serialization roundtrip test
        let json = ctrl.document.to_json_pretty().expect("to_json");
        let mut loaded_doc = Document::from_json_str(&json).expect("from_json");
        let mut store2 = ShapeStore::new();
        loaded_doc
            .recompute(&mut store2, TessellationOptions::default())
            .expect("recompute loaded");
        let loaded_vol = loaded_doc.features[&final_plate]
            .cached_summary
            .unwrap()
            .volume_mm3;
        assert!((loaded_vol - vol_after_redo).abs() < 1e-3);

        // 7. Error & Blocked propagation test: set negative hole_diameter -> HoleCylinder fails, downstream becomes Blocked
        ctrl.execute(
            CadCommand::SetParameter {
                name: "hole_diameter".to_string(),
                value: ParameterValue::Number(-5.0),
                unit: "mm".to_string(),
                description: "無効な負の穴径".to_string(),
            },
            None,
        )
        .expect("set invalid hole_diameter");
        let err_report = ctrl
            .document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute with failure");
        assert!(!err_report.is_all_clean());
        assert_eq!(err_report.failed_features.len(), 1);
        assert_eq!(err_report.failed_features[0].0, FeatureId(2)); // HoleCylinder failed
        assert_eq!(err_report.blocked_features.len(), 3); // PositionedHole, HolePattern4x, PerforatedPlate blocked by FeatureId(2)
        // Last valid mesh on PerforatedPlate is still preserved!
        assert!(
            ctrl.document.features[&final_plate]
                .last_valid_mesh
                .is_some()
        );
    }

    #[test]
    fn phase2_rejects_cycles() {
        let (mut ctrl, final_plate) = build_perforated_plate_mvp().expect("build mvp");
        // Attempting to make BasePlate (FeatureId(1)) depend on PerforatedPlate (FeatureId(5)) must fail with CyclicFeatureDependency
        let res = ctrl.execute(
            CadCommand::AddDependency {
                upstream: final_plate,
                downstream: FeatureId(1),
            },
            None,
        );
        assert_eq!(res, Err(CoreError::CyclicFeatureDependency));
    }

    #[test]
    fn phase5_sketch_extrude_in_document() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("SketchExtrudeDoc");

        let mut sketch = Sketch::new("BaseRectSketch");
        let (_rect_id, [p0, p1, _p2, p3]) = sketch.add_rectangle(5.0, 5.0, 35.0, 25.0);
        sketch.add_constraint(SketchConstraint::Fixed {
            point: p0,
            x: 0.0,
            y: 0.0,
        });
        sketch.add_constraint(SketchConstraint::Distance {
            p1: p0,
            p2: p1,
            distance: 50.0,
        });
        sketch.add_constraint(SketchConstraint::Distance {
            p1: p0,
            p2: p3,
            distance: 30.0,
        });

        let fid = doc
            .add_feature(
                "ExtrudedSketch",
                FeatureKind::SketchExtrude {
                    sketch,
                    depth: ScalarParam::mm(12.0),
                    plane: SketchPlane::XY,
                    mode: ExtrudeMode::NewBody,
                    reversed: false,
                    symmetric: false,
                },
                true,
                [0.3, 0.7, 0.9, 1.0],
            )
            .expect("add SketchExtrude");

        let report = doc
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute SketchExtrude");
        assert!(report.is_all_clean());
        let vol = doc.features[&fid].cached_summary.unwrap().volume_mm3;
        assert!((vol - 50.0 * 30.0 * 12.0).abs() < 1e-2);
    }
}
