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
    EntityId, PointId, ProfileLoop, Sketch, SketchConstraint, SketchEntity, SketchSolveReport,
    SketchStatus, constraint_entity, constraint_points, entity_points,
};
use cad_geometry::ProfileSegment;

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

fn zero_scalar() -> ScalarParam {
    ScalarParam::Literal(0.0)
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
    /// A plane parallel to a referenced face, translated along its normal by `offset`.
    /// This is stored on sketch features so the construction plane follows upstream edits.
    OffsetFace {
        feature: FeatureId,
        face: SemanticReference,
        offset: ScalarParam,
    },
    /// Plane halfway between two parallel referenced faces.
    MidFace {
        first_feature: FeatureId,
        first: SemanticReference,
        second_feature: FeatureId,
        second: SemanticReference,
    },
}

impl SketchPlane {
    pub fn features(&self) -> Vec<FeatureId> {
        match self {
            Self::Face { feature, .. } | Self::OffsetFace { feature, .. } => vec![*feature],
            Self::MidFace { first_feature, second_feature, .. } if first_feature == second_feature => vec![*first_feature],
            Self::MidFace { first_feature, second_feature, .. } => vec![*first_feature, *second_feature],
            _ => Vec::new(),
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

/// Extrudes a set of closed loops along +Z by `depth`. A loop nested inside an odd
/// number of other loops is a hole and is subtracted; the rest are outer boundaries and
/// are fused. Holes are cut with a prism slightly taller than the body so the boolean
/// never has to resolve coplanar caps.
fn extrude_loops(
    store: &mut ShapeStore,
    loops: &[ProfileLoop],
    depth: f64,
) -> Result<ShapeId, CoreError> {
    const MARGIN: f64 = 0.01;
    let depth_of = |i: usize| {
        loops
            .iter()
            .enumerate()
            .filter(|&(j, other)| j != i && point_in_polygon(loops[i].points[0], &other.points))
            .count()
    };
    let (holes, outers): (Vec<usize>, Vec<usize>) =
        (0..loops.len()).partition(|&i| depth_of(i) % 2 == 1);

    let mut body: Option<ShapeId> = None;
    for i in outers {
        let prism = store.extrude_profile(&loops[i].segments, Length::mm(depth))?;
        body = Some(match body {
            None => prism,
            Some(b) => {
                let fused = store.fuse(b, prism)?;
                let _ = store.remove(b);
                let _ = store.remove(prism);
                fused
            }
        });
    }
    let mut body = body.ok_or_else(|| {
        CoreError::Geometry(GeometryError::InvalidParameter {
            name: "sketch".to_string(),
            reason: "Sketch contains only hole loops and no outer boundary".to_string(),
        })
    })?;
    for i in holes {
        let tall = store.extrude_profile(&loops[i].segments, Length::mm(depth + 2.0 * MARGIN))?;
        let tool = store.translate(tall, 0.0, 0.0, -MARGIN)?;
        let _ = store.remove(tall);
        let cut = store.cut(body, tool)?;
        let _ = store.remove(body);
        let _ = store.remove(tool);
        body = cut;
    }
    Ok(body)
}

fn point_in_polygon(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Which reference of a feature a [`RefSlot`] is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefRole {
    /// The i-th filleted / chamfered edge.
    Edge(usize),
    /// The i-th open face of a shell.
    OpenFace(usize),
    /// The face a sketch is drawn on, or a mirror plane face.
    Plane,
    /// The first face defining a midpoint sketch plane.
    MidPlaneFirst,
    /// The second face defining a midpoint sketch plane.
    MidPlaneSecond,
    /// A face tilted by a draft feature.
    DraftFace(usize),
    /// The fixed neutral plane of a draft feature.
    DraftNeutral,
    /// The planar face offset by a Push/Pull feature.
    PushPullFace,
    /// The planar face displaced by a FaceMove feature.
    FaceMoveFace,
    /// The base face for a PlanePattern feature.
    PlanePatternFace,
    /// A referenced edge or face for PatternOnReference.
    PatternAxisRef,
}

/// One face/edge reference held by a feature.
#[derive(Debug, Clone, PartialEq)]
pub struct RefSlot {
    pub role: RefRole,
    /// The feature whose shape the reference points into.
    pub upstream: FeatureId,
    pub reference: SemanticReference,
}

impl RefSlot {
    pub fn is_edge(&self) -> bool {
        matches!(self.role, RefRole::Edge(_))
    }
}

/// Reference geometry used as a direction or rotation axis for patterns (M10).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ReferenceAxis {
    /// A linear edge direction or circular edge axis.
    Edge {
        feature: FeatureId,
        edge: SemanticReference,
    },
    /// A cylindrical surface axis or planar face normal.
    Face {
        feature: FeatureId,
        face: SemanticReference,
    },
    /// A reference datum plane or face plane.
    Plane(SketchPlane),
}

impl ReferenceAxis {
    pub fn features(&self) -> Vec<FeatureId> {
        match self {
            Self::Edge { feature, .. } | Self::Face { feature, .. } => vec![*feature],
            Self::Plane(plane) => plane.features(),
        }
    }
}

/// Arrangement of pattern instances driven by a reference geometry (M10).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PatternReferenceLayout {
    Linear {
        spacing: ScalarParam,
        count: u32,
    },
    Circular {
        count: u32,
        total_angle_deg: ScalarParam,
    },
}

/// Revolve axis in sketch coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SketchAxis {
    /// The sketch's vertical axis through its origin.
    #[default]
    Y,
    /// The sketch's horizontal axis through its origin.
    X,
    /// A line of the sketch, given by its two points (follows the solved sketch).
    Line { a: PointId, b: PointId },
}

/// Arrangement of pattern instances.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PatternLayout {
    /// `count` instances around an axis; a 360° total spaces them evenly with no
    /// duplicate at the end, a smaller total spans first to last instance.
    Circular {
        axis_origin: [f64; 3],
        axis_dir: [f64; 3],
        count: u32,
        total_angle_deg: ScalarParam,
    },
    /// `count` instances along `direction`, `spacing` apart.
    Linear {
        direction: [f64; 3],
        spacing: ScalarParam,
        count: u32,
    },
}

impl PatternLayout {
    fn instances(&self, params: &BTreeMap<String, f64>) -> Result<Vec<Instance>, CoreError> {
        let bad = |reason: &str| {
            CoreError::Geometry(GeometryError::InvalidParameter {
                name: "pattern".to_string(),
                reason: reason.to_string(),
            })
        };
        match self {
            Self::Circular {
                axis_origin,
                axis_dir,
                count,
                total_angle_deg,
            } => {
                if *count < 1 {
                    return Err(bad("count must be >= 1"));
                }
                let total = total_angle_deg.evaluate(params)?;
                let step = if (total.abs() - 360.0).abs() < 1e-6 || *count == 1 {
                    total / *count as f64
                } else {
                    total / (*count - 1) as f64
                };
                Ok((0..*count)
                    .map(|i| match i {
                        0 => Instance::Identity,
                        _ => Instance::Rotate {
                            origin: DVec3::from_array(*axis_origin),
                            axis: DVec3::from_array(*axis_dir),
                            angle_deg: step * i as f64,
                        },
                    })
                    .collect())
            }
            Self::Linear {
                direction,
                spacing,
                count,
            } => {
                if *count < 1 {
                    return Err(bad("count must be >= 1"));
                }
                let dir = DVec3::from_array(*direction)
                    .try_normalize()
                    .ok_or_else(|| bad("direction must be non-zero"))?;
                let s = spacing.evaluate(params)?;
                Ok((0..*count)
                    .map(|i| match i {
                        0 => Instance::Identity,
                        _ => Instance::Translate(dir * s * i as f64),
                    })
                    .collect())
            }
        }
    }
}

/// One placement of a repeated tool or body.
#[derive(Debug, Clone, Copy)]
enum Instance {
    Identity,
    Translate(DVec3),
    Rotate {
        origin: DVec3,
        axis: DVec3,
        angle_deg: f64,
    },
    Mirror {
        origin: DVec3,
        normal: DVec3,
    },
}

impl Instance {
    fn apply(&self, store: &mut ShapeStore, src: ShapeId) -> Result<ShapeId, CoreError> {
        Ok(match *self {
            Self::Identity => store.clone_shape(src)?,
            Self::Translate(d) => store.translate(src, d.x, d.y, d.z)?,
            Self::Rotate {
                origin,
                axis,
                angle_deg,
            } => store.rotate(src, origin, axis, Angle::deg(angle_deg))?,
            Self::Mirror { origin, normal } => store.mirror(src, origin, normal)?,
        })
    }
}

/// Solves a sketch and returns its closed loops, with plain errors when it can't.
fn solved_loops(sketch: &Sketch) -> Result<(Sketch, Vec<ProfileLoop>), CoreError> {
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
    let loops = solved.extract_profile_loops();
    if loops.is_empty() {
        return Err(CoreError::Geometry(GeometryError::InvalidParameter {
            name: solved.name.clone(),
            reason: "Sketch does not contain a valid closed 2D profile".to_string(),
        }));
    }
    Ok((solved, loops))
}

/// Re-expresses a sketch loop in (distance along `side`, distance along `dir`) from `a`
/// — a rigid map, possibly mirrored — keeping arcs and circles exact.
fn map_profile_loop(l: &ProfileLoop, a: DVec3, side: DVec3, dir: DVec3) -> ProfileLoop {
    let map = |p: [f64; 2]| {
        let v = DVec3::new(p[0], p[1], 0.0) - a;
        [v.dot(side), v.dot(dir)]
    };
    let angle_of = |t: f64| {
        let u = DVec3::new(t.cos(), t.sin(), 0.0);
        u.dot(dir).atan2(u.dot(side))
    };
    let mirrored = side.x * dir.y - side.y * dir.x < 0.0;
    let segments = l
        .segments
        .iter()
        .map(|s| match *s {
            ProfileSegment::Line { a: p, b: q } => ProfileSegment::Line { a: map(p), b: map(q) },
            ProfileSegment::Circle { center, radius } => ProfileSegment::Circle {
                center: map(center),
                radius,
            },
            ProfileSegment::Arc {
                center,
                radius,
                start_rad,
                end_rad,
            } => {
                let sweep = end_rad - start_rad;
                // A mirror flips the turning direction, so the arc now runs CCW from
                // the image of its old end point.
                let start = if mirrored { angle_of(end_rad) } else { angle_of(start_rad) };
                ProfileSegment::Arc {
                    center: map(center),
                    radius,
                    start_rad: start,
                    end_rad: start + sweep,
                }
            }
        })
        .collect();
    ProfileLoop {
        points: l.points.iter().map(|&p| map(p)).collect(),
        segments,
    }
}

/// Revolves loops given as (radius, height) about +Z; nested loops become holes.
fn revolve_loops(store: &mut ShapeStore, loops: &[ProfileLoop], angle_deg: f64) -> Result<ShapeId, CoreError> {
    let depth_of = |i: usize| {
        loops
            .iter()
            .enumerate()
            .filter(|&(j, other)| j != i && point_in_polygon(loops[i].points[0], &other.points))
            .count()
    };
    let (holes, outers): (Vec<usize>, Vec<usize>) =
        (0..loops.len()).partition(|&i| depth_of(i) % 2 == 1);
    let mut body: Option<ShapeId> = None;
    for i in outers {
        let rev = store.revolve_profile(&loops[i].segments, Angle::deg(angle_deg))?;
        body = Some(match body {
            None => rev,
            Some(b) => {
                let fused = store.fuse(b, rev)?;
                let _ = store.remove(b);
                let _ = store.remove(rev);
                fused
            }
        });
    }
    let mut body = body.ok_or_else(|| {
        CoreError::Geometry(GeometryError::InvalidParameter {
            name: "sketch".to_string(),
            reason: "Sketch contains only hole loops and no outer boundary".to_string(),
        })
    })?;
    for i in holes {
        let tool = store.revolve_profile(&loops[i].segments, Angle::deg(angle_deg))?;
        let cut = store.cut(body, tool)?;
        let _ = store.remove(body);
        let _ = store.remove(tool);
        body = cut;
    }
    Ok(body)
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

/// Types and specifications of machined holes (M10).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum HoleType {
    /// Simple straight cylindrical hole.
    Simple,
    /// Counterbore hole with step diameter and depth.
    Counterbore {
        bore_radius: ScalarParam,
        bore_depth: ScalarParam,
    },
    /// Countersink hole with taper diameter and depth.
    Countersink {
        sink_radius: ScalarParam,
        sink_depth: ScalarParam,
    },
    /// Threaded / tapped hole with tap drill diameter and thread depth.
    Tap {
        thread_radius: ScalarParam,
        tap_drill_radius: ScalarParam,
        thread_depth: ScalarParam,
    },
}

impl Default for HoleType {
    fn default() -> Self {
        Self::Simple
    }
}

/// Parametric feature definition (Phase 2.2 & Phase 4.2).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeatureKind {
    /// Face-referenced parametric hole with machining specifications (M10).
    Hole {
        input: FeatureId,
        plane: SketchPlane,
        #[serde(default = "zero_scalar")]
        center_x: ScalarParam,
        #[serde(default = "zero_scalar")]
        center_y: ScalarParam,
        radius: ScalarParam,
        depth: ScalarParam,
        #[serde(default)]
        through_all: bool,
        #[serde(default)]
        hole_type: HoleType,
        #[serde(default)]
        tip_angle_deg: Option<ScalarParam>,
    },
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
        /// Ignore `depth` and go all the way through the target body (Join/Cut only).
        #[serde(default)]
        through_all: bool,
    },
    /// Revolves a sketch's closed loops around an axis lying in the sketch plane.
    SketchRevolve {
        sketch: Sketch,
        plane: SketchPlane,
        axis: SketchAxis,
        angle_deg: ScalarParam,
        mode: ExtrudeMode,
    },
    /// Solid loft between two closed sketches in the common XY sketch frame.
    /// The upper profile is placed `distance` millimeters along +Z.
    Loft {
        lower: Sketch,
        upper: Sketch,
        distance: ScalarParam,
    },
    /// Circular profile swept along one straight parametric path.
    SweepCircle { radius: ScalarParam, #[serde(default = "zero_scalar")] start_x: ScalarParam, #[serde(default = "zero_scalar")] start_y: ScalarParam, #[serde(default = "zero_scalar")] start_z: ScalarParam, dx: ScalarParam, dy: ScalarParam, dz: ScalarParam },
    /// Counterbore hole cut along +Z from a parametric XY position.
    Counterbore {
        input: FeatureId,
        #[serde(default = "zero_scalar")]
        center_x: ScalarParam,
        #[serde(default = "zero_scalar")]
        center_y: ScalarParam,
        hole_radius: ScalarParam,
        bore_radius: ScalarParam,
        bore_depth: ScalarParam,
        depth: ScalarParam,
    },
    /// Countersunk hole cut along +Z from a parametric XY position.
    Countersink {
        input: FeatureId,
        #[serde(default = "zero_scalar")]
        center_x: ScalarParam,
        #[serde(default = "zero_scalar")]
        center_y: ScalarParam,
        hole_radius: ScalarParam,
        sink_radius: ScalarParam,
        sink_depth: ScalarParam,
        depth: ScalarParam,
    },
    /// Repeats a feature. With `tool_only` and a sketch Join/Cut input, the input's tool
    /// is repeated on its target (e.g. a ring of holes); otherwise the input's body is
    /// repeated and fused. The choice is stored, not inferred, so re-wiring the input
    /// (e.g. when inserting a feature mid-history) never changes the meaning.
    Pattern {
        input: FeatureId,
        layout: PatternLayout,
        #[serde(default)]
        tool_only: bool,
    },
    /// Mirrors a feature across a plane: its tool onto its target (`tool_only`), or its
    /// body, fused with the original when `merge` is set.
    Mirror {
        input: FeatureId,
        plane: SketchPlane,
        merge: bool,
        #[serde(default)]
        tool_only: bool,
    },
    /// Hollows a body inward to a wall thickness, opening the given faces.
    Shell {
        input: FeatureId,
        thickness: ScalarParam,
        open_faces: Vec<SemanticReference>,
    },
    /// A fixed (non-parametric) body read from a STEP file. The file's text is stored in
    /// the document so the model keeps working if the original file moves or changes.
    ImportedBody {
        source_name: String,
        step_text: String,
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
        /// When set, applies a variable radius blend from `radius` (start) to `radius_end` (end).
        #[serde(default)]
        radius_end: Option<ScalarParam>,
        /// When true, automatically propagates the fillet along tangent continuous edges.
        #[serde(default)]
        propagate_tangent: bool,
    },
    Chamfer {
        input: FeatureId,
        distance: ScalarParam,
        edge_indices: Vec<u32>,
        #[serde(default)]
        semantic_refs: Vec<SemanticReference>,
    },
    /// Tilts selected planar faces around a planar neutral face.
    Draft {
        input: FeatureId,
        faces: Vec<SemanticReference>,
        neutral_face: SemanticReference,
        angle_deg: ScalarParam,
        #[serde(default)]
        reversed: bool,
    },
    /// OCCT Shape Healing applied as a non-destructive history feature.
    Heal { input: FeatureId },
    /// Removes same-domain splitter lines as a non-destructive history feature.
    Refine { input: FeatureId },
    /// Converts a single closed shell to a solid after import or sewing.
    SolidFromShell { input: FeatureId },
    /// Stitches coincident shell boundaries before solidification. Supports multiple inputs (M11).
    Sew {
        input: FeatureId,
        #[serde(default)]
        extra_inputs: Vec<FeatureId>,
        tolerance: ScalarParam,
    },
    /// Partitions `input` with `tool` or `plane`, retaining the split topology (M11).
    Split {
        input: FeatureId,
        #[serde(default)]
        tool: Option<FeatureId>,
        #[serde(default)]
        plane: Option<SketchPlane>,
    },
    /// Offsets a selected face along its normal (M11).
    SurfaceOffset {
        input: FeatureId,
        face: SemanticReference,
        distance: ScalarParam,
    },
    /// Trims a selected face to UV parameter bounds (M11).
    TrimSurface {
        input: FeatureId,
        face: SemanticReference,
        u_min: ScalarParam,
        u_max: ScalarParam,
        v_min: ScalarParam,
        v_max: ScalarParam,
    },
    /// Direct planar-face offset, retaining a semantic face reference.
    PushPull { input: FeatureId, face: SemanticReference, distance: ScalarParam },
    /// Moves a planar face along an arbitrary displacement vector.
    FaceMove {
        input: FeatureId,
        face: SemanticReference,
        dx: ScalarParam,
        dy: ScalarParam,
        dz: ScalarParam,
    },
    /// Sweeps a 2D closed sketch profile along a 3D polyline path.
    Sweep {
        profile: Sketch,
        path_points: Vec<[ScalarParam; 3]>,
    },
    /// Structural rib added from a sketch profile onto an existing body.
    Rib {
        input: FeatureId,
        sketch: Sketch,
        plane: SketchPlane,
        thickness: ScalarParam,
        depth: ScalarParam,
    },
    /// Array of construction planes generated from a base face by spacing and count.
    PlanePattern {
        input: FeatureId,
        base_face: SemanticReference,
        spacing: ScalarParam,
        count: u32,
        #[serde(default)]
        reverse: bool,
    },
    LinearPattern {
        input: FeatureId,
        spacing_x: ScalarParam,
        spacing_y: ScalarParam,
        spacing_z: ScalarParam,
        count_x: u32,
        count_y: u32,
    },
    /// Repeats a feature along a reference direction or around a reference axis (M10).
    PatternOnReference {
        input: FeatureId,
        reference: ReferenceAxis,
        layout: PatternReferenceLayout,
        #[serde(default)]
        tool_only: bool,
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
            | Self::RevolvePolygon { .. }
            | Self::Loft { .. }
            | Self::SweepCircle { .. }
            | Self::Sweep { .. }
            | Self::ImportedBody { .. } => Vec::new(),
            Self::Pattern { input, .. } | Self::Mirror { input, plane: SketchPlane::XY | SketchPlane::XZ | SketchPlane::YZ, .. } | Self::Shell { input, .. } => {
                vec![*input]
            }
            Self::Mirror {
                input,
                plane: SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. },
                ..
            } => {
                if input == feature {
                    vec![*input]
                } else {
                    vec![*input, *feature]
                }
            }
            Self::Mirror { input, plane: SketchPlane::MidFace { first_feature, second_feature, .. }, .. } => {
                let mut deps = vec![*input];
                for id in [*first_feature, *second_feature] { if !deps.contains(&id) { deps.push(id); } }
                deps
            }
            Self::SketchExtrude { plane, mode, .. } | Self::SketchRevolve { plane, mode, .. } => {
                let mut deps = Vec::new();
                for id in plane.features().into_iter().chain(mode.target()) {
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
            | Self::Draft { input, .. }
            | Self::Heal { input }
            | Self::Refine { input }
            | Self::SolidFromShell { input }
            | Self::PushPull { input, .. }
            | Self::FaceMove { input, .. }
            | Self::PlanePattern { input, .. }
            | Self::Counterbore { input, .. }
            | Self::Countersink { input, .. }
            | Self::SurfaceOffset { input, .. }
            | Self::TrimSurface { input, .. }
            | Self::LinearPattern { input, .. } => vec![*input],
            Self::Sew { input, extra_inputs, .. } => {
                let mut deps = vec![*input];
                for &extra in extra_inputs {
                    if !deps.contains(&extra) {
                        deps.push(extra);
                    }
                }
                deps
            }
            Self::Rib { input, plane, .. } | Self::Hole { input, plane, .. } => {
                let mut deps = vec![*input];
                for id in plane.features() {
                    if !deps.contains(&id) {
                        deps.push(id);
                    }
                }
                deps
            }
            Self::PatternOnReference { input, reference, .. } => {
                let mut deps = vec![*input];
                for id in reference.features() {
                    if !deps.contains(&id) {
                        deps.push(id);
                    }
                }
                deps
            }
            Self::Split { input, tool, plane } => {
                let mut deps = vec![*input];
                if let Some(t) = tool {
                    if !deps.contains(t) {
                        deps.push(*t);
                    }
                }
                if let Some(p) = plane {
                    for id in p.features() {
                        if !deps.contains(&id) {
                            deps.push(id);
                        }
                    }
                }
                deps
            }
        }
    }

    /// Replaces every reference to feature `old` with `new`. Returns true if anything changed.
    pub fn retarget(&mut self, old: FeatureId, new: FeatureId) -> bool {
        let mut changed = false;
        let mut swap = |id: &mut FeatureId| {
            if *id == old {
                *id = new;
                changed = true;
            }
        };
        match self {
            Self::Transform { input, .. }
            | Self::Fillet { input, .. }
            | Self::Chamfer { input, .. }
            | Self::Draft { input, .. }
            | Self::Heal { input }
            | Self::Refine { input }
            | Self::SolidFromShell { input }
            | Self::PushPull { input, .. }
            | Self::FaceMove { input, .. }
            | Self::PlanePattern { input, .. }
            | Self::Counterbore { input, .. }
            | Self::Countersink { input, .. }
            | Self::LinearPattern { input, .. }
            | Self::Pattern { input, .. }
            | Self::Shell { input, .. } => swap(input),
            Self::Sew { input, extra_inputs, .. } => {
                swap(input);
                for extra in extra_inputs {
                    swap(extra);
                }
            }
            Self::SurfaceOffset { input, face, .. } | Self::TrimSurface { input, face, .. } => {
                swap(input);
                face.retarget(old, new);
            }
            Self::Hole { input, plane, .. } | Self::Rib { input, plane, .. } => {
                swap(input);
                match plane {
                    SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. } => swap(feature),
                    SketchPlane::MidFace { first_feature, second_feature, .. } => {
                        swap(first_feature);
                        swap(second_feature);
                    }
                    _ => {}
                }
            }
            Self::PatternOnReference { input, reference, .. } => {
                swap(input);
                match reference {
                    ReferenceAxis::Edge { feature, .. } | ReferenceAxis::Face { feature, .. } => swap(feature),
                    ReferenceAxis::Plane(plane) => match plane {
                        SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. } => swap(feature),
                        SketchPlane::MidFace { first_feature, second_feature, .. } => {
                            swap(first_feature);
                            swap(second_feature);
                        }
                        _ => {}
                    }
                }
            }
            Self::Split { input, tool, plane } => {
                swap(input);
                if let Some(t) = tool {
                    swap(t);
                }
                if let Some(p) = plane {
                    match p {
                        SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. } => swap(feature),
                        SketchPlane::MidFace { first_feature, second_feature, .. } => {
                            swap(first_feature);
                            swap(second_feature);
                        }
                        _ => {}
                    }
                }
            }
            Self::Mirror { input, plane, .. } => {
                swap(input);
                match plane {
                    SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. } => swap(feature),
                    SketchPlane::MidFace { first_feature, second_feature, .. } => {
                        swap(first_feature);
                        swap(second_feature);
                    }
                    _ => {}
                }
            }
            Self::Boolean {
                primary, secondary, ..
            } => {
                swap(primary);
                swap(secondary);
            }
            Self::SketchExtrude { plane, mode, .. } | Self::SketchRevolve { plane, mode, .. } => {
                match plane {
                    SketchPlane::Face { feature, .. } | SketchPlane::OffsetFace { feature, .. } => swap(feature),
                    SketchPlane::MidFace { first_feature, second_feature, .. } => {
                        swap(first_feature);
                        swap(second_feature);
                    }
                    _ => {}
                }
                if let ExtrudeMode::Join { target } | ExtrudeMode::Cut { target } = mode {
                    swap(target);
                }
            }
            _ => {}
        }
        changed
    }

    /// The face/edge references this feature holds, each tied to the feature whose
    /// shape it points into.
    pub fn reference_slots(&self) -> Vec<RefSlot> {
        let mut out = Vec::new();
        match self {
            Self::Fillet {
                input,
                semantic_refs,
                ..
            }
            | Self::Chamfer {
                input,
                semantic_refs,
                ..
            } => {
                for (i, r) in semantic_refs.iter().enumerate() {
                    out.push(RefSlot {
                        role: RefRole::Edge(i),
                        upstream: *input,
                        reference: r.clone(),
                    });
                }
            }
            Self::Draft { input, faces, neutral_face, .. } => {
                for (i, r) in faces.iter().enumerate() {
                    out.push(RefSlot { role: RefRole::DraftFace(i), upstream: *input, reference: r.clone() });
                }
                out.push(RefSlot { role: RefRole::DraftNeutral, upstream: *input, reference: neutral_face.clone() });
            }
            Self::PushPull { input, face, .. } => out.push(RefSlot { role: RefRole::PushPullFace, upstream: *input, reference: face.clone() }),
            Self::FaceMove { input, face, .. } => out.push(RefSlot { role: RefRole::FaceMoveFace, upstream: *input, reference: face.clone() }),
            Self::PlanePattern { input, base_face, .. } => out.push(RefSlot { role: RefRole::PlanePatternFace, upstream: *input, reference: base_face.clone() }),
            Self::SurfaceOffset { input, face, .. } | Self::TrimSurface { input, face, .. } => out.push(RefSlot { role: RefRole::Plane, upstream: *input, reference: face.clone() }),
            Self::Split { plane: Some(plane), .. } => {
                match plane {
                    SketchPlane::Face { feature, face } | SketchPlane::OffsetFace { feature, face, .. } => {
                        out.push(RefSlot { role: RefRole::Plane, upstream: *feature, reference: face.clone() });
                    }
                    SketchPlane::MidFace { first_feature, first, second_feature, second } => {
                        out.push(RefSlot { role: RefRole::MidPlaneFirst, upstream: *first_feature, reference: first.clone() });
                        out.push(RefSlot { role: RefRole::MidPlaneSecond, upstream: *second_feature, reference: second.clone() });
                    }
                    _ => {}
                }
            }
            Self::Hole { plane, .. } | Self::Rib { plane, .. } => {
                match plane {
                    SketchPlane::Face { feature, face } | SketchPlane::OffsetFace { feature, face, .. } => {
                        out.push(RefSlot { role: RefRole::Plane, upstream: *feature, reference: face.clone() });
                    }
                    SketchPlane::MidFace { first_feature, first, second_feature, second } => {
                        out.push(RefSlot { role: RefRole::MidPlaneFirst, upstream: *first_feature, reference: first.clone() });
                        out.push(RefSlot { role: RefRole::MidPlaneSecond, upstream: *second_feature, reference: second.clone() });
                    }
                    _ => {}
                }
            }
            Self::PatternOnReference { reference, .. } => {
                match reference {
                    ReferenceAxis::Edge { feature, edge } => {
                        out.push(RefSlot { role: RefRole::PatternAxisRef, upstream: *feature, reference: edge.clone() });
                    }
                    ReferenceAxis::Face { feature, face } => {
                        out.push(RefSlot { role: RefRole::PatternAxisRef, upstream: *feature, reference: face.clone() });
                    }
                    ReferenceAxis::Plane(plane) => match plane {
                        SketchPlane::Face { feature, face } | SketchPlane::OffsetFace { feature, face, .. } => {
                            out.push(RefSlot { role: RefRole::Plane, upstream: *feature, reference: face.clone() });
                        }
                        SketchPlane::MidFace { first_feature, first, second_feature, second } => {
                            out.push(RefSlot { role: RefRole::MidPlaneFirst, upstream: *first_feature, reference: first.clone() });
                            out.push(RefSlot { role: RefRole::MidPlaneSecond, upstream: *second_feature, reference: second.clone() });
                        }
                        _ => {}
                    }
                }
            }
            Self::Shell {
                input, open_faces, ..
            } => {
                for (i, r) in open_faces.iter().enumerate() {
                    out.push(RefSlot {
                        role: RefRole::OpenFace(i),
                        upstream: *input,
                        reference: r.clone(),
                    });
                }
            }
            Self::SketchExtrude { plane, .. }
            | Self::SketchRevolve { plane, .. }
            | Self::Mirror { plane, .. } => {
                match plane {
                    SketchPlane::Face { feature, face } | SketchPlane::OffsetFace { feature, face, .. } => {
                        out.push(RefSlot { role: RefRole::Plane, upstream: *feature, reference: face.clone() });
                    }
                    SketchPlane::MidFace { first_feature, first, second_feature, second } => {
                        out.push(RefSlot { role: RefRole::MidPlaneFirst, upstream: *first_feature, reference: first.clone() });
                        out.push(RefSlot { role: RefRole::MidPlaneSecond, upstream: *second_feature, reference: second.clone() });
                    }
                    _ => {}
                }
            }
            _ => {}
        }
        out
    }

    /// Replaces the reference in `role` (re-binding it to a newly picked face/edge).
    pub fn replace_reference(&mut self, role: RefRole, reference: SemanticReference) -> bool {
        match (self, role) {
            (
                Self::Fillet { semantic_refs, .. } | Self::Chamfer { semantic_refs, .. },
                RefRole::Edge(i),
            ) => match semantic_refs.get_mut(i) {
                Some(slot) => {
                    *slot = reference;
                    true
                }
                None => false,
            },
            (Self::Shell { open_faces, .. }, RefRole::OpenFace(i)) => match open_faces.get_mut(i) {
                Some(slot) => {
                    *slot = reference;
                    true
                }
                None => false,
            },
            (Self::Draft { faces, .. }, RefRole::DraftFace(i)) => match faces.get_mut(i) {
                Some(slot) => { *slot = reference; true }
                None => false,
            },
            (Self::Draft { neutral_face, .. }, RefRole::DraftNeutral) => { *neutral_face = reference; true },
            (Self::PushPull { face, .. }, RefRole::PushPullFace) => { *face = reference; true },
            (Self::FaceMove { face, .. }, RefRole::FaceMoveFace) => { *face = reference; true },
            (Self::PlanePattern { base_face, .. }, RefRole::PlanePatternFace) => { *base_face = reference; true },
            (Self::SurfaceOffset { face, .. } | Self::TrimSurface { face, .. }, RefRole::Plane) => { *face = reference; true },
            (Self::PatternOnReference { reference: ref_axis, .. }, RefRole::PatternAxisRef) => match ref_axis {
                ReferenceAxis::Edge { edge, .. } => { *edge = reference; true }
                ReferenceAxis::Face { face, .. } => { *face = reference; true }
                _ => false,
            },
            (
                Self::SketchExtrude { plane, .. }
                | Self::SketchRevolve { plane, .. }
                | Self::Rib { plane, .. }
                | Self::Hole { plane, .. }
                | Self::PatternOnReference { reference: ReferenceAxis::Plane(plane), .. }
                | Self::Split { plane: Some(plane), .. }
                | Self::Mirror { plane, .. },
                RefRole::Plane,
            ) => match plane {
                SketchPlane::Face { face, .. } | SketchPlane::OffsetFace { face, .. } => {
                    *face = reference;
                    true
                }
                _ => false,
            },
            (
                Self::SketchExtrude { plane, .. }
                | Self::SketchRevolve { plane, .. }
                | Self::Hole { plane, .. }
                | Self::PatternOnReference { reference: ReferenceAxis::Plane(plane), .. }
                | Self::Mirror { plane, .. },
                RefRole::MidPlaneFirst,
            ) => match plane {
                SketchPlane::MidFace { first, .. } => { *first = reference; true }
                _ => false,
            },
            (
                Self::SketchExtrude { plane, .. }
                | Self::SketchRevolve { plane, .. }
                | Self::Hole { plane, .. }
                | Self::PatternOnReference { reference: ReferenceAxis::Plane(plane), .. }
                | Self::Mirror { plane, .. },
                RefRole::MidPlaneSecond,
            ) => match plane {
                SketchPlane::MidFace { second, .. } => { *second = reference; true }
                _ => false,
            },
            _ => false,
        }
    }

    /// The body this feature modifies, if any. A suppressed feature passes this
    /// input's shape through unchanged, as if the feature were not there.
    pub fn primary_input(&self) -> Option<FeatureId> {
        match self {
            Self::Transform { input, .. }
            | Self::Fillet { input, .. }
            | Self::Chamfer { input, .. }
            | Self::Draft { input, .. }
            | Self::Heal { input }
            | Self::Refine { input }
            | Self::SolidFromShell { input }
            | Self::Sew { input, .. }
            | Self::PushPull { input, .. }
            | Self::FaceMove { input, .. }
            | Self::Hole { input, .. }
            | Self::PatternOnReference { input, .. }
            | Self::Counterbore { input, .. }
            | Self::Countersink { input, .. }
            | Self::LinearPattern { input, .. } => Some(*input),
            Self::Split { input, .. } => Some(*input),
            Self::Boolean { primary, .. } => Some(*primary),
            Self::SketchExtrude { mode, .. } | Self::SketchRevolve { mode, .. } => mode.target(),
            Self::Pattern { input, .. } | Self::Mirror { input, .. } | Self::Shell { input, .. } => {
                Some(*input)
            }
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
            Self::Fillet { radius, radius_end, .. } => {
                add(radius);
                if let Some(r_end) = radius_end {
                    add(r_end);
                }
            }
            Self::Chamfer { distance, .. } => {
                add(distance);
            }
            Self::Draft { angle_deg, .. } => add(angle_deg),
            Self::Heal { .. } => {}
            Self::Refine { .. } => {}
            Self::SolidFromShell { .. } => {}
            Self::Sew { tolerance, .. } => add(tolerance),
            Self::Split { .. } => {}
            Self::SurfaceOffset { distance, .. } => add(distance),
            Self::TrimSurface { u_min, u_max, v_min, v_max, .. } => {
                add(u_min); add(u_max); add(v_min); add(v_max);
            }
            Self::PushPull { distance, .. } => add(distance),
            Self::FaceMove { dx, dy, dz, .. } => { add(dx); add(dy); add(dz); }
            Self::PlanePattern { spacing, .. } => add(spacing),
            Self::Rib { thickness, depth, .. } => { add(thickness); add(depth); }
            Self::Hole { center_x, center_y, radius, depth, hole_type, tip_angle_deg, .. } => {
                add(center_x);
                add(center_y);
                add(radius);
                add(depth);
                if let Some(angle) = tip_angle_deg {
                    add(angle);
                }
                match hole_type {
                    HoleType::Simple => {}
                    HoleType::Counterbore { bore_radius, bore_depth } => {
                        add(bore_radius);
                        add(bore_depth);
                    }
                    HoleType::Countersink { sink_radius, sink_depth } => {
                        add(sink_radius);
                        add(sink_depth);
                    }
                    HoleType::Tap { thread_radius, tap_drill_radius, thread_depth } => {
                        add(thread_radius);
                        add(tap_drill_radius);
                        add(thread_depth);
                    }
                }
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
            Self::SketchRevolve { angle_deg, .. } => add(angle_deg),
            Self::Loft { distance, .. } => add(distance),
            Self::SweepCircle { radius, start_x, start_y, start_z, dx, dy, dz } => { add(radius); add(start_x); add(start_y); add(start_z); add(dx); add(dy); add(dz); }
            Self::Sweep { path_points, .. } => {
                for pt in path_points {
                    add(&pt[0]);
                    add(&pt[1]);
                    add(&pt[2]);
                }
            }
            Self::Counterbore { center_x, center_y, hole_radius, bore_radius, bore_depth, depth, .. } => { add(center_x); add(center_y); add(hole_radius); add(bore_radius); add(bore_depth); add(depth); }
            Self::Countersink { center_x, center_y, hole_radius, sink_radius, sink_depth, depth, .. } => { add(center_x); add(center_y); add(hole_radius); add(sink_radius); add(sink_depth); add(depth); }
            Self::Pattern { layout, .. } => match layout {
                PatternLayout::Circular { total_angle_deg, .. } => add(total_angle_deg),
                PatternLayout::Linear { spacing, .. } => add(spacing),
            },
            Self::PatternOnReference { layout, .. } => match layout {
                PatternReferenceLayout::Linear { spacing, .. } => add(spacing),
                PatternReferenceLayout::Circular { total_angle_deg, .. } => add(total_angle_deg),
            },
            Self::Mirror { .. } | Self::ImportedBody { .. } => {}
            Self::Shell { thickness, .. } => add(thickness),
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
    /// Structured reason for the last `Failed` state, for user-facing explanations.
    #[serde(skip)]
    pub failure: Option<CoreError>,
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
    /// Removes a feature and everything downstream of it.
    RemoveFeatureCascade {
        id: FeatureId,
    },
    SetSuppressed {
        id: FeatureId,
        suppressed: bool,
    },
    SetRollback {
        marker: Option<FeatureId>,
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
    #[serde(default)]
    pub order: Vec<FeatureId>,
    #[serde(default)]
    pub rollback: Option<FeatureId>,
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
    /// Timeline order of features (creation order, plus features inserted while rolled
    /// back). Features missing from it are appended by id.
    pub order: Vec<FeatureId>,
    /// Rollback marker: features after it in the timeline are temporarily not evaluated,
    /// and new features are inserted right after it. `None` = at the end.
    pub rollback: Option<FeatureId>,
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
            order: Vec::new(),
            rollback: None,
        }
    }

    /// Features in timeline order.
    pub fn timeline(&self) -> Vec<FeatureId> {
        let mut out: Vec<FeatureId> = self
            .order
            .iter()
            .copied()
            .filter(|id| self.features.contains_key(id))
            .collect();
        for &id in self.features.keys() {
            if !out.contains(&id) {
                out.push(id);
            }
        }
        out
    }

    /// True if `id` sits after the rollback marker (temporarily not evaluated).
    pub fn is_rolled_back(&self, id: FeatureId) -> bool {
        let Some(marker) = self.rollback else {
            return false;
        };
        let tl = self.timeline();
        match (tl.iter().position(|&f| f == marker), tl.iter().position(|&f| f == id)) {
            (Some(m), Some(i)) => i > m,
            _ => false,
        }
    }

    /// Moves the rollback marker (`None` = roll forward to the end).
    pub fn set_rollback(&mut self, marker: Option<FeatureId>) -> Result<(), CoreError> {
        if let Some(m) = marker
            && !self.features.contains_key(&m)
        {
            return Err(CoreError::FeatureNotFound(m));
        }
        // Rolling to the last feature is the same as no rollback.
        let marker = marker.filter(|m| self.timeline().last() != Some(m));
        if self.rollback == marker {
            return Ok(());
        }
        self.rollback = marker;
        self.revision += 1;
        self.operation_history.push(CadCommand::SetRollback { marker });
        Ok(())
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

    /// Imports a STEP file using STEPCAF, creating an `ImportedBody` feature for each component
    /// with its embedded name, color, and unit-scaling applied.
    pub fn import_step_caf(
        &mut self,
        store: &mut ShapeStore,
        path: impl AsRef<Path>,
    ) -> Result<Vec<FeatureId>, CoreError> {
        let asm = store.import_step_caf(path)?;
        let mut feature_ids = Vec::with_capacity(asm.components.len());
        for comp in asm.components {
            let color = [comp.color[0] as f32, comp.color[1] as f32, comp.color[2] as f32, comp.color[3] as f32];
            let temp_path = std::env::temp_dir().join(format!("ferrocad_comp_{}.step", Uuid::new_v4()));
            store.export_step(comp.shape_id, &temp_path)?;
            let step_text = std::fs::read_to_string(&temp_path).unwrap_or_default();
            let _ = std::fs::remove_file(&temp_path);

            let fid = self.add_feature(
                comp.name.clone(),
                FeatureKind::ImportedBody {
                    source_name: comp.name,
                    step_text,
                },
                true,
                color,
            )?;
            feature_ids.push(fid);
        }
        Ok(feature_ids)
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
            failure: None,
        };
        let modified = kind.primary_input();
        self.features.insert(id, feature);
        let mut tl = self.timeline();
        match self.rollback {
            Some(marker) => {
                // Insert right after the marker, then move the marker onto the new
                // feature. Later features that built on the body this feature modifies
                // now build on the modified result instead.
                tl.retain(|&f| f != id);
                let pos = tl.iter().position(|&f| f == marker).map_or(tl.len(), |p| p + 1);
                let later: Vec<FeatureId> = tl[pos..].to_vec();
                tl.insert(pos, id);
                self.rollback = Some(id);
                if let Some(old) = modified {
                    for f in later {
                        let changed = self
                            .features
                            .get_mut(&f)
                            .is_some_and(|feat| feat.kind.retarget(old, id));
                        if changed {
                            self.mark_dirty(f);
                        }
                    }
                }
            }
            None => {
                if !tl.contains(&id) {
                    tl.push(id);
                }
            }
        }
        self.order = tl;
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

    /// Writes the given features' current shapes as one binary STL, tessellated with
    /// `options` (use finer settings than the viewport for printing). Returns the
    /// triangle count.
    pub fn export_stl(
        &self,
        store: &ShapeStore,
        ids: &[FeatureId],
        path: impl AsRef<Path>,
        options: TessellationOptions,
    ) -> Result<usize, CoreError> {
        let mut meshes = Vec::new();
        for id in ids {
            let shape = self.feature_shape(*id)?;
            meshes.push(store.tessellate(shape, options)?);
        }
        if meshes.is_empty() {
            return Err(CoreError::Serialization("nothing to export".to_string()));
        }
        let refs: Vec<&TriMesh> = meshes.iter().collect();
        cad_geometry::write_stl_binary(&refs, path).map_err(|e| CoreError::Serialization(e.to_string()))?;
        Ok(meshes.iter().map(|m| m.indices.len()).sum())
    }

    /// Checks whether a reference still resolves against its upstream shape right now.
    pub fn check_reference(&self, slot: &RefSlot, store: &ShapeStore) -> Result<(), ResolutionFailureReason> {
        let shape = self
            .features
            .get(&slot.upstream)
            .and_then(|f| f.cached_shape)
            .ok_or_else(|| {
                ResolutionFailureReason::MissingTargetShape(format!("feature {:?} has no shape", slot.upstream))
            })?;
        if slot.is_edge() {
            slot.reference.resolve_edge(store, shape).map(|_| ())
        } else {
            slot.reference.resolve_face(store, shape).map(|_| ())
        }
    }

    /// All features downstream of `id` (transitively), in timeline order.
    pub fn downstream_of(&self, id: FeatureId) -> Vec<FeatureId> {
        let mut found: HashSet<FeatureId> = HashSet::new();
        let mut queue = VecDeque::from([id]);
        while let Some(cur) = queue.pop_front() {
            for d in self.direct_dependents(cur) {
                if found.insert(d) {
                    queue.push_back(d);
                }
            }
        }
        self.timeline().into_iter().filter(|f| found.contains(f)).collect()
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
            SketchPlane::Face { feature, face } | SketchPlane::OffsetFace { feature, face, .. } => {
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
                let offset = match plane {
                    SketchPlane::OffsetFace { offset, .. } => offset.evaluate(&self.resolve_all_parameters()?)?,
                    _ => 0.0,
                };
                Ok(PlaneFrame::from_point_normal(
                    resolved.centroid + resolved.normal * offset,
                    resolved.normal,
                ))
            }
            SketchPlane::MidFace { first_feature, first, second_feature, second } => {
                let resolve = |feature: &FeatureId, reference: &SemanticReference| -> Result<_, CoreError> {
                    let shape = self.features.get(feature).and_then(|f| f.cached_shape).ok_or(CoreError::FeatureNotFound(*feature))?;
                    reference.resolve_face(store, shape).map_err(|reason| CoreError::SemanticReferenceLost { feature: *feature, reason })
                };
                let (a, b) = (resolve(first_feature, first)?, resolve(second_feature, second)?);
                if a.normal.dot(b.normal).abs() < 0.999 {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter { name: "MidPlane".to_string(), reason: "中間平面には平行な2面を選んでください".to_string() }));
                }
                Ok(PlaneFrame::from_point_normal((a.centroid + b.centroid) * 0.5, a.normal))
            }
        }
    }

    /// Resolves a reference axis to a 3D origin and unit direction vector (M10).
    pub fn resolve_reference_axis(
        &self,
        axis: &ReferenceAxis,
        store: &ShapeStore,
    ) -> Result<(DVec3, DVec3), CoreError> {
        match axis {
            ReferenceAxis::Edge { feature, edge } => {
                let shape = self.features.get(feature).and_then(|f| f.cached_shape).ok_or(CoreError::FeatureNotFound(*feature))?;
                let resolved = edge.resolve_edge(store, shape).map_err(|reason| CoreError::SemanticReferenceLost { feature: *feature, reason })?;
                let (origin, dir) = store.inspect_edge_axis(shape, resolved.index)?;
                let dir = dir.try_normalize().unwrap_or(DVec3::Z);
                Ok((origin, dir))
            }
            ReferenceAxis::Face { feature, face } => {
                let shape = self.features.get(feature).and_then(|f| f.cached_shape).ok_or(CoreError::FeatureNotFound(*feature))?;
                let resolved = face.resolve_face(store, shape).map_err(|reason| CoreError::SemanticReferenceLost { feature: *feature, reason })?;
                let (origin, dir) = store.inspect_face_axis(shape, resolved.index)?;
                let dir = dir.try_normalize().unwrap_or(DVec3::Z);
                Ok((origin, dir))
            }
            ReferenceAxis::Plane(plane) => {
                let frame = self.sketch_plane_frame(plane, store)?;
                Ok((frame.origin, frame.normal))
            }
        }
    }

    /// Removes a feature together with everything that depends on it (in one step).
    /// Returns the removed ids, dependents first.
    pub fn remove_feature_cascade(
        &mut self,
        id: FeatureId,
        mut store: Option<&mut ShapeStore>,
    ) -> Result<Vec<FeatureId>, CoreError> {
        if !self.features.contains_key(&id) {
            return Err(CoreError::FeatureNotFound(id));
        }
        // Later timeline entries can only depend on earlier ones, so removing in
        // reverse timeline order never trips the "still in use" check.
        let mut doomed = self.downstream_of(id);
        doomed.reverse();
        doomed.push(id);
        for &f in &doomed {
            self.remove_feature(f, store.as_deref_mut())?;
            self.operation_history.pop();
        }
        self.operation_history
            .push(CadCommand::RemoveFeatureCascade { id });
        Ok(doomed)
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

        if self.rollback == Some(id) {
            let tl = self.timeline();
            let pos = tl.iter().position(|&f| f == id).unwrap_or(0);
            self.rollback = pos.checked_sub(1).map(|p| tl[p]);
        }
        self.order.retain(|&f| f != id);
        let removed = self.features.remove(&id).unwrap();
        if let (Some(shape_store), Some(shape_id)) = (store, removed.cached_shape) {
            let _ = shape_store.remove(shape_id);
        }
        // A body hidden because this feature replaced it on screen (join/cut, fillet,
        // pattern, ...) becomes the visible result again once nothing else uses it.
        let mut inputs: Vec<FeatureId> = removed.kind.primary_input().into_iter().collect();
        if let FeatureKind::Boolean { secondary, .. } = removed.kind {
            inputs.push(secondary);
        }
        for input in inputs {
            if self.direct_dependents(input).is_empty()
                && let Some(f) = self.features.get_mut(&input)
            {
                f.visible = true;
            }
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

        let rolled_back: HashSet<FeatureId> = match self.rollback {
            Some(marker) => {
                let tl = self.timeline();
                let pos = tl.iter().position(|&f| f == marker).unwrap_or(tl.len());
                tl[pos + 1..].iter().copied().collect()
            }
            None => HashSet::new(),
        };

        for &fid in &order {
            let feat_snapshot = self.features.get(&fid).cloned().unwrap();

            // Past the rollback marker: not evaluated and not shown until rolled forward.
            if rolled_back.contains(&fid) {
                let feat = self.features.get_mut(&fid).unwrap();
                if let Some(old_id) = feat.cached_shape.take() {
                    let _ = store.remove(old_id);
                }
                feat.cached_mesh = None;
                feat.cached_summary = None;
                // The renderer falls back to the last valid mesh (meant for failures);
                // rolled-back features must not be drawn at all.
                feat.last_valid_mesh = None;
                feat.last_valid_summary = None;
                feat.failure = None;
                feat.state = FeatureState::Dirty;
                continue;
            }

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
                            feat.failure = None;
                            feat.state = FeatureState::Clean;
                            report.recomputed_features.push(fid);
                        }
                        Err(err) => {
                            let _ = store.remove(new_shape_id);
                            let msg = err.to_string();
                            let feat = self.features.get_mut(&fid).unwrap();
                            feat.failure = Some(CoreError::Geometry(err));
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
                    feat.failure = Some(err);
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

    fn feature_shape(&self, id: FeatureId) -> Result<ShapeId, CoreError> {
        self.features
            .get(&id)
            .and_then(|f| f.cached_shape)
            .ok_or(CoreError::FeatureNotFound(id))
    }

    /// True for features whose cutting/joining tool can be repeated onto a target.
    fn is_tool_feature(&self, id: FeatureId) -> bool {
        self.features.get(&id).is_some_and(|f| match &f.kind {
            FeatureKind::SketchExtrude { mode, .. } | FeatureKind::SketchRevolve { mode, .. } => {
                *mode != ExtrudeMode::NewBody
            }
            FeatureKind::Hole { .. } => true,
            _ => false,
        })
    }

    fn hole_tool(
        &self,
        target_id: FeatureId,
        plane: &SketchPlane,
        cx: f64,
        cy: f64,
        r: f64,
        d: f64,
        through_all: bool,
        hole_type: &HoleType,
        tip_angle_deg: Option<&ScalarParam>,
        params: &BTreeMap<String, f64>,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        let src = self.feature_shape(target_id)?;
        if r <= 0.0 {
            return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                name: "Hole".to_string(),
                reason: "穴半径は正の数値である必要があります".to_string(),
            }));
        }
        if !through_all && d <= 0.0 {
            return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                name: "Hole".to_string(),
                reason: "穴深さは正の数値である必要があります".to_string(),
            }));
        }

        let frame = self.sketch_plane_frame(plane, store)?;
        let bbox = store.bounding_box(src)?;
        let diag = bbox.diagonal_length();
        let actual_depth = if through_all {
            (diag + 20.0).max(d).max(50.0)
        } else {
            d
        };

        let margin = 1.0;
        let bottom_z = -actual_depth - if through_all { margin } else { 0.0 };
        let total_length = margin - bottom_z;

        let main_tool = match hole_type {
            HoleType::Simple => {
                let cyl = store.create_cylinder(Length::mm(r), Length::mm(total_length))?;
                let cyl = store.translate(cyl, 0.0, 0.0, bottom_z)?;
                store.translate(cyl, cx, cy, 0.0)?
            }
            HoleType::Counterbore { bore_radius, bore_depth } => {
                let br = bore_radius.evaluate(params)?;
                let bd = bore_depth.evaluate(params)?;
                if br < r || bd <= 0.0 || (!through_all && bd > d) {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Hole".to_string(),
                        reason: "座ぐり径は穴径以上、座ぐり深さは全深さ以下にしてください".to_string(),
                    }));
                }
                let bore_len = bd + margin;
                let bore = store.create_cylinder(Length::mm(br), Length::mm(bore_len))?;
                let bore = store.translate(bore, 0.0, 0.0, -bd)?;
                let shaft_len = (-bd - bottom_z).max(0.1);
                let shaft = store.create_cylinder(Length::mm(r), Length::mm(shaft_len))?;
                let shaft = store.translate(shaft, 0.0, 0.0, bottom_z)?;
                let combined = store.fuse(bore, shaft)?;
                store.translate(combined, cx, cy, 0.0)?
            }
            HoleType::Countersink { sink_radius, sink_depth } => {
                let sr = sink_radius.evaluate(params)?;
                let sd = sink_depth.evaluate(params)?;
                if sr < r || sd <= 0.0 || (!through_all && sd > d) {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Hole".to_string(),
                        reason: "皿穴径は穴径以上、皿深さは全深さ以下にしてください".to_string(),
                    }));
                }
                let sink_len = sd + margin;
                let cone = store.create_cone(Length::mm(r), Length::mm(sr), Length::mm(sink_len))?;
                let cone = store.translate(cone, 0.0, 0.0, -sd)?;
                let shaft_len = (-sd - bottom_z).max(0.1);
                let shaft = store.create_cylinder(Length::mm(r), Length::mm(shaft_len))?;
                let shaft = store.translate(shaft, 0.0, 0.0, bottom_z)?;
                let combined = store.fuse(cone, shaft)?;
                store.translate(combined, cx, cy, 0.0)?
            }
            HoleType::Tap { thread_radius, tap_drill_radius, thread_depth } => {
                let tr = thread_radius.evaluate(params)?;
                let tdr = tap_drill_radius.evaluate(params)?;
                let td = thread_depth.evaluate(params)?;
                if tr < tdr || td <= 0.0 || (!through_all && td > d) {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Hole".to_string(),
                        reason: "ねじ呼び径は下穴径以上、ねじ深さは全深さ以下にしてください".to_string(),
                    }));
                }
                let thread_len = td + margin;
                let thread_cyl = store.create_cylinder(Length::mm(tr), Length::mm(thread_len))?;
                let thread_cyl = store.translate(thread_cyl, 0.0, 0.0, -td)?;
                let drill_len = (-td - bottom_z).max(0.1);
                let drill_cyl = store.create_cylinder(Length::mm(tdr), Length::mm(drill_len))?;
                let drill_cyl = store.translate(drill_cyl, 0.0, 0.0, bottom_z)?;
                let combined = store.fuse(thread_cyl, drill_cyl)?;
                store.translate(combined, cx, cy, 0.0)?
            }
        };

        let full_tool = if !through_all {
            if let Some(tip_deg_param) = tip_angle_deg {
                let tip_deg = tip_deg_param.evaluate(params)?;
                if tip_deg > 10.0 && tip_deg < 179.0 {
                    let half_angle_rad = (tip_deg * 0.5).to_radians();
                    let tip_height = r / half_angle_rad.tan();
                    let tip_cone = store.create_cone(Length::mm(0.001), Length::mm(r), Length::mm(tip_height))?;
                    let tip_cone = store.translate(tip_cone, 0.0, 0.0, -actual_depth - tip_height)?;
                    let tip_cone = store.translate(tip_cone, cx, cy, 0.0)?;
                    store.fuse(main_tool, tip_cone)?
                } else {
                    main_tool
                }
            } else {
                main_tool
            }
        } else {
            main_tool
        };

        place_on_frame(store, full_tool, &frame, 0.0)
    }

    /// The solid tool a feature produces before it is combined with its target.
    fn feature_tool(
        &self,
        kind: &FeatureKind,
        params: &BTreeMap<String, f64>,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        match kind {
            FeatureKind::SketchExtrude { .. } | FeatureKind::SketchRevolve { .. } => {
                self.sketch_tool(kind, params, store)
            }
            FeatureKind::Hole {
                input,
                plane,
                center_x,
                center_y,
                radius,
                depth,
                through_all,
                hole_type,
                tip_angle_deg,
            } => {
                let cx = center_x.evaluate(params)?;
                let cy = center_y.evaluate(params)?;
                let r = radius.evaluate(params)?;
                let d = depth.evaluate(params)?;
                self.hole_tool(
                    *input,
                    plane,
                    cx,
                    cy,
                    r,
                    d,
                    *through_all,
                    hole_type,
                    tip_angle_deg.as_ref(),
                    params,
                    store,
                )
            }
            _ => Err(CoreError::Geometry(GeometryError::InvalidParameter {
                name: "feature".to_string(),
                reason: "not a tool feature".to_string(),
            })),
        }
    }

    /// The solid a sketch feature produces before it is combined with its target: the
    /// placed extrusion or revolution of the sketch's loops.
    fn sketch_tool(
        &self,
        kind: &FeatureKind,
        params: &BTreeMap<String, f64>,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        match kind {
            FeatureKind::SketchExtrude {
                sketch,
                depth,
                plane,
                mode,
                reversed,
                symmetric,
                through_all,
            } => {
                let (_, loops) = solved_loops(sketch)?;
                let mut d = depth.evaluate(params)?;
                if *through_all && let Some(target) = mode.target() {
                    let bbox = store.bounding_box(self.feature_shape(target)?)?;
                    d = bbox.diagonal_length() * 2.0 + 1.0;
                }
                let frame = self.sketch_plane_frame(plane, store)?;
                let prism = extrude_loops(store, &loops, d)?;
                let z_shift = if *symmetric {
                    -d * 0.5
                } else if *reversed {
                    -d
                } else {
                    0.0
                };
                place_on_frame(store, prism, &frame, z_shift)
            }
            FeatureKind::SketchRevolve {
                sketch,
                plane,
                axis,
                angle_deg,
                ..
            } => {
                let (solved, loops) = solved_loops(sketch)?;
                let pos = |p: &PointId| -> Result<DVec3, CoreError> {
                    solved
                        .points
                        .get(p)
                        .map(|q| DVec3::new(q.x, q.y, 0.0))
                        .ok_or_else(|| {
                            CoreError::Geometry(GeometryError::InvalidParameter {
                                name: solved.name.clone(),
                                reason: "Revolve axis refers to a point that no longer exists".to_string(),
                            })
                        })
                };
                let (a, dir) = match axis {
                    SketchAxis::X => (DVec3::ZERO, DVec3::X),
                    SketchAxis::Y => (DVec3::ZERO, DVec3::Y),
                    SketchAxis::Line { a, b } => {
                        let (pa, pb) = (pos(a)?, pos(b)?);
                        (pa, (pb - pa).normalize_or_zero())
                    }
                };
                if dir == DVec3::ZERO {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: solved.name.clone(),
                        reason: "Revolve axis has zero length".to_string(),
                    }));
                }
                // Express each loop as (distance from axis, position along axis).
                let mut side = DVec3::new(dir.y, -dir.x, 0.0);
                let (neg, posi) = loops.iter().flat_map(|l| l.points.iter()).fold((false, false), |(n, p), q| {
                    let r = (DVec3::new(q[0], q[1], 0.0) - a).dot(side);
                    (n || r < -1e-7, p || r > 1e-7)
                });
                if neg && posi {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: solved.name.clone(),
                        reason: "The profile crosses the revolve axis; keep it on one side".to_string(),
                    }));
                }
                if neg {
                    side = -side;
                }
                let rz: Vec<ProfileLoop> = loops.iter().map(|l| map_profile_loop(l, a, side, dir)).collect();
                let solid = revolve_loops(store, &rz, angle_deg.evaluate(params)?)?;
                let frame = self.sketch_plane_frame(plane, store)?;
                let to3 = |v: DVec3| frame.x_axis * v.x + frame.y_axis * v.y;
                let (x3, z3) = (to3(side), to3(dir));
                let placement = PlaneFrame {
                    origin: frame.to_world(a.x, a.y),
                    x_axis: x3,
                    y_axis: z3.cross(x3),
                    normal: z3,
                };
                place_on_frame(store, solid, &placement, 0.0)
            }
            _ => Err(CoreError::Geometry(GeometryError::InvalidParameter {
                name: "feature".to_string(),
                reason: "not a sketch feature".to_string(),
            })),
        }
    }

    fn combine_with_target(
        &self,
        tool: ShapeId,
        mode: ExtrudeMode,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        match mode {
            ExtrudeMode::NewBody => Ok(tool),
            ExtrudeMode::Join { target } | ExtrudeMode::Cut { target } => {
                let base = self.feature_shape(target)?;
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

    /// Applies `instances` of a feature. With `use_tool` (a sketch Join/Cut input) the
    /// tool is placed at every instance on the input's target; otherwise the input's body
    /// is placed at every instance and the copies are fused.
    fn replicate(
        &self,
        input: FeatureId,
        instances: &[Instance],
        use_tool: bool,
        params: &BTreeMap<String, f64>,
        store: &mut ShapeStore,
    ) -> Result<ShapeId, CoreError> {
        let feat = self.features.get(&input).ok_or(CoreError::FeatureNotFound(input))?;
        if use_tool {
            let (target_id, is_cut) = match &feat.kind {
                FeatureKind::SketchExtrude { mode, .. } | FeatureKind::SketchRevolve { mode, .. } => {
                    (mode.target().unwrap(), matches!(mode, ExtrudeMode::Cut { .. }))
                }
                FeatureKind::Hole { input: target, .. } => (*target, true),
                _ => unreachable!(),
            };
            let tool = self.feature_tool(&feat.kind, params, store)?;
            let mut current = self.feature_shape(target_id)?;
            let mut owned = false;
            for inst in instances {
                let placed = inst.apply(store, tool)?;
                let next = if is_cut {
                    store.cut(current, placed)
                } else {
                    store.fuse(current, placed)
                };
                let _ = store.remove(placed);
                if owned {
                    let _ = store.remove(current);
                }
                current = next?;
                owned = true;
            }
            let _ = store.remove(tool);
            if !owned {
                current = store.clone_shape(current)?;
            }
            return Ok(current);
        }
        let src = self.feature_shape(input)?;
        let mut acc: Option<ShapeId> = None;
        for inst in instances {
            let placed = inst.apply(store, src)?;
            acc = Some(match acc {
                None => placed,
                Some(prev) => {
                    let fused = store.fuse(prev, placed)?;
                    let _ = store.remove(prev);
                    let _ = store.remove(placed);
                    fused
                }
            });
        }
        acc.ok_or_else(|| {
            CoreError::Geometry(GeometryError::InvalidParameter {
                name: "pattern".to_string(),
                reason: "no instances".to_string(),
            })
        })
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
            FeatureKind::SketchExtrude { mode, .. } | FeatureKind::SketchRevolve { mode, .. } => {
                let tool = self.sketch_tool(kind, params, store)?;
                self.combine_with_target(tool, *mode, store)
            }
            FeatureKind::Loft { lower, upper, distance } => {
                let (_, lower_loops) = solved_loops(lower)?;
                let (_, upper_loops) = solved_loops(upper)?;
                if lower_loops.len() != 1 || upper_loops.len() != 1 {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Loft".to_string(),
                        reason: "初期版のロフトは、各断面が1つの閉じた輪郭である必要があります".to_string(),
                    }));
                }
                Ok(store.loft_profiles(
                    &lower_loops[0].segments,
                    &upper_loops[0].segments,
                    distance.evaluate(params)?,
                )?)
            }
            FeatureKind::SweepCircle { radius, start_x, start_y, start_z, dx, dy, dz } => {
                let shape = store.sweep_circle(Length::mm(radius.evaluate(params)?), DVec3::new(dx.evaluate(params)?, dy.evaluate(params)?, dz.evaluate(params)?))?;
                Ok(store.translate(shape, start_x.evaluate(params)?, start_y.evaluate(params)?, start_z.evaluate(params)?)?)
            },
            FeatureKind::Hole {
                input,
                plane,
                center_x,
                center_y,
                radius,
                depth,
                through_all,
                hole_type,
                tip_angle_deg,
            } => {
                let src = lookup_shape(*input)?;
                let cx = center_x.evaluate(params)?;
                let cy = center_y.evaluate(params)?;
                let r = radius.evaluate(params)?;
                let d = depth.evaluate(params)?;
                let placed_tool = self.hole_tool(
                    *input,
                    plane,
                    cx,
                    cy,
                    r,
                    d,
                    *through_all,
                    hole_type,
                    tip_angle_deg.as_ref(),
                    params,
                    store,
                )?;
                let cut = store.cut(src, placed_tool)?;
                let _ = store.remove(placed_tool);
                Ok(cut)
            }
            FeatureKind::Counterbore { input, center_x, center_y, hole_radius, bore_radius, bore_depth, depth } => {
                let src = lookup_shape(*input)?;
                let center_x = center_x.evaluate(params)?;
                let center_y = center_y.evaluate(params)?;
                let hole_radius = hole_radius.evaluate(params)?;
                let bore_radius = bore_radius.evaluate(params)?;
                let bore_depth = bore_depth.evaluate(params)?;
                let depth = depth.evaluate(params)?;
                if bore_radius < hole_radius || bore_depth <= 0.0 || bore_depth > depth {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter { name: "Counterbore".to_string(), reason: "座ぐり径は穴径以上、座ぐり深さは全深さ以下にしてください".to_string() }));
                }
                let through = store.create_cylinder(Length::mm(hole_radius), Length::mm(depth))?;
                let through = store.translate(through, center_x, center_y, 0.0)?;
                let bore = store.create_cylinder(Length::mm(bore_radius), Length::mm(bore_depth))?;
                let bore = store.translate(bore, center_x, center_y, 0.0)?;
                let tool = store.fuse(through, bore)?;
                Ok(store.cut(src, tool)?)
            }
            FeatureKind::Countersink { input, center_x, center_y, hole_radius, sink_radius, sink_depth, depth } => {
                let src = lookup_shape(*input)?;
                let center_x = center_x.evaluate(params)?;
                let center_y = center_y.evaluate(params)?;
                let hole_radius = hole_radius.evaluate(params)?;
                let sink_radius = sink_radius.evaluate(params)?;
                let sink_depth = sink_depth.evaluate(params)?;
                let depth = depth.evaluate(params)?;
                if sink_radius < hole_radius || sink_depth <= 0.0 || sink_depth > depth {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter { name: "Countersink".to_string(), reason: "皿穴径は穴径以上、皿深さは全深さ以下にしてください".to_string() }));
                }
                let cone = store.create_cone(Length::mm(sink_radius), Length::mm(hole_radius), Length::mm(sink_depth))?;
                let cone = store.translate(cone, center_x, center_y, 0.0)?;
                let shaft = store.create_cylinder(Length::mm(hole_radius), Length::mm(depth - sink_depth))?;
                let shaft = store.translate(shaft, center_x, center_y, sink_depth)?;
                let tool = store.fuse(cone, shaft)?;
                Ok(store.cut(src, tool)?)
            }
            FeatureKind::Pattern {
                input,
                layout,
                tool_only,
            } => {
                let instances = layout.instances(params)?;
                let tool = *tool_only && self.is_tool_feature(*input);
                self.replicate(*input, &instances, tool, params, store)
            }
            FeatureKind::PatternOnReference {
                input,
                reference,
                layout,
                tool_only,
            } => {
                let (origin, dir) = self.resolve_reference_axis(reference, store)?;
                let instances: Vec<Instance> = match layout {
                    PatternReferenceLayout::Linear { spacing, count } => {
                        if *count < 1 {
                            return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                                name: "PatternOnReference".to_string(),
                                reason: "個数は1以上を指定してください".to_string(),
                            }));
                        }
                        let s = spacing.evaluate(params)?;
                        (0..*count)
                            .map(|i| match i {
                                0 => Instance::Identity,
                                _ => Instance::Translate(dir * (s * i as f64)),
                            })
                            .collect()
                    }
                    PatternReferenceLayout::Circular { count, total_angle_deg } => {
                        if *count < 1 {
                            return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                                name: "PatternOnReference".to_string(),
                                reason: "個数は1以上を指定してください".to_string(),
                            }));
                        }
                        let total = total_angle_deg.evaluate(params)?;
                        let step = if (total.abs() - 360.0).abs() < 1e-6 || *count == 1 {
                            total / *count as f64
                        } else {
                            total / (*count - 1) as f64
                        };
                        (0..*count)
                            .map(|i| match i {
                                0 => Instance::Identity,
                                _ => Instance::Rotate {
                                    origin,
                                    axis: dir,
                                    angle_deg: step * i as f64,
                                },
                            })
                            .collect()
                    }
                };
                let tool = *tool_only && self.is_tool_feature(*input);
                self.replicate(*input, &instances, tool, params, store)
            }
            FeatureKind::Mirror {
                input,
                plane,
                merge,
                tool_only,
            } => {
                let frame = self.sketch_plane_frame(plane, store)?;
                let mirror = Instance::Mirror {
                    origin: frame.origin,
                    normal: frame.normal,
                };
                let tool = *tool_only && self.is_tool_feature(*input);
                let instances = if *merge || tool {
                    vec![Instance::Identity, mirror]
                } else {
                    vec![mirror]
                };
                self.replicate(*input, &instances, tool, params, store)
            }
            FeatureKind::ImportedBody { step_text, .. } => {
                // OCCT's STEP reader works on files, so round-trip through a temp file.
                let path = std::env::temp_dir().join(format!("ferrocad_import_{}.step", Uuid::new_v4()));
                std::fs::write(&path, step_text).map_err(|e| CoreError::Serialization(e.to_string()))?;
                let imported = store.import_step(&path);
                let _ = std::fs::remove_file(&path);
                Ok(imported?)
            }
            FeatureKind::Shell {
                input,
                thickness,
                open_faces,
            } => {
                let src = lookup_shape(*input)?;
                let t = thickness.evaluate(params)?;
                let mut indices = Vec::with_capacity(open_faces.len());
                for face in open_faces {
                    let resolved = face.resolve_face(store, src).map_err(|reason| {
                        CoreError::SemanticReferenceLost {
                            feature: *input,
                            reason,
                        }
                    })?;
                    indices.push(resolved.index);
                }
                Ok(store.thick_solid(src, &indices, Length::mm(t))?)
            }
            FeatureKind::Draft { input, faces, neutral_face, angle_deg, reversed } => {
                let src = lookup_shape(*input)?;
                let mut indices = Vec::with_capacity(faces.len());
                for face in faces {
                    let resolved = face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost {
                        feature: *input, reason,
                    })?;
                    indices.push(resolved.index);
                }
                let neutral = neutral_face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost {
                    feature: *input, reason,
                })?;
                Ok(store.draft_faces(src, &indices, neutral.index, angle_deg.evaluate(params)?, *reversed)?)
            }
            FeatureKind::Heal { input } => Ok(store.heal_shape(lookup_shape(*input)?)?),
            FeatureKind::Refine { input } => Ok(store.refine_shape(lookup_shape(*input)?)?),
            FeatureKind::SolidFromShell { input } => Ok(store.solid_from_shell(lookup_shape(*input)?)?),
            FeatureKind::Sew { input, extra_inputs, tolerance } => {
                let tol = tolerance.evaluate(params)?;
                if extra_inputs.is_empty() {
                    Ok(store.sew_shape(lookup_shape(*input)?, tol)?)
                } else {
                    let mut shape_ids = vec![lookup_shape(*input)?];
                    for extra in extra_inputs {
                        shape_ids.push(lookup_shape(*extra)?);
                    }
                    Ok(store.sew_multiple_shapes(&shape_ids, tol)?)
                }
            }
            FeatureKind::Split { input, tool, plane } => {
                let src = lookup_shape(*input)?;
                if let Some(tool_id) = tool {
                    Ok(store.split(src, lookup_shape(*tool_id)?)?)
                } else if let Some(p) = plane {
                    let frame = self.sketch_plane_frame(p, store)?;
                    Ok(store.split_by_plane(src, frame.origin, frame.normal)?)
                } else {
                    Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Split".to_string(),
                        reason: "分割には工具形状または平面が必要です".to_string(),
                    }))
                }
            }
            FeatureKind::SurfaceOffset { input, face, distance } => {
                let src = lookup_shape(*input)?;
                let resolved = face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost { feature: *input, reason })?;
                Ok(store.offset_surface(src, resolved.index, distance.evaluate(params)?)?)
            }
            FeatureKind::TrimSurface { input, face, u_min, u_max, v_min, v_max } => {
                let src = lookup_shape(*input)?;
                let resolved = face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost { feature: *input, reason })?;
                Ok(store.trim_surface(
                    src,
                    resolved.index,
                    u_min.evaluate(params)?,
                    u_max.evaluate(params)?,
                    v_min.evaluate(params)?,
                    v_max.evaluate(params)?,
                )?)
            }
            FeatureKind::PushPull { input, face, distance } => {
                let src = lookup_shape(*input)?;
                let resolved = face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost { feature: *input, reason })?;
                Ok(store.push_pull_face(src, resolved.index, distance.evaluate(params)?)?)
            }
            FeatureKind::FaceMove { input, face, dx, dy, dz } => {
                let src = lookup_shape(*input)?;
                let resolved = face.resolve_face(store, src).map_err(|reason| CoreError::SemanticReferenceLost { feature: *input, reason })?;
                let disp = DVec3::new(dx.evaluate(params)?, dy.evaluate(params)?, dz.evaluate(params)?);
                Ok(store.face_move(src, resolved.index, disp)?)
            }
            FeatureKind::Sweep { profile, path_points } => {
                let (_, loops) = solved_loops(profile)?;
                if loops.is_empty() {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Sweep".to_string(),
                        reason: "スイープには1つ以上の閉じたスケッチ輪郭が必要です".to_string(),
                    }));
                }
                let mut path_coords = Vec::with_capacity(path_points.len() * 3);
                for pt in path_points {
                    path_coords.push(pt[0].evaluate(params)?);
                    path_coords.push(pt[1].evaluate(params)?);
                    path_coords.push(pt[2].evaluate(params)?);
                }
                Ok(store.sweep(&loops[0].segments, &path_coords)?)
            }
            FeatureKind::Rib { input, sketch, plane, thickness, depth } => {
                let src = lookup_shape(*input)?;
                let frame = self.sketch_plane_frame(plane, store)?;
                let (solved_sketch, loops) = solved_loops(sketch)?;
                let t = thickness.evaluate(params)?;
                let d = depth.evaluate(params)?;
                let rib_shape = if !loops.is_empty() {
                    let slab = store.extrude_profile(&loops[0].segments, Length::mm(d))?;
                    place_on_frame(store, slab, &frame, 0.0)?
                } else if solved_sketch.points.len() >= 2 {
                    let pts: Vec<_> = solved_sketch.points.values().collect();
                    let len = ((pts[1].x - pts[0].x).powi(2) + (pts[1].y - pts[0].y).powi(2)).sqrt();
                    let box_shape = store.create_box(Length::mm(len), Length::mm(t), Length::mm(d))?;
                    place_on_frame(store, box_shape, &frame, 0.0)?
                } else {
                    return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                        name: "Rib".to_string(),
                        reason: "リブには閉じた輪郭または2点以上のスケッチ要素が必要です".to_string(),
                    }));
                };
                let fused = store.fuse(src, rib_shape)?;
                let _ = store.remove(rib_shape);
                Ok(fused)
            }
            FeatureKind::PlanePattern { input, .. } => {
                let src = lookup_shape(*input)?;
                Ok(src)
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
                radius_end,
                propagate_tangent,
            } => {
                let src = lookup_shape(*input)?;
                let r = radius.evaluate(params)?;
                let target_edges = if !semantic_refs.is_empty() {
                    self.resolve_semantic_edges(*input, semantic_refs, store)?
                } else if edge_indices.is_empty() {
                    Vec::new()
                } else {
                    edge_indices.clone()
                };

                let final_edges = if *propagate_tangent && !target_edges.is_empty() {
                    let mut expanded = std::collections::BTreeSet::new();
                    for &e in &target_edges {
                        expanded.insert(e);
                        if let Ok(tangents) = store.find_tangent_edges(src, e, 0.087) {
                            for t in tangents {
                                expanded.insert(t);
                            }
                        }
                    }
                    expanded.into_iter().collect::<Vec<_>>()
                } else {
                    target_edges
                };

                if let Some(r_end_param) = radius_end {
                    let r2 = r_end_param.evaluate(params)?;
                    if final_edges.is_empty() {
                        return Err(CoreError::Geometry(GeometryError::InvalidParameter {
                            name: "Fillet".to_string(),
                            reason: "可変フィレットには対象辺の指定が必要です".to_string(),
                        }));
                    }
                    Ok(store.fillet_variable_edges(src, &final_edges, Length::mm(r), Length::mm(r2))?)
                } else if final_edges.is_empty() {
                    Ok(store.fillet_all(src, Length::mm(r))?)
                } else {
                    Ok(store.fillet_edges(src, &final_edges, Length::mm(r))?)
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
            order: self.order.clone(),
            rollback: self.rollback,
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
            order: snapshot.order,
            rollback: snapshot.rollback,
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
            CadCommand::RemoveFeatureCascade { id } => {
                self.document.remove_feature_cascade(id, store)?;
                Ok(None)
            }
            CadCommand::SetSuppressed { id, suppressed } => {
                self.document.set_suppressed(id, suppressed)?;
                Ok(Some(id))
            }
            CadCommand::SetRollback { marker } => {
                self.document.set_rollback(marker)?;
                Ok(None)
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
                        radius_end: None,
                        propagate_tangent: false,
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
                    through_all: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);

        // Circles are exact (true cylindrical faces).
        let hole_area = std::f64::consts::PI * 25.0;
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
                    through_all: false,
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

    /// M3: a rectangle with a circular hole, fully constrained by dimensions, extrudes
    /// into a plate with a hole; an arc+line "D" profile closes into a loop, and the
    /// solver keeps the arc's endpoints on its circle.
    #[test]
    fn m3_constrained_profile_with_hole_and_arc_loop() {
        let mut sketch = Sketch::new("PlateWithHole");
        let (_, [p0, p1, _, p3]) = sketch.add_rectangle(1.0, -2.0, 70.0, 45.0);
        sketch.add_constraint(SketchConstraint::Fixed { point: p0, x: 0.0, y: 0.0 });
        sketch.add_constraint(SketchConstraint::Distance { p1: p0, p2: p1, distance: 80.0 });
        sketch.add_constraint(SketchConstraint::Distance { p1: p0, p2: p3, distance: 50.0 });
        let c = sketch.add_point(38.0, 22.0);
        let circle = sketch.add_circle(c, 7.0);
        sketch.add_constraint(SketchConstraint::Fixed { point: c, x: 40.0, y: 25.0 });
        sketch.add_constraint(SketchConstraint::Radius { entity: circle, radius: 10.0 });
        let report = sketch.solve(60, 1e-10);
        assert_eq!(report.status, SketchStatus::FullyConstrained, "{report:?}");
        assert_eq!(sketch.extract_closed_profiles().len(), 2);

        let mut doc = Document::new("M3");
        let mut store = ShapeStore::new();
        let fid = doc
            .add_feature(
                "Plate",
                FeatureKind::SketchExtrude {
                    sketch,
                    depth: ScalarParam::mm(10.0),
                    plane: SketchPlane::XY,
                    mode: ExtrudeMode::NewBody,
                    reversed: false,
                    symmetric: false,
                    through_all: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, TessellationOptions::default()).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let hole = std::f64::consts::PI * 100.0;
        let vol = doc.features[&fid].cached_summary.unwrap().volume_mm3;
        assert!((vol - (4000.0 - hole) * 10.0).abs() < 1.0, "vol={vol}");

        // "D" shape: a diameter line closed by the upper half-arc.
        let mut d = Sketch::new("D");
        let a = d.add_point(-10.0, 0.0);
        let b = d.add_point(10.0, 0.0);
        let center = d.add_point(0.0, 0.0);
        d.add_line(a, b);
        d.add_arc(center, b, a, 10.0);
        // Knock the arc start off its circle; solving must pull it back.
        d.points.get_mut(&b).unwrap().x = 10.7;
        d.add_constraint(SketchConstraint::Fixed { point: center, x: 0.0, y: 0.0 });
        d.solve(60, 1e-10);
        let pb = d.points[&b];
        assert!((pb.x.hypot(pb.y) - d.entities.values().find_map(|e| match e {
            SketchEntity::Arc { radius, .. } => Some(*radius),
            _ => None,
        }).unwrap()).abs() < 1e-6);
        let loops = d.extract_closed_profiles();
        assert_eq!(loops.len(), 1);
        assert!(loops[0].len() > 10, "arc should be discretised");
    }

    /// M4: a flange revolved from an L-profile, one through-hole, a circular pattern of
    /// that hole; a mirrored+merged box; a shelled box; a revolve about a sketch line.
    #[test]
    fn m4_flange_pattern_mirror_shell() {
        let tess = TessellationOptions::default();
        let mut store = ShapeStore::new();
        let mut doc = Document::new("M4");
        let vol = |doc: &Document, id: FeatureId| doc.features[&id].cached_summary.unwrap().volume_mm3;
        let pi = std::f64::consts::PI;

        // Flange: disk r10..50 x 8 thick plus hub r10..20 up to 25, revolved about Z.
        let mut profile = Sketch::new("FlangeProfile");
        let pts: Vec<PointId> = [[10.0, 0.0], [50.0, 0.0], [50.0, 8.0], [20.0, 8.0], [20.0, 25.0], [10.0, 25.0]]
            .iter()
            .map(|p| profile.add_point(p[0], p[1]))
            .collect();
        profile.add_polyline(pts, true);
        let flange = doc
            .add_feature(
                "Flange",
                FeatureKind::SketchRevolve {
                    sketch: profile,
                    plane: SketchPlane::XZ,
                    axis: SketchAxis::Y,
                    angle_deg: ScalarParam::mm(360.0),
                    mode: ExtrudeMode::NewBody,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let flange_vol = 24_300.0 * pi;
        assert!((vol(&doc, flange) - flange_vol).abs() < 1.0, "{}", vol(&doc, flange));
        let b = doc.features[&flange].cached_summary.unwrap().bounds;
        assert!((b.max.x - 50.0).abs() < 1e-6 && (b.max.z - 25.0).abs() < 1e-6 && (b.min.z).abs() < 1e-6);

        // One bolt hole through everything at r=35, then a ring of 6.
        let mut hole = Sketch::new("Bolt");
        let c = hole.add_point(35.0, 0.0);
        hole.add_circle(c, 4.0);
        let bolt = doc
            .add_feature(
                "BoltHole",
                FeatureKind::SketchExtrude {
                    sketch: hole,
                    depth: ScalarParam::mm(1.0),
                    plane: SketchPlane::XY,
                    mode: ExtrudeMode::Cut { target: flange },
                    reversed: false,
                    symmetric: false,
                    through_all: true,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let ring = doc
            .add_feature(
                "BoltRing",
                FeatureKind::Pattern {
                    input: bolt,
                    layout: PatternLayout::Circular {
                        axis_origin: [0.0, 0.0, 0.0],
                        axis_dir: [0.0, 0.0, 1.0],
                        count: 6,
                        total_angle_deg: ScalarParam::mm(360.0),
                    },
                    tool_only: true,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let hole_area = std::f64::consts::PI * 16.0;
        assert!((vol(&doc, bolt) - (flange_vol - hole_area * 8.0)).abs() < 1.0);
        assert!((vol(&doc, ring) - (flange_vol - 6.0 * hole_area * 8.0)).abs() < 1.0, "{}", vol(&doc, ring));

        // Mirror a box touching the YZ plane and merge: one solid twice the size.
        let block = doc
            .add_feature(
                "Block",
                FeatureKind::Box {
                    width: ScalarParam::mm(20.0),
                    height: ScalarParam::mm(10.0),
                    depth: ScalarParam::mm(5.0),
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let mirrored = doc
            .add_feature(
                "Mirrored",
                FeatureKind::Mirror {
                    input: block,
                    plane: SketchPlane::YZ,
                    merge: true,
                    tool_only: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        // Shell a 40x30x20 box with its top face open, 2mm walls.
        let tub = doc
            .add_feature(
                "Tub",
                FeatureKind::Box {
                    width: ScalarParam::mm(40.0),
                    height: ScalarParam::mm(30.0),
                    depth: ScalarParam::mm(20.0),
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        doc.recompute(&mut store, tess).unwrap();
        let tub_shape = doc.features[&tub].cached_shape.unwrap();
        let top = store.resolve_face_by_signature(tub_shape, DVec3::Z, None, 0.99).unwrap();
        let top_ref = SemanticReference::from_face(tub, &store, tub_shape, top.index).unwrap();
        let shell = doc
            .add_feature(
                "Shell",
                FeatureKind::Shell {
                    input: tub,
                    thickness: ScalarParam::mm(2.0),
                    open_faces: vec![top_ref],
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        // Revolve about a sketch line: a 10x20 rectangle around its left side -> cylinder.
        let mut rect = Sketch::new("Cyl");
        let (_, [p0, _, _, p3]) = rect.add_rectangle(0.0, 0.0, 10.0, 20.0);
        let cyl = doc
            .add_feature(
                "Cyl",
                FeatureKind::SketchRevolve {
                    sketch: rect,
                    plane: SketchPlane::XY,
                    axis: SketchAxis::Line { a: p0, b: p3 },
                    angle_deg: ScalarParam::mm(360.0),
                    mode: ExtrudeMode::NewBody,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        assert!((vol(&doc, mirrored) - 2000.0).abs() < 1e-3);
        let mb = doc.features[&mirrored].cached_summary.unwrap();
        assert_eq!(mb.solids, 1);
        assert!((mb.bounds.min.x + 20.0).abs() < 1e-6 && (mb.bounds.max.x - 20.0).abs() < 1e-6);
        assert!((vol(&doc, shell) - (24_000.0 - 36.0 * 26.0 * 18.0)).abs() < 1e-2, "{}", vol(&doc, shell));
        assert!((vol(&doc, cyl) - 2000.0 * pi).abs() < 1.0, "{}", vol(&doc, cyl));

        // A profile straddling the axis is refused with a clear reason.
        let mut bad = Sketch::new("Bad");
        bad.add_rectangle(-5.0, 0.0, 5.0, 10.0);
        let bad_id = doc
            .add_feature(
                "Bad",
                FeatureKind::SketchRevolve {
                    sketch: bad,
                    plane: SketchPlane::XY,
                    axis: SketchAxis::Y,
                    angle_deg: ScalarParam::mm(360.0),
                    mode: ExtrudeMode::NewBody,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert_eq!(report.failed_features.len(), 1);
        assert_eq!(report.failed_features[0].0, bad_id);
        assert!(report.failed_features[0].1.contains("crosses the revolve axis"));
    }

    /// M5: a lost reference is reported with a structured reason and can be re-bound;
    /// rolling back, inserting a hole mid-history and rolling forward makes the later
    /// mirror build on the drilled plate.
    #[test]
    fn m5_rebind_and_rollback_insert() {
        let tess = TessellationOptions::default();
        let mut store = ShapeStore::new();
        let mut doc = Document::new("M5");
        let plate = doc
            .add_feature(
                "Plate",
                FeatureKind::Box {
                    width: ScalarParam::mm(100.0),
                    height: ScalarParam::mm(60.0),
                    depth: ScalarParam::mm(10.0),
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let mirror = doc
            .add_feature(
                "Mirror",
                FeatureKind::Mirror {
                    input: plate,
                    plane: SketchPlane::YZ,
                    merge: true,
                    tool_only: false,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        doc.recompute(&mut store, tess).unwrap();
        let vol = |doc: &Document, id: FeatureId| doc.features[&id].cached_summary.unwrap().volume_mm3;
        assert!((vol(&doc, mirror) - 120_000.0).abs() < 1e-3);

        // Roll back to the plate: the mirror is set aside.
        doc.set_rollback(Some(plate)).unwrap();
        doc.recompute(&mut store, tess).unwrap();
        assert!(doc.is_rolled_back(mirror));
        assert!(doc.features[&mirror].cached_mesh.is_none());
        assert!(doc.features[&mirror].last_valid_mesh.is_none(), "rolled-back features are not drawn");

        // Insert a through hole on the plate's top face while rolled back.
        let shape = doc.features[&plate].cached_shape.unwrap();
        let top = store.resolve_face_by_signature(shape, DVec3::Z, None, 0.99).unwrap();
        let top_ref = SemanticReference::from_face(plate, &store, shape, top.index).unwrap();
        let mut circle = Sketch::new("Hole");
        let c = circle.add_point(30.0, 20.0);
        circle.add_circle(c, 5.0);
        let cut = doc
            .add_feature(
                "Hole",
                FeatureKind::SketchExtrude {
                    sketch: circle,
                    depth: ScalarParam::mm(1.0),
                    plane: SketchPlane::Face {
                        feature: plate,
                        face: top_ref,
                    },
                    mode: ExtrudeMode::Cut { target: plate },
                    reversed: true,
                    symmetric: false,
                    through_all: true,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        assert_eq!(doc.timeline(), vec![plate, cut, mirror]);
        assert_eq!(doc.rollback, Some(cut), "the marker follows the inserted feature");
        match &doc.features[&mirror].kind {
            FeatureKind::Mirror { input, .. } => assert_eq!(*input, cut, "mirror now builds on the drilled plate"),
            other => panic!("{other:?}"),
        }
        assert_eq!(doc.downstream_of(plate), vec![cut, mirror]);

        doc.set_rollback(None).unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let hole = std::f64::consts::PI * 25.0;
        assert!((vol(&doc, mirror) - 2.0 * (60_000.0 - hole * 10.0)).abs() < 1.0, "{}", vol(&doc, mirror));

        // Break the hole's sketch-plane reference, then re-bind it to the top face.
        let mut broken = doc.features[&cut].kind.clone();
        let bogus = SemanticReference {
            target_feature: plate,
            kind: cad_geometry::SubshapeKind::Face,
            expected_normal: Some(DVec3::new(1.0, 1.0, 1.0).normalize()),
            normalized_bbox_centroid: None,
            world_centroid_hint: None,
            metric_hint: None,
            normal_cos_threshold: 0.99,
        };
        assert!(broken.replace_reference(RefRole::Plane, bogus));
        doc.update_feature(cut, None, Some(broken), None, None).unwrap();
        doc.recompute(&mut store, tess).unwrap();
        assert!(matches!(
            doc.features[&cut].failure,
            Some(CoreError::SemanticReferenceLost { .. })
        ));
        let slot = doc.features[&cut].kind.reference_slots().remove(0);
        assert_eq!((slot.role, slot.upstream), (RefRole::Plane, plate));
        assert!(doc.check_reference(&slot, &store).is_err());

        let shape = doc.features[&plate].cached_shape.unwrap();
        let fresh = SemanticReference::from_face(plate, &store, shape, top.index).unwrap();
        let mut fixed = doc.features[&cut].kind.clone();
        assert!(fixed.replace_reference(RefRole::Plane, fresh));
        doc.update_feature(cut, None, Some(fixed), None, None).unwrap();
        let report = doc.recompute(&mut store, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        assert!(doc.features[&cut].failure.is_none());

        // Deleting the plate takes everything built on it in one go.
        let mut ctrl = DocumentController::new(doc.clone());
        let removed = ctrl
            .document
            .remove_feature_cascade(plate, Some(&mut store))
            .unwrap();
        assert_eq!(removed, vec![mirror, cut, plate]);
        assert!(ctrl.document.features.is_empty());

        // Deleting a cut that had replaced (hidden) its target shows the target again.
        let mut doc2 = doc.clone();
        doc2.features.get_mut(&plate).unwrap().visible = false;
        doc2.remove_feature(mirror, None).unwrap();
        doc2.remove_feature(cut, None).unwrap();
        assert!(doc2.features[&plate].visible, "the plate is the visible result again");

        // Order and rollback survive a save/load round trip.
        doc.set_rollback(Some(plate)).unwrap();
        let reloaded = Document::from_snapshot(doc.to_snapshot()).unwrap();
        assert_eq!(reloaded.timeline(), vec![plate, cut, mirror]);
        assert_eq!(reloaded.rollback, Some(plate));
    }

    /// Reads a binary STL and returns (triangles, enclosed volume by the divergence
    /// theorem). A closed, consistently oriented mesh gives the solid's volume.
    fn read_stl(path: &std::path::Path) -> (usize, f64) {
        let bytes = std::fs::read(path).unwrap();
        let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
        assert_eq!(bytes.len(), 84 + 50 * n, "binary STL size must match its triangle count");
        let f = |o: usize| f32::from_le_bytes(bytes[o..o + 4].try_into().unwrap()) as f64;
        let mut vol = 0.0;
        for t in 0..n {
            let o = 84 + 50 * t + 12;
            let v = |k: usize| DVec3::new(f(o + 12 * k), f(o + 12 * k + 4), f(o + 12 * k + 8));
            vol += v(0).dot(v(1).cross(v(2))) / 6.0;
        }
        (n, vol)
    }

    /// M6: STL export is a closed mesh with the solid's volume; a STEP file imported as a
    /// body can be modelled on and survives a save/load round trip.
    #[test]
    fn m6_stl_export_and_step_import() {
        let tess = TessellationOptions::default();
        let mut store = ShapeStore::new();
        let (mut ctrl, plate) = build_perforated_plate_mvp().unwrap();
        ctrl.document.recompute(&mut store, tess).unwrap();
        let expected = ctrl.document.features[&plate].cached_summary.unwrap().volume_mm3;

        let dir = std::env::temp_dir().join(format!("ferrocad_m6_{}", Uuid::new_v4()));
        std::fs::create_dir_all(&dir).unwrap();
        let stl = dir.join("plate.stl");
        let fine = TessellationOptions {
            linear_deflection_mm: 0.02,
            angular_deflection_rad: 0.1,
        };
        let tris = ctrl.document.export_stl(&store, &[plate], &stl, fine).unwrap();
        let (n, vol) = read_stl(&stl);
        assert_eq!(n, tris);
        assert!((vol - expected).abs() / expected < 2e-3, "STL volume {vol} vs solid {expected}");

        // Import the plate back from STEP as a fixed body, then cut a hole into it.
        let step = dir.join("plate.step");
        store.export_step(ctrl.document.features[&plate].cached_shape.unwrap(), &step).unwrap();
        let mut doc = Document::new("Imported");
        let imported = doc
            .add_feature(
                "ImportedPlate",
                FeatureKind::ImportedBody {
                    source_name: "plate.step".to_string(),
                    step_text: std::fs::read_to_string(&step).unwrap(),
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let mut circle = Sketch::new("Hole");
        let c = circle.add_point(50.0, 30.0);
        circle.add_circle(c, 5.0);
        let cut = doc
            .add_feature(
                "Hole",
                FeatureKind::SketchExtrude {
                    sketch: circle,
                    depth: ScalarParam::mm(1.0),
                    plane: SketchPlane::XY,
                    mode: ExtrudeMode::Cut { target: imported },
                    reversed: false,
                    symmetric: false,
                    through_all: true,
                },
                true,
                [1.0; 4],
            )
            .unwrap();
        let mut store2 = ShapeStore::new();
        let report = doc.recompute(&mut store2, tess).unwrap();
        assert!(report.is_all_clean(), "{:?}", report.failed_features);
        let imported_vol = doc.features[&imported].cached_summary.unwrap().volume_mm3;
        assert!((imported_vol - expected).abs() < 1e-3);
        let hole = std::f64::consts::PI * 25.0;
        let cut_vol = doc.features[&cut].cached_summary.unwrap().volume_mm3;
        assert!((cut_vol - (expected - hole * 10.0)).abs() < 1.0);

        // The STEP text lives in the document: delete the file, reload, recompute.
        let saved = dir.join("model.fcad");
        doc.save_json(&saved).unwrap();
        std::fs::remove_file(&step).unwrap();
        let mut reloaded = Document::load_json(&saved).unwrap();
        let mut store3 = ShapeStore::new();
        assert!(reloaded.recompute(&mut store3, tess).unwrap().is_all_clean());
        assert!((reloaded.features[&cut].cached_summary.unwrap().volume_mm3 - cut_vol).abs() < 1e-3);
        let _ = std::fs::remove_dir_all(&dir);
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
                    through_all: false,
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

    #[test]
    fn midplane_sketch_tracks_both_semantic_faces() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Midplane");
        let plate = doc.add_feature(
            "Plate",
            FeatureKind::Box { width: ScalarParam::mm(40.0), height: ScalarParam::mm(30.0), depth: ScalarParam::mm(10.0) },
            true,
            [1.0; 4],
        ).unwrap();
        doc.recompute(&mut store, TessellationOptions::default()).unwrap();
        let shape = doc.features[&plate].cached_shape.unwrap();
        let top = store.resolve_face_by_signature(shape, DVec3::Z, None, 0.99).unwrap();
        let bottom = store.resolve_face_by_signature(shape, DVec3::NEG_Z, None, 0.99).unwrap();
        let plane = SketchPlane::MidFace {
            first_feature: plate,
            first: SemanticReference::from_face(plate, &store, shape, top.index).unwrap(),
            second_feature: plate,
            second: SemanticReference::from_face(plate, &store, shape, bottom.index).unwrap(),
        };

        let expected_z = (top.centroid.z + bottom.centroid.z) * 0.5;
        assert!((doc.sketch_plane_frame(&plane, &store).unwrap().origin.z - expected_z).abs() < 1e-9);
        let mut sketch = Sketch::new("Midplane Sketch");
        sketch.add_rectangle(-5.0, -5.0, 5.0, 5.0);
        let feature = FeatureKind::SketchExtrude {
            sketch,
            depth: ScalarParam::mm(2.0),
            plane,
            mode: ExtrudeMode::NewBody,
            reversed: false,
            symmetric: false,
            through_all: false,
        };
        assert_eq!(feature.upstream_features(), vec![plate]);
        assert_eq!(feature.reference_slots().len(), 2);
        assert!(doc.add_feature("Midplane extrusion", feature, true, [1.0; 4]).is_ok());
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
    }

    #[test]
    fn counterbore_position_is_parametric() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Positioned counterbore");
        let plate = doc.add_feature(
            "Plate",
            FeatureKind::Box { width: ScalarParam::mm(100.0), height: ScalarParam::mm(60.0), depth: ScalarParam::mm(10.0) },
            true,
            [1.0; 4],
        ).unwrap();
        let hole = doc.add_feature(
            "Counterbore",
            FeatureKind::Counterbore {
                input: plate,
                center_x: ScalarParam::mm(35.0),
                center_y: ScalarParam::mm(20.0),
                hole_radius: ScalarParam::mm(3.0),
                bore_radius: ScalarParam::mm(6.0),
                bore_depth: ScalarParam::mm(4.0),
                depth: ScalarParam::mm(10.0),
            },
            true,
            [1.0; 4],
        ).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let shape = store.inspect_shape(doc.features[&hole].cached_shape.unwrap()).unwrap();
        assert!(shape.faces.iter().any(|face| (face.centroid.x - 35.0).abs() < 1e-3 && (face.centroid.y - 20.0).abs() < 1e-3));
    }

    #[test]
    fn split_feature_partitions_intersecting_bodies() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Split");
        let input = doc.add_feature("Input", FeatureKind::Box { width: ScalarParam::mm(20.0), height: ScalarParam::mm(20.0), depth: ScalarParam::mm(20.0) }, true, [1.0; 4]).unwrap();
        let tool = doc.add_feature("Tool", FeatureKind::Box { width: ScalarParam::mm(10.0), height: ScalarParam::mm(30.0), depth: ScalarParam::mm(30.0) }, true, [1.0; 4]).unwrap();
        let split = doc.add_feature("Split", FeatureKind::Split { input, tool: Some(tool), plane: None }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        assert!(doc.features[&split].cached_summary.unwrap().solids >= 1);
    }

    #[test]
    fn sweep_start_position_moves_the_generated_solid() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Positioned sweep");
        let sweep = doc.add_feature("Sweep", FeatureKind::SweepCircle {
            radius: ScalarParam::mm(2.0), start_x: ScalarParam::mm(30.0), start_y: ScalarParam::mm(40.0), start_z: ScalarParam::mm(5.0),
            dx: ScalarParam::mm(0.0), dy: ScalarParam::mm(0.0), dz: ScalarParam::mm(20.0),
        }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let bounds = doc.features[&sweep].cached_summary.unwrap().bounds;
        assert!((bounds.center().x - 30.0).abs() < 1e-3 && (bounds.center().y - 40.0).abs() < 1e-3);
        assert!((bounds.min.z - 5.0).abs() < 1e-3);
    }

    #[test]
    fn face_move_in_document_recomputes_and_tracks_semantic_face() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("FaceMove doc");
        let b = doc.add_feature("Base", FeatureKind::Box {
            width: ScalarParam::mm(20.0),
            height: ScalarParam::mm(20.0),
            depth: ScalarParam::mm(20.0),
        }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());

        let base_shape = doc.features[&b].cached_shape.unwrap();
        let faces = store.inspect_shape(base_shape).unwrap().faces;
        let top_face = faces.iter().find(|f| (f.normal.z - 1.0).abs() < 1e-3).expect("top face");
        let sem_ref = SemanticReference::from_face(b, &store, base_shape, top_face.index).expect("sem_ref");

        let fm = doc.add_feature("MoveTop", FeatureKind::FaceMove {
            input: b,
            face: sem_ref,
            dx: ScalarParam::mm(0.0),
            dy: ScalarParam::mm(0.0),
            dz: ScalarParam::mm(15.0),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let moved_shape = doc.features[&fm].cached_shape.unwrap();
        let vol = store.volume(moved_shape).unwrap();
        // 20 * 20 * 35 = 14000
        assert!((vol - 14000.0).abs() < 1e-1, "vol was {vol}");
    }

    #[test]
    fn generalized_sweep_and_rib_in_document() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Sweep and Rib");

        // Profile sketch for sweep: 10 x 6 rectangle
        let mut sk = Sketch::new("Profile");
        let p0 = sk.add_point(-5.0, -3.0);
        let p1 = sk.add_point(5.0, -3.0);
        let p2 = sk.add_point(5.0, 3.0);
        let p3 = sk.add_point(-5.0, 3.0);
        sk.add_line(p0, p1);
        sk.add_line(p1, p2);
        sk.add_line(p2, p3);
        sk.add_line(p3, p0);

        let path = vec![
            [ScalarParam::mm(0.0), ScalarParam::mm(0.0), ScalarParam::mm(0.0)],
            [ScalarParam::mm(0.0), ScalarParam::mm(0.0), ScalarParam::mm(40.0)],
        ];
        let sweep = doc.add_feature("Sweep", FeatureKind::Sweep {
            profile: sk.clone(),
            path_points: path,
        }, true, [1.0; 4]).unwrap();

        let rib = doc.add_feature("Rib", FeatureKind::Rib {
            input: sweep,
            sketch: sk,
            plane: SketchPlane::XY,
            thickness: ScalarParam::mm(4.0),
            depth: ScalarParam::mm(10.0),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let rib_shape = doc.features[&rib].cached_shape.unwrap();
        assert!(store.volume(rib_shape).unwrap() > 2000.0);
    }

    #[test]
    fn hole_feature_on_face_with_specifications() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Hole Specifications");
        let b = doc.add_feature("Base", FeatureKind::Box {
            width: ScalarParam::mm(60.0),
            height: ScalarParam::mm(60.0),
            depth: ScalarParam::mm(20.0),
        }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());

        let base_shape = doc.features[&b].cached_shape.unwrap();
        let init_vol = store.volume(base_shape).unwrap();
        // 60 * 60 * 20 = 72000
        assert!((init_vol - 72000.0).abs() < 1e-1);

        let faces = store.inspect_shape(base_shape).unwrap().faces;
        let top_face = faces.iter().find(|f| (f.normal.z - 1.0).abs() < 1e-3).expect("top face");
        let face_ref = SemanticReference::from_face(b, &store, base_shape, top_face.index).expect("top face ref");

        // 1. Through-all Simple Hole (r=5.0) on face
        let h1 = doc.add_feature("ThroughHole", FeatureKind::Hole {
            input: b,
            plane: SketchPlane::Face { feature: b, face: face_ref.clone() },
            center_x: ScalarParam::mm(15.0),
            center_y: ScalarParam::mm(15.0),
            radius: ScalarParam::mm(5.0),
            depth: ScalarParam::mm(20.0),
            through_all: true,
            hole_type: HoleType::Simple,
            tip_angle_deg: None,
        }, true, [1.0; 4]).unwrap();

        // 2. Counterbore Blind Hole (r=3.0, d=15.0, bore_r=6.0, bore_d=5.0)
        let h2 = doc.add_feature("CounterboreHole", FeatureKind::Hole {
            input: h1,
            plane: SketchPlane::Face { feature: b, face: face_ref.clone() },
            center_x: ScalarParam::mm(45.0),
            center_y: ScalarParam::mm(15.0),
            radius: ScalarParam::mm(3.0),
            depth: ScalarParam::mm(15.0),
            through_all: false,
            hole_type: HoleType::Counterbore {
                bore_radius: ScalarParam::mm(6.0),
                bore_depth: ScalarParam::mm(5.0),
            },
            tip_angle_deg: None,
        }, true, [1.0; 4]).unwrap();

        // 3. Countersink Blind Hole with 118° drill tip
        let h3 = doc.add_feature("CountersinkHole", FeatureKind::Hole {
            input: h2,
            plane: SketchPlane::Face { feature: b, face: face_ref.clone() },
            center_x: ScalarParam::mm(45.0),
            center_y: ScalarParam::mm(45.0),
            radius: ScalarParam::mm(3.0),
            depth: ScalarParam::mm(12.0),
            through_all: false,
            hole_type: HoleType::Countersink {
                sink_radius: ScalarParam::mm(5.0),
                sink_depth: ScalarParam::mm(3.0),
            },
            tip_angle_deg: Some(ScalarParam::Literal(118.0)),
        }, true, [1.0; 4]).unwrap();

        // 4. Tap Hole (M6: thread_r=3.0, tap_drill_r=2.5, thread_d=10.0, d=16.0)
        let h4 = doc.add_feature("TapHole", FeatureKind::Hole {
            input: h3,
            plane: SketchPlane::Face { feature: b, face: face_ref },
            center_x: ScalarParam::mm(15.0),
            center_y: ScalarParam::mm(45.0),
            radius: ScalarParam::mm(2.5),
            depth: ScalarParam::mm(16.0),
            through_all: false,
            hole_type: HoleType::Tap {
                thread_radius: ScalarParam::mm(3.0),
                tap_drill_radius: ScalarParam::mm(2.5),
                thread_depth: ScalarParam::mm(10.0),
            },
            tip_angle_deg: Some(ScalarParam::Literal(118.0)),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());

        let final_shape = doc.features[&h4].cached_shape.unwrap();
        let final_vol = store.volume(final_shape).unwrap();
        // Each hole reduces volume significantly
        assert!(final_vol < init_vol - 2500.0, "final vol was {final_vol}, initial was {init_vol}");

        // Modify upstream Box height to 30.0: holes should follow the moving top face
        doc.update_feature(b, None, Some(FeatureKind::Box {
            width: ScalarParam::mm(60.0),
            height: ScalarParam::mm(60.0),
            depth: ScalarParam::mm(30.0),
        }), None, None).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let resized_shape = doc.features[&h4].cached_shape.unwrap();
        let resized_vol = store.volume(resized_shape).unwrap();
        // Base is now 60*60*30 = 108000. Through hole now cuts through 30mm thickness.
        assert!(resized_vol > final_vol + 30000.0, "resized vol was {resized_vol}");
    }

    #[test]
    fn pattern_on_reference_edge_and_cylinder_axis() {
        let mut store = ShapeStore::new();
        let mut doc = Document::new("Pattern on Reference");

        // 1. Base Box 100 x 50 x 20
        let b = doc.add_feature("Base", FeatureKind::Box {
            width: ScalarParam::mm(100.0),
            height: ScalarParam::mm(50.0),
            depth: ScalarParam::mm(20.0),
        }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());

        let base_shape = doc.features[&b].cached_shape.unwrap();
        let edges = store.inspect_shape(base_shape).unwrap().edges;
        // Find an edge parallel to X-axis (length 100)
        let x_edge = edges.iter().find(|e| (e.length_mm - 100.0).abs() < 1e-3).expect("x edge");
        let edge_ref = SemanticReference::from_edge(b, &store, base_shape, x_edge.index).expect("edge ref");

        // 2. Through-hole near the start (x=15, y=25)
        let faces = store.inspect_shape(base_shape).unwrap().faces;
        let top_face = faces.iter().find(|f| (f.normal.z - 1.0).abs() < 1e-3).expect("top face");
        let face_ref = SemanticReference::from_face(b, &store, base_shape, top_face.index).expect("top face ref");

        let h = doc.add_feature("Hole", FeatureKind::Hole {
            input: b,
            plane: SketchPlane::Face { feature: b, face: face_ref.clone() },
            center_x: ScalarParam::mm(15.0),
            center_y: ScalarParam::mm(25.0),
            radius: ScalarParam::mm(4.0),
            depth: ScalarParam::mm(20.0),
            through_all: true,
            hole_type: HoleType::Simple,
            tip_angle_deg: None,
        }, true, [1.0; 4]).unwrap();

        // 3. Linear pattern along the referenced Edge: 4 holes with spacing 20mm
        let pat_lin = doc.add_feature("LinearPat", FeatureKind::PatternOnReference {
            input: h,
            reference: ReferenceAxis::Edge { feature: b, edge: edge_ref },
            layout: PatternReferenceLayout::Linear {
                spacing: ScalarParam::mm(20.0),
                count: 4,
            },
            tool_only: true,
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let pat_shape = doc.features[&pat_lin].cached_shape.unwrap();
        let pat_vol = store.volume(pat_shape).unwrap();
        // Base is 100000. 4 through-holes of r=4, depth=20 cut ~4 * pi * 16 * 20 ~= 4021 mm3
        assert!(pat_vol < 100000.0 - 3500.0, "pat_vol was {pat_vol}");

        // 4. Cylinder face as a rotation axis reference
        let cyl = doc.add_feature("CenterBoss", FeatureKind::Cylinder {
            radius: ScalarParam::mm(10.0),
            height: ScalarParam::mm(30.0),
        }, true, [1.0; 4]).unwrap();
        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());

        let cyl_shape = doc.features[&cyl].cached_shape.unwrap();
        let cyl_faces = store.inspect_shape(cyl_shape).unwrap().faces;
        let cyl_face = cyl_faces.iter().find(|f| f.normal.z.abs() < 1e-3).expect("cyl face");
        let cyl_face_ref = SemanticReference::from_face(cyl, &store, cyl_shape, cyl_face.index).expect("cyl face ref");

        // Tool pin to rotate: Small cylinder offset at x=20
        let pin = doc.add_feature("Pin", FeatureKind::Cylinder {
            radius: ScalarParam::mm(2.0),
            height: ScalarParam::mm(10.0),
        }, true, [1.0; 4]).unwrap();
        let pin_moved = doc.add_feature("PinMoved", FeatureKind::Transform {
            input: pin,
            dx: ScalarParam::mm(20.0),
            dy: ScalarParam::mm(0.0),
            dz: ScalarParam::mm(0.0),
            rot_origin: [0.0; 3],
            rot_axis: [0.0, 0.0, 1.0],
            rot_angle_deg: ScalarParam::Literal(0.0),
        }, true, [1.0; 4]).unwrap();

        // Circular pattern on cylinder face axis: 6 pins around the boss
        let pat_circ = doc.add_feature("CircularPat", FeatureKind::PatternOnReference {
            input: pin_moved,
            reference: ReferenceAxis::Face { feature: cyl, face: cyl_face_ref },
            layout: PatternReferenceLayout::Circular {
                count: 6,
                total_angle_deg: ScalarParam::Literal(360.0),
            },
            tool_only: false,
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let circ_shape = doc.features[&pat_circ].cached_shape.unwrap();
        let circ_vol = store.volume(circ_shape).unwrap();
        // 6 pins of r=2, h=10: 6 * pi * 4 * 10 ~= 753.98 mm3
        assert!((circ_vol - 753.98).abs() < 5.0, "circ_vol was {circ_vol}");
    }

    #[test]
    fn test_m11_surface_interop() {
        let mut doc = Document::default();
        let mut store = ShapeStore::default();

        // 1. Box split by XY plane
        let box_raw = doc.add_feature("RawBox", FeatureKind::Box {
            width: ScalarParam::mm(20.0),
            height: ScalarParam::mm(20.0),
            depth: ScalarParam::mm(40.0),
        }, true, [1.0; 4]).unwrap();

        // Shift so Z extends from -20 to +20
        let box_centered = doc.add_feature("BoxCentered", FeatureKind::Transform {
            input: box_raw,
            dx: ScalarParam::mm(0.0),
            dy: ScalarParam::mm(0.0),
            dz: ScalarParam::mm(-20.0),
            rot_origin: [0.0; 3],
            rot_axis: [0.0, 0.0, 1.0],
            rot_angle_deg: ScalarParam::Literal(0.0),
        }, true, [1.0; 4]).unwrap();

        let split_box = doc.add_feature("SplitBox", FeatureKind::Split {
            input: box_centered,
            tool: None,
            plane: Some(SketchPlane::XY),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let split_shape = doc.features[&split_box].cached_shape.unwrap();
        let split_faces = store.inspect_shape(split_shape).unwrap().faces;
        // Splitting 20x20x40 box at Z=0 introduces split sections; total faces >= 8
        assert!(split_faces.len() >= 8, "Expected split box to have >= 8 faces, got {}", split_faces.len());

        // 2. SurfaceOffset: offset the top face (+Z) by 5.0 mm
        let top_face = split_faces.iter().find(|f| f.normal.z > 0.9).expect("top face");
        let top_ref = SemanticReference::from_face(split_box, &store, split_shape, top_face.index).expect("top ref");
        let offset_feat = doc.add_feature("OffsetTop", FeatureKind::SurfaceOffset {
            input: split_box,
            face: top_ref.clone(),
            distance: ScalarParam::mm(5.0),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let offset_shape = doc.features[&offset_feat].cached_shape.unwrap();
        let offset_faces = store.inspect_shape(offset_shape).unwrap().faces;
        assert_eq!(offset_faces.len(), 1);
        assert!((offset_faces[0].area_mm2 - 400.0).abs() < 1.0);
        assert!((offset_faces[0].centroid.z - 5.0).abs() < 1.0);

        // 3. TrimSurface: trim the offset face in UV from [0.25..0.75] x [0.25..0.75]
        let offset_face_ref = SemanticReference::from_face(offset_feat, &store, offset_shape, offset_faces[0].index).expect("offset ref");
        let trim_feat = doc.add_feature("TrimmedTop", FeatureKind::TrimSurface {
            input: offset_feat,
            face: offset_face_ref,
            u_min: ScalarParam::Literal(0.0),
            u_max: ScalarParam::Literal(10.0),
            v_min: ScalarParam::Literal(0.0),
            v_max: ScalarParam::Literal(10.0),
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let trim_shape = doc.features[&trim_feat].cached_shape.unwrap();
        let trim_faces = store.inspect_shape(trim_shape).unwrap().faces;
        assert_eq!(trim_faces.len(), 1);
        // Half width x half height => area should be ~100 mm2 (1/4 of 400)
        assert!((trim_faces[0].area_mm2 - 100.0).abs() < 1.0, "trim area was {}", trim_faces[0].area_mm2);

        // 4. Sew multiple shapes: Sew the trimmed surface with another face/shape
        let side_face = split_faces.iter().find(|f| f.normal.x > 0.9).expect("side face");
        let side_ref = SemanticReference::from_face(split_box, &store, split_shape, side_face.index).expect("side ref");
        let side_offset = doc.add_feature("OffsetSide", FeatureKind::SurfaceOffset {
            input: split_box,
            face: side_ref,
            distance: ScalarParam::mm(0.0), // exact face surface
        }, true, [1.0; 4]).unwrap();

        let multi_sew = doc.add_feature("MultiSew", FeatureKind::Sew {
            input: offset_feat,
            extra_inputs: vec![side_offset],
            tolerance: ScalarParam::mm(10.0), // large tolerance to sew adjacent boundary
        }, true, [1.0; 4]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let sew_shape = doc.features[&multi_sew].cached_shape.unwrap();
        let sew_faces = store.inspect_shape(sew_shape).unwrap().faces;
        assert_eq!(sew_faces.len(), 2, "Expected 2 sewn faces in compound shell, got {}", sew_faces.len());
    }

    #[test]
    fn test_m11_stepcaf_document_import() {
        let mut doc = Document::default();
        let mut store = ShapeStore::default();

        let b = doc.add_feature("Box1", FeatureKind::Box {
            width: ScalarParam::mm(15.0),
            height: ScalarParam::mm(25.0),
            depth: ScalarParam::mm(35.0),
        }, true, [0.9, 0.2, 0.2, 1.0]).unwrap();

        assert!(doc.recompute(&mut store, TessellationOptions::default()).unwrap().is_all_clean());
        let shape_id = doc.features[&b].cached_shape.unwrap();
        let temp_dir = std::env::temp_dir();
        let step_path = temp_dir.join("ferrocad_core_stepcaf.step");
        store.export_step(shape_id, &step_path).expect("export_step");

        let mut doc2 = Document::default();
        let mut store2 = ShapeStore::default();
        let imported_fids = doc2.import_step_caf(&mut store2, &step_path).expect("import_step_caf");
        assert!(!imported_fids.is_empty());
        assert!(doc2.recompute(&mut store2, TessellationOptions::default()).unwrap().is_all_clean());

        let imported_shape = doc2.features[&imported_fids[0]].cached_shape.unwrap();
        let vol = store2.volume(imported_shape).unwrap();
        assert!((vol - 13125.0).abs() < 1.0);

        let _ = std::fs::remove_file(&step_path);
    }
}
