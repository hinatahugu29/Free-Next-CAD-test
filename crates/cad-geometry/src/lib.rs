//! Pure Rust geometry abstractions, unit types, `TriMesh`, and `ShapeStore` (Phase 1).
//!
//! This crate encapsulates `cad-occt` so that downstream crates (`cad-core`,
//! `cad-render`, `cad-ui`, `cad-cli`) never interact with OCCT handles directly
//! and instead reference shapes via generational [`ShapeId`]s managed by [`ShapeStore`].

use std::path::Path;

use cad_occt::{OcctError, OcctShape};
use glam::{DVec3, Vec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Error type for geometric operations and `ShapeStore` lookups.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Error)]
pub enum GeometryError {
    #[error("Invalid dimension or parameter '{name}': {reason}")]
    InvalidParameter { name: String, reason: String },
    #[error("ShapeId {0:?} not found in ShapeStore")]
    ShapeNotFound(ShapeId),
    #[error("ShapeId {0:?} is stale (already freed or overwritten)")]
    StaleShapeId(ShapeId),
    #[error("Resulting shape failed topological validity check")]
    InvalidTopology,
    #[error("Semantic reference resolution failed: {0}")]
    ReferenceResolutionFailed(String),
    #[error("Kernel error: {0}")]
    Kernel(#[from] OcctError),
}

/// Length value stored in millimeters ($\text{mm}$).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Length(pub f64);

impl Length {
    /// Creates a length in millimeters.
    pub const fn mm(val: f64) -> Self {
        Self(val)
    }

    /// Creates a length from centimeters.
    pub fn cm(val: f64) -> Self {
        Self(val * 10.0)
    }

    /// Creates a length from meters.
    pub fn m(val: f64) -> Self {
        Self(val * 1000.0)
    }

    /// Creates a length from inches.
    pub fn inch(val: f64) -> Self {
        Self(val * 25.4)
    }

    /// Returns the value in millimeters.
    pub const fn in_mm(self) -> f64 {
        self.0
    }

    /// Validates that the length is finite and strictly positive.
    pub fn ensure_positive(self, name: &str) -> Result<f64, GeometryError> {
        if !self.0.is_finite() || self.0 <= 1e-9 {
            return Err(GeometryError::InvalidParameter {
                name: name.to_string(),
                reason: format!("expected positive length in mm, got {}", self.0),
            });
        }
        Ok(self.0)
    }
}

/// Angle value stored in degrees.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct Angle(pub f64);

impl Angle {
    /// Creates an angle in degrees.
    pub const fn deg(val: f64) -> Self {
        Self(val)
    }

    /// Creates an angle from radians.
    pub fn rad(val: f64) -> Self {
        Self(val.to_degrees())
    }

    /// Returns the angle in degrees.
    pub const fn in_deg(self) -> f64 {
        self.0
    }

    /// Returns the angle in radians.
    pub fn in_rad(self) -> f64 {
        self.0.to_radians()
    }
}

/// Generational identifier referencing a shape stored inside [`ShapeStore`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ShapeId {
    pub index: u32,
    pub generation: u32,
}

/// Identifier for a topological subshape (Solid, Face, Edge, Vertex).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SubshapeKind {
    Solid,
    Face,
    Edge,
    Vertex,
}

/// Reference to a specific subshape index within a shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SubshapeRef {
    pub shape_id: ShapeId,
    pub kind: SubshapeKind,
    pub index: u32,
}

/// Topological element counts for a [`Shape`].
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ShapeSummary {
    pub solids: u32,
    pub faces: u32,
    pub edges: u32,
    pub vertices: u32,
    pub volume_mm3: f64,
    pub surface_area_mm2: f64,
    pub bounds: BoundingBox,
}

/// High-level representation of a topological `Solid` metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Solid {
    pub index: u32,
}

/// High-level representation of a topological `Face` with geometric signature for semantic references.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Face {
    pub index: u32,
    pub area_mm2: f64,
    pub centroid: DVec3,
    pub normal: DVec3,
}

/// High-level representation of a topological `Edge` with geometric signature.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Edge {
    pub index: u32,
    pub length_mm: f64,
    pub centroid: DVec3,
}

/// High-level representation of a topological `Vertex`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Vertex {
    pub index: u32,
    pub position: DVec3,
}

/// High-level topological inspection of a `Shape`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Shape {
    pub id: ShapeId,
    pub summary: ShapeSummary,
    pub faces: Vec<Face>,
    pub edges: Vec<Edge>,
}

/// Axis-aligned 3D bounding box in millimeters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoundingBox {
    pub min: DVec3,
    pub max: DVec3,
}

impl BoundingBox {
    pub fn new(min: DVec3, max: DVec3) -> Self {
        Self { min, max }
    }

    pub fn center(&self) -> DVec3 {
        (self.min + self.max) * 0.5
    }

    pub fn size(&self) -> DVec3 {
        self.max - self.min
    }

    pub fn diagonal_length(&self) -> f64 {
        self.size().length()
    }

    pub fn union(&self, other: &Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }
}

/// Spatial transformation in 3D space (translation + axis-angle rotation + scale).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform {
    pub translation: DVec3,
    pub rotation_origin: DVec3,
    pub rotation_axis: DVec3,
    pub rotation_angle_deg: f64,
    pub scale_factor: f64,
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: DVec3::ZERO,
            rotation_origin: DVec3::ZERO,
            rotation_axis: DVec3::Z,
            rotation_angle_deg: 0.0,
            scale_factor: 1.0,
        }
    }
}

impl Transform {
    pub fn from_translation(dx: f64, dy: f64, dz: f64) -> Self {
        Self {
            translation: DVec3::new(dx, dy, dz),
            ..Self::default()
        }
    }

    pub fn from_rotation(origin: DVec3, axis: DVec3, angle_deg: f64) -> Self {
        Self {
            rotation_origin: origin,
            rotation_axis: axis,
            rotation_angle_deg: angle_deg,
            ..Self::default()
        }
    }
}

///Tessellated triangle mesh and boundary edges extracted from a B-Rep shape.
///
/// Retains subshape correspondence (`triangle_face_ids` and `edge_segment_ids`)
/// so that viewport picking can resolve a clicked triangle or edge back to its
/// originating B-Rep `Face` or `Edge`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TriMesh {
    /// Interleaved `[x, y, z]` vertex coordinates in millimeters.
    pub positions: Vec<[f32; 3]>,
    /// Interleaved `[nx, ny, nz]` vertex normals.
    pub normals: Vec<[f32; 3]>,
    /// Triangle vertex indices (`[i0, i1, i2]` per triangle).
    pub indices: Vec<[u32; 3]>,
    /// 0-based B-Rep `Face` index corresponding to each triangle in `indices`.
    pub triangle_face_ids: Vec<u32>,
    /// Discretized B-Rep boundary edge line segments (`[start_xyz, end_xyz]`).
    pub edge_segments: Vec<[[f32; 3]; 2]>,
    /// 0-based B-Rep `Edge` index corresponding to each segment in `edge_segments`.
    pub edge_segment_ids: Vec<u32>,
}

/// One exact segment of a closed 2D profile (millimetres, radians).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ProfileSegment {
    Line { a: [f64; 2], b: [f64; 2] },
    /// Counter-clockwise arc from `start_rad` to `end_rad` (`end_rad > start_rad`).
    Arc {
        center: [f64; 2],
        radius: f64,
        start_rad: f64,
        end_rad: f64,
    },
    Circle { center: [f64; 2], radius: f64 },
}

impl ProfileSegment {
    fn encode(&self, out: &mut Vec<f64>) {
        match *self {
            Self::Line { a, b } => out.extend_from_slice(&[0.0, a[0], a[1], b[0], b[1]]),
            Self::Arc {
                center,
                radius,
                start_rad,
                end_rad,
            } => out.extend_from_slice(&[1.0, center[0], center[1], radius, start_rad, end_rad]),
            Self::Circle { center, radius } => out.extend_from_slice(&[2.0, center[0], center[1], radius]),
        }
    }
}

/// Writes triangle meshes as one binary STL file (millimetres, facet normals computed
/// from the triangle winding).
pub fn write_stl_binary(meshes: &[&TriMesh], path: impl AsRef<Path>) -> std::io::Result<()> {
    let count: usize = meshes.iter().map(|m| m.indices.len()).sum();
    let count = u32::try_from(count)
        .map_err(|_| std::io::Error::new(std::io::ErrorKind::InvalidInput, "too many triangles for STL"))?;
    let mut out = Vec::with_capacity(84 + 50 * count as usize);
    let mut header = [0u8; 80];
    let tag = b"FerroCAD binary STL (mm)";
    header[..tag.len()].copy_from_slice(tag);
    out.extend_from_slice(&header);
    out.extend_from_slice(&count.to_le_bytes());
    for mesh in meshes {
        for tri in &mesh.indices {
            let [a, b, c] = tri.map(|i| DVec3::from_array(mesh.positions[i as usize].map(f64::from)));
            let n = (b - a).cross(c - a).normalize_or_zero();
            for v in [n, a, b, c] {
                for comp in [v.x, v.y, v.z] {
                    out.extend_from_slice(&(comp as f32).to_le_bytes());
                }
            }
            out.extend_from_slice(&0u16.to_le_bytes());
        }
    }
    std::fs::write(path, out)
}

impl TriMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// Performs Möller–Trumbore ray-triangle intersection against the mesh,
    /// returning `(distance, triangle_index, face_index, hit_point)`.
    pub fn raycast(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<(f32, usize, u32, Vec3)> {
        let dir = ray_dir.normalize_or_zero();
        if dir.length_squared() < 1e-6 {
            return None;
        }
        let mut closest: Option<(f32, usize, u32, Vec3)> = None;
        for (tri_idx, &[i0, i1, i2]) in self.indices.iter().enumerate() {
            let v0 = Vec3::from_array(self.positions[i0 as usize]);
            let v1 = Vec3::from_array(self.positions[i1 as usize]);
            let v2 = Vec3::from_array(self.positions[i2 as usize]);

            let e1 = v1 - v0;
            let e2 = v2 - v0;
            let h = dir.cross(e2);
            let a = e1.dot(h);
            if a.abs() < 1e-7 {
                continue;
            }
            let f = 1.0 / a;
            let s = ray_origin - v0;
            let u = f * s.dot(h);
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = s.cross(e1);
            let v = f * dir.dot(q);
            if v < 0.0 || u + v > 1.0 {
                continue;
            }
            let t = f * e2.dot(q);
            if t > 1e-4 && (closest.is_none() || t < closest.unwrap().0) {
                let face_id = self.triangle_face_ids.get(tri_idx).copied().unwrap_or(0);
                let hit = ray_origin + dir * t;
                closest = Some((t, tri_idx, face_id, hit));
            }
        }
        closest
    }
}

/// Tessellation quality settings.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TessellationOptions {
    pub linear_deflection_mm: f64,
    pub angular_deflection_rad: f64,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self {
            linear_deflection_mm: 0.25,
            angular_deflection_rad: 0.25,
        }
    }
}

struct SlotEntry {
    generation: u32,
    shape: Option<OcctShape>,
}

/// Centralized owner and lifecycle manager for all B-Rep shapes (Phase 1.4).
///
/// `Document` and features store only [`ShapeId`] handles rather than raw OCCT objects.
#[derive(Default)]
pub struct ShapeStore {
    slots: Vec<SlotEntry>,
    free_indices: Vec<u32>,
}

impl ShapeStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts an internal OCCT shape and returns a new generational [`ShapeId`].
    fn insert(&mut self, shape: OcctShape) -> Result<ShapeId, GeometryError> {
        if !shape.is_valid()? {
            return Err(GeometryError::InvalidTopology);
        }
        if let Some(idx) = self.free_indices.pop() {
            let slot = &mut self.slots[idx as usize];
            slot.shape = Some(shape);
            Ok(ShapeId {
                index: idx,
                generation: slot.generation,
            })
        } else {
            let idx = self.slots.len() as u32;
            let generation = 1;
            self.slots.push(SlotEntry {
                generation,
                shape: Some(shape),
            });
            Ok(ShapeId {
                index: idx,
                generation,
            })
        }
    }

    fn get(&self, id: ShapeId) -> Result<&OcctShape, GeometryError> {
        let Some(slot) = self.slots.get(id.index as usize) else {
            return Err(GeometryError::ShapeNotFound(id));
        };
        if slot.generation != id.generation {
            return Err(GeometryError::StaleShapeId(id));
        }
        slot.shape.as_ref().ok_or(GeometryError::StaleShapeId(id))
    }

    /// Returns `true` if `id` points to a currently live shape.
    pub fn contains(&self, id: ShapeId) -> bool {
        self.get(id).is_ok()
    }

    /// Returns the number of active shapes currently stored.
    pub fn active_count(&self) -> usize {
        self.slots.iter().filter(|s| s.shape.is_some()).count()
    }

    /// Frees a shape from the store, incrementing its slot generation so old [`ShapeId`]s become stale.
    pub fn remove(&mut self, id: ShapeId) -> Result<(), GeometryError> {
        let Some(slot) = self.slots.get_mut(id.index as usize) else {
            return Err(GeometryError::ShapeNotFound(id));
        };
        if slot.generation != id.generation || slot.shape.is_none() {
            return Err(GeometryError::StaleShapeId(id));
        }
        slot.shape = None;
        slot.generation = slot.generation.wrapping_add(1);
        self.free_indices.push(id.index);
        Ok(())
    }

    /// Clears all shapes from the store.
    pub fn clear(&mut self) {
        for (idx, slot) in self.slots.iter_mut().enumerate() {
            if slot.shape.is_some() {
                slot.shape = None;
                slot.generation = slot.generation.wrapping_add(1);
                self.free_indices.push(idx as u32);
            }
        }
    }

    /// Clones an existing shape in the store and returns its new [`ShapeId`].
    pub fn clone_shape(&mut self, id: ShapeId) -> Result<ShapeId, GeometryError> {
        let cloned = self.get(id)?.try_clone()?;
        self.insert(cloned)
    }

    /// Creates a box primitive with dimensions `(width, height, depth)` in millimeters.
    pub fn create_box(
        &mut self,
        width: Length,
        height: Length,
        depth: Length,
    ) -> Result<ShapeId, GeometryError> {
        let dx = width.ensure_positive("width")?;
        let dy = height.ensure_positive("height")?;
        let dz = depth.ensure_positive("depth")?;
        let shape = OcctShape::make_box(dx, dy, dz)?;
        self.insert(shape)
    }

    /// Creates a cylinder primitive with `radius` and `height`.
    pub fn create_cylinder(
        &mut self,
        radius: Length,
        height: Length,
    ) -> Result<ShapeId, GeometryError> {
        let r = radius.ensure_positive("radius")?;
        let h = height.ensure_positive("height")?;
        let shape = OcctShape::make_cylinder(r, h)?;
        self.insert(shape)
    }

    /// Creates a sphere primitive with `radius`.
    pub fn create_sphere(&mut self, radius: Length) -> Result<ShapeId, GeometryError> {
        let r = radius.ensure_positive("radius")?;
        let shape = OcctShape::make_sphere(r)?;
        self.insert(shape)
    }

    /// Creates a cone or frustum primitive with `bottom_radius`, `top_radius`, and `height`.
    pub fn create_cone(
        &mut self,
        bottom_radius: Length,
        top_radius: Length,
        height: Length,
    ) -> Result<ShapeId, GeometryError> {
        let r1 = bottom_radius.in_mm();
        let r2 = top_radius.in_mm();
        if !r1.is_finite() || r1 < 0.0 || !r2.is_finite() || r2 < 0.0 || (r1 <= 1e-9 && r2 <= 1e-9)
        {
            return Err(GeometryError::InvalidParameter {
                name: "cone_radius".to_string(),
                reason: "radii must be non-negative and at least one positive".to_string(),
            });
        }
        let h = height.ensure_positive("height")?;
        let shape = OcctShape::make_cone(r1, r2, h)?;
        self.insert(shape)
    }

    /// Extrudes a 2D polygon on the XY plane along +Z by `depth`.
    pub fn extrude_polygon(
        &mut self,
        points_xy: &[[f64; 2]],
        depth: Length,
    ) -> Result<ShapeId, GeometryError> {
        let dz = depth.ensure_positive("depth")?;
        if points_xy.len() < 3 {
            return Err(GeometryError::InvalidParameter {
                name: "points_xy".to_string(),
                reason: "polygon requires at least 3 vertices".to_string(),
            });
        }
        let shape = OcctShape::make_extrude_polygon(points_xy, dz)?;
        self.insert(shape)
    }

    /// Extrudes a 2D circle on the XY plane along +Z by `depth`.
    pub fn extrude_circle(
        &mut self,
        radius: Length,
        depth: Length,
    ) -> Result<ShapeId, GeometryError> {
        let r = radius.ensure_positive("radius")?;
        let dz = depth.ensure_positive("depth")?;
        let shape = OcctShape::make_extrude_circle(r, dz)?;
        self.insert(shape)
    }

    /// Revolves a 2D polygon on the XZ plane around the Z axis by `angle`.
    /// Solid from a closed profile of exact lines/arcs/circles: extruded along +Z by
    /// `depth` from the XY plane, giving true cylindrical faces for circles and arcs.
    pub fn extrude_profile(&mut self, segments: &[ProfileSegment], depth: Length) -> Result<ShapeId, GeometryError> {
        let d = depth.ensure_positive("depth")?;
        let mut flat = Vec::new();
        segments.iter().for_each(|s| s.encode(&mut flat));
        let shape = OcctShape::make_profile_solid(&flat, d, false)?;
        self.insert(shape)
    }

    /// Revolves a closed profile drawn in the XZ plane (u = X, v = Z) around Z.
    pub fn revolve_profile(&mut self, segments: &[ProfileSegment], angle: Angle) -> Result<ShapeId, GeometryError> {
        let deg = angle.in_deg();
        if !deg.is_finite() || deg <= 1e-6 || deg > 360.0 {
            return Err(GeometryError::InvalidParameter {
                name: "angle".to_string(),
                reason: "revolve angle must be in (0, 360] degrees".to_string(),
            });
        }
        let mut flat = Vec::new();
        segments.iter().for_each(|s| s.encode(&mut flat));
        let shape = OcctShape::make_profile_solid(&flat, deg, true)?;
        self.insert(shape)
    }

    /// Creates a solid loft between two closed planar sketch profiles. The second profile
    /// is placed at `second_z` in the common sketch coordinate frame.
    pub fn loft_profiles(
        &mut self,
        first: &[ProfileSegment],
        second: &[ProfileSegment],
        second_z: f64,
    ) -> Result<ShapeId, GeometryError> {
        let mut first_flat = Vec::new();
        first.iter().for_each(|segment| segment.encode(&mut first_flat));
        let mut second_flat = Vec::new();
        second.iter().for_each(|segment| segment.encode(&mut second_flat));
        self.insert(OcctShape::loft_profiles(&first_flat, &second_flat, second_z)?)
    }

    /// Converts one closed shell into a solid. This is useful after importing or sewing
    /// surface data that has no solid topology yet.
    pub fn solid_from_shell(&mut self, id: ShapeId) -> Result<ShapeId, GeometryError> {
        let solid = self.get(id)?.solid_from_shell()?;
        self.insert(solid)
    }

    /// Stitches coincident shell boundaries within `tolerance_mm`.
    pub fn sew_shape(&mut self, id: ShapeId, tolerance_mm: f64) -> Result<ShapeId, GeometryError> {
        self.insert(self.get(id)?.sew(tolerance_mm)?)
    }

    pub fn revolve_polygon(
        &mut self,
        points_xz: &[[f64; 2]],
        angle: Angle,
    ) -> Result<ShapeId, GeometryError> {
        let deg = angle.in_deg();
        if !deg.is_finite() || deg <= 1e-6 || deg > 360.0 {
            return Err(GeometryError::InvalidParameter {
                name: "angle".to_string(),
                reason: "revolve angle must be in (0, 360] degrees".to_string(),
            });
        }
        let shape = OcctShape::make_revolve_polygon(points_xz, deg)?;
        self.insert(shape)
    }

    /// Boolean union (`Fuse`) of two shapes.
    pub fn fuse(&mut self, a: ShapeId, b: ShapeId) -> Result<ShapeId, GeometryError> {
        let sa = self.get(a)?;
        let sb = self.get(b)?;
        let res = sa.fuse(sb)?;
        self.insert(res)
    }

    /// Boolean subtraction (`Cut`) of `tool` from `base`.
    pub fn cut(&mut self, base: ShapeId, tool: ShapeId) -> Result<ShapeId, GeometryError> {
        let s_base = self.get(base)?;
        let s_tool = self.get(tool)?;
        let res = s_base.cut(s_tool)?;
        self.insert(res)
    }

    /// Boolean intersection (`Common`) of two shapes.
    pub fn common(&mut self, a: ShapeId, b: ShapeId) -> Result<ShapeId, GeometryError> {
        let sa = self.get(a)?;
        let sb = self.get(b)?;
        let res = sa.common(sb)?;
        self.insert(res)
    }

    /// Translates a shape by `(dx, dy, dz)` in millimeters.
    pub fn translate(
        &mut self,
        id: ShapeId,
        dx: f64,
        dy: f64,
        dz: f64,
    ) -> Result<ShapeId, GeometryError> {
        let s = self.get(id)?;
        let res = s.translate(dx, dy, dz)?;
        self.insert(res)
    }

    /// Rotates a shape around `(origin, axis)` by `angle`.
    pub fn rotate(
        &mut self,
        id: ShapeId,
        origin: DVec3,
        axis: DVec3,
        angle: Angle,
    ) -> Result<ShapeId, GeometryError> {
        let s = self.get(id)?;
        let res = s.rotate(origin.to_array(), axis.to_array(), angle.in_deg())?;
        self.insert(res)
    }

    /// Mirror image of a shape across the plane through `origin` with `normal`.
    pub fn mirror(&mut self, id: ShapeId, origin: DVec3, normal: DVec3) -> Result<ShapeId, GeometryError> {
        let s = self.get(id)?;
        let res = s.mirror(origin.to_array(), normal.to_array())?;
        self.insert(res)
    }

    /// Hollows a solid inward to wall `thickness`, opening the given faces (0-based).
    pub fn thick_solid(
        &mut self,
        id: ShapeId,
        removed_face_indices: &[u32],
        thickness: Length,
    ) -> Result<ShapeId, GeometryError> {
        let t = thickness.ensure_positive("thickness")?;
        let s = self.get(id)?;
        let res = s.thick_solid(removed_face_indices, t)?;
        self.insert(res)
    }

    /// Applies a composite [`Transform`] to a shape.
    pub fn apply_transform(
        &mut self,
        id: ShapeId,
        transform: &Transform,
    ) -> Result<ShapeId, GeometryError> {
        let mut current = self.get(id)?.try_clone()?;
        if (transform.scale_factor - 1.0).abs() > 1e-9 {
            current =
                current.scale(transform.rotation_origin.to_array(), transform.scale_factor)?;
        }
        if transform.rotation_angle_deg.abs() > 1e-9 {
            current = current.rotate(
                transform.rotation_origin.to_array(),
                transform.rotation_axis.to_array(),
                transform.rotation_angle_deg,
            )?;
        }
        if transform.translation.length_squared() > 1e-18 {
            current = current.translate(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            )?;
        }
        self.insert(current)
    }

    /// Fillets all edges of `id` with `radius`.
    pub fn fillet_all(&mut self, id: ShapeId, radius: Length) -> Result<ShapeId, GeometryError> {
        let r = radius.ensure_positive("radius")?;
        let s = self.get(id)?;
        let res = s.fillet_all_edges(r)?;
        self.insert(res)
    }

    /// Fillets selected edges (0-based indices) of `id` with `radius`.
    pub fn fillet_edges(
        &mut self,
        id: ShapeId,
        edge_indices: &[u32],
        radius: Length,
    ) -> Result<ShapeId, GeometryError> {
        let r = radius.ensure_positive("radius")?;
        let s = self.get(id)?;
        let res = s.fillet_edges(edge_indices, r)?;
        self.insert(res)
    }

    /// Chamfers all edges of `id` with `distance`.
    pub fn chamfer_all(&mut self, id: ShapeId, distance: Length) -> Result<ShapeId, GeometryError> {
        let d = distance.ensure_positive("distance")?;
        let s = self.get(id)?;
        let res = s.chamfer_all_edges(d)?;
        self.insert(res)
    }

    /// Chamfers selected edges (0-based indices) of `id` with `distance`.
    pub fn chamfer_edges(
        &mut self,
        id: ShapeId,
        edge_indices: &[u32],
        distance: Length,
    ) -> Result<ShapeId, GeometryError> {
        let d = distance.ensure_positive("distance")?;
        let s = self.get(id)?;
        let res = s.chamfer_edges(edge_indices, d)?;
        self.insert(res)
    }

    /// Tilts selected planar faces while keeping another face fixed.
    pub fn draft_faces(
        &mut self,
        id: ShapeId,
        face_indices: &[u32],
        neutral_face_index: u32,
        angle_deg: f64,
        reversed: bool,
    ) -> Result<ShapeId, GeometryError> {
        let s = self.get(id)?;
        let res = s.draft_faces(face_indices, neutral_face_index, angle_deg, reversed)?;
        self.insert(res)
    }

    /// Pushes or pulls a planar face along its normal by `distance`.
    pub fn push_pull_face(&mut self, id: ShapeId, face_index: u32, distance: f64) -> Result<ShapeId, GeometryError> {
        let res = self.get(id)?.push_pull_face(face_index, distance)?;
        self.insert(res)
    }

    /// Sweeps a circular profile from the origin along a straight path.
    pub fn sweep_circle(&mut self, radius: Length, direction: DVec3) -> Result<ShapeId, GeometryError> {
        let radius = radius.ensure_positive("sweep radius")?;
        self.insert(OcctShape::sweep_circle(radius, direction.to_array())?)
    }

    /// Returns the volume in $\text{mm}^3$.
    pub fn volume(&self, id: ShapeId) -> Result<f64, GeometryError> {
        Ok(self.get(id)?.volume()?)
    }

    /// Returns the surface area in $\text{mm}^2$.
    pub fn surface_area(&self, id: ShapeId) -> Result<f64, GeometryError> {
        Ok(self.get(id)?.surface_area()?)
    }

    /// Returns the axis-aligned [`BoundingBox`].
    pub fn bounding_box(&self, id: ShapeId) -> Result<BoundingBox, GeometryError> {
        let raw = self.get(id)?.bounding_box()?;
        Ok(BoundingBox::new(
            DVec3::new(raw.xmin, raw.ymin, raw.zmin),
            DVec3::new(raw.xmax, raw.ymax, raw.zmax),
        ))
    }

    /// Verifies topological validity of the shape via `BRepCheck_Analyzer`.
    pub fn is_valid(&self, id: ShapeId) -> Result<bool, GeometryError> {
        Ok(self.get(id)?.is_valid()?)
    }

    /// Runs OCCT Shape Healing on a copy of a stored shape.
    pub fn heal_shape(&mut self, id: ShapeId) -> Result<ShapeId, GeometryError> {
        let healed = self.get(id)?.heal()?;
        self.insert(healed)
    }

    /// Removes unnecessary same-domain face and edge splits from a copied shape.
    pub fn refine_shape(&mut self, id: ShapeId) -> Result<ShapeId, GeometryError> {
        let refined = self.get(id)?.refine()?;
        self.insert(refined)
    }

    /// Inspects the full topological structure and face/edge geometric signatures of `id`.
    pub fn inspect_shape(&self, id: ShapeId) -> Result<Shape, GeometryError> {
        let s = self.get(id)?;
        let counts = s.topology_counts()?;
        let volume_mm3 = s.volume()?;
        let surface_area_mm2 = s.surface_area()?;
        let bounds = self.bounding_box(id)?;

        let faces = s
            .inspect_faces()?
            .into_iter()
            .map(|f| Face {
                index: f.index,
                area_mm2: f.area,
                centroid: DVec3::new(f.cx, f.cy, f.cz),
                normal: DVec3::new(f.nx, f.ny, f.nz),
            })
            .collect();

        let edges = s
            .inspect_edges()?
            .into_iter()
            .map(|e| Edge {
                index: e.index,
                length_mm: e.length,
                centroid: DVec3::new(e.cx, e.cy, e.cz),
            })
            .collect();

        Ok(Shape {
            id,
            summary: ShapeSummary {
                solids: counts.solids,
                faces: counts.faces,
                edges: counts.edges,
                vertices: counts.vertices,
                volume_mm3,
                surface_area_mm2,
                bounds,
            },
            faces,
            edges,
        })
    }

    /// Resolves a semantic face selector (normal direction + optional approximate centroid)
    /// to a concrete 0-based `Face` index. Rejects ambiguous matches rather than guessing.
    pub fn resolve_face_by_signature(
        &self,
        id: ShapeId,
        expected_normal: DVec3,
        expected_centroid: Option<DVec3>,
        normal_tolerance_cos: f64,
    ) -> Result<Face, GeometryError> {
        let shape = self.inspect_shape(id)?;
        let target_n = expected_normal.normalize_or_zero();
        let mut candidates: Vec<(Face, f64)> = shape
            .faces
            .into_iter()
            .filter(|f| f.normal.dot(target_n) >= normal_tolerance_cos)
            .map(|f| {
                let dist = expected_centroid
                    .map(|c| f.centroid.distance(c))
                    .unwrap_or(0.0);
                (f, dist)
            })
            .collect();

        if candidates.is_empty() {
            return Err(GeometryError::ReferenceResolutionFailed(
                "No face matched expected normal direction".to_string(),
            ));
        }

        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        if candidates.len() >= 2 && (candidates[1].1 - candidates[0].1).abs() < 1e-4 {
            return Err(GeometryError::ReferenceResolutionFailed(format!(
                "Ambiguous semantic face reference between face #{} and face #{}",
                candidates[0].0.index, candidates[1].0.index
            )));
        }

        Ok(candidates[0].0)
    }

    /// Exports `id` to a STEP file.
    pub fn export_step(&self, id: ShapeId, path: impl AsRef<Path>) -> Result<(), GeometryError> {
        self.get(id)?.write_step(path)?;
        Ok(())
    }

    /// Imports a STEP file into the store and returns its [`ShapeId`].
    pub fn import_step(&mut self, path: impl AsRef<Path>) -> Result<ShapeId, GeometryError> {
        let s = OcctShape::read_step(path)?;
        self.insert(s)
    }

    /// Generates a [`TriMesh`] from `id` for 3D viewport rendering and picking.
    pub fn tessellate(
        &self,
        id: ShapeId,
        options: TessellationOptions,
    ) -> Result<TriMesh, GeometryError> {
        let raw = self
            .get(id)?
            .tessellate(options.linear_deflection_mm, options.angular_deflection_rad)?;

        let positions = raw.positions.as_chunks::<3>().0.to_vec();
        let normals = raw.normals.as_chunks::<3>().0.to_vec();
        let indices = raw.indices.as_chunks::<3>().0.to_vec();
        let edge_segments = raw
            .edge_positions
            .as_chunks::<6>()
            .0
            .iter()
            .map(|c| [[c[0], c[1], c[2]], [c[3], c[4], c[5]]])
            .collect();

        Ok(TriMesh {
            positions,
            normals,
            indices,
            triangle_face_ids: raw.triangle_face_ids,
            edge_segments,
            edge_segment_ids: raw.edge_segment_ids,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase1_primitives_and_booleans() {
        let mut store = ShapeStore::new();

        // Box 100 x 50 x 10 mm -> 50,000 mm^3
        let box_id = store
            .create_box(Length::mm(100.0), Length::mm(50.0), Length::mm(10.0))
            .expect("create_box");
        assert!((store.volume(box_id).unwrap() - 50_000.0).abs() < 1e-2);

        // Invalid dimensions rejected
        assert!(
            store
                .create_box(Length::mm(0.0), Length::mm(10.0), Length::mm(10.0))
                .is_err()
        );

        // Cylinder radius 5mm, height 20mm translated to center of box (50, 25, -5)
        let cyl_raw = store
            .create_cylinder(Length::mm(5.0), Length::mm(20.0))
            .expect("create_cylinder");
        let cyl_moved = store
            .translate(cyl_raw, 50.0, 25.0, -5.0)
            .expect("translate");

        // Cut cylinder hole from box
        let cut_id = store.cut(box_id, cyl_moved).expect("boolean cut");
        assert!(store.is_valid(cut_id).unwrap());

        let expected_hole_vol = std::f64::consts::PI * 5.0 * 5.0 * 10.0;
        let actual_vol = store.volume(cut_id).unwrap();
        assert!(
            (actual_vol - (50_000.0 - expected_hole_vol)).abs() < 1.0,
            "expected {}, got {}",
            50_000.0 - expected_hole_vol,
            actual_vol
        );

        // Remove temporary shapes and verify generational StaleShapeId protection
        store.remove(cyl_raw).expect("remove cyl_raw");
        assert_eq!(
            store.volume(cyl_raw),
            Err(GeometryError::StaleShapeId(cyl_raw))
        );
    }

    #[test]
    fn phase1_semantic_face_signature_resolution() {
        let mut store = ShapeStore::new();
        let box_id = store
            .create_box(Length::mm(100.0), Length::mm(60.0), Length::mm(10.0))
            .expect("box");

        // Resolve top face (+Z normal)
        let top_face = store
            .resolve_face_by_signature(box_id, DVec3::Z, None, 0.99)
            .expect("resolve +Z face");
        assert!((top_face.area_mm2 - 6000.0).abs() < 1e-2);
        assert!((top_face.centroid.z - 10.0).abs() < 1e-2);
    }
}
