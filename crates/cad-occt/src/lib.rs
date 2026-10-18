//! Safe Rust wrapper around Open CASCADE Technology (OCCT) via `cxx`.
//!
//! No raw OCCT C++ types are exposed outside this crate. All C++ exceptions
//! (`Standard_Failure` and `std::exception`) are caught at the FFI boundary
//! and converted into [`OcctError`].

use std::path::Path;

use cxx::UniquePtr;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[allow(clippy::too_many_arguments)]
#[cxx::bridge(namespace = "ferrocad")]
mod ffi {
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct RawBBox {
        pub xmin: f64,
        pub ymin: f64,
        pub zmin: f64,
        pub xmax: f64,
        pub ymax: f64,
        pub zmax: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct RawTopoCounts {
        pub solids: u32,
        pub faces: u32,
        pub edges: u32,
        pub vertices: u32,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct RawFaceInfo {
        pub index: u32,
        pub area: f64,
        pub cx: f64,
        pub cy: f64,
        pub cz: f64,
        pub nx: f64,
        pub ny: f64,
        pub nz: f64,
    }

    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct RawEdgeInfo {
        pub index: u32,
        pub length: f64,
        pub cx: f64,
        pub cy: f64,
        pub cz: f64,
    }

    #[derive(Debug, Clone, Default)]
    pub struct RawMeshResult {
        pub positions: Vec<f32>,
        pub normals: Vec<f32>,
        pub indices: Vec<u32>,
        pub triangle_face_ids: Vec<u32>,
        pub edge_positions: Vec<f32>,
        pub edge_segment_ids: Vec<u32>,
    }

    unsafe extern "C++" {
        include!("cad-occt/include/occt_bridge.h");

        type OcctShapeHandle;

        fn clone_shape(handle: &OcctShapeHandle) -> Result<UniquePtr<OcctShapeHandle>>;

        fn make_box(dx: f64, dy: f64, dz: f64) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_cylinder(radius: f64, height: f64) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_sphere(radius: f64) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_cone(radius1: f64, radius2: f64, height: f64)
        -> Result<UniquePtr<OcctShapeHandle>>;

        fn make_extrude_polygon(
            coords_xy: &[f64],
            depth: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_extrude_circle(radius: f64, depth: f64) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_profile_solid(
            segments: &[f64],
            amount: f64,
            revolve: bool,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_revolve_polygon(
            coords_xz: &[f64],
            angle_deg: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;

        fn boolean_fuse(
            a: &OcctShapeHandle,
            b: &OcctShapeHandle,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn boolean_cut(
            a: &OcctShapeHandle,
            b: &OcctShapeHandle,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn boolean_common(
            a: &OcctShapeHandle,
            b: &OcctShapeHandle,
        ) -> Result<UniquePtr<OcctShapeHandle>>;

        fn translate_shape(
            handle: &OcctShapeHandle,
            dx: f64,
            dy: f64,
            dz: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn rotate_shape(
            handle: &OcctShapeHandle,
            ox: f64,
            oy: f64,
            oz: f64,
            ax: f64,
            ay: f64,
            az: f64,
            angle_deg: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn scale_shape(
            handle: &OcctShapeHandle,
            cx: f64,
            cy: f64,
            cz: f64,
            factor: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn mirror_shape(
            handle: &OcctShapeHandle,
            ox: f64,
            oy: f64,
            oz: f64,
            nx: f64,
            ny: f64,
            nz: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn make_thick_solid(
            handle: &OcctShapeHandle,
            removed_face_indices: &[u32],
            thickness: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;

        fn fillet_all_edges(
            handle: &OcctShapeHandle,
            radius: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn fillet_selected_edges(
            handle: &OcctShapeHandle,
            edge_indices: &[u32],
            radius: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn chamfer_all_edges(
            handle: &OcctShapeHandle,
            distance: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn chamfer_selected_edges(
            handle: &OcctShapeHandle,
            edge_indices: &[u32],
            distance: f64,
        ) -> Result<UniquePtr<OcctShapeHandle>>;
        fn draft_faces(
            handle: &OcctShapeHandle,
            face_indices: &[u32],
            neutral_face_index: u32,
            angle_deg: f64,
            reversed: bool,
        ) -> Result<UniquePtr<OcctShapeHandle>>;

        fn compute_volume(handle: &OcctShapeHandle) -> Result<f64>;
        fn compute_surface_area(handle: &OcctShapeHandle) -> Result<f64>;
        fn compute_bounding_box(handle: &OcctShapeHandle) -> Result<RawBBox>;
        fn check_shape_valid(handle: &OcctShapeHandle) -> Result<bool>;
        fn get_topology_counts(handle: &OcctShapeHandle) -> Result<RawTopoCounts>;
        fn inspect_faces(handle: &OcctShapeHandle) -> Result<Vec<RawFaceInfo>>;
        fn inspect_edges(handle: &OcctShapeHandle) -> Result<Vec<RawEdgeInfo>>;

        fn write_step_file(handle: &OcctShapeHandle, file_path: &str) -> Result<()>;
        fn read_step_file(file_path: &str) -> Result<UniquePtr<OcctShapeHandle>>;

        fn tessellate_shape(
            handle: &OcctShapeHandle,
            linear_deflection: f64,
            angular_deflection: f64,
        ) -> Result<RawMeshResult>;
    }
}

pub use ffi::{RawBBox, RawEdgeInfo, RawFaceInfo, RawMeshResult, RawTopoCounts};

/// Error type returned by OCCT FFI operations.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Error)]
pub enum OcctError {
    #[error("OCCT kernel exception: {0}")]
    KernelException(String),
    #[error("Invalid path for STEP I/O: {0}")]
    InvalidPath(String),
    #[error("Null OCCT shape handle")]
    NullHandle,
}

impl From<cxx::Exception> for OcctError {
    fn from(err: cxx::Exception) -> Self {
        Self::KernelException(err.what().to_string())
    }
}

/// Safe opaque handle to an OCCT `TopoDS_Shape`.
///
/// Intentionally does not implement `Send` or `Sync` until thread-safety
/// requirements are explicitly verified, per `roadmap.md` section 1.4.
pub struct OcctShape {
    inner: UniquePtr<ffi::OcctShapeHandle>,
}

impl std::fmt::Debug for OcctShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OcctShape")
            .field("is_null", &self.inner.is_null())
            .finish()
    }
}

impl OcctShape {
    fn from_ptr(ptr: UniquePtr<ffi::OcctShapeHandle>) -> Result<Self, OcctError> {
        if ptr.is_null() {
            return Err(OcctError::NullHandle);
        }
        Ok(Self { inner: ptr })
    }

    fn as_ref(&self) -> Result<&ffi::OcctShapeHandle, OcctError> {
        self.inner.as_ref().ok_or(OcctError::NullHandle)
    }

    /// Deep-copies the underlying topological shape.
    pub fn try_clone(&self) -> Result<Self, OcctError> {
        let ptr = ffi::clone_shape(self.as_ref()?)?;
        Self::from_ptr(ptr)
    }

    /// Creates a box with dimensions `(dx, dy, dz)` in millimeters from the origin `(0, 0, 0)`.
    pub fn make_box(dx: f64, dy: f64, dz: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_box(dx, dy, dz)?)
    }

    /// Creates a cylinder with `radius` and `height` along the +Z axis.
    pub fn make_cylinder(radius: f64, height: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_cylinder(radius, height)?)
    }

    /// Creates a sphere with `radius` centered at the origin.
    pub fn make_sphere(radius: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_sphere(radius)?)
    }

    /// Creates a cone or frustum with bottom `radius1`, top `radius2`, and `height` along +Z.
    pub fn make_cone(radius1: f64, radius2: f64, height: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_cone(radius1, radius2, height)?)
    }

    /// Extrudes a closed 2D polygon on the XY plane along +Z by `depth`.
    pub fn make_extrude_polygon(points_xy: &[[f64; 2]], depth: f64) -> Result<Self, OcctError> {
        let mut flat = Vec::with_capacity(points_xy.len() * 2);
        for &[x, y] in points_xy {
            flat.push(x);
            flat.push(y);
        }
        Self::from_ptr(ffi::make_extrude_polygon(&flat, depth)?)
    }

    /// Extrudes a 2D circle of `radius` on the XY plane along +Z by `depth`.
    pub fn make_extrude_circle(radius: f64, depth: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_extrude_circle(radius, depth)?)
    }

    /// Builds a solid from a closed profile of exact lines, arcs and circles (see
    /// `make_profile_solid` in the bridge for the encoding): extruded along +Z by
    /// `amount` mm from the XY plane, or (`revolve`) drawn in the XZ plane and revolved
    /// `amount` degrees around Z.
    pub fn make_profile_solid(segments: &[f64], amount: f64, revolve: bool) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_profile_solid(segments, amount, revolve)?)
    }

    /// Revolves a closed 2D polygon on the XZ plane around the Z axis by `angle_deg` degrees.
    pub fn make_revolve_polygon(points_xz: &[[f64; 2]], angle_deg: f64) -> Result<Self, OcctError> {
        let mut flat = Vec::with_capacity(points_xz.len() * 2);
        for &[x, z] in points_xz {
            flat.push(x);
            flat.push(z);
        }
        Self::from_ptr(ffi::make_revolve_polygon(&flat, angle_deg)?)
    }

    /// Boolean union (`Fuse`) of two shapes.
    pub fn fuse(&self, other: &Self) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::boolean_fuse(self.as_ref()?, other.as_ref()?)?)
    }

    /// Boolean subtraction (`Cut`) of `other` from `self`.
    pub fn cut(&self, other: &Self) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::boolean_cut(self.as_ref()?, other.as_ref()?)?)
    }

    /// Boolean intersection (`Common`) of two shapes.
    pub fn common(&self, other: &Self) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::boolean_common(self.as_ref()?, other.as_ref()?)?)
    }

    /// Returns a new shape translated by `(dx, dy, dz)`.
    pub fn translate(&self, dx: f64, dy: f64, dz: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::translate_shape(self.as_ref()?, dx, dy, dz)?)
    }

    /// Returns a new shape rotated around an axis passing through `origin` with direction `axis` by `angle_deg`.
    pub fn rotate(
        &self,
        origin: [f64; 3],
        axis: [f64; 3],
        angle_deg: f64,
    ) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::rotate_shape(
            self.as_ref()?,
            origin[0],
            origin[1],
            origin[2],
            axis[0],
            axis[1],
            axis[2],
            angle_deg,
        )?)
    }

    /// Uniformly scales a shape around `center` by `factor`.
    pub fn scale(&self, center: [f64; 3], factor: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::scale_shape(
            self.as_ref()?,
            center[0],
            center[1],
            center[2],
            factor,
        )?)
    }

    /// Mirror image of the shape across the plane through `origin` with `normal`.
    pub fn mirror(&self, origin: [f64; 3], normal: [f64; 3]) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::mirror_shape(
            self.as_ref()?,
            origin[0],
            origin[1],
            origin[2],
            normal[0],
            normal[1],
            normal[2],
        )?)
    }

    /// Hollows the solid inward to wall `thickness`, opening the given faces (0-based).
    pub fn thick_solid(&self, removed_face_indices: &[u32], thickness: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::make_thick_solid(self.as_ref()?, removed_face_indices, thickness)?)
    }

    /// Fillets all edges of the shape with constant `radius`.
    pub fn fillet_all_edges(&self, radius: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::fillet_all_edges(self.as_ref()?, radius)?)
    }

    /// Fillets specific edges (0-based indices) with constant `radius`.
    pub fn fillet_edges(&self, edge_indices: &[u32], radius: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::fillet_selected_edges(
            self.as_ref()?,
            edge_indices,
            radius,
        )?)
    }

    /// Chamfers all edges of the shape with symmetric `distance`.
    pub fn chamfer_all_edges(&self, distance: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::chamfer_all_edges(self.as_ref()?, distance)?)
    }

    /// Chamfers specific edges (0-based indices) with symmetric `distance`.
    pub fn chamfer_edges(&self, edge_indices: &[u32], distance: f64) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::chamfer_selected_edges(
            self.as_ref()?,
            edge_indices,
            distance,
        )?)
    }

    /// Applies a draft angle to selected faces, keeping `neutral_face_index` fixed.
    pub fn draft_faces(
        &self,
        face_indices: &[u32],
        neutral_face_index: u32,
        angle_deg: f64,
        reversed: bool,
    ) -> Result<Self, OcctError> {
        Self::from_ptr(ffi::draft_faces(
            self.as_ref()?, face_indices, neutral_face_index, angle_deg, reversed,
        )?)
    }

    /// Computes volume of the shape in cubic millimeters ($\text{mm}^3$).
    pub fn volume(&self) -> Result<f64, OcctError> {
        Ok(ffi::compute_volume(self.as_ref()?)?)
    }

    /// Computes surface area of the shape in square millimeters ($\text{mm}^2$).
    pub fn surface_area(&self) -> Result<f64, OcctError> {
        Ok(ffi::compute_surface_area(self.as_ref()?)?)
    }

    /// Computes the axis-aligned bounding box of the shape.
    pub fn bounding_box(&self) -> Result<RawBBox, OcctError> {
        Ok(ffi::compute_bounding_box(self.as_ref()?)?)
    }

    /// Checks whether the topological shape is valid via `BRepCheck_Analyzer`.
    pub fn is_valid(&self) -> Result<bool, OcctError> {
        Ok(ffi::check_shape_valid(self.as_ref()?)?)
    }

    /// Returns counts of solids, faces, edges, and vertices in the shape.
    pub fn topology_counts(&self) -> Result<RawTopoCounts, OcctError> {
        Ok(ffi::get_topology_counts(self.as_ref()?)?)
    }

    /// Returns geometric descriptors for each face (area, centroid, outward normal).
    pub fn inspect_faces(&self) -> Result<Vec<RawFaceInfo>, OcctError> {
        Ok(ffi::inspect_faces(self.as_ref()?)?)
    }

    /// Returns geometric descriptors for each edge (length, centroid).
    pub fn inspect_edges(&self) -> Result<Vec<RawEdgeInfo>, OcctError> {
        Ok(ffi::inspect_edges(self.as_ref()?)?)
    }

    /// Exports the shape to a STEP (`.step` / `.stp`) file.
    pub fn write_step(&self, path: impl AsRef<Path>) -> Result<(), OcctError> {
        let path_ref = path.as_ref();
        let path_str = path_ref
            .to_str()
            .ok_or_else(|| OcctError::InvalidPath(path_ref.display().to_string()))?;
        ffi::write_step_file(self.as_ref()?, path_str)?;
        Ok(())
    }

    /// Reads a shape from a STEP (`.step` / `.stp`) file.
    pub fn read_step(path: impl AsRef<Path>) -> Result<Self, OcctError> {
        let path_ref = path.as_ref();
        let path_str = path_ref
            .to_str()
            .ok_or_else(|| OcctError::InvalidPath(path_ref.display().to_string()))?;
        Self::from_ptr(ffi::read_step_file(path_str)?)
    }

    /// Tessellates the B-Rep shape into triangles and boundary edges for 3D rendering and picking.
    pub fn tessellate(
        &self,
        linear_deflection: f64,
        angular_deflection: f64,
    ) -> Result<RawMeshResult, OcctError> {
        Ok(ffi::tessellate_shape(
            self.as_ref()?,
            linear_deflection,
            angular_deflection,
        )?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase0_box_volume_and_step_roundtrip() {
        let b = OcctShape::make_box(100.0, 60.0, 10.0).expect("make_box");
        assert!(b.is_valid().expect("is_valid"));

        let vol = b.volume().expect("volume");
        assert!(
            (vol - 60_000.0).abs() < 1e-3,
            "expected volume 60000.0, got {vol}"
        );

        let counts = b.topology_counts().expect("topology_counts");
        assert_eq!(counts.solids, 1);
        assert_eq!(counts.faces, 6);
        assert_eq!(counts.edges, 12);
        assert_eq!(counts.vertices, 8);

        let mesh = b.tessellate(0.5, 0.3).expect("tessellate");
        assert_eq!(mesh.triangle_face_ids.len(), 12); // 6 faces * 2 triangles
        assert_eq!(mesh.indices.len(), 36);

        let temp_step = std::env::temp_dir().join("ferrocad_phase0_test_box.step");
        b.write_step(&temp_step).expect("write_step");
        let loaded = OcctShape::read_step(&temp_step).expect("read_step");
        let _ = std::fs::remove_file(&temp_step);

        assert!(loaded.is_valid().expect("loaded valid"));
        let loaded_vol = loaded.volume().expect("loaded volume");
        assert!((loaded_vol - 60_000.0).abs() < 1e-2);
    }

    #[test]
    fn phase0_cpp_exception_caught_cleanly() {
        // Invalid zero/negative dimensions must return Err without crashing process
        assert!(OcctShape::make_box(0.0, 10.0, 10.0).is_err());
        assert!(OcctShape::make_box(-5.0, 10.0, 10.0).is_err());
        assert!(OcctShape::make_cylinder(0.0, 10.0).is_err());
        // Fillet with excessively large radius on 10mm box must fail with Err, not abort
        let b = OcctShape::make_box(10.0, 10.0, 10.0).expect("make_box");
        assert!(b.fillet_all_edges(50.0).is_err());
    }
}
