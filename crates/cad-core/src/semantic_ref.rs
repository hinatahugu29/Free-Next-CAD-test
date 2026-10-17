//! Semantic Reference System for Stable Subshape (`Face` / `Edge`) Identification (Phase 5B).
//!
//! Addresses the Topological Naming Problem (TNP) by separating `FeatureId` from
//! `SubshapeId` and combining geometric signatures (outward normal, normalized/relative
//! centroid within the shape's bounding box, and area/length hints) with explicit
//! ambiguity detection and failure reporting.

use cad_geometry::{BoundingBox, Edge, Face, ShapeId, ShapeStore, SubshapeKind};
use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::FeatureId;

/// Detailed reason why a [`SemanticReference`] failed to resolve after a model modification.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ResolutionFailureReason {
    /// Target feature or shape does not exist in `ShapeStore`.
    MissingTargetShape(String),
    /// No topological element matched the required geometric orientation/normal criteria.
    NoMatchingSubshape { description: String },
    /// Multiple topological elements matched with nearly identical scores (e.g., symmetric geometry
    /// or a split face without a disambiguating centroid hint). Refuses to guess.
    AmbiguousCandidates {
        candidate_indices: Vec<u32>,
        score_delta: f64,
    },
}

/// Stable semantic reference to a topological `Face` or `Edge` produced by a feature (Phase 5B).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SemanticReference {
    pub target_feature: FeatureId,
    pub kind: SubshapeKind,
    /// Expected outward unit normal (used when `kind == SubshapeKind::Face`).
    pub expected_normal: Option<DVec3>,
    /// Expected normalized position `[0.0 ..= 1.0]^3` inside the shape's `BoundingBox`
    /// so that uniform or non-uniform dimension changes preserve relative face/edge location!
    pub normalized_bbox_centroid: Option<DVec3>,
    /// Expected world-space centroid as secondary tie-breaker.
    pub world_centroid_hint: Option<DVec3>,
    /// Expected area ($\text{mm}^2$) or length ($\text{mm}$) hint.
    pub metric_hint: Option<f64>,
    /// Minimum cosine similarity for face normal matching (default `0.95`).
    pub normal_cos_threshold: f64,
}

impl SemanticReference {
    /// Captures a semantic reference from a specific `face_index` of `shape_id` at design time.
    pub fn from_face(
        target_feature: FeatureId,
        store: &ShapeStore,
        shape_id: ShapeId,
        face_index: u32,
    ) -> Result<Self, ResolutionFailureReason> {
        let shape = store
            .inspect_shape(shape_id)
            .map_err(|e| ResolutionFailureReason::MissingTargetShape(e.to_string()))?;
        let face = shape
            .faces
            .iter()
            .find(|f| f.index == face_index)
            .ok_or_else(|| ResolutionFailureReason::NoMatchingSubshape {
                description: format!("Face #{face_index} does not exist on shape"),
            })?;

        let rel = normalize_in_bbox(face.centroid, &shape.summary.bounds);
        Ok(Self {
            target_feature,
            kind: SubshapeKind::Face,
            expected_normal: Some(face.normal.normalize_or_zero()),
            normalized_bbox_centroid: Some(rel),
            world_centroid_hint: Some(face.centroid),
            metric_hint: Some(face.area_mm2),
            normal_cos_threshold: 0.95,
        })
    }

    /// Captures a semantic reference from a specific `edge_index` of `shape_id` at design time.
    pub fn from_edge(
        target_feature: FeatureId,
        store: &ShapeStore,
        shape_id: ShapeId,
        edge_index: u32,
    ) -> Result<Self, ResolutionFailureReason> {
        let shape = store
            .inspect_shape(shape_id)
            .map_err(|e| ResolutionFailureReason::MissingTargetShape(e.to_string()))?;
        let edge = shape
            .edges
            .iter()
            .find(|e| e.index == edge_index)
            .ok_or_else(|| ResolutionFailureReason::NoMatchingSubshape {
                description: format!("Edge #{edge_index} does not exist on shape"),
            })?;

        let rel = normalize_in_bbox(edge.centroid, &shape.summary.bounds);
        Ok(Self {
            target_feature,
            kind: SubshapeKind::Edge,
            expected_normal: None,
            normalized_bbox_centroid: Some(rel),
            world_centroid_hint: Some(edge.centroid),
            metric_hint: Some(edge.length_mm),
            normal_cos_threshold: 0.95,
        })
    }

    /// Resolves this semantic reference against a newly recomputed `shape_id`, returning the matched [`Face`].
    pub fn resolve_face(
        &self,
        store: &ShapeStore,
        shape_id: ShapeId,
    ) -> Result<Face, ResolutionFailureReason> {
        let shape = store
            .inspect_shape(shape_id)
            .map_err(|e| ResolutionFailureReason::MissingTargetShape(e.to_string()))?;

        let mut scored: Vec<(Face, f64)> = Vec::new();
        for face in shape.faces {
            if let Some(target_n) = self.expected_normal {
                let cos_sim = face
                    .normal
                    .normalize_or_zero()
                    .dot(target_n.normalize_or_zero());
                if cos_sim < self.normal_cos_threshold {
                    continue;
                }
            }

            let mut penalty = 0.0;
            if let Some(target_rel) = self.normalized_bbox_centroid {
                let actual_rel = normalize_in_bbox(face.centroid, &shape.summary.bounds);
                penalty += actual_rel.distance(target_rel) * 10.0;
            }
            if let Some(target_world) = self.world_centroid_hint {
                let diag = shape.summary.bounds.diagonal_length().max(1.0);
                penalty += (face.centroid.distance(target_world) / diag) * 2.0;
            }
            if let Some(target_n) = self.expected_normal {
                let cos_sim = face
                    .normal
                    .normalize_or_zero()
                    .dot(target_n.normalize_or_zero());
                penalty += (1.0 - cos_sim) * 5.0;
            }

            scored.push((face, penalty));
        }

        if scored.is_empty() {
            return Err(ResolutionFailureReason::NoMatchingSubshape {
                description: "No face satisfied semantic orientation/position constraints"
                    .to_string(),
            });
        }

        scored.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        if scored.len() >= 2 {
            let delta = (scored[1].1 - scored[0].1).abs();
            if delta < 1e-4 {
                return Err(ResolutionFailureReason::AmbiguousCandidates {
                    candidate_indices: vec![scored[0].0.index, scored[1].0.index],
                    score_delta: delta,
                });
            }
        }

        Ok(scored[0].0)
    }

    /// Resolves this semantic reference against a newly recomputed `shape_id`, returning the matched [`Edge`].
    pub fn resolve_edge(
        &self,
        store: &ShapeStore,
        shape_id: ShapeId,
    ) -> Result<Edge, ResolutionFailureReason> {
        let shape = store
            .inspect_shape(shape_id)
            .map_err(|e| ResolutionFailureReason::MissingTargetShape(e.to_string()))?;

        let mut scored: Vec<(Edge, f64)> = Vec::new();
        for edge in shape.edges {
            let mut penalty = 0.0;
            if let Some(target_rel) = self.normalized_bbox_centroid {
                let actual_rel = normalize_in_bbox(edge.centroid, &shape.summary.bounds);
                penalty += actual_rel.distance(target_rel) * 10.0;
            }
            if let Some(target_world) = self.world_centroid_hint {
                let diag = shape.summary.bounds.diagonal_length().max(1.0);
                penalty += (edge.centroid.distance(target_world) / diag) * 2.0;
            }
            scored.push((edge, penalty));
        }

        if scored.is_empty() {
            return Err(ResolutionFailureReason::NoMatchingSubshape {
                description: "Shape contains no edges".to_string(),
            });
        }

        scored.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        if scored.len() >= 2 {
            let delta = (scored[1].1 - scored[0].1).abs();
            if delta < 1e-4 {
                return Err(ResolutionFailureReason::AmbiguousCandidates {
                    candidate_indices: vec![scored[0].0.index, scored[1].0.index],
                    score_delta: delta,
                });
            }
        }

        Ok(scored[0].0)
    }
}

fn normalize_in_bbox(pt: DVec3, bbox: &BoundingBox) -> DVec3 {
    let sz = bbox.size();
    let rx = if sz.x.abs() > 1e-9 {
        (pt.x - bbox.min.x) / sz.x
    } else {
        0.5
    };
    let ry = if sz.y.abs() > 1e-9 {
        (pt.y - bbox.min.y) / sz.y
    } else {
        0.5
    };
    let rz = if sz.z.abs() > 1e-9 {
        (pt.z - bbox.min.z) / sz.z
    } else {
        0.5
    };
    DVec3::new(rx, ry, rz)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_geometry::Length;

    #[test]
    fn phase5b_regression_suite_all_6_cases() {
        let mut store = ShapeStore::new();

        // Case 1: Box dimension change preserves top (+Z) and right (+X) face references
        let box1 = store
            .create_box(Length::mm(100.0), Length::mm(60.0), Length::mm(10.0))
            .unwrap();
        let top_init = store
            .resolve_face_by_signature(box1, DVec3::Z, None, 0.99)
            .unwrap();
        let sem_top =
            SemanticReference::from_face(FeatureId(1), &store, box1, top_init.index).unwrap();

        let box2 = store
            .create_box(Length::mm(180.0), Length::mm(90.0), Length::mm(25.0))
            .unwrap();
        let resolved_top = sem_top
            .resolve_face(&store, box2)
            .expect("resolve after resize");
        assert!((resolved_top.centroid.z - 25.0).abs() < 1e-2);
        assert!((resolved_top.area_mm2 - 180.0 * 90.0).abs() < 1.0);

        // Case 2: Adding a hole to the plate preserves top face reference
        let hole_cyl = store
            .create_cylinder(Length::mm(5.0), Length::mm(30.0))
            .unwrap();
        let hole_pos = store.translate(hole_cyl, 30.0, 20.0, -5.0).unwrap();
        let box_with_hole = store.cut(box1, hole_pos).unwrap();
        let resolved_after_hole = sem_top
            .resolve_face(&store, box_with_hole)
            .expect("resolve after hole cut");
        assert!((resolved_after_hole.centroid.z - 10.0).abs() < 1e-2);

        // Case 3: Adding a Fillet on an edge preserves top face reference
        let filleted_box = store
            .fillet_edges(box1, &[0], Length::mm(2.0))
            .expect("fillet edge 0");
        let resolved_after_fillet = sem_top
            .resolve_face(&store, filleted_box)
            .expect("resolve after fillet");
        assert!((resolved_after_fillet.centroid.z - 10.0).abs() < 1e-2);

        // Case 4 & 5: Boolean splits top face into two symmetric halves ->
        // An un-hinted center reference MUST detect ambiguity (`AmbiguousCandidates`),
        // while a left/right hinted reference resolves to the intended side!
        let slot_cutter = store
            .create_box(Length::mm(10.0), Length::mm(100.0), Length::mm(20.0))
            .unwrap();
        let slot_centered = store.translate(slot_cutter, 45.0, -10.0, 5.0).unwrap();
        let split_top_box = store.cut(box1, slot_centered).unwrap();

        let ambiguous_ref = SemanticReference {
            target_feature: FeatureId(1),
            kind: SubshapeKind::Face,
            expected_normal: Some(DVec3::Z),
            normalized_bbox_centroid: None,
            world_centroid_hint: None,
            metric_hint: None,
            normal_cos_threshold: 0.99,
        };
        assert!(matches!(
            ambiguous_ref.resolve_face(&store, split_top_box),
            Err(ResolutionFailureReason::AmbiguousCandidates { .. })
        ));

        // Disambiguated reference targeting the left (+Z, x ~ 22.5) top face
        let left_top_ref = SemanticReference {
            target_feature: FeatureId(1),
            kind: SubshapeKind::Face,
            expected_normal: Some(DVec3::Z),
            normalized_bbox_centroid: Some(DVec3::new(0.22, 0.5, 1.0)),
            world_centroid_hint: Some(DVec3::new(22.5, 30.0, 10.0)),
            metric_hint: None,
            normal_cos_threshold: 0.99,
        };
        let left_face = left_top_ref
            .resolve_face(&store, split_top_box)
            .expect("resolve left split face");
        assert!(left_face.centroid.x < 45.0);

        // Case 6: Sketch reference plane missing normal (e.g., querying tilted normal that doesn't exist)
        // reports explicit `NoMatchingSubshape` reason
        let missing_plane_ref = SemanticReference {
            target_feature: FeatureId(1),
            kind: SubshapeKind::Face,
            expected_normal: Some(DVec3::new(1.0, 1.0, 1.0).normalize()),
            normalized_bbox_centroid: None,
            world_centroid_hint: None,
            metric_hint: None,
            normal_cos_threshold: 0.99,
        };
        assert!(matches!(
            missing_plane_ref.resolve_face(&store, box1),
            Err(ResolutionFailureReason::NoMatchingSubshape { .. })
        ));
    }
}
