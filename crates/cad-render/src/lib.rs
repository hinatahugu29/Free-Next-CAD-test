//! 3D Viewport Renderer powered by `wgpu`, CAD Camera Controller,
//! Subshape Raycast Picking, and Headless PNG Exporter (Phase 3).

use std::path::Path;

use bytemuck::{Pod, Zeroable};
use cad_core::{Document, FeatureId, FeatureState};
use cad_geometry::BoundingBox;
use glam::{Mat4, Vec3, Vec4};
use image::{ImageBuffer, Rgba};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use wgpu::util::DeviceExt;

#[derive(Debug, Error)]
pub enum RenderError {
    #[error("Failed to request wgpu adapter")]
    NoAdapter,
    #[error("Failed to request wgpu device: {0}")]
    DeviceRequest(String),
    #[error("Buffer mapping failed")]
    BufferMapFailed,
    #[error("Failed to save PNG image: {0}")]
    ImageSave(String),
}

/// Projection mode for the 3D CAD camera (Phase 3.2).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ProjectionMode {
    Perspective { fov_y_deg: f32 },
    Orthographic { scale: f32 },
}

impl Default for ProjectionMode {
    fn default() -> Self {
        Self::Perspective { fov_y_deg: 40.0 }
    }
}

/// Standard orthographic/isometric CAD camera presets (Phase 3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum StandardView {
    Isometric,
    Front,
    Back,
    Top,
    Bottom,
    Right,
    Left,
}

/// Viewport display style (Phase 3.2).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisplayMode {
    #[default]
    ShadedWithEdges,
    Shaded,
    Wireframe,
}

/// Interactive CAD camera supporting Orbit, Pan, Zoom, Fit All, Standard Views, and Perspective/Ortho toggle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    pub yaw_rad: f32,
    pub pitch_rad: f32,
    pub up: Vec3,
    pub projection: ProjectionMode,
    pub near: f32,
    pub far: f32,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            target: Vec3::new(50.0, 30.0, 5.0),
            distance: 180.0,
            yaw_rad: -0.85,
            pitch_rad: 0.60,
            up: Vec3::Z,
            projection: ProjectionMode::default(),
            near: 0.5,
            far: 10_000.0,
        }
    }
}

impl Camera {
    /// Computes the camera eye position in world space (Z-up CAD convention).
    pub fn eye_position(&self) -> Vec3 {
        let cos_p = self.pitch_rad.cos();
        let sin_p = self.pitch_rad.sin();
        let cos_y = self.yaw_rad.cos();
        let sin_y = self.yaw_rad.sin();
        let offset = Vec3::new(cos_p * cos_y, cos_p * sin_y, sin_p) * self.distance;
        self.target + offset
    }

    pub fn forward_dir(&self) -> Vec3 {
        (self.target - self.eye_position()).normalize_or_zero()
    }

    pub fn right_dir(&self) -> Vec3 {
        let f = self.forward_dir();
        let r = f.cross(self.up).normalize_or_zero();
        if r.length_squared() < 1e-5 {
            Vec3::X
        } else {
            r
        }
    }

    pub fn camera_up_dir(&self) -> Vec3 {
        self.right_dir()
            .cross(self.forward_dir())
            .normalize_or_zero()
    }

    pub fn view_matrix(&self) -> Mat4 {
        Mat4::look_at_rh(self.eye_position(), self.target, self.camera_up_dir())
    }

    pub fn projection_matrix(&self, aspect: f32) -> Mat4 {
        let safe_aspect = aspect.max(0.05);
        match self.projection {
            ProjectionMode::Perspective { fov_y_deg } => {
                Mat4::perspective_rh(fov_y_deg.to_radians(), safe_aspect, self.near, self.far)
            }
            ProjectionMode::Orthographic { scale } => {
                let half_h = (self.distance * 0.45 * scale).max(1.0);
                let half_w = half_h * safe_aspect;
                Mat4::orthographic_rh(-half_w, half_w, -half_h, half_h, -self.far, self.far)
            }
        }
    }

    pub fn view_proj_matrix(&self, aspect: f32) -> Mat4 {
        self.projection_matrix(aspect) * self.view_matrix()
    }

    /// Orbits the camera around `target`.
    pub fn orbit(&mut self, delta_yaw_rad: f32, delta_pitch_rad: f32) {
        self.yaw_rad += delta_yaw_rad;
        let limit = std::f32::consts::FRAC_PI_2 - 0.02;
        self.pitch_rad = (self.pitch_rad + delta_pitch_rad).clamp(-limit, limit);
    }

    /// Pans the camera target along the screen plane.
    pub fn pan(&mut self, delta_right_px: f32, delta_up_px: f32) {
        let factor = self.distance * 0.0015;
        let r = self.right_dir();
        let u = self.camera_up_dir();
        self.target += (-r * delta_right_px + u * delta_up_px) * factor;
    }

    /// Zooms the camera in/out (`factor < 1.0` zooms in, `factor > 1.0` zooms out).
    pub fn zoom(&mut self, factor: f32) {
        self.distance = (self.distance * factor).clamp(2.0, 8_000.0);
    }

    /// Fits the camera to encompass the given [`BoundingBox`].
    pub fn fit_bounding_box(&mut self, bbox: &BoundingBox) {
        let center = bbox.center();
        self.target = Vec3::new(center.x as f32, center.y as f32, center.z as f32);
        let diag = (bbox.diagonal_length() as f32).max(10.0);
        self.distance = diag * 1.45;
    }

    /// Snaps the camera orientation to a standard CAD view.
    pub fn set_standard_view(&mut self, view: StandardView) {
        match view {
            StandardView::Isometric => {
                self.yaw_rad = -0.85;
                self.pitch_rad = 0.60;
            }
            StandardView::Front => {
                self.yaw_rad = -std::f32::consts::FRAC_PI_2;
                self.pitch_rad = 0.0;
            }
            StandardView::Back => {
                self.yaw_rad = std::f32::consts::FRAC_PI_2;
                self.pitch_rad = 0.0;
            }
            StandardView::Top => {
                self.yaw_rad = -std::f32::consts::FRAC_PI_2;
                self.pitch_rad = std::f32::consts::FRAC_PI_2 - 0.025;
            }
            StandardView::Bottom => {
                self.yaw_rad = -std::f32::consts::FRAC_PI_2;
                self.pitch_rad = -std::f32::consts::FRAC_PI_2 + 0.025;
            }
            StandardView::Right => {
                self.yaw_rad = 0.0;
                self.pitch_rad = 0.0;
            }
            StandardView::Left => {
                self.yaw_rad = std::f32::consts::PI;
                self.pitch_rad = 0.0;
            }
        }
    }

    /// Toggles between Perspective and Orthographic projection.
    pub fn toggle_projection(&mut self) {
        self.projection = match self.projection {
            ProjectionMode::Perspective { .. } => ProjectionMode::Orthographic { scale: 1.0 },
            ProjectionMode::Orthographic { .. } => ProjectionMode::Perspective { fov_y_deg: 40.0 },
        };
    }

    /// Generates a world-space ray `(origin, direction)` from normalized viewport coordinates `(uv_x, uv_y)` in `[0, 1]`.
    pub fn screen_ray(&self, uv_x: f32, uv_y: f32, aspect: f32) -> (Vec3, Vec3) {
        let ndc_x = uv_x * 2.0 - 1.0;
        let ndc_y = 1.0 - uv_y * 2.0;
        let inv_vp = self.view_proj_matrix(aspect).inverse();
        let near_clip = Vec4::new(ndc_x, ndc_y, 0.0, 1.0);
        let far_clip = Vec4::new(ndc_x, ndc_y, 1.0, 1.0);
        let near_w = inv_vp * near_clip;
        let far_w = inv_vp * far_clip;
        let p_near = near_w.truncate() / near_w.w;
        let p_far = far_w.truncate() / far_w.w;
        let dir = (p_far - p_near).normalize_or_zero();
        (p_near, dir)
    }
}

/// Result of a 3D viewport picking operation (Phase 3.4).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PickHit {
    pub feature_id: FeatureId,
    pub face_index: u32,
    pub nearest_edge_index: Option<u32>,
    pub hit_point: Vec3,
    pub distance: f32,
}

/// Raycasts against all visible features in `document` to pick the closest `FeatureId`, `Face`, and nearby `Edge`.
pub fn pick_document(
    document: &Document,
    camera: &Camera,
    uv_x: f32,
    uv_y: f32,
    aspect: f32,
) -> Option<PickHit> {
    let (ray_origin, ray_dir) = camera.screen_ray(uv_x, uv_y, aspect);
    let mut best: Option<PickHit> = None;

    for feat in document.features.values().filter(|f| f.visible) {
        let Some(mesh) = feat.cached_mesh.as_ref().or(feat.last_valid_mesh.as_ref()) else {
            continue;
        };
        if let Some((dist, _tri_idx, face_index, hit_point)) = mesh.raycast(ray_origin, ray_dir)
            && (best.is_none() || dist < best.unwrap().distance)
        {
            // Also check if the hit point is close to any B-Rep boundary edge segment
            let edge_threshold = (camera.distance * 0.025).max(1.2);
            let mut nearest_edge: Option<(u32, f32)> = None;
            for (seg_idx, &[a, b]) in mesh.edge_segments.iter().enumerate() {
                let pa = Vec3::from_array(a);
                let pb = Vec3::from_array(b);
                let d_seg = point_to_segment_distance(hit_point, pa, pb);
                if d_seg <= edge_threshold
                    && (nearest_edge.is_none() || d_seg < nearest_edge.unwrap().1)
                {
                    let eid = mesh.edge_segment_ids.get(seg_idx).copied().unwrap_or(0);
                    nearest_edge = Some((eid, d_seg));
                }
            }

            best = Some(PickHit {
                feature_id: feat.id,
                face_index,
                nearest_edge_index: nearest_edge.map(|(id, _)| id),
                hit_point,
                distance: dist,
            });
        }
    }
    best
}

fn point_to_segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq <= 1e-8 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    let proj = a + ab * t;
    p.distance(proj)
}

/// Visual selection and rendering configuration for a frame.
#[derive(Debug, Clone, Default)]
pub struct ViewportRenderConfig {
    pub display_mode: DisplayMode,
    pub show_grid: bool,
    pub selected_feature: Option<FeatureId>,
    pub selected_face: Option<(FeatureId, u32)>,
    pub selected_edge: Option<(FeatureId, u32)>,
    /// Additional selected faces / edges (multi-selection), highlighted like the above.
    pub selected_faces: Vec<(FeatureId, u32)>,
    pub selected_edges: Vec<(FeatureId, u32)>,
}

impl ViewportRenderConfig {
    pub fn is_face_selected(&self, face: (FeatureId, u32)) -> bool {
        self.selected_face == Some(face) || self.selected_faces.contains(&face)
    }

    pub fn is_edge_selected(&self, edge: (FeatureId, u32)) -> bool {
        self.selected_edge == Some(edge) || self.selected_edges.contains(&edge)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct GpuVertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
struct GpuUniforms {
    view_proj: [[f32; 4]; 4],
    eye_pos: [f32; 4],
    light_dir: [f32; 4],
}

const CAD_SHADER_WGSL: &str = r#"
struct Uniforms {
    view_proj: mat4x4<f32>,
    eye_pos: vec4<f32>,
    light_dir: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> ubo: Uniforms;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_pos: vec3<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = ubo.view_proj * vec4<f32>(in.position, 1.0);
    out.world_pos = in.position;
    out.world_normal = in.normal;
    out.color = in.color;
    return out;
}

@fragment
fn fs_lit(in: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let v = normalize(ubo.eye_pos.xyz - in.world_pos);
    let l1 = normalize(ubo.light_dir.xyz);
    let l2 = normalize(vec3<f32>(-0.55, 0.45, 0.65));

    // Studio two-point CAD lighting + view-aligned headlamp
    let diff1 = max(dot(n, l1), 0.0) * 0.55;
    let diff2 = max(dot(n, l2), 0.0) * 0.25;
    let headlamp = max(abs(dot(n, v)), 0.0) * 0.30;
    let ambient = 0.22;

    let h = normalize(l1 + v);
    let spec = pow(max(dot(n, h), 0.0), 36.0) * 0.18;

    let lit_rgb = in.color.rgb * (ambient + diff1 + diff2 + headlamp) + vec3<f32>(spec);
    return vec4<f32>(clamp(lit_rgb, vec3<f32>(0.0), vec3<f32>(1.0)), in.color.a);
}

@fragment
fn fs_unlit(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}
"#;

/// Offscreen & viewport `wgpu` renderer capable of rendering any [`Document`] into an RGBA8 buffer or PNG file.
pub struct CadRenderer {
    device: wgpu::Device,
    queue: wgpu::Queue,
    tri_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
}

impl CadRenderer {
    /// Initializes a `wgpu` device and compiles the CAD triangle and wireframe/edge pipelines.
    pub fn new_headless() -> Result<Self, RenderError> {
        pollster::block_on(Self::new_async())
    }

    async fn new_async() -> Result<Self, RenderError> {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: None,
                force_fallback_adapter: false,
            })
            .await
            .ok_or(RenderError::NoAdapter)?;

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("FerroCAD wgpu Device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::default(),
                },
                None,
            )
            .await
            .map_err(|e| RenderError::DeviceRequest(e.to_string()))?;

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FerroCAD CAD Shader"),
            source: wgpu::ShaderSource::Wgsl(CAD_SHADER_WGSL.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Uniform Bind Group Layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("CAD Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let vertex_buffers = [wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GpuVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 12,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: 24,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }];

        let tri_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("CAD Triangle Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &vertex_buffers,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_lit"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8Unorm,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState {
                    constant: 2,
                    slope_scale: 1.5,
                    clamp: 0.0,
                },
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("CAD Line Pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &vertex_buffers,
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_unlit"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: wgpu::TextureFormat::Rgba8Unorm,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: wgpu::PipelineCompilationOptions::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::LessEqual,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Ok(Self {
            device,
            queue,
            tri_pipeline,
            line_pipeline,
            bind_group_layout,
        })
    }

    /// Renders `document` into an RGBA8 pixel buffer of size `(width, height)`.
    pub fn render_to_rgba(
        &self,
        document: &Document,
        camera: &Camera,
        config: &ViewportRenderConfig,
        width: u32,
        height: u32,
    ) -> Result<Vec<u8>, RenderError> {
        let w = width.max(16);
        let h = height.max(16);
        let aspect = (w as f32) / (h as f32);

        let mut tri_vertices: Vec<GpuVertex> = Vec::new();
        let mut line_vertices: Vec<GpuVertex> = Vec::new();

        if config.show_grid {
            append_grid_and_axes(&mut line_vertices, 150.0, 10.0);
        }

        for feat in document.features.values().filter(|f| f.visible) {
            let is_degraded = matches!(
                feat.state,
                FeatureState::Failed(_) | FeatureState::Blocked { .. }
            );
            let Some(mesh) = feat.cached_mesh.as_ref().or(feat.last_valid_mesh.as_ref()) else {
                continue;
            };

            let is_feat_selected = config.selected_feature == Some(feat.id);
            let base_color = if is_degraded {
                // Amber translucent tint when displaying last valid geometry of a Failed/Blocked feature
                [0.92, 0.55, 0.18, 0.78]
            } else if is_feat_selected {
                [0.38, 0.72, 0.98, 1.0]
            } else {
                feat.color_rgba
            };

            if config.display_mode != DisplayMode::Wireframe {
                for (tri_idx, &[i0, i1, i2]) in mesh.indices.iter().enumerate() {
                    let face_id = mesh.triangle_face_ids.get(tri_idx).copied().unwrap_or(0);
                    let tri_color = if config.is_face_selected((feat.id, face_id)) {
                        [0.98, 0.78, 0.22, 1.0] // Gold highlight for selected Face
                    } else {
                        base_color
                    };

                    for &vi in &[i0, i1, i2] {
                        let pos = mesh.positions.get(vi as usize).copied().unwrap_or([0.0; 3]);
                        let norm = mesh
                            .normals
                            .get(vi as usize)
                            .copied()
                            .unwrap_or([0.0, 0.0, 1.0]);
                        tri_vertices.push(GpuVertex {
                            position: pos,
                            normal: norm,
                            color: tri_color,
                        });
                    }
                }
            }

            if config.display_mode != DisplayMode::Shaded {
                for (seg_idx, &[a, b]) in mesh.edge_segments.iter().enumerate() {
                    let edge_id = mesh.edge_segment_ids.get(seg_idx).copied().unwrap_or(0);
                    let edge_color = if config.is_edge_selected((feat.id, edge_id)) {
                        [1.0, 0.35, 0.12, 1.0] // Bright orange highlight for selected Edge
                    } else if is_feat_selected {
                        [0.88, 0.96, 1.0, 1.0]
                    } else {
                        [0.08, 0.11, 0.16, 1.0]
                    };
                    line_vertices.push(GpuVertex {
                        position: a,
                        normal: [0.0, 0.0, 1.0],
                        color: edge_color,
                    });
                    line_vertices.push(GpuVertex {
                        position: b,
                        normal: [0.0, 0.0, 1.0],
                        color: edge_color,
                    });
                }
            }
        }

        let eye = camera.eye_position();
        let ubo = GpuUniforms {
            view_proj: camera.view_proj_matrix(aspect).to_cols_array_2d(),
            eye_pos: [eye.x, eye.y, eye.z, 1.0],
            light_dir: [0.45, -0.60, 0.85, 0.0],
        };

        let ubo_buffer = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("CAD Uniform Buffer"),
                contents: bytemuck::bytes_of(&ubo),
                usage: wgpu::BufferUsages::UNIFORM,
            });

        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CAD Bind Group"),
            layout: &self.bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: ubo_buffer.as_entire_binding(),
            }],
        });

        let color_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Offscreen Color Texture"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8Unorm,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_tex.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_tex = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Offscreen Depth Texture"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_tex.create_view(&wgpu::TextureViewDescriptor::default());

        let tri_buf = (!tri_vertices.is_empty()).then(|| {
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Triangle Vertex Buffer"),
                    contents: bytemuck::cast_slice(&tri_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                })
        });

        let line_buf = (!line_vertices.is_empty()).then(|| {
            self.device
                .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("Line Vertex Buffer"),
                    contents: bytemuck::cast_slice(&line_vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                })
        });

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("CAD Render Encoder"),
            });

        {
            let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("CAD Main RenderPass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.11,
                            g: 0.13,
                            b: 0.16,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            rpass.set_bind_group(0, &bind_group, &[]);
            if let Some(ref tb) = tri_buf {
                rpass.set_pipeline(&self.tri_pipeline);
                rpass.set_vertex_buffer(0, tb.slice(..));
                rpass.draw(0..(tri_vertices.len() as u32), 0..1);
            }
            if let Some(ref lb) = line_buf {
                rpass.set_pipeline(&self.line_pipeline);
                rpass.set_vertex_buffer(0, lb.slice(..));
                rpass.draw(0..(line_vertices.len() as u32), 0..1);
            }
        }

        let bytes_per_row = (w * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let readback_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Readback Buffer"),
            size: (bytes_per_row * h) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &color_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &readback_buf,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(h),
                },
            },
            wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
        );

        self.queue.submit(std::iter::once(encoder.finish()));

        let slice = readback_buf.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |res| {
            let _ = tx.send(res);
        });
        self.device.poll(wgpu::Maintain::Wait);
        rx.recv()
            .map_err(|_| RenderError::BufferMapFailed)?
            .map_err(|_| RenderError::BufferMapFailed)?;

        let mapped = slice.get_mapped_range();
        let mut rgba = Vec::with_capacity((w * h * 4) as usize);
        for row in 0..h {
            let start = (row * bytes_per_row) as usize;
            let end = start + (w * 4) as usize;
            rgba.extend_from_slice(&mapped[start..end]);
        }
        drop(mapped);
        readback_buf.unmap();

        Ok(rgba)
    }

    /// Renders `document` and saves the output directly as a PNG image on disk.
    pub fn render_to_png(
        &self,
        document: &Document,
        camera: &Camera,
        config: &ViewportRenderConfig,
        width: u32,
        height: u32,
        path: impl AsRef<Path>,
    ) -> Result<(), RenderError> {
        let rgba = self.render_to_rgba(document, camera, config, width, height)?;
        let img: ImageBuffer<Rgba<u8>, Vec<u8>> =
            ImageBuffer::from_raw(width.max(16), height.max(16), rgba)
                .ok_or_else(|| RenderError::ImageSave("buffer size mismatch".to_string()))?;
        img.save(path)
            .map_err(|e| RenderError::ImageSave(e.to_string()))
    }
}

fn append_grid_and_axes(lines: &mut Vec<GpuVertex>, half_extent: f32, step: f32) {
    let grid_color = [0.22, 0.25, 0.30, 0.65];
    let n = (half_extent / step) as i32;
    for i in -n..=n {
        let v = (i as f32) * step;
        if i == 0 {
            continue;
        }
        lines.push(GpuVertex {
            position: [v, -half_extent, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: grid_color,
        });
        lines.push(GpuVertex {
            position: [v, half_extent, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: grid_color,
        });
        lines.push(GpuVertex {
            position: [-half_extent, v, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: grid_color,
        });
        lines.push(GpuVertex {
            position: [half_extent, v, 0.0],
            normal: [0.0, 0.0, 1.0],
            color: grid_color,
        });
    }

    // X axis (Red), Y axis (Green), Z axis (Blue)
    let axes = [
        (
            [0.0, 0.0, 0.0],
            [half_extent, 0.0, 0.0],
            [0.92, 0.28, 0.28, 1.0],
        ),
        (
            [0.0, 0.0, 0.0],
            [0.0, half_extent, 0.0],
            [0.28, 0.85, 0.38, 1.0],
        ),
        (
            [0.0, 0.0, 0.0],
            [0.0, 0.0, half_extent * 0.6],
            [0.32, 0.58, 0.96, 1.0],
        ),
    ];
    for (start, end, col) in axes {
        lines.push(GpuVertex {
            position: start,
            normal: [0.0, 0.0, 1.0],
            color: col,
        });
        lines.push(GpuVertex {
            position: end,
            normal: [0.0, 0.0, 1.0],
            color: col,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cad_core::build_perforated_plate_mvp;
    use cad_geometry::{ShapeStore, TessellationOptions};

    #[test]
    fn phase3_camera_and_headless_render_and_picking() {
        let mut store = ShapeStore::new();
        let (mut ctrl, final_id) = build_perforated_plate_mvp().expect("mvp");
        ctrl.document
            .recompute(&mut store, TessellationOptions::default())
            .expect("recompute");

        let mut camera = Camera::default();
        if let Some(bbox) = ctrl.document.visible_bounding_box() {
            camera.fit_bounding_box(&bbox);
        }
        camera.set_standard_view(StandardView::Isometric);

        // Verify center raycast picks PerforatedPlate
        let hit = pick_document(&ctrl.document, &camera, 0.5, 0.5, 16.0 / 9.0)
            .expect("should hit plate at center");
        assert_eq!(hit.feature_id, final_id);

        // Verify orbit, pan, zoom, and projection toggle
        camera.orbit(0.1, -0.05);
        camera.pan(5.0, -3.0);
        camera.zoom(0.95);
        camera.toggle_projection();
        assert!(matches!(
            camera.projection,
            ProjectionMode::Orthographic { .. }
        ));
        camera.toggle_projection();

        // Verify wgpu headless rendering produces non-empty RGBA buffer
        let renderer = CadRenderer::new_headless().expect("headless wgpu");
        let cfg = ViewportRenderConfig {
            display_mode: DisplayMode::ShadedWithEdges,
            show_grid: true,
            selected_feature: Some(final_id),
            selected_face: Some((hit.feature_id, hit.face_index)),
            selected_edge: None,
            ..Default::default()
        };
        let rgba = renderer
            .render_to_rgba(&ctrl.document, &camera, &cfg, 320, 200)
            .expect("render_to_rgba");
        assert_eq!(rgba.len(), 320 * 200 * 4);
    }
}
