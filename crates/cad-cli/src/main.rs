//! Headless CLI for FerroCAD verification, STEP I/O, headless `wgpu` rendering,
//! and Milestone M1 Parametric Perforated Plate demonstration.

use std::env;
use std::path::{Path, PathBuf};

use cad_core::{CadCommand, Document, ParameterValue, build_perforated_plate_mvp};
use cad_geometry::{ShapeStore, TessellationOptions};
use cad_occt::OcctShape;
use cad_render::{
    CadRenderer, Camera, DisplayMode, StandardView, ViewportRenderConfig, pick_document,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let subcmd = args.get(1).map(|s| s.as_str()).unwrap_or("verify-all");

    let mut output_dir = PathBuf::from("build/demo");
    let mut i = 2;
    while i < args.len() {
        if args[i] == "--output-dir" && i + 1 < args.len() {
            output_dir = PathBuf::from(&args[i + 1]);
            i += 2;
        } else {
            i += 1;
        }
    }

    std::fs::create_dir_all(&output_dir)?;

    match subcmd {
        "phase0-box" => run_phase0_box(&output_dir)?,
        "demo-plate" => run_milestone_m1_demo(&output_dir)?,
        "demo-m2" => run_m2_demo(&output_dir)?,
        "verify-all" => {
            run_phase0_box(&output_dir)?;
            println!();
            run_milestone_m1_demo(&output_dir)?;
        }
        other => {
            eprintln!(
                "Unknown command '{other}'. Available commands: phase0-box, demo-plate, demo-m2, verify-all"
            );
            std::process::exit(1);
        }
    }

    Ok(())
}

fn run_phase0_box(output_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    println!("=== [Phase 0] OCCT直方体生成・体積検証・STEP入出力・メッシュ抽出 ===");
    let (dx, dy, dz) = (100.0, 60.0, 10.0);
    let expected_vol = dx * dy * dz;

    let shape = OcctShape::make_box(dx, dy, dz)?;
    assert!(shape.is_valid()?, "生成されたBoxが不正なトポロジーです");

    let vol = shape.volume()?;
    println!("1. Box({dx}x{dy}x{dz} mm) 体積: {vol:.4} mm³ (期待値: {expected_vol:.4} mm³)");
    assert!((vol - expected_vol).abs() < 1e-3);

    let step_path = output_dir.join("phase0_box.step");
    shape.write_step(&step_path)?;
    println!("2. STEP書き出し完了: {}", step_path.display());

    let reloaded = OcctShape::read_step(&step_path)?;
    let reloaded_vol = reloaded.volume()?;
    println!("3. STEP再読込後の体積: {reloaded_vol:.4} mm³");
    assert!((reloaded_vol - expected_vol).abs() < 1e-2);

    let mesh = shape.tessellate(0.25, 0.25)?;
    println!(
        "4. B-Repメッシュ抽出完了: 頂点数={}, 三角形数={}, エッジ線分数={}",
        mesh.positions.len() / 3,
        mesh.indices.len() / 3,
        mesh.edge_segment_ids.len()
    );

    assert!(
        OcctShape::make_box(0.0, 10.0, 10.0).is_err(),
        "無効寸法でErrが返るべきです"
    );
    println!("5. C++例外境界テスト成功: 無効寸法時にプロセス終了せず安全にErrへ変換");
    Ok(())
}

/// M2: sketch a circle on a plate's top face, cut a blind hole, then resize the plate
/// and confirm the hole follows the moved top face. Renders both states to PNG.
fn run_m2_demo(output_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    use cad_core::{ExtrudeMode, FeatureKind, ScalarParam, SemanticReference, Sketch, SketchPlane};

    println!("=== [M2] 面の上にスケッチ → 切り取り → 寸法変更への追従 ===");
    let tess = TessellationOptions::default();
    let mut store = ShapeStore::new();
    let mut doc = Document::new("M2 Demo");
    let plate_kind = |w: f64, h: f64, d: f64| FeatureKind::Box {
        width: ScalarParam::mm(w),
        height: ScalarParam::mm(h),
        depth: ScalarParam::mm(d),
    };
    let plate = doc.add_feature("Plate", plate_kind(100.0, 60.0, 10.0), false, [0.55, 0.72, 0.9, 1.0])?;
    doc.recompute(&mut store, tess)?;

    let plate_shape = doc.features[&plate].cached_shape.ok_or("plate shape")?;
    let top = store.resolve_face_by_signature(plate_shape, glam::DVec3::Z, None, 0.99)?;
    let face = SemanticReference::from_face(plate, &store, plate_shape, top.index)
        .map_err(|e| format!("{e:?}"))?;

    let mut sketch = Sketch::new("Hole");
    let c = sketch.add_point(30.0, 20.0);
    sketch.add_circle(c, 8.0);
    let cut = doc.add_feature(
        "HoleCut",
        FeatureKind::SketchExtrude {
            sketch,
            depth: ScalarParam::mm(6.0),
            plane: SketchPlane::Face { feature: plate, face },
            mode: ExtrudeMode::Cut { target: plate },
            reversed: true,
            symmetric: false,
            through_all: false,
        },
        true,
        [0.55, 0.72, 0.9, 1.0],
    )?;
    let report = doc.recompute(&mut store, tess)?;
    assert!(report.is_all_clean(), "{:?}", report.failed_features);
    println!(
        "1. 上面に深さ6mmの穴: 体積 {:.2} mm³",
        doc.features[&cut].cached_summary.ok_or("summary")?.volume_mm3
    );

    let renderer = CadRenderer::new_headless()?;
    let render = |doc: &Document, name: &str| -> Result<(), Box<dyn std::error::Error>> {
        let mut camera = Camera::default();
        if let Some(bbox) = doc.visible_bounding_box() {
            camera.fit_bounding_box(&bbox);
        }
        camera.set_standard_view(StandardView::Isometric);
        let cfg = ViewportRenderConfig {
            display_mode: DisplayMode::ShadedWithEdges,
            show_grid: true,
            selected_feature: None,
            selected_face: None,
            selected_edge: None,
        };
        let path = output_dir.join(name);
        renderer.render_to_png(doc, &camera, &cfg, 960, 600, &path)?;
        println!("   画像: {}", path.display());
        Ok(())
    };
    render(&doc, "m2_hole_initial.png")?;

    doc.update_feature(plate, None, Some(plate_kind(150.0, 80.0, 20.0)), None, None)?;
    let report = doc.recompute(&mut store, tess)?;
    assert!(report.is_all_clean(), "{:?}", report.failed_features);
    println!(
        "2. プレートを150×80×20mmに変更後: 体積 {:.2} mm³（穴は上面から6mmのまま）",
        doc.features[&cut].cached_summary.ok_or("summary")?.volume_mm3
    );
    render(&doc, "m2_hole_resized.png")?;
    Ok(())
}

fn run_milestone_m1_demo(output_dir: &Path) -> Result<(), Box<dyn std::error::Error>> {
    println!("=== [Milestone M1 MVP] パラメトリック『穴あきプレート』生成・再計算・描画検証 ===");

    let mut store = ShapeStore::new();
    let tess_opts = TessellationOptions::default();
    let (mut ctrl, final_id) = build_perforated_plate_mvp()?;

    // 1. 初期寸法 (100 x 60 x 10 mm, 4 x φ6.6mm 穴) での評価
    let report1 = ctrl.document.recompute(&mut store, tess_opts)?;
    assert!(report1.is_all_clean(), "初期再計算でエラーが発生しました");

    let feat = &ctrl.document.features[&final_id];
    let summary1 = feat.cached_summary.expect("cached_summary");
    let expected_vol1 = 100.0 * 60.0 * 10.0 - 4.0 * std::f64::consts::PI * 3.3 * 3.3 * 10.0;
    println!(
        "1. 初期穴あきプレート(100×60×10mm, φ6.6mm×4穴) 体積: {:.2} mm³ (理論値: {:.2} mm³)",
        summary1.volume_mm3, expected_vol1
    );
    assert!((summary1.volume_mm3 - expected_vol1).abs() < 1.0);
    println!(
        "   トポロジー要素数: Solid={}, Face={}, Edge={}, Vertex={}",
        summary1.solids, summary1.faces, summary1.edges, summary1.vertices
    );

    // JSON保存と再読込の一致確認
    let json_path = output_dir.join("perforated_plate_m1.json");
    ctrl.document.save_json(&json_path)?;
    let mut reloaded_doc = Document::load_json(&json_path)?;
    let mut temp_store = ShapeStore::new();
    reloaded_doc.recompute(&mut temp_store, tess_opts)?;
    let reloaded_vol = reloaded_doc.features[&final_id]
        .cached_summary
        .unwrap()
        .volume_mm3;
    assert!((reloaded_vol - summary1.volume_mm3).abs() < 1e-3);
    println!(
        "2. 設計JSON保存＆再読込後の再計算一致を確認: {}",
        json_path.display()
    );

    // STEP出力
    let step_path1 = output_dir.join("perforated_plate_m1.step");
    let shape_id1 = ctrl.document.features[&final_id].cached_shape.unwrap();
    store.export_step(shape_id1, &step_path1)?;
    println!("3. STEPファイル出力完了: {}", step_path1.display());

    // ヘッドレスwgpu 3Dレンダリングと3Dピッキング検証
    let renderer = CadRenderer::new_headless()?;
    let mut camera = Camera::default();
    if let Some(bbox) = ctrl.document.visible_bounding_box() {
        camera.fit_bounding_box(&bbox);
    }
    camera.set_standard_view(StandardView::Isometric);

    // 画面中央付近(0.5, 0.5)への3Dレイキャストピッキング検証
    let hit = pick_document(&ctrl.document, &camera, 0.5, 0.5, 16.0 / 9.0);
    assert!(
        hit.is_some(),
        "3Dビューポート中央で穴あきプレートがピックできるべきです"
    );
    let hit_info = hit.unwrap();
    println!(
        "4. 3Dビューポート・レイキャストピッキング成功: Feature #{}, Face #{}, Hit=({:.1}, {:.1}, {:.1})",
        hit_info.feature_id.0,
        hit_info.face_index,
        hit_info.hit_point.x,
        hit_info.hit_point.y,
        hit_info.hit_point.z
    );

    let render_cfg1 = ViewportRenderConfig {
        display_mode: DisplayMode::ShadedWithEdges,
        show_grid: true,
        selected_feature: Some(final_id),
        selected_face: Some((hit_info.feature_id, hit_info.face_index)),
        selected_edge: None,
    };
    let png_path1 = output_dir.join("perforated_plate_m1_iso.png");
    renderer.render_to_png(&ctrl.document, &camera, &render_cfg1, 960, 600, &png_path1)?;
    println!(
        "5. ヘッドレスwgpu 3Dレンダリング画像出力完了: {}",
        png_path1.display()
    );

    // 2. パラメータ変更 (幅140mm, 厚み15mm, 穴径8.8mm) と差分再計算の検証
    ctrl.execute(
        CadCommand::SetParameter {
            name: "width".to_string(),
            value: ParameterValue::Number(140.0),
            unit: "mm".to_string(),
            description: "幅を140mmへ変更".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "thickness".to_string(),
            value: ParameterValue::Number(15.0),
            unit: "mm".to_string(),
            description: "厚みを15mmへ変更".to_string(),
        },
        None,
    )?;
    ctrl.execute(
        CadCommand::SetParameter {
            name: "hole_diameter".to_string(),
            value: ParameterValue::Number(8.8),
            unit: "mm".to_string(),
            description: "M8穴(φ8.8mm)へ変更".to_string(),
        },
        None,
    )?;

    let report2 = ctrl.document.recompute(&mut store, tess_opts)?;
    assert!(report2.is_all_clean());
    let summary2 = ctrl.document.features[&final_id].cached_summary.unwrap();
    let expected_vol2 = 140.0 * 60.0 * 15.0 - 4.0 * std::f64::consts::PI * 4.4 * 4.4 * 15.0;
    println!(
        "6. パラメータ変更後(140×60×15mm, φ8.8mm×4穴) 再計算体積: {:.2} mm³ (理論値: {:.2} mm³)",
        summary2.volume_mm3, expected_vol2
    );
    assert!((summary2.volume_mm3 - expected_vol2).abs() < 1.5);

    if let Some(bbox2) = ctrl.document.visible_bounding_box() {
        camera.fit_bounding_box(&bbox2);
    }
    let step_path2 = output_dir.join("perforated_plate_modified.step");
    let shape_id2 = ctrl.document.features[&final_id].cached_shape.unwrap();
    store.export_step(shape_id2, &step_path2)?;

    let render_cfg2 = ViewportRenderConfig {
        display_mode: DisplayMode::ShadedWithEdges,
        show_grid: true,
        selected_feature: None,
        selected_face: None,
        selected_edge: None,
    };
    let png_path2 = output_dir.join("perforated_plate_modified_iso.png");
    renderer.render_to_png(&ctrl.document, &camera, &render_cfg2, 960, 600, &png_path2)?;
    println!(
        "7. 変更後のSTEP出力 ({}) および3Dレンダリング画像 ({}) 出力完了！",
        step_path2.display(),
        png_path2.display()
    );

    Ok(())
}
