use cad_ui::FerroCadApp;

fn main() -> eframe::Result {
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("FerroCAD — Parametric 3D CAD (Rust + OCCT + wgpu)")
            .with_inner_size([1360.0, 840.0]),
        ..Default::default()
    };

    eframe::run_native(
        "FerroCAD",
        native_options,
        Box::new(|_cc| Ok(Box::new(FerroCadApp::new_with_mvp_demo()))),
    )
}
