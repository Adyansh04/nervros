//! What the window's tests share: the viewer's style, and snapshots that tolerate CI's renderer.

use rerun::external::egui;

/// Applies `re_ui`'s style for tests that draw widgets without the viewer.
pub fn style_for_tests(ctx: &egui::Context) {
    rerun::external::re_ui::apply_style_and_install_loaders(ctx);
}

/// Compares a render with its stored snapshot. CI draws with the lavapipe software rasterizer,
/// which anti-aliases differently from the GPU the snapshots come from, so there a difference is
/// reported and the test only proves the screen renders.
pub fn compare<S>(
    harness: &mut egui_kittest::Harness<'_, S>,
    name: &str,
    options: &egui_kittest::SnapshotOptions,
) {
    let result = harness.try_snapshot_options(name, options);
    if std::env::var_os("CI").is_some() {
        if let Err(e) = result {
            eprintln!("snapshot {name} differs on this renderer: {e}");
        }
    } else {
        result.unwrap();
    }
}
