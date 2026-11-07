//! What the window's tests share: the viewer's style, and snapshots that tolerate CI's renderer.

use rerun::external::egui;

/// Applies `re_ui`'s style for tests that draw widgets without the viewer.
pub fn style_for_tests(ctx: &egui::Context) {
    rerun::external::re_ui::apply_style_and_install_loaders(ctx);
}

/// How far CI's software rasterizer may differ from the GPU the snapshots come from: its
/// anti-aliasing moves edge pixels a little, while a moved or missing element moves many.
const CI_THRESHOLD: f32 = 4.0;
/// Pixels CI's renderer may draw past [`CI_THRESHOLD`]: the edges of text and rounded corners.
const CI_PIXELS: usize = 2000;

/// Compares a render with its stored snapshot. CI draws with the lavapipe software rasterizer,
/// which anti-aliases differently: there the match is looser, but a changed screen still fails.
pub fn compare<S>(
    harness: &mut egui_kittest::Harness<'_, S>,
    name: &str,
    options: &egui_kittest::SnapshotOptions,
) {
    if std::env::var_os("CI").is_some() {
        let loose = options
            .clone()
            .threshold(CI_THRESHOLD)
            .max_failed_pixels(CI_PIXELS);
        harness.try_snapshot_options(name, &loose).unwrap();
    } else {
        harness.try_snapshot_options(name, options).unwrap();
    }
}
