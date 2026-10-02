//! The app's own look: a slate palette and type a size up from the viewer's, set on the `Ui` of
//! each panel NervROS draws (the bars, the chat, the dock) so the embedded viewer keeps `re_ui`'s
//! compact 12 px style, which its fixed-height rows are laid out for.

use rerun::external::egui::collapsing_header::CollapsingState;
use rerun::external::egui::{
    self, Color32, CornerRadius, Frame, Margin, Rect, RichText, Stroke, TextStyle, WidgetText,
};

/// The panels.
pub const BG: Color32 = Color32::from_rgb(0x12, 0x15, 0x1b);
/// The bars above and below, a shade darker so the panels read as one surface.
pub const BAR: Color32 = Color32::from_rgb(0x0d, 0x0f, 0x13);
/// Cards, chips and rows.
pub const SURFACE: Color32 = Color32::from_rgb(0x1a, 0x1e, 0x26);
/// What sits on a card: fields, code, the selected tab.
pub const RAISED: Color32 = Color32::from_rgb(0x23, 0x28, 0x32);
pub const BORDER: Color32 = Color32::from_rgb(0x2c, 0x32, 0x3d);
pub const TEXT: Color32 = Color32::from_rgb(0xe8, 0xeb, 0xf0);
/// Secondary text: still read, never skimmed past.
pub const DIM: Color32 = Color32::from_rgb(0xa7, 0xaf, 0xbc);
/// Metadata a reader can skip: ids, timings, the model's name.
pub const FAINT: Color32 = Color32::from_rgb(0x7a, 0x84, 0x92);
pub const ACCENT: Color32 = Color32::from_rgb(0x3b, 0x7c, 0xf0);
pub const SUCCESS: Color32 = Color32::from_rgb(0x3e, 0xc7, 0x8a);
pub const WARN: Color32 = Color32::from_rgb(0xf3, 0xa6, 0x2b);
pub const ERROR: Color32 = Color32::from_rgb(0xf2, 0x60, 0x5d);
pub const INFO: Color32 = Color32::from_rgb(0x62, 0xa8, 0xff);
/// What the robot itself reports, apart from what the agent says.
pub const ROBOT: Color32 = Color32::from_rgb(0x4c, 0xc9, 0xc4);
/// The stop button's fill: white text on it stays readable.
pub const STOP: Color32 = Color32::from_rgb(0xd2, 0x3b, 0x3b);

/// The app's style for `ui` and everything drawn in it.
pub fn apply(ui: &mut egui::Ui) {
    let style = ui.style_mut();
    for (text_style, font) in &mut style.text_styles {
        font.size = match text_style {
            TextStyle::Small => 12.0,
            TextStyle::Body => 14.0,
            TextStyle::Button => 13.0,
            TextStyle::Heading => 17.0,
            TextStyle::Monospace => 12.5,
            TextStyle::Name(_) => font.size,
        };
    }
    // The viewer's style extends labels; here text wraps at the panel's edge.
    style.wrap_mode = Some(egui::TextWrapMode::Wrap);
    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(10.0, 4.0);
    style.spacing.interact_size.y = 22.0;
    let v = &mut style.visuals;
    v.override_text_color = None;
    v.weak_text_color = Some(FAINT);
    v.panel_fill = BG;
    v.window_fill = SURFACE;
    v.faint_bg_color = SURFACE;
    v.extreme_bg_color = RAISED;
    v.text_edit_bg_color = Some(RAISED);
    v.code_bg_color = RAISED;
    v.hyperlink_color = INFO;
    v.warn_fg_color = WARN;
    v.error_fg_color = ERROR;
    v.selection.bg_fill = ACCENT.gamma_multiply(0.55);
    v.selection.stroke = Stroke::new(1.0, TEXT);
    v.widgets.noninteractive.fg_stroke.color = TEXT;
    v.widgets.noninteractive.bg_stroke.color = BORDER;
    v.widgets.inactive.fg_stroke.color = TEXT;
    v.widgets.inactive.bg_fill = RAISED;
    v.widgets.inactive.weak_bg_fill = RAISED;
    v.widgets.hovered.fg_stroke.color = TEXT;
    v.widgets.active.fg_stroke.color = Color32::WHITE;
}

/// A card: a raised block that holds one thing.
pub fn card() -> Frame {
    Frame::new()
        .fill(SURFACE)
        .stroke(Stroke::new(1.0, BORDER))
        .corner_radius(CornerRadius::same(10))
        .inner_margin(Margin::same(12))
}

/// A card with a strip of `colour` down its left edge, so its state reads at a glance from
/// across the room.
pub fn status_card<R>(
    ui: &mut egui::Ui,
    colour: Color32,
    add: impl FnOnce(&mut egui::Ui) -> R,
) -> R {
    let shown = card()
        .inner_margin(Margin {
            left: 16,
            right: 12,
            top: 12,
            bottom: 12,
        })
        .show(ui, add);
    let rect = shown.response.rect;
    let strip = Rect::from_min_max(rect.min, egui::pos2(rect.left() + 4.0, rect.bottom()));
    let corners = CornerRadius {
        nw: 10,
        sw: 10,
        ne: 0,
        se: 0,
    };
    ui.painter().rect_filled(strip, corners, colour);
    shown.inner
}

/// A small label over a group of rows, as a settings page has.
pub fn section(ui: &mut egui::Ui, text: &str) {
    ui.label(
        RichText::new(text.to_uppercase())
            .size(11.5)
            .extra_letter_spacing(0.6)
            .color(FAINT),
    );
}

/// A small triangle that says a row opens: pointing right while shut, down while open.
pub fn chevron(ui: &mut egui::Ui, open: bool) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
    let c = rect.center();
    let points = if open {
        vec![
            c + egui::vec2(-4.0, -2.0),
            c + egui::vec2(4.0, -2.0),
            c + egui::vec2(0.0, 3.0),
        ]
    } else {
        vec![
            c + egui::vec2(-2.0, -4.0),
            c + egui::vec2(3.0, 0.0),
            c + egui::vec2(-2.0, 4.0),
        ]
    };
    ui.painter()
        .add(egui::Shape::convex_polygon(points, FAINT, Stroke::NONE));
}

/// A row that opens on a click to show more, its label cut to one line: egui's collapsing
/// header lays its label out unwrapped, which widens the panel to fit the longest.
pub fn disclosure(
    ui: &mut egui::Ui,
    id: egui::Id,
    label: impl Into<WidgetText>,
    body: impl FnOnce(&mut egui::Ui),
) -> egui::Response {
    let mut state = CollapsingState::load_with_default_open(ui.ctx(), id, false);
    let row = ui
        .horizontal(|ui| {
            chevron(ui, state.is_open());
            ui.add(egui::Label::new(label).truncate().selectable(false));
        })
        .response;
    let row = ui.interact(row.rect, id.with("row"), egui::Sense::click());
    if row.clicked() {
        state.toggle(ui);
    }
    state.show_body_unindented(ui, |ui| {
        ui.indent(id.with("body"), body);
    });
    row
}
