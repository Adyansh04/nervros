//! Images the operator gives the agent, dropped on the window or pasted: each is kept as a
//! snapshot the `look` tool can answer about, shown in the chat when the message goes, and named
//! in the message as `[image sN]`.

use std::sync::Arc;

use nervros_core::look::SnapshotStore;
use nervros_core::session::Event;
use nervros_core::tools::ImageArtifact;
use rerun::external::egui::{self, RichText};
use rerun::external::re_ui::{UiExt as _, icons};

/// The longest edge kept: more is slower to send and no clearer to a vision model.
const EDGE_PX: u32 = 1280;

struct Attached {
    id: String,
    texture: egui::TextureHandle,
    artifact: ImageArtifact,
}

/// The images waiting to go with the next message.
pub struct Attachments {
    snapshots: Arc<SnapshotStore>,
    waiting: Vec<Attached>,
    /// Why the last image could not be taken.
    pub problem: Option<String>,
}

impl Attachments {
    /// None waiting; each one added is kept in `snapshots`.
    pub fn new(snapshots: Arc<SnapshotStore>) -> Self {
        Self {
            snapshots,
            waiting: Vec::new(),
            problem: None,
        }
    }

    /// Takes the images dropped on the window this frame.
    pub fn take_dropped(&mut self, ctx: &egui::Context) {
        for file in ctx.input(|i| i.raw.dropped_files.clone()) {
            let read = file
                .bytes()
                .and_then(|b| image::load_from_memory(&b).map_err(|e| e.to_string()));
            match read {
                Ok(img) => self.add(ctx, &img),
                Err(e) => self.problem = Some(format!("{}: {e}", file.path().display())),
            }
        }
    }

    /// Takes the image on the clipboard.
    pub fn paste(&mut self, ctx: &egui::Context) {
        let pasted = arboard::Clipboard::new()
            .and_then(|mut c| c.get_image())
            .map_err(|e| e.to_string())
            .and_then(|data| {
                let (w, h) = (
                    u32::try_from(data.width).map_err(|e| e.to_string())?,
                    u32::try_from(data.height).map_err(|e| e.to_string())?,
                );
                image::RgbaImage::from_raw(w, h, data.bytes.into_owned())
                    .ok_or_else(|| "the clipboard image is malformed".to_owned())
            });
        match pasted {
            Ok(rgba) => self.add(ctx, &image::DynamicImage::ImageRgba8(rgba)),
            Err(e) => self.problem = Some(format!("no image on the clipboard: {e}")),
        }
    }

    fn add(&mut self, ctx: &egui::Context, img: &image::DynamicImage) {
        let rgb = nervros_ros::image::capped(&img.to_rgb8(), EDGE_PX);
        let jpeg = match nervros_ros::image::encode_jpeg(&rgb, nervros_ros::image::JPEG_QUALITY) {
            Ok(j) => j,
            Err(e) => {
                self.problem = Some(e.to_string());
                return;
            }
        };
        let snapshot =
            self.snapshots
                .store(jpeg, rgb.dimensions(), nervros_core::now_s(), Vec::new());
        let (id, artifact) = (snapshot.id.clone(), snapshot.image.clone());
        let size = [rgb.width(), rgb.height()].map(|v| usize::try_from(v).unwrap_or(0));
        let pixels = egui::ColorImage::from_rgb(size, rgb.as_raw());
        let texture = ctx.load_texture(&id, pixels, egui::TextureOptions::LINEAR);
        self.waiting.push(Attached {
            id,
            texture,
            artifact,
        });
        self.problem = None;
    }

    /// The waiting images as small cards, each with a way to drop it.
    pub fn show(&mut self, ui: &mut egui::Ui) {
        if self.waiting.is_empty() && self.problem.is_none() {
            return;
        }
        let mut dropped = None;
        ui.horizontal_wrapped(|ui| {
            for (i, a) in self.waiting.iter().enumerate() {
                ui.add(
                    egui::Image::new(&a.texture)
                        .max_height(40.0)
                        .corner_radius(4),
                );
                ui.label(RichText::new(&a.id).small().color(ui.tokens().text_subdued));
                if ui
                    .small_icon_button(&icons::CLOSE_SMALL, "Remove this image")
                    .clicked()
                {
                    dropped = Some(i);
                }
            }
        });
        if let Some(i) = dropped {
            self.waiting.remove(i);
        }
        if let Some(problem) = &self.problem {
            ui.label(
                RichText::new(problem)
                    .small()
                    .color(ui.tokens().warn_fg_color),
            );
        }
        ui.add_space(4.0);
    }

    /// The message with the waiting images named in it, and the events that show them in the
    /// chat; nothing waits afterwards.
    pub fn send(&mut self, text: &str) -> (String, Vec<Event>) {
        if self.waiting.is_empty() {
            return (text.to_owned(), Vec::new());
        }
        let names: Vec<String> = self
            .waiting
            .iter()
            .map(|a| format!("[image {}]", a.id))
            .collect();
        let events = self
            .waiting
            .drain(..)
            .map(|a| Event::from(&a.artifact))
            .collect();
        (format!("{text} {}", names.join(" ")), events)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_image_is_kept_as_a_snapshot_and_named_in_the_message() {
        let ctx = egui::Context::default();
        let store = Arc::new(SnapshotStore::default());
        let mut attached = Attachments::new(Arc::clone(&store));
        let big = image::DynamicImage::new_rgb8(2560, 1440);
        attached.add(&ctx, &big);

        let (message, events) = attached.send("What is this?");

        assert_eq!(message, "What is this? [image s1]");
        let kept = store.get("s1").unwrap();
        assert_eq!((kept.image.width, kept.image.height), (1280, 720));
        assert!(matches!(&events[..], [Event::Snapshot { id, .. }] if id == "s1"));
        assert_eq!(attached.send("again").0, "again", "sent once");
    }
}
