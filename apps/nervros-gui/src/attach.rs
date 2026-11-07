//! Images the operator gives the agent, dropped on the window or pasted: each is kept as a
//! snapshot the `look` tool can answer about, shown in the chat when the message goes, and named
//! in the message as `[image sN]`.

use std::io::Cursor;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use crate::theme;
use nervros_core::session::Event;
use nervros_core::tools::ImageArtifact;
use nervros_core::vision::SnapshotStore;
use rerun::external::egui::{self, RichText};
use rerun::external::re_ui::{UiExt as _, icons};

/// The longest edge kept: more is slower to send and no clearer to a vision model.
const EDGE_PX: u32 = 1280;
/// A dropped file past this size is refused before it is read.
const FILE_BYTES: u64 = 25 * 1024 * 1024;
/// At most this many images are read at once, from one drop or several.
const READING: usize = 4;
/// The most memory one image's decoding may take, checked from its header before decoding.
const DECODE_BYTES: u64 = 200 * 1024 * 1024;

struct Attached {
    id: String,
    texture: egui::TextureHandle,
    artifact: ImageArtifact,
}

/// The images waiting to go with the next message.
pub struct Attachments {
    snapshots: Arc<SnapshotStore>,
    waiting: Vec<Attached>,
    /// Images being read, decoded and scaled off the UI thread, which keeps drawing and reading
    /// the stop key meanwhile. Left to end on their own when the window closes: each is bounded
    /// by the caps above.
    reading: Vec<JoinHandle<Result<Attached, String>>>,
    /// Why the last image could not be taken.
    pub problem: Option<String>,
}

impl Attachments {
    /// None waiting; each one added is kept in `snapshots`.
    pub fn new(snapshots: Arc<SnapshotStore>) -> Self {
        Self {
            snapshots,
            waiting: Vec::new(),
            reading: Vec::new(),
            problem: None,
        }
    }

    /// Takes the images dropped on the window this frame, and the ones read since.
    pub fn take_dropped(&mut self, ctx: &egui::Context) {
        let (read, still): (Vec<_>, Vec<_>) = std::mem::take(&mut self.reading)
            .into_iter()
            .partition(JoinHandle::is_finished);
        self.reading = still;
        for handle in read {
            match handle.join() {
                Ok(Ok(a)) => {
                    self.waiting.push(a);
                    self.problem = None;
                }
                Ok(Err(e)) => self.problem = Some(e),
                Err(_) => self.problem = Some("reading the image failed".to_owned()),
            }
        }
        for file in ctx.input(|i| i.raw.dropped_files.clone()) {
            let name = file.path().display().to_string();
            self.start(ctx, move || {
                let size = std::fs::metadata(file.path()).map_or(0, |m| m.len());
                if size > FILE_BYTES {
                    return Err(format!(
                        "{name}: {} MB, over the {} MB an image may be",
                        size / (1024 * 1024),
                        FILE_BYTES / (1024 * 1024)
                    ));
                }
                file.bytes()
                    .and_then(|b| decode(&b, DECODE_BYTES))
                    .map_err(|e| format!("{name}: {e}"))
            });
        }
        if !self.reading.is_empty() {
            ctx.request_repaint_after(Duration::from_millis(100));
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
            Ok(rgba) => self.start(ctx, move || Ok(image::DynamicImage::ImageRgba8(rgba))),
            Err(e) => self.problem = Some(format!("no image on the clipboard: {e}")),
        }
    }

    /// Reads an image with `read` and attaches it, off the UI thread; refused while as many are
    /// being read as may be.
    fn start(
        &mut self,
        ctx: &egui::Context,
        read: impl FnOnce() -> Result<image::DynamicImage, String> + Send + 'static,
    ) {
        if self.reading.len() >= READING {
            self.problem = Some(format!(
                "still reading {READING} images: drop more once they show"
            ));
            return;
        }
        let (ctx, snapshots) = (ctx.clone(), Arc::clone(&self.snapshots));
        let spawned = std::thread::Builder::new()
            .name("nervros-attach".to_owned())
            .spawn(move || {
                let done = read().and_then(|img| attach(&ctx, &snapshots, &img));
                ctx.request_repaint();
                done
            });
        match spawned {
            Ok(handle) => self.reading.push(handle),
            Err(e) => self.problem = Some(format!("cannot read the image: {e}")),
        }
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
                ui.label(RichText::new(&a.id).small().color(theme::DIM));
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
            ui.label(RichText::new(problem).small().color(theme::WARN));
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

/// An image file's pixels, its header read first: one whose decoding would take more than
/// `max_alloc` bytes is refused before it is decoded.
fn decode(bytes: &[u8], max_alloc: u64) -> Result<image::DynamicImage, String> {
    let mut reader = image::ImageReader::new(Cursor::new(bytes))
        .with_guessed_format()
        .map_err(|e| e.to_string())?;
    let mut limits = image::Limits::default();
    limits.max_alloc = Some(max_alloc);
    reader.limits(limits);
    reader.decode().map_err(|e| e.to_string())
}

/// `img`, scaled to [`EDGE_PX`], kept as a snapshot and made a texture to show.
fn attach(
    ctx: &egui::Context,
    snapshots: &SnapshotStore,
    img: &image::DynamicImage,
) -> Result<Attached, String> {
    let rgb = nervros_ros::image::capped(&img.to_rgb8(), EDGE_PX);
    let jpeg = nervros_core::vision::jpeg(&rgb)?;
    let snapshot = snapshots.store(jpeg, rgb.dimensions(), nervros_core::now_s(), Vec::new());
    let (id, artifact) = (snapshot.id.clone(), snapshot.image.clone());
    let size = [rgb.width(), rgb.height()].map(|v| usize::try_from(v).unwrap_or(0));
    let pixels = egui::ColorImage::from_rgb(size, rgb.as_raw());
    let texture = ctx.load_texture(&id, pixels, egui::TextureOptions::LINEAR);
    Ok(Attached {
        id,
        texture,
        artifact,
    })
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
        attached.waiting.push(attach(&ctx, &store, &big).unwrap());

        let (message, events) = attached.send("What is this?");

        assert_eq!(message, "What is this? [image s1]");
        let kept = store.get("s1").unwrap();
        assert_eq!((kept.image.width, kept.image.height), (1280, 720));
        assert!(matches!(&events[..], [Event::Snapshot { id, .. }] if id == "s1"));
        assert_eq!(attached.send("again").0, "again", "sent once");
    }

    #[test]
    fn an_image_whose_decoding_would_take_too_much_is_refused_from_its_header() {
        let mut png = Vec::new();
        image::DynamicImage::new_rgb8(64, 64)
            .write_to(&mut Cursor::new(&mut png), image::ImageFormat::Png)
            .unwrap();
        assert!(decode(&png, 1 << 20).is_ok());
        // 64 x 64 RGB is 12 kB to decode.
        assert!(decode(&png, 4096).is_err());
    }
}
