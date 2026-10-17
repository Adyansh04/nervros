//! Keyframes: each camera's frame kept on disk when its view changes, so "when did the mug go" is
//! answered from the hours before, not the seconds a camera's history holds. A frame is kept when
//! it differs enough from that camera's last kept one; the oldest go once past the profile's age
//! or size.

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use image::GrayImage;
use tokio::task::JoinHandle;

use super::{Camera, Cameras};
use crate::profile::KeyframesConfig;

/// The side of the grey thumbnail two frames are compared by.
const THUMB: (u32, u32) = (32, 18);
/// The mean difference of the thumbnails, out of 255, that makes a frame worth keeping: a person
/// walking through or an object moved, not sensor noise or a flicker of light.
const CHANGE: f64 = 6.0;
/// How often the oldest frames are removed, in looks.
const PRUNE_EVERY: u32 = 30;

/// One kept frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Keyframe {
    /// The camera's name.
    pub camera: String,
    /// The frame's stamp, milliseconds since 1970.
    pub at_ms: u64,
    /// The JPEG.
    pub path: PathBuf,
}

/// The kept frames of every camera, and the task that keeps them.
#[derive(Debug)]
pub struct Keyframes {
    dir: PathBuf,
    keeper: Mutex<Option<JoinHandle<()>>>,
}

impl Keyframes {
    /// Keeps `cameras`' frames under `dir`, one folder per camera. Must be called inside a tokio
    /// runtime.
    #[must_use]
    pub fn start(config: KeyframesConfig, cameras: Arc<Cameras>, dir: PathBuf) -> Arc<Self> {
        let keeper = tokio::spawn(keep(config, cameras, dir.clone()));
        Arc::new(Self {
            dir,
            keeper: Mutex::new(Some(keeper)),
        })
    }

    /// The kept frames from `from_ms` to `to_ms`, oldest first: of `camera`, or of every one.
    #[must_use]
    pub fn between(&self, camera: Option<&str>, from_ms: u64, to_ms: u64) -> Vec<Keyframe> {
        let mut out: Vec<Keyframe> = cameras_in(&self.dir)
            .into_iter()
            .filter(|c| camera.is_none_or(|want| want == c))
            .flat_map(|c| frames_of(&self.dir, &c))
            .filter(|k| (from_ms..=to_ms).contains(&k.at_ms))
            .collect();
        out.sort_by_key(|k| k.at_ms);
        out
    }
}

impl Drop for Keyframes {
    fn drop(&mut self) {
        if let Some(keeper) = crate::lock(&self.keeper).take() {
            keeper.abort();
        }
    }
}

/// Looks at each camera's newest frame every `period_s`, keeping the ones that changed.
async fn keep(config: KeyframesConfig, cameras: Arc<Cameras>, dir: PathBuf) {
    // The profile's check keeps period_s finite and positive; 0.2 s is the floor of sense.
    let period =
        Duration::try_from_secs_f64(config.period_s.max(0.2)).unwrap_or(Duration::from_secs(2));
    let mut last: Vec<(String, f64, GrayImage)> = Vec::new();
    let mut looks = 0u32;
    // Said once a run of failures, not every look: a full disk fails every one.
    let mut failing = false;
    loop {
        tokio::time::sleep(period).await;
        for name in cameras.names() {
            let Ok(frame) = cameras.get(Some(name)).and_then(Camera::newest) else {
                continue;
            };
            let seen = last.iter().find(|(n, ..)| n == name);
            if seen.is_some_and(|(_, stamp, _)| *stamp >= frame.stamp_s) {
                continue;
            }
            let before = seen.map(|(.., thumb)| thumb.clone());
            let (name, folder, edge) = (name.to_owned(), dir.join(name), config.edge_px);
            // Tens of milliseconds of pixels and a file: off the threads that serve the stop.
            let kept = tokio::task::spawn_blocking(move || {
                let rgb = frame.to_rgb().map_err(|e| e.to_string())?;
                let thumb = thumbnail(&rgb);
                if before.is_some_and(|b| same_view(&b, &thumb)) {
                    return Ok(None);
                }
                save(&folder, &rgb, frame.stamp_s, edge).map_err(|e| e.to_string())?;
                Ok::<_, String>(Some((frame.stamp_s, thumb)))
            })
            .await
            .map_err(|e| e.to_string())
            .and_then(|kept| kept);
            match kept {
                Ok(Some((stamp, thumb))) => {
                    failing = false;
                    last.retain(|(n, ..)| *n != name);
                    last.push((name, stamp, thumb));
                }
                Ok(None) => {}
                Err(e) => {
                    if !failing {
                        tracing::warn!(error = %e, camera = %name, "a keyframe was not kept");
                    }
                    failing = true;
                }
            }
        }
        looks += 1;
        if looks.is_multiple_of(PRUNE_EVERY) {
            let (dir, keep_h, max_mb) = (dir.clone(), config.keep_h, config.max_mb);
            let now_ms = millis(crate::now_s());
            let _ = tokio::task::spawn_blocking(move || prune(&dir, now_ms, keep_h, max_mb)).await;
        }
    }
}

/// Whether two thumbnails show the same view: noise and a flicker of light aside.
pub(crate) fn same_view(a: &GrayImage, b: &GrayImage) -> bool {
    difference(a, b) < CHANGE
}

/// A frame's grey thumbnail, which a change of view shows in and noise does not.
pub(crate) fn thumbnail(rgb: &image::RgbImage) -> GrayImage {
    let grey = image::DynamicImage::ImageRgb8(rgb.clone()).to_luma8();
    image::imageops::resize(
        &grey,
        THUMB.0,
        THUMB.1,
        image::imageops::FilterType::Triangle,
    )
}

/// The mean absolute difference of two thumbnails, out of 255.
fn difference(a: &GrayImage, b: &GrayImage) -> f64 {
    let total: u64 = a
        .pixels()
        .zip(b.pixels())
        .map(|(p, q)| u64::from(p.0[0].abs_diff(q.0[0])))
        .sum();
    let count = u64::from(THUMB.0 * THUMB.1);
    #[expect(clippy::cast_precision_loss, reason = "a mean of at most 255")]
    let mean = total as f64 / count as f64;
    mean
}

/// Writes `rgb` as `<folder>/<ms>.jpg`, its longest side cut to `edge` px: whole when it shows
/// up, by a rename, but not synced to disk, as a frame lost to a crash costs nothing.
fn save(folder: &Path, rgb: &image::RgbImage, stamp_s: f64, edge: u32) -> std::io::Result<()> {
    std::fs::create_dir_all(folder)?;
    let small = nervros_ros::image::capped(rgb, edge);
    let jpeg = nervros_ros::image::encode_jpeg(&small, nervros_ros::image::JPEG_QUALITY)
        .map_err(std::io::Error::other)?;
    let name = millis(stamp_s);
    let partial = folder.join(format!("{name}.part"));
    std::fs::write(&partial, &jpeg)?;
    std::fs::rename(&partial, folder.join(format!("{name}.jpg")))
}

/// Removes the frames older than `keep_h`, then the oldest until all fit in `max_mb`.
fn prune(dir: &Path, now_ms: u64, keep_h: f64, max_mb: u64) {
    let oldest = now_ms.saturating_sub(millis(keep_h * 3600.0));
    let mut kept: Vec<(Keyframe, u64)> = Vec::new();
    for camera in cameras_in(dir) {
        for k in frames_of(dir, &camera) {
            if k.at_ms < oldest {
                let _ = std::fs::remove_file(&k.path);
            } else {
                let size = std::fs::metadata(&k.path).map_or(0, |m| m.len());
                kept.push((k, size));
            }
        }
    }
    kept.sort_by_key(|(k, _)| k.at_ms);
    let mut total: u64 = kept.iter().map(|(_, size)| size).sum();
    let most = max_mb.saturating_mul(1024 * 1024);
    for (k, size) in kept {
        if total <= most {
            break;
        }
        if std::fs::remove_file(&k.path).is_ok() {
            total -= size;
        }
    }
}

/// The cameras with a folder under `dir`.
fn cameras_in(dir: &Path) -> Vec<String> {
    std::fs::read_dir(dir)
        .map(|entries| {
            entries
                .filter_map(Result::ok)
                .filter(|e| e.path().is_dir())
                .filter_map(|e| e.file_name().into_string().ok())
                .collect()
        })
        .unwrap_or_default()
}

/// A camera's kept frames, in no order.
fn frames_of(dir: &Path, camera: &str) -> Vec<Keyframe> {
    std::fs::read_dir(dir.join(camera))
        .map(|entries| {
            entries
                .filter_map(Result::ok)
                .filter_map(|e| {
                    let path = e.path();
                    let at_ms = path.file_stem()?.to_str()?.parse().ok()?;
                    (path.extension()? == "jpg").then(|| Keyframe {
                        camera: camera.to_owned(),
                        at_ms,
                        path,
                    })
                })
                .collect()
        })
        .unwrap_or_default()
}

/// Seconds as whole milliseconds; none before 1970.
#[must_use]
pub fn millis(seconds: f64) -> u64 {
    std::time::Duration::try_from_secs_f64(seconds.max(0.0))
        .map_or(0, |d| u64::try_from(d.as_millis()).unwrap_or(u64::MAX))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::LookConfig;
    use bytes::Bytes;
    use nervros_ros::fake::FakeRobot;
    use nervros_ros::{Frame, RobotPort};

    fn frame(stamp_s: f64, shade: u8) -> Frame {
        Frame {
            stamp_s,
            frame_id: "camera".into(),
            width: 64,
            height: 48,
            encoding: "rgb8".into(),
            step: 64 * 3,
            is_bigendian: false,
            data: Bytes::from(vec![shade; 64 * 48 * 3]),
        }
    }

    #[tokio::test]
    async fn a_frame_is_kept_when_the_view_changes_and_found_by_its_time() {
        let fake = Arc::new(FakeRobot::new().with_frame("/camera", frame(10.0, 40)));
        let robot: Arc<dyn RobotPort> = Arc::clone(&fake) as Arc<dyn RobotPort>;
        let look: LookConfig = toml::from_str(
            "image = \"/camera\"\ndetections = { topic = \"/m\", type = \"canopy_msgs/msg/InstanceMaskArray\" }\n",
        )
        .unwrap();
        let cameras = Cameras::start(&look, &robot).unwrap();
        let dir = tempfile::tempdir().unwrap();
        let config = KeyframesConfig {
            period_s: 0.2,
            keep_h: 1.0,
            max_mb: 10,
            edge_px: 64,
        };
        let keyframes = Keyframes::start(config, cameras, dir.path().to_owned());
        let count = |n| {
            let keyframes = Arc::clone(&keyframes);
            async move {
                for _ in 0..50 {
                    if keyframes.between(None, 0, u64::MAX).len() == n {
                        return true;
                    }
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
                false
            }
        };
        assert!(count(1).await, "the first frame is kept");
        fake.set_frame("/camera", frame(11.0, 41));
        tokio::time::sleep(Duration::from_millis(700)).await;
        assert_eq!(
            keyframes.between(None, 0, u64::MAX).len(),
            1,
            "a flicker is not"
        );
        fake.set_frame("/camera", frame(12.0, 200));
        assert!(count(2).await, "a changed view is");
        let later = keyframes.between(Some("main"), 11_500, 13_000);
        assert_eq!(later.len(), 1);
        assert_eq!(later[0].at_ms, 12_000);
        assert!(keyframes.between(Some("other"), 0, u64::MAX).is_empty());
    }

    #[test]
    fn the_oldest_go_by_age_then_by_size() {
        let dir = tempfile::tempdir().unwrap();
        let folder = dir.path().join("main");
        std::fs::create_dir_all(&folder).unwrap();
        for ms in [1_000, 3_600_000, 7_000_000, 7_100_000] {
            std::fs::write(folder.join(format!("{ms}.jpg")), vec![0u8; 600_000]).unwrap();
        }
        prune(dir.path(), 7_200_000, 1.0, 1);
        let left: Vec<u64> = frames_of(dir.path(), "main")
            .into_iter()
            .map(|k| k.at_ms)
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect();
        assert_eq!(
            left,
            [7_100_000],
            "an hour old and more, then past a megabyte"
        );
    }
}
