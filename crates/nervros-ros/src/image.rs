//! Camera frames: a `sensor_msgs/Image` held as bytes, converted to RGB and JPEG on demand.
//!
//! Pixels never pass through JSON. Colour encodings become RGB; depth (`16UC1` millimetres,
//! `32FC1` metres) becomes a grey image with near bright and far dark.

use bytes::Bytes;
use image::{ImageBuffer, Rgb, RgbImage};

/// One image message.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    /// Header stamp, seconds since the epoch (ROS time).
    pub stamp_s: f64,
    /// Header frame id.
    pub frame_id: String,
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// The ROS encoding string, such as `rgb8`.
    pub encoding: String,
    /// Bytes per row, which may include padding.
    pub step: u32,
    /// Multi-byte samples are big-endian.
    pub is_bigendian: bool,
    /// The pixel data.
    pub data: Bytes,
}

/// A frame that cannot be converted.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum FrameError {
    /// The encoding is not one NervROS converts.
    #[error("unsupported image encoding `{0}`")]
    Encoding(String),
    /// The buffer is shorter than width, height and step say.
    #[error("image data is {got} bytes, expected at least {want}")]
    Short {
        /// Bytes present.
        got: usize,
        /// Bytes needed.
        want: usize,
    },
    /// JPEG encoding failed.
    #[error("JPEG encoding failed: {0}")]
    Jpeg(String),
}

/// The depth range shown, in metres; values outside are clipped.
const DEPTH_RANGE_M: (f32, f32) = (0.2, 6.0);

impl Frame {
    fn row(&self, y: u32) -> &[u8] {
        let start = y as usize * self.step as usize;
        &self.data[start..start + self.step as usize]
    }

    /// Converts to RGB.
    ///
    /// # Errors
    ///
    /// Unsupported encodings and truncated buffers.
    pub fn to_rgb(&self) -> Result<RgbImage, FrameError> {
        let (w, h) = (self.width, self.height);
        let bytes_per_pixel = match self.encoding.as_str() {
            "rgb8" | "bgr8" => 3,
            "rgba8" | "bgra8" | "32FC1" => 4,
            "mono8" | "8UC1" => 1,
            "16UC1" | "mono16" => 2,
            other => return Err(FrameError::Encoding(other.to_owned())),
        };
        let want = self.step as usize * h as usize;
        if self.data.len() < want || (self.step as usize) < w as usize * bytes_per_pixel {
            return Err(FrameError::Short {
                got: self.data.len(),
                want,
            });
        }
        let mut out: RgbImage = ImageBuffer::new(w, h);
        for y in 0..h {
            let row = self.row(y);
            for x in 0..w {
                let i = x as usize * bytes_per_pixel;
                let px = match self.encoding.as_str() {
                    "rgb8" | "rgba8" => [row[i], row[i + 1], row[i + 2]],
                    "bgr8" | "bgra8" => [row[i + 2], row[i + 1], row[i]],
                    "mono8" | "8UC1" => [row[i]; 3],
                    "16UC1" | "mono16" => {
                        let raw = [row[i], row[i + 1]];
                        let mm = if self.is_bigendian {
                            u16::from_be_bytes(raw)
                        } else {
                            u16::from_le_bytes(raw)
                        };
                        [depth_grey(f32::from(mm) / 1000.0); 3]
                    }
                    _ => {
                        let raw = [row[i], row[i + 1], row[i + 2], row[i + 3]];
                        let m = if self.is_bigendian {
                            f32::from_be_bytes(raw)
                        } else {
                            f32::from_le_bytes(raw)
                        };
                        [depth_grey(m); 3]
                    }
                };
                out.put_pixel(x, y, Rgb(px));
            }
        }
        Ok(out)
    }
}

/// Near is bright, far is dark; zero, NaN and infinity (no reading) are black.
fn depth_grey(metres: f32) -> u8 {
    let (near, far) = DEPTH_RANGE_M;
    if !metres.is_finite() || metres <= 0.0 {
        return 0;
    }
    let t = ((metres.clamp(near, far) - near) / (far - near)).clamp(0.0, 1.0);
    // The value is in 0..=255 by construction.
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "clamped to 0..=255"
    )]
    let v = (255.0 - t * 235.0).round() as u8;
    v
}

/// Encodes an RGB image as JPEG.
///
/// # Errors
///
/// [`FrameError::Jpeg`] from the encoder.
pub fn encode_jpeg(image: &RgbImage, quality: u8) -> Result<Vec<u8>, FrameError> {
    let mut out = Vec::new();
    let encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, quality);
    image
        .write_with_encoder(encoder)
        .map_err(|e| FrameError::Jpeg(e.to_string()))?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(encoding: &str, w: u32, h: u32, step: u32, data: Vec<u8>) -> Frame {
        Frame {
            stamp_s: 0.0,
            frame_id: "camera".into(),
            width: w,
            height: h,
            encoding: encoding.into(),
            step,
            is_bigendian: false,
            data: Bytes::from(data),
        }
    }

    #[test]
    fn swaps_bgr_and_honours_row_padding() {
        // 2x1 bgr8 with 2 bytes of padding per row.
        let f = frame("bgr8", 2, 1, 8, vec![1, 2, 3, 4, 5, 6, 0, 0]);
        let rgb = f.to_rgb().unwrap();
        assert_eq!(rgb.get_pixel(0, 0).0, [3, 2, 1]);
        assert_eq!(rgb.get_pixel(1, 0).0, [6, 5, 4]);
    }

    #[test]
    fn depth_is_near_bright_far_dark_invalid_black() {
        let mm = |v: u16| v.to_le_bytes();
        let data = [mm(300), mm(5_000), mm(0)].concat();
        let rgb = frame("16UC1", 3, 1, 6, data).to_rgb().unwrap();
        let (near, far, none) = (
            rgb.get_pixel(0, 0).0[0],
            rgb.get_pixel(1, 0).0[0],
            rgb.get_pixel(2, 0).0[0],
        );
        assert!(near > far && far > none && none == 0, "{near} {far} {none}");
    }

    #[test]
    fn rejects_short_buffers_and_unknown_encodings() {
        assert!(matches!(
            frame("rgb8", 2, 2, 6, vec![0; 6]).to_rgb(),
            Err(FrameError::Short { .. })
        ));
        assert!(matches!(
            frame("yuv422", 1, 1, 2, vec![0; 2]).to_rgb(),
            Err(FrameError::Encoding(_))
        ));
    }

    #[test]
    fn encodes_jpeg() {
        let rgb = frame("rgb8", 4, 4, 12, vec![200; 48]).to_rgb().unwrap();
        let jpeg = encode_jpeg(&rgb, 85).unwrap();
        assert_eq!(&jpeg[..2], &[0xFF, 0xD8]);
    }
}
