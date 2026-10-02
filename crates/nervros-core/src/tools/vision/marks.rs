//! Drawing on a frame: numbered marks, tints, close-ups, and the marks as JSON.

use image::{Rgb, RgbImage};
use serde_json::{Value, json};

/// A close-up's long side at least.
pub(crate) const CLOSE_UP_PX: u32 = 384;

use super::detect::Instance;

/// Distinct colours for marks, readable on camera images.
pub(crate) const PALETTE: [[u8; 3]; 12] = [
    [230, 25, 75],
    [60, 180, 75],
    [255, 225, 25],
    [0, 130, 200],
    [245, 130, 48],
    [145, 30, 180],
    [70, 240, 240],
    [240, 50, 230],
    [210, 245, 60],
    [250, 190, 212],
    [0, 128, 128],
    [170, 110, 40],
];

/// 5x7 digit glyphs, one bit per pixel, top row first.
const DIGITS: [[u8; 7]; 10] = [
    [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
    [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
    [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
    [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
    [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
    [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
    [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
    [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
    [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
    [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
];

fn put(img: &mut RgbImage, at: (i64, i64), colour: [u8; 3]) {
    if let (Ok(x), Ok(y)) = (u32::try_from(at.0), u32::try_from(at.1))
        && x < img.width()
        && y < img.height()
    {
        img.put_pixel(x, y, Rgb(colour));
    }
}

/// A rectangle in pixels; may lie partly outside the image.
#[derive(Debug, Clone, Copy)]
struct Rect {
    left: i64,
    top: i64,
    width: i64,
    height: i64,
}

fn fill(img: &mut RgbImage, r: Rect, colour: [u8; 3]) {
    for row in r.top..r.top + r.height {
        for col in r.left..r.left + r.width {
            put(img, (col, row), colour);
        }
    }
}

/// Draws a number as a filled badge with white digits, scaled so digits are about 21 px tall.
pub(crate) fn badge(img: &mut RgbImage, at: (i64, i64), number: usize, colour: [u8; 3]) {
    const SCALE: i64 = 3;
    let digits: Vec<usize> = number
        .to_string()
        .bytes()
        .map(|b| usize::from(b - b'0'))
        .collect();
    let count = i64::try_from(digits.len()).unwrap_or(1);
    let size = Rect {
        left: at.0,
        top: at.1,
        width: count * 6 * SCALE + 2 * SCALE,
        height: 9 * SCALE,
    };
    fill(img, size, colour);
    for (i, digit) in digits.iter().enumerate() {
        let origin = at.0 + SCALE + i64::try_from(i).unwrap_or(0) * 6 * SCALE;
        for (row, bits) in (0_i64..).zip(DIGITS[*digit]) {
            for col in 0..5 {
                if bits & (0x10 >> col) != 0 {
                    let dot = Rect {
                        left: origin + col * SCALE,
                        top: at.1 + SCALE + row * SCALE,
                        width: SCALE,
                        height: SCALE,
                    };
                    fill(img, dot, [255, 255, 255]);
                }
            }
        }
    }
}

/// Blends a mask colour into the pixels a mask covers: three parts image, two parts colour.
pub(crate) fn tint(img: &mut RgbImage, inst: &Instance, colour: [u8; 3]) {
    let Some(mask) = &inst.mask else { return };
    let (bx, by, bw, bh) = inst.bbox;
    for row in 0..bh {
        for col in 0..bw {
            if mask[(row * bw + col) as usize] == 0 {
                continue;
            }
            let (x, y) = (bx + col, by + row);
            if x < img.width() && y < img.height() {
                let pixel = img.get_pixel_mut(x, y);
                for (channel, c) in pixel.0.iter_mut().zip(colour) {
                    *channel = u8::try_from((u16::from(*channel) * 3 + u16::from(c) * 2) / 5)
                        .unwrap_or(u8::MAX);
                }
            }
        }
    }
}

/// Draws the marks: a tinted mask where there is one, a box outline, and a numbered badge.
pub fn draw_marks(img: &mut RgbImage, instances: &[Instance]) {
    for (i, inst) in instances.iter().enumerate() {
        let colour = PALETTE[i % PALETTE.len()];
        tint(img, inst, colour);
        let (bx, by, bw, bh) = inst.bbox;
        let (left, top) = (i64::from(bx), i64::from(by));
        let (right, bottom) = (left + i64::from(bw), top + i64::from(bh));
        for t in 0..2 {
            for col in left..=right {
                put(img, (col, top + t), colour);
                put(img, (col, bottom - t), colour);
            }
            for row in top..=bottom {
                put(img, (left + t, row), colour);
                put(img, (right - t, row), colour);
            }
        }
        badge(img, (left, (top - 27).max(0)), i + 1, colour);
    }
}

/// A mark's box with a quarter of its size around it, at least 48 px a side, and enlarged so its
/// long side has at least `CLOSE_UP_PX`: a small mug's label is a few pixels in the full frame.
pub(crate) fn close_up(img: &RgbImage, (x, y, w, h): (u32, u32, u32, u32)) -> RgbImage {
    let margin = |side: u32| (side / 4).max(24);
    let (mx, my) = (margin(w), margin(h));
    let x0 = x.saturating_sub(mx);
    let y0 = y.saturating_sub(my);
    let x1 = (x + w + mx).min(img.width());
    let y1 = (y + h + my).min(img.height());
    let crop = image::imageops::crop_imm(img, x0, y0, x1 - x0, y1 - y0).to_image();
    let long = crop.width().max(crop.height()).max(1);
    if long >= CLOSE_UP_PX {
        return crop;
    }
    let scale = f64::from(CLOSE_UP_PX) / f64::from(long);
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "a positive size of a few hundred pixels"
    )]
    let size = |side: u32| (f64::from(side) * scale).round() as u32;
    image::imageops::resize(
        &crop,
        size(crop.width()),
        size(crop.height()),
        image::imageops::FilterType::CatmullRom,
    )
}

/// Marks as the model reads them: number, label, score and box.
pub(crate) fn marks_json(instances: &[Instance]) -> Vec<Value> {
    instances
        .iter()
        .enumerate()
        .map(|(i, inst)| {
            let (x, y, w, h) = inst.bbox;
            let score = (f64::from(inst.score) * 100.0).round() / 100.0;
            json!({"mark": i + 1, "label": inst.label, "score": score, "box": [x, y, w, h]})
        })
        .collect()
}
