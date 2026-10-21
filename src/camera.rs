//! Camera picture of the attached drive recorder: an NTSC frame source for the VDC4 scaler-0 write-back.
//!
//! The picture is synthetic (a road seen from the windscreen, lane markings moving with the vehicle speed). Each frame
//! is written in the format and orientation the firmware configured (SCL1_WR1: YCbCr422 / RGB565 / RGB888, mirror and
//! rotations), at the frame-buffer address the write-back would use.

use crate::mem::Memory;

/// NTSC field period in P0 clocks (59.94 Hz).
pub const FIELD_P0: u64 = (100.0e6 / 3.0 / 59.94) as u64;
const P0_HZ: f64 = 100.0e6 / 3.0;

#[derive(Default)]
pub struct Camera {
    pub next_frame: u64,
    pub fields: u64,
    /// Frames since frame-buffer writing was enabled.
    pub writes: u64,
    pub frames_written: u64,
    /// Vehicle speed for the moving road (km/h), and the distance travelled (m).
    pub speed_kmh: f64,
    distance: f64,
    last: u64,
}

/// Where and how the write-back stores one frame.
pub struct Target { pub base: u32, pub stride: u32, pub width: u32, pub height: u32, pub format: u8, pub rotation: u8 }

impl Camera {
    /// Scene colour at normalised picture coordinates (u, v in 0..1, v down).
    fn scene(&self, u: f64, v: f64) -> (u8, u8, u8) {
        let horizon = 0.42;
        if v < horizon {
            let t = v / horizon;
            return ((150.0 + 90.0 * t) as u8, (190.0 + 55.0 * t) as u8, (235.0 + 15.0 * t) as u8);
        }
        // Perspective: depth z for this row, road half-width shrinking toward the horizon.
        let d = (v - horizon) / (1.0 - horizon);
        let half = 0.04 + 0.62 * d;
        let x = u - 0.5;
        let z = 1.0 / d.max(0.02);
        let stripe = ((z * 4.0 + self.distance * 0.35) % 6.0) < 3.0;
        if x.abs() > half + 0.02 * d {
            let g = if ((z * 2.0 + self.distance * 0.18) % 4.0) < 2.0 { 110.0 } else { 98.0 };
            return ((g * 0.55) as u8, g as u8, (g * 0.4) as u8);
        }
        let edge = (x.abs() - half).abs() < 0.012 + 0.012 * d;
        let centre = x.abs() < 0.006 + 0.01 * d && stripe;
        if edge || centre { return (235, 235, 230); }
        let grey = 78.0 + 18.0 * d;
        (grey as u8, grey as u8, (grey + 4.0) as u8)
    }

    pub fn render(&mut self, now: u64, t: &Target, mem: &mut Memory) -> Result<(), String> {
        if self.last != 0 { self.distance += self.speed_kmh / 3.6 * (now.saturating_sub(self.last)) as f64 / P0_HZ; }
        self.last = now;
        if t.width < 2 || t.height < 2 || t.stride == 0 { return Err(format!("capture size {}x{} stride {}", t.width, t.height, t.stride)); }
        let rotated = matches!(t.rotation, 2 | 4);
        let (ow, oh) = if rotated { (t.height, t.width) } else { (t.width, t.height) };
        let bpp: u32 = match t.format { 0 | 1 => 2, 2 => 4, _ => return Err(format!("write-back format {}", t.format)) };
        let mut line = vec![0u8; (ow * bpp) as usize];
        for oy in 0..oh {
            for ox in 0..ow {
                // Output pixel -> source picture pixel for the configured writing mode.
                let (sx, sy) = match t.rotation {
                    1 => (t.width - 1 - ox, oy),
                    2 => (oy, t.height - 1 - ox),
                    3 => (t.width - 1 - ox, t.height - 1 - oy),
                    4 => (t.width - 1 - oy, ox),
                    _ => (ox, oy),
                };
                let (r, g, b) = self.scene((sx as f64 + 0.5) / t.width as f64, (sy as f64 + 0.5) / t.height as f64);
                let o = (ox * bpp) as usize;
                match t.format {
                    1 => { let p = ((r as u16 >> 3) << 11) | ((g as u16 >> 2) << 5) | (b as u16 >> 3); line[o..o + 2].copy_from_slice(&p.to_be_bytes()); }
                    2 => line[o..o + 4].copy_from_slice(&[0, r, g, b]),
                    _ => {
                        let (rf, gf, bf) = (r as f64, g as f64, b as f64);
                        let y = (16.0 + 0.257 * rf + 0.504 * gf + 0.098 * bf).round() as u8;
                        let c = if ox % 2 == 0 { 128.0 - 0.148 * rf - 0.291 * gf + 0.439 * bf } else { 128.0 + 0.439 * rf - 0.368 * gf - 0.071 * bf };
                        line[o] = c.round().clamp(0.0, 255.0) as u8;
                        line[o + 1] = y;
                    }
                }
            }
            if !mem.store_bytes(t.base + oy * t.stride, &line) { return Err(format!("frame buffer {:08x} not in memory", t.base + oy * t.stride)); }
        }
        self.frames_written += 1;
        Ok(())
    }
}
