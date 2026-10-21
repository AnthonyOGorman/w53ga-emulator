//! High-level emulation of the Renesas RGAH 2D graphics API (SH7269 vector graphics processor driver).
//!
//! The GPU is microcoded with an undocumented instruction set, so instead of emulating it the four RGAH entry points that
//! reach the hardware (`RGAH_fillRect`, `RGAH_drawImage`, `RGAH_bitBlt`, `RGAH_bitBltFixedColor`) and the completion
//! wait (`RGAH_finish`) can be intercepted and rendered in software, following the API's argument and structure
//! layouts. Where these functions are in a given program comes from an HLE profile (`crate::hle`).
//!
//! - bitmap (`RGAH_initBmp`): u32 width, height, clip x, y, w, h, format, address.
//!   Formats: 0 RGB565, 1 ARGB1555, 2 ARGB4444, 3 ARGB8888, 4 ARGB8888 premultiplied, 5 XRGB8888 (opaque).
//! - effect: u32 antialias, filter (bilinear), blend mode (0 copy, 1 src-over, 2 dst-over, 3 src-in, 4 dst-in,
//!   5 multiply, 6 additive), tile/projective flag.
//! - colour: ARGB8888 with an extra 8-bit alpha factor (`A * alpha / 255`).
//! Pixels are big-endian in memory, as the VDC reads them.

use crate::mem::Memory;

#[derive(Clone, Copy, Debug)]
pub struct Bmp { pub w: i32, pub h: i32, pub clip: (i32, i32, i32, i32), pub fmt: u32, pub addr: u32 }

#[derive(Default, Debug, serde::Serialize, serde::Deserialize)]
pub struct GfxStats { pub fills: u64, pub images: u64, pub blits: u64, pub masks: u64, pub pixels: u64, pub warned: Vec<String> }

fn rd32(mem: &Memory, a: u32) -> Option<u32> { mem.read_plain(a, 4).map(|v| v as u32) }
fn rdf(mem: &Memory, a: u32) -> Option<f32> { rd32(mem, a).map(f32::from_bits) }

pub fn read_bmp(mem: &Memory, p: u32) -> Option<Bmp> {
    if p == 0 || p & 3 != 0 { return None; }
    let w: Vec<u32> = (0..8).map(|i| rd32(mem, p + 4 * i)).collect::<Option<_>>()?;
    Some(Bmp { w: w[0] as i32, h: w[1] as i32, clip: (w[2] as i32, w[3] as i32, w[4] as i32, w[5] as i32), fmt: w[6], addr: w[7] })
}

fn bpp(fmt: u32) -> u32 { if fmt <= 2 { 2 } else { 4 } }

/// Straight-alpha RGBA in 0..=1.
type Px = [f32; 4];

fn decode(fmt: u32, raw: u32) -> Px {
    let c = |v: u32, bits: u32| v as f32 / ((1u32 << bits) - 1) as f32;
    match fmt {
        0 => [c((raw >> 11) & 31, 5), c((raw >> 5) & 63, 6), c(raw & 31, 5), 1.0],
        1 => [c((raw >> 10) & 31, 5), c((raw >> 5) & 31, 5), c(raw & 31, 5), ((raw >> 15) & 1) as f32],
        2 => [c((raw >> 8) & 15, 4), c((raw >> 4) & 15, 4), c(raw & 15, 4), c((raw >> 12) & 15, 4)],
        4 => { let a = c(raw >> 24, 8); if a > 0.0 { [(c((raw >> 16) & 255, 8) / a).min(1.0), (c((raw >> 8) & 255, 8) / a).min(1.0), (c(raw & 255, 8) / a).min(1.0), a] } else { [0.0; 4] } }
        5 => [c((raw >> 16) & 255, 8), c((raw >> 8) & 255, 8), c(raw & 255, 8), 1.0],
        _ => [c((raw >> 16) & 255, 8), c((raw >> 8) & 255, 8), c(raw & 255, 8), c(raw >> 24, 8)],
    }
}

fn encode(fmt: u32, p: Px) -> u32 {
    let q = |v: f32, bits: u32| ((v.clamp(0.0, 1.0) * ((1u32 << bits) - 1) as f32) + 0.5) as u32;
    match fmt {
        0 => (q(p[0], 5) << 11) | (q(p[1], 6) << 5) | q(p[2], 5),
        1 => (if p[3] >= 0.5 { 0x8000 } else { 0 }) | (q(p[0], 5) << 10) | (q(p[1], 5) << 5) | q(p[2], 5),
        2 => (q(p[3], 4) << 12) | (q(p[0], 4) << 8) | (q(p[1], 4) << 4) | q(p[2], 4),
        4 => (q(p[3], 8) << 24) | (q(p[0] * p[3], 8) << 16) | (q(p[1] * p[3], 8) << 8) | q(p[2] * p[3], 8),
        5 => 0xFF00_0000 | (q(p[0], 8) << 16) | (q(p[1], 8) << 8) | q(p[2], 8),
        _ => (q(p[3], 8) << 24) | (q(p[0], 8) << 16) | (q(p[1], 8) << 8) | q(p[2], 8),
    }
}

fn argb(c: u32, alpha: u32) -> Px {
    [((c >> 16) & 255) as f32 / 255.0, ((c >> 8) & 255) as f32 / 255.0, (c & 255) as f32 / 255.0, ((c >> 24) * alpha) as f32 / 65025.0]
}

/// Blend `s` over destination `d` with the RGAH blend mode.
fn blend(mode: u32, s: Px, d: Px) -> Px {
    let mix = |sa: f32, da: f32, f: &dyn Fn(f32, f32) -> f32| -> Px {
        let a = sa + da * (1.0 - sa);
        if a <= 0.0 { return [0.0; 4]; }
        let ch = |k: usize| (f(s[k], d[k]) * sa * da + s[k] * sa * (1.0 - da) + d[k] * da * (1.0 - sa)) / a;
        [ch(0), ch(1), ch(2), a]
    };
    match mode {
        0 => s,
        1 => mix(s[3], d[3], &|sc, _| sc),
        2 => mix(s[3], d[3], &|_, dc| dc),
        3 => [s[0], s[1], s[2], s[3] * d[3]],
        4 => [d[0], d[1], d[2], d[3] * s[3]],
        5 => mix(s[3], d[3], &|sc, dc| sc * dc),
        _ => { let a = (s[3] + d[3]).min(1.0); [(s[0] * s[3] + d[0] * d[3]).min(1.0), (s[1] * s[3] + d[1] * d[3]).min(1.0), (s[2] * s[3] + d[2] * d[3]).min(1.0), a] }
    }
}

struct Surface { bmp: Bmp, stride: u32, data: Vec<u8>, base_y: i32 }

impl Surface {
    /// Load rows y0..y1 of a bitmap.
    fn load(mem: &Memory, bmp: Bmp, y0: i32, y1: i32) -> Option<Surface> {
        let stride = bpp(bmp.fmt) * bmp.w as u32;
        let rows = (y1 - y0).max(0) as u32;
        let data = mem.load_bytes(bmp.addr + y0 as u32 * stride, (rows * stride) as usize)?;
        Some(Surface { bmp, stride, data, base_y: y0 })
    }
    fn get(&self, x: i32, y: i32) -> Px {
        let o = ((y - self.base_y) as u32 * self.stride + x as u32 * bpp(self.bmp.fmt)) as usize;
        let raw = if bpp(self.bmp.fmt) == 2 { u16::from_be_bytes([self.data[o], self.data[o + 1]]) as u32 } else { u32::from_be_bytes([self.data[o], self.data[o + 1], self.data[o + 2], self.data[o + 3]]) };
        decode(self.bmp.fmt, raw)
    }
    fn put(&mut self, x: i32, y: i32, p: Px) {
        let o = ((y - self.base_y) as u32 * self.stride + x as u32 * bpp(self.bmp.fmt)) as usize;
        let raw = encode(self.bmp.fmt, p);
        if bpp(self.bmp.fmt) == 2 { self.data[o..o + 2].copy_from_slice(&(raw as u16).to_be_bytes()); } else { self.data[o..o + 4].copy_from_slice(&raw.to_be_bytes()); }
    }
    fn store(&self, mem: &mut Memory) -> bool { mem.store_bytes(self.bmp.addr + self.base_y as u32 * self.stride, &self.data) }
}

/// Intersect rect (x, y, w, h) with the bitmap clip area.
fn clip(r: (i32, i32, i32, i32), c: (i32, i32, i32, i32)) -> Option<(i32, i32, i32, i32)> {
    let (x0, y0) = (r.0.max(c.0), r.1.max(c.1));
    let (x1, y1) = ((r.0 + r.2).min(c.0 + c.2), (r.1 + r.3).min(c.1 + c.3));
    if x1 > x0 && y1 > y0 { Some((x0, y0, x1 - x0, y1 - y0)) } else { None }
}

/// Sample a source bitmap at continuous coordinates (pixel centres at +0.5).
fn sample(src: &Surface, u: f32, v: f32, filter: bool, tile: bool) -> Option<Px> {
    let (w, h) = (src.bmp.w, src.bmp.h);
    let fetch = |x: i32, y: i32| -> Option<Px> {
        let (x, y) = if tile { (x.rem_euclid(w), y.rem_euclid(h)) } else { (x, y) };
        if x < 0 || y < 0 || x >= w || y >= h { None } else { Some(src.get(x, y)) }
    };
    if !filter { return fetch(u.floor() as i32, v.floor() as i32); }
    let (fx, fy) = (u - 0.5, v - 0.5);
    let (x0, y0) = (fx.floor() as i32, fy.floor() as i32);
    let (ax, ay) = (fx - x0 as f32, fy - y0 as f32);
    let p = [fetch(x0, y0), fetch(x0 + 1, y0), fetch(x0, y0 + 1), fetch(x0 + 1, y0 + 1)];
    if p.iter().all(|q| q.is_none()) { return None; }
    let wts = [(1.0 - ax) * (1.0 - ay), ax * (1.0 - ay), (1.0 - ax) * ay, ax * ay];
    let mut acc = [0.0f32; 4];
    let mut wa = 0.0;
    for (q, wt) in p.iter().zip(wts) {
        if let Some(q) = q { for k in 0..3 { acc[k] += q[k] * q[3] * wt; } acc[3] += q[3] * wt; wa += wt; }
    }
    if wa <= 0.0 { return None; }
    let a = acc[3];
    Some(if a > 0.0 { [acc[0] / a, acc[1] / a, acc[2] / a, a] } else { [0.0; 4] })
}

/// Arguments of an intercepted call: registers R4..R7 and the stack (R15) for the rest.
pub struct Call<'a> { pub mem: &'a mut Memory, pub r: [u32; 4], pub sp: u32 }

impl Call<'_> {
    fn stack(&self, i: u32) -> u32 { rd32(self.mem, self.sp + 4 * i).unwrap_or(0) }
}

pub fn fill_rect(c: &mut Call, st: &mut GfxStats) -> Option<()> {
    let dst = read_bmp(c.mem, c.r[0])?;
    let (x, y) = (rd32(c.mem, c.r[1])? as i32, rd32(c.mem, c.r[1] + 4)? as i32);
    let (w, h) = (c.r[2] as i32, c.r[3] as i32);
    let (color, alpha, effect) = (c.stack(0), c.stack(1) & 255, c.stack(2));
    let mode = rd32(c.mem, effect + 8)?;
    let r = clip((x, y, w, h), dst.clip)?;
    let src = argb(color, alpha);
    let mut s = Surface::load(c.mem, dst, r.1, r.1 + r.3)?;
    for yy in r.1..r.1 + r.3 { for xx in r.0..r.0 + r.2 { let d = s.get(xx, yy); s.put(xx, yy, if mode == 0 { src } else { blend(mode, src, d) }); } }
    s.store(c.mem);
    st.fills += 1; st.pixels += (r.2 * r.3) as u64;
    Some(())
}

fn invert3(m: &[f32; 9]) -> Option<[f32; 9]> {
    let det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    if det.abs() < 1e-12 { return None; }
    let i = 1.0 / det;
    Some([(m[4] * m[8] - m[5] * m[7]) * i, (m[2] * m[7] - m[1] * m[8]) * i, (m[1] * m[5] - m[2] * m[4]) * i,
          (m[5] * m[6] - m[3] * m[8]) * i, (m[0] * m[8] - m[2] * m[6]) * i, (m[2] * m[3] - m[0] * m[5]) * i,
          (m[3] * m[7] - m[4] * m[6]) * i, (m[1] * m[6] - m[0] * m[7]) * i, (m[0] * m[4] - m[1] * m[3]) * i])
}

pub fn draw_image(c: &mut Call, st: &mut GfxStats) -> Option<()> {
    let dst = read_bmp(c.mem, c.r[0])?;
    let mp = c.r[1];
    let m: [f32; 9] = std::array::from_fn(|i| rdf(c.mem, mp + 4 * i as u32).unwrap_or(0.0));
    let src_bmp = read_bmp(c.mem, rd32(c.mem, mp + 36)?)?;
    let (px, py) = (rd32(c.mem, c.r[2])? as i32 as f32, rd32(c.mem, c.r[2] + 4)? as i32 as f32);
    let alpha = c.r[3] & 255;
    let effect = c.stack(0);
    let e: [u32; 4] = std::array::from_fn(|i| rd32(c.mem, effect + 4 * i as u32).unwrap_or(0));
    let inv = invert3(&m)?;
    // destination bounds: transformed source corners
    let (sw, sh) = (src_bmp.w as f32, src_bmp.h as f32);
    let mut bx = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for (u, v) in [(0.0, 0.0), (sw, 0.0), (0.0, sh), (sw, sh)] {
        let wq = m[6] * u + m[7] * v + m[8];
        let (x, y) = ((m[0] * u + m[1] * v + m[2]) / wq + px, (m[3] * u + m[4] * v + m[5]) / wq + py);
        bx = (bx.0.min(x), bx.1.min(y), bx.2.max(x), bx.3.max(y));
    }
    let r = clip((bx.0.floor() as i32, bx.1.floor() as i32, (bx.2.ceil() - bx.0.floor()) as i32, (bx.3.ceil() - bx.1.floor()) as i32), dst.clip)?;
    let src = Surface::load(c.mem, src_bmp, 0, src_bmp.h)?;
    let mut d = Surface::load(c.mem, dst, r.1, r.1 + r.3)?;
    let (filter, tile) = (e[1] == 1, false);
    let mut n = 0u64;
    for yy in r.1..r.1 + r.3 {
        for xx in r.0..r.0 + r.2 {
            let (qx, qy) = (xx as f32 + 0.5 - px, yy as f32 + 0.5 - py);
            let wq = inv[6] * qx + inv[7] * qy + inv[8];
            let (u, v) = ((inv[0] * qx + inv[1] * qy + inv[2]) / wq, (inv[3] * qx + inv[4] * qy + inv[5]) / wq);
            let Some(mut s) = sample(&src, u, v, filter, tile) else { continue };
            if alpha != 255 { s[3] *= alpha as f32 / 255.0; }
            let old = d.get(xx, yy);
            d.put(xx, yy, blend(e[2], s, old));
            n += 1;
        }
    }
    d.store(c.mem);
    st.images += 1; st.pixels += n;
    Some(())
}

pub fn bit_blt(c: &mut Call, st: &mut GfxStats) -> Option<()> {
    let dst = read_bmp(c.mem, c.r[0])?;
    let src_bmp = read_bmp(c.mem, c.r[1])?;
    let rp = c.r[2];
    let q: [i32; 8] = std::array::from_fn(|i| rd32(c.mem, rp + 4 * i as u32).unwrap_or(0) as i32);
    let alpha = c.r[3] & 255;
    let effect = c.stack(0);
    let e: [u32; 4] = std::array::from_fn(|i| rd32(c.mem, effect + 4 * i as u32).unwrap_or(0));
    let r = clip((q[4], q[5], q[6], q[7]), dst.clip)?;
    let src = Surface::load(c.mem, src_bmp, 0, src_bmp.h)?;
    let mut d = Surface::load(c.mem, dst, r.1, r.1 + r.3)?;
    let tile = e[3] == 1;
    let (scx, scy) = if tile || q[6] == 0 || q[7] == 0 { (1.0, 1.0) } else { (q[2] as f32 / q[6] as f32, q[3] as f32 / q[7] as f32) };
    let filter = e[1] == 1 && !tile && (scx != 1.0 || scy != 1.0);
    let mut n = 0u64;
    for yy in r.1..r.1 + r.3 {
        for xx in r.0..r.0 + r.2 {
            let (u, v) = (q[0] as f32 + (xx - q[4]) as f32 * scx + 0.5 * scx, q[1] as f32 + (yy - q[5]) as f32 * scy + 0.5 * scy);
            let s = if tile { sample(&src, (q[0] + (xx - q[4]).rem_euclid(q[2].max(1))) as f32 + 0.5, (q[1] + (yy - q[5]).rem_euclid(q[3].max(1))) as f32 + 0.5, false, false) }
                    else { sample(&src, u, v, filter, false) };
            let Some(mut s) = s else { continue };
            if alpha != 255 { s[3] *= alpha as f32 / 255.0; }
            let old = d.get(xx, yy);
            d.put(xx, yy, blend(e[2], s, old));
            n += 1;
        }
    }
    d.store(c.mem);
    st.blits += 1; st.pixels += n;
    Some(())
}

pub fn bit_blt_fixed_color(c: &mut Call, st: &mut GfxStats) -> Option<()> {
    let dst = read_bmp(c.mem, c.r[0])?;
    let mp = c.r[1];
    let (color, mw, _mh, maddr) = (rd32(c.mem, mp)?, rd32(c.mem, mp + 4)? as i32, rd32(c.mem, mp + 8)? as i32, rd32(c.mem, mp + 12)?);
    let rp = c.r[2];
    let q: [i32; 8] = std::array::from_fn(|i| rd32(c.mem, rp + 4 * i as u32).unwrap_or(0) as i32);
    let r = clip((q[4], q[5], q[6], q[7]), dst.clip)?;
    let mut d = Surface::load(c.mem, dst, r.1, r.1 + r.3)?;
    let base = if color == 0xFFFF_FFFF { [1.0, 1.0, 1.0, 1.0] } else { argb(color, 255) };
    let mut n = 0u64;
    for yy in r.1..r.1 + r.3 {
        let my = q[1] + (yy - q[5]);
        let row = c.mem.load_bytes(maddr + (my * mw + q[0] + (r.0 - q[4])) as u32, r.2 as usize)?;
        for (i, xx) in (r.0..r.0 + r.2).enumerate() {
            let a = row[i] as f32 / 255.0;
            if a == 0.0 { continue; }
            let s = [base[0], base[1], base[2], base[3] * a];
            let old = d.get(xx, yy);
            d.put(xx, yy, blend(1, s, old));
            n += 1;
        }
    }
    d.store(c.mem);
    st.masks += 1; st.pixels += n;
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mem::{Memory, FLASH_SIZE};

    #[test]
    fn pixel_formats_round_trip() {
        for (fmt, raw) in [(0u32, 0xF81Fu32), (1, 0x83E0), (2, 0xF0F0), (3, 0x80FF_8000), (5, 0xFF12_3456)] {
            assert_eq!(encode(fmt, decode(fmt, raw)), raw, "format {fmt}");
        }
    }

    #[test]
    fn src_over_blend_of_opaque_source_is_the_source() {
        let s = [0.2, 0.4, 0.6, 1.0];
        assert_eq!(blend(1, s, [0.9, 0.9, 0.9, 1.0]), s);
        let half = blend(1, [1.0, 0.0, 0.0, 0.5], [0.0, 0.0, 1.0, 1.0]);
        assert!((half[0] - 0.5).abs() < 1e-6 && (half[2] - 0.5).abs() < 1e-6 && (half[3] - 1.0).abs() < 1e-6);
    }

    fn put32(m: &mut Memory, a: u32, v: u32) { assert!(m.store_bytes(a, &v.to_be_bytes())); }

    #[test]
    fn fill_rect_writes_the_clipped_rectangle() {
        let mut mem = Memory::new(vec![0; FLASH_SIZE]);
        let (bmp, pt, sp, eff, pixels) = (0x1C00_0000, 0x1C00_0100, 0x1C00_0200, 0x1C00_0300, 0x1C00_1000);
        // 4x4 RGB565 bitmap, clipped to x < 3
        for (i, v) in [4, 4, 0, 0, 3, 4, 0, pixels].into_iter().enumerate() { put32(&mut mem, bmp + 4 * i as u32, v); }
        put32(&mut mem, pt, 1); put32(&mut mem, pt + 4, 1);
        put32(&mut mem, sp, 0xFFFF_0000); put32(&mut mem, sp + 4, 255); put32(&mut mem, sp + 8, eff);
        put32(&mut mem, eff + 8, 0);
        let mut st = GfxStats::default();
        let mut call = Call { mem: &mut mem, r: [bmp, pt, 3, 2], sp };
        assert!(fill_rect(&mut call, &mut st).is_some());
        let px = |m: &Memory, x: u32, y: u32| m.read_plain(pixels + (y * 4 + x) * 2, 2).unwrap();
        assert_eq!(px(&mem, 1, 1), 0xF800);
        assert_eq!(px(&mem, 2, 2), 0xF800);
        assert_eq!(px(&mem, 3, 1), 0, "outside the clip rectangle");
        assert_eq!(px(&mem, 0, 0), 0);
        assert_eq!(st.pixels, 4);
    }
}
