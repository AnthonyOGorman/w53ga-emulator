//! JPEG Codec Unit (0xE801_7000), decompression path.
//! Register/command/interrupt behaviour follows the SH7268/69 manual (section 41); pixels come from a baseline
//! JPEG decoder (functional reference, not bit-exact JCU IDCT/colour arithmetic). Output in line chunks every
//! `chunk_steps` steps (count mode: JIFDDLC lines, doubled for 4:2:0) with DOUTLF/DBTF; JEDI on header pause.

use serde::Deserialize;

pub const BASE: u32 = 0xE801_7000;
const JCMOD: usize = 0x00;
const JCCMD: usize = 0x01;
const JCVSZU: usize = 0x07;
const JCHSZU: usize = 0x09;
const JINTE0: usize = 0x0E;
const JINTS0: usize = 0x0F;
const JCDERR: usize = 0x10;
const JCRST: usize = 0x11;
const JIFDCNT: usize = 0x58;
const JIFDSA: usize = 0x5C;
const JIFDDOFST: usize = 0x60;
const JIFDDA: usize = 0x64;
const JIFDDLC: usize = 0x6C;
const JIFDADT: usize = 0x70;
const JINTE1: usize = 0x8C;
const JINTS1: usize = 0x90;
const INS6: u8 = 0x40;
const INS5: u8 = 0x20;
const INS3: u8 = 0x08;
const DBTF: u8 = 0x04;
const JINF: u8 = 0x02;
const DOUTLF: u8 = 0x01;
const MAX_CODED: usize = 4 << 20;

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawJcu {
    regs: Vec<i32>, chunk_steps: u64, busy: bool, header_paused: bool, output_paused: bool, jedi_latched: bool, next_event_step: i64,
    #[serde(default)] pixels: Option<Vec<i64>>, width: u32, height: u32, padded_w: u32, padded_h: u32, out_line: u32, out_addr: u64,
    decodes: u64, chunks: u64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Jcu {
    #[serde(with = "serde_big_array::BigArray")]
    regs: [u8; 0x100],
    chunk_steps: u64,
    busy: bool, header_paused: bool, output_paused: bool, jedi_latched: bool,
    next_event_step: i64,
    pixels: Vec<u32>,
    width: u32, height: u32, padded_w: u32, padded_h: u32, out_line: u32, out_addr: u32,
    pub decodes: u64, pub chunks: u64,
}

type R<T> = Result<T, String>;

/// Memory access the JCU needs (coded input from flash/RAM, output into RAM).
pub trait JcuBus { fn read(&mut self, a: u32, n: usize) -> Option<Vec<u8>>; fn write(&mut self, a: u32, d: &[u8]) -> bool; }

impl Jcu {
    pub fn from_raw(x: RawJcu) -> R<Jcu> {
        if x.regs.len() != 0x100 { return Err("JCU register image size".into()); }
        let mut regs = [0u8; 0x100];
        for (i, v) in x.regs.iter().enumerate() { regs[i] = *v as u8; }
        Ok(Jcu {
            regs, chunk_steps: x.chunk_steps, busy: x.busy, header_paused: x.header_paused, output_paused: x.output_paused, jedi_latched: x.jedi_latched,
            next_event_step: x.next_event_step, pixels: x.pixels.unwrap_or_default().into_iter().map(|p| p as u32).collect(),
            width: x.width, height: x.height, padded_w: x.padded_w, padded_h: x.padded_h, out_line: x.out_line, out_addr: x.out_addr as u32,
            decodes: x.decodes, chunks: x.chunks,
        })
    }

    pub fn handles(a: u32, n: usize) -> bool { n > 0 && a >= BASE && a as u64 + n as u64 <= BASE as u64 + 0x100 }
    fn get(&self, o: usize, n: usize) -> u32 { (0..n).fold(0, |v, i| (v << 8) | self.regs[o + i] as u32) }
    fn set(&mut self, o: usize, n: usize, mut v: u32) { for i in (0..n).rev() { self.regs[o + i] = v as u8; v >>= 8; } }
    fn decode_mode(&self) -> bool { self.regs[JCMOD] & 8 != 0 }

    pub fn read(&self, a: u32, n: usize) -> R<u64> {
        let o = (a - BASE) as usize;
        let mut v = 0u64;
        for i in 0..n { let b = if o + i == JCCMD { 0 } else { self.regs[o + i] }; v = (v << 8) | b as u64; }
        Ok(v)
    }

    pub fn write(&mut self, a: u32, n: usize, value: u64, step: u64, bus: &mut dyn JcuBus) -> R<()> {
        let o = (a - BASE) as usize;
        if std::env::var_os("AR_JCU_LOG").is_some() { eprintln!("[jcu] step {step} write +{o:02x}/{n} = {value:x} (busy {} paused {})", self.busy, self.output_paused); }
        for i in 0..n {
            let r = o + i;
            let mut v = (value >> (8 * (n - 1 - i))) as u8;
            if r == JCCMD { if v != 0 { self.command(v, step, bus)?; } continue; }
            if r == JINTS0 { self.regs[r] &= v & (INS6 | INS5 | INS3); continue; }
            if (JINTS1..JINTS1 + 4).contains(&r) { self.regs[r] &= v; continue; }
            if r == JCRST || r == JCDERR { continue; }
            if r == JCMOD && self.busy { return Err("JCMOD change while decoding".into()); }
            if (JIFDCNT..JIFDCNT + 4).contains(&r) {
                if r == JIFDCNT + 3 && v & 0x20 != 0 { let reinit = v & 0x40 != 0 || self.regs[r] & 0x40 != 0; self.resume_output(step, reinit)?; v &= !0x20; }
                if r == JIFDCNT + 2 && v & 0x20 != 0 { return Err("input resume (JINRCMD) unsupported".into()); }
                self.regs[r] = v; continue;
            }
            self.regs[r] = v;
        }
        Ok(())
    }

    fn command(&mut self, v: u8, step: u64, bus: &mut dyn JcuBus) -> R<()> {
        if v.count_ones() != 1 { return Err(format!("multiple JCCMD commands {v:02x}")); }
        match v {
            0x04 => { self.jedi_latched = false; }
            0x80 => { if self.busy { return Err("bus reset while decoding".into()); } self.regs[JINTS0] = 0; self.regs[JCDERR] = 0x0A; self.regs[JCRST] = 0; self.jedi_latched = false; }
            0x02 => { if !self.header_paused { return Err("JRST without header pause".into()); } self.header_paused = false; self.regs[JCRST] = 0; self.schedule(step); }
            0x01 => self.start(step, bus)?,
            _ => return Err(format!("unsupported JCCMD {v:02x}")),
        }
        Ok(())
    }

    fn start(&mut self, step: u64, bus: &mut dyn JcuBus) -> R<()> {
        if !self.decode_mode() { return Err("compression unsupported".into()); }
        if self.busy { return Err("JSRT while busy".into()); }
        let cnt = self.get(JIFDCNT, 4);
        if cnt & 0x1000 != 0 { return Err("input count mode unsupported".into()); }
        if cnt & 0x707 != 0 { return Err("JINSWAP/DOUTSWAP unsupported".into()); }
        if cnt & 0x3C00_0000 != 0 { return Err("output subsampling unsupported".into()); }
        let opf = (cnt >> 24) & 3;
        if opf != 1 && opf != 2 { return Err(format!("output format {opf} unsupported")); }
        let src = self.get(JIFDSA, 4);
        let coded = Self::read_coded(src, bus)?;
        let (h, w, hmax, vmax) = sof0(&coded).ok_or("no baseline SOF0 marker")?;
        if std::env::var_os("AR_JCU_SRC").is_some() { eprintln!("[jcu-src] step {step} src {src:08x} {w}x{h} {} bytes", coded.len()); }
        let redu = match (hmax, vmax) { (2, 2) => 2, (2, 1) => 1, (1, 1) => 0, (4, 1) => 6, _ => return Err(format!("sampling {hmax}x{vmax}")) };
        let mut dec = jpeg_decoder::Decoder::new(&coded[..]);
        let data = dec.decode().map_err(|e| format!("JPEG at {src:08x}: {e}"))?;
        let info = dec.info().ok_or("JPEG info")?;
        if info.width as u32 != w || info.height as u32 != h { return Err("decoder size mismatch".into()); }
        let (mcu_w, mcu_h) = (8 * hmax, 8 * vmax);
        self.width = w; self.height = h;
        self.padded_w = w.div_ceil(mcu_w) * mcu_w;
        self.padded_h = h.div_ceil(mcu_h) * mcu_h;
        let gray = info.pixel_format == jpeg_decoder::PixelFormat::L8;
        self.pixels = Vec::with_capacity((self.padded_w * self.padded_h) as usize);
        for y in 0..self.padded_h {
            for x in 0..self.padded_w {
                let (sx, sy) = (x.min(w - 1) as usize, y.min(h - 1) as usize);
                let p = if gray { let g = data[sy * w as usize + sx] as u32; (g << 16) | (g << 8) | g } else {
                    let i = (sy * w as usize + sx) * 3; ((data[i] as u32) << 16) | ((data[i + 1] as u32) << 8) | data[i + 2] as u32 };
                self.pixels.push(p);
            }
        }
        self.regs[JCMOD] = (self.regs[JCMOD] & 8) | redu;
        self.set(JCVSZU, 2, h); self.set(JCHSZU, 2, w); self.regs[JCDERR] = 0;
        self.busy = true; self.out_line = 0; self.out_addr = self.get(JIFDDA, 4); self.decodes += 1;
        if self.regs[JINTE0] & INS3 != 0 { self.regs[JINTS0] |= INS3; self.regs[JCRST] = 1; self.header_paused = true; self.jedi_latched = true; }
        else { self.schedule(step); }
        Ok(())
    }

    fn read_coded(src: u32, bus: &mut dyn JcuBus) -> R<Vec<u8>> {
        let mut out = Vec::new();
        let mut a = src;
        let mut prev = -1i32;
        while out.len() < MAX_CODED {
            let blk = bus.read(a, 4096).ok_or(format!("JCU coded source {a:08x} unmapped"))?;
            for &b in &blk { out.push(b); if prev == 0xFF && b == 0xD9 { return Ok(out); } prev = b as i32; }
            a += 4096;
        }
        Err(format!("no EOI within {MAX_CODED} bytes of {src:08x}"))
    }

    fn schedule(&mut self, step: u64) { self.next_event_step = (step + self.chunk_steps) as i64; }
    fn count_mode(&self) -> bool { self.get(JIFDCNT, 4) & 0x10 != 0 }
    fn quota_lines(&self) -> u32 { let l = self.get(JIFDDLC, 4) & 0xFFF8; if self.regs[JCMOD] & 7 == 2 { l * 2 } else { l } }

    fn resume_output(&mut self, step: u64, reinit: bool) -> R<()> {
        if !self.output_paused { return Err("DOUTRCMD without output pause".into()); }
        self.output_paused = false;
        if reinit { self.out_addr = self.get(JIFDDA, 4); }
        self.schedule(step);
        Ok(())
    }

    /// Perform a due output chunk at `step`.
    pub fn tick(&mut self, step: u64, bus: &mut dyn JcuBus) -> R<bool> {
        if !self.busy || self.header_paused || self.output_paused || self.next_event_step < 0 || (step as i64) < self.next_event_step { return Ok(false); }
        self.next_event_step = -1;
        let mut lines = self.padded_h - self.out_line;
        if self.count_mode() { let q = self.quota_lines(); if q < 1 { return Err("zero output line quota".into()); } lines = lines.min(q); }
        let opf = (self.get(JIFDCNT, 4) >> 24) & 3;
        let bpp = if opf == 2 { 2 } else { 4 };
        let stride = self.get(JIFDDOFST, 4) & 0x7FF8;
        if stride < self.padded_w * bpp { return Err(format!("output stride {stride} < line bytes")); }
        let alpha = self.regs[JIFDADT + 3];
        for y in 0..lines {
            let mut row = Vec::with_capacity((self.padded_w * bpp) as usize);
            for x in 0..self.padded_w {
                let p = self.pixels[((self.out_line + y) * self.padded_w + x) as usize];
                let (r, g, b) = ((p >> 16) & 255, (p >> 8) & 255, p & 255);
                if bpp == 2 { let v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3); row.extend([(v >> 8) as u8, v as u8]); }
                else { row.extend([alpha, r as u8, g as u8, b as u8]); }
            }
            if !bus.write(self.out_addr, &row) { return Err(format!("JCU output {:08x} unmapped", self.out_addr)); }
            self.out_addr += stride;
        }
        self.out_line += lines;
        self.chunks += 1;
        if self.out_line >= self.padded_h { self.busy = false; self.regs[JINTS1 + 3] |= DBTF; } else { self.output_paused = true; self.regs[JINTS1 + 3] |= DOUTLF; }
        if std::env::var_os("AR_JCU_LOG").is_some() { eprintln!("[jcu] step {step} chunk {lines} lines -> {}/{} JINTS1 {:02x} JINTE1 {:02x}", self.out_line, self.padded_h, self.regs[JINTS1 + 3], self.regs[JINTE1 + 3]); }
        Ok(true)
    }

    pub fn jdti_level(&self) -> bool { self.regs[JINTS1 + 3] & self.regs[JINTE1 + 3] & (DBTF | JINF | DOUTLF) != 0 }
    pub fn jedi_level(&self) -> bool { self.jedi_latched }
}

/// Baseline SOF0: (height, width, Hmax, Vmax).
fn sof0(d: &[u8]) -> Option<(u32, u32, u32, u32)> {
    let mut i = 2;
    while i + 9 < d.len() {
        if d[i] != 0xFF { i += 1; continue; }
        let m = d[i + 1];
        if m == 0xC0 {
            let h = ((d[i + 5] as u32) << 8) | d[i + 6] as u32;
            let w = ((d[i + 7] as u32) << 8) | d[i + 8] as u32;
            let nc = d[i + 9] as usize;
            let (mut hm, mut vm) = (1, 1);
            for c in 0..nc { let s = *d.get(i + 11 + 3 * c)? as u32; hm = hm.max(s >> 4); vm = vm.max(s & 15); }
            return Some((h, w, hm, vm));
        }
        if m == 0xD8 || m == 0x01 || (0xD0..=0xD7).contains(&m) { i += 2; continue; }
        if matches!(m, 0xC1 | 0xC2 | 0xC3) { return None; }
        let len = ((*d.get(i + 2)? as usize) << 8) | *d.get(i + 3)? as usize;
        i += 2 + len;
    }
    None
}
