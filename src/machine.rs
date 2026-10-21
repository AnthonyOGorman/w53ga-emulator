//! Free-running machine: CPU + board, one instruction step at a time, peripherals ticked after every step and at
//! most one interrupt accepted per step boundary.

use crate::checkpoint::Checkpoint;
use crate::cpu::{Cpu, Fault, Mmio};
use crate::intc::IBNR;
use crate::mem::Memory;
use crate::periph::display::GR;
use crate::periph::Board;

impl Mmio for Board {
    fn read(&mut self, pc: u32, a: u32, n: usize) -> u64 {
        self.irq_dirty = true;
        if crate::periph::nor::Nor::handles(a) {
            if let Some(v) = self.nor.mode_read(a) { return match n { 4 => ((v as u64) << 16) | v as u64, 1 => (v & 0xFF) as u64, _ => v as u64 }; }
        }
        if self.owns(a, n) || self.latches.find(a, n).is_some() {
            if let Some(v) = Board::read(self, a, n) { return v; }
        }
        if let Some(v) = self.observed.get(&(a, n)) { return *v; }
        self.fault.get_or_insert(format!("unmodelled MMIO read {a:08x}/{n} at pc {pc:08x}"));
        0
    }
    fn write(&mut self, pc: u32, a: u32, n: usize, v: u64) {
        self.irq_dirty = true;
        if !Board::write(self, a, n, v) {
            // Unmodelled register: the store is accepted and remembered, nothing else happens.
            if self.observed.insert((a, n), v).is_none() { eprintln!("[mmio] unmodelled write {a:08x}/{n} = {v:x} at pc {pc:08x} (observed)"); }
        }
    }
    fn ibnr(&mut self) -> u16 { self.latches.value(IBNR) as u16 }
    fn set_ibnr(&mut self, v: u16) { self.latches.set(IBNR, v as u64); }
    fn flash_cmd_mode(&self) -> bool { self.nor.in_command_mode() }
}

fn one() -> u32 { 1 }
fn zero64() -> [u32; 64] { [0; 64] }
fn false64() -> [bool; 64] { [false; 64] }

/// Header of saved states; bump when the serialized layout changes.
pub const STATE_MAGIC: &[u8; 16] = b"W53GA-STATE-0001";

#[derive(Debug)]
pub enum Stop { Cpu(Fault), Model(String) }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Machine {
    pub cpu: Cpu,
    pub board: Board,
    pub steps: u64,
    pub interrupts: u64,
    /// Last executed PCs (interrupt entries recorded as vector | 0x8000_0000) for post-mortem dumps.
    #[serde(skip, default = "zero64")]
    pub history: [u32; 64],
    #[serde(skip, default = "false64")]
    pub hist_irq: [bool; 64],
    #[serde(skip)]
    pub hist_pos: usize,
    /// CPU instructions per peripheral (P0) clock. The real chip runs its core at Iφ = 8 x P0 with roughly one
    /// instruction per 1-1.5 clocks, so ~6 approximates hardware timing.
    pub cpu_per_p0: u32,
    sub: u32,
    /// P0 clocks advanced per peripheral tick (1 = finest timing; larger batches are faster).
    #[serde(skip, default = "one")]
    pub p0_batch: u32,
    /// Diagnostics: execution counts for selected addresses (function entries), with the covered PC range.
    #[serde(skip)]
    pub call_counts: Option<(u32, u32, std::collections::HashMap<u32, u64>)>,
    /// Software rendering of the RGAH graphics API in place of the (unemulated) vector graphics processor, at the
    /// entry points of the HLE profile in `hooks`.
    pub gfx_hle: bool,
    #[serde(skip)]
    pub gfx: crate::gfx::GfxStats,
    /// Entry points from the HLE profile in use (`crate::hle`); empty without one.
    #[serde(skip)]
    pub hooks: crate::hle::Hooks,
    /// Skip CPU execution while the RTOS idle loop named by the HLE profile polls with nothing to do (peripherals keep
    /// running).
    #[serde(skip)]
    pub idle_skip: bool,
    /// P0 clocks spent skipping idle time.
    #[serde(skip)]
    pub idle_skipped: u64,
    /// Accepted interrupts per vector.
    #[serde(skip)]
    pub irq_hist: std::collections::BTreeMap<u32, u64>,
    /// Debug: stop when execution reaches this PC.
    #[serde(skip)]
    pub stop_pc: Option<u32>,
    /// Chip resets taken (watchdog).
    #[serde(skip)]
    pub resets: u64,
    /// Debug (AR_SH_ARG_LOG=hex,...): print r4-r7 whenever execution reaches one of these PCs (function arguments).
    #[serde(skip)]
    pub arg_log: Vec<u32>,
    /// A development hook is set (stop PC, argument log, call counts): `step` checks them (refreshed by `run`).
    #[serde(skip)]
    debug_hooks: bool,
}

impl Machine {
    pub fn from_checkpoint(flash: Vec<u8>, c: &Checkpoint) -> Result<Machine, String> {
        let mut cpu = Cpu::new(Memory::new(flash));
        c.apply(&mut cpu);
        Ok(Machine { cpu, board: Board::from_checkpoint(c)?, steps: c.steps, interrupts: 0, history: [0; 64], hist_irq: [false; 64], hist_pos: 0, cpu_per_p0: 1, sub: 0, p0_batch: 1, call_counts: None, gfx_hle: false, gfx: Default::default(), hooks: Default::default(), idle_skip: false, idle_skipped: 0, irq_hist: Default::default(), stop_pc: std::env::var("AR_SH_STOP_PC").ok().and_then(|v| u32::from_str_radix(&v, 16).ok()), resets: 0, arg_log: arg_log_env(), debug_hooks: false })
    }

    /// The unit as power is applied: the chip's power-on reset state (`Checkpoint::power_on`), the flash image as
    /// supplied, and the CPU started from the image's power-on reset vectors (PC at 0x0, SP at 0x4), as the SH-2A does.
    pub fn power_on(flash: Vec<u8>) -> Result<Machine, String> {
        if flash.len() != crate::mem::FLASH_SIZE { return Err(format!("flash image is {} bytes; expected a {} MiB dump of the NOR flash", flash.len(), crate::mem::FLASH_SIZE >> 20)); }
        let c = Checkpoint::power_on()?;
        let mut m = Machine::from_checkpoint(flash, &c)?;
        let rd = |a: usize| u32::from_be_bytes(m.cpu.mem.flash[a..a + 4].try_into().unwrap());
        let (pc, sp) = (rd(0), rd(4));
        m.cpu.pc = pc;
        m.cpu.r[15] = sp;
        Ok(m)
    }

    /// Use an HLE profile (or none): its hooks replace the current ones. The profile should have been verified against
    /// the flash image (`Profile::verify`).
    pub fn set_profile(&mut self, p: Option<&crate::hle::Profile>) {
        self.hooks = p.map(|p| p.hooks()).unwrap_or_default();
    }

    /// Typical settings for real-time use: about 6 SH-2A instructions per peripheral clock (the 266 MHz core), 32-clock
    /// peripheral batches, idle skipping and software 2D graphics where the profile allows, and the free-running video
    /// decoder input.
    pub fn configure_realtime(&mut self) {
        self.cpu_per_p0 = 6;
        self.p0_batch = 32;
        self.idle_skip = true;
        self.gfx_hle = true;
        self.board.set_tdfe_auto_clear(true);
        self.board.vdec_free_run = true;
    }

    /// Interrupt acceptance, then one instruction, then peripheral ticks.
    #[inline]
    pub fn step(&mut self) -> Result<(), Stop> {
        if let Some(power_on) = self.board.wdt.reset.take() { self.chip_reset(power_on).map_err(Stop::Model)?; }
        // Idle loop head named by the HLE profile: with interrupts unmasked and nothing deliverable, no task can become
        // ready, so only the peripherals advance until an interrupt is due.
        if self.idle_skip && Some(self.cpu.pc) == self.hooks.idle_loop && self.cpu.imask() == 0 {
            let deliverable = self.board.pending_request().is_some_and(|(_, l)| l > 0);
            if !deliverable {
                self.sub = 0; // the rest of this tick's instruction slots are idle too
                self.board.p0_per_step = self.p0_batch as u64;
                self.steps += self.p0_batch as u64;
                self.idle_skipped += self.p0_batch as u64;
                crate::prof::set(crate::prof::BOARD);
                self.board.tick_with(&mut self.cpu.mem);
                if let Some(f) = self.board.fault.take() { return Err(Stop::Model(f)); }
                return Ok(());
            }
        }
        if let Some((v, l)) = self.board.pending_request() {
            if l > self.cpu.imask() {
                self.cpu.take_interrupt(&mut self.board, v, l, true).map_err(Stop::Cpu)?;
                self.board.irq_dirty = true;
                self.interrupts += 1;
                *self.irq_hist.entry(v).or_default() += 1;
                self.history[self.hist_pos & 63] = v; self.hist_irq[self.hist_pos & 63] = true; self.hist_pos += 1;
            }
        }
        if self.debug_hooks { self.debug_checks()?; }
        self.history[self.hist_pos & 63] = self.cpu.pc; self.hist_irq[self.hist_pos & 63] = false; self.hist_pos += 1;
        if self.gfx_hle && self.hooks.rgah_range.is_some() && self.gfx_call() {
            self.sub += 1;
            if self.sub < self.cpu_per_p0 * self.p0_batch { return Ok(()); }
            self.sub = 0;
            self.board.p0_per_step = self.p0_batch as u64;
            self.steps += self.p0_batch as u64;
            self.board.tick_with(&mut self.cpu.mem);
            if let Some(f) = self.board.fault.take() { return Err(Stop::Model(f)); }
            return Ok(());
        }
        if self.debug_hooks { if let Some((lo, hi, m)) = &mut self.call_counts { let pc = self.cpu.pc; if pc >= *lo && pc <= *hi { if let Some(c) = m.get_mut(&pc) { *c += 1; } } } }
        crate::prof::set(crate::prof::CPU);
        self.cpu.step(&mut self.board).map_err(Stop::Cpu)?;
        if self.cpu_per_p0 > 1 { self.board.flush_deferred(&mut self.cpu.mem); }
        self.sub += 1;
        if self.sub < self.cpu_per_p0 * self.p0_batch { return Ok(()); }
        self.sub = 0;
        self.board.p0_per_step = self.p0_batch as u64;
        self.steps += self.p0_batch as u64;
        crate::prof::set(crate::prof::BOARD);
        self.board.tick_with(&mut self.cpu.mem);
        if let Some(f) = self.board.fault.take() { return Err(Stop::Model(f)); }
        Ok(())
    }

    /// The development hooks AR_SH_STOP_PC and AR_SH_ARG_LOG, checked before an instruction only while a hook is set
    /// (`debug_hooks`, refreshed by `run`; --count-calls is counted in `step`).
    #[cold]
    #[inline(never)]
    fn debug_checks(&mut self) -> Result<(), Stop> {
        if Some(self.cpu.pc) == self.stop_pc { return Err(Stop::Model(format!("reached stop PC {:08x}", self.cpu.pc))); }
        if !self.arg_log.is_empty() && self.arg_log.contains(&self.cpu.pc) {
            let r = &self.cpu.r;
            let at_r4 = self.cpu.mem.load_bytes(r[4], 8).map_or(String::new(), |b| b.iter().map(|x| format!("{x:02x}")).collect());
            eprintln!("[args] {:>11} pc {:08x} pr {:08x} r4 {:08x} r5 {:08x} r6 {:08x} r7 {:08x} @r4 {at_r4}", self.steps, self.cpu.pc, self.cpu.pr, r[4], r[5], r[6], r[7]);
        }
        Ok(())
    }

    /// SH7269 reset (watchdog): the CPU restarts from the reset vectors of the flash as it is now (power-on reset:
    /// vectors 0/4, manual reset: 8/12) with its power-on register state; RAM keeps its contents (a reset does not clear
    /// it); the board resets its on-chip peripherals (`Board::chip_reset`).
    pub fn chip_reset(&mut self, power_on: bool) -> Result<(), String> {
        let c = Checkpoint::power_on()?;
        let (pc_v, sp_v) = if power_on { (0, 4) } else { (8, 12) };
        let rd = |m: &Memory, a: usize| u32::from_be_bytes(m.flash[a..a + 4].try_into().unwrap());
        let mem = std::mem::replace(&mut self.cpu.mem, Memory { flash_cmd_mode: false, flash: Vec::new(), sdram: Vec::new(), oram: Vec::new() });
        let mut cpu = Cpu::new(mem);
        cpu.pc = rd(&cpu.mem, pc_v);
        cpu.r[15] = rd(&cpu.mem, sp_v);
        cpu.float_mode = self.cpu.float_mode;
        cpu.mem.flash_cmd_mode = false;
        self.cpu = cpu;
        self.board.chip_reset(&c)?;
        self.resets += 1;
        eprintln!("[reset] watchdog {} reset at step {}: pc {:08x} sp {:08x}", if power_on { "power-on" } else { "manual" }, self.steps, self.cpu.pc, self.cpu.r[15]);
        Ok(())
    }

    /// Intercept an RGAH entry point: render in software, return 0 (RGAH_OK) to the caller. False if `pc` is not one.
    #[inline]
    fn gfx_call(&mut self) -> bool {
        use crate::gfx::*;
        use crate::hle::RgahFn;
        let Some(f) = self.hooks.rgah_at(self.cpu.pc) else { return false };
        let r = [self.cpu.r[4], self.cpu.r[5], self.cpu.r[6], self.cpu.r[7]];
        let mut call = Call { mem: &mut self.cpu.mem, r, sp: self.cpu.r[15] };
        let ok = match f {
            RgahFn::FillRect => fill_rect(&mut call, &mut self.gfx),
            RgahFn::DrawImage => draw_image(&mut call, &mut self.gfx),
            RgahFn::BitBlt => bit_blt(&mut call, &mut self.gfx),
            RgahFn::BitBltFixedColor => bit_blt_fixed_color(&mut call, &mut self.gfx),
            RgahFn::Finish => Some(()),
        };
        if ok.is_none() && self.gfx.warned.len() < 20 {
            let msg = format!("RGAH_{} not rendered (args {:08x?}, sp {:08x})", f.name(), r, self.cpu.r[15]);
            eprintln!("[gfx] {msg}");
            self.gfx.warned.push(msg);
        }
        self.cpu.r[0] = 0;
        self.cpu.pc = self.cpu.pr;
        true
    }

    /// Save the complete emulator state (both CPUs, all peripheral models, memories) as gzip-compressed bincode.
    pub fn save_state(&self, path: &std::path::Path) -> Result<(), String> {
        let f = std::fs::File::create(path).map_err(|e| e.to_string())?;
        self.save_state_to(std::io::BufWriter::new(f))
    }

    /// Save the state into any writer (gzip-compressed bincode with the version header).
    pub fn save_state_to(&self, w: impl std::io::Write) -> Result<(), String> {
        let mut z = flate2::write::GzEncoder::new(w, flate2::Compression::fast());
        std::io::Write::write_all(&mut z, STATE_MAGIC).map_err(|e| e.to_string())?;
        bincode::serialize_into(&mut z, self).map_err(|e| e.to_string())?;
        z.finish().map_err(|e| e.to_string())?;
        Ok(())
    }

    /// Load a state written by `save_state`.
    pub fn load_state(path: &std::path::Path) -> Result<Machine, String> {
        let f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::load_state_from(std::io::BufReader::new(f)).map_err(|e| format!("{}: {e}", path.display()))
    }

    /// Load a state from any reader (as written by `save_state_to`).
    pub fn load_state_from(r: impl std::io::Read) -> Result<Machine, String> {
        let mut z = flate2::read::GzDecoder::new(r);
        let mut magic = [0u8; 16];
        std::io::Read::read_exact(&mut z, &mut magic).map_err(|e| e.to_string())?;
        if &magic != STATE_MAGIC { return Err(format!("not a state of this emulator version (header {:?})", String::from_utf8_lossy(&magic))); }
        let mut m: Machine = bincode::deserialize_from(z).map_err(|e| e.to_string())?;
        m.history = [0; 64]; m.hist_irq = [false; 64];
        m.stop_pc = std::env::var("AR_SH_STOP_PC").ok().and_then(|v| u32::from_str_radix(&v, 16).ok());
        m.arg_log = arg_log_env();
        if let Some(d) = &mut m.board.display { d.vdc.ensure_arc_registers(); d.vdc.ensure_video_registers(); }
        let p = crate::hle::detect(&m.cpu.mem.flash);
        m.set_profile(p.as_ref());
        Ok(m)
    }

    /// Recent PCs, oldest first, with registers (for stop diagnostics).
    pub fn dump_history(&self) -> String {
        let mut out = String::new();
        for k in 0..64 { let i = (self.hist_pos + k) & 63; let v = self.history[i]; out += &if self.hist_irq[i] { format!(" [IRQ vec {v}]") } else { format!(" {v:08x}") }; }
        out += &format!("
  R: {:08x?}
  SR {:08x} PR {:08x} GBR {:08x}", self.cpu.r, self.cpu.sr, self.cpu.pr, self.cpu.gbr);
        out
    }

    /// Run for `n` peripheral clocks (P0 ticks).
    pub fn run(&mut self, n: u64) -> Result<(), Stop> {
        self.debug_hooks = self.stop_pc.is_some() || !self.arg_log.is_empty() || self.call_counts.is_some();
        let end = self.steps + n;
        while self.steps < end { self.step()?; }
        Ok(())
    }

    /// Active GR layer as RGBA rows (portrait, as stored), or None if the layer is not reading a framebuffer.
    /// Formats RGB565, RGB888, ARGB1555, ARGB4444, ARGB8888, CLUT8/4/1, with RGB- and CLUT-index chroma-key.
    pub fn layer_rgba(&self, layer: usize) -> Option<(u32, u32, Vec<u8>)> {
        let l = self.layer_view(layer)?;
        let mut out = Vec::with_capacity(l.width * l.height * 4);
        for y in 0..l.height {
            let row = self.cpu.mem.slice(l.addr + y as u32 * l.stride, l.row_bytes())?;
            for x in 0..l.width { let p = l.pixel(row, x); out.extend([(p >> 16) as u8, (p >> 8) as u8, p as u8, (p >> 24) as u8]); }
        }
        Some((l.width as u32, l.height as u32, out))
    }

    /// Geometry, format, colour table and blend settings of an enabled GR layer (active registers).
    pub fn layer_view(&self, layer: usize) -> Option<LayerView> {
        let d = self.board.display.as_ref()?;
        let v = &d.vdc;
        let b = GR[layer];
        if v.active(b + 4) & 1 == 0 { return None; }
        let (flm1, flm3, flm5, flm6) = (v.active(b + 8), v.active(b + 16), v.active(b + 24), v.active(b + 28));
        let frame = if (flm1 >> 8) & 3 == 1 { flm3 & 1023 } else { 0 };
        let format = ((flm6 >> 28) & 15) as u8;
        if format > 8 { return None; }
        let clut = match format { 5 => (0..256).map(|i| v.clut(layer, i)).collect(), 6 => (0..16).map(|i| v.clut(layer, i)).collect(), _ => Vec::new() };
        let (ab1, ab7, ab8, ab9) = (v.active(b + 0x20), v.active(b + 0x38), v.active(b + 0x3C), v.active(b + 0x40));
        let ab = |off: u32| if layer > 0 { v.active(b + off) } else { 0 };
        let rect = |r: u64| (((r >> 16) & 0x7FF) as i32, (r & 0x7FF) as i32);
        let (grc_vs, _) = rect(v.active(b + 0x24));
        let (grc_hs, _) = rect(v.active(b + 0x28));
        let (arc_vs, arc_vw) = rect(ab(0x2C));
        let (arc_hs, arc_hw) = rect(ab(0x30));
        let arc_on = layer > 0 && ab1 & 0x1000 != 0;
        Some(LayerView {
            width: ((flm6 >> 16) & 1023) as usize + 1, height: ((flm5 >> 16) & 1023) as usize + 1,
            stride: ((flm3 >> 16) & 0x7FFF) as u32, addr: (v.active(b + 12) + frame * v.active(b + 20)) as u32,
            little: (flm6 >> 12) & 1 == 1, format, clut,
            ck: if ab7 & 1 != 0 { Some((ab8 as u32, ab9 as u32)) } else { None },
            disp_sel: (ab1 & 3) as u8, base: v.active(b + 0x4C) as u32,
            arc: if arc_on { Some((v.arc[layer].0, arc_hs - grc_hs, arc_vs - grc_vs, arc_hw, arc_vw)) } else { None },
            origin: (grc_hs, grc_vs),
        })
    }
}

/// One GR layer as the image synthesizer reads it.
pub struct LayerView {
    pub width: usize, pub height: usize, pub stride: u32, pub addr: u32, pub little: bool, pub format: u8,
    /// Displayed colour table (ARGB) for CLUT8/CLUT4.
    pub clut: Vec<u32>,
    /// Chroma-key when GR_CK_ON: GR_AB8 (KCLUT, KG, KB, KR bytes) and GR_AB9 replacement (A, G, B, R bytes).
    pub ck: Option<(u32, u32)>,
    /// GR_DISP_SEL: 0 background colour, 1 lower layer only, 2 current graphics, 3 blended.
    pub disp_sel: u8,
    /// GR_BASE (G, B, R bytes).
    pub base: u32,
    /// Rectangle fade: alpha and the rectangle (x, y, w, h) in layer pixels.
    pub arc: Option<(u8, i32, i32, i32, i32)>,
    /// Graphics area start (GR_GRC_HS, GR_GRC_VS) in panel timing coordinates.
    pub origin: (i32, i32),
}

impl LayerView {
    pub fn row_bytes(&self) -> usize {
        match self.format { 0 | 2 | 3 | 8 => self.width * 2, 1 | 4 => self.width * 4, 5 => self.width, 6 => self.width.div_ceil(2), _ => self.width.div_ceil(8) }
    }

    /// Pixel `x` of a frame-buffer row as 0xAARRGGBB, after chroma-key.
    #[inline]
    pub fn pixel(&self, row: &[u8], x: usize) -> u32 {
        let w16 = |i: usize| if self.little { u16::from_le_bytes([row[i], row[i + 1]]) as u32 } else { u16::from_be_bytes([row[i], row[i + 1]]) as u32 };
        let w32 = |i: usize| if self.little { u32::from_le_bytes([row[i], row[i + 1], row[i + 2], row[i + 3]]) } else { u32::from_be_bytes([row[i], row[i + 1], row[i + 2], row[i + 3]]) };
        let rgb = |a: u32, r: u32, g: u32, b: u32| (a << 24) | (r << 16) | (g << 8) | b;
        // n-bit to 8-bit with rounding (matches the firmware's chroma-key values, e.g. RGB565 0x6181 -> 63/31/08)
        let x5 = |v: u32| (v * 255 + 15) / 31;
        let x6 = |v: u32| (v * 255 + 31) / 63;
        let (argb, index) = match self.format {
            0 => { let v = w16(x * 2); (rgb(255, x5((v >> 11) & 31), x6((v >> 5) & 63), x5(v & 31)), None) }
            1 => (0xFF00_0000 | (w32(x * 4) & 0xFF_FFFF), None),
            2 => { let v = w16(x * 2); (rgb(if v & 0x8000 != 0 { 255 } else { 0 }, x5((v >> 10) & 31), x5((v >> 5) & 31), x5(v & 31)), None) }
            3 => { let v = w16(x * 2); (rgb(((v >> 12) & 15) * 17, ((v >> 8) & 15) * 17, ((v >> 4) & 15) * 17, (v & 15) * 17), None) }
            4 => (w32(x * 4), None),
            8 => {
                // YCbCr422: Cb, Y0, Cr, Y1 per pixel pair (ITU-R BT.601 video range)
                let k = (x / 2) * 4;
                let (cb, y, cr) = (row[k] as f64 - 128.0, (row[k + 1 + 2 * (x % 2)] as f64 - 16.0) * 1.164, row[k + 2] as f64 - 128.0);
                let c = |v: f64| v.round().clamp(0.0, 255.0) as u32;
                (rgb(255, c(y + 1.596 * cr), c(y - 0.392 * cb - 0.813 * cr), c(y + 2.017 * cb)), None)
            }
            5 => { let i = row[x] as usize; (self.clut.get(i).copied().unwrap_or(0), Some(i as u32)) }
            6 => { let i = ((row[x / 2] >> if x % 2 == 0 { 4 } else { 0 }) & 15) as usize; (self.clut.get(i).copied().unwrap_or(0), Some(i as u32)) }
            _ => { let i = ((row[x / 8] >> (7 - x % 8)) & 1) as u32; (if i == 1 { 0xFFFF_FFFF } else { 0xFF00_0000 }, Some(i)) }
        };
        if let Some((key, rep)) = self.ck {
            let hit = match index {
                Some(i) => i == key >> 24,
                None => argb & 0xFF_FFFF == ((key & 0xFF) << 16) | (((key >> 16) & 0xFF) << 8) | ((key >> 8) & 0xFF),
            };
            if hit { return rgb(rep >> 24, rep & 0xFF, (rep >> 16) & 0xFF, (rep >> 8) & 0xFF); }
        }
        argb
    }

    /// Blend weight of pixel (x, y) with colour `argb` over the layers below, by GR_DISP_SEL and the fade rectangle.
    #[inline]
    pub fn alpha(&self, x: usize, y: usize, argb: u32) -> u32 {
        match self.disp_sel {
            1 => 0,
            0 | 2 => 255,
            _ => match self.arc {
                Some((a, ax, ay, aw, ah)) if (x as i32) >= ax && (x as i32) < ax + aw && (y as i32) >= ay && (y as i32) < ay + ah => a as u32 * (argb >> 24) / 255,
                _ => argb >> 24,
            },
        }
    }

    /// Displayed colour: GR_BASE for DISP_SEL 0, otherwise the pixel.
    #[inline]
    pub fn colour(&self, argb: u32) -> u32 {
        if self.disp_sel == 0 { ((self.base & 0xFF) << 16) | (((self.base >> 16) & 0xFF) << 8) | ((self.base >> 8) & 0xFF) } else { argb & 0xFF_FFFF }
    }
}

impl Machine {
    /// All active GR layers blended bottom (GR at 0xFFFF_7600) to top, as the panel shows them (portrait RGBA).
    pub fn compose_rgba(&self) -> Option<(u32, u32, Vec<u8>)> {
        let (w, h) = (0..GR.len()).filter_map(|k| self.layer_view(k).map(|l| (l.width, l.height))).max_by_key(|&(w, h)| w * h)?;
        let mut out = vec![0u32; w * h];
        if !self.compose_into(w, h, |x, y| y * w + x, &mut out) { return None; }
        Some((w as u32, h as u32, out.iter().flat_map(|&p| [(p >> 16) as u8, (p >> 8) as u8, p as u8, 255]).collect()))
    }

    /// The panel image in landscape orientation (`w` x `h` = layer height x layer width, rotated counter-clockwise), all
    /// active GR layers blended bottom to top, as 0x00RRGGBB written straight into `out`. False if no layer is active.
    pub fn render_landscape(&self, out: &mut [u32], w_out: usize, h_out: usize) -> bool {
        // rotate counter-clockwise: portrait (x, y) -> landscape (y, width-1-x)
        self.compose_into(h_out, w_out, |x, y| (h_out - 1 - x) * w_out + y, out)
    }

    /// Scaler-0 post-scaling video plane (the lower layer of graphics 1): the write-back frame read through the GR0
    /// frame-buffer registers (frame chosen by the scaler when GR_FLM1.BUFSEL = 0), RES_IN size (SCL0_US4), placed at
    /// the output position SCL0_US2/US3 relative to the panel graphics area, on black. Drawn when graphics 1 shows its
    /// lower layer (GR_DISP_SEL = 1) and RES_DISP_ON is 0 (frame display on).
    fn draw_video_plane(&self, w: usize, h: usize, panel: (i32, i32), index: &impl Fn(usize, usize) -> usize, out: &mut [u32]) -> bool {
        let Some(d) = self.board.display.as_ref() else { return false };
        let v = &d.vdc;
        let b = GR[0];
        if v.active(b + 4) & 1 == 0 || v.active(b + 0x20) & 3 != 1 || v.active(0xFFFF_7564) & 1 != 0 { return false; }
        let (flm1, flm3, flm6) = (v.active(b + 8), v.active(b + 16), v.active(b + 28));
        let frame = if (flm1 >> 8) & 3 == 1 { flm3 & 1023 } else { v.wr_frame as u64 };
        let view = LayerView {
            width: (v.active(0xFFFF_7554) & 0x7FF) as usize, height: ((v.active(0xFFFF_7554) >> 16) & 0x7FF) as usize,
            stride: ((flm3 >> 16) & 0x7FFF) as u32, addr: (v.active(b + 12) + frame * v.active(b + 20)) as u32,
            little: (flm6 >> 12) & 1 == 1, format: ((flm6 >> 28) & 15) as u8, clut: Vec::new(), ck: None, disp_sel: 2, base: 0, arc: None, origin: (0, 0),
        };
        let x0 = ((v.active(0xFFFF_7550) >> 16) & 0x7FF) as i64 - panel.0 as i64;
        let y0 = ((v.active(0xFFFF_754C) >> 16) & 0x7FF) as i64 - panel.1 as i64;
        for y in 0..h { for x in 0..w { out[index(x, y)] = 0; } }
        for vy in 0..view.height {
            let py = y0 + vy as i64;
            if py < 0 || py >= h as i64 { continue; }
            let Some(row) = self.cpu.mem.slice(view.addr + vy as u32 * view.stride, view.row_bytes()) else { continue };
            for vx in 0..view.width {
                let px = x0 + vx as i64;
                if px < 0 || px >= w as i64 { continue; }
                out[index(px as usize, py as usize)] = view.pixel(row, vx) & 0xFF_FFFF;
            }
        }
        true
    }

    /// Blend the enabled layers of portrait size `w` x `h` bottom to top into `out` at `index(x, y)`.
    fn compose_into(&self, w: usize, h: usize, index: impl Fn(usize, usize) -> usize, out: &mut [u32]) -> bool {
        // Panel origin: the graphics area of a full-panel layer (layers smaller than the panel sit at their own GRC start).
        let views: Vec<Option<LayerView>> = (0..GR.len()).map(|k| self.layer_view(k)).collect();
        let panel = views.iter().flatten().find(|l| l.width == w && l.height == h).map_or((0, 0), |l| l.origin);
        let mut any = self.draw_video_plane(w, h, panel, &index, out);
        for l in views.into_iter().flatten() {
            if l.disp_sel == 1 && any { continue; }
            let (ox, oy) = (l.origin.0 - panel.0, l.origin.1 - panel.1);
            if l.width as i32 + ox <= 0 || l.height as i32 + oy <= 0 || ox >= w as i32 || oy >= h as i32 { continue; }
            if !any && (l.width != w || l.height != h) { for y in 0..h { for x in 0..w { out[index(x, y)] = 0; } } }
            let bottom = !any;
            for ly in 0..l.height {
                let y = ly as i32 + oy;
                if y < 0 || y >= h as i32 { continue; }
                let Some(row) = self.cpu.mem.slice(l.addr + ly as u32 * l.stride, l.row_bytes()) else { continue };
                for lx in 0..l.width {
                    let x = lx as i32 + ox;
                    if x < 0 || x >= w as i32 { continue; }
                    let p = l.pixel(row, lx);
                    let (x, y) = (x as usize, y as usize);
                    let o = index(x, y);
                    let c = l.colour(p);
                    let a = if bottom { 255 } else { l.alpha(lx, ly, p) };
                    if a >= 255 { out[o] = c; }
                    else if a > 0 {
                        let d0 = out[o];
                        let mix = |s: u32, dst: u32| (s * a + dst * (255 - a)) / 255;
                        out[o] = (mix((c >> 16) & 255, (d0 >> 16) & 255) << 16) | (mix((c >> 8) & 255, (d0 >> 8) & 255) << 8) | mix(c & 255, d0 & 255);
                    }
                }
            }
            any = true;
        }
        any
    }

    /// One line per GR layer: enabled, format, size, address (diagnostics).
    pub fn layer_report(&self) -> String {
        let Some(d) = self.board.display.as_ref() else { return "no display".into() };
        let v = &d.vdc;
        GR.iter().enumerate().map(|(i, &b)| {
            let (flm5, flm6) = (v.active(b + 24), v.active(b + 28));
            let arc = if i > 0 { format!(" ab6 {:08x} alpha {} ab8 {:08x} ab9 {:08x}", v.active(b + 0x34), v.arc[i].0, v.active(b + 0x3C), v.active(b + 0x40)) } else { String::new() };
            format!("GR{i}@{b:08x}: en {} fmt {} le {} {}x{} addr {:08x} ab1 {:x} ab2 {:x} ab3 {:x} ab4 {:x} ab5 {:x} ab7 {:x}{arc}", v.active(b + 4) & 1, (flm6 >> 28) & 15, (flm6 >> 12) & 1, ((flm6 >> 16) & 1023) + 1, ((flm5 >> 16) & 1023) + 1, v.active(b + 12),
                v.active(b + 0x20), v.active(b + 0x24), v.active(b + 0x28), if i > 0 { v.active(b + 0x2c) } else { 0 }, if i > 0 { v.active(b + 0x30) } else { 0 }, v.active(b + 0x38))
        }).collect::<Vec<_>>().join("; ")
    }
}

/// Rotate a portrait RGBA image 90 degrees counter-clockwise (the unit's landscape view).
pub fn rotate_ccw(w: u32, h: u32, src: &[u8]) -> (u32, u32, Vec<u8>) {
    let mut out = vec![0u8; src.len()];
    for y in 0..h { for x in 0..w {
        let (nx, ny) = (y, w - 1 - x);
        let (s, d) = (((y * w + x) * 4) as usize, ((ny * h + nx) * 4) as usize);
        out[d..d + 4].copy_from_slice(&src[s..s + 4]);
    } }
    (h, w, out)
}

/// Minimal 24-bit BMP writer (no extra dependencies).
pub fn write_bmp(path: &std::path::Path, w: u32, h: u32, rgba: &[u8]) -> std::io::Result<()> {
    let row = (w * 3 + 3) & !3;
    let size = 54 + row * h;
    let mut f = Vec::with_capacity(size as usize);
    f.extend(b"BM"); f.extend(size.to_le_bytes()); f.extend([0u8; 4]); f.extend(54u32.to_le_bytes());
    f.extend(40u32.to_le_bytes()); f.extend((w as i32).to_le_bytes()); f.extend((h as i32).to_le_bytes());
    f.extend(1u16.to_le_bytes()); f.extend(24u16.to_le_bytes()); f.extend([0u8; 24]);
    for y in (0..h).rev() {
        for x in 0..w { let i = ((y * w + x) * 4) as usize; f.extend([rgba[i + 2], rgba[i + 1], rgba[i]]); }
        f.extend(vec![0u8; (row - w * 3) as usize]);
    }
    std::fs::write(path, f)
}

fn arg_log_env() -> Vec<u32> {
    std::env::var("AR_SH_ARG_LOG").map(|v| v.split(',').filter_map(|p| u32::from_str_radix(p.trim().trim_start_matches("0x"), 16).ok()).collect()).unwrap_or_default()
}
