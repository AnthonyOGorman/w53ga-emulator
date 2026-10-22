//! On-chip peripherals of the SH7269 and board devices.
//!
//! Timing: one P0 tick per executed instruction step (functional clock, FRQCR I:P0 = 8:1
//! with roughly one instruction per I-clock... approximated as 1 P0/step). Per step, after the instruction:
//! peripherals tick, then at most one interrupt is accepted (highest IPR level above SR.IMASK; ties by vector).

pub mod audio;
pub mod display;
pub mod dmac;
pub mod iic;
pub mod nor;
pub mod ir;
pub mod jcu;
pub mod regs;
pub mod rtc;
pub mod scif;
pub mod rgpvg;
pub mod wdt;
pub mod sdhi;
pub mod subcpu;

use crate::checkpoint::Checkpoint;
use display::{DisplayClock, Vdc4};
use audio::{Src, Ssi};
use dmac::Dmac;
use iic::Iic2;
use ir::IrRemote;
use jcu::{Jcu, JcuBus};
use crate::mem::Memory;
use rtc::Rtc;
use scif::Scif;
use sdhi::{Sdhi, SdhiEmpty};
use std::collections::HashMap;
use regs::Latches;

pub const CMSTR: u32 = 0xFFFE_C000;
pub const STBCR7: u32 = 0xFFFE_0418;
pub const IPR12: u32 = 0xFFFE_0C0C;
pub const IPR10: u32 = 0xFFFE_0C08;
pub const STBCR10: u32 = 0xFFFE_0444;
pub const IPR21: u32 = 0xFFFE_0C1E;
pub const IPR20: u32 = 0xFFFE_0C1C;
pub const DMAOR: u32 = 0xFFFE_1200;
pub const IPR13: u32 = 0xFFFE_0C0E;
const MTU_TSTR: u32 = 0xFFFE_4280;
const STBCR3: u32 = 0xFFFE_0408;
const TCR1: u32 = 0xFFFE_4380;
const TIOR1: u32 = 0xFFFE_4382;
const TIER1: u32 = 0xFFFE_4384;
const TSR1: u32 = 0xFFFE_4385;
const TCNT1: u32 = 0xFFFE_4386;
const TGRA1: u32 = 0xFFFE_4388;
const TGRB1: u32 = 0xFFFE_438A;
/// ADC data registers (A/D converter 0xE800_5800): AN5 = ADDRF carries the supply-voltage divider.
pub const ADC_BASE: u32 = 0xE800_5800;
pub const VDEC_BASE: u32 = 0xFFFF_A000;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Board {
    pub latches: Latches,
    pub cmt_remainder: [u64; 2],
    pub p0_per_step: u64,
    pub cmt_matches: [u64; 2],
    /// Latch indices for the per-step hot path: CMSTR, STBCR7, IPR12, CMCSR0, CMCNT0, CMCOR0, CMCSR1, CMCNT1, CMCOR1.
    hot: [usize; 9],
    i_ipr10: usize,
    i_stbcr10: usize,
    pub display: Option<DisplayClock>,
    pub iic2: Option<Iic2>,
    pub iic1: Option<Iic2>,
    pub rtc: Option<Rtc>,
    pub scif7: Option<Scif>,
    pub scif1: Option<Scif>,
    pub scif6: Option<Scif>,
    pub dmac: Dmac,
    /// ADC conversion results per channel AN0..AN7 (10-bit code << 6); None = channel not wired in this board model.
    pub adc: [Option<u16>; 8],
    pub adc_csr: u64,
    /// VDEC register file (video decoder; the unit has no camera signal: VSYNCSR = 0x0800).
    pub vdec: HashMap<u32, u64>,
    i_ipr21: usize,
    /// MTU1 prescaler phase (P0/64) and the infrared remote driving TIOC1A.
    pub mtu1_remainder: u64,
    pub ir: IrRemote,
    pub jcu: Option<Jcu>,
    pub src: Src,
    pub ssi: Option<Ssi>,
    /// SD host 0 with the virtual card (attached by `attach_sd`).
    pub sdhi: Option<Sdhi>,
    pub sdhi1: SdhiEmpty,
    /// Companion MCU peer on SCIF7.
    pub peer: subcpu::Peer,
    /// Emulated companion controller on SCIF7 (replaces the scripted peer when attached).
    pub companion: Option<Box<crate::rl78::companion::Companion>>,
    /// Pending JCU register write (needs memory access, performed by `tick_with`); see `write`.
    jcu_writes: Vec<(u32, usize, u64, u64)>,
    /// Absolute step count (P0 ticks at 1 P0/step), advanced by `tick`.
    pub now: u64,
    /// MTU1 latch indices: TSTR, STBCR3, TCR1, TIOR1, TIER1, TSR1, TCNT1, TGRA1, TGRB1, IPR13.
    mtu1_i: [usize; 10],
    /// MTU channel 3/4 prescaler phases (a fraction of one count; not kept in saved states).
    #[serde(skip)]
    mtu34_remainder: [u64; 2],
    /// First model error (unsupported configuration); the caller stops on it.
    pub fault: Option<String>,
    /// Stores to registers without a model (accepted and remembered, and logged once).
    pub observed: HashMap<(u32, usize), u64>,
    /// CS0 NOR flash command interface (array = `Memory::flash`).
    pub nor: nor::Nor,
    /// Interrupt state may have changed since `cached_request` was computed (any tick or MMIO access).
    #[serde(skip, default = "yes")]
    pub irq_dirty: bool,
    #[serde(skip)]
    cached_request: Option<Request>,
    /// RSPI0/RSPI1 register files (transfers complete immediately; transmitted bytes are logged).
    pub rspi: [Vec<u8>; 2],
    pub rspi_tx: [Vec<u8>; 2],
    /// Emulated OBD-II adapter on SCIF6 (plugged in by the front end; not part of saved states).
    #[serde(skip)]
    pub obd2: Option<crate::obd2::Obd2>,
    /// Emulated drive recorder control link on SCIF1 (plugged in by the front end).
    #[serde(skip)]
    pub dvr: Option<crate::dvr::Dvr>,
    /// NTSC camera signal at the video decoder input (the recorder's video output).
    #[serde(skip)]
    pub camera: bool,
    /// Camera picture source and frame timing (when `camera`).
    #[serde(skip)]
    pub cam: crate::camera::Camera,
    /// Without a camera signal the video decoder free-runs its sync generator: input Vsyncs at the NTSC field rate
    /// reach the VDC (needed when SCL0_FRC3 makes the output follow the input). Off for strict
    /// hardware-input-only runs.
    #[serde(skip)]
    pub vdec_free_run: bool,
    /// Graphics core start-up model (only the RGAH initialisation talks to it; not kept in saved states).
    #[serde(skip)]
    pub gpu: rgpvg::Rgpvg,
    /// MMIO accesses by the CPU (all devices), for runtime HLE detection (`crate::detect`).
    #[serde(skip)]
    pub mmio_accesses: u64,
    /// Watchdog (only the boot loader starts it, to restart the unit after an update).
    #[serde(skip, default = "wdt::Wdt::power_on")]
    pub wdt: wdt::Wdt,
    /// While the busy-wait fast-forward looks for a loop (`Machine::spin_find`): the CPU's MMIO accesses (address, size,
    /// write).
    #[serde(skip)]
    pub spin_log: Option<Vec<(u32, usize, bool)>>,
}

/// Plain registers without behaviour the firmware relies on, at their reset values :
/// motor-control PWM unit 2 (PWCR1/PWCR2/PWPR2/PWCYR2/PWBFR2A..G; no compare-match interrupt is enabled by the firmware).
fn reset_register_file() -> HashMap<(u32, usize), u64> {
    let mut m = HashMap::new();
    for (a, n, v) in [(0xFFFE_F4E0, 1, 0xC0), (0xFFFE_F4F0, 1, 0xC0), (0xFFFE_F4F4, 1, 0), (0xFFFE_F4F6, 2, 0xFFFF),
        (0xFFFE_F4F8, 2, 0), (0xFFFE_F4FA, 2, 0), (0xFFFE_F4FC, 2, 0), (0xFFFE_F4FE, 2, 0), (0xFFFE_F406, 1, 0)] { m.insert((a, n), v); }
    m
}

/// RSPI channel and register offset for an address (RSPI0 0xE800_E000, RSPI1 0xE800_E800).
fn rspi_reg(a: u32) -> Option<(usize, usize)> {
    match a { 0xE800_E000..=0xE800_E02F => Some((0, (a - 0xE800_E000) as usize)), 0xE800_E800..=0xE800_E82F => Some((1, (a - 0xE800_E800) as usize)), _ => None }
}

/// JCU access to memory (coded input, decoded output).
struct JcuMem<'a>(&'a mut Memory);
impl JcuBus for JcuMem<'_> {
    fn read(&mut self, a: u32, n: usize) -> Option<Vec<u8>> {
        // Coded data may run past the end of a region: read what exists (flash/RAM) up to n bytes.
        let mut v = Vec::with_capacity(n);
        for k in 0..n as u32 { match self.0.load_bytes(a + k, 1) { Some(b) => v.push(b[0]), None => break } }
        if v.is_empty() { None } else { Some(v) }
    }
    fn write(&mut self, a: u32, d: &[u8]) -> bool { self.0.store_bytes(a, d) }
}

fn yes() -> bool { true }

/// A pending interrupt request: (vector, priority level).
pub type Request = (u32, u32);

impl Board {
    pub fn from_checkpoint(c: &Checkpoint) -> Result<Board, String> {
        let latches = Latches::from_checkpoint(&c.latches)?;
        let mut hot = [0usize; 9];
        for (k, a) in [CMSTR, STBCR7, IPR12, 0xFFFE_C002, 0xFFFE_C004, 0xFFFE_C006, 0xFFFE_C008, 0xFFFE_C00A, 0xFFFE_C00C].into_iter().enumerate() {
            hot[k] = latches.find(a, 1).ok_or(format!("missing latch {a:08x}"))?;
        }
        let i_ipr10 = latches.find(IPR10, 1).ok_or("missing IPR10")?;
        let i_stbcr10 = latches.find(STBCR10, 1).ok_or("missing STBCR10")?;
        let display = match &c.display { Some(r) => { let mut d = DisplayClock::from_raw(r.clone())?; d.vdc.ensure_arc_registers(); d.vdc.ensure_video_registers(); Some(d) } None => None };
        let iic2 = match &c.iic2 { Some(r) => Some(Iic2::from_raw(r.clone())?), None => Some(Iic2::iic2_reset()) };
        let iic1 = match &c.iic1 { Some(r) => Some(Iic2::iic1_from_raw(r.clone())?), None => Some(Iic2::iic1_reset()) };
        let rtc = match &c.rtc { Some(r) => Some(Rtc::from_raw(r.clone())?), None => None };
        let scif7 = match &c.scif7 { Some((r, active)) => Some(Scif::scif7_from_raw(r.clone(), *active)?), None => Some(Scif::reset(0xE800_A800, [286, 287, 288, 289])) };
        let mut vdec = HashMap::new();
        if let Some(v) = &c.vdec { for (k, x) in v.values.iter().chain(v.active.iter()) { vdec.insert(k.parse::<u64>().map_err(|e| e.to_string())? as u32, *x); } }
        vdec.insert(0xFFFF_A080, 0x0800);
        let mut adc = [None; 8];
        adc[5] = Some(0x4DC0); // 12.0 V supply: code 311 (fixture)
        let i_ipr21 = latches.find(IPR21, 1).ok_or("missing IPR21")?;
        let mut mtu1_i = [0usize; 10];
        for (k, a) in [MTU_TSTR, STBCR3, TCR1, TIOR1, TIER1, TSR1, TCNT1, TGRA1, TGRB1, IPR13].into_iter().enumerate() {
            mtu1_i[k] = latches.find(a, 1).ok_or(format!("missing MTU1 latch {a:08x}"))?;
        }
        let scif1 = match &c.scif1 { Some((r, a)) => Some(Scif::from_adapter(r.clone(), 0xE800_7800, [262, 263, 264, 265], *a)?), None => Some(Scif::reset(0xE800_7800, [262, 263, 264, 265])) };
        let scif6 = match &c.scif6 { Some((r, a)) => Some(Scif::from_adapter(r.clone(), 0xE800_A000, [282, 283, 284, 285], *a)?), None => Some(Scif::reset(0xE800_A000, [282, 283, 284, 285])) };
        let mut dmac = Dmac::new();
        // DMAC registers kept as generic latches (e.g. DMA5 SDHI, DMARS2).
        for ch in 0..16u32 {
            for (k, slot) in [(0u32, 0usize), (4, 1), (8, 2), (12, 3)] {
                if let Some(i) = latches.find(dmac::BASE + 16 * ch + k, 4) {
                    let v = latches.regs[i].value as u32;
                    let c = &mut dmac.ch[ch as usize];
                    match slot { 0 => c.sar = v, 1 => c.dar = v, 2 => c.tcr = v, _ => { c.chcr = v; c.arm_count = c.tcr; } }
                }
            }
        }
        for k in 0..8u32 { if let Some(i) = latches.find(dmac::DMARS_BASE + 4 * k, 2) { dmac.dmars[k as usize] = latches.regs[i].value as u16; } }
        for (ch, r, arm) in &c.dma { let x = &mut dmac.ch[*ch]; x.sar = r[0] as u32; x.dar = r[1] as u32; x.tcr = r[2] as u32; x.chcr = r[3] as u32; x.arm_count = *arm as u32; }
        for (k, w) in &c.dmars { dmac.dmars[*k] = *w as u16; }
        if let Some(r) = c.dma1_rdar { dmac.ch[1].rdar = r as u32; }
        dmac.refresh();
        Ok(Board { latches, cmt_remainder: c.cmt_remainder, p0_per_step: 1, cmt_matches: [0; 2], hot, i_ipr10, i_stbcr10, display, iic2, iic1, rtc, scif7,
            scif1, scif6, dmac, adc, adc_csr: 0x203D, vdec, i_ipr21,
            mtu1_remainder: c.mtu_remainder.get(1).copied().unwrap_or(0), ir: IrRemote::default(), now: c.steps, mtu1_i, mtu34_remainder: [0; 2],
            jcu: match &c.jcu { Some(r) => Some(Jcu::from_raw(r.clone())?), None => None }, jcu_writes: Vec::new(),
            src: { let mut s = Src::new(); if let Some(r) = &c.src { let _ = s.write(audio::SRC_BASE + 8, 2, r.input_control as u64); let _ = s.write(audio::SRC_BASE + 0xA, 2, r.output_control as u64); } s },
            ssi: Some(c.ssi.as_ref().map_or_else(|| Ssi::new(0, 0, 0, false), |t| Ssi::new(t.cr as u32, t.fcr as u32, t.clock.fifo.len(), t.clock.ten))),
            sdhi: None, sdhi1: SdhiEmpty::new(c.sdhi1.as_ref()), peer: subcpu::Peer::new(1), companion: None,
            fault: None, observed: reset_register_file(), nor: nor::Nor::default(), irq_dirty: true, cached_request: None, rspi: [vec![0; 0x30], vec![0; 0x30]], rspi_tx: [Vec::new(), Vec::new()], obd2: None, dvr: None, camera: false, cam: Default::default(), vdec_free_run: false, gpu: Default::default(), mmio_accesses: 0, wdt: wdt::Wdt::power_on(), spin_log: None })
    }

    /// Chip reset of the SH7269 (watchdog overflow): every on-chip peripheral returns to its power-on state (`c`, the
    /// power-on checkpoint). What lies outside the chip, or is battery-backed, carries over: the NOR flash (array and
    /// protection state), the RTC, the RL78 controller, the SD card and Wi-Fi module, the accessories, the IR remote and
    /// the time base. WRCSR keeps WOVF, so software can tell a watchdog reset.
    pub fn chip_reset(&mut self, c: &Checkpoint) -> Result<(), String> {
        let mut b = Board::from_checkpoint(c)?;
        b.now = self.now;
        b.p0_per_step = self.p0_per_step;
        b.nor = std::mem::take(&mut self.nor);
        b.rtc = self.rtc.take();
        b.ir = std::mem::take(&mut self.ir);
        if let (Some(old), Some(raw)) = (self.sdhi.take(), c.sdhi.clone()) { b.sdhi = Some(Sdhi::from_raw(raw, old.card)?); }
        b.sdhi1.card = self.sdhi1.card.take();
        b.companion = self.companion.take();
        if let (Some(comp), Some(s)) = (b.companion.as_mut(), b.scif7.as_ref()) { comp.sync(s); }
        b.obd2 = self.obd2.take().map(|o| { let mut n = crate::obd2::Obd2::new(0); n.car = o.car; n });
        b.dvr = self.dvr.take().map(|_| crate::dvr::Dvr::new(0));
        b.camera = self.camera;
        b.vdec_free_run = self.vdec_free_run;
        b.set_tdfe_auto_clear(self.scif7.as_ref().is_some_and(|s| s.tdfe_auto_clear));
        b.wdt.wrcsr |= self.wdt.wrcsr & 0x80;
        *self = b;
        Ok(())
    }

    /// TDFE clearing rule for all SCIF channels (see `Scif::tdfe_auto_clear`).
    pub fn set_tdfe_auto_clear(&mut self, on: bool) {
        for s in [&mut self.scif1, &mut self.scif6, &mut self.scif7].into_iter().flatten() { s.tdfe_auto_clear = on; }
    }

    /// Attach a card built from an SD folder by the FAT32 synthesiser, with optional extra files (card path, host
    /// file). Returns the skipped (non-8.3) host paths.
    pub fn attach_sd_folder(&mut self, c: &Checkpoint, root: &std::path::Path, extra: &[(String, std::path::PathBuf)]) -> Result<Vec<String>, String> {
        let raw = c.sdhi.clone().ok_or("checkpoint has no SDHI state")?;
        let (vol, skipped) = crate::fat32::Volume::build(root, extra)?;
        let mut card = sdhi::VirtualSd::from_volume(vol);
        card.root = Some(root.to_path_buf());
        self.sdhi = Some(Sdhi::from_raw(raw, card)?);
        Ok(skipped)
    }

    /// Attach a card synthesised elsewhere (e.g. from a browser's file list) with the checkpoint's host controller.
    pub fn attach_sd_volume(&mut self, c: &Checkpoint, vol: crate::fat32::Volume) -> Result<(), String> {
        let raw = c.sdhi.clone().ok_or("checkpoint has no SDHI state")?;
        self.sdhi = Some(Sdhi::from_raw(raw, sdhi::VirtualSd::from_volume(vol))?);
        Ok(())
    }

    /// Serve a rebuilt volume from the same card (e.g. the user's SD folder in a web front end): the host controller
    /// state and the CID stay, only where sector data comes from changes. The folder must hold the same files, so the
    /// cluster layout matches what the firmware has cached.
    pub fn replace_sd_volume(&mut self, vol: crate::fat32::Volume) -> Result<(), String> {
        let s = self.sdhi.as_mut().ok_or("no SD host")?;
        let cid = s.card.cid;
        s.card = sdhi::VirtualSd::from_volume(vol);
        s.card.cid = cid;
        Ok(())
    }

    /// Give the card this CID (card identification register) instead of the emulator's default. Software that binds
    /// data to a particular card checks it, so to use files copied from a real card, give that card's CID.
    pub fn set_card_cid(&mut self, cid: [u8; 16]) -> Result<(), String> {
        self.sdhi.as_mut().ok_or("no SD card attached")?.card.cid = cid;
        Ok(())
    }

    /// Address ranges implemented by peripheral models.
    pub fn owns(&self, a: u32, n: usize) -> bool {
        let cmt = (0xFFFE_C000..0xFFFE_C00E).contains(&a);
        let intc = (0xFFFE_0800..0xFFFE_0C2A).contains(&a);
        if (0xF000_0000..0xF200_0000).contains(&a) || rspi_reg(a).is_some() { return true; }
        if self.display.is_some() && Vdc4::handles(a, n) { return true; }
        if self.iic2.as_ref().is_some_and(|c| c.handles(a, n)) || self.iic1.as_ref().is_some_and(|c| c.handles(a, n)) { return true; }
        if self.rtc.as_ref().is_some_and(|r| r.handles(a, n)) { return true; }
        if self.scif7.as_ref().is_some_and(|s| s.handles(a, n)) { return true; }
        if self.scif1.as_ref().is_some_and(|s| s.handles(a, n)) || self.scif6.as_ref().is_some_and(|s| s.handles(a, n)) { return true; }
        if Dmac::handles(a, n) { return true; }
        if self.jcu.is_some() && Jcu::handles(a, n) { return true; }
        if Src::handles(a, n) || (self.ssi.is_some() && Ssi::handles(a, n)) { return true; }
        if self.sdhi.is_some() && Sdhi::handles(a, n) { return true; }
        if SdhiEmpty::handles(a, n) || rgpvg::Rgpvg::handles(a) { return true; }
        if (ADC_BASE..ADC_BASE + 0x400).contains(&a) || (VDEC_BASE..VDEC_BASE + 0x300).contains(&a) { return true; }
        (cmt || intc) && self.latches.find(a, n).is_some()
    }

    /// The register latch that alone answers a CPU read of `a`/`n`: no device model is involved (`read` would reach
    /// the latch file), so the read's only effect is the latch's own read behaviour (status flags noted as read), which
    /// a second read from the same latch state repeats exactly. None for anything else, and for GPIO data registers
    /// (their value also depends on the direction latch). For the busy-wait fast-forward (`Machine::spin_find`).
    pub fn latch_only(&self, a: u32, n: usize) -> Option<usize> {
        if nor::Nor::handles(a) || (0xF000_0000..0xF200_0000).contains(&a) || rspi_reg(a).is_some() { return None; }
        if self.display.is_some() && Vdc4::handles(a, n) { return None; }
        if self.iic2.as_ref().is_some_and(|c| c.handles(a, n)) || self.iic1.as_ref().is_some_and(|c| c.handles(a, n)) { return None; }
        if self.rtc.as_ref().is_some_and(|r| r.handles(a, n)) { return None; }
        if [&self.scif7, &self.scif1, &self.scif6].iter().any(|s| s.as_ref().is_some_and(|s| s.handles(a, n))) { return None; }
        if (self.sdhi.is_some() && Sdhi::handles(a, n)) || SdhiEmpty::handles(a, n) || Src::handles(a, n) || (self.ssi.is_some() && Ssi::handles(a, n)) { return None; }
        if (self.jcu.is_some() && Jcu::handles(a, n)) || Dmac::handles(a, n) { return None; }
        if (ADC_BASE..ADC_BASE + 0x400).contains(&a) || (VDEC_BASE..VDEC_BASE + 0x300).contains(&a) || wdt::Wdt::handles(a) || rgpvg::Rgpvg::handles(a) { return None; }
        let i = self.latches.find(a, n)?;
        if self.latches.regs[i].behavior == regs::Behavior::GpioData { return None; }
        Some(i)
    }

    /// The state of latch `i` that its reads depend on or change (value, flags noted as read).
    pub fn latch_state(&self, i: usize) -> (u64, u64, bool) { let r = &self.latches.regs[i]; (r.value, r.status_read, r.cmf_read) }

    pub fn read(&mut self, a: u32, n: usize) -> Option<u64> {
        // Memory-mapped cache address/data arrays: the emulator has no cache, so every line reads invalid (0).
        if (0xF000_0000..0xF200_0000).contains(&a) { return Some(0); }
        if let Some((ch, o)) = rspi_reg(a) {
            let r = &self.rspi[ch];
            let mut v = 0u64;
            for i in 0..n { let b = if o + i == 3 { r[3] | 0x60 } else { r[o + i] }; v = (v << 8) | b as u64; } // SPSR: SPTEF | TEND
            return Some(v);
        }
        if let Some(d) = &self.display { if Vdc4::handles(a, n) {
            return match d.vdc.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("VDC read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        for c in [&mut self.iic2, &mut self.iic1].into_iter().flatten() { if c.handles(a, n) {
            return match c.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("IIC read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        if let Some(r) = &mut self.rtc { if r.handles(a, n) {
            return match r.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("RTC read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        if let Some(s) = &mut self.scif7 { if s.handles(a, n) {
            return match s.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SCIF7 read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        for s in [&mut self.scif1, &mut self.scif6].into_iter().flatten() { if s.handles(a, n) {
            let base = s.base;
            return match s.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SCIF@{base:08x} read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        if let Some(s) = &mut self.sdhi { if Sdhi::handles(a, n) {
            return match s.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SDHI read {a:08x}/{n}: {e}")); Some(0) } };
        } }
        if SdhiEmpty::handles(a, n) {
            return match self.sdhi1.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SDHI1 read: {e}")); Some(0) } };
        }
        { {
        } }
        if Src::handles(a, n) {
            return match self.src.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SRC read: {e}")); Some(0) } };
        }
        if let Some(s) = &mut self.ssi { if Ssi::handles(a, n) {
            return match s.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("SSI read: {e}")); Some(0) } };
        } }
        if let Some(j) = &self.jcu { if Jcu::handles(a, n) {
            return match j.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("JCU read: {e}")); Some(0) } };
        } }
        if Dmac::handles(a, n) {
            return match self.dmac.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(format!("DMAC read: {e}")); Some(0) } };
        }
        if (ADC_BASE..ADC_BASE + 0x400).contains(&a) {
            let off = a - ADC_BASE;
            return Some(match (off, n) {
                (0..=0x0E, 2) if off % 2 == 0 => match self.adc[(off / 2) as usize] { Some(v) => v as u64, None => { self.fault.get_or_insert(format!("ADC channel AN{} not modelled", off / 2)); 0 } },
                (0x20, 2) => self.adc_csr,
                _ => { self.fault.get_or_insert(format!("ADC register {a:08x}/{n} not modelled")); 0 }
            });
        }
        if (VDEC_BASE..VDEC_BASE + 0x300).contains(&a) {
            // VSYNCSR with a camera: H and V locked, 60 Hz interlaced, vertical cycle 16.7 ms (0.1 ms units).
            if a == 0xFFFF_A080 && n == 2 && self.camera { return Some(0x4000 | 0x0400 | 0x0200 | 0x0100 | 167); }
            return Some(match self.vdec.get(&a) { Some(&v) if n == 2 => v, _ => { self.fault.get_or_insert(format!("VDEC register {a:08x}/{n} not modelled")); 0 } });
        }
        if wdt::Wdt::handles(a) { return Some(self.wdt.read(a, n)); }
        if rgpvg::Rgpvg::handles(a) {
            return match self.gpu.read(a, n) { Ok(v) => Some(v), Err(e) => { self.fault.get_or_insert(e); Some(0) } };
        }
        let i = self.latches.find(a, n)?;
        Some(self.latches.read(i, a, n))
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> bool {
        if nor::Nor::handles(a) {
            if let Err(e) = self.nor.write(a, n, v) { self.fault.get_or_insert(e); }
            return true;
        }
        if (0xF000_0000..0xF200_0000).contains(&a) { return true; } // cache arrays: invalidate/flush writes have no effect
        if rgpvg::Rgpvg::handles(a) { if let Err(e) = self.gpu.write(a, n, v) { self.fault.get_or_insert(e); } return true; }
        if wdt::Wdt::handles(a) { if let Err(e) = self.wdt.write(a, n, v) { self.fault.get_or_insert(e); } return true; }
        if let Some((ch, o)) = rspi_reg(a) {
            for i in 0..n { let b = (v >> (8 * (n - 1 - i))) as u8; if o + i == 4 { self.rspi_tx[ch].push(b); } if o + i < 0x30 { self.rspi[ch][o + i] = b; } }
            return true;
        }
        if let Some(d) = &mut self.display { if Vdc4::handles(a, n) {
            if let Err(e) = d.vdc.write(a, n, v) { self.fault.get_or_insert(format!("VDC write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        for c in [&mut self.iic2, &mut self.iic1].into_iter().flatten() { if c.handles(a, n) {
            if let Err(e) = c.write(a, n, v) { self.fault.get_or_insert(format!("IIC write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        if let Some(r) = &mut self.rtc { if r.handles(a, n) {
            if let Err(e) = r.write(a, n, v) { self.fault.get_or_insert(format!("RTC write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        if let Some(s) = &mut self.scif7 { if s.handles(a, n) {
            if let Err(e) = s.write(a, n, v) { self.fault.get_or_insert(format!("SCIF7 write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        for s in [&mut self.scif1, &mut self.scif6].into_iter().flatten() { if s.handles(a, n) {
            let base = s.base;
            if let Err(e) = s.write(a, n, v) { self.fault.get_or_insert(format!("SCIF@{base:08x} write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        if let Some(s) = &mut self.sdhi { if Sdhi::handles(a, n) {
            if let Err(e) = s.write(a, n, v) { self.fault.get_or_insert(format!("SDHI write {a:08x}/{n}={v:x}: {e}")); }
            return true;
        } }
        if SdhiEmpty::handles(a, n) {
            if let Err(e) = self.sdhi1.write(a, n, v) { self.fault.get_or_insert(format!("SDHI1 write: {e}")); }
            return true;
        }
        if Src::handles(a, n) {
            if let Err(e) = self.src.write(a, n, v) { self.fault.get_or_insert(format!("SRC write {a:08x}={v:x}: {e}")); }
            if let Some(s) = &mut self.ssi { s.p0_per_frame = 100.0e6 / 3.0 / self.src.out_rate(); }
            return true;
        }
        if let Some(s) = &mut self.ssi { if Ssi::handles(a, n) {
            if let Err(e) = s.write(a, n, v) { self.fault.get_or_insert(format!("SSI write {a:08x}={v:x}: {e}")); }
            return true;
        } }
        if self.jcu.is_some() && Jcu::handles(a, n) {
            // JCU commands read coded data from memory: performed at the end of this step by `tick_with`
            // (the store applies during the step; no other JCU access can intervene).
            self.jcu_writes.push((a, n, v, self.now));
            return true;
        }
        if Dmac::handles(a, n) {
            if let Err(e) = self.dmac.write(a, n, v) { self.fault.get_or_insert(format!("DMAC write: {e}")); }
            return true;
        }
        if (ADC_BASE..ADC_BASE + 0x400).contains(&a) {
            if a == ADC_BASE + 0x20 && n == 2 { self.adc_csr = v; } else { self.fault.get_or_insert(format!("ADC write {a:08x}/{n}={v:x} not modelled")); }
            return true;
        }
        if (VDEC_BASE..VDEC_BASE + 0x300).contains(&a) {
            if n == 2 { self.vdec.insert(a, v); } else { self.fault.get_or_insert(format!("VDEC write {a:08x}/{n} not modelled")); }
            return true;
        }
        let Some(i) = self.latches.find(a, n) else { return false };
        let before = self.latches.write(i, a, n, v);
        if a == CMSTR {
            let now = self.latches.regs[i].value;
            for c in 0..2 {
                if before & (1 << c) != 0 && now & (1 << c) == 0 { self.latches.set(0xFFFE_C004 + c as u32 * 6, 0); self.cmt_remainder[c] = 0; }
            }
        }
        true
    }

    /// CMT0/1: count at P0/{8,32,128,512}; on CMCNT == CMCOR the counter clears and CMF sets .
    fn tick_cmt(&mut self) {
        let r = &mut self.latches.regs;
        if r[self.hot[1]].value & 4 != 0 { return; }
        let str_ = r[self.hot[0]].value;
        for c in 0..2usize {
            if str_ & (1 << c) == 0 { continue; }
            let (csr, cnt, cor) = (self.hot[3 + 3 * c], self.hot[4 + 3 * c], self.hot[5 + 3 * c]);
            let div = [8u64, 32, 128, 512][(r[csr].value & 3) as usize];
            self.cmt_remainder[c] += self.p0_per_step;
            while self.cmt_remainder[c] >= div {
                self.cmt_remainder[c] -= div;
                if r[cnt].value == r[cor].value {
                    r[cnt].value = 0;
                    r[csr].value |= 0x80;
                    self.cmt_matches[c] += 1;
                } else {
                    r[cnt].value = (r[cnt].value + 1) & 0xFFFF;
                }
            }
        }
    }

    /// Is the DMAC servicing this SCIF's TXI (selector routed, channel armed, DMAOR.DME)? The CPU TXI is then masked.
    fn dma_routes(&self, selector: u8) -> bool {
        let dmaor = self.latches.value(DMAOR);
        (0..16).any(|c| self.dmac.selector(c) == selector && self.dmac.ch[c].chcr & dmac::DE != 0 && dmaor & 1 != 0)
    }

    /// One DMAC unit (cycle steal) after the SCIF ticks.
    fn tick_dma(&mut self, mem: &mut Memory) {
        // Cycle-steal throughput: one transfer unit per P0 clock.
        for _ in 0..self.p0_per_step { if !self.tick_dma_unit(mem) { break; } }
    }

    /// One DMA unit; false when no channel has a request.
    fn tick_dma_unit(&mut self, mem: &mut Memory) -> bool {
        if !self.dmac.any_armed() { return false; }
        let dmaor = self.latches.value(DMAOR);
        let (s1, s6) = (self.scif1.as_ref().is_some_and(|s| s.txi_level()), self.scif6.as_ref().is_some_and(|s| s.txi_level()));
        let (ssi_tx, idei, odfi) = (self.ssi.as_ref().is_some_and(|s| s.tx_request()), self.src.idei(), self.src.odfi());
        let sd = self.sdhi.as_ref().is_some_and(|s| s.dma_read_request());
        let sdw = self.sdhi.as_ref().is_some_and(|s| s.dma_write_request());
        if !s1 && !s6 && !ssi_tx && !idei && !odfi && !sd && !sdw { return false; }
        let unit = match self.dmac.next_unit(dmaor, |_, sel| match sel { 0x85 => s1, 0x99 => s6, 0x21 => ssi_tx, 0x41 => idei, 0x42 => odfi, 0x12 => sd, 0x11 => sdw, _ => false }) {
            Ok(Some(u)) => u, Ok(None) => return false, Err(e) => { self.fault.get_or_insert(e); return false; }
        };
        // 16-byte units with SAF/DAF move four longwords through a fixed address (SRC FIFOs).
        let (beats, beat) = if unit.size == 16 { (4u32, 4usize) } else { (1, unit.size) };
        for k in 0..beats {
            let (sa, da) = (if unit.saf { unit.src } else { unit.src + k * beat as u32 }, if unit.daf { unit.dst } else { unit.dst + k * beat as u32 });
            let Some(data) = self.dma_read(mem, sa, beat) else { self.fault.get_or_insert(format!("DMA{} source {sa:08x}/{beat} not modelled", unit.channel)); return false; };
            if !self.dma_write(mem, da, &data[8 - beat..]) { self.fault.get_or_insert(format!("DMA{} destination {da:08x}/{beat} not modelled", unit.channel)); return false; }
        }
        true
    }

    /// DMA source read: plain memory or a DMA-capable peripheral data register; the `n` bytes are the last `n` of the
    /// big-endian array (no allocation: this runs for every transfer unit).
    fn dma_read(&mut self, mem: &mut Memory, a: u32, n: usize) -> Option<[u8; 8]> {
        if let Some(b) = mem.slice(a, n) { let mut out = [0u8; 8]; out[8 - n..].copy_from_slice(b); return Some(out); }
        if Src::handles(a, n) { let v = self.src.read(a, n).ok()?; return Some(v.to_be_bytes()); }
        if let Some(s) = &mut self.sdhi { if Sdhi::handles(a, n) {
            return match s.read(a, n) { Ok(v) => Some(v.to_be_bytes()), Err(e) => { self.fault.get_or_insert(format!("DMA from SDHI: {e}")); None } };
        } }
        None
    }

    /// DMA destination write: plain memory or a DMA-capable peripheral data register.
    fn dma_write(&mut self, mem: &mut Memory, a: u32, d: &[u8]) -> bool {
        if mem.store_bytes(a, d) { return true; }
        let v = d.iter().fold(0u64, |v, &b| (v << 8) | b as u64);
        match a {
            0xE800_780C | 0xE800_A00C if d.len() == 1 => {
                let s = if a == 0xE800_780C { &mut self.scif1 } else { &mut self.scif6 };
                match s { Some(s) => { if let Err(e) = s.dma_write_tdr(d[0]) { self.fault.get_or_insert(format!("DMA to SCFTDR: {e}")); } true } None => false }
            }
            0xFFFF_0018 if d.len() == 4 => match &mut self.ssi { Some(s) => { s.push(v as u32); true } None => false },
            _ if Src::handles(a, d.len()) => self.src.write(a, d.len(), v).is_ok(),
            _ if Sdhi::handles(a, d.len()) => match &mut self.sdhi {
                Some(s) => { if let Err(e) = s.write(a, d.len(), v) { self.fault.get_or_insert(format!("DMA to SDHI: {e}")); } true }
                None => false,
            },
            _ => false,
        }
    }

    /// Apply register writes that need memory access (NOR array changes, JCU commands) right after the instruction
    /// that made them, so a following read sees the new state.
    #[inline(always)]
    pub fn flush_deferred(&mut self, mem: &mut Memory) {
        if self.nor.pending.is_empty() && self.jcu_writes.is_empty() { return; }
        self.flush_deferred_now(mem);
    }

    #[cold]
    #[inline(never)]
    fn flush_deferred_now(&mut self, mem: &mut Memory) {
        if !self.nor.pending.is_empty() {
            mem.code_gen += 1;
            if let Err(e) = self.nor.apply(&mut mem.flash) { self.fault.get_or_insert(e); }
        }
        if !self.jcu_writes.is_empty() {
            let writes = std::mem::take(&mut self.jcu_writes);
            if let Some(j) = &mut self.jcu {
                for (a, n, v, step) in writes { if let Err(e) = j.write(a, n, v, step, &mut JcuMem(mem)) { self.fault.get_or_insert(format!("JCU write {a:08x}={v:x}: {e}")); } }
            }
        }
    }

    /// Highest-priority pending request, recomputed only when peripheral state may have changed.
    #[inline]
    pub fn pending_request(&mut self) -> Option<Request> {
        if self.irq_dirty { crate::prof::set(crate::prof::IRQ); self.cached_request = self.best_request(); self.irq_dirty = false; }
        self.cached_request
    }

    pub fn tick_with(&mut self, mem: &mut Memory) {
        self.irq_dirty = true;

        self.flush_deferred(mem);
        self.tick();
        let now = self.now;
        if let Some(j) = &mut self.jcu { if let Err(e) = j.tick(now, &mut JcuMem(mem)) { self.fault.get_or_insert(format!("JCU decode: {e}")); } }
        if let Some(s) = &mut self.scif1 {
            s.advance_p0(self.p0_per_step);
            if let Some(d) = &mut self.dvr { d.advance(now, s); }
        }
        if let Some(s) = &mut self.scif6 {
            s.advance_p0(self.p0_per_step);
            if let Some(o) = &mut self.obd2 {
                if o.car.follow_gps { if let Some(g) = self.companion.as_ref().and_then(|c| c.gnss.as_ref()) { o.car.speed_kmh = g.last_speed; } }
                o.advance(now, s);
            }
        }
        if self.camera && now >= self.cam.next_frame {
            if let Some(g) = self.companion.as_ref().and_then(|c| c.gnss.as_ref()) { self.cam.speed_kmh = g.last_speed; }
            self.camera_frame(mem);
        } else if !self.camera && self.vdec_free_run && now >= self.cam.next_frame {
            self.cam.next_frame = now + crate::camera::FIELD_P0;
            if let Some(d) = &mut self.display { if d.vdc.module_enabled && d.vdec_enabled() { d.queue_input_vsync(); } }
        }
        if self.sdhi1.card.is_some() { self.sdhi1.advance(now); }
        crate::prof::set(crate::prof::DMA);
        self.tick_dma(mem);
        crate::prof::set(crate::prof::BOARD);
    }

    /// One NTSC field from the camera: an input Vsync for the VDC (VI_VSYNC, SCL1 register latching, output sync when
    /// SCL0_FRC3 selects the input) and, every second field, with
    /// frame-buffer writing enabled (SCL1_WR5.RES_WENB), the picture stored by the scaler-0 write-back.
    fn camera_frame(&mut self, mem: &mut Memory) {
        let now = self.now;
        self.cam.next_frame = now + crate::camera::FIELD_P0;
        let Some(d) = &mut self.display else { return };
        if !d.vdc.module_enabled { return; }
        d.queue_input_vsync();
        self.cam.fields += 1;
        if self.cam.fields % 2 == 1 { return; } // the picture is written once per frame (two fields)
        let v = &mut d.vdc;
        let (wr1, wr2, wr3, wr4, wr5, ds7) = (v.active(0xFFFF_7588), v.active(0xFFFF_758C), v.active(0xFFFF_7590), v.active(0xFFFF_7594), v.active(0xFFFF_759C), v.active(0xFFFF_7544));
        if wr5 & 1 == 0 { self.cam.writes = 0; return; }
        self.cam.writes += 1;
        if self.cam.writes < 2 { return; } // writing starts from the second frame
        let frames = (wr3 & 0x3FF) as u32 + 1;
        let next = (v.wr_frame + 1) % frames;
        let target = crate::camera::Target {
            base: wr2 as u32 + (wr4 & 0x7F_FFFF) as u32 * next, stride: ((wr3 >> 16) & 0x7FFF) as u32,
            width: (ds7 & 0x7FF) as u32, height: ((ds7 >> 16) & 0x7FF) as u32,
            format: ((wr1 >> 2) & 3) as u8, rotation: ((wr1 >> 4) & 7) as u8,
        };
        v.wr_frame = next;
        if let Err(e) = self.cam.render(now, &target, mem) { self.fault.get_or_insert(format!("camera frame: {e}")); }
    }

    /// MTU1 : internal P0/64 count, TGRB compare, overflow, and TGRA capture on
    /// IR falling edges (TIOR1 = 0x09: capture A on falling edge).
    fn tick_mtu1(&mut self) {
        let i = self.mtu1_i;
        let r = &mut self.latches.regs;
        let edge = self.ir.edge_at(self.now);
        let running = r[i[0]].value & 2 != 0 && r[i[1]].value & 0x20 == 0;
        if !running {
            if edge { self.fault.get_or_insert("IR edge while MTU1 stopped or gated".into()); }
            return;
        }
        if r[i[2]].value != 0x0B { if edge { self.fault.get_or_insert(format!("MTU1 TCR {:x} not modelled", r[i[2]].value)); } return; }
        self.mtu1_remainder += self.p0_per_step;
        while self.mtu1_remainder >= 64 {
            self.mtu1_remainder -= 64;
            let cnt = r[i[6]].value;
            if cnt == r[i[8]].value { r[i[5]].value |= 2; r[i[5]].status_read &= !2; }
            if cnt == 0xFFFF { r[i[6]].value = 0; r[i[5]].value |= 0x10; r[i[5]].status_read &= !0x10; } else { r[i[6]].value = cnt + 1; }
        }
        if edge {
            if r[i[3]].value & 0x0F != 0x09 { self.fault.get_or_insert("IR edge without TIOR1 capture-A falling".into()); return; }
            r[i[7]].value = r[i[6]].value;
            r[i[5]].value |= 1; r[i[5]].status_read &= !1;
        }
    }

    /// MTU2 channels 3 and 4 : normal mode, internal P0 prescaler, TGRA..D compare flags, counter
    /// clear on compare (CCLR), overflow flag. Interrupts, input capture, PWM/complementary modes and external clocks are
    /// not modelled and stop the run when configured.
    fn tick_mtu34(&mut self) {
        let starts = self.latches.value(MTU_TSTR);
        if starts & 0xC0 == 0 || self.latches.value(STBCR3) & 0x20 != 0 { return; }
        for ch in 3u32..=4 {
            if starts & (1 << (ch + 3)) == 0 { continue; }
            let o = ch - 3;
            let (tcr, tmdr) = (self.latches.value(0xFFFE_4200 + o), self.latches.value(0xFFFE_4202 + o));
            let (src, edge, clr) = ((tcr & 7) as usize, (tcr >> 3) & 3, (tcr >> 5) as u32);
            if tmdr != 0 { self.fault.get_or_insert(format!("MTU{ch} TMDR {tmdr:x}: only normal mode is modelled")); return; }
            if src > 5 { self.fault.get_or_insert(format!("MTU{ch} external clock input")); return; }
            if clr == 3 || clr == 7 { self.fault.get_or_insert(format!("MTU{ch} synchronous clear")); return; }
            let mut div = [1u64, 4, 16, 64, 256, 1024][src];
            if edge >= 2 && div > 1 { div /= 2; }
            let (cnt_a, tsr_a) = if ch == 3 { (0xFFFE_4210, 0xFFFE_422C) } else { (0xFFFE_4212, 0xFFFE_422D) };
            let tg: [u32; 4] = if ch == 3 { [0xFFFE_4218, 0xFFFE_421A, 0xFFFE_4224, 0xFFFE_4226] } else { [0xFFFE_421C, 0xFFFE_421E, 0xFFFE_4228, 0xFFFE_422A] };
            let tgr = tg.map(|a| self.latches.value(a));
            for k in 0..4u32 {
                let io = self.latches.value(0xFFFE_4204 + o * 2 + k / 2);
                if (io >> ((k % 2) * 4)) & 15 >= 8 { self.fault.get_or_insert(format!("MTU{ch} input capture")); return; }
            }
            self.mtu34_remainder[o as usize] += self.p0_per_step;
            let (mut cnt, mut tsr) = (self.latches.value(cnt_a), self.latches.value(tsr_a));
            while self.mtu34_remainder[o as usize] >= div {
                self.mtu34_remainder[o as usize] -= div;
                let mut clear = false;
                for k in 0..4 { if cnt == tgr[k] { tsr |= 1 << k; if clr == [1, 2, 5, 6][k] { clear = true; } } }
                if clear { cnt = 0; } else { cnt = (cnt + 1) & 0xFFFF; if cnt == 0 { tsr |= 0x10; } }
            }
            self.latches.set(cnt_a, cnt);
            self.latches.set(tsr_a, tsr);
            if tsr & self.latches.value(0xFFFE_4208 + o) & 0x1F != 0 { self.fault.get_or_insert(format!("MTU{ch} interrupt enabled (not modelled)")); }
        }
    }

    pub fn tick(&mut self) {
        crate::prof::set(crate::prof::BOARD);
        self.now += self.p0_per_step;
        crate::prof::set(crate::prof::TIMERS);
        self.tick_mtu1();
        self.tick_mtu34();
        self.wdt.tick(self.p0_per_step);
        crate::prof::set(crate::prof::AUDIO);
        if self.src.enabled() {
            if self.src.quiescent() { self.src.tick_quiescent(); } else { for _ in 0..self.p0_per_step { self.src.tick(); if self.src.quiescent() { self.src.tick_quiescent(); break; } } }
        }
        if let Some(s) = &mut self.ssi { s.tick(self.p0_per_step); }
        crate::prof::set(crate::prof::TIMERS);
        self.tick_cmt();
        if let Some(r) = &mut self.rtc { if let Err(e) = r.tick_p0(self.p0_per_step) { self.fault.get_or_insert(format!("RTC clock: {e}")); } }
        crate::prof::set(crate::prof::DISPLAY);
        self.tick_display();
        crate::prof::set(crate::prof::SERIAL);
        if let Some(c) = &mut self.iic2 { if let Err(e) = c.advance_p0(self.p0_per_step) { self.fault.get_or_insert(format!("IIC2 clock: {e}")); } }
        if let Some(c) = &mut self.iic1 { if let Err(e) = c.advance_p0(self.p0_per_step) { self.fault.get_or_insert(format!("IIC1 clock: {e}")); } }
        if let Some(c) = &mut self.iic2 { c.update_motion(self.now, self.companion.as_mut().and_then(|c| c.gnss.as_mut())); }
        if let Some(c) = &mut self.companion { c.advance(self.p0_per_step, self.scif7.as_mut()); }
        crate::prof::set(crate::prof::SERIAL);
        if let Some(s) = &mut self.scif7 {
            s.advance_p0(self.p0_per_step);
            if !self.peer.queue.is_empty() { for f in self.peer.poll(self.now, &s.tx_log) { s.queue_rx(&f); } }
        }
    }

    /// Display tick: VDC module gate STBCR7.MSTP73 (bit3), VDEC gate STBCR10 bit7, then one P0 tick.
    fn tick_display(&mut self) {
        let (stbcr7, stbcr10) = (self.latches.regs[self.hot[1]].value, self.latches.regs[self.i_stbcr10].value);
        let p0 = self.p0_per_step;
        if let Some(d) = &mut self.display {
            d.vdc.module_enabled = stbcr7 & 8 == 0;
            d.set_vdec_enabled(stbcr10 & 0x80 == 0);
            if let Err(e) = d.advance_p0(p0) { self.fault.get_or_insert(format!("display clock: {e}")); }
        }
    }

    /// Highest-priority request from Rust-modelled sources (ordering: higher level wins,
    /// equal level -> lower vector).
    pub fn best_request(&self) -> Option<Request> {
        let mut best: Option<Request> = None;
        let mut offer = |v: u32, l: u32| {
            if l == 0 { return; }
            match best { Some((bv, bl)) if l < bl || (l == bl && v >= bv) => {}, _ => best = Some((v, l)) }
        };
        let r = &self.latches.regs;
        if let Some(d) = &self.display {
            let p = d.vdc.pending();
            if p != 0 {
                let ipr10 = r[self.i_ipr10].value;
                for idx in 0..9u32 { if p & (1 << idx) != 0 { offer(171 + idx, ((ipr10 >> if idx < 3 { 8 } else if idx < 5 { 4 } else { 0 }) & 15) as u32); } }
            }
        }
        if let Some(s) = &self.scif7 {
            let l = (r[self.i_ipr21].value & 15) as u32;
            for v in s.requests().into_iter().flatten() { offer(v, l); }
        }
        if let Some(s) = &self.scif6 {
            let req = s.requests();
            if req.iter().any(|x| x.is_some()) {
                let l = ((r[self.i_ipr21].value >> 4) & 15) as u32;
                let masked = self.dma_routes(0x99);
                for (k, v) in req.into_iter().enumerate() { if let Some(v) = v { if !(k == 3 && masked) { offer(v, l); } } }
            }
        }
        if let Some(s) = &self.scif1 {
            let req = s.requests();
            if req.iter().any(|x| x.is_some()) {
                let l = ((self.latches.value(IPR20) >> 8) & 15) as u32;
                let masked = self.dma_routes(0x85);
                for (k, v) in req.into_iter().enumerate() { if let Some(v) = v { if !(k == 3 && masked) { offer(v, l); } } }
            }
        }
        {
            let l = ((self.latches.value(0xFFFE_0C28) >> 12) & 15) as u32;
            if l > 0 { for (v, p) in self.src.irqs() { if p { offer(v, l); } } }
            if let Some(s) = &self.ssi { if s.irq() { offer(224, ((self.latches.value(0xFFFE_0C16) >> 8) & 15) as u32); } }
        }
        if self.sdhi.as_ref().is_some_and(|s| s.irq_pending()) { offer(332, ((self.latches.value(0xFFFE_0C26) >> 12) & 15) as u32); }
        if self.sdhi1.irq_pending() { offer(335, ((self.latches.value(0xFFFE_0C26) >> 8) & 15) as u32); }
        if self.gpu.irq() { offer(rgpvg::VECTOR, (self.latches.value(0xFFFE_0C0A) & 15) as u32); }
        if let Some(j) = &self.jcu {
            if j.jedi_level() || j.jdti_level() { offer(if j.jedi_level() { 181 } else { 182 }, ((self.latches.value(0xFFFE_0C0A) >> 8) & 15) as u32); }
        }
        if self.dmac.any_irq() {
            for (v, c) in self.dmac.requests() {
                let (ipr, shift) = Dmac::ipr(c);
                offer(v, ((self.latches.value(ipr) >> shift) & 15) as u32);
            }
        }
        {
            let i = self.mtu1_i;
            let pend = r[i[4]].value & r[i[5]].value & 0x11;
            if pend != 0 {
                let ipr13 = r[i[9]].value;
                if pend & 1 != 0 { offer(199, ((ipr13 >> 4) & 15) as u32); }
                if pend & 0x10 != 0 { offer(201, (ipr13 & 15) as u32); }
            }
        }
        let ipr = r[self.hot[2]].value;
        for c in 0..2u32 {
            let csr = r[self.hot[3 + 3 * c as usize]].value;
            if csr & 0xC0 == 0xC0 { offer(188 + c, ((ipr >> if c == 0 { 12 } else { 8 }) & 15) as u32); }
        }
        best
    }
}
