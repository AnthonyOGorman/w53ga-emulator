//! RL78/G13 (R5F100LLA) on-chip peripherals: clock generator, interrupt controller (IF/MK/PR), and the
//! peripherals the controller firmware uses, added as the firmware reaches them.
//!
//! Interrupt request flag n (IF0L.0 = bit 0 ... IF3L) has its vector at 0x0004 + 2n; flag groups:
//! IF0 FFFE0/1, IF1 FFFE2/3, IF2 FFFD0/1, IF3 FFFD2 (MK at +4 / FFFD4.., PR0 at FFFE8.. / FFFD8.., PR1 at FFFEC.. / FFFDC..).

use super::cpu::{Rl78, Rl78Bus, Rl78Fault};
use std::collections::BTreeMap;

pub const IFN: usize = 56;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct G13 {
    #[serde(with = "serde_big_array::BigArray")]
    pub sfr: [u8; 0x100],
    pub sfr2: Vec<u8>,
    pub log: BTreeMap<u32, (u64, u64, u8)>,
    pub logging: bool,
    pub cycles: u64,
    /// Debug: log accesses inside this range with the instruction address.
    pub watch: Option<(u32, u32)>,
    pub cur_pc: u32,
    /// External pin levels of ports P0..P15 (used for bits in input mode, PMn = 1).
    pub pins: [u8; 16],
    pub tau: [Tau; 2],
    pub sau: [Sau; 2],
    /// Serial NOR flash on CSI20 (SAU1 channel 0), chip select P1.6.
    pub spi: super::spiflash::SpiFlash,
    /// RAM window F0800-F0FFF used by the flash self-programming library (state at F0880.., code copied to F0A80/F0B80).
    pub fsl_ram: Vec<u8>,
    /// Boot-swap flag reported by the flash firmware (0 = image in the first area, bank 0).
    pub boot_swap: u8,
    pub fsl_log: bool,
    /// Internal reset requested (illegal WDTE write), taken by the companion.
    #[serde(skip)]
    pub reset_request: bool,
    /// Cached highest pending request; recomputed only after IF/MK/PR changes.
    #[serde(skip, default = "yes")]
    irq_dirty: bool,
    #[serde(skip)]
    cached: Option<(usize, u8)>,
    /// A/D input voltages by ADS channel code (0x00-0x1F ANIx, 0x80 temperature sensor, 0x81 internal reference),
    /// as fractions of AVREF (= VDD). Unlisted channels read mid-scale; each new channel is reported once.
    pub adc_inputs: std::collections::HashMap<u8, f64>,
    adc_seen: std::collections::HashSet<u8>,
    /// Accepted interrupts per flag index.
    #[serde(with = "serde_big_array::BigArray")]
    pub irq_count: [u64; IFN],
    /// Earliest cycle at which any timer fires (fast path for `tick`).
    next_event: u64,
    /// SFR accesses by the CPU and interrupts accepted, for the busy-wait fast-forward (`Companion::spin_record`).
    #[serde(skip)]
    pub bus_accesses: u64,
    #[serde(skip)]
    pub accepted: u64,
}

/// Timer array unit: 8 channels; only the free-running interval/one-count behaviour is modelled.
#[derive(Default, Clone, serde::Serialize, serde::Deserialize)]
pub struct Tau { pub due: [Option<u64>; 8], pub period: [u64; 8], pub one_shot: [bool; 8] }

const TAU_BASE: [u32; 2] = [0xF0180, 0xF01C0];
const TDR: [[u32; 8]; 2] = [[0xFFF18, 0xFFF1A, 0xFFF64, 0xFFF66, 0xFFF68, 0xFFF6A, 0xFFF6C, 0xFFF6E], [0xFFF70, 0xFFF72, 0xFFF74, 0xFFF76, 0xFFF78, 0xFFF7A, 0xFFF7C, 0xFFF7E]];
/// Serial array unit: per channel transfer completion time, received-data queue and transmit log.
#[derive(Default, Clone, serde::Serialize, serde::Deserialize)]
pub struct Sau { pub due: [Option<u64>; 4], pub rx: [std::collections::VecDeque<u8>; 4], pub tx: [Vec<u8>; 4], pub rx_due: [Option<u64>; 4] }

const SAU_BASE: [u32; 2] = [0xF0100, 0xF0140];
const SDR: [[u32; 4]; 2] = [[0xFFF10, 0xFFF12, 0xFFF44, 0xFFF46], [0xFFF48, 0xFFF4A, 0xFFF14, 0xFFF16]];
/// INTSTx/INTCSIxx/INTSRx flag index per channel, and the UART error flag per channel pair.
const SAU_IRQ: [[usize; 4]; 2] = [[13, 14, 16, 17], [8, 9, 28, 29]];
const SAU_ERR: [[usize; 2]; 2] = [[15, 18], [10, 44]];

fn sdr_chan(a: u32) -> Option<(usize, usize)> { for u in 0..2 { for ch in 0..4 { if SDR[u][ch] == a { return Some((u, ch)); } } } None }

/// Interrupt flag index of INTTMmn.
const TAU_IRQ: [[usize; 8]; 2] = [[20, 21, 22, 23, 31, 32, 33, 34], [43, 44, 45, 30, 52, 53, 54, 55]];

impl Default for G13 {
    fn default() -> Self {
        let mut g = G13 { reset_request: false, sfr: [0; 0x100], sfr2: vec![0; 0x800], log: BTreeMap::new(), logging: true, cycles: 0, watch: None, cur_pc: 0, pins: [0xFF; 16], tau: Default::default(), sau: Default::default(), spi: super::spiflash::SpiFlash::provisioned(), fsl_ram: vec![0; 0x800], boot_swap: 0, fsl_log: std::env::var_os("AR_FSL_LOG").is_some(),
            irq_dirty: true, cached: None, adc_inputs: [(0x80u8, 1.05 / 3.3), (0x81u8, 1.45 / 3.3)].into_iter().collect(), adc_seen: Default::default(), irq_count: [0; IFN], next_event: u64::MAX,
            bus_accesses: 0, accepted: 0 };
        // reset values: MK all ones, PR all ones
        for a in [0xE4, 0xE5, 0xE6, 0xE7, 0xD4, 0xD5, 0xD6, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xD8, 0xD9, 0xDA, 0xDC, 0xDD, 0xDE] { g.sfr[a] = 0xFF; }
        for a in 0x20..0x30 { g.sfr[a] = 0xFF; } // PMx
        g.sfr[0xA1] = 0xC0; // CSC: X1/XT1 stopped
        g
    }
}

fn yes() -> bool { true }

/// (IF, MK, PR0, PR1) SFR offsets of flag group g (8 flags each).
fn group(g: usize) -> (usize, usize, usize, usize) {
    match g { 0 => (0xE0, 0xE4, 0xE8, 0xEC), 1 => (0xE1, 0xE5, 0xE9, 0xED), 2 => (0xE2, 0xE6, 0xEA, 0xEE), 3 => (0xE3, 0xE7, 0xEB, 0xEF),
        4 => (0xD0, 0xD4, 0xD8, 0xDC), 5 => (0xD1, 0xD5, 0xD9, 0xDD), _ => (0xD2, 0xD6, 0xDA, 0xDE) }
}

impl G13 {
    pub fn request(&mut self, n: usize) { let (f, ..) = group(n / 8); self.sfr[f] |= 1 << (n % 8); self.irq_dirty = true; }

    /// Highest-priority pending unmasked request: (flag index, level).
    pub fn pending(&self) -> Option<(usize, u8)> {
        let mut best: Option<(usize, u8)> = None;
        for g in 0..7 {
            let (f, m, p0, p1) = group(g);
            let req = self.sfr[f] & !self.sfr[m];
            if req == 0 { continue; }
            for b in 0..8 {
                if req & (1 << b) == 0 { continue; }
                let lvl = ((self.sfr[p1] >> b) & 1) << 1 | ((self.sfr[p0] >> b) & 1);
                if best.map_or(true, |(_, l)| lvl < l) { best = Some((g * 8 + b, lvl)); }
            }
        }
        best
    }

    /// Accept an interrupt if allowed (IE set and level higher priority than ISP).
    #[inline(always)]
    pub fn service(&mut self, cpu: &mut Rl78) -> Result<bool, Rl78Fault> {
        if self.irq_dirty { self.cached = self.pending(); self.irq_dirty = false; }
        if let Some((n, lvl)) = self.cached {
            cpu.halted = false; // any unmasked request releases HALT
            if cpu.ie() && lvl <= cpu.isp() {
                let (f, ..) = group(n / 8);
                self.sfr[f] &= !(1 << (n % 8));
                self.irq_dirty = true;
                // ISP afterwards enables only higher levels (level 0 stays nestable).
                self.irq_count[n] += 1;
                self.accepted += 1;
                cpu.interrupt(self, 4 + 2 * n as u32, lvl.saturating_sub(1))?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Would `service` accept an interrupt before the next instruction (the same check, without accepting it)?
    #[inline(always)]
    pub fn interrupt_due(&mut self, cpu: &Rl78) -> bool {
        if self.irq_dirty { self.cached = self.pending(); self.irq_dirty = false; }
        matches!(self.cached, Some((_, lvl)) if cpu.ie() && lvl <= cpu.isp())
    }

    /// fCLK cycles until the next timer or serial event (`tick` processes events once `cycles` reaches it).
    pub fn cycles_to_event(&self) -> u64 { self.next_event.saturating_sub(self.cycles) }

    /// Advance time by `n` cycles that end before the next event (`cycles_to_event`): what `tick` does then.
    pub fn add_cycles(&mut self, n: u64) { debug_assert!(self.cycles + n < self.next_event); self.cycles += n; }

    /// One CPU step with interrupt acceptance and high-level emulation of the boot-ROM flash entry (CALL !!EFFF8).
    /// The instruction's clock count is left in `cpu.cycles` (interrupt acceptance adds 9 clocks).
    #[inline(always)]
    pub fn step(&mut self, cpu: &mut Rl78) -> Result<(), Rl78Fault> {
        if self.service(cpu)? { cpu.steps += 1; cpu.cycles = 9; return Ok(()); }
        match cpu.step(self) {
            Err(Rl78Fault::Fetch { pc: 0xEFFF8 }) => { self.flash_firmware(cpu); cpu.ret(self); Ok(()) }
            r => r,
        }
    }

    /// Flash firmware command in C; parameters in the F0800 window. Returns status in C (0 = OK).
    fn flash_firmware(&mut self, cpu: &mut Rl78) {
        let c = cpu.r(2);
        let hl = cpu.rp(3) as usize;
        let at = |s: &Self, a: usize| s.fsl_ram[(a - 0x800) & 0x7FF];
        let status = match c {
            0x00 => 0, // initialise (frequency checked by the library)
            0x10 | 0x14 => {
                // information reads into the buffer at HL (F0880): 0x14 = boot-swap / boot-flag state
                let v = if c == 0x14 { self.boot_swap } else { self.boot_swap };
                if (0x800..0x1000).contains(&hl) { self.fsl_ram[hl - 0x800] = v; }
                0
            }
            0x03 | 0x04 | 0x06 | 0x08 => {
                let addr = at(self, 0x800) as usize | (at(self, 0x801) as usize) << 8 | (at(self, 0x802) as usize) << 16;
                let src = 0xF0000 | (at(self, 0x804) as u32 | (at(self, 0x805) as u32) << 8);
                let len = (at(self, 0x806) as usize | (at(self, 0x807) as usize) << 8).max(1);
                let end = (addr + len).min(cpu.flash.len());
                match c {
                    0x08 => if addr < end && cpu.flash[addr..end].iter().all(|&b| b == 0xFF) { 0 } else { 0x1B },
                    0x03 => { for b in &mut cpu.flash[addr.min(end)..end] { *b = 0xFF; } 0 }
                    0x04 => { for i in 0..end.saturating_sub(addr) { let v = cpu.read8(self, src + i as u32); cpu.flash[addr + i] &= v; } 0 }
                    _ => 0, // internal verify
                }
            }
            _ => { eprintln!("[rl78 fsl] unhandled flash command {c:02x} HL={hl:04x} F0800: {:02x?}", (0..8).map(|i| at(self, 0x800 + i)).collect::<Vec<_>>()); 0 }
        };
        if self.fsl_log {
            let sp = cpu.sp() as u32;
            let caller = (0..3).fold(0u32, |v, i| v | (cpu.read8(self, 0xF0000 | (sp + i)) as u32) << (8 * i)) & 0xFFFFF;
            eprintln!("[rl78 fsl] cmd {c:02x} HL={hl:04x} F0800 {:02x?} -> {status:02x} (from {caller:05x})", (0..8).map(|i| at(self, 0x800 + i)).collect::<Vec<_>>());
        }
        cpu.set_r(2, status);
    }

    /// Multiplier/divider (MDUC F00E8): 32/32 unsigned division MDAH:MDAL / MDBH:MDBL -> quotient MDA, remainder
    /// MDCH:MDCL, completing immediately (INTMD when MACMODE = 0).
    fn mdu_divide(&mut self) {
        let a = (self.r16(0xFFFF2) as u32) << 16 | self.r16(0xFFFF0) as u32;
        let b = (self.r16(0xFFFF4) as u32) << 16 | self.r16(0xFFFF6) as u32;
        let (q, r) = if b == 0 { (0xFFFF_FFFF, a) } else { (a / b, a % b) };
        self.w16(0xFFFF0, q as u16); self.w16(0xFFFF2, (q >> 16) as u16);
        self.w16(0xF00E0, r as u16); self.w16(0xF00E2, (r >> 16) as u16);
        self.sfr2[0xE8] &= !1;
        if self.sfr2[0xE8] & 0x40 == 0 { self.request(45); }
    }
    /// Multiplication / multiply-accumulate on a write of MDAH or MDAL: MDAH x MDAL -> MDBH:MDBL (+ MDCH:MDCL).
    fn mdu_multiply(&mut self) {
        let mduc = self.sfr2[0xE8];
        let signed = mduc & 0x08 != 0;
        let (h, l) = (self.r16(0xFFFF2), self.r16(0xFFFF0));
        let p: u32 = if signed { (h as i16 as i32 * l as i16 as i32) as u32 } else { h as u32 * l as u32 };
        self.w16(0xFFFF6, p as u16); self.w16(0xFFFF4, (p >> 16) as u16);
        if mduc & 0x40 != 0 {
            let c = (self.r16(0xF00E2) as u32) << 16 | self.r16(0xF00E0) as u32;
            let (sum, of) = if signed { let (x, o) = (c as i32).overflowing_add(p as i32); (x as u32, o) } else { c.overflowing_add(p) };
            self.w16(0xF00E0, sum as u16); self.w16(0xF00E2, (sum >> 16) as u16);
            let mut m = mduc & !0x06;
            if of { m |= 0x04; }
            if signed && (sum as i32) < 0 { m |= 0x02; }
            self.sfr2[0xE8] = m;
        }
    }

    fn r16(&self, a: u32) -> u16 { let o = |x: u32| if x >= 0xFFF00 { self.sfr[(x - 0xFFF00) as usize] } else { self.sfr2[(x - 0xF0000) as usize] }; o(a) as u16 | (o(a + 1) as u16) << 8 }
    fn w16(&mut self, a: u32, v: u16) { for (i, b) in [v as u8, (v >> 8) as u8].into_iter().enumerate() { if let Some(c) = self.cell(a + i as u32) { *c = b; } } }

    /// Timer clock divider of unit u channel ch (fCLK cycles per count).
    fn tau_div(&self, u: usize, ch: usize) -> u64 {
        let tps = self.r16(TAU_BASE[u] + 0x36);
        let tmr = self.r16(TAU_BASE[u] + 0x10 + 2 * ch as u32);
        match tmr >> 14 { 0 => 1 << (tps & 15), 2 => 1 << ((tps >> 4) & 15), 1 => 1 << (1 + 2 * ((tps >> 8) & 3)), _ => 1 << (8 + 2 * ((tps >> 12) & 3)) }
    }

    fn tau_start(&mut self, u: usize, mask: u8) {
        let te = TAU_BASE[u] + 0x30;
        for ch in 0..8 {
            if mask & (1 << ch) == 0 { continue; }
            let tmr = self.r16(TAU_BASE[u] + 0x10 + 2 * ch as u32);
            let md = (tmr >> 1) & 7;
            let period = (self.r16(TDR[u][ch]) as u64 + 1) * self.tau_div(u, ch);
            self.tau[u].period[ch] = period;
            self.tau[u].one_shot[ch] = md == 4 || md == 6;
            self.tau[u].due[ch] = if md == 0 || md == 4 { Some(self.cycles + period) } else { None };
            if md == 0 && tmr & 1 != 0 { self.request(TAU_IRQ[u][ch]); }
            let v = self.r16(te) | (1 << ch); self.w16(te, v);
        }
        self.recompute_next();
    }
    fn tau_stop(&mut self, u: usize, mask: u8) {
        let te = TAU_BASE[u] + 0x30;
        for ch in 0..8 { if mask & (1 << ch) != 0 { self.tau[u].due[ch] = None; } }
        let v = self.r16(te) & !(mask as u16); self.w16(te, v);
        self.recompute_next();
    }
    fn recompute_next(&mut self) {
        let t = self.tau.iter().flat_map(|t| t.due.iter().flatten());
        let s = self.sau.iter().flat_map(|s| s.due.iter().chain(s.rx_due.iter()).flatten());
        self.next_event = t.chain(s).copied().min().unwrap_or(u64::MAX);
    }

    /// fCLK cycles per transferred character of SAU u channel ch (10 bits for UART, 8 for CSI).
    fn sau_char_cycles(&self, u: usize, ch: usize) -> u64 {
        let sps = self.r16(SAU_BASE[u] + 0x26);
        let smr = self.r16(SAU_BASE[u] + 0x10 + 2 * ch as u32);
        let pres = if smr & 0x8000 != 0 { 1u64 << ((sps >> 4) & 15) } else { 1u64 << (sps & 15) };
        let div = ((self.r16(SDR[u][ch]) >> 9) as u64 + 1) * 2;
        let bits = if (smr >> 1) & 3 == 1 { 10 } else { 8 };
        pres * div * bits
    }
    fn sau_mode(&self, u: usize, ch: usize) -> u16 { (self.r16(SAU_BASE[u] + 0x10 + 2 * ch as u32) >> 1) & 3 }
    fn sau_enabled(&self, u: usize, ch: usize) -> bool { self.r16(SAU_BASE[u] + 0x20) & (1 << ch) != 0 }

    /// Byte written to SDRmn's data part.
    fn sau_write_data(&mut self, u: usize, ch: usize, v: u8) {
        if !self.sau_enabled(u, ch) { return; }
        self.sau[u].tx[ch].push(v);
        let ssr = SAU_BASE[u] + 2 * ch as u32;
        let st = self.r16(ssr) | 0x40; self.w16(ssr, st); // TSF
        let t = self.cycles + self.sau_char_cycles(u, ch);
        self.sau[u].due[ch] = Some(t);
        if self.sau_mode(u, ch) == 0 { // CSI: full duplex, the received byte arrives with the transfer
            let r = if (u, ch) == (1, 0) { self.spi.xfer(v) } else { 0xFF };
            self.sau[u].rx[ch].push_back(r);
        }
        self.recompute_next();
    }

    /// Queue bytes for reception on UART k (0..3).
    pub fn uart_receive(&mut self, k: usize, data: &[u8]) {
        let (u, ch) = (k / 2, (k % 2) * 2 + 1);
        self.sau[u].rx[ch].extend(data);
        if self.sau[u].rx_due[ch].is_none() { self.sau[u].rx_due[ch] = Some(self.cycles + self.sau_char_cycles(u, ch)); self.recompute_next(); }
    }
    /// Bytes transmitted on UART k so far (drained).
    pub fn uart_take_tx(&mut self, k: usize) -> Vec<u8> { std::mem::take(&mut self.sau[k / 2].tx[(k % 2) * 2]) }

    fn sau_events(&mut self) {
        for u in 0..2 {
            for ch in 0..4 {
                if let Some(d) = self.sau[u].due[ch] {
                    if d <= self.cycles {
                        self.sau[u].due[ch] = None;
                        let ssr = SAU_BASE[u] + 2 * ch as u32;
                        let st = self.r16(ssr) & !0x40; self.w16(ssr, st);
                        if self.sau_mode(u, ch) == 0 {
                            let b = self.sau[u].rx[ch].pop_front().unwrap_or(0xFF);
                            let a = SDR[u][ch]; if let Some(c) = self.cell(a) { *c = b; }
                        }
                        self.request(SAU_IRQ[u][ch]);
                    }
                }
                if let Some(d) = self.sau[u].rx_due[ch] {
                    if d <= self.cycles {
                        self.sau[u].rx_due[ch] = None;
                        if let Some(b) = self.sau[u].rx[ch].pop_front() {
                            if self.sau_enabled(u, ch) {
                                let ssr = SAU_BASE[u] + 2 * ch as u32;
                                let mut st = self.r16(ssr);
                                if st & 0x20 != 0 { st |= 1; self.request(SAU_ERR[u][ch / 2]); } // overrun
                                st |= 0x20; self.w16(ssr, st);
                                let a = SDR[u][ch]; if let Some(c) = self.cell(a) { *c = b; }
                                self.request(SAU_IRQ[u][ch]);
                            }
                        }
                        if !self.sau[u].rx[ch].is_empty() { self.sau[u].rx_due[ch] = Some(self.cycles + self.sau_char_cycles(u, ch)); }
                    }
                }
            }
        }
    }

    /// HALT fast-forward: fCLK cycles until the next timer/serial event (u64::MAX: none) if no request is pending, so
    /// nothing can release a HALT before then (requests only come from those events or from outside the CPU loop).
    pub fn quiet_cycles(&mut self) -> Option<u64> {
        if self.irq_dirty { self.cached = self.pending(); self.irq_dirty = false; }
        if self.cached.is_some() { return None; }
        Some(self.next_event.saturating_sub(self.cycles))
    }

    /// Advance peripheral time by `n` fCLK cycles.
    #[inline(always)]
    pub fn tick(&mut self, n: u64) {
        self.cycles += n;
        if self.cycles < self.next_event { return; }
        self.tick_events();
    }

    /// Timer and serial events now due (`tick`).
    #[inline(never)]
    fn tick_events(&mut self) {
        for u in 0..2 {
            for ch in 0..8 {
                while let Some(d) = self.tau[u].due[ch] {
                    if d > self.cycles { break; }
                    self.request(TAU_IRQ[u][ch]);
                    // a one-count channel waits for the next start trigger
                    self.tau[u].due[ch] = if self.tau[u].one_shot[ch] { None } else { Some(d + self.tau[u].period[ch]) };
                }
            }
        }
        self.sau_events();
        self.recompute_next();
    }

    /// Current count of a running channel (counting down from TDR).
    fn tau_count(&self, u: usize, ch: usize) -> u16 {
        match self.tau[u].due[ch] { Some(d) => ((d.saturating_sub(self.cycles)) / self.tau_div(u, ch)).min(0xFFFF) as u16, None => 0xFFFF }
    }

    fn cell(&mut self, a: u32) -> Option<&mut u8> {
        match a {
            0xFFF00..=0xFFFFF => Some(&mut self.sfr[(a - 0xFFF00) as usize]),
            0xF0000..=0xF07FF => Some(&mut self.sfr2[(a - 0xF0000) as usize]),
            0xF0800..=0xF0FFF => Some(&mut self.fsl_ram[(a - 0xF0800) as usize]),
            _ => None,
        }
    }
}

impl Rl78Bus for G13 {
    fn read(&mut self, a: u32) -> u8 {
        self.bus_accesses += 1;
        if let Some((lo, hi)) = self.watch { if (lo..hi).contains(&a) { eprintln!("[rl78] {:05x}: read {a:05x}", self.cur_pc); } }
        if self.logging { self.log.entry(a).or_insert((0, 0, 0)).0 += 1; }
        match a {
            0xFFF00..=0xFFF0F => { let i = (a - 0xFFF00) as usize; let pm = self.sfr[0x20 + i]; return (self.sfr[i] & !pm) | (self.pins[i] & pm); }
            0xFFFA2 => return 0xFF, // OSTC: oscillation stable
            // Boot-ROM flash firmware information block: last code-flash block (512 x 1 KiB), fCLK range 1..32 MHz.
            0xEFFE6 => return 0xFF,
            0xEFFE7 => return 0x01,
            0xEFFE8 => return 1,
            0xEFFE9 => return 32,
            0xFFF10 | 0xFFF12 | 0xFFF44 | 0xFFF46 | 0xFFF48 | 0xFFF4A | 0xFFF14 | 0xFFF16 => {
                let (u, ch) = sdr_chan(a).unwrap();
                let ssr = SAU_BASE[u] + 2 * ch as u32; let st = self.r16(ssr) & !0x20; self.w16(ssr, st);
            }
            0xF0180..=0xF018F | 0xF01C0..=0xF01CF => { let u = (a >= 0xF01C0) as usize; let ch = ((a - TAU_BASE[u]) / 2) as usize; let c = self.tau_count(u, ch); return if a & 1 == 0 { c as u8 } else { (c >> 8) as u8 }; }
            0xFFFA4 => { let v = self.sfr[0xA4]; return (v & 0x50) | ((v & 0x10) << 1) | ((v & 0x40) << 1); } // CKC: MCS/CLS follow MCM0/CSS
            _ => {}
        }
        self.cell(a).map_or(0xFF, |c| *c)
    }
    fn write(&mut self, a: u32, v: u8) {
        self.bus_accesses += 1;
        if (0xFFFD0..=0xFFFEF).contains(&a) { self.irq_dirty = true; }
        if let Some((lo, hi)) = self.watch { if (lo..hi).contains(&a) { eprintln!("[rl78] {:05x}: write {a:05x} = {v:02x}", self.cur_pc); } }
        if self.logging { let e = self.log.entry(a).or_insert((0, 0, 0)); e.1 += 1; e.2 = v; }
        match a {
            0xFFF10 | 0xFFF12 | 0xFFF44 | 0xFFF46 | 0xFFF48 | 0xFFF4A | 0xFFF14 | 0xFFF16 => {
                let (u, ch) = sdr_chan(a).unwrap();
                self.sfr[(a - 0xFFF00) as usize] = v;
                self.sau_write_data(u, ch, v);
                return;
            }
            0xFFF01 => { self.sfr[1] = v; self.spi.select(v & 0x40 == 0); return; }
            // WDTE: anything but ACh is an illegal write and resets the chip (the firmware's software reset,
            // `ONEB !WDTE` at 00176, used after a firmware update).
            0xFFFAB => { if v != 0xAC { self.reset_request = true; } return; }
            0xFFF30 => { // ADM0: ADCS (bit 7) with ADCE (bit 0) starts a conversion; it completes at once (ADIF, ADCR)
                self.sfr[0x30] = v;
                if v & 0x81 == 0x81 {
                    let ch = self.sfr[0x31];
                    if self.adc_seen.insert(ch) { eprintln!("[rl78 adc] first conversion of channel {ch:02x}"); }
                    let f = self.adc_inputs.get(&ch).copied().unwrap_or(0.5).clamp(0.0, 1.0);
                    let code = ((f * 1023.0).round() as u16) << 6;
                    self.sfr[0x1E] = code as u8; self.sfr[0x1F] = (code >> 8) as u8;
                    self.request(24);
                    if self.sfr[0x32] & 0x20 != 0 { self.sfr[0x30] &= !0x80; } // one-shot conversion mode: ADCS clears
                }
                return;
            }
            0xF00E8 => { self.sfr2[0xE8] = v; if v & 0x81 == 0x81 { self.mdu_divide(); } return; }
            0xFFFF0..=0xFFFF3 => { self.sfr[(a - 0xFFF00) as usize] = v; if self.sfr2[0xE8] & 0x80 == 0 { self.mdu_multiply(); } return; }
            0xF0122 | 0xF0162 => { let u = (a == 0xF0162) as usize; let se = SAU_BASE[u] + 0x20; let x = self.r16(se) | (v as u16 & 15); self.w16(se, x); return; } // SSm
            0xF0124 | 0xF0164 => { let u = (a == 0xF0164) as usize; let se = SAU_BASE[u] + 0x20; let x = self.r16(se) & !(v as u16 & 15); self.w16(se, x); for ch in 0..4 { if v & (1 << ch) != 0 { self.sau[u].due[ch] = None; } } return; } // STm
            0xF0123 | 0xF0125 | 0xF0163 | 0xF0165 | 0xF0120 | 0xF0121 | 0xF0160 | 0xF0161 => return,
            0xF0108..=0xF010F | 0xF0148..=0xF014F => { // SIRmn: clear error flags
                let u = (a >= 0xF0148) as usize; let ch = ((a - SAU_BASE[u] - 8) / 2) as usize;
                if a & 1 == 0 { let ssr = SAU_BASE[u] + 2 * ch as u32; let st = self.r16(ssr) & !(v as u16 & 7); self.w16(ssr, st); }
                return;
            }
            0xF01B2 | 0xF01F2 => { self.tau_start((a == 0xF01F2) as usize, v); return; } // TSm (trigger, reads 0)
            0xF01B4 | 0xF01F4 => { self.tau_stop((a == 0xF01F4) as usize, v); return; }  // TTm
            0xF01B3 | 0xF01B5 | 0xF01F3 | 0xF01F5 => return,
            0xF01B0 | 0xF01B1 | 0xF01F0 | 0xF01F1 => return, // TEm read-only
            _ => {}
        }
        if let Some(c) = self.cell(a) { *c = v; }
    }
}
