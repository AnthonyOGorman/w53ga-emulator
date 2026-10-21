//! The companion controller (RL78/G13) attached to the SH-2A's SCIF7 through its UART1, with the GNSS receiver on its
//! UART0.

use super::{cpu::Rl78, g13::G13, Rl78Fault};
use crate::periph::scif::Scif;
use crate::periph::subcpu::{decode_main, decode_peer};

/// RL78 fCLK (20 MHz) per SH-2A P0 (100/3 MHz): 3/5.
const NUM: u64 = 3;
const DEN: u64 = 5;
/// The RL78 UART wired to SCIF7.
pub const LINK_UART: usize = 1;
/// The RL78 UART wired to the GNSS receiver.
pub const GPS_UART: usize = 0;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Companion {
    pub cpu: Rl78,
    pub g13: G13,
    frac: i64,
    scif_cursor: usize,
    pub fault: Option<Rl78Fault>,
    pub log_link: bool,
    /// Wire bytes per direction (0 = SH-2A -> RL78, 1 = RL78 -> SH-2A) not yet decoded, for the frame log.
    pending: [Vec<u8>; 2],
    /// Decoded frames: (direction, P0 time, seq, command, payload).
    pub frames: Vec<(u8, u64, u8, u8, Vec<u8>)>,
    now: u64,
    /// Debug breakpoints (AR_RL78_BREAK=hex,hex,...): dump registers and stack when reached.
    breaks: Vec<u32>,
    /// GNSS receiver on UART0.
    pub gnss: Option<crate::gnss::Gnss>,
    /// Diagnostics: RL78 PC histogram when enabled.
    #[serde(skip)]
    pub pc_hist: Option<std::collections::HashMap<u32, u64>>,
    /// Internal resets taken.
    #[serde(skip)]
    pub resets: u64,
}

impl Companion {
    /// The controller's code flash image from a 1 MiB two-bank image, or from an update package that carries one
    /// behind an 88-byte header: the 512 KiB bank `half` (0 = the one the controller runs).
    pub fn firmware_image(file: &[u8], half: usize) -> Result<Vec<u8>, String> {
        if half > 1 { return Err("bank must be 0 or 1".into()); }
        let payload = match file.len() { 0x100058 => &file[88..], 0x100000 => file, n => return Err(format!("controller image is {n} bytes: expected a 1 MiB image (1048576 bytes) or an update package carrying one (1048664 bytes)")) };
        Ok(payload[half * 0x80000..(half + 1) * 0x80000].to_vec())
    }

    pub fn new(flash: Vec<u8>) -> Companion {
        let mut g13 = G13::default();
        g13.logging = false;
        Companion { cpu: Rl78::new(flash), g13, frac: 0, scif_cursor: 0, fault: None, log_link: std::env::var_os("AR_LINK_LOG").is_some(), pending: [Vec::new(), Vec::new()], frames: Vec::new(), now: 0,
            breaks: std::env::var("AR_RL78_BREAK").map(|v| v.split(',').filter_map(|x| u32::from_str_radix(x, 16).ok()).collect()).unwrap_or_default(), gnss: None, pc_hist: None, resets: 0 }
    }
    /// Re-read the debug environment switches (after loading a saved state).
    pub fn refresh_debug_env(&mut self) {
        self.log_link = std::env::var_os("AR_LINK_LOG").is_some();
        self.breaks = std::env::var("AR_RL78_BREAK").map(|v| v.split(',').filter_map(|x| u32::from_str_radix(x, 16).ok()).collect()).unwrap_or_default();
        self.g13.fsl_log = std::env::var_os("AR_FSL_LOG").is_some();
        if std::env::var_os("AR_RL78_SAU").is_some() { self.g13.logging = true; }   // register access counts (run.exe report)
        // AR_RL78_WATCH=lo,hi (hex): print every access in that range with the instruction address
        self.g13.watch = std::env::var("AR_RL78_WATCH").ok().and_then(|v| { let (a, b) = v.split_once(',')?; Some((u32::from_str_radix(a, 16).ok()?, u32::from_str_radix(b, 16).ok()?)) });
    }

    /// Internal reset of the RL78 (watchdog: illegal WDTE write): the CPU restarts from the reset vector of the code flash
    /// as it is now (after self-programming), RAM keeps its contents, on-chip peripherals return to their reset state and
    /// RESF reports WDTRF. The external SPI flash, the pins and the analog inputs are outside the chip and carry over.
    pub fn chip_reset(&mut self) {
        let flash = std::mem::take(&mut self.cpu.flash);
        let (ram, data_flash) = (std::mem::take(&mut self.cpu.ram), std::mem::take(&mut self.cpu.data_flash));
        let mut cpu = Rl78::new(flash);
        cpu.ram = ram; cpu.data_flash = data_flash;
        let old = std::mem::take(&mut self.g13);
        let mut g = G13::default();
        g.logging = false;
        g.spi = old.spi; g.pins = old.pins; g.adc_inputs = old.adc_inputs; g.boot_swap = old.boot_swap; g.fsl_log = old.fsl_log;
        g.sfr[0xA8] = 0x10; // RESF.WDTRF
        self.cpu = cpu;
        self.g13 = g;
        self.resets += 1;
        eprintln!("[rl78] internal reset (WDTE) at {}: pc {:05x}", self.now, self.cpu.pc);
    }

    /// Link time (SH-2A P0 clocks since attach).
    pub fn now(&self) -> u64 { self.now }

    /// Start forwarding only SCIF7 bytes transmitted after this point.
    pub fn sync(&mut self, scif: &Scif) { self.scif_cursor = scif.tx_log.len(); }

    fn dump(&mut self) {
        let c = &self.cpu;
        let sp = c.sp() as u32;
        let stack: Vec<u8> = (0..64).map(|i| c.ram.get((0xF0000 + sp + i - 0xF7F00) as usize).copied().unwrap_or(0)).collect();
        if let Ok(spec) = std::env::var("AR_RL78_MEM") {
            for item in spec.split(';') {
                // "HL+129,10" or "f9b43,10": address relative to HL/DE or absolute
                let Some((a, n)) = item.split_once(',') else { continue };
                let base = if let Some(o) = a.strip_prefix("HL+") { 0xF0000 + c.rp(3) as u32 + u32::from_str_radix(o, 16).unwrap_or(0) }
                    else if let Some(o) = a.strip_prefix("DE+") { 0xF0000 + c.rp(2) as u32 + u32::from_str_radix(o, 16).unwrap_or(0) }
                    else { u32::from_str_radix(a, 16).unwrap_or(0) };
                let n = usize::from_str_radix(n, 16).unwrap_or(16);
                let bytes: Vec<u8> = (0..n as u32).map(|i| c.ram.get((base + i).wrapping_sub(0xF7F00) as usize).copied().unwrap_or(0xEE)).collect();
                eprintln!("[rl78 mem] {base:05x}: {:02x?}", bytes);
            }
        }
        eprintln!("[rl78 break] t {} pc {:05x} AX={:04x} BC={:04x} DE={:04x} HL={:04x} SP={:04x} PSW={:02x} stack {:02x?}", self.now, c.pc, c.rp(0), c.rp(1), c.rp(2), c.rp(3), c.sp(), c.psw(), stack);
    }

    fn record(&mut self, dir: u8, bytes: &[u8]) {
        let buf = &mut self.pending[dir as usize];
        buf.extend_from_slice(bytes);
        loop {
            let r = if dir == 0 { decode_main(buf).map(|r| r.map(|(f, n)| ((f.seq, f.command, f.event, f.state, f.status, f.payload), n))) }
                else { decode_peer(buf).map(|r| r.map(|(f, n)| ((f.seq, f.command, f.event, f.state, f.status, f.payload), n))) };
            match r {
                None => break,
                Some(Err(k)) => { buf.drain(..k); }
                Some(Ok(((seq, cmd, ev, st, stat, payload), n))) => {
                    buf.drain(..n);
                    if self.log_link {
                        let hex: String = payload.iter().take(48).map(|b| format!("{b:02x}")).collect();
                        eprintln!("[link] {:>11} {} seq {seq:3} cmd {cmd:02x} ev {ev:04x} st {st:04x} status {stat:02x} len {} {hex}{}", self.now, if dir == 0 { "SH->RL" } else { "RL->SH" }, payload.len(), if payload.len() > 48 { "..." } else { "" });
                    }
                    self.frames.push((dir, self.now, seq, cmd, payload));
                }
            }
        }
    }

    /// Advance by `p0` SH-2A peripheral clocks and exchange link bytes with SCIF7.
    pub fn advance(&mut self, p0: u64, scif: Option<&mut Scif>) {
        self.now += p0;
        self.frac += (p0 * NUM) as i64;
        crate::prof::set(crate::prof::RL78);
        // `frac` counts fCLK cycles x DEN available; each instruction consumes its clock count.
        let debug = !self.breaks.is_empty() || self.pc_hist.is_some();
        while self.frac >= DEN as i64 {
            if self.fault.is_some() { self.frac = 0; break; }
            if debug {
                if !self.breaks.is_empty() && self.breaks.contains(&self.cpu.pc) { self.dump(); }
                if let Some(h) = &mut self.pc_hist { *h.entry(self.cpu.pc).or_default() += 1; }
            }
            // halted with nothing pending: skip to the next peripheral event (the same cycles as stepping one by one)
            if self.cpu.halted && !self.cpu.stopped && !debug {
                if let Some(q) = self.g13.quiet_cycles() {
                    let c = q.max(1).min((self.frac / DEN as i64) as u64).max(1);
                    self.cpu.steps += c;
                    self.frac -= (c * DEN) as i64;
                    self.g13.tick(c);
                    continue;
                }
            }
            if let Err(e) = self.g13.step(&mut self.cpu) { eprintln!("[rl78] fault {e:?}"); self.fault = Some(e); }
            if self.g13.reset_request { self.chip_reset(); continue; }
            let c = self.cpu.cycles.max(1) as u64;
            self.frac -= (c * DEN) as i64;
            self.g13.tick(c);
        }
        if let Some(g) = &mut self.gnss {
            let mut out = Vec::new();
            let tx = self.g13.uart_take_tx(GPS_UART);
            if !tx.is_empty() { g.on_rx(&tx, &mut out); }
            g.tick(self.now, &mut out);
            if !out.is_empty() { self.g13.uart_receive(GPS_UART, &out); }
        }
        if let Some(s) = scif {
            if s.tx_log.len() > self.scif_cursor {
                let new = s.tx_log[self.scif_cursor..].to_vec();
                self.scif_cursor = s.tx_log.len();
                self.g13.uart_receive(LINK_UART, &new);
                self.record(0, &new);
            }
            let out = self.g13.uart_take_tx(LINK_UART);
            if !out.is_empty() {
                s.queue_rx(&out);
                self.record(1, &out);
            }
        }
    }
}
