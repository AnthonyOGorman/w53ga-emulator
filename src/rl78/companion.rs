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
    /// Radar transmitters the radar receiver can hear.
    #[serde(default)]
    pub radar: crate::radar::Scene,
    /// Debug taps from the environment (read once).
    #[serde(skip)]
    taps: Option<Box<Taps>>,
    /// Diagnostics: RL78 PC histogram when enabled.
    #[serde(skip)]
    pub pc_hist: Option<std::collections::HashMap<u32, u64>>,
    /// Internal resets taken.
    #[serde(skip)]
    pub resets: u64,
    /// Fast-forward through busy-wait loops whose passes provably repeat (`spin_record`): on unless W53GA_SPIN=0.
    #[serde(skip, default = "spin_default")]
    pub spin_skip: bool,
    #[serde(skip)]
    spin: RlSpin,
}

fn spin_default() -> bool { std::env::var("W53GA_SPIN").map_or(true, |v| v != "0") }

// ---------- busy-wait loops ----------
//
// The controller program often waits in a loop that only reads RAM until an interrupt handler changes it. When one pass
// through such a loop returns to the same registers (the register banks and SP, PSW, CS, ES, PMC), makes no SFR
// access and every store in it writes the value already in RAM, the next pass is the same computation on the same
// inputs, and so is every pass after it until an interrupt is accepted (nothing else can change RAM). So until then
// each instruction's effect is known: the registers it leaves, its clock count. `run_steps` then applies those
// instead of executing (whole passes at once where no peripheral event or the end of the clock budget falls inside),
// and stops before the instruction at which an interrupt would be accepted. The run is the same as executing them.

/// Steps per pass at most for a loop to be fast-forwarded, and advance calls to wait at most between attempts.
const SPIN_MAX: usize = 64;
const SPIN_MAX_BACKOFF: u32 = 4096;
const SPIN_IDLE: u8 = 0;
const SPIN_PASS1: u8 = 1;
const SPIN_PASS2: u8 = 2;
const SPIN_ACTIVE: u8 = 3;

/// The controller state a busy-wait loop changes: registers, PC; with `cur_pc` and the clocks of the instruction that
/// led to it (raw, and as consumed: at least 1).
#[derive(Clone, Copy, Default)]
struct RlState { banks: [u8; 32], hi: [u8; 8], pc: u32, cur_pc: u32, cycles: u32, used: u32 }

impl RlState {
    fn of(c: &Rl78) -> RlState { let (banks, hi) = c.spin_regs(); RlState { banks, hi, pc: c.pc, cur_pc: c.cur_pc, cycles: c.cycles, used: c.cycles.max(1) } }
    fn same(&self, o: &RlState) -> bool { self.banks == o.banks && self.hi == o.hi && self.pc == o.pc }
}

/// State of the controller's busy-wait fast-forward; not part of saved states.
#[derive(Default)]
struct RlSpin {
    mode: u8,
    /// `advance` calls until the next attempt, and the current wait after failures.
    wait: u32,
    backoff: u32,
    /// Where the attempt started, steps so far, steps per pass, SFR accesses and accepted interrupts at the start.
    s0: RlState,
    n: usize,
    k: usize,
    bus: u64,
    accepted: u64,
    /// One pass: the state after each of its steps; the next one (phase) and the clocks of a pass.
    states: Vec<RlState>,
    phase: usize,
    cycles: u64,
    tries: u64,
    found: u64,
    skipped: u64,
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
            breaks: std::env::var("AR_RL78_BREAK").map(|v| v.split(',').filter_map(|x| u32::from_str_radix(x, 16).ok()).collect()).unwrap_or_default(), gnss: None, radar: Default::default(), taps: None, pc_hist: None, resets: 0, spin_skip: spin_default(), spin: Default::default() }
    }
    /// Re-read the debug environment switches (after loading a saved state).
    pub fn refresh_debug_env(&mut self) {
        self.spin_stop();
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
        self.spin_stop();
        let flash = std::mem::take(&mut self.cpu.flash);
        let (ram, data_flash) = (std::mem::take(&mut self.cpu.ram), std::mem::take(&mut self.cpu.data_flash));
        let mut cpu = Rl78::new(flash);
        cpu.ram = ram; cpu.data_flash = data_flash;
        let old = std::mem::take(&mut self.g13);
        let mut g = G13::default();
        g.logging = false;
        g.spi = old.spi; g.pins = old.pins; g.rx = old.rx; g.adc_inputs = old.adc_inputs; g.boot_swap = old.boot_swap; g.fsl_log = old.fsl_log;
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
                    // main -> controller frames carry a 64-byte mirror block after the status byte (offset 11)
                    let mirror: String = if dir == 0 && n >= 75 { buf[11..75].iter().map(|b| format!("{b:02x}")).collect() } else { String::new() };
                    buf.drain(..n);
                    if self.log_link {
                        // AR_LINK_LOG=full: whole payloads and the mirror block; otherwise the first 48 payload bytes
                        let full = std::env::var("AR_LINK_LOG").is_ok_and(|v| v == "full");
                        let lim = if full { usize::MAX } else { 48 };
                        let hex: String = payload.iter().take(lim).map(|b| format!("{b:02x}")).collect();
                        let mir = if full && !mirror.is_empty() { format!(" mirror {mirror}") } else { String::new() };
                        eprintln!("[link] {:>11} {} seq {seq:3} cmd {cmd:02x} ev {ev:04x} st {st:04x} status {stat:02x} len {} {hex}{}{mir}", self.now, if dir == 0 { "SH->RL" } else { "RL->SH" }, payload.len(), if payload.len() > lim { "..." } else { "" });
                    }
                    self.frames.push((dir, self.now, seq, cmd, payload));
                }
            }
        }
    }

    /// Instructions while the controller runs (not halted or stopped, no debugging) and clocks remain: the same steps
    /// as `advance`'s loop, with its clock budget kept in a register. Returns to that loop when it ends, when the CPU
    /// halts or stops, on a fault and on an internal reset.
    #[inline(never)]
    fn run_steps(&mut self) {
        let mut frac = self.frac;
        if self.spin_skip && self.spin.mode == SPIN_IDLE {
            if self.spin.wait > 0 { self.spin.wait -= 1; } else { self.spin_start(); }
        }
        loop {
            if self.spin.mode == SPIN_ACTIVE {
                // the check `service` makes before an instruction: an interrupt to accept ends the fast-forward there
                if !self.g13.interrupt_due(&self.cpu) {
                    self.spin_forward(&mut frac);
                    if frac < DEN as i64 { break; }
                    continue;
                }
                self.spin_stop();
            }
            if let Err(e) = self.g13.step(&mut self.cpu) { eprintln!("[rl78] fault {e:?}"); self.fault = Some(e); }
            if self.g13.reset_request { self.frac = frac; self.chip_reset(); return; }
            let c = self.cpu.cycles.max(1) as u64;
            frac -= (c * DEN) as i64;
            self.g13.tick(c);
            if self.spin.mode != SPIN_IDLE { self.spin_record(); }
            if frac < DEN as i64 || self.cpu.halted || self.cpu.stopped || self.fault.is_some() { break; }
        }
        self.frac = frac;
    }

    /// Start looking for a busy-wait loop from the current instruction (the steps run as usual meanwhile).
    fn spin_start(&mut self) {
        if self.cpu.pc >= 0x80000 || self.cpu.halted || self.cpu.stopped { return self.spin_fail(); }
        self.cpu.wnoisy = false;
        let s = &mut self.spin;
        (s.s0, s.mode, s.n, s.bus, s.accepted) = (RlState::of(&self.cpu), SPIN_PASS1, 0, self.g13.bus_accesses, self.g13.accepted);
        s.tries += 1;
    }

    /// After a step while looking for a loop: first a pass of at most `SPIN_MAX` steps that returns to the registers
    /// it started from, then one more such pass with the store check on, recording the state after each step. The
    /// loop qualifies when that pass also returns to them, every store wrote the value already in RAM, IE and ISP
    /// stayed the same, and neither pass made an SFR access, accepted an interrupt, halted, faulted or left code flash.
    fn spin_record(&mut self) {
        let ok = !self.cpu.halted && !self.cpu.stopped && self.fault.is_none() && self.cpu.pc < 0x80000 && !self.cpu.wnoisy
            && self.g13.bus_accesses == self.spin.bus && self.g13.accepted == self.spin.accepted;
        if !ok { return self.spin_fail(); }
        let st = RlState::of(&self.cpu);
        let s = &mut self.spin;
        s.n += 1;
        if s.mode == SPIN_PASS1 {
            if st.same(&s.s0) {
                (s.k, s.n, s.mode) = (s.n, 0, SPIN_PASS2);
                s.states.clear();
                (self.cpu.wcheck, self.cpu.wnoisy) = (true, false);
            } else if s.n >= SPIN_MAX { self.spin_fail(); }
            return;
        }
        s.states.push(st);
        if s.n < s.k { return; }
        self.cpu.wcheck = false;
        let ie = s.s0.hi[2] & 0x86;
        if st.same(&s.s0) && s.states.iter().all(|x| x.hi[2] & 0x86 == ie) {
            (s.mode, s.phase, s.backoff) = (SPIN_ACTIVE, 0, 1);
            s.cycles = s.states.iter().map(|x| x.used as u64).sum();
            s.found += 1;
        } else { self.spin_fail(); }
    }

    /// No loop found here: try again after a wait that doubles with each failure.
    fn spin_fail(&mut self) {
        self.cpu.wcheck = false;
        let s = &mut self.spin;
        s.mode = SPIN_IDLE;
        s.backoff = (s.backoff * 2).clamp(1, SPIN_MAX_BACKOFF);
        s.wait = s.backoff;
    }

    /// End a fast-forward (the controller's state is always exact) or an attempt; look again at the next call.
    fn spin_stop(&mut self) {
        self.cpu.wcheck = false;
        let s = &mut self.spin;
        (s.mode, s.wait, s.backoff) = (SPIN_IDLE, 0, 1);
    }

    /// Fast-forward the verified loop by one instruction, after as many whole passes as the clock budget allows
    /// without reaching the next peripheral event, as `run_steps` would have executed them.
    fn spin_forward(&mut self, frac: &mut i64) {
        let den = DEN as i64;
        let k = self.spin.states.len();
        if self.spin.phase == 0 {
            // whole passes: each instruction must start with at least DEN left, and none may reach the next event
            let (c, last) = (self.spin.cycles as i64, self.spin.states[k - 1].used as i64);
            let by_frac = (*frac + (last - 1) * den) / (c * den);
            let to_event = self.g13.cycles_to_event();
            let by_event = if to_event == 0 { 0 } else { ((to_event - 1) / c as u64).min(i64::MAX as u64) as i64 };
            let m = by_frac.min(by_event);
            if m > 0 {
                self.cpu.steps += m as u64 * k as u64;
                *frac -= m * c * den;
                self.g13.add_cycles(m as u64 * c as u64);
                self.spin.skipped += m as u64 * k as u64;
                if *frac < den { return; }
            }
        }
        let s = self.spin.states[self.spin.phase];
        self.cpu.set_spin_regs(&s.banks, &s.hi);
        (self.cpu.pc, self.cpu.cur_pc, self.cpu.cycles) = (s.pc, s.cur_pc, s.cycles);
        self.cpu.steps += 1;
        *frac -= s.used as i64 * den;
        self.g13.tick(s.used as u64);
        self.spin.phase = (self.spin.phase + 1) % k;
        self.spin.skipped += 1;
    }

    /// Busy-wait fast-forward statistics: (attempts, loops found, instructions not executed).
    pub fn spin_stats(&self) -> (u64, u64, u64) { (self.spin.tries, self.spin.found, self.spin.skipped) }

    /// Advance by `p0` SH-2A peripheral clocks and exchange link bytes with SCIF7.
    pub fn advance(&mut self, p0: u64, scif: Option<&mut Scif>) {
        self.now += p0;
        self.frac += (p0 * NUM) as i64;
        crate::prof::set(crate::prof::RL78);
        // `frac` counts fCLK cycles x DEN available; each instruction consumes its clock count.
        let taps = self.taps.get_or_insert_with(|| Box::new(Taps::from_env()));
        let debug = !self.breaks.is_empty() || self.pc_hist.is_some() || !taps.hits.is_empty();
        while self.frac >= DEN as i64 {
            if self.fault.is_some() { self.frac = 0; break; }
            if debug {
                if !self.breaks.is_empty() && self.breaks.contains(&self.cpu.pc) { self.dump(); }
                if let Some(h) = &mut self.pc_hist { *h.entry(self.cpu.pc).or_default() += 1; }
                if let Some(t) = &mut self.taps { t.hit(self.now, self.cpu.pc); }
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
            if !debug && !self.cpu.halted && !self.cpu.stopped { self.run_steps(); continue; }
            if self.spin.mode != SPIN_IDLE { self.spin_stop(); }
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
        // the position only for radars placed at a location: this runs after every batch
        let pos = if self.radar.needs_position() { self.gnss.as_mut().map(|g| { let f = g.current_mut(); (f.lat, f.lon) }) } else { None };
        if let Some(t) = &mut self.taps { if t.active { t.after_batch(self.now, &self.cpu); } }
        self.g13.rx.radar = self.radar.strength(self.now as f64 / (100.0e6 / 3.0), pos);
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

/// Debug taps on the controller, set by environment variables (read once):
/// - AR_RL78_PEEK=addr,addr,... (hex RAM addresses): print a byte whenever it has changed since the last batch;
/// - AR_RL78_HITS=addr,addr,... (hex code addresses): print when the controller executes them (at most twice a second each);
/// - AR_RL78_SAMPLE=1: print the controller's PC and the top of its stack every half second (where its time goes).
#[derive(Default)]
struct Taps {
    active: bool,
    peek: Vec<(u32, Option<u8>)>,
    hits: Vec<(u32, u64)>,
    sample: bool,
    next_sample: u64,
}

impl Taps {
    fn from_env() -> Taps {
        let list = |name: &str| std::env::var(name).map(|v| v.split(',').filter_map(|x| u32::from_str_radix(x.trim(), 16).ok()).collect::<Vec<u32>>()).unwrap_or_default();
        let peek: Vec<(u32, Option<u8>)> = list("AR_RL78_PEEK").into_iter().map(|a| (a, None)).collect();
        let hits: Vec<(u32, u64)> = list("AR_RL78_HITS").into_iter().map(|a| (a, 0)).collect();
        let sample = std::env::var_os("AR_RL78_SAMPLE").is_some();
        Taps { active: !peek.is_empty() || sample, peek, hits, sample, next_sample: 0 }
    }

    fn hit(&mut self, now: u64, pc: u32) {
        if let Some(e) = self.hits.iter_mut().find(|e| e.0 == pc) {
            if now >= e.1 { eprintln!("[hit] {now:>12} {pc:05x}"); e.1 = now + 16_666_666; }
        }
    }

    fn after_batch(&mut self, now: u64, cpu: &Rl78) {
        if self.sample && now >= self.next_sample {
            self.next_sample = now + 16_666_666;
            let sp = cpu.sp() as u32;
            let st: String = (0..16).map(|i| format!("{:02x}", cpu.ram.get((0xF0000 + sp + i - 0xF7F00) as usize).copied().unwrap_or(0))).collect();
            eprintln!("[sample] {now:>12} pc {:05x} sp {sp:04x} {st}", cpu.pc);
        }
        for (a, last) in &mut self.peek {
            let v = cpu.ram.get(a.wrapping_sub(0xF7F00) as usize).copied().unwrap_or(0);
            if *last != Some(v) { eprintln!("[peek] {now:>12} {a:05x} = {v:02x}"); *last = Some(v); }
        }
    }
}
