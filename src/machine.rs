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
        self.mmio_accesses += 1;
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
        self.mmio_accesses += 1;
        mmio_log(self.now, pc, a, n, v);
        if !Board::write(self, a, n, v) {
            // Unmodelled register: the store is accepted and remembered, nothing else happens.
            if self.observed.insert((a, n), v).is_none() { eprintln!("[mmio] unmodelled write {a:08x}/{n} = {v:x} at pc {pc:08x} (observed)"); }
        }
    }
    fn ibnr(&mut self) -> u16 { self.latches.value(IBNR) as u16 }
    fn set_ibnr(&mut self, v: u16) { self.latches.set(IBNR, v as u64); }
    fn flash_cmd_mode(&self) -> bool { self.nor.in_command_mode() }
}

/// Diagnostics: AR_MMIO_LOG=FILE writes every MMIO store as "P0-time pc W addr/size value" (a program's register
/// set-up sequences, e.g. to study how it drives the hardware). Off unless the variable is set.
fn mmio_log_file() -> Option<&'static std::sync::Mutex<std::io::BufWriter<std::fs::File>>> {
    static LOG: std::sync::OnceLock<Option<std::sync::Mutex<std::io::BufWriter<std::fs::File>>>> = std::sync::OnceLock::new();
    LOG.get_or_init(|| std::env::var_os("AR_MMIO_LOG").and_then(|p| std::fs::File::create(p).ok()).map(|f| std::sync::Mutex::new(std::io::BufWriter::new(f)))).as_ref()
}

fn mmio_log(now: u64, pc: u32, a: u32, n: usize, v: u64) {
    use std::io::Write;
    if let Some(Ok(mut w)) = mmio_log_file().map(|l| l.lock()) { let _ = writeln!(w, "{now} {pc:08x} W {a:08x}/{n} {v:x}"); }
}

/// Flush the AR_MMIO_LOG file (it is buffered); front ends call this before they exit.
pub fn mmio_log_flush() {
    use std::io::Write;
    if let Some(Ok(mut w)) = mmio_log_file().map(|l| l.lock()) { let _ = w.flush(); }
}

fn one() -> u32 { 1 }
fn zero64() -> [u32; 64] { [0; 64] }
fn false64() -> [bool; 64] { [false; 64] }

/// Header of saved states; bump when the serialized layout changes.
pub const STATE_MAGIC: &[u8; 16] = b"W53GA-STATE-0001";
/// Optional trailer of a saved state: the results of runtime HLE detection (`crate::detect::Found`).
pub const DETECT_MAGIC: &[u8; 16] = b"W53GA-DETECT-001";

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
    /// Runtime detection of the HLE hooks when no profile names them (`crate::detect`); its results are kept in saved
    /// states (`save_state_to`). While it runs, the CPU carries its trace (`Cpu::trace`).
    #[serde(skip)]
    pub detect: Option<Box<crate::detect::Detector>>,
    /// Decoded code in flash for `Cpu::run`.
    #[serde(skip)]
    blocks: crate::cpu::BlockCache,
    /// Fast-forward through busy-wait loops whose iterations provably repeat (`spin_try`): on unless W53GA_SPIN=0.
    #[serde(skip, default = "spin_default")]
    pub spin_skip: bool,
    #[serde(skip)]
    spin: Spin,
}

fn spin_default() -> bool { std::env::var("W53GA_SPIN").map_or(true, |v| v != "0") }

impl Machine {
    pub fn from_checkpoint(flash: Vec<u8>, c: &Checkpoint) -> Result<Machine, String> {
        let mut cpu = Cpu::new(Memory::new(flash));
        c.apply(&mut cpu);
        Ok(Machine { cpu, board: Board::from_checkpoint(c)?, steps: c.steps, interrupts: 0, history: [0; 64], hist_irq: [false; 64], hist_pos: 0, cpu_per_p0: 1, sub: 0, p0_batch: 1, call_counts: None, gfx_hle: false, gfx: Default::default(), hooks: Default::default(), idle_skip: false, idle_skipped: 0, irq_hist: Default::default(), stop_pc: std::env::var("AR_SH_STOP_PC").ok().and_then(|v| u32::from_str_radix(&v, 16).ok()), resets: 0, arg_log: arg_log_env(), debug_hooks: false, detect: None, blocks: Default::default(), spin_skip: spin_default(), spin: Default::default() })
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

    /// Use an HLE profile: its hooks replace the current ones and runtime detection is off. The profile should have
    /// been verified against the flash image (`Profile::verify`). Without one (None) the hooks are detected at run time
    /// (`crate::detect`), continuing from what an earlier detection (for example in a loaded state) found. Callers choose
    /// in this order: an explicit profile, else a matching built-in one, else detection.
    pub fn set_profile(&mut self, p: Option<&crate::hle::Profile>) {
        match p {
            Some(p) => { self.hooks = p.hooks(); self.detect = None; self.cpu.trace = None; }
            None => {
                let d = self.detect.take().unwrap_or_default();
                self.hooks = d.hooks();
                self.cpu.trace = Some(Box::new(d.trace()));
                self.detect = Some(d);
            }
        }
    }

    /// No HLE at all: no profile and no runtime detection (2D graphics calls are not rendered, no idle skipping).
    pub fn disable_hle(&mut self) {
        self.hooks = Default::default();
        self.detect = None;
        self.cpu.trace = None;
    }

    /// What runtime detection has found so far (None when a profile is in use or HLE is off).
    pub fn detected(&self) -> Option<&crate::detect::Found> { self.detect.as_ref().map(|d| &d.found) }

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
        if self.idle_now() { return self.idle_tick(); }
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
        let pc0 = self.cpu.pc;
        self.cpu.step(&mut self.board).map_err(Stop::Cpu)?;
        if self.cpu.trace.is_some() { self.detect_check(pc0); }
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

    /// At the idle loop head with interrupts unmasked and nothing deliverable (`step` then skips the CPU).
    #[inline]
    fn idle_now(&mut self) -> bool {
        self.idle_skip && Some(self.cpu.pc) == self.hooks.idle_loop && self.cpu.imask() == 0 && !self.board.pending_request().is_some_and(|(_, l)| l > 0)
    }

    /// A tick of the peripherals with the CPU idle.
    fn idle_tick(&mut self) -> Result<(), Stop> {
        self.sub = 0; // the rest of this tick's instruction slots are idle too
        self.board.p0_per_step = self.p0_batch as u64;
        self.steps += self.p0_batch as u64;
        self.idle_skipped += self.p0_batch as u64;
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
        let mem = std::mem::replace(&mut self.cpu.mem, Memory { flash_cmd_mode: false, code_gen: 0, ext_writes: 0, flash: Vec::new(), sdram: Vec::new(), oram: Vec::new() });
        let mut cpu = Cpu::new(mem);
        cpu.pc = rd(&cpu.mem, pc_v);
        cpu.r[15] = rd(&cpu.mem, sp_v);
        cpu.float_mode = self.cpu.float_mode;
        cpu.mem.flash_cmd_mode = false;
        self.cpu = cpu;
        self.blocks.invalidate();
        self.board.chip_reset(&c)?;
        self.resets += 1;
        eprintln!("[reset] watchdog {} reset at step {}: pc {:08x} sp {:08x}", if power_on { "power-on" } else { "manual" }, self.steps, self.cpu.pc, self.cpu.r[15]);
        Ok(())
    }

    /// Intercept an RGAH entry point: render in software, return 0 (RGAH_OK) to the caller. False if `pc` is not one.
    #[inline]
    fn gfx_call(&mut self) -> bool {
        let Some(f) = self.hooks.rgah_at(self.cpu.pc) else { return false };
        let r = [self.cpu.r[4], self.cpu.r[5], self.cpu.r[6], self.cpu.r[7]];
        if let Some(d) = self.detect.as_deref_mut() {
            if !d.probation.is_empty() && !d.probe(self.cpu.trace.as_deref_mut(), &self.cpu.mem, self.cpu.pc, r, self.cpu.r[15], self.steps) {
                self.hooks = d.hooks();
                return false;
            }
        }
        self.render_call(f, r, self.cpu.r[15]);
        self.cpu.r[0] = 0;
        self.cpu.pc = self.cpu.pr;
        if let Some(t) = self.cpu.trace.as_deref_mut() { t.ret(self.cpu.pc, self.cpu.r[15]); }
        true
    }

    /// Render one RGAH call in software from its arguments (R4..R7 and the stack at `sp`).
    fn render_call(&mut self, f: crate::hle::RgahFn, r: [u32; 4], sp: u32) {
        use crate::gfx::*;
        use crate::hle::RgahFn;
        let mut call = Call { mem: &mut self.cpu.mem, r, sp };
        let ok = match f {
            RgahFn::FillRect => fill_rect(&mut call, &mut self.gfx),
            RgahFn::DrawImage => draw_image(&mut call, &mut self.gfx),
            RgahFn::BitBlt => bit_blt(&mut call, &mut self.gfx),
            RgahFn::BitBltFixedColor => bit_blt_fixed_color(&mut call, &mut self.gfx),
            RgahFn::Finish => Some(()),
        };
        if ok.is_none() && self.gfx.warned.len() < 20 {
            let msg = format!("RGAH_{} not rendered (args {:08x?}, sp {:08x})", f.name(), r, sp);
            eprintln!("[gfx] {msg}");
            self.gfx.warned.push(msg);
        }
    }

    /// After an instruction while runtime detection runs (`pc0`: where it executed): cheap checks, then the detector
    /// when the graphics core was accessed, the trace flagged something or an idle loop is being verified.
    #[inline]
    fn detect_check(&mut self, pc0: u32) {
        let Some(d) = self.detect.as_deref() else { return };
        let flagged = self.cpu.trace.as_ref().is_some_and(|t| t.event);
        if flagged || self.board.gpu.accesses != d.gpu_seen || d.verify.is_some() { self.detect_event(pc0); }
    }

    #[cold]
    #[inline(never)]
    fn detect_event(&mut self, pc0: u32) {
        use crate::detect::Action;
        let (Some(mut d), Some(mut t)) = (self.detect.take(), self.cpu.trace.take()) else { return };
        t.event = false;
        let mut changed = false;
        if self.board.gpu.accesses != d.gpu_seen {
            d.gpu_seen = self.board.gpu.accesses;
            let (a, w) = self.board.gpu.last;
            if let Some(Action::Unwind { index, f }) = d.on_gpu(&mut t, &self.cpu.mem, (a, w), self.steps, false) {
                let fr = t.frames[index];
                t.frames.truncate(index);
                self.unwind_and_render(&fr, f);
                changed = true;
            }
        }
        if t.entry_event {
            t.entry_event = false;
            changed |= d.on_entry(&mut t, &self.cpu.mem, self.steps);
            if t.light { t.frames.clear(); }
        }
        if t.cand_event {
            t.cand_event = false;
            if let Some(entry) = d.candidate(&t, &self.cpu.mem) {
                let before = d.found.rgah;
                self.speculate(&mut d, &t);
                if d.found.rgah == before { d.spec_rejected.push(entry); }
                d.sync(&mut t);
                changed = true;
            }
        }
        if let Some((to, sp)) = t.loop_event.take() {
            if d.verify.is_none() && d.found.idle_loop.is_none() { d.start_verify(to, sp); }
        }
        if d.verify.is_some() {
            let op = self.cpu.mem.fetch16(pc0).unwrap_or(0);
            d.verify_step(&mut t, &self.cpu, pc0, op, self.board.mmio_accesses, self.interrupts, self.steps);
            changed |= d.found.idle_loop.is_some() && self.hooks.idle_loop.is_none();
        }
        if changed { self.hooks = d.hooks(); d.sync(&mut t); }
        self.cpu.trace = Some(t);
        self.detect = Some(d);
    }

    /// Explore the call the CPU has just entered (it looks like a drawing call) ahead of time, without letting it affect
    /// the machine: the CPU runs on alone, with time and interrupts held, until it reaches the graphics core and then
    /// `finish`, or returns, or would touch any other device, or a budget runs out. The detector watches as usual. Then
    /// the CPU registers and RAM are put back exactly as they were, so the program goes on from the same point with
    /// whatever hooks were found, as if they had been there from the start.
    fn speculate(&mut self, d: &mut crate::detect::Detector, t: &crate::detect::Trace) {
        use crate::detect::Action;
        use crate::hle::RgahFn;
        let regs = CpuRegs::save(&self.cpu);
        let (sdram, oram) = (self.cpu.mem.sdram.clone(), self.cpu.mem.oram.clone());
        let depth = t.frames.len();
        let start = d.found.rgah;
        let mut io = SpecIo { board: &self.board, gpu: self.board.gpu.clone(), blocked: None };
        let mut seen = io.gpu.accesses;
        self.cpu.trace = Some(Box::new(t.clone()));
        let mut why = "budget";
        let mut n = 0;
        while n < SPEC_BUDGET {
            n += 1;
            if self.cpu.step(&mut io).is_err() { why = "fault"; break; }
            if io.blocked.is_some() { why = "device access"; break; }
            if self.cpu.sleeping { why = "sleep"; break; }
            let Some(tr) = self.cpu.trace.as_deref_mut() else { break };
            tr.event = false;
            tr.cand_event = false;
            tr.loop_event = None;
            if tr.entry_event { tr.entry_event = false; d.on_entry(tr, &self.cpu.mem, self.steps); }
            if io.gpu.accesses != seen {
                seen = io.gpu.accesses;
                if let Some(Action::Unwind { f: RgahFn::Finish, .. }) = d.on_gpu(tr, &self.cpu.mem, io.gpu.last, self.steps, true) { why = "finish found"; break; }
            }
            if tr.frames.len() < depth && d.found.rgah == start { why = "returned"; break; }
        }
        if crate::detect::debug() {
            eprintln!("[detect] explored {:x} for {n} instructions: stopped by {why}{}", regs.pc, io.blocked.map_or(String::new(), |a| format!(" ({a:08x})")));
        }
        regs.restore(&mut self.cpu);
        self.cpu.mem.sdram.copy_from_slice(&sdram);
        self.cpu.mem.oram.copy_from_slice(&oram);
        self.cpu.trace = None;
    }

    /// Complete a call identified while it was running: return to its caller with the callee-saved registers it was
    /// entered with (and its stack arguments as they were), rendering it in software from its entry arguments, with
    /// RGAH_OK as the result.
    fn unwind_and_render(&mut self, fr: &crate::detect::Frame, f: crate::hle::RgahFn) {
        let c = &mut self.cpu;
        c.r[8..15].copy_from_slice(&fr.saved);
        c.r[15] = fr.sp;
        (c.mach, c.macl, c.gbr, c.sr, c.fpscr) = (fr.mach, fr.macl, fr.gbr, fr.sr, fr.fpscr);
        c.fr[12..16].copy_from_slice(&fr.fr);
        for (k, w) in fr.stk.iter().enumerate() {
            let a = fr.sp.wrapping_add(4 * k as u32);
            if c.mem.read_plain(a, 4).is_some() { c.mem.write_plain(a, 4, *w as u64); }
        }
        if f != crate::hle::RgahFn::Finish { self.render_call(f, fr.args, fr.sp); }
        let c = &mut self.cpu;
        c.r[0] = 0;
        c.pr = fr.ret;
        c.pc = fr.ret;
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
        // Runtime HLE detection results follow the machine as an optional trailer (older readers ignore it).
        if let Some(d) = &self.detect {
            std::io::Write::write_all(&mut z, DETECT_MAGIC).map_err(|e| e.to_string())?;
            bincode::serialize_into(&mut z, &d.found).map_err(|e| e.to_string())?;
        }
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
        let mut m: Machine = bincode::deserialize_from(&mut z).map_err(|e| e.to_string())?;
        let mut tag = [0u8; 16];
        if std::io::Read::read_exact(&mut z, &mut tag).is_ok() && &tag == DETECT_MAGIC {
            let found: crate::detect::Found = bincode::deserialize_from(&mut z).map_err(|e| e.to_string())?;
            m.detect = Some(Box::new(crate::detect::Detector::new(found)));
        }
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

    /// Run for `n` peripheral clocks (P0 ticks). The same as calling `step` until then, but runs of instructions that
    /// need none of `step`'s checks go through `Cpu::run` (see `run_fast`).
    pub fn run(&mut self, n: u64) -> Result<(), Stop> {
        self.debug_hooks = self.stop_pc.is_some() || !self.arg_log.is_empty() || self.call_counts.is_some();
        let end = self.steps + n;
        self.blocks.invalidate();
        self.spin.active = false; // anything may have been changed from outside since the last call
        while self.steps < end {
            if self.debug_hooks || self.board.wdt.reset.is_some() || self.detect.as_ref().is_some_and(|d| d.verify.is_some()) { self.step()?; }
            else if self.idle_now() { self.idle_tick()?; } // as `step` would (no reset is pending)
            else if self.spin.active { self.spin_forward(end)?; }
            else {
                self.run_fast()?;
                if self.sub == 0 && self.spin_skip && self.cpu.trace.is_none() && self.cpu.watch.is_none() && self.steps < end { self.spin_try(end)?; }
            }
        }
        Ok(())
    }

    // ---------- busy-wait loops ----------
    //
    // Programs often wait in a loop that only reads memory until an interrupt handler changes it (a tick counter, a
    // receive ring, a flag). When one pass through such a loop leaves every CPU register as it found it and every
    // store it makes writes the value already there, the next pass is the same computation on the same inputs, and so
    // is every pass after it until something outside the CPU acts: an interrupt is accepted, or a device (DMA, the JPEG
    // unit, the camera, NOR programming) writes memory. The loop touches no device itself. So between those events the
    // CPU's state at any instruction count is known without executing anything: the peripherals are ticked as usual
    // (exactly as `run_fast` would tick them, every `cpu_per_p0 * p0_batch` instructions), and when the fast-forward ends
    // the CPU is put in the state it has at that point of the loop. The run is the same as executing every instruction.

    /// After a peripheral tick: look for a busy-wait loop from the current instruction, unless backing off after
    /// failed attempts. Instructions run through `step` meanwhile, so nothing is skipped while looking.
    fn spin_try(&mut self, end: u64) -> Result<(), Stop> {
        if self.spin.wait > 0 { self.spin.wait -= 1; return Ok(()); }
        self.spin.tries += 1;
        if self.spin_find(end)? {
            self.spin.found += 1;
            self.spin.backoff = 1;
            self.spin.active = true;
        } else {
            self.spin.backoff = (self.spin.backoff * 2).clamp(1, SPIN_MAX_BACKOFF);
            self.spin.wait = self.spin.backoff;
        }
        Ok(())
    }

    /// Can the next instruction be part of a fast-forwarded loop: nothing `step` checks before an instruction applies
    /// (an interrupt to accept, the idle loop, a hooked entry point, a pending reset) and the instruction (with its
    /// delay slot) is not one of those whose effects `SpinRegs` and the store check do not cover (`Cpu::spin_safe`).
    fn spin_can_step(&mut self, end: u64) -> bool {
        if self.steps >= end || self.board.wdt.reset.is_some() || self.debug_hooks { return false; }
        let imask = self.cpu.imask();
        if self.board.pending_request().is_some_and(|(_, l)| l > imask) { return false; }
        let pc = self.cpu.pc;
        if self.idle_skip && Some(pc) == self.hooks.idle_loop && imask == 0 { return false; }
        if self.gfx_hle { if let Some((lo, hi)) = self.hooks.rgah_range { if pc >= lo && pc <= hi && self.hooks.rgah.iter().any(|h| h.0 == pc) { return false; } } }
        self.cpu.spin_safe(pc)
    }

    /// One `step` while looking for a loop; false if it did anything a fast-forwarded loop may not do (a device access,
    /// an interrupt, SLEEP) or a device wrote memory meanwhile (in a tick).
    fn spin_step(&mut self, mark: &(u64, u64, u64, u64)) -> Result<bool, Stop> {
        self.step()?;
        Ok(!self.cpu.sleeping && (self.interrupts, self.board.mmio_accesses, self.cpu.mem.ext_writes, self.cpu.mem.code_gen) == *mark)
    }

    /// Run on from the current instruction looking for a pass of at most `SPIN_MAX` steps that returns to the same
    /// registers, then run one more such pass recording the registers before each step, with the store check on. True
    /// when the second pass also came back to the same registers, changed no memory (every store wrote the value
    /// already there) and kept SR.IMASK: the loop is then ready to be fast-forwarded from phase 0 (where the CPU is).
    fn spin_find(&mut self, end: u64) -> Result<bool, Stop> {
        let s0 = SpinRegs::of(&self.cpu);
        let mark = (self.interrupts, self.board.mmio_accesses, self.cpu.mem.ext_writes, self.cpu.mem.code_gen);
        let mut k = 0;
        for i in 1..=SPIN_MAX {
            if !self.spin_can_step(end) || !self.spin_step(&mark)? { return Ok(false); }
            if self.cpu.pc == s0.pc && SpinRegs::of(&self.cpu) == s0 { k = i; break; }
        }
        if k == 0 { return Ok(false); }
        let mut states = std::mem::take(&mut self.spin.states);
        let mut cur = std::mem::take(&mut self.spin.cur);
        states.clear();
        cur.clear();
        self.cpu.set_store_check(true);
        let mut ok = true;
        for _ in 0..k {
            if !self.spin_can_step(end) { ok = false; break; }
            states.push(SpinRegs::of(&self.cpu));
            match self.spin_step(&mark) {
                Ok(true) => cur.push(self.cpu.cur_pc),
                Ok(false) => { ok = false; break; }
                Err(e) => { self.cpu.set_store_check(false); return Err(e); }
            }
        }
        ok = ok && !self.cpu.wnoisy;
        self.cpu.set_store_check(false);
        let imask = (s0.sr >> 4) & 15;
        ok = ok && SpinRegs::of(&self.cpu) == s0 && states.iter().all(|s| (s.sr >> 4) & 15 == imask);
        if ok {
            self.spin.phase = 0;
            self.spin.imask = imask;
            self.spin.ext = mark.2;
            self.spin.gen = mark.3;
        }
        self.spin.states = states;
        self.spin.cur = cur;
        Ok(ok)
    }

    /// Fast-forward the verified loop: each tick's remaining instructions are passes (or parts of passes) through the
    /// loop, so only the peripherals are ticked, as `run_fast` would tick them after executing those instructions.
    /// Stops after the first tick at which the loop may no longer repeat (an interrupt it would accept, memory written
    /// by a device, a reset or a model fault pending) or the run ends; the CPU then gets the state of that point.
    fn spin_forward(&mut self, end: u64) -> Result<(), Stop> {
        let limit = (self.cpu_per_p0 * self.p0_batch) as usize;
        let k = self.spin.states.len();
        loop {
            let n = limit - self.sub as usize;
            // the PC history as executing those instructions would leave it (the last 64 of them)
            for i in n.saturating_sub(64)..n {
                let at = (self.hist_pos + i) & 63;
                self.history[at] = self.spin.states[(self.spin.phase + i) % k].pc;
                self.hist_irq[at] = false;
            }
            self.hist_pos += n;
            self.spin.phase = (self.spin.phase + n) % k;
            self.spin.skipped += n as u64;
            self.sub = 0;
            self.board.p0_per_step = self.p0_batch as u64;
            self.steps += self.p0_batch as u64;
            crate::prof::set(crate::prof::BOARD);
            self.board.tick_with(&mut self.cpu.mem);
            if let Some(f) = self.board.fault.take() { self.spin_stop(); return Err(Stop::Model(f)); }
            let imask = self.spin.imask;
            if self.steps >= end || self.board.wdt.reset.is_some() || self.cpu.mem.ext_writes != self.spin.ext || self.cpu.mem.code_gen != self.spin.gen
                || self.board.pending_request().is_some_and(|(_, l)| l > imask) {
                self.spin_stop();
                return Ok(());
            }
        }
    }

    /// End a fast-forward: the CPU registers of the current phase (before its instruction), and `cur_pc` as the
    /// instruction before it left it. Look again for a loop at the next tick.
    fn spin_stop(&mut self) {
        let (p, k) = (self.spin.phase, self.spin.states.len());
        self.spin.states[p].put(&mut self.cpu);
        self.cpu.cur_pc = self.spin.cur[(p + k - 1) % k];
        self.spin.active = false;
        self.spin.wait = 0;
        self.spin.backoff = 1;
    }

    /// Busy-wait fast-forward statistics: (attempts, loops found, instructions not executed).
    pub fn spin_stats(&self) -> (u64, u64, u64) { (self.spin.tries, self.spin.found, self.spin.skipped) }

    /// Instructions up to the next peripheral tick, as `step` would run them: the CPU runs on alone while nothing
    /// that `step` checks before an instruction can change, which needs a device access (then the CPU stops after that
    /// instruction) or a tick (the budget ends there). Whenever a check applies to the next instruction (idle loop, an
    /// interrupt to accept, a hooked entry point), that step is left to `step`.
    fn run_fast(&mut self) -> Result<(), Stop> {
        let limit = self.cpu_per_p0 * self.p0_batch;
        let irq_level = self.board.pending_request().map_or(0, |(_, l)| l);
        let idle_pc = if self.idle_skip { self.hooks.idle_loop.unwrap_or(1) } else { 1 };
        let (hooks, hooks_range) = match self.hooks.rgah_range { Some(r) if self.gfx_hle => (&self.hooks.rgah[..], r), _ => (&[][..], (1, 0)) };
        let mut lim = crate::cpu::RunLimits { budget: limit - self.sub, idle_pc, irq_level, hooks, hooks_range, history: &mut self.history, hist_irq: &mut self.hist_irq, hist_pos: &mut self.hist_pos };
        crate::prof::set(crate::prof::CPU);
        let (done, r) = self.cpu.run(&mut self.board, &mut lim, &mut self.blocks);
        self.sub += done;
        match r.map_err(Stop::Cpu)? {
            crate::cpu::RunExit::Budget => {}
            crate::cpu::RunExit::Check => return self.step(),
            crate::cpu::RunExit::After(pc0) => {
                if self.cpu.trace.is_some() { self.detect_check(pc0); }
                if self.cpu_per_p0 > 1 { self.board.flush_deferred(&mut self.cpu.mem); }
            }
        }
        if self.sub < limit { return Ok(()); }
        self.sub = 0;
        self.board.p0_per_step = self.p0_batch as u64;
        self.steps += self.p0_batch as u64;
        crate::prof::set(crate::prof::BOARD);
        self.board.tick_with(&mut self.cpu.mem);
        if let Some(f) = self.board.fault.take() { return Err(Stop::Model(f)); }
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

/// Instructions a speculative exploration may run (`Machine::speculate`).
const SPEC_BUDGET: u32 = 400_000;

/// Busy-wait loops of at most this many steps per pass are fast-forwarded (`Machine::spin_find`).
const SPIN_MAX: usize = 1024;
/// Ticks to wait at most between attempts to find one after failed attempts.
const SPIN_MAX_BACKOFF: u32 = 4096;

/// State of the busy-wait fast-forward (`Machine::spin_try`); not part of saved states.
#[derive(Default)]
struct Spin {
    /// Ticks until the next attempt, and the current wait after failures (doubles up to `SPIN_MAX_BACKOFF`).
    wait: u32,
    backoff: u32,
    /// A verified loop is ready or being fast-forwarded.
    active: bool,
    /// One pass of the loop: the registers before each of its steps, and `cur_pc` after each.
    states: Vec<SpinRegs>,
    cur: Vec<u32>,
    /// Position in the pass (the next step), SR.IMASK throughout, and memory's device-write and code generations.
    phase: usize,
    imask: u32,
    ext: u64,
    gen: u64,
    tries: u64,
    found: u64,
    skipped: u64,
}

/// The CPU registers a busy-wait loop can change or depend on (with memory); the others it cannot touch
/// (`Cpu::spin_safe` excludes the instructions that do).
#[derive(Clone, Copy, PartialEq, Eq)]
struct SpinRegs { r: [u32; 16], fr: [u32; 16], sr: u32, gbr: u32, vbr: u32, mach: u32, macl: u32, pr: u32, pc: u32, tbr: u32, fpscr: u32, fpul: u32 }

impl SpinRegs {
    fn of(c: &Cpu) -> SpinRegs {
        SpinRegs { r: c.r, fr: c.fr, sr: c.sr, gbr: c.gbr, vbr: c.vbr, mach: c.mach, macl: c.macl, pr: c.pr, pc: c.pc, tbr: c.tbr, fpscr: c.fpscr, fpul: c.fpul }
    }
    fn put(&self, c: &mut Cpu) {
        (c.r, c.fr, c.sr, c.gbr, c.vbr, c.mach, c.macl, c.pr, c.pc, c.tbr, c.fpscr, c.fpul) = (self.r, self.fr, self.sr, self.gbr, self.vbr, self.mach, self.macl, self.pr, self.pc, self.tbr, self.fpscr, self.fpul);
    }
}

/// The CPU state saved around a speculative exploration (everything but memory and the detection trace).
struct CpuRegs { r: [u32; 16], sr: u32, gbr: u32, vbr: u32, mach: u32, macl: u32, pr: u32, pc: u32, tbr: u32, fpscr: u32, fpul: u32, fr: [u32; 16], banks: [crate::cpu::Bank; 15], sleeping: bool, cur_pc: u32, bank_anomalies: u64 }

impl CpuRegs {
    fn save(c: &Cpu) -> CpuRegs {
        CpuRegs { r: c.r, sr: c.sr, gbr: c.gbr, vbr: c.vbr, mach: c.mach, macl: c.macl, pr: c.pr, pc: c.pc, tbr: c.tbr, fpscr: c.fpscr, fpul: c.fpul, fr: c.fr, banks: c.banks, sleeping: c.sleeping, cur_pc: c.cur_pc, bank_anomalies: c.bank_anomalies }
    }
    fn restore(&self, c: &mut Cpu) {
        (c.r, c.sr, c.gbr, c.vbr, c.mach, c.macl, c.pr, c.pc, c.tbr) = (self.r, self.sr, self.gbr, self.vbr, self.mach, self.macl, self.pr, self.pc, self.tbr);
        (c.fpscr, c.fpul, c.fr, c.banks, c.sleeping, c.cur_pc, c.bank_anomalies) = (self.fpscr, self.fpul, self.fr, self.banks, self.sleeping, self.cur_pc, self.bank_anomalies);
    }
}

/// The bus seen during a speculative exploration: the graphics core is a private copy of its model; any other device
/// access is refused (and ends the exploration) so that nothing outside the CPU and RAM can change.
struct SpecIo<'a> { board: &'a Board, gpu: crate::periph::rgpvg::Rgpvg, blocked: Option<u32> }

impl Mmio for SpecIo<'_> {
    fn read(&mut self, _pc: u32, a: u32, n: usize) -> u64 {
        if crate::periph::rgpvg::Rgpvg::handles(a) { return self.gpu.read(a, n).unwrap_or(0); }
        self.blocked.get_or_insert(a);
        0
    }
    fn write(&mut self, _pc: u32, a: u32, n: usize, v: u64) {
        if crate::periph::rgpvg::Rgpvg::handles(a) { let _ = self.gpu.write(a, n, v); return; }
        if (0xF000_0000..0xF200_0000).contains(&a) { return; } // cache arrays: no effect (as `Board::write`)
        self.blocked.get_or_insert(a);
    }
    fn ibnr(&mut self) -> u16 { self.board.latches.value(IBNR) as u16 }
    fn set_ibnr(&mut self, _v: u16) { self.blocked.get_or_insert(IBNR); }
    fn flash_cmd_mode(&self) -> bool { self.board.nor.in_command_mode() }
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
