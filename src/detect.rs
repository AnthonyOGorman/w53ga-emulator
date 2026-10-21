//! Runtime detection of the HLE hooks (`crate::hle`) when no profile names them.
//!
//! Without a profile the emulator watches the program run and finds the entry points itself, from what the hardware
//! sees and how the program calls into it. Nothing about a particular program is built in: no addresses, no code
//! bytes, no hashes. Everything is relative to what is observed, so it does not depend on where the program keeps
//! its code.
//!
//! **Call tracking.** While detection runs, the CPU keeps a shadow call stack (`Trace`): every call (JSR, BSR, BSRF,
//! JSR/N) pushes a frame with its entry point, return address, stack pointer, arguments (R4..R7 and the first three
//! stack words) and the callee-saved registers; returns, RTE and exception entry maintain it.
//!
//! **2D graphics (RGAH API).** The graphics core's registers (`periph::rgpvg`) are only reached through its driver.
//! At a graphics-core access made at task level, the frames of the shadow stack are matched against the argument
//! layouts of the RGAH drawing calls as `gfx` reads them: a frame matches when every argument reads as what that call
//! takes (a target bitmap descriptor with a valid format, a clip area inside it and pixels in RAM; source bitmaps,
//! points, rectangles, transform matrices, glyph masks, effect records, alpha values). The innermost frame matching
//! exactly one call is the API entry point: the driver functions below it take other arguments, and wrappers above it
//! pass the same ones but are outer. The frames from that entry point down to the hardware access delimit the
//! library's code. `finish` takes no drawing arguments: it is the outermost library frame, called from outside the
//! library, matching no drawing call, whose first graphics-core access is a write to the register the first identified
//! drawing call wrote first (the command port).
//!
//! The first drawing call must not be lost, so it is found ahead of time: when a call's first argument looks like a
//! bitmap and its arguments read as exactly one drawing call, the machine explores it (`Machine::speculate`): the CPU
//! runs on alone, with time held and every device but a private copy of the graphics core refused, until the program
//! has reached the graphics core and then `finish` (or returns, or would touch a device, or a budget runs out). Then
//! the CPU and RAM are put back exactly as they were and the program continues with the hooks found, as if they had
//! been there from the start. From then on, a call from outside the library to an unknown entry inside it whose
//! arguments match exactly one still unknown drawing call is recognised on entry, before any of it runs. As a last
//! resort, a call identified at a real graphics-core access is unwound to its caller (with the callee-saved registers
//! it was entered with) and completed in software from its entry arguments.
//!
//! A newly hooked drawing entry is on probation for its first calls: if one does not match its layout, the hook is
//! withdrawn and the entry never offered again. Ambiguous matches (more than one layout, or a second entry for a call
//! already found) identify nothing and are reported. Once the library, `finish` and the idle loop are known, the trace
//! drops to a light mode that keeps no call stack and only watches calls into the library (for drawing calls not seen
//! yet); its cost is then within measurement noise.
//!
//! **Idle loop.** A loop is a candidate when the same backward branch (same target, same stack pointer) keeps being
//! taken. It is then verified instruction by instruction: from the loop head back to it, with interrupts unmasked,
//! no interrupt and no MMIO access, consecutive iterations must go through identical complete CPU states (all
//! registers, FPU and control registers, after every instruction). Then each iteration's loads and stores are
//! identical too: the loop is a fixed point that only an interrupt can leave, so skipping it while no interrupt is
//! deliverable is exact. Three such idle periods, separated by interrupts, are required. The reported address is the
//! start of the basic block that makes the loop's first call (the call into the RTOS), or the loop's branch target
//! for a loop without calls.

use crate::cpu::Cpu;
use crate::hle::RgahFn;
use crate::mem::Memory;

/// One frame of the shadow call stack.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Frame {
    pub entry: u32,
    pub ret: u32,
    /// Stack pointer at entry (after the call; the stack arguments start here).
    pub sp: u32,
    pub args: [u32; 4],
    pub stk: [u32; 3],
    /// R8..R14 at entry; with MACH, MACL, GBR, SR, FPSCR and FR12..FR15 restored when the frame is unwound.
    pub saved: [u32; 7],
    pub mach: u32,
    pub macl: u32,
    pub gbr: u32,
    pub sr: u32,
    pub fpscr: u32,
    pub fr: [u32; 4],
    /// Exception/interrupt entry rather than a call.
    pub irq: bool,
    /// A graphics-core access has been seen while this frame was live.
    pub touched: bool,
}

/// CPU-side observer kept in `Cpu::trace` while detection runs: the shadow call stack and backward-branch counting.
#[derive(Clone, Debug, Default)]
pub struct Trace {
    pub frames: Vec<Frame>,
    pub irq_depth: u32,
    /// Something for the machine to examine after this instruction.
    pub event: bool,
    /// A call into the learned library range from outside it, to an entry not yet known (the frame on top).
    pub entry_event: bool,
    /// Library code range learned from the first identified call, and the entries already known.
    pub range: Option<(u32, u32)>,
    pub known: Vec<u32>,
    /// Count backward branches for idle-loop detection.
    pub loops: bool,
    /// Direct-mapped table of backward branches: (target, stack pointer, count).
    table: Vec<(u32, u32, i32)>,
    /// A loop (target, sp) taken often enough to verify.
    pub loop_event: Option<(u32, u32)>,
    /// Before the library is known: flag calls whose first argument looks like a bitmap descriptor.
    pub probe_calls: bool,
    /// The call on top may be a drawing call (its first argument looks like a bitmap descriptor).
    pub cand_event: bool,
    /// Light mode (`Detector::light`): no shadow stack; only calls from outside the library to unknown entries inside
    /// it are recorded, to recognise drawing calls not seen yet on entry.
    pub light: bool,
}

const MAX_FRAMES: usize = 256;
const LOOP_TABLE: usize = 64;
const LOOP_TRIGGER: i32 = 64;

impl Trace {
    pub fn new(loops: bool) -> Trace { Trace { loops, table: vec![(0, 0, 0); LOOP_TABLE], ..Default::default() } }

    /// Drop frames that can no longer be live: a callee's stack pointer never rises above its entry value.
    #[inline]
    fn prune(&mut self, sp: u32) { while self.frames.last().is_some_and(|f| f.sp < sp) { self.frames.pop(); } }

    /// After a call instruction (and its delay slot): the CPU is at the callee's entry.
    #[inline]
    pub fn call(&mut self, cpu: &CallRegs, mem: &Memory) {
        if self.light {
            let Some((lo, hi)) = self.range else { return };
            if !(lo..=hi).contains(&cpu.pc) || (lo..=hi).contains(&cpu.pr) || self.known.contains(&cpu.pc) { return; }
        }
        self.push(cpu, mem);
    }

    fn push(&mut self, cpu: &CallRegs, mem: &Memory) {
        let sp = cpu.r[15];
        self.prune(sp);
        if self.frames.len() >= MAX_FRAMES { self.frames.drain(..MAX_FRAMES / 2); }
        let rd = |a: u32| mem.read_plain(a, 4).map_or(0, |v| v as u32);
        let mut saved = [0; 7];
        saved.copy_from_slice(&cpu.r[8..15]);
        self.frames.push(Frame {
            entry: cpu.pc, ret: cpu.pr, sp, args: [cpu.r[4], cpu.r[5], cpu.r[6], cpu.r[7]], stk: [rd(sp), rd(sp.wrapping_add(4)), rd(sp.wrapping_add(8))],
            saved, mach: cpu.mach, macl: cpu.macl, gbr: cpu.gbr, sr: cpu.sr, fpscr: cpu.fpscr, fr: [cpu.fr[12], cpu.fr[13], cpu.fr[14], cpu.fr[15]], irq: false, touched: false,
        });
        if let Some((lo, hi)) = self.range {
            if (lo..=hi).contains(&cpu.pc) && !(lo..=hi).contains(&cpu.pr) && !self.known.contains(&cpu.pc) { self.entry_event = true; self.event = true; }
        } else if self.probe_calls && looks_like_bitmap(mem, cpu.r[4]) {
            self.cand_event = true;
            self.event = true;
        }
    }

    /// After a return (RTS, RTS/N, RTV/N, or a hooked call returning): the CPU is at `pc` with stack pointer `sp`.
    pub fn ret(&mut self, pc: u32, sp: u32) {
        if self.light { return; }
        let n = self.frames.len();
        if let Some(i) = (n.saturating_sub(4)..n).rev().find(|&i| !self.frames[i].irq && self.frames[i].ret == pc && self.frames[i].sp == sp) {
            self.frames.truncate(i);
        }
        self.prune(sp);
    }

    /// Interrupt or exception entry: `ret` is the PC it returns to, `sp` the stack pointer before the entry.
    pub fn interrupt(&mut self, entry: u32, ret: u32, sp: u32) {
        if self.light { return; }
        self.irq_depth += 1;
        if self.frames.len() >= MAX_FRAMES { self.frames.drain(..MAX_FRAMES / 2); }
        self.frames.push(Frame { entry, ret, sp, irq: true, ..Default::default() });
    }

    /// After RTE.
    pub fn rte(&mut self, pc: u32, sp: u32) {
        if self.light { return; }
        self.irq_depth = self.irq_depth.saturating_sub(1);
        let n = self.frames.len();
        if let Some(i) = (n.saturating_sub(4)..n).rev().find(|&i| self.frames[i].irq && self.frames[i].ret == pc && self.frames[i].sp == sp) {
            self.frames.truncate(i);
        }
        self.prune(sp);
    }

    /// A taken branch back to `to` (not a call or return), with stack pointer `sp`.
    #[inline]
    pub fn back(&mut self, to: u32, sp: u32) {
        if !self.loops { return; }
        let e = &mut self.table[((to >> 1) as usize) & (LOOP_TABLE - 1)];
        if e.0 == to && e.1 == sp { e.2 += 1; } else { *e = (to, sp, 1); }
        if e.2 >= LOOP_TRIGGER && self.loop_event.is_none() { e.2 = 0; self.loop_event = Some((to, sp)); self.event = true; }
    }

    /// Make the loop (to, sp) wait `times` x longer before it is offered again.
    pub fn back_off(&mut self, to: u32, sp: u32, times: i32) {
        let e = &mut self.table[((to >> 1) as usize) & (LOOP_TABLE - 1)];
        if e.0 == to && e.1 == sp { e.2 = -LOOP_TRIGGER.saturating_mul(times); }
    }

    /// Live call frames from the innermost outwards, stopping at an interrupt entry.
    pub fn task_frames(&self) -> impl Iterator<Item = (usize, &Frame)> {
        self.frames.iter().enumerate().rev().take_while(|(_, f)| !f.irq)
    }
}

/// The CPU registers a call records (borrowed from the CPU while its trace is borrowed mutably).
pub struct CallRegs<'a> { pub r: &'a [u32; 16], pub fr: &'a [u32; 16], pub pc: u32, pub pr: u32, pub sr: u32, pub gbr: u32, pub mach: u32, pub macl: u32, pub fpscr: u32 }

// ---------- argument layouts of the RGAH drawing calls (as `gfx` reads them) ----------

/// An aligned, non-null word of plain memory.
fn rd(mem: &Memory, a: u32) -> Option<u32> { if a == 0 || a & 3 != 0 { return None; } mem.read_plain(a, 4).map(|v| v as u32) }
fn words<const N: usize>(mem: &Memory, p: u32) -> Option<[u32; N]> {
    let mut w = [0; N];
    for (i, x) in w.iter_mut().enumerate() { *x = rd(mem, p.wrapping_add(4 * i as u32))?; }
    Some(w)
}
fn readable(mem: &Memory, a: u32, n: u32) -> bool { a != 0 && n > 0 && mem.slice(a, n as usize).is_some() }
fn in_ram(a: u32, n: u32) -> bool {
    use crate::mem::{region, Region};
    matches!((region(a), region(a.wrapping_add(n - 1))), (Region::Sdram(_), Region::Sdram(_)) | (Region::Oram(_), Region::Oram(_)))
}
fn coord(v: u32) -> bool { (-8192..=8192).contains(&(v as i32)) }
fn extent(v: u32) -> bool { v <= 4096 }

/// Cheap first test of a bitmap descriptor (size words).
#[inline]
fn looks_like_bitmap(mem: &Memory, p: u32) -> bool {
    p & 3 == 0 && matches!((mem.read_plain(p, 4), mem.read_plain(p.wrapping_add(4), 4)), (Some(w), Some(h)) if (1..=4096).contains(&w) && (1..=4096).contains(&h))
}

/// A bitmap descriptor: size, a clip area inside it, a known pixel format and readable pixels.
pub fn bitmap(mem: &Memory, p: u32) -> bool { bitmap_at(mem, p).is_some() }

/// A bitmap that can be drawn into: its pixels are in RAM.
pub fn target(mem: &Memory, p: u32) -> bool { bitmap_at(mem, p).is_some_and(|(a, n)| in_ram(a, n)) }

/// Pixel address and size of a valid bitmap descriptor.
fn bitmap_at(mem: &Memory, p: u32) -> Option<(u32, u32)> {
    let w = words::<8>(mem, p)?;
    if !(1..=4096).contains(&w[0]) || !(1..=4096).contains(&w[1]) || w[6] > 5 { return None; }
    let (cx, cy, cw, ch) = (w[2] as i32, w[3] as i32, w[4] as i32, w[5] as i32);
    if cx < 0 || cy < 0 || cw < 0 || ch < 0 || cx + cw > w[0] as i32 || cy + ch > w[1] as i32 { return None; }
    let n = w[0] * w[1] * if w[6] <= 2 { 2 } else { 4 };
    readable(mem, w[7], n).then_some((w[7], n))
}
fn point(mem: &Memory, p: u32) -> bool { words::<2>(mem, p).is_some_and(|q| coord(q[0]) && coord(q[1])) }
/// An effect record: only its blend mode is read by every call (the other fields may be left uninitialised).
fn effect(mem: &Memory, p: u32) -> bool { words::<4>(mem, p).is_some_and(|e| e[2] <= 6) }
fn rect8(mem: &Memory, p: u32) -> bool {
    words::<8>(mem, p).is_some_and(|q| coord(q[0]) && coord(q[1]) && extent(q[2]) && extent(q[3]) && coord(q[4]) && coord(q[5]) && extent(q[6]) && extent(q[7]))
}
fn matrix(mem: &Memory, p: u32) -> bool {
    let Some(m) = words::<10>(mem, p) else { return false };
    m[..9].iter().all(|&v| { let f = f32::from_bits(v); f.is_finite() && f.abs() < 1.0e6 }) && f32::from_bits(m[8]) != 0.0 && bitmap(mem, m[9])
}
fn mask(mem: &Memory, p: u32) -> bool { words::<4>(mem, p).is_some_and(|q| (1..=4096).contains(&q[1]) && (1..=4096).contains(&q[2]) && readable(mem, q[3], q[1] * q[2])) }

/// Whether arguments `r` (R4..R7) and stack words `stk` read as a call to the drawing call `f`.
pub fn layout_matches(mem: &Memory, f: RgahFn, r: [u32; 4], stk: [u32; 3]) -> bool {
    match f {
        RgahFn::FillRect => target(mem, r[0]) && point(mem, r[1]) && extent(r[2]) && extent(r[3]) && stk[1] <= 255 && effect(mem, stk[2]),
        RgahFn::DrawImage => target(mem, r[0]) && matrix(mem, r[1]) && point(mem, r[2]) && r[3] <= 255 && effect(mem, stk[0]),
        RgahFn::BitBlt => target(mem, r[0]) && bitmap(mem, r[1]) && rect8(mem, r[2]) && r[3] <= 255 && effect(mem, stk[0]),
        RgahFn::BitBltFixedColor => target(mem, r[0]) && mask(mem, r[1]) && rect8(mem, r[2]),
        RgahFn::Finish => false,
    }
}

/// The four drawing calls.
pub const DRAW: [RgahFn; 4] = [RgahFn::FillRect, RgahFn::DrawImage, RgahFn::BitBlt, RgahFn::BitBltFixedColor];

/// How many drawing-call layouts these arguments match, and the last one matched.
pub fn classify(mem: &Memory, r: [u32; 4], stk: [u32; 3]) -> (usize, Option<RgahFn>) {
    let mut n = 0;
    let mut hit = None;
    for f in DRAW { if layout_matches(mem, f, r, stk) { n += 1; hit = Some(f); } }
    (n, hit)
}

// ---------- detection state ----------

/// What has been detected and when (P0 clock count). Kept in saved states.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Found {
    /// Entry points by `RgahFn` index (fillRect, drawImage, bitBlt, bitBltFixedColor, finish).
    pub rgah: [Option<u32>; 5],
    pub rgah_at: [u64; 5],
    pub idle_loop: Option<u32>,
    pub idle_at: u64,
    /// Library code range learned from the identified calls.
    pub range: Option<(u32, u32)>,
    /// Graphics-core register the first identified drawing call wrote first.
    pub port: Option<u32>,
    /// Entries withdrawn after a mismatching call (never offered again).
    pub rejected: Vec<u32>,
}

/// What the machine must do after a graphics-core access.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Unwind frame `index` (identified as `f`) to its caller and complete the call in software.
    Unwind { index: usize, f: RgahFn },
}

/// Idle-loop verification in progress (instruction by instruction).
#[derive(Clone, Debug)]
pub struct Verify {
    pub to: u32,
    pub sp: u32,
    started: bool,
    base: usize,
    hash: u64,
    len: u32,
    /// Basic-block leaders at the loop's level before its first call, and that call.
    leaders: Vec<u32>,
    first_call: Option<u32>,
    prev: Option<(u64, u32, u32)>,
    same: u32,
    changes: u32,
    steps: u32,
    mmio: u64,
    irqs: u64,
}

#[derive(Clone, Debug, Default)]
pub struct Detector {
    pub found: Found,
    pub gpu_seen: u64,
    pub verify: Option<Verify>,
    /// Confirmed idle periods: (head, interrupt count at confirmation).
    episodes: Vec<(u32, u64)>,
    /// Remaining probation calls per hooked drawing entry.
    pub probation: Vec<(u32, RgahFn, u32)>,
    /// Calls already tried speculatively without result.
    pub spec_rejected: Vec<u32>,
    /// Entries reported as left unidentified.
    unsure: Vec<u32>,
    pub log: Vec<String>,
}

const PROBATION: u32 = 32;
const VERIFY_ITER: u32 = 4;
const EPISODES: usize = 3;
const VERIFY_BUDGET: u32 = 20_000;
const MAX_ITER_LEN: u32 = 5_000;
const FNV: u64 = 0xCBF2_9CE4_8422_2325;

/// W53GA_DETECT_DEBUG set: describe what detection sees (graphics-core accesses, calls into the library).
pub fn debug() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("W53GA_DETECT_DEBUG").is_some())
}
const P0_HZ: f64 = 100.0e6 / 3.0;

fn fold(h: u64, v: u32) -> u64 { (h ^ v as u64).wrapping_mul(0x0000_0100_0000_01B3) }

impl Detector {
    pub fn new(found: Found) -> Detector { Detector { found, ..Default::default() } }

    /// Hooks for the machine from what has been found so far.
    pub fn hooks(&self) -> crate::hle::Hooks {
        let mut rgah: Vec<(u32, u8)> = self.found.rgah.iter().enumerate().filter_map(|(k, e)| e.map(|a| (a, k as u8))).collect();
        rgah.sort();
        let rgah_range = if rgah.is_empty() { None } else { Some((rgah[0].0, rgah[rgah.len() - 1].0)) };
        crate::hle::Hooks { idle_loop: self.found.idle_loop, rgah, rgah_range, profile: Some("detected at run time".into()) }
    }

    /// A trace for the CPU that continues from what has been found.
    pub fn trace(&self) -> Trace {
        let mut t = Trace::new(self.found.idle_loop.is_none());
        self.sync(&mut t);
        t
    }

    /// Bring a trace up to date with what has been found.
    pub fn sync(&self, t: &mut Trace) {
        t.range = self.found.range;
        t.known = self.found.rgah.iter().flatten().copied().collect();
        t.probe_calls = self.found.range.is_none();
        t.loops = self.found.idle_loop.is_none();
        let light = self.light();
        if light && !t.light { t.frames.clear(); t.irq_depth = 0; }
        t.light = light;
    }

    /// The library, `finish` and the idle loop are known: the trace only needs to watch calls into the library.
    pub fn light(&self) -> bool {
        self.found.range.is_some() && self.found.rgah[RgahFn::Finish as usize].is_some() && self.found.idle_loop.is_some()
    }

    /// Whether the call on top of the trace should be explored speculatively: its arguments read as exactly one drawing
    /// call not found yet, and it has not been tried before.
    pub fn candidate(&self, trace: &Trace, mem: &Memory) -> Option<u32> {
        let f = trace.frames.last()?;
        if f.irq || self.spec_rejected.contains(&f.entry) || self.found.rejected.contains(&f.entry) || trace.known.contains(&f.entry) { return None; }
        match classify(mem, f.args, f.stk) {
            (1, Some(d)) if self.found.rgah[d as usize].is_none() => Some(f.entry),
            _ => None,
        }
    }

    /// Report once per entry that a call was left unidentified.
    fn unsure(&mut self, entry: u32, steps: u64, why: &str) {
        if self.unsure.contains(&entry) { return; }
        self.unsure.push(entry);
        self.note(steps, format!("not identifying the call at {entry:#x}: {why}"));
    }

    fn note(&mut self, steps: u64, msg: String) {
        let line = format!("t={:.3}s {msg}", steps as f64 / P0_HZ);
        eprintln!("[hle] {line}");
        self.log.push(line);
    }

    fn learn(&mut self, trace: Option<&mut Trace>, f: RgahFn, entry: u32, span: (u32, u32), steps: u64, how: &str) {
        self.found.rgah[f as usize] = Some(entry);
        self.found.rgah_at[f as usize] = steps;
        let r = match self.found.range { Some((lo, hi)) => (lo.min(span.0), hi.max(span.1)), None => span };
        self.found.range = Some(r);
        if let Some(t) = trace { t.range = Some(r); t.known.push(entry); }
        if f != RgahFn::Finish { self.probation.push((entry, f, PROBATION)); }
        self.note(steps, format!("detected RGAH_{} at {entry:#x} ({how})", f.name()));
    }

    /// A graphics-core access (`last`: register address, write) was just made at task or interrupt level.
    /// `speculative`: the access happened while exploring a call ahead (nothing is unwound then).
    pub fn on_gpu(&mut self, trace: &mut Trace, mem: &Memory, last: (u32, bool), steps: u64, speculative: bool) -> Option<Action> {
        let how = if speculative { "explored ahead from its caller, which then ran it with the hook in place" } else { "at its first graphics-core access; that call completed in software" };
        if trace.irq_depth > 0 { return None; }
        let chain: Vec<(usize, Frame)> = trace.task_frames().take(16).map(|(i, f)| (i, *f)).collect();
        // stop at a frame from another stack (stale after a task switch)
        let chain: Vec<(usize, Frame)> = chain.iter().enumerate().take_while(|(k, (_, f))| *k == 0 || f.sp.wrapping_sub(chain[k - 1].1.sp) < 0x1_0000).map(|(_, x)| *x).collect();
        if debug() {
            let s: Vec<String> = chain.iter().map(|(_, f)| format!("{:x}{:x?}{:x?}", f.entry, f.args, f.stk)).collect();
            eprintln!("[detect] {} {:x} by {}", if last.1 { "write" } else { "read" }, last.0, s.join(" <- "));
        }
        let first_touch: Vec<bool> = chain.iter().map(|(_, f)| !f.touched).collect();
        for (i, _) in &chain { trace.frames[*i].touched = true; }
        // An access made inside a call already identified: the frames below it are library code too.
        if let Some(k) = chain.iter().position(|(_, f)| trace.known.contains(&f.entry)) {
            if let Some((lo, hi)) = self.found.range {
                let r = chain[..k].iter().fold((lo, hi), |(lo, hi), (_, f)| (lo.min(f.entry), hi.max(f.entry)));
                self.found.range = Some(r);
                trace.range = Some(r);
            }
            return None;
        }
        // A drawing call: the innermost frame whose arguments read as exactly one drawing call.
        for (k, (i, f)) in chain.iter().enumerate() {
            if self.found.rejected.contains(&f.entry) { continue; }
            let (n, hit) = classify(mem, f.args, f.stk);
            if n > 1 { self.unsure(f.entry, steps, "its arguments match more than one drawing call"); return None; }
            let Some(d) = hit else { continue };
            if self.found.rgah[d as usize].is_some() { self.unsure(f.entry, steps, "it reads as a drawing call already found elsewhere"); return None; }
            let lo = chain[..=k].iter().map(|(_, g)| g.entry).min().unwrap();
            let hi = chain[..=k].iter().map(|(_, g)| g.entry).max().unwrap();
            if self.found.port.is_none() && last.1 && first_touch[0] { self.found.port = Some(last.0); }
            self.learn(Some(trace), d, f.entry, (lo, hi), steps, how);
            return Some(Action::Unwind { index: *i, f: d });
        }
        // finish: the outermost library frame, called from outside the library, matching no drawing call, whose first
        // graphics-core access is a write to the command port.
        let (Some((lo, hi)), Some(port)) = (self.found.range, self.found.port) else { return None };
        if self.found.rgah[RgahFn::Finish as usize].is_some() || !(last.1 && last.0 == port) { return None; }
        let inside = |a: u32| (lo..=hi).contains(&a);
        let k = chain.iter().take_while(|(_, f)| inside(f.entry)).count();
        if k == 0 { return None; }
        let (i, f) = chain[k - 1];
        if inside(f.ret) || !first_touch[k - 1] || trace.known.contains(&f.entry) || self.found.rejected.contains(&f.entry) { return None; }
        if classify(mem, f.args, f.stk).0 != 0 { return None; }
        self.learn(Some(trace), RgahFn::Finish, f.entry, (f.entry, f.entry), steps, how);
        Some(Action::Unwind { index: i, f: RgahFn::Finish })
    }

    /// A call from outside the library to an unknown entry inside it (the top frame): identify it before it runs.
    pub fn on_entry(&mut self, trace: &mut Trace, mem: &Memory, steps: u64) -> bool {
        let Some(f) = trace.frames.last().copied() else { return false };
        self.entry_call(Some(trace), &f, mem, steps)
    }

    /// Identify the call `f` (just entered, nothing of it run) if its arguments read as exactly one drawing call not
    /// found yet. True if it was identified.
    pub fn entry_call(&mut self, trace: Option<&mut Trace>, f: &Frame, mem: &Memory, steps: u64) -> bool {
        if self.found.rejected.contains(&f.entry) || self.found.rgah.contains(&Some(f.entry)) { return false; }
        if debug() { eprintln!("[detect] call into the library: {:x} from {:x} args {:x?} {:x?}", f.entry, f.ret, f.args, f.stk); }
        let (n, hit) = classify(mem, f.args, f.stk);
        let Some(d) = hit else { return false };
        if n != 1 { self.unsure(f.entry, steps, "its arguments match more than one drawing call"); return false; }
        if self.found.rgah[d as usize].is_some() { self.unsure(f.entry, steps, "it reads as a drawing call already found elsewhere"); return false; }
        self.learn(trace, d, f.entry, (f.entry, f.entry), steps, "on entry, before it ran");
        true
    }

    /// A hooked drawing call at `pc`: check its arguments while on probation. False if the hook was withdrawn.
    pub fn probe(&mut self, trace: Option<&mut Trace>, mem: &Memory, pc: u32, r: [u32; 4], sp: u32, steps: u64) -> bool {
        let Some(p) = self.probation.iter().position(|e| e.0 == pc) else { return true };
        let rdw = |a: u32| mem.read_plain(a, 4).map_or(0, |v| v as u32);
        let stk = [rdw(sp), rdw(sp.wrapping_add(4)), rdw(sp.wrapping_add(8))];
        let f = self.probation[p].1;
        if layout_matches(mem, f, r, stk) {
            self.probation[p].2 -= 1;
            if self.probation[p].2 == 0 { self.probation.remove(p); }
            return true;
        }
        self.probation.remove(p);
        self.found.rgah[f as usize] = None;
        self.found.rejected.push(pc);
        if let Some(t) = trace { t.known.retain(|&e| e != pc); }
        self.note(steps, format!("withdrew RGAH_{} at {pc:#x}: a later call does not match its argument layout", f.name()));
        false
    }

    // ---------- idle loop ----------

    pub fn start_verify(&mut self, to: u32, sp: u32) {
        self.verify = Some(Verify { to, sp, started: false, base: 0, hash: FNV, len: 0, leaders: Vec::new(), first_call: None, prev: None, same: 0, changes: 0, steps: 0, mmio: 0, irqs: 0 });
    }

    /// One instruction of a verification, after it executed at `cur` (`op` its opcode). `mmio` and `irqs` count MMIO
    /// accesses and accepted interrupts. False when the verification is over.
    pub fn verify_step(&mut self, trace: &mut Trace, cpu: &Cpu, cur: u32, op: u16, mmio: u64, irqs: u64, steps: u64) -> bool {
        let depth = trace.frames.len();
        let Some(v) = self.verify.as_mut() else { return false };
        v.steps += 1;
        if v.steps > VERIFY_BUDGET { return self.verify_fail(trace, 4); }
        let at_head = cpu.pc == v.to && cpu.r[15] == v.sp;
        let restart = |v: &mut Verify| { v.hash = FNV; v.len = 0; v.leaders.clear(); v.leaders.push(v.to); v.first_call = None; };
        if !v.started {
            if at_head && cpu.imask() == 0 { v.started = true; v.base = depth; v.mmio = mmio; v.irqs = irqs; restart(v); }
            return true;
        }
        if irqs != v.irqs { v.started = false; v.prev = None; v.same = 0; v.changes = 0; return true; } // interrupted: wait for the next pass
        if mmio != v.mmio { return self.verify_fail(trace, 64); } // polls hardware: not idle
        let mut h = fold(v.hash, cpu.pc);
        for x in cpu.r { h = fold(h, x); }
        for x in [cpu.sr, cpu.pr, cpu.gbr, cpu.mach, cpu.macl, cpu.fpscr, cpu.fpul] { h = fold(h, x); }
        for x in cpu.fr { h = fold(h, x); }
        v.hash = h;
        v.len += 1;
        if v.len > MAX_ITER_LEN || depth < v.base { return self.verify_fail(trace, 16); }
        if depth == v.base && v.first_call.is_none() {
            let sequential = cpu.pc == cur.wrapping_add(2) || cpu.pc == cur.wrapping_add(4);
            let conditional = matches!(op >> 8, 0x89 | 0x8B | 0x8D | 0x8F);
            if !sequential || conditional { v.leaders.push(cpu.pc); }
        }
        if depth > v.base && v.first_call.is_none() { v.first_call = Some(cur); }
        if !(at_head && depth == v.base) { return true; }
        if cpu.imask() != 0 { return self.verify_fail(trace, 16); }
        let head = match v.first_call { Some(c) => v.leaders.iter().copied().filter(|&l| l <= c).max().unwrap_or(v.to), None => v.to };
        let it = (v.hash, v.len, head);
        if v.prev == Some(it) { v.same += 1; } else { v.same = 0; v.changes += 1; }
        v.prev = Some(it);
        restart(v);
        // the first pass after arriving may differ; a loop whose state keeps changing is busy, not idle
        if v.changes > 2 { return self.verify_fail(trace, 16); }
        if v.same + 1 < VERIFY_ITER { return true; }
        let (to, sp) = (v.to, v.sp);
        self.verify = None;
        trace.back_off(to, sp, 1);
        if self.episodes.last().is_some_and(|e| e.1 == irqs) { return false; }
        if self.episodes.iter().any(|e| e.0 != head) { self.episodes.clear(); }
        self.episodes.push((head, irqs));
        if self.episodes.len() >= EPISODES {
            self.found.idle_loop = Some(head);
            self.found.idle_at = steps;
            trace.loops = false;
            self.note(steps, format!("detected the RTOS idle loop at {head:#x} (loop branch target {to:#x}; {VERIFY_ITER} identical iterations in each of {EPISODES} idle periods)"));
        }
        false
    }

    fn verify_fail(&mut self, trace: &mut Trace, backoff: i32) -> bool {
        if let Some(v) = self.verify.take() { trace.back_off(v.to, v.sp, backoff); }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mem::{Memory, FLASH_SIZE};

    const RAM: u32 = 0x1C00_0000;

    fn mem() -> Memory { Memory::new(vec![0; FLASH_SIZE]) }
    fn put(m: &mut Memory, a: u32, words: &[u32]) { for (i, w) in words.iter().enumerate() { assert!(m.store_bytes(a + 4 * i as u32, &w.to_be_bytes())); } }

    /// A 16x8 RGB565 bitmap descriptor at `at` with pixels at `px`.
    fn bmp(m: &mut Memory, at: u32, px: u32) { put(m, at, &[16, 8, 0, 0, 16, 8, 0, px]); }

    /// Memory with one argument set for each drawing call.
    fn scene() -> Memory {
        let mut m = mem();
        bmp(&mut m, RAM, RAM + 0x1000);
        bmp(&mut m, RAM + 0x40, RAM + 0x2000);
        put(&mut m, RAM + 0x80, &[2, 3]); // point
        put(&mut m, RAM + 0xA0, &[0, 0, 1, 0]); // effect: src-over
        put(&mut m, RAM + 0xC0, &[0, 0, 4, 4, 1, 1, 4, 4]); // rectangle
        put(&mut m, RAM + 0x100, &[1f32.to_bits(), 0, 0, 0, 1f32.to_bits(), 0, 0, 0, 1f32.to_bits(), RAM + 0x40]); // matrix + source
        put(&mut m, RAM + 0x140, &[0xFFFF_FFFF, 8, 8, RAM + 0x3000]); // glyph mask
        put(&mut m, 0x1C00_7F00, &[0xFF00_0000, 255, RAM + 0xA0]); // stack arguments of a fillRect
        m
    }

    #[test]
    fn layouts_identify_each_drawing_call() {
        let m = scene();
        let eff = RAM + 0xA0;
        assert_eq!(classify(&m, [RAM, RAM + 0x80, 4, 4], [0xFF00_0000, 255, eff]), (1, Some(RgahFn::FillRect)));
        assert_eq!(classify(&m, [RAM, RAM + 0x100, RAM + 0x80, 255], [eff, 0, 0]), (1, Some(RgahFn::DrawImage)));
        assert_eq!(classify(&m, [RAM, RAM + 0x40, RAM + 0xC0, 255], [eff, 0, 0]), (1, Some(RgahFn::BitBlt)));
        assert_eq!(classify(&m, [RAM, RAM + 0x140, RAM + 0xC0, 0], [0, 0, 0]), (1, Some(RgahFn::BitBltFixedColor)));
    }

    #[test]
    fn layouts_reject_other_arguments() {
        let mut m = scene();
        let eff = RAM + 0xA0;
        for r in [[0, 0, 0, 0], [1, 2, 3, 4], [RAM + 2, RAM + 0x80, 4, 4], [RAM, 0, 0xFF, RAM + 0x80], [0x0012_3456, RAM, 4, 4], [RAM, RAM + 0x80, RAM + 0x80, 4]] {
            assert_eq!(classify(&m, r, [0, 255, eff]).0, 0, "{r:x?}");
        }
        put(&mut m, eff, &[0, 0, 9, 0]); // blend mode out of range
        assert_eq!(classify(&m, [RAM, RAM + 0x80, 4, 4], [0, 255, eff]).0, 0);
        put(&mut m, RAM + 0x200, &[16, 8, 4, 0, 16, 8, 0, RAM + 0x1000]); // clip area outside the bitmap
        assert!(!bitmap(&m, RAM + 0x200));
        put(&mut m, RAM + 0x200, &[16, 8, 0, 0, 16, 8, 0, 0x8000_0000]); // pixels not in memory
        assert!(!bitmap(&m, RAM + 0x200));
        put(&mut m, RAM + 0x200, &[16, 8, 0, 0, 16, 8, 7, RAM + 0x1000]); // unknown pixel format
        assert!(!bitmap(&m, RAM + 0x200));
        put(&mut m, RAM + 0x200, &[16, 8, 0, 0, 16, 8, 0, 0x0010_0000]); // pixels in flash: a source, not a target
        assert!(bitmap(&m, RAM + 0x200) && !target(&m, RAM + 0x200));
    }

    fn regs(sp: u32) -> [u32; 16] { let mut r = [0; 16]; r[15] = sp; for (i, x) in r.iter_mut().enumerate().take(15).skip(8) { *x = 0x100 + i as u32; } r }
    const FR: [u32; 16] = [0; 16];
    fn call(t: &mut Trace, m: &Memory, r: &[u32; 16], pc: u32, pr: u32) { t.call(&CallRegs { r, fr: &FR, pc, pr, sr: 0, gbr: 0, mach: 0, macl: 0, fpscr: 0 }, m); }

    #[test]
    fn shadow_stack_follows_calls_returns_and_interrupts() {
        let m = mem();
        let mut t = Trace::new(false);
        call(&mut t, &m, &regs(0x1C00_8000), 0x1000, 0x2004);
        call(&mut t, &m, &regs(0x1C00_7F00), 0x3000, 0x1010);
        t.interrupt(0x4000, 0x3008, 0x1C00_7E00);
        assert_eq!(t.irq_depth, 1);
        assert_eq!(t.task_frames().count(), 0, "an interrupt hides the task's frames");
        call(&mut t, &m, &regs(0x1C00_7D00), 0x5000, 0x4010);
        t.ret(0x4010, 0x1C00_7D00);
        t.rte(0x3008, 0x1C00_7E00);
        assert_eq!(t.irq_depth, 0);
        assert_eq!(t.task_frames().map(|(_, f)| f.entry).collect::<Vec<_>>(), vec![0x3000, 0x1000]);
        assert_eq!(t.frames[1].saved[0], 0x108, "callee-saved registers are kept");
        t.ret(0x1010, 0x1C00_7F00);
        assert_eq!(t.frames.len(), 1);
        // a return that skips frames (longjmp, task switch) drops the dead ones
        call(&mut t, &m, &regs(0x1C00_7F00), 0x3000, 0x1010);
        t.ret(0x2004, 0x1C00_8000);
        assert!(t.frames.is_empty());
    }

    #[test]
    fn calls_into_the_learned_library_are_offered_once() {
        let m = mem();
        let mut t = Trace::new(false);
        t.range = Some((0x8000, 0x9000));
        call(&mut t, &m, &regs(0x1C00_8000), 0x8100, 0x8204);
        assert!(!t.entry_event, "a call from inside the library is internal");
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8100, 0x1004);
        assert!(t.entry_event);
        t.entry_event = false;
        t.known.push(0x8100);
        call(&mut t, &m, &regs(0x1C00_7E00), 0x8100, 0x1004);
        assert!(!t.entry_event, "known entries are not offered again");
    }

    fn fill_args(sp: u32) -> [u32; 16] { let mut r = regs(sp); r[4] = RAM; r[5] = RAM + 0x80; r[6] = 4; r[7] = 4; r }

    #[test]
    fn graphics_access_identifies_the_innermost_matching_frame() {
        let m = scene();
        let mut t = Trace::new(false);
        let r = fill_args(0x1C00_7F00);
        call(&mut t, &m, &r, 0x5000, 0x1004); // application wrapper with the same arguments
        call(&mut t, &m, &r, 0x8100, 0x5010); // the API entry
        let mut inner = regs(0x1C00_7E00);
        inner[4] = RAM; inner[5] = 0; inner[6] = 0xFF; inner[7] = 0x1C00_7F80;
        call(&mut t, &m, &inner, 0x8800, 0x8120); // a driver function
        let mut d = Detector::default();
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4000, true), 0, false), Some(Action::Unwind { index: 1, f: RgahFn::FillRect }));
        assert_eq!(d.found.rgah[0], Some(0x8100));
        assert_eq!(d.found.range, Some((0x8100, 0x8800)));
        assert_eq!(d.found.port, Some(0xE810_4000));
        assert_eq!(d.hooks().rgah, vec![(0x8100, 0)]);
        // inside an interrupt nothing is identified
        let mut t = Trace::new(false);
        call(&mut t, &m, &r, 0x8200, 0x5010);
        t.interrupt(0x4000, 0x8208, 0x1C00_7D00);
        let mut d = Detector::default();
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4000, true), 0, false), None);
    }

    #[test]
    fn a_later_drawing_call_is_recognised_on_entry() {
        let m = scene();
        let mut d = Detector::default();
        let mut t = Trace::new(false);
        d.learn(Some(&mut t), RgahFn::FillRect, 0x8100, (0x8100, 0x8800), 0, "test");
        let mut r = regs(0x1C00_7F00);
        r[4] = RAM; r[5] = RAM + 0x40; r[6] = RAM + 0xC0; r[7] = 255;
        let mut m2 = m;
        put(&mut m2, 0x1C00_7F00, &[RAM + 0xA0, 0, 0]);
        call(&mut t, &m2, &r, 0x8300, 0x1004);
        assert!(t.entry_event);
        assert!(d.on_entry(&mut t, &m2, 0));
        assert_eq!(d.found.rgah[2], Some(0x8300));
        // the same arguments to another entry: bitBlt is known, nothing more is learned
        call(&mut t, &m2, &r, 0x8400, 0x1004);
        assert!(!d.on_entry(&mut t, &m2, 0));
    }

    #[test]
    fn light_mode_only_records_calls_into_the_library() {
        let m = mem();
        let mut d = Detector::default();
        (d.found.range, d.found.rgah[0], d.found.rgah[4], d.found.idle_loop) = (Some((0x8100, 0x8800)), Some(0x8100), Some(0x8700), Some(0x900));
        let mut t = d.trace();
        assert!(t.light && !t.loops && !t.probe_calls);
        call(&mut t, &m, &regs(0x1C00_8000), 0x5000, 0x1004);
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8300, 0x8104);
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8100, 0x5010);
        assert!(t.frames.is_empty() && !t.entry_event, "other calls, internal calls and known entries are not recorded");
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8300, 0x5010);
        assert_eq!(t.frames.len(), 1);
        assert!(t.entry_event);
        t.interrupt(0x4000, 0x8302, 0x1C00_7E00);
        assert_eq!(t.irq_depth, 0);
    }

    #[test]
    fn finish_is_the_outermost_library_frame_writing_the_command_port() {
        let m = mem();
        let mut d = Detector::default();
        d.found.rgah[0] = Some(0x8100);
        d.found.range = Some((0x8100, 0x8800));
        d.found.port = Some(0xE810_4000);
        let mut t = d.trace();
        call(&mut t, &m, &regs(0x1C00_8000), 0x5000, 0x1004);
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8400, 0x5010);
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4004, true), 0, false), None, "a write elsewhere (initialisation) is not finish");
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4000, true), 0, false), None, "only the first access of a call counts");
        let mut t = d.trace();
        call(&mut t, &m, &regs(0x1C00_8000), 0x5000, 0x1004);
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8400, 0x5010);
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4000, true), 0, false), Some(Action::Unwind { index: 1, f: RgahFn::Finish }));
        assert_eq!(d.found.rgah[4], Some(0x8400));
    }

    #[test]
    fn accesses_inside_an_identified_call_are_never_finish() {
        let m = mem();
        let mut d = Detector::default();
        (d.found.rgah[0], d.found.range, d.found.port) = (Some(0x8100), Some((0x8100, 0x8800)), Some(0xE810_4000));
        let mut t = d.trace();
        call(&mut t, &m, &regs(0x1C00_8000), 0x5000, 0x1004); // application
        call(&mut t, &m, &regs(0x1C00_7F00), 0x8100, 0x5010); // the identified drawing call
        call(&mut t, &m, &regs(0x1C00_7E00), 0x8A00, 0x8120); // a driver function beyond the learned range
        call(&mut t, &m, &regs(0x1C00_7D00), 0x8400, 0x8A10); // a helper inside the range, "called from outside"
        assert_eq!(d.on_gpu(&mut t, &m, (0xE810_4000, true), 0, true), None);
        assert_eq!(d.found.rgah[4], None);
        assert_eq!(d.found.range, Some((0x8100, 0x8A00)), "the driver's frames widen the library range");
    }

    #[test]
    fn probation_withdraws_a_hook_whose_calls_stop_matching() {
        let m = scene();
        let mut d = Detector::default();
        let mut t = Trace::new(false);
        d.learn(Some(&mut t), RgahFn::FillRect, 0x8100, (0x8100, 0x8100), 0, "test");
        assert!(d.probe(Some(&mut t), &m, 0x8100, [RAM, RAM + 0x80, 4, 4], 0x1C00_7F00, 0));
        assert!(!d.probe(Some(&mut t), &m, 0x8100, [7, 7, 7, 7], 0x1C00_7F00, 0));
        assert_eq!(d.found.rgah[0], None);
        assert!(d.found.rejected.contains(&0x8100));
        assert!(d.hooks().rgah.is_empty());
        assert!(!t.known.contains(&0x8100));
    }

    #[test]
    fn backward_branches_offer_a_loop_after_many_iterations() {
        let mut t = Trace::new(true);
        for _ in 0..LOOP_TRIGGER - 1 { t.back(0x900, 0x1C00_7000); }
        assert_eq!(t.loop_event, None);
        t.back(0x900, 0x1C00_7000);
        assert_eq!(t.loop_event, Some((0x900, 0x1C00_7000)));
        let mut t = Trace::new(true);
        for i in 0..LOOP_TRIGGER * 2 { t.back(0x900, 0x1C00_7000 - (i as u32 & 1) * 4); }
        assert_eq!(t.loop_event, None, "a different stack pointer is a different loop");
    }

    /// Bus for the idle-loop tests: one device register that counts reads.
    #[derive(Default)]
    struct Io { reads: u64 }
    impl crate::cpu::Mmio for Io {
        fn read(&mut self, _pc: u32, _a: u32, _n: usize) -> u64 { self.reads += 1; self.reads }
        fn write(&mut self, _pc: u32, _a: u32, _n: usize, _v: u64) {}
        fn ibnr(&mut self) -> u16 { 0 }
        fn set_ibnr(&mut self, _v: u16) {}
    }

    /// Run `code` (at 0x1000; a subroutine `sub` at 0x1100) with detection, simulating an interrupt between idle
    /// periods. Returns the detected idle loop.
    fn idle_of(code: &[u16], sub: &[u16], steps: usize) -> Option<u32> {
        let mut flash = vec![0u8; FLASH_SIZE];
        for (i, w) in code.iter().enumerate() { flash[0x1000 + 2 * i..0x1002 + 2 * i].copy_from_slice(&w.to_be_bytes()); }
        for (i, w) in sub.iter().enumerate() { flash[0x1100 + 2 * i..0x1102 + 2 * i].copy_from_slice(&w.to_be_bytes()); }
        let mut cpu = Cpu::new(Memory::new(flash));
        (cpu.pc, cpu.sr, cpu.r[15], cpu.r[12], cpu.r[11]) = (0x1000, 0, 0x1C00_8000, 0x1100, 0xE000_0000);
        cpu.trace = Some(Box::new(Trace::new(true)));
        let mut io = Io::default();
        let mut d = Detector::default();
        let mut irqs = 0;
        for _ in 0..steps {
            let pc0 = cpu.pc;
            let op = cpu.mem.fetch16(pc0).unwrap();
            cpu.step(&mut io).expect("runs");
            let mut t = cpu.trace.take().unwrap();
            if let Some((to, sp)) = t.loop_event.take() { if d.verify.is_none() { d.start_verify(to, sp); } }
            if d.verify.is_some() && !d.verify_step(&mut t, &cpu, pc0, op, io.reads, irqs, 0) { irqs += 1; }
            cpu.trace = Some(t);
            if d.found.idle_loop.is_some() { break; }
        }
        d.found.idle_loop
    }

    #[test]
    fn an_idle_loop_is_found_at_the_block_that_calls_the_kernel() {
        // 1000 mov #0,r1 | 1002 tst r1,r1 | 1004 bt 1008 | 1006 nop | 1008 jsr @r12 | 100a nop | 100c tst r0,r0 | 100e bt 1002
        let code = [0xE100, 0x2118, 0x8900, 0x0009, 0x4C0B, 0x0009, 0x2008, 0x89F8];
        let sub = [0x000B, 0xE000]; // rts; mov #0,r0: "nothing to do"
        assert_eq!(idle_of(&code, &sub, 2_000_000), Some(0x1008));
        // without a call: the loop's branch target
        let code = [0xE100, 0x2118, 0x8900, 0x0009, 0x0009, 0x0009, 0x2118, 0x89F8];
        assert_eq!(idle_of(&code, &sub, 2_000_000), Some(0x1002));
    }

    #[test]
    fn busy_loops_are_not_idle_loops() {
        // a loop that counts (its state changes every iteration)
        let code = [0xE100, 0x7101, 0x0009, 0x4C0B, 0x0009, 0x2008, 0x89F9];
        let sub = [0x000B, 0xE000];
        assert_eq!(idle_of(&code, &sub, 2_000_000), None);
        // a loop that polls a device register (the value read is discarded: the CPU state does not change)
        let code = [0xE100, 0x0009, 0x0009, 0x4C0B, 0x0009, 0x2008, 0x89F9];
        let sub = [0x000B, 0x60B2]; // rts; mov.l @r11,r0 (r11 points to a device)
        assert_eq!(idle_of(&code, &sub, 2_000_000), None);
    }
}
