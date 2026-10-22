//! SH-2A (SH7269) CPU core: integer, SH-2A extensions, register banks and the FPU subset used by the firmware.
//!
//! Semantics follow the Renesas SH-2A/SH2A-FPU software manual (REJ09B0051). One `step()` executes one
//! instruction; a delayed branch executes together with its slot instruction (counted as one step). Branch targets are computed before the slot executes.

use crate::mem::Memory;

pub const SR_T: u32 = 1;
pub const SR_S: u32 = 2;
pub const SR_Q: u32 = 1 << 8;
pub const SR_M: u32 = 1 << 9;
pub const SR_CS: u32 = 1 << 13;
pub const SR_BO: u32 = 1 << 14;
pub const SR_MASK: u32 = 0x63F3;
pub const FPSCR_PR: u32 = 1 << 19;
pub const FPSCR_SZ: u32 = 1 << 20;
pub const FPSCR_DN: u32 = 1 << 18;
pub const FPSCR_MASK: u32 = 0x003F_FFFF;

/// Register-bank entry order used by LDBANK/STBANK entry numbers (and start-state files): r0..r14, mach, vto, pr, gbr, macl.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Bank {
    pub r: [u32; 15],
    pub gbr: u32,
    pub mach: u32,
    pub macl: u32,
    pub pr: u32,
    pub vto: u32,
}

/// Floating-point rounding behaviour.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FloatMode {
    /// Honour FPSCR.RM / DN like the hardware.
    Hardware,
    /// Round-to-nearest regardless of FPSCR.RM, as Ghidra's p-code (used only for lockstep comparison).
    GhidraCompat,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Fault {
    Illegal { pc: u32, op: u32 },
    IllegalSlot { pc: u32, op: u32 },
    Address { pc: u32, addr: u32, size: usize, write: bool },
    Unsupported { pc: u32, what: &'static str },
    BankOverflow { pc: u32 },
    BankEmpty { pc: u32 },
    DivideByZero { pc: u32 },
}

/// Everything outside flat memory: on-chip peripherals and external devices.
pub trait Mmio {
    fn read(&mut self, cpu_pc: u32, addr: u32, size: usize) -> u64;
    fn write(&mut self, cpu_pc: u32, addr: u32, size: usize, value: u64);
    /// IBNR (0xFFFE080E) bank number field, owned by the interrupt controller.
    fn ibnr(&mut self) -> u16;
    fn set_ibnr(&mut self, v: u16);
    /// True while the NOR flash answers reads with status/protection data instead of array data (after a command).
    fn flash_cmd_mode(&self) -> bool { false }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Cpu {
    pub r: [u32; 16],
    pub sr: u32,
    pub gbr: u32,
    pub vbr: u32,
    pub mach: u32,
    pub macl: u32,
    pub pr: u32,
    pub pc: u32,
    pub tbr: u32,
    pub fpscr: u32,
    pub fpul: u32,
    pub fr: [u32; 16],
    pub banks: [Bank; 15],
    pub mem: Memory,
    pub float_mode: FloatMode,
    pub sleeping: bool,
    /// PC of the instruction currently executing (for MMIO attribution and faults).
    pub cur_pc: u32,
    /// Debug watchpoint [start, end) on data accesses; hits are appended to `watch_log` (pc, addr, size, write, value).
    pub watch: Option<(u32, u32)>,
    pub watch_log: Vec<(u32, u32, usize, bool, u32)>,
    /// LDBANK/STBANK with a selector naming no bank entry (see `op_bank`).
    #[serde(skip)]
    pub bank_anomalies: u64,
    /// The fault of the instruction that failed, passed from its handler to `exec`.
    #[serde(skip)]
    fault: Option<Fault>,
    /// A data access went to the MMIO bus (a device) since `run` started its current instruction.
    #[serde(skip)]
    io_touched: bool,
    /// Decoded code in flash (`run`).
    #[serde(skip)]
    blocks: BlockCache,
    /// Call/return/branch observer for runtime HLE detection (`crate::detect`); None when not detecting.
    #[serde(skip)]
    pub trace: Option<Box<crate::detect::Trace>>,
}

#[inline]
fn sext8(v: u32) -> u32 { v as u8 as i8 as i32 as u32 }
#[inline]
fn sext16(v: u32) -> u32 { v as u16 as i16 as i32 as u32 }
#[inline]
fn sext12(v: u32) -> u32 { (((v & 0xFFF) << 20) as i32 >> 20) as u32 }

impl Cpu {
    pub fn new(mem: Memory) -> Self {
        Cpu {
            r: [0; 16], sr: 0xF0, gbr: 0, vbr: 0, mach: 0, macl: 0, pr: 0, pc: 0, tbr: 0,
            fpscr: 0x0004_0001, fpul: 0, fr: [0; 16], banks: [Bank::default(); 15], mem,
            float_mode: FloatMode::Hardware, sleeping: false, cur_pc: 0, watch: None, watch_log: Vec::new(), bank_anomalies: 0, fault: None, io_touched: false, blocks: BlockCache::default(), trace: None,
        }
    }

    #[inline] pub fn t(&self) -> bool { self.sr & SR_T != 0 }
    #[inline] pub fn set_t(&mut self, t: bool) { if t { self.sr |= SR_T } else { self.sr &= !SR_T } }
    #[inline] pub fn imask(&self) -> u32 { (self.sr >> 4) & 15 }

    /// A call has reached its callee (the delay slot, if any, executed): tell the detection trace, if any.
    #[inline(always)]
    fn traced_call(&mut self) { if self.trace.is_some() { self.trace_call(); } }
    #[inline(always)]
    fn traced_ret(&mut self) { if self.trace.is_some() { self.trace_ret(); } }
    /// A taken branch from `from`: backward ones are loop candidates.
    #[inline(always)]
    fn traced_branch(&mut self, from: u32) { if self.trace.is_some() && self.pc < from { self.trace_back(); } }

    #[cold]
    #[inline(never)]
    fn trace_call(&mut self) {
        if let Some(t) = self.trace.as_deref_mut() {
            t.call(&crate::detect::CallRegs { r: &self.r, fr: &self.fr, pc: self.pc, pr: self.pr, sr: self.sr, gbr: self.gbr, mach: self.mach, macl: self.macl, fpscr: self.fpscr }, &self.mem);
        }
    }
    #[cold]
    #[inline(never)]
    fn trace_ret(&mut self) { if let Some(t) = self.trace.as_deref_mut() { t.ret(self.pc, self.r[15]); } }
    #[cold]
    #[inline(never)]
    fn trace_back(&mut self) { if let Some(t) = self.trace.as_deref_mut() { t.back(self.pc, self.r[15]); } }

    // ---------- memory ----------
    /// Data read. SDRAM and flash (array mode) are read directly; anything else goes through `Memory::read_plain`
    /// and then the MMIO bus, as before.
    #[inline(always)]
    pub fn read<M: Mmio + ?Sized>(&mut self, io: &mut M, a: u32, n: usize) -> Result<u32, Fault> {
        if a as usize & (n - 1) != 0 { return Err(Fault::Address { pc: self.cur_pc, addr: a, size: n, write: false }); }
        let v = match self.mem.read_fast(a, n) {
            Some(v) => v,
            None => match self.mem.read_plain(a, n) { Some(v) => v as u32, None => { self.io_touched = true; io.read(self.cur_pc, a, n) as u32 } },
        };
        if let Some((s, e)) = self.watch { if a < e && a + n as u32 > s && self.watch_log.len() < 100_000 { self.watch_log.push((self.cur_pc, a, n, false, v)); } }
        Ok(v)
    }
    #[inline(always)]
    pub fn write<M: Mmio + ?Sized>(&mut self, io: &mut M, a: u32, n: usize, v: u32) -> Result<(), Fault> {
        if a as usize & (n - 1) != 0 { return Err(Fault::Address { pc: self.cur_pc, addr: a, size: n, write: true }); }
        if let Some((s, e)) = self.watch { if a < e && a + n as u32 > s && self.watch_log.len() < 100_000 { self.watch_log.push((self.cur_pc, a, n, true, v)); } }
        if !self.mem.write_fast(a, n, v) && !self.mem.write_plain(a, n, v as u64) {
            self.io_touched = true;
            io.write(self.cur_pc, a, n, v as u64);
            if crate::periph::nor::Nor::handles(a) { self.mem.flash_cmd_mode = io.flash_cmd_mode(); }
        }
        Ok(())
    }
    #[inline(always)] fn r8(&mut self, io: &mut dyn Mmio, a: u32) -> Result<u32, Fault> { self.read(io, a, 1) }
    #[inline(always)] fn r16(&mut self, io: &mut dyn Mmio, a: u32) -> Result<u32, Fault> { self.read(io, a, 2) }
    #[inline(always)] fn r32(&mut self, io: &mut dyn Mmio, a: u32) -> Result<u32, Fault> { self.read(io, a, 4) }
    #[inline(always)] fn w8(&mut self, io: &mut dyn Mmio, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 1, v & 0xFF) }
    #[inline(always)] fn w16(&mut self, io: &mut dyn Mmio, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 2, v & 0xFFFF) }
    #[inline(always)] fn w32(&mut self, io: &mut dyn Mmio, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 4, v) }

    /// Instruction fetch (code runs from flash, SDRAM or on-chip RAM only).
    #[inline(always)]
    fn fetch(&self, a: u32) -> Result<u32, Fault> {
        match self.mem.fetch16(a) {
            Some(v) if a & 1 == 0 => Ok(v as u32),
            _ => Err(Fault::Address { pc: a, addr: a, size: 2, write: false }),
        }
    }

    // ---------- exceptions / banks ----------
    /// Interrupt acceptance with register banking (IBNR.BE = 01): save R0-R14/GBR/MACH/MACL/PR/VTO to bank[BN],
    /// BN += 1, push SR then PC, set SR.IMASK = level, jump to the vector table entry.
    pub fn take_interrupt<M: Mmio>(&mut self, io: &mut M, vector: u32, level: u32, banked: bool) -> Result<(), Fault> {
        self.take_interrupt_dyn(io, vector, level, banked)
    }

    fn take_interrupt_dyn(&mut self, io: &mut dyn Mmio, vector: u32, level: u32, banked: bool) -> Result<(), Fault> {
        self.cur_pc = self.pc;
        if banked {
            let ibnr = io.ibnr();
            let bn = (ibnr & 15) as usize;
            if bn >= 15 { return Err(Fault::BankOverflow { pc: self.pc }); }
            let mut b = Bank { gbr: self.gbr, mach: self.mach, macl: self.macl, pr: self.pr, vto: vector * 4, ..Default::default() };
            b.r.copy_from_slice(&self.r[..15]);
            self.banks[bn] = b;
            io.set_ibnr((ibnr & !15) | ((bn as u16 + 1) & 15));
        }
        let sp = self.r[15];
        self.w32(io, sp.wrapping_sub(4), self.sr)?;
        self.w32(io, sp.wrapping_sub(8), self.pc)?;
        self.r[15] = sp.wrapping_sub(8);
        self.sr = (self.sr & !0xF0) | ((level & 15) << 4);
        let va = self.vbr.wrapping_add(vector * 4);
        let ret = self.pc;
        self.pc = self.r32(io, va)?;
        if let Some(t) = self.trace.as_deref_mut() { t.interrupt(self.pc, ret, sp); }
        self.sleeping = false;
        Ok(())
    }

    fn general_exception(&mut self, io: &mut dyn Mmio, vector: u32, return_pc: u32) -> Result<(), Fault> {
        let sp = self.r[15];
        self.w32(io, sp.wrapping_sub(4), self.sr)?;
        self.w32(io, sp.wrapping_sub(8), return_pc)?;
        self.r[15] = sp.wrapping_sub(8);
        self.pc = self.r32(io, self.vbr.wrapping_add(vector * 4))?;
        if let Some(t) = self.trace.as_deref_mut() { t.interrupt(self.pc, return_pc, sp); }
        Ok(())
    }

    // ---------- execution ----------
    pub fn step<M: Mmio>(&mut self, io: &mut M) -> Result<(), Fault> { self.step_dyn(io) }

    /// One instruction (a delayed branch together with its slot).
    #[inline]
    pub fn step_dyn(&mut self, io: &mut dyn Mmio) -> Result<(), Fault> {
        let pc = self.pc;
        self.cur_pc = pc;
        let op = self.fetch(pc)?;
        self.exec(io, op, pc, false)
    }

    /// Run instructions until `l.budget` is used up or the machine must look at something before or after an
    /// instruction (see `RunLimits`, `RunExit`). Returns the instructions completed (a faulting one is not counted).
    /// Each instruction is exactly what `step` would execute; only the machine's checks between instructions are
    /// reduced to the ones that can change without a device access.
    ///
    /// Code in flash runs from decoded blocks (`BlockCache`): within a block nothing the checks look at can change, so
    /// they are made once at its first instruction.
    pub fn run<M: Mmio>(&mut self, io: &mut M, l: &mut RunLimits) -> (u32, Result<RunExit, Fault>) {
        let io: &mut dyn Mmio = io;
        let mut cache = std::mem::take(&mut self.blocks);
        if cache.gen != self.mem.code_gen { cache.gen = self.mem.code_gen; cache.epoch = cache.epoch.wrapping_add(1); }
        let r = self.run_with(io, l, &mut cache);
        self.blocks = cache;
        r
    }

    #[inline(always)]
    fn run_with(&mut self, io: &mut dyn Mmio, l: &mut RunLimits, cache: &mut BlockCache) -> (u32, Result<RunExit, Fault>) {
        let (lo, hi) = l.hooks_range;
        let hooks = l.hooks;
        let trace_on = self.trace.is_some();
        let mut done = 0;
        self.io_touched = false;
        loop {
            let pc = self.pc;
            let imask = (self.sr >> 4) & 15;
            if l.irq_level > imask || (pc == l.idle_pc && imask == 0) || (pc >= lo && pc <= hi && hooks.iter().any(|h| h.0 == pc)) {
                return (done, Ok(RunExit::Check));
            }
            if let Some(b) = cache.block(&self.mem.flash, pc) {
                // The checks above hold for every instruction of the block: SR.IMASK changes only at its end, the
                // interrupt level only with a device access, and no idle-loop or hooked PC lies inside it.
                let b = &cache.blocks[b];
                let inside = |a: u32| a > pc && a < b.end;
                if !inside(l.idle_pc) && !(lo <= hi && hooks.iter().any(|h| inside(h.0))) {
                    let n = b.n.min(l.budget - done) as usize;
                    for e in &cache.ops[b.first as usize..b.first as usize + n] {
                        let k = *l.hist_pos & 63;
                        l.history[k] = e.pc;
                        l.hist_irq[k] = false;
                        *l.hist_pos += 1;
                        self.cur_pc = e.pc;
                        self.pc = e.pc.wrapping_add(2);
                        if !(e.f)(self, io, e.op) { return (done, Err(self.fault.take().unwrap_or(Fault::Illegal { pc: e.pc, op: e.op }))); }
                        done += 1;
                        if self.io_touched || (trace_on && self.trace.as_ref().is_some_and(|t| t.event)) {
                            self.io_touched = false;
                            return (done, Ok(RunExit::After(e.pc)));
                        }
                    }
                    if done >= l.budget { return (done, Ok(RunExit::Budget)); }
                    continue;
                }
            }
            let k = *l.hist_pos & 63;
            l.history[k] = pc;
            l.hist_irq[k] = false;
            *l.hist_pos += 1;
            self.cur_pc = pc;
            let r = match self.fetch(pc) { Ok(op) => self.exec(io, op, pc, false), Err(f) => Err(f) };
            if let Err(f) = r { return (done, Err(f)); }
            done += 1;
            if self.io_touched || self.trace.as_ref().is_some_and(|t| t.event) {
                self.io_touched = false;
                return (done, Ok(RunExit::After(pc)));
            }
            if done >= l.budget { return (done, Ok(RunExit::Budget)); }
        }
    }

    /// Code in flash may have been changed from outside the CPU (a new image, a test): blocks are compared with flash
    /// again before they next run. `Machine::run` calls this on every call.
    pub fn code_may_have_changed(&mut self) { self.blocks.epoch = self.blocks.epoch.wrapping_add(1); }

    /// Execute the slot instruction of a delayed branch at `slot_pc`.
    fn slot(&mut self, io: &mut dyn Mmio, slot_pc: u32) -> Result<(), Fault> {
        let op = self.fetch(slot_pc)?;
        self.cur_pc = slot_pc;
        self.exec(io, op, slot_pc, true)
    }

    #[inline]
    fn branch_delayed(&mut self, io: &mut dyn Mmio, pc: u32, target: u32, in_slot: bool, op: u32) -> Result<(), Fault> {
        if in_slot { return Err(Fault::IllegalSlot { pc, op }); }
        self.slot(io, pc.wrapping_add(2))?;
        self.pc = target;
        Ok(())
    }

    /// Execute `op` fetched from `pc` (= `cur_pc`): every instruction falls through to pc+2 unless it sets the PC
    /// itself. The opcode selects its handler from a table decoded once (`OPS`).
    #[inline(always)]
    fn exec(&mut self, io: &mut dyn Mmio, op: u32, pc: u32, in_slot: bool) -> Result<(), Fault> {
        self.pc = pc.wrapping_add(2);
        if HANDLERS[OPS[op as usize] as usize](self, io, op | (in_slot as u32) << 16) { Ok(()) } else { Err(self.fault.take().unwrap_or(Fault::Illegal { pc, op })) }
    }

    fn resbank(&mut self, io: &mut dyn Mmio) -> Result<(), Fault> {
        let ibnr = io.ibnr();
        let bn = (ibnr & 15) as usize;
        if self.sr & SR_BO != 0 { return Err(Fault::Unsupported { pc: self.cur_pc, what: "RESBANK with SR.BO (bank overflow restore from stack)" }); }
        if bn == 0 { return Err(Fault::BankEmpty { pc: self.cur_pc }); }
        let b = self.banks[bn - 1];
        self.r[..15].copy_from_slice(&b.r);
        self.gbr = b.gbr; self.mach = b.mach; self.macl = b.macl; self.pr = b.pr;
        io.set_ibnr((ibnr & !15) | ((bn as u16 - 1) & 15));
        Ok(())
    }

    /// DIV1 Rm,Rn — transcription of the manual's pseudocode.
    #[inline(always)]
    fn div1(&mut self, n: usize, m: usize) {
        let old_q = self.sr & SR_Q != 0;
        let mval = self.sr & SR_M != 0;
        let mut q = self.r[n] >> 31 != 0;
        let tmp2 = self.r[m];
        let rn = (self.r[n] << 1) | (self.sr & SR_T);
        let subtract = old_q == mval;
        let res = if subtract { rn.wrapping_sub(tmp2) } else { rn.wrapping_add(tmp2) };
        let tmp1 = if subtract { res > rn } else { res < rn };
        // old_q=0: M=0 -> Q = Q ? !tmp1 : tmp1 ; M=1 -> Q = Q ? tmp1 : !tmp1
        // old_q=1: M=0 -> Q = Q ? !tmp1 : tmp1 ; M=1 -> Q = Q ? tmp1 : !tmp1
        q = if !mval { if q { !tmp1 } else { tmp1 } } else if q { tmp1 } else { !tmp1 };
        self.r[n] = res;
        self.sr = (self.sr & !(SR_Q | SR_T)) | if q { SR_Q } else { 0 } | if q == mval { SR_T } else { 0 };
    }

    fn mac_l(&mut self, io: &mut dyn Mmio, n: usize, m: usize) -> Result<(), Fault> {
        let an = self.r[n]; let vn = self.r32(io, an)? as i32 as i64; self.r[n] = an.wrapping_add(4);
        let am = self.r[m]; let vm = self.r32(io, am)? as i32 as i64; self.r[m] = am.wrapping_add(4);
        let mac = (((self.mach as u64) << 32) | self.macl as u64) as i64;
        let p = vn.wrapping_mul(vm);
        let mut res = mac.wrapping_add(p);
        if self.sr & SR_S != 0 {
            // 48-bit saturation
            let max = 0x0000_7FFF_FFFF_FFFFi64; let min = -0x0000_8000_0000_0000i64;
            let wide = mac as i128 + p as i128;
            res = if wide > max as i128 { max } else if wide < min as i128 { min } else { wide as i64 };
        }
        self.mach = (res >> 32) as u32; self.macl = res as u32;
        Ok(())
    }

    fn mac_w(&mut self, io: &mut dyn Mmio, n: usize, m: usize) -> Result<(), Fault> {
        let an = self.r[n]; let vn = self.r16(io, an)? as i16 as i64; self.r[n] = an.wrapping_add(2);
        let am = self.r[m]; let vm = self.r16(io, am)? as i16 as i64; self.r[m] = am.wrapping_add(2);
        let p = vn * vm;
        if self.sr & SR_S != 0 {
            // 32-bit saturating accumulate into MACL; MACH bit0 = overflow flag
            let s = self.macl as i32 as i64 + p;
            if s > i32::MAX as i64 { self.macl = i32::MAX as u32; self.mach |= 1; }
            else if s < i32::MIN as i64 { self.macl = i32::MIN as u32; self.mach |= 1; }
            else { self.macl = s as u32; }
        } else {
            let mac = (((self.mach as u64) << 32) | self.macl as u64) as i64;
            let res = mac.wrapping_add(p);
            self.mach = (res >> 32) as u32; self.macl = res as u32;
        }
        Ok(())
    }

    /// 0011nnnnmmmm0001 + 16-bit: MOV/MOVU/FMOV with 12-bit displacement.
    fn op32_disp12(&mut self, io: &mut dyn Mmio, op: u32, pc: u32) -> Result<(), Fault> {
        let n = ((op >> 8) & 15) as usize; let m = ((op >> 4) & 15) as usize;
        let lo = self.fetch(pc.wrapping_add(2))?;
        let d = lo & 0xFFF;
        match lo >> 12 {
            0x0 => { let a = self.r[n].wrapping_add(d); self.w8(io, a, self.r[m])?; }
            0x1 => { let a = self.r[n].wrapping_add(d * 2); self.w16(io, a, self.r[m])?; }
            0x2 => { let a = self.r[n].wrapping_add(d * 4); self.w32(io, a, self.r[m])?; }
            0x3 => { // FMOV.S FRm,@(disp12,Rn) / FMOV.D (SZ=1)
                if self.fpscr & FPSCR_SZ != 0 { let a = self.r[n].wrapping_add(d * 8); self.w32(io, a, self.fr[m & 14])?; self.w32(io, a.wrapping_add(4), self.fr[m | 1])?; }
                else { let a = self.r[n].wrapping_add(d * 4); self.w32(io, a, self.fr[m])?; }
            }
            0x4 => { let a = self.r[m].wrapping_add(d); self.r[n] = sext8(self.r8(io, a)?); }
            0x5 => { let a = self.r[m].wrapping_add(d * 2); self.r[n] = sext16(self.r16(io, a)?); }
            0x6 => { let a = self.r[m].wrapping_add(d * 4); self.r[n] = self.r32(io, a)?; }
            0x7 => {
                if self.fpscr & FPSCR_SZ != 0 { let a = self.r[m].wrapping_add(d * 8); self.fr[n & 14] = self.r32(io, a)?; self.fr[n | 1] = self.r32(io, a.wrapping_add(4))?; }
                else { let a = self.r[m].wrapping_add(d * 4); self.fr[n] = self.r32(io, a)?; }
            }
            0x8 => { let a = self.r[m].wrapping_add(d); self.r[n] = self.r8(io, a)?; }
            0x9 => { let a = self.r[m].wrapping_add(d * 2); self.r[n] = self.r16(io, a)?; }
            _ => return Err(Fault::Illegal { pc, op: (op << 16) | lo }),
        }
        self.pc = pc.wrapping_add(4);
        Ok(())
    }

    /// 0011nnnn0iii1001 + 16-bit: bit operations on @(disp12,Rn).
    fn op32_bitops(&mut self, io: &mut dyn Mmio, op: u32, pc: u32) -> Result<(), Fault> {
        let n = ((op >> 8) & 15) as usize;
        if op & 0x80 != 0 { return Err(Fault::Illegal { pc, op }); }
        let bit = 1u32 << ((op >> 4) & 7);
        let lo = self.fetch(pc.wrapping_add(2))?;
        let a = self.r[n].wrapping_add(lo & 0xFFF);
        let v = self.r8(io, a)?;
        let b = v & bit != 0;
        match lo >> 12 {
            0x0 => self.w8(io, a, v & !bit)?,
            0x1 => self.w8(io, a, v | bit)?,
            0x2 => { let nv = if self.t() { v | bit } else { v & !bit }; self.w8(io, a, nv)? }
            0x3 => self.set_t(b),
            0x4 => { let t = self.t() & b; self.set_t(t) }
            0x5 => { let t = self.t() | b; self.set_t(t) }
            0x6 => { let t = self.t() ^ b; self.set_t(t) }
            0xB => self.set_t(!b),
            0xC => { let t = self.t() & !b; self.set_t(t) }
            0xD => { let t = self.t() | !b; self.set_t(t) }
            _ => return Err(Fault::Illegal { pc, op: (op << 16) | lo }),
        }
        self.pc = pc.wrapping_add(4);
        Ok(())
    }
    // ---------- FPU ----------
    #[inline] fn frf(&self, i: usize) -> f32 { f32::from_bits(self.fr[i]) }
    #[inline] fn set_frf(&mut self, i: usize, v: f32) { self.fr[i] = v.to_bits(); }
    #[inline] fn drf(&self, i: usize) -> f64 { f64::from_bits(((self.fr[i & 14] as u64) << 32) | self.fr[i | 1] as u64) }
    #[inline] fn set_drf(&mut self, i: usize, v: f64) { let b = v.to_bits(); self.fr[i & 14] = (b >> 32) as u32; self.fr[i | 1] = b as u32; }

    fn fpu(&mut self, io: &mut dyn Mmio, op: u32, pc: u32, n: usize, m: usize) -> Result<(), Fault> {
        // Ghidra 12.1.4 superh.sinc implements no PR=1 (double) arithmetic and no SZ=1 (FMOV.D) transfer: every
        // such encoding executes as its single-precision form ("TODO" entries). GhidraCompat reproduces that for
        // lockstep comparison only; FCNVSD/FCNVDS are implemented by Ghidra on DR registers regardless of PR.
        let compat = self.float_mode == FloatMode::GhidraCompat;
        let pr = !compat && self.fpscr & FPSCR_PR != 0;
        let sz = !compat && self.fpscr & FPSCR_SZ != 0;
        let rz = self.float_mode == FloatMode::Hardware && self.fpscr & 3 == 1;
        let dn = self.float_mode == FloatMode::Hardware && self.fpscr & FPSCR_DN != 0;
        let illegal = Fault::Illegal { pc, op };
        match op & 0xF {
            0x0..=0x3 => {
                if pr {
                    let (a, b) = (fz64(self.drf(n), dn), fz64(self.drf(m), dn));
                    let r = arith64(op & 3, a, b, rz);
                    self.set_drf(n, fz64(r, dn));
                } else {
                    let (a, b) = (fz32(self.frf(n), dn), fz32(self.frf(m), dn));
                    let r = arith32(op & 3, a, b, rz);
                    self.set_frf(n, fz32(r, dn));
                }
            }
            0x4 => { let t = if pr { self.drf(n) == self.drf(m) } else { self.frf(n) == self.frf(m) }; self.set_t(t); }
            0x5 => { let t = if pr { self.drf(n) > self.drf(m) } else { self.frf(n) > self.frf(m) }; self.set_t(t); }
            0x6 => { let a = self.r[m].wrapping_add(self.r[0]); self.fload(io, n, a, sz)?; }
            0x7 => { let a = self.r[n].wrapping_add(self.r[0]); self.fstore(io, m, a, sz)?; }
            0x8 => { let a = self.r[m]; self.fload(io, n, a, sz)?; }
            0x9 => { let a = self.r[m]; self.fload(io, n, a, sz)?; self.r[m] = a.wrapping_add(if sz { 8 } else { 4 }); }
            0xA => { let a = self.r[n]; self.fstore(io, m, a, sz)?; }
            0xB => { let a = self.r[n].wrapping_sub(if sz { 8 } else { 4 }); self.fstore(io, m, a, sz)?; self.r[n] = a; }
            0xC => { if sz { self.fr[n & 14] = self.fr[m & 14]; self.fr[n | 1] = self.fr[m | 1]; } else { self.fr[n] = self.fr[m]; } }
            0xE => { // FMAC FR0,FRm,FRn (single only)
                if pr { return Err(illegal); }
                let r = fmac32(fz32(self.frf(0), dn), fz32(self.frf(m), dn), fz32(self.frf(n), dn), rz);
                self.set_frf(n, fz32(r, dn));
            }
            0xD => match m {
                0x0 => self.fr[n] = self.fpul, // FSTS
                0x1 => self.fpul = self.fr[n], // FLDS
                0x2 => { // FLOAT FPUL
                    let v = self.fpul as i32;
                    if pr { self.set_drf(n, v as f64) } else { self.set_frf(n, int_to_f32(v, rz)) }
                }
                0x3 => { // FTRC -> FPUL (round toward zero, saturating)
                    let v = if pr { self.drf(n) } else { self.frf(n) as f64 };
                    self.fpul = ftrc(v) as u32;
                }
                0x4 => { if pr { self.fr[n & 14] ^= 0x8000_0000 } else { self.fr[n] ^= 0x8000_0000 } }
                0x5 => { if pr { self.fr[n & 14] &= 0x7FFF_FFFF } else { self.fr[n] &= 0x7FFF_FFFF } }
                0x6 => {
                    if pr { let r = sqrt64(fz64(self.drf(n), dn), rz); self.set_drf(n, fz64(r, dn)); }
                    else { let r = sqrt32(fz32(self.frf(n), dn), rz); self.set_frf(n, fz32(r, dn)); }
                }
                0x8 => self.fr[n] = 0,
                0x9 => self.fr[n] = 0x3F80_0000,
                0xA => { // FCNVSD FPUL,DRn
                    if !pr && !compat { return Err(illegal); }
                    self.set_drf(n, fz32(f32::from_bits(self.fpul), dn) as f64);
                }
                0xB => { // FCNVDS DRm,FPUL
                    if !pr && !compat { return Err(illegal); }
                    self.fpul = fz32(f64_to_f32(fz64(self.drf(n), dn), rz), dn).to_bits();
                }
                0xF => match n {
                    0x3 => self.fpscr ^= FPSCR_SZ, // FSCHG
                    0x7 => self.fpscr ^= FPSCR_PR, // FPCHG
                    _ => return Err(illegal),
                },
                _ => return Err(illegal),
            },
            _ => return Err(illegal),
        }
        Ok(())
    }

    fn fload(&mut self, io: &mut dyn Mmio, n: usize, a: u32, sz: bool) -> Result<(), Fault> {
        if sz { self.fr[n & 14] = self.r32(io, a)?; self.fr[n | 1] = self.r32(io, a.wrapping_add(4))?; } else { self.fr[n] = self.r32(io, a)?; }
        Ok(())
    }
    fn fstore(&mut self, io: &mut dyn Mmio, m: usize, a: u32, sz: bool) -> Result<(), Fault> {
        if sz { self.w32(io, a, self.fr[m & 14])?; self.w32(io, a.wrapping_add(4), self.fr[m | 1])?; } else { self.w32(io, a, self.fr[m])?; }
        Ok(())
    }
}

/// Stop conditions of `Cpu::run`: the checks the machine makes before each instruction (`Machine::step`) that can
/// change without a device access, and the instruction history it keeps.
pub struct RunLimits<'a> {
    /// Instructions to run at most (up to the next peripheral tick).
    pub budget: u32,
    /// Stop at this PC while SR.IMASK is 0: the idle loop to skip (1: none).
    pub idle_pc: u32,
    /// Stop when this interrupt priority level is above SR.IMASK: a pending request would be accepted (0: none).
    pub irq_level: u32,
    /// Stop at these PCs: entry points handled in software (sorted, as `Hooks::rgah`); `hooks_range` is the first and
    /// last ((1, 0): none).
    pub hooks: &'a [(u32, u8)],
    pub hooks_range: (u32, u32),
    /// Ring of the last 64 executed PCs (`Machine::history`).
    pub history: &'a mut [u32; 64],
    pub hist_irq: &'a mut [bool; 64],
    pub hist_pos: &'a mut usize,
}

/// Why `Cpu::run` stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunExit {
    /// The budget is used up.
    Budget,
    /// The next instruction needs the machine's checks first (idle loop, interrupt acceptance, a hooked entry point).
    Check,
    /// The instruction at this PC accessed a device or raised a detection event: the machine handles that next.
    After(u32),
}

/// Decoded straight-line code in flash, for `Cpu::run`. A block runs from its first instruction to the first one that
/// can change the PC (a branch, call, return, TRAPA) or SR.IMASK (LDC to SR), inclusive, and at most `MAX_BLOCK`
/// instructions. Blocks are found by their first PC (a table per 64 KiB of flash). Whenever flash may have changed,
/// each block is compared with it again before it next runs (`epoch`): after NOR programming (`Memory::code_gen`) and
/// on every `Machine::run` (in case the image was replaced or edited from outside).
#[derive(Default)]
pub struct BlockCache {
    /// Per 64 KiB of flash: for each halfword, the index + 1 of the block starting there (0: none).
    pages: Vec<Option<Box<[u32]>>>,
    blocks: Vec<Block>,
    ops: Vec<Decoded>,
    /// The code bytes each block was decoded from.
    code: Vec<u8>,
    /// Blocks checked in an older epoch are compared with flash before they run.
    epoch: u32,
    /// `Memory::code_gen` when last seen.
    gen: u64,
}

struct Block {
    /// Address after the block's last instruction.
    end: u32,
    /// First instruction in `ops`, and the number of instructions.
    first: u32,
    n: u32,
    /// The block's bytes in `code` (end - start of them).
    code: u32,
    epoch: u32,
}

#[derive(Clone, Copy)]
struct Decoded { f: OpFn, op: u32, pc: u32 }

/// Instructions per block at most.
const MAX_BLOCK: usize = 64;
/// Decoded instructions kept before the cache starts again from empty.
const MAX_DECODED: usize = 1 << 20;

impl BlockCache {
    /// The block starting at `pc`, decoded first if needed; None when `pc` is not an instruction address in flash.
    #[inline(always)]
    fn block(&mut self, flash: &[u8], pc: u32) -> Option<usize> {
        if pc & 1 != 0 || pc as usize + 2 > flash.len() { return None; }
        if let Some(Some(p)) = self.pages.get((pc >> 16) as usize) {
            let i = p[((pc & 0xFFFF) >> 1) as usize];
            if i != 0 {
                let i = i as usize - 1;
                if self.blocks[i].epoch == self.epoch { return Some(i); }
                return self.revalidate(flash, pc, i);
            }
        }
        self.build(flash, pc)
    }

    /// A block from an older epoch: still the same code, or decoded again.
    #[cold]
    #[inline(never)]
    fn revalidate(&mut self, flash: &[u8], pc: u32, i: usize) -> Option<usize> {
        let b = &mut self.blocks[i];
        let (s, len) = (pc as usize, (b.end - pc) as usize);
        if flash.get(s..s + len) == self.code.get(b.code as usize..b.code as usize + len) {
            b.epoch = self.epoch;
            return Some(i);
        }
        self.build(flash, pc)
    }

    /// Decode the block starting at `pc` (None if not even its first instruction lies in flash).
    #[cold]
    #[inline(never)]
    fn build(&mut self, flash: &[u8], pc: u32) -> Option<usize> {
        if self.ops.len() > MAX_DECODED { *self = BlockCache { epoch: self.epoch, gen: self.gen, ..Default::default() }; }
        let first = self.ops.len();
        let mut a = pc as usize;
        while self.ops.len() - first < MAX_BLOCK && a + 2 <= flash.len() {
            let op = u16::from_be_bytes([flash[a], flash[a + 1]]) as u32;
            let k = OPS[op as usize];
            let len = if k == K::op_movi20 as u8 || k == K::op_disp12 as u8 || k == K::op_bitops as u8 { 4 } else { 2 };
            if a + len > flash.len() { break; }
            self.ops.push(Decoded { f: HANDLERS[k as usize], op, pc: a as u32 });
            a += len;
            if ENDS_BLOCK[k as usize] { break; }
        }
        let n = self.ops.len() - first;
        if n == 0 { return None; }
        let code = self.code.len();
        self.code.extend_from_slice(&flash[pc as usize..a]);
        self.blocks.push(Block { end: a as u32, first: first as u32, n: n as u32, code: code as u32, epoch: self.epoch });
        let page = (pc >> 16) as usize;
        if self.pages.len() <= page { self.pages.resize_with(page + 1, || None); }
        let p = self.pages[page].get_or_insert_with(|| vec![0u32; 1 << 15].into_boxed_slice());
        p[((pc & 0xFFFF) >> 1) as usize] = self.blocks.len() as u32;
        Some(self.blocks.len() - 1)
    }
}

/// Instruction kinds that end a block: they can change the PC, or SR.IMASK (and so which interrupts are accepted).
static ENDS_BLOCK: [bool; 256] = {
    let mut t = [false; 256];
    let ends = [K::op_illegal, K::op_bsrf, K::op_braf, K::op_rts, K::op_rte, K::op_rtsn, K::op_rtvn, K::op_jsr, K::op_jmp, K::op_jsrn,
        K::op_jsrn_tbr, K::op_bt_bf, K::op_bts_bfs, K::op_bra, K::op_bsr, K::op_trapa, K::op_ldc_sr, K::op_ldcl_sr];
    let mut i = 0;
    while i < ends.len() { t[ends[i] as usize] = true; i += 1; }
    t
};

// ---------- instruction handlers ----------
//
// Every 16-bit opcode is decoded once, at compile time, to the handler of its instruction (`OPS`). A handler runs one
// instruction with the CPU's PC already advanced to pc+2 (`Cpu::exec`); the instruction's own address is `cur_pc`.
// It returns false after storing a fault in `Cpu::fault`. The handlers are the arms of the former single `exec` match,
// one small function each, so that a step costs one table lookup and a call instead of nested matches through a large
// function (whose prologue alone saved and restored every callee-saved register for every instruction).

/// An instruction handler: (CPU, bus, opcode | in a delay slot << 16) -> completed without a fault.
pub type OpFn = fn(&mut Cpu, &mut dyn Mmio, u32) -> bool;

/// Handler of every 16-bit opcode.
static OPS: [u8; 65536] = {
    let mut t = [0u8; 65536];
    let mut op = 0;
    while op < 65536 { t[op] = decode(op as u32) as u8; op += 1; }
    t
};

#[inline(always)] fn rn(op: u32) -> usize { ((op >> 8) & 15) as usize }
#[inline(always)] fn rm(op: u32) -> usize { ((op >> 4) & 15) as usize }

/// Defines handlers: `name(cpu, io, op, in_slot) { body }`; the body may use `?` and `return` like the former match arms.
macro_rules! handlers {
    ($( $name:ident($c:ident, $io:ident, $op:ident, $s:ident) $body:block )*) => {
        $(
            #[allow(unused_variables)]
            fn $name($c: &mut Cpu, $io: &mut dyn Mmio, arg: u32) -> bool {
                let ($op, $s) = (arg & 0xFFFF, arg >> 16 != 0);
                #[inline(always)]
                #[allow(unused_variables, unused_mut, unreachable_code, clippy::needless_return)]
                fn body($c: &mut Cpu, $io: &mut dyn Mmio, $op: u32, $s: bool) -> Result<(), Fault> { $body Ok(()) }
                match body($c, $io, $op, $s) { Ok(()) => true, Err(f) => { $c.fault = Some(f); false } }
            }
        )*
        /// Instruction kinds, in the order of `HANDLERS`.
        #[allow(non_camel_case_types, dead_code)]
        #[derive(Clone, Copy)]
        #[repr(u8)]
        enum K { $($name),* }
        static HANDLERS: [OpFn; [$(stringify!($name)),*].len()] = [$($name),*];
    };
}

#[inline(always)]
fn no_slot(c: &Cpu, op: u32, in_slot: bool) -> Result<(), Fault> { if in_slot { Err(Fault::IllegalSlot { pc: c.cur_pc, op }) } else { Ok(()) } }

handlers! {
    op_illegal(c, io, op, s) { return Err(Fault::Illegal { pc: c.cur_pc, op }); }

    // ----- 0000 -----
    op_movi20(c, io, op, s) { // MOVI20 / MOVI20S (32-bit)
        no_slot(c, op, s)?;
        let pc = c.cur_pc;
        let lo = c.fetch(pc.wrapping_add(2))?;
        let imm = (((op >> 4) & 15) << 16) | lo;
        let v = ((imm << 12) as i32 >> 12) as u32;
        c.r[rn(op)] = if op & 1 == 0 { v } else { v << 8 };
        c.pc = pc.wrapping_add(4);
    }
    op_stc_sr(c, io, op, s) { c.r[rn(op)] = c.sr; }
    op_stc_gbr(c, io, op, s) { c.r[rn(op)] = c.gbr; }
    op_stc_vbr(c, io, op, s) { c.r[rn(op)] = c.vbr; }
    op_bsrf(c, io, op, s) { // BSRF Rn
        let pc = c.cur_pc;
        let t = pc.wrapping_add(4).wrapping_add(c.r[rn(op)]);
        c.pr = pc.wrapping_add(4);
        c.branch_delayed(io, pc, t, s, op)?;
        c.traced_call();
    }
    op_braf(c, io, op, s) { // BRAF Rn
        let pc = c.cur_pc;
        let t = pc.wrapping_add(4).wrapping_add(c.r[rn(op)]);
        c.branch_delayed(io, pc, t, s, op)?;
        c.traced_branch(pc);
    }
    op_nop(c, io, op, s) {} // NOP, PREF @Rn (no cache model)
    op_movb_s0(c, io, op, s) { let a = c.r[rn(op)].wrapping_add(c.r[0]); c.w8(io, a, c.r[rm(op)])?; }
    op_movw_s0(c, io, op, s) { let a = c.r[rn(op)].wrapping_add(c.r[0]); c.w16(io, a, c.r[rm(op)])?; }
    op_movl_s0(c, io, op, s) { let a = c.r[rn(op)].wrapping_add(c.r[0]); c.w32(io, a, c.r[rm(op)])?; }
    op_mull(c, io, op, s) { c.macl = c.r[rn(op)].wrapping_mul(c.r[rm(op)]); }
    op_clrt(c, io, op, s) { c.sr &= !SR_T; }
    op_sett(c, io, op, s) { c.sr |= SR_T; }
    op_clrmac(c, io, op, s) { c.mach = 0; c.macl = 0; }
    op_clrs(c, io, op, s) { c.sr &= !SR_S; }
    op_sets(c, io, op, s) { c.sr |= SR_S; }
    op_nott(c, io, op, s) { c.sr ^= SR_T; } // NOTT (T only)
    op_div0u(c, io, op, s) { c.sr &= !(SR_M | SR_Q | SR_T); }
    op_movt(c, io, op, s) { c.r[rn(op)] = c.sr & SR_T; }
    op_movrt(c, io, op, s) { c.r[rn(op)] = (c.sr & SR_T) ^ 1; }
    op_sts_mach(c, io, op, s) { c.r[rn(op)] = c.mach; }
    op_sts_macl(c, io, op, s) { c.r[rn(op)] = c.macl; }
    op_sts_pr(c, io, op, s) { c.r[rn(op)] = c.pr; }
    op_stc_tbr(c, io, op, s) { c.r[rn(op)] = c.tbr; }
    op_sts_fpul(c, io, op, s) { c.r[rn(op)] = c.fpul; }
    op_sts_fpscr(c, io, op, s) { c.r[rn(op)] = c.fpscr; }
    op_rts(c, io, op, s) { let pc = c.cur_pc; let t = c.pr; c.branch_delayed(io, pc, t, s, op)?; c.traced_ret(); }
    op_sleep(c, io, op, s) { c.sleeping = true; }
    op_rte(c, io, op, s) {
        let pc = c.cur_pc;
        if s { return Err(Fault::IllegalSlot { pc, op }); }
        let sp = c.r[15];
        let t = c.r32(io, sp)?;
        let sr = c.r32(io, sp.wrapping_add(4))?;
        c.r[15] = sp.wrapping_add(8);
        c.sr = sr & SR_MASK;
        c.slot(io, pc.wrapping_add(2))?;
        c.pc = t;
        if let Some(tr) = c.trace.as_deref_mut() { tr.rte(c.pc, c.r[15]); }
    }
    op_resbank(c, io, op, s) { no_slot(c, op, s)?; c.resbank(io)?; }
    op_rtsn(c, io, op, s) { no_slot(c, op, s)?; c.pc = c.pr; c.traced_ret(); }
    op_rtvn(c, io, op, s) { no_slot(c, op, s)?; c.r[0] = c.r[rn(op)]; c.pc = c.pr; c.traced_ret(); }
    op_movb_l0(c, io, op, s) { let a = c.r[rm(op)].wrapping_add(c.r[0]); c.r[rn(op)] = sext8(c.r8(io, a)?); }
    op_movw_l0(c, io, op, s) { let a = c.r[rm(op)].wrapping_add(c.r[0]); c.r[rn(op)] = sext16(c.r16(io, a)?); }
    op_movl_l0(c, io, op, s) { let a = c.r[rm(op)].wrapping_add(c.r[0]); c.r[rn(op)] = c.r32(io, a)?; }
    op_macl(c, io, op, s) { c.mac_l(io, rn(op), rm(op))?; }

    // ----- 0001 -----
    op_movl_sdisp(c, io, op, s) { let a = c.r[rn(op)].wrapping_add((op & 15) * 4); c.w32(io, a, c.r[rm(op)])?; }

    // ----- 0010 -----
    op_movb_s(c, io, op, s) { let a = c.r[rn(op)]; c.w8(io, a, c.r[rm(op)])?; }
    op_movw_s(c, io, op, s) { let a = c.r[rn(op)]; c.w16(io, a, c.r[rm(op)])?; }
    op_movl_s(c, io, op, s) { let a = c.r[rn(op)]; c.w32(io, a, c.r[rm(op)])?; }
    op_movb_m(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(1); c.w8(io, a, c.r[rm(op)])?; c.r[n] = a; }
    op_movw_m(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(2); c.w16(io, a, c.r[rm(op)])?; c.r[n] = a; }
    op_movl_m(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.r[rm(op)])?; c.r[n] = a; }
    op_div0s(c, io, op, s) {
        let q = c.r[rn(op)] >> 31 != 0; let mm = c.r[rm(op)] >> 31 != 0;
        c.sr = (c.sr & !(SR_Q | SR_M | SR_T)) | if q { SR_Q } else { 0 } | if mm { SR_M } else { 0 } | if q != mm { SR_T } else { 0 };
    }
    op_tst(c, io, op, s) { let t = c.r[rn(op)] & c.r[rm(op)] == 0; c.set_t(t); }
    op_and(c, io, op, s) { let v = c.r[rm(op)]; c.r[rn(op)] &= v; }
    op_xor(c, io, op, s) { let v = c.r[rm(op)]; c.r[rn(op)] ^= v; }
    op_or(c, io, op, s) { let v = c.r[rm(op)]; c.r[rn(op)] |= v; }
    op_cmpstr(c, io, op, s) { let x = c.r[rn(op)] ^ c.r[rm(op)]; let t = x & 0xFF == 0 || x & 0xFF00 == 0 || x & 0xFF_0000 == 0 || x & 0xFF00_0000 == 0; c.set_t(t); }
    op_xtrct(c, io, op, s) { let n = rn(op); c.r[n] = (c.r[n] >> 16) | (c.r[rm(op)] << 16); }
    op_muluw(c, io, op, s) { c.macl = (c.r[rn(op)] & 0xFFFF).wrapping_mul(c.r[rm(op)] & 0xFFFF); }
    op_mulsw(c, io, op, s) { c.macl = ((c.r[rn(op)] as i16 as i32).wrapping_mul(c.r[rm(op)] as i16 as i32)) as u32; }

    // ----- 0011 -----
    op_cmpeq(c, io, op, s) { let t = c.r[rn(op)] == c.r[rm(op)]; c.set_t(t); }
    op_cmphs(c, io, op, s) { let t = c.r[rn(op)] >= c.r[rm(op)]; c.set_t(t); }
    op_cmpge(c, io, op, s) { let t = c.r[rn(op)] as i32 >= c.r[rm(op)] as i32; c.set_t(t); }
    op_div1(c, io, op, s) { c.div1(rn(op), rm(op)); }
    op_dmulu(c, io, op, s) { let p = c.r[rn(op)] as u64 * c.r[rm(op)] as u64; c.mach = (p >> 32) as u32; c.macl = p as u32; }
    op_cmphi(c, io, op, s) { let t = c.r[rn(op)] > c.r[rm(op)]; c.set_t(t); }
    op_cmpgt(c, io, op, s) { let t = c.r[rn(op)] as i32 > c.r[rm(op)] as i32; c.set_t(t); }
    op_sub(c, io, op, s) { let n = rn(op); c.r[n] = c.r[n].wrapping_sub(c.r[rm(op)]); }
    op_subc(c, io, op, s) {
        let n = rn(op);
        let (a, b, cy) = (c.r[n], c.r[rm(op)], c.t() as u32);
        let tmp1 = a.wrapping_sub(b); let res = tmp1.wrapping_sub(cy);
        c.set_t(a < tmp1 || tmp1 < res);
        c.r[n] = res;
    }
    op_subv(c, io, op, s) {
        let n = rn(op);
        let (a, b) = (c.r[n] as i32, c.r[rm(op)] as i32);
        let (res, o) = a.overflowing_sub(b); c.r[n] = res as u32; c.set_t(o);
    }
    op_add(c, io, op, s) { let n = rn(op); c.r[n] = c.r[n].wrapping_add(c.r[rm(op)]); }
    op_dmuls(c, io, op, s) { let p = (c.r[rn(op)] as i32 as i64) * (c.r[rm(op)] as i32 as i64); c.mach = (p >> 32) as u32; c.macl = p as u32; }
    op_addc(c, io, op, s) {
        let n = rn(op);
        let (a, b, cy) = (c.r[n], c.r[rm(op)], c.t() as u32);
        let (s1, c1) = a.overflowing_add(b); let (s2, c2) = s1.overflowing_add(cy);
        c.r[n] = s2; c.set_t(c1 || c2);
    }
    op_addv(c, io, op, s) { let n = rn(op); let (res, o) = (c.r[n] as i32).overflowing_add(c.r[rm(op)] as i32); c.r[n] = res as u32; c.set_t(o); }
    op_disp12(c, io, op, s) { no_slot(c, op, s)?; let pc = c.cur_pc; c.op32_disp12(io, op, pc)?; }
    op_bitops(c, io, op, s) { no_slot(c, op, s)?; let pc = c.cur_pc; c.op32_bitops(io, op, pc)?; }

    // ----- 0100 -----
    op_shad(c, io, op, s) { let n = rn(op); c.r[n] = shad(c.r[n], c.r[rm(op)]); }
    op_shld(c, io, op, s) { let n = rn(op); c.r[n] = shld(c.r[n], c.r[rm(op)]); }
    op_macw(c, io, op, s) { c.mac_w(io, rn(op), rm(op))?; }
    op_shll(c, io, op, s) { let n = rn(op); c.set_t(c.r[n] >> 31 != 0); c.r[n] <<= 1; }
    op_shlr(c, io, op, s) { let n = rn(op); c.set_t(c.r[n] & 1 != 0); c.r[n] >>= 1; }
    op_stsl_mach(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.mach)?; c.r[n] = a; }
    op_stcl_sr(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.sr)?; c.r[n] = a; }
    op_rotl(c, io, op, s) { let n = rn(op); let t = c.r[n] >> 31; c.r[n] = (c.r[n] << 1) | t; c.set_t(t != 0); }
    op_rotr(c, io, op, s) { let n = rn(op); let t = c.r[n] & 1; c.r[n] = (c.r[n] >> 1) | (t << 31); c.set_t(t != 0); }
    op_ldsl_mach(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.mach = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_ldcl_sr(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.sr = c.r32(io, a)? & SR_MASK; c.r[n] = a.wrapping_add(4); }
    op_shll2(c, io, op, s) { c.r[rn(op)] <<= 2; }
    op_shlr2(c, io, op, s) { c.r[rn(op)] >>= 2; }
    op_lds_mach(c, io, op, s) { c.mach = c.r[rn(op)]; }
    op_jsr(c, io, op, s) { let pc = c.cur_pc; let t = c.r[rn(op)]; c.pr = pc.wrapping_add(4); c.branch_delayed(io, pc, t, s, op)?; c.traced_call(); }
    op_ldc_sr(c, io, op, s) { c.sr = c.r[rn(op)] & SR_MASK; }
    op_dt(c, io, op, s) { let n = rn(op); c.r[n] = c.r[n].wrapping_sub(1); let t = c.r[n] == 0; c.set_t(t); }
    op_cmppz(c, io, op, s) { let t = c.r[rn(op)] as i32 >= 0; c.set_t(t); }
    op_stsl_macl(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.macl)?; c.r[n] = a; }
    op_stcl_gbr(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.gbr)?; c.r[n] = a; }
    op_cmppl(c, io, op, s) { let t = c.r[rn(op)] as i32 > 0; c.set_t(t); }
    op_ldsl_macl(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.macl = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_ldcl_gbr(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.gbr = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_shll8(c, io, op, s) { c.r[rn(op)] <<= 8; }
    op_shlr8(c, io, op, s) { c.r[rn(op)] >>= 8; }
    op_lds_macl(c, io, op, s) { c.macl = c.r[rn(op)]; }
    op_tas(c, io, op, s) { let a = c.r[rn(op)]; let v = c.r8(io, a)?; c.set_t(v == 0); c.w8(io, a, v | 0x80)?; }
    op_ldc_gbr(c, io, op, s) { c.gbr = c.r[rn(op)]; }
    op_shar(c, io, op, s) { let n = rn(op); c.set_t(c.r[n] & 1 != 0); c.r[n] = ((c.r[n] as i32) >> 1) as u32; }
    op_stsl_pr(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.pr)?; c.r[n] = a; }
    op_stcl_vbr(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.vbr)?; c.r[n] = a; }
    op_rotcl(c, io, op, s) { let n = rn(op); let t = c.r[n] >> 31; c.r[n] = (c.r[n] << 1) | c.t() as u32; c.set_t(t != 0); }
    op_rotcr(c, io, op, s) { let n = rn(op); let t = c.r[n] & 1; c.r[n] = (c.r[n] >> 1) | ((c.t() as u32) << 31); c.set_t(t != 0); }
    op_ldsl_pr(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.pr = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_ldcl_vbr(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.vbr = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_shll16(c, io, op, s) { c.r[rn(op)] <<= 16; }
    op_shlr16(c, io, op, s) { c.r[rn(op)] >>= 16; }
    op_lds_pr(c, io, op, s) { c.pr = c.r[rn(op)]; }
    op_jmp(c, io, op, s) { let pc = c.cur_pc; let t = c.r[rn(op)]; return c.branch_delayed(io, pc, t, s, op); }
    op_ldc_vbr(c, io, op, s) { c.vbr = c.r[rn(op)]; }
    op_ldc_tbr(c, io, op, s) { c.tbr = c.r[rn(op)]; }
    op_jsrn(c, io, op, s) { let pc = c.cur_pc; if s { return Err(Fault::IllegalSlot { pc, op }); } c.pr = pc.wrapping_add(2); c.pc = c.r[rn(op)]; c.traced_call(); } // JSR/N
    op_stsl_fpul(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.fpul)?; c.r[n] = a; }
    op_ldsl_fpul(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.fpul = c.r32(io, a)?; c.r[n] = a.wrapping_add(4); }
    op_lds_fpul(c, io, op, s) { c.fpul = c.r[rn(op)]; }
    op_stsl_fpscr(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.w32(io, a, c.fpscr)?; c.r[n] = a; }
    op_ldsl_fpscr(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.fpscr = c.r32(io, a)? & FPSCR_MASK; c.r[n] = a.wrapping_add(4); }
    op_lds_fpscr(c, io, op, s) { c.fpscr = c.r[rn(op)] & FPSCR_MASK; }
    op_mulr(c, io, op, s) { let n = rn(op); c.r[n] = c.r[0].wrapping_mul(c.r[n]); } // MULR
    op_clipub(c, io, op, s) { let n = rn(op); let v = c.r[n]; if v > 0xFF { c.r[n] = 0xFF; c.sr |= SR_CS; } } // CLIPU.B
    op_clipuw(c, io, op, s) { let n = rn(op); let v = c.r[n]; if v > 0xFFFF { c.r[n] = 0xFFFF; c.sr |= SR_CS; } } // CLIPU.W
    op_clipsb(c, io, op, s) { let n = rn(op); let v = c.r[n] as i32; if v > 127 { c.r[n] = 127; c.sr |= SR_CS } else if v < -128 { c.r[n] = (-128i32) as u32; c.sr |= SR_CS } }
    op_clipsw(c, io, op, s) { let n = rn(op); let v = c.r[n] as i32; if v > 32767 { c.r[n] = 32767; c.sr |= SR_CS } else if v < -32768 { c.r[n] = (-32768i32) as u32; c.sr |= SR_CS } }
    op_divu(c, io, op, s) { if c.r[0] == 0 { return Err(Fault::DivideByZero { pc: c.cur_pc }); } let d = c.r[0]; c.r[rn(op)] /= d; } // DIVU
    op_divs(c, io, op, s) { if c.r[0] == 0 { return Err(Fault::DivideByZero { pc: c.cur_pc }); } let n = rn(op); c.r[n] = (c.r[n] as i32).wrapping_div(c.r[0] as i32) as u32; } // DIVS
    op_movb_r0p(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.w8(io, a, c.r[0])?; c.r[n] = a.wrapping_add(1); }
    op_movw_r0p(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.w16(io, a, c.r[0])?; c.r[n] = a.wrapping_add(2); }
    op_movl_r0p(c, io, op, s) { let n = rn(op); let a = c.r[n]; c.w32(io, a, c.r[0])?; c.r[n] = a.wrapping_add(4); }
    op_movb_mr0(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(1); c.r[0] = sext8(c.r8(io, a)?); c.r[n] = a; }
    op_movw_mr0(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(2); c.r[0] = sext16(c.r16(io, a)?); c.r[n] = a; }
    op_movl_mr0(c, io, op, s) { let n = rn(op); let a = c.r[n].wrapping_sub(4); c.r[0] = c.r32(io, a)?; c.r[n] = a; }
    op_bank(c, io, op, s) {
        // STBANK R0,@Rn / LDBANK @Rm,R0: the selector's bits 15-7 give the bank and 6-2 the entry; other bits are
        // ignored. A bank or entry that does not exist reads as 0 and ignores writes (undefined in the manual).
        // The firmware reaches this when an interrupt nests inside the RTOS entry before IMASK is raised
        // (the interrupted code resumes with a stale R4); on the chip the stray value only reaches GBR, which the
        // handlers reload themselves.
        let low = op & 0xFF;
        let sel = c.r[rn(op)];
        let (bn, en) = (((sel >> 7) & 511) as usize, ((sel >> 2) & 31) as usize);
        if bn >= 15 || en >= 20 {
            c.bank_anomalies += 1;
            if low == 0xE5 { c.r[0] = 0; }
            return Ok(());
        }
        let r0 = c.r[0];
        let b = &mut c.banks[bn];
        let slot: &mut u32 = match en { 0..=14 => &mut b.r[en], 15 => &mut b.mach, 16 => &mut b.vto, 17 => &mut b.pr, 18 => &mut b.gbr, _ => &mut b.macl };
        if low == 0xE1 { *slot = r0 } else { let v = *slot; c.r[0] = v; }
    }
    op_movmu_push(c, io, op, s) { for i in (rn(op)..=15).rev() { let v = if i == 15 { c.pr } else { c.r[i] }; let a = c.r[15].wrapping_sub(4); c.w32(io, a, v)?; c.r[15] = a; } } // MOVMU.L Rm,@-R15
    op_movml_push(c, io, op, s) { for i in (0..=rn(op)).rev() { let v = if i == 15 { c.pr } else { c.r[i] }; let a = c.r[15].wrapping_sub(4); c.w32(io, a, v)?; c.r[15] = a; } } // MOVML.L Rm,@-R15
    op_movmu_pop(c, io, op, s) { for i in rn(op)..=15 { let a = c.r[15]; let v = c.r32(io, a)?; c.r[15] = a.wrapping_add(4); if i == 15 { c.pr = v } else { c.r[i] = v } } } // MOVMU.L @R15+,Rn
    op_movml_pop(c, io, op, s) { for i in 0..=rn(op) { let a = c.r[15]; let v = c.r32(io, a)?; c.r[15] = a.wrapping_add(4); if i == 15 { c.pr = v } else { c.r[i] = v } } } // MOVML.L @R15+,Rn

    // ----- 0101 -----
    op_movl_ldisp(c, io, op, s) { let a = c.r[rm(op)].wrapping_add((op & 15) * 4); c.r[rn(op)] = c.r32(io, a)?; }

    // ----- 0110 -----
    op_movb_l(c, io, op, s) { let a = c.r[rm(op)]; c.r[rn(op)] = sext8(c.r8(io, a)?); }
    op_movw_l(c, io, op, s) { let a = c.r[rm(op)]; c.r[rn(op)] = sext16(c.r16(io, a)?); }
    op_movl_l(c, io, op, s) { let a = c.r[rm(op)]; c.r[rn(op)] = c.r32(io, a)?; }
    op_mov(c, io, op, s) { c.r[rn(op)] = c.r[rm(op)]; }
    op_movb_p(c, io, op, s) { let (n, m) = (rn(op), rm(op)); let a = c.r[m]; let v = sext8(c.r8(io, a)?); if n != m { c.r[m] = a.wrapping_add(1); } c.r[n] = v; }
    op_movw_p(c, io, op, s) { let (n, m) = (rn(op), rm(op)); let a = c.r[m]; let v = sext16(c.r16(io, a)?); if n != m { c.r[m] = a.wrapping_add(2); } c.r[n] = v; }
    op_movl_p(c, io, op, s) { let (n, m) = (rn(op), rm(op)); let a = c.r[m]; let v = c.r32(io, a)?; if n != m { c.r[m] = a.wrapping_add(4); } c.r[n] = v; }
    op_not(c, io, op, s) { c.r[rn(op)] = !c.r[rm(op)]; }
    op_swapb(c, io, op, s) { let v = c.r[rm(op)]; c.r[rn(op)] = (v & 0xFFFF_0000) | ((v & 0xFF) << 8) | ((v >> 8) & 0xFF); }
    op_swapw(c, io, op, s) { c.r[rn(op)] = c.r[rm(op)].rotate_left(16); }
    op_negc(c, io, op, s) {
        let tmp = 0u32.wrapping_sub(c.r[rm(op)]); let res = tmp.wrapping_sub(c.t() as u32);
        c.set_t(0 < tmp || tmp < res); c.r[rn(op)] = res;
    }
    op_neg(c, io, op, s) { c.r[rn(op)] = 0u32.wrapping_sub(c.r[rm(op)]); }
    op_extub(c, io, op, s) { c.r[rn(op)] = c.r[rm(op)] & 0xFF; }
    op_extuw(c, io, op, s) { c.r[rn(op)] = c.r[rm(op)] & 0xFFFF; }
    op_extsb(c, io, op, s) { c.r[rn(op)] = sext8(c.r[rm(op)]); }
    op_extsw(c, io, op, s) { c.r[rn(op)] = sext16(c.r[rm(op)]); }

    // ----- 0111 -----
    op_add_imm(c, io, op, s) { let n = rn(op); c.r[n] = c.r[n].wrapping_add(sext8(op)); }

    // ----- 1000 -----
    op_movb_s4(c, io, op, s) { let a = c.r[rm(op)].wrapping_add(op & 15); c.w8(io, a, c.r[0])?; }
    op_movw_s4(c, io, op, s) { let a = c.r[rm(op)].wrapping_add((op & 15) * 2); c.w16(io, a, c.r[0])?; }
    op_movb_l4(c, io, op, s) { let a = c.r[rm(op)].wrapping_add(op & 15); c.r[0] = sext8(c.r8(io, a)?); }
    op_movw_l4(c, io, op, s) { let a = c.r[rm(op)].wrapping_add((op & 15) * 2); c.r[0] = sext16(c.r16(io, a)?); }
    op_jsrn_tbr(c, io, op, s) { // JSR/N @@(disp8,TBR)
        no_slot(c, op, s)?;
        let a = c.tbr.wrapping_add((op & 0xFF) * 4);
        let t = c.r32(io, a)?;
        c.pr = c.cur_pc.wrapping_add(2); c.pc = t;
        c.traced_call();
    }
    op_bclr_bset(c, io, op, s) { // BCLR/BSET #imm3,Rn
        let r = ((op >> 4) & 15) as usize; let bit = 1 << (op & 7);
        if op & 8 == 0 { c.r[r] &= !bit } else { c.r[r] |= bit }
    }
    op_bst_bld(c, io, op, s) { // BST/BLD #imm3,Rn
        let r = ((op >> 4) & 15) as usize; let bit = 1 << (op & 7);
        if op & 8 == 0 { if c.t() { c.r[r] |= bit } else { c.r[r] &= !bit } } else { let t = c.r[r] & bit != 0; c.set_t(t); }
    }
    op_cmpeq_imm(c, io, op, s) { let t = c.r[0] == sext8(op); c.set_t(t); }
    op_bt_bf(c, io, op, s) { // BT / BF
        no_slot(c, op, s)?;
        let pc = c.cur_pc;
        if c.t() == (rn(op) == 0x9) { c.pc = pc.wrapping_add(4).wrapping_add(sext8(op) << 1); c.traced_branch(pc); }
    }
    op_bts_bfs(c, io, op, s) { // BT/S / BF/S
        let pc = c.cur_pc;
        if s { return Err(Fault::IllegalSlot { pc, op }); }
        let taken = c.t() == (rn(op) == 0xD);
        let t = if taken { pc.wrapping_add(4).wrapping_add(sext8(op) << 1) } else { pc.wrapping_add(4) };
        c.slot(io, pc.wrapping_add(2))?;
        c.pc = t;
        if taken { c.traced_branch(pc); }
    }

    // ----- 1001 / 1101 -----
    op_movw_pc(c, io, op, s) { let a = c.cur_pc.wrapping_add(4).wrapping_add((op & 0xFF) * 2); c.r[rn(op)] = sext16(c.r16(io, a)?); }
    op_movl_pc(c, io, op, s) { let a = (c.cur_pc.wrapping_add(4) & !3).wrapping_add((op & 0xFF) * 4); c.r[rn(op)] = c.r32(io, a)?; }

    // ----- 1010 / 1011 -----
    op_bra(c, io, op, s) { let pc = c.cur_pc; let t = pc.wrapping_add(4).wrapping_add(sext12(op) << 1); c.branch_delayed(io, pc, t, s, op)?; c.traced_branch(pc); }
    op_bsr(c, io, op, s) { let pc = c.cur_pc; let t = pc.wrapping_add(4).wrapping_add(sext12(op) << 1); c.pr = pc.wrapping_add(4); c.branch_delayed(io, pc, t, s, op)?; c.traced_call(); }

    // ----- 1100 -----
    op_movb_sgbr(c, io, op, s) { let a = c.gbr.wrapping_add(op & 0xFF); c.w8(io, a, c.r[0])?; }
    op_movw_sgbr(c, io, op, s) { let a = c.gbr.wrapping_add((op & 0xFF) * 2); c.w16(io, a, c.r[0])?; }
    op_movl_sgbr(c, io, op, s) { let a = c.gbr.wrapping_add((op & 0xFF) * 4); c.w32(io, a, c.r[0])?; }
    op_trapa(c, io, op, s) { no_slot(c, op, s)?; let next = c.cur_pc.wrapping_add(2); c.general_exception(io, op & 0xFF, next)?; } // TRAPA
    op_movb_lgbr(c, io, op, s) { let a = c.gbr.wrapping_add(op & 0xFF); c.r[0] = sext8(c.r8(io, a)?); }
    op_movw_lgbr(c, io, op, s) { let a = c.gbr.wrapping_add((op & 0xFF) * 2); c.r[0] = sext16(c.r16(io, a)?); }
    op_movl_lgbr(c, io, op, s) { let a = c.gbr.wrapping_add((op & 0xFF) * 4); c.r[0] = c.r32(io, a)?; }
    op_mova(c, io, op, s) { c.r[0] = (c.cur_pc.wrapping_add(4) & !3).wrapping_add((op & 0xFF) * 4); }
    op_tst_imm(c, io, op, s) { let t = c.r[0] & (op & 0xFF) == 0; c.set_t(t); }
    op_and_imm(c, io, op, s) { c.r[0] &= op & 0xFF; }
    op_xor_imm(c, io, op, s) { c.r[0] ^= op & 0xFF; }
    op_or_imm(c, io, op, s) { c.r[0] |= op & 0xFF; }
    op_tstb_gbr(c, io, op, s) { let a = c.gbr.wrapping_add(c.r[0]); let v = c.r8(io, a)?; c.set_t(v & (op & 0xFF) == 0); }
    op_andb_gbr(c, io, op, s) { let a = c.gbr.wrapping_add(c.r[0]); let v = c.r8(io, a)?; c.w8(io, a, v & (op & 0xFF))?; }
    op_xorb_gbr(c, io, op, s) { let a = c.gbr.wrapping_add(c.r[0]); let v = c.r8(io, a)?; c.w8(io, a, v ^ (op & 0xFF))?; }
    op_orb_gbr(c, io, op, s) { let a = c.gbr.wrapping_add(c.r[0]); let v = c.r8(io, a)?; c.w8(io, a, v | (op & 0xFF))?; }

    // ----- 1110 -----
    op_mov_imm(c, io, op, s) { c.r[rn(op)] = sext8(op); }

    // ----- 1111 -----
    op_fpu(c, io, op, s) { let pc = c.cur_pc; c.fpu(io, op, pc, rn(op), rm(op))?; }
}

/// The handler of an opcode: the structure of the former `exec` match, arm by arm.
const fn decode(op: u32) -> K {
    let n = (op >> 8) & 15;
    let m = (op >> 4) & 15;
    match op >> 12 {
        0x0 => match op & 0xF {
            0x0 | 0x1 => K::op_movi20,
            0x2 => match m { 0 => K::op_stc_sr, 1 => K::op_stc_gbr, 2 => K::op_stc_vbr, _ => K::op_illegal },
            0x3 => match m { 0 => K::op_bsrf, 2 => K::op_braf, 8 => K::op_nop, _ => K::op_illegal },
            0x4 => K::op_movb_s0,
            0x5 => K::op_movw_s0,
            0x6 => K::op_movl_s0,
            0x7 => K::op_mull,
            0x8 => match op { 0x0008 => K::op_clrt, 0x0018 => K::op_sett, 0x0028 => K::op_clrmac, 0x0048 => K::op_clrs, 0x0058 => K::op_sets, 0x0068 => K::op_nott, _ => K::op_illegal },
            0x9 => match m { 0 if n == 0 => K::op_nop, 1 if n == 0 => K::op_div0u, 2 => K::op_movt, 3 => K::op_movrt, _ => K::op_illegal },
            0xA => match m { 0 => K::op_sts_mach, 1 => K::op_sts_macl, 2 => K::op_sts_pr, 4 => K::op_stc_tbr, 5 => K::op_sts_fpul, 6 => K::op_sts_fpscr, _ => K::op_illegal },
            0xB => match op {
                0x000B => K::op_rts,
                0x001B => K::op_sleep,
                0x002B => K::op_rte,
                0x005B => K::op_resbank,
                0x006B => K::op_rtsn,
                _ if op & 0xF0FF == 0x007B => K::op_rtvn,
                _ => K::op_illegal,
            },
            0xC => K::op_movb_l0,
            0xD => K::op_movw_l0,
            0xE => K::op_movl_l0,
            _ => K::op_macl,
        },
        0x1 => K::op_movl_sdisp,
        0x2 => match op & 0xF {
            0x0 => K::op_movb_s, 0x1 => K::op_movw_s, 0x2 => K::op_movl_s,
            0x4 => K::op_movb_m, 0x5 => K::op_movw_m, 0x6 => K::op_movl_m,
            0x7 => K::op_div0s, 0x8 => K::op_tst, 0x9 => K::op_and, 0xA => K::op_xor, 0xB => K::op_or,
            0xC => K::op_cmpstr, 0xD => K::op_xtrct, 0xE => K::op_muluw, 0xF => K::op_mulsw,
            _ => K::op_illegal,
        },
        0x3 => match op & 0xF {
            0x0 => K::op_cmpeq, 0x1 => K::op_disp12, 0x2 => K::op_cmphs, 0x3 => K::op_cmpge, 0x4 => K::op_div1, 0x5 => K::op_dmulu,
            0x6 => K::op_cmphi, 0x7 => K::op_cmpgt, 0x8 => K::op_sub, 0x9 => K::op_bitops, 0xA => K::op_subc, 0xB => K::op_subv,
            0xC => K::op_add, 0xD => K::op_dmuls, 0xE => K::op_addc, _ => K::op_addv,
        },
        0x4 => match op & 0xF {
            0xC => K::op_shad,
            0xD => K::op_shld,
            0xF => K::op_macw,
            _ => match op & 0xFF {
                0x00 => K::op_shll, 0x01 => K::op_shlr, 0x02 => K::op_stsl_mach, 0x03 => K::op_stcl_sr, 0x04 => K::op_rotl, 0x05 => K::op_rotr,
                0x06 => K::op_ldsl_mach, 0x07 => K::op_ldcl_sr, 0x08 => K::op_shll2, 0x09 => K::op_shlr2, 0x0A => K::op_lds_mach, 0x0B => K::op_jsr,
                0x0E => K::op_ldc_sr, 0x10 => K::op_dt, 0x11 => K::op_cmppz, 0x12 => K::op_stsl_macl, 0x13 => K::op_stcl_gbr, 0x15 => K::op_cmppl,
                0x16 => K::op_ldsl_macl, 0x17 => K::op_ldcl_gbr, 0x18 => K::op_shll8, 0x19 => K::op_shlr8, 0x1A => K::op_lds_macl, 0x1B => K::op_tas,
                0x1E => K::op_ldc_gbr, 0x20 => K::op_shll, 0x21 => K::op_shar, 0x22 => K::op_stsl_pr, 0x23 => K::op_stcl_vbr, 0x24 => K::op_rotcl,
                0x25 => K::op_rotcr, 0x26 => K::op_ldsl_pr, 0x27 => K::op_ldcl_vbr, 0x28 => K::op_shll16, 0x29 => K::op_shlr16, 0x2A => K::op_lds_pr,
                0x2B => K::op_jmp, 0x2E => K::op_ldc_vbr, 0x4A => K::op_ldc_tbr, 0x4B => K::op_jsrn, 0x52 => K::op_stsl_fpul, 0x56 => K::op_ldsl_fpul,
                0x5A => K::op_lds_fpul, 0x62 => K::op_stsl_fpscr, 0x66 => K::op_ldsl_fpscr, 0x6A => K::op_lds_fpscr, 0x80 => K::op_mulr,
                0x81 => K::op_clipub, 0x85 => K::op_clipuw, 0x91 => K::op_clipsb, 0x95 => K::op_clipsw, 0x84 => K::op_divu, 0x94 => K::op_divs,
                0x8B => K::op_movb_r0p, 0x9B => K::op_movw_r0p, 0xAB => K::op_movl_r0p, 0xCB => K::op_movb_mr0, 0xDB => K::op_movw_mr0,
                0xEB => K::op_movl_mr0, 0xE1 | 0xE5 => K::op_bank, 0xF0 => K::op_movmu_push, 0xF1 => K::op_movml_push, 0xF4 => K::op_movmu_pop,
                0xF5 => K::op_movml_pop,
                _ => K::op_illegal,
            },
        },
        0x5 => K::op_movl_ldisp,
        0x6 => match op & 0xF {
            0x0 => K::op_movb_l, 0x1 => K::op_movw_l, 0x2 => K::op_movl_l, 0x3 => K::op_mov, 0x4 => K::op_movb_p, 0x5 => K::op_movw_p,
            0x6 => K::op_movl_p, 0x7 => K::op_not, 0x8 => K::op_swapb, 0x9 => K::op_swapw, 0xA => K::op_negc, 0xB => K::op_neg,
            0xC => K::op_extub, 0xD => K::op_extuw, 0xE => K::op_extsb, _ => K::op_extsw,
        },
        0x7 => K::op_add_imm,
        0x8 => match n {
            0x0 => K::op_movb_s4, 0x1 => K::op_movw_s4, 0x4 => K::op_movb_l4, 0x5 => K::op_movw_l4, 0x3 => K::op_jsrn_tbr,
            0x6 => K::op_bclr_bset, 0x7 => K::op_bst_bld, 0x8 => K::op_cmpeq_imm, 0x9 | 0xB => K::op_bt_bf, 0xD | 0xF => K::op_bts_bfs,
            _ => K::op_illegal,
        },
        0x9 => K::op_movw_pc,
        0xA => K::op_bra,
        0xB => K::op_bsr,
        0xC => match n {
            0x0 => K::op_movb_sgbr, 0x1 => K::op_movw_sgbr, 0x2 => K::op_movl_sgbr, 0x3 => K::op_trapa, 0x4 => K::op_movb_lgbr,
            0x5 => K::op_movw_lgbr, 0x6 => K::op_movl_lgbr, 0x7 => K::op_mova, 0x8 => K::op_tst_imm, 0x9 => K::op_and_imm,
            0xA => K::op_xor_imm, 0xB => K::op_or_imm, 0xC => K::op_tstb_gbr, 0xD => K::op_andb_gbr, 0xE => K::op_xorb_gbr, _ => K::op_orb_gbr,
        },
        0xD => K::op_movl_pc,
        0xE => K::op_mov_imm,
        _ => K::op_fpu,
    }
}

#[inline]
pub fn shad(v: u32, count: u32) -> u32 {
    let c = count as i32;
    if c >= 0 { v << (c & 31) }
    else if c & 31 == 0 { if (v as i32) < 0 { 0xFFFF_FFFF } else { 0 } }
    else { ((v as i32) >> (((!c) & 31) + 1)) as u32 }
}
#[inline]
pub fn shld(v: u32, count: u32) -> u32 {
    let c = count as i32;
    if c >= 0 { v << (c & 31) }
    else if c & 31 == 0 { 0 }
    else { v >> (((!c) & 31) + 1) }
}

// ----- float helpers -----
#[inline] fn fz32(v: f32, dn: bool) -> f32 { if dn && v.is_subnormal() { 0.0f32.copysign(v) } else { v } }
#[inline] fn fz64(v: f64, dn: bool) -> f64 { if dn && v.is_subnormal() { 0.0f64.copysign(v) } else { v } }

/// Narrow an exact value `hi + lo` (|lo| tiny vs hi, as from an error-free transform) to f32 toward zero.
fn narrow_rz32(hi: f64, lo: f64) -> f32 {
    let mut f = hi as f32;
    if !f.is_finite() { return if hi.is_finite() { f32::MAX.copysign(hi as f32) } else { f }; }
    // Compare candidate with the exact value; step toward zero if it lies farther from zero.
    let diff = (f as f64 - hi) - lo; // candidate - exact
    if f > 0.0 && diff > 0.0 || f < 0.0 && diff < 0.0 { f = f32::from_bits(f.to_bits() - 1); }
    else if f == 0.0 && (hi + lo) != 0.0 && false { }
    f
}
fn two_sum(a: f64, b: f64) -> (f64, f64) { let s = a + b; let bb = s - a; (s, (a - (s - bb)) + (b - bb)) }

fn arith32(k: u32, a: f32, b: f32, rz: bool) -> f32 {
    if !rz { return match k { 0 => a + b, 1 => a - b, 2 => a * b, _ => a / b }; }
    let (x, y) = (a as f64, b as f64);
    match k {
        0 => { let (s, e) = two_sum(x, y); narrow_rz32(s, e) }
        1 => { let (s, e) = two_sum(x, -y); narrow_rz32(s, e) }
        2 => narrow_rz32(x * y, 0.0), // exact in f64
        _ => { let q = x / y; if !q.is_finite() || q == 0.0 { return q as f32; } let r = (-q).mul_add(y, x); narrow_rz32(q, r / y) }
    }
}
fn fmac32(a: f32, b: f32, c: f32, rz: bool) -> f32 {
    if !rz { return a * b + c; }
    let p = a as f64 * b as f64; let (s, e) = two_sum(p, c as f64); narrow_rz32(s, e)
}
fn sqrt32(a: f32, rz: bool) -> f32 {
    if !rz || a <= 0.0 || !a.is_finite() { return a.sqrt(); }
    let s = (a as f64).sqrt(); let r = (-s).mul_add(s, a as f64); narrow_rz32(s, r / (2.0 * s))
}
fn int_to_f32(v: i32, rz: bool) -> f32 { if rz { narrow_rz32(v as f64, 0.0) } else { v as f32 } }
fn f64_to_f32(v: f64, rz: bool) -> f32 { if rz { narrow_rz32(v, 0.0) } else { v as f32 } }

/// Double precision toward zero: compute nearest then step back if it overshot the exact result.
fn rz64(r: f64, err_sign: f64) -> f64 {
    // err_sign = exact - r ; if r is farther from zero than exact, step toward zero.
    if r.is_finite() && r != 0.0 && ((r > 0.0 && err_sign < 0.0) || (r < 0.0 && err_sign > 0.0)) {
        f64::from_bits(r.to_bits() - 1)
    } else { r }
}
fn arith64(k: u32, a: f64, b: f64, rz: bool) -> f64 {
    match k {
        0 => { let (s, e) = two_sum(a, b); if rz { rz64(s, e) } else { s } }
        1 => { let (s, e) = two_sum(a, -b); if rz { rz64(s, e) } else { s } }
        2 => { let p = a * b; if rz { rz64(p, a.mul_add(b, -p)) } else { p } }
        _ => { let q = a / b; if rz && q.is_finite() && q != 0.0 { rz64(q, (-q).mul_add(b, a) / b) } else { q } }
    }
}
fn sqrt64(a: f64, rz: bool) -> f64 {
    let s = a.sqrt();
    if rz && s.is_finite() && s > 0.0 { rz64(s, (-s).mul_add(s, a)) } else { s }
}
fn ftrc(v: f64) -> i32 {
    if v.is_nan() { return i32::MIN; } // SH: NaN -> H'8000_0000 (invalid)
    if v >= 2147483647.0 { i32::MAX } else if v <= -2147483648.0 { i32::MIN } else { v.trunc() as i32 }
}
