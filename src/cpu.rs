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
    /// LDBANK/STBANK with a selector naming no bank entry (see `exec` 0xE1/0xE5).
    #[serde(skip)]
    pub bank_anomalies: u64,
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
            float_mode: FloatMode::Hardware, sleeping: false, cur_pc: 0, watch: None, watch_log: Vec::new(), bank_anomalies: 0, trace: None,
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
    #[inline]
    pub fn read<M: Mmio>(&mut self, io: &mut M, a: u32, n: usize) -> Result<u32, Fault> {
        if a as usize & (n - 1) != 0 { return Err(Fault::Address { pc: self.cur_pc, addr: a, size: n, write: false }); }
        let v = match self.mem.read_plain(a, n) { Some(v) => v as u32, None => io.read(self.cur_pc, a, n) as u32 };
        if let Some((s, e)) = self.watch { if a < e && a + n as u32 > s && self.watch_log.len() < 100_000 { self.watch_log.push((self.cur_pc, a, n, false, v)); } }
        Ok(v)
    }
    #[inline]
    pub fn write<M: Mmio>(&mut self, io: &mut M, a: u32, n: usize, v: u32) -> Result<(), Fault> {
        if a as usize & (n - 1) != 0 { return Err(Fault::Address { pc: self.cur_pc, addr: a, size: n, write: true }); }
        if let Some((s, e)) = self.watch { if a < e && a + n as u32 > s && self.watch_log.len() < 100_000 { self.watch_log.push((self.cur_pc, a, n, true, v)); } }
        if !self.mem.write_plain(a, n, v as u64) {
            io.write(self.cur_pc, a, n, v as u64);
            if crate::periph::nor::Nor::handles(a) { self.mem.flash_cmd_mode = io.flash_cmd_mode(); }
        }
        Ok(())
    }
    #[inline] fn r8<M: Mmio>(&mut self, io: &mut M, a: u32) -> Result<u32, Fault> { self.read(io, a, 1) }
    #[inline] fn r16<M: Mmio>(&mut self, io: &mut M, a: u32) -> Result<u32, Fault> { self.read(io, a, 2) }
    #[inline] fn r32<M: Mmio>(&mut self, io: &mut M, a: u32) -> Result<u32, Fault> { self.read(io, a, 4) }
    #[inline] fn w8<M: Mmio>(&mut self, io: &mut M, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 1, v & 0xFF) }
    #[inline] fn w16<M: Mmio>(&mut self, io: &mut M, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 2, v & 0xFFFF) }
    #[inline] fn w32<M: Mmio>(&mut self, io: &mut M, a: u32, v: u32) -> Result<(), Fault> { self.write(io, a, 4, v) }

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

    fn general_exception<M: Mmio>(&mut self, io: &mut M, vector: u32, return_pc: u32) -> Result<(), Fault> {
        let sp = self.r[15];
        self.w32(io, sp.wrapping_sub(4), self.sr)?;
        self.w32(io, sp.wrapping_sub(8), return_pc)?;
        self.r[15] = sp.wrapping_sub(8);
        self.pc = self.r32(io, self.vbr.wrapping_add(vector * 4))?;
        if let Some(t) = self.trace.as_deref_mut() { t.interrupt(self.pc, return_pc, sp); }
        Ok(())
    }

    // ---------- execution ----------
    pub fn step<M: Mmio>(&mut self, io: &mut M) -> Result<(), Fault> {
        let pc = self.pc;
        self.cur_pc = pc;
        let op = self.fetch(pc)?;
        self.exec(io, op, pc, false)
    }

    /// Execute the slot instruction of a delayed branch at `slot_pc`.
    fn slot<M: Mmio>(&mut self, io: &mut M, slot_pc: u32) -> Result<(), Fault> {
        let op = self.fetch(slot_pc)?;
        self.cur_pc = slot_pc;
        self.exec(io, op, slot_pc, true)
    }

    #[inline]
    fn branch_delayed<M: Mmio>(&mut self, io: &mut M, pc: u32, target: u32, in_slot: bool, op: u32) -> Result<(), Fault> {
        if in_slot { return Err(Fault::IllegalSlot { pc, op }); }
        self.slot(io, pc.wrapping_add(2))?;
        self.pc = target;
        Ok(())
    }

    fn exec<M: Mmio>(&mut self, io: &mut M, op: u32, pc: u32, in_slot: bool) -> Result<(), Fault> {
        let n = ((op >> 8) & 15) as usize;
        let m = ((op >> 4) & 15) as usize;
        let next = pc.wrapping_add(2);
        // Every non-branch instruction falls through to pc+2 unless it sets self.pc itself.
        self.pc = next;
        let illegal = Fault::Illegal { pc, op };
        let no_slot = |cpu_in_slot: bool| if cpu_in_slot { Err(Fault::IllegalSlot { pc, op }) } else { Ok(()) };
        match op >> 12 {
            0x0 => match op & 0xF {
                0x0 | 0x1 => {
                    // MOVI20 / MOVI20S (32-bit)
                    no_slot(in_slot)?;
                    let lo = self.fetch(pc.wrapping_add(2))?;
                    let imm = ((((op >> 4) & 15) << 16) | lo) as u32;
                    let v = ((imm << 12) as i32 >> 12) as u32;
                    self.r[n] = if op & 1 == 0 { v } else { v << 8 };
                    self.pc = pc.wrapping_add(4);
                }
                0x2 => match m {
                    0 => self.r[n] = self.sr,
                    1 => self.r[n] = self.gbr,
                    2 => self.r[n] = self.vbr,
                    _ => return Err(illegal),
                },
                0x3 => match m {
                    0 => { // BSRF Rn
                        let t = next.wrapping_add(2).wrapping_add(self.r[n]);
                        self.pr = pc.wrapping_add(4);
                        self.branch_delayed(io, pc, t, in_slot, op)?;
                        self.traced_call();
                        return Ok(());
                    }
                    2 => { // BRAF Rn
                        let t = next.wrapping_add(2).wrapping_add(self.r[n]);
                        self.branch_delayed(io, pc, t, in_slot, op)?;
                        self.traced_branch(pc);
                        return Ok(());
                    }
                    8 => {} // PREF @Rn (no cache model)
                    _ => return Err(illegal),
                },
                0x4 => { let a = self.r[n].wrapping_add(self.r[0]); self.w8(io, a, self.r[m])?; }
                0x5 => { let a = self.r[n].wrapping_add(self.r[0]); self.w16(io, a, self.r[m])?; }
                0x6 => { let a = self.r[n].wrapping_add(self.r[0]); self.w32(io, a, self.r[m])?; }
                0x7 => self.macl = self.r[n].wrapping_mul(self.r[m]),
                0x8 => match op {
                    0x0008 => self.sr &= !SR_T,
                    0x0018 => self.sr |= SR_T,
                    0x0028 => { self.mach = 0; self.macl = 0; }
                    0x0048 => self.sr &= !SR_S,
                    0x0058 => self.sr |= SR_S,
                    0x0068 => self.sr ^= SR_T, // NOTT (T only)
                    _ => return Err(illegal),
                },
                0x9 => match m {
                    0 if n == 0 => {}
                    1 if n == 0 => { self.sr &= !(SR_M | SR_Q | SR_T); }
                    2 => self.r[n] = self.sr & SR_T,
                    3 => self.r[n] = (self.sr & SR_T) ^ 1, // MOVRT
                    _ => return Err(illegal),
                },
                0xA => match m {
                    0 => self.r[n] = self.mach,
                    1 => self.r[n] = self.macl,
                    2 => self.r[n] = self.pr,
                    4 => self.r[n] = self.tbr,
                    5 => self.r[n] = self.fpul,
                    6 => self.r[n] = self.fpscr,
                    _ => return Err(illegal),
                },
                0xB => match op {
                    0x000B => { let t = self.pr; self.branch_delayed(io, pc, t, in_slot, op)?; self.traced_ret(); return Ok(()); } // RTS
                    0x001B => { self.sleeping = true; } // SLEEP
                    0x002B => { // RTE
                        if in_slot { return Err(Fault::IllegalSlot { pc, op }); }
                        let sp = self.r[15];
                        let t = self.r32(io, sp)?;
                        let sr = self.r32(io, sp.wrapping_add(4))?;
                        self.r[15] = sp.wrapping_add(8);
                        self.sr = sr & SR_MASK;
                        self.slot(io, next)?;
                        self.pc = t;
                        if let Some(tr) = self.trace.as_deref_mut() { tr.rte(self.pc, self.r[15]); }
                    }
                    0x005B => { no_slot(in_slot)?; self.resbank(io)?; } // RESBANK
                    0x006B => { no_slot(in_slot)?; self.pc = self.pr; self.traced_ret(); } // RTS/N
                    _ if op & 0xF0FF == 0x007B => { no_slot(in_slot)?; self.r[0] = self.r[n]; self.pc = self.pr; self.traced_ret(); } // RTV/N
                    _ => return Err(illegal),
                },
                0xC => { let a = self.r[m].wrapping_add(self.r[0]); self.r[n] = sext8(self.r8(io, a)?); }
                0xD => { let a = self.r[m].wrapping_add(self.r[0]); self.r[n] = sext16(self.r16(io, a)?); }
                0xE => { let a = self.r[m].wrapping_add(self.r[0]); self.r[n] = self.r32(io, a)?; }
                0xF => self.mac_l(io, n, m)?,
                _ => unreachable!(),
            },
            0x1 => { let a = self.r[n].wrapping_add((op & 15) * 4); self.w32(io, a, self.r[m])?; }
            0x2 => match op & 0xF {
                0x0 => { let a = self.r[n]; self.w8(io, a, self.r[m])?; }
                0x1 => { let a = self.r[n]; self.w16(io, a, self.r[m])?; }
                0x2 => { let a = self.r[n]; self.w32(io, a, self.r[m])?; }
                0x4 => { let a = self.r[n].wrapping_sub(1); self.w8(io, a, self.r[m])?; self.r[n] = a; }
                0x5 => { let a = self.r[n].wrapping_sub(2); self.w16(io, a, self.r[m])?; self.r[n] = a; }
                0x6 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.r[m])?; self.r[n] = a; }
                0x7 => { // DIV0S
                    let q = self.r[n] >> 31 != 0; let mm = self.r[m] >> 31 != 0;
                    self.sr = (self.sr & !(SR_Q | SR_M | SR_T)) | if q { SR_Q } else { 0 } | if mm { SR_M } else { 0 } | if q != mm { SR_T } else { 0 };
                }
                0x8 => { let t = self.r[n] & self.r[m] == 0; self.set_t(t); }
                0x9 => self.r[n] &= self.r[m],
                0xA => self.r[n] ^= self.r[m],
                0xB => self.r[n] |= self.r[m],
                0xC => { let x = self.r[n] ^ self.r[m]; let t = x & 0xFF == 0 || x & 0xFF00 == 0 || x & 0xFF_0000 == 0 || x & 0xFF00_0000 == 0; self.set_t(t); }
                0xD => self.r[n] = (self.r[n] >> 16) | (self.r[m] << 16),
                0xE => self.macl = (self.r[n] & 0xFFFF).wrapping_mul(self.r[m] & 0xFFFF),
                0xF => self.macl = ((self.r[n] as i16 as i32).wrapping_mul(self.r[m] as i16 as i32)) as u32,
                _ => return Err(illegal),
            },
            0x3 => match op & 0xF {
                0x0 => { let t = self.r[n] == self.r[m]; self.set_t(t); }
                0x2 => { let t = self.r[n] >= self.r[m]; self.set_t(t); }
                0x3 => { let t = self.r[n] as i32 >= self.r[m] as i32; self.set_t(t); }
                0x4 => self.div1(n, m),
                0x5 => { let p = self.r[n] as u64 * self.r[m] as u64; self.mach = (p >> 32) as u32; self.macl = p as u32; }
                0x6 => { let t = self.r[n] > self.r[m]; self.set_t(t); }
                0x7 => { let t = self.r[n] as i32 > self.r[m] as i32; self.set_t(t); }
                0x8 => self.r[n] = self.r[n].wrapping_sub(self.r[m]),
                0xA => { // SUBC
                    let (a, b, c) = (self.r[n], self.r[m], self.t() as u32);
                    let tmp1 = a.wrapping_sub(b); let res = tmp1.wrapping_sub(c);
                    self.set_t(a < tmp1 || tmp1 < res);
                    self.r[n] = res;
                }
                0xB => { // SUBV
                    let (a, b) = (self.r[n] as i32, self.r[m] as i32);
                    let (res, o) = a.overflowing_sub(b); self.r[n] = res as u32; self.set_t(o);
                }
                0xC => self.r[n] = self.r[n].wrapping_add(self.r[m]),
                0xD => { let p = (self.r[n] as i32 as i64) * (self.r[m] as i32 as i64); self.mach = (p >> 32) as u32; self.macl = p as u32; }
                0xE => { // ADDC
                    let (a, b, c) = (self.r[n], self.r[m], self.t() as u32);
                    let (s1, c1) = a.overflowing_add(b); let (s2, c2) = s1.overflowing_add(c);
                    self.r[n] = s2; self.set_t(c1 || c2);
                }
                0xF => { let (res, o) = (self.r[n] as i32).overflowing_add(self.r[m] as i32); self.r[n] = res as u32; self.set_t(o); }
                0x1 => { no_slot(in_slot)?; self.op32_disp12(io, op, pc)?; }
                0x9 => { no_slot(in_slot)?; self.op32_bitops(io, op, pc)?; }
                _ => return Err(illegal),
            },
            0x4 => self.group4(io, op, pc, n, m, in_slot)?,
            0x5 => { let a = self.r[m].wrapping_add((op & 15) * 4); self.r[n] = self.r32(io, a)?; }
            0x6 => match op & 0xF {
                0x0 => { let a = self.r[m]; self.r[n] = sext8(self.r8(io, a)?); }
                0x1 => { let a = self.r[m]; self.r[n] = sext16(self.r16(io, a)?); }
                0x2 => { let a = self.r[m]; self.r[n] = self.r32(io, a)?; }
                0x3 => self.r[n] = self.r[m],
                0x4 => { let a = self.r[m]; let v = sext8(self.r8(io, a)?); if n != m { self.r[m] = a.wrapping_add(1); } self.r[n] = v; }
                0x5 => { let a = self.r[m]; let v = sext16(self.r16(io, a)?); if n != m { self.r[m] = a.wrapping_add(2); } self.r[n] = v; }
                0x6 => { let a = self.r[m]; let v = self.r32(io, a)?; if n != m { self.r[m] = a.wrapping_add(4); } self.r[n] = v; }
                0x7 => self.r[n] = !self.r[m],
                0x8 => { let v = self.r[m]; self.r[n] = (v & 0xFFFF_0000) | ((v & 0xFF) << 8) | ((v >> 8) & 0xFF); }
                0x9 => self.r[n] = self.r[m].rotate_left(16),
                0xA => { // NEGC
                    let tmp = 0u32.wrapping_sub(self.r[m]); let res = tmp.wrapping_sub(self.t() as u32);
                    self.set_t(0 < tmp || tmp < res); self.r[n] = res;
                }
                0xB => self.r[n] = 0u32.wrapping_sub(self.r[m]),
                0xC => self.r[n] = self.r[m] & 0xFF,
                0xD => self.r[n] = self.r[m] & 0xFFFF,
                0xE => self.r[n] = sext8(self.r[m]),
                0xF => self.r[n] = sext16(self.r[m]),
                _ => unreachable!(),
            },
            0x7 => self.r[n] = self.r[n].wrapping_add(sext8(op)),
            0x8 => match n {
                0x0 => { let a = self.r[m].wrapping_add(op & 15); self.w8(io, a, self.r[0])?; }
                0x1 => { let a = self.r[m].wrapping_add((op & 15) * 2); self.w16(io, a, self.r[0])?; }
                0x4 => { let a = self.r[m].wrapping_add(op & 15); self.r[0] = sext8(self.r8(io, a)?); }
                0x5 => { let a = self.r[m].wrapping_add((op & 15) * 2); self.r[0] = sext16(self.r16(io, a)?); }
                0x3 => { // JSR/N @@(disp8,TBR)
                    no_slot(in_slot)?;
                    let a = self.tbr.wrapping_add((op & 0xFF) * 4);
                    let t = self.r32(io, a)?;
                    self.pr = next; self.pc = t;
                    self.traced_call();
                }
                0x6 => { // BCLR/BSET #imm3,Rn
                    let rn = ((op >> 4) & 15) as usize; let bit = 1 << (op & 7);
                    if op & 8 == 0 { self.r[rn] &= !bit } else { self.r[rn] |= bit }
                }
                0x7 => { // BST/BLD #imm3,Rn
                    let rn = ((op >> 4) & 15) as usize; let bit = 1 << (op & 7);
                    if op & 8 == 0 { if self.t() { self.r[rn] |= bit } else { self.r[rn] &= !bit } } else { let t = self.r[rn] & bit != 0; self.set_t(t); }
                }
                0x8 => { let t = self.r[0] == sext8(op); self.set_t(t); }
                0x9 | 0xB => { // BT / BF
                    no_slot(in_slot)?;
                    if self.t() == (n == 0x9) { self.pc = next.wrapping_add(2).wrapping_add(sext8(op) << 1); self.traced_branch(pc); }
                }
                0xD | 0xF => { // BT/S / BF/S
                    if in_slot { return Err(Fault::IllegalSlot { pc, op }); }
                    let taken = self.t() == (n == 0xD);
                    let t = if taken { next.wrapping_add(2).wrapping_add(sext8(op) << 1) } else { pc.wrapping_add(4) };
                    self.slot(io, next)?;
                    self.pc = t;
                    if taken { self.traced_branch(pc); }
                }
                _ => return Err(illegal),
            },
            0x9 => { let a = next.wrapping_add(2).wrapping_add((op & 0xFF) * 2); self.r[n] = sext16(self.r16(io, a)?); }
            0xA => { let t = next.wrapping_add(2).wrapping_add(sext12(op) << 1); self.branch_delayed(io, pc, t, in_slot, op)?; self.traced_branch(pc); return Ok(()); }
            0xB => { let t = next.wrapping_add(2).wrapping_add(sext12(op) << 1); self.pr = pc.wrapping_add(4); self.branch_delayed(io, pc, t, in_slot, op)?; self.traced_call(); return Ok(()); }
            0xC => {
                let d = op & 0xFF;
                match n {
                    0x0 => { let a = self.gbr.wrapping_add(d); self.w8(io, a, self.r[0])?; }
                    0x1 => { let a = self.gbr.wrapping_add(d * 2); self.w16(io, a, self.r[0])?; }
                    0x2 => { let a = self.gbr.wrapping_add(d * 4); self.w32(io, a, self.r[0])?; }
                    0x3 => { no_slot(in_slot)?; self.general_exception(io, d, next)?; } // TRAPA
                    0x4 => { let a = self.gbr.wrapping_add(d); self.r[0] = sext8(self.r8(io, a)?); }
                    0x5 => { let a = self.gbr.wrapping_add(d * 2); self.r[0] = sext16(self.r16(io, a)?); }
                    0x6 => { let a = self.gbr.wrapping_add(d * 4); self.r[0] = self.r32(io, a)?; }
                    0x7 => self.r[0] = (next.wrapping_add(2) & !3).wrapping_add(d * 4),
                    0x8 => { let t = self.r[0] & d == 0; self.set_t(t); }
                    0x9 => self.r[0] &= d,
                    0xA => self.r[0] ^= d,
                    0xB => self.r[0] |= d,
                    0xC => { let a = self.gbr.wrapping_add(self.r[0]); let v = self.r8(io, a)?; self.set_t(v & d == 0); }
                    0xD => { let a = self.gbr.wrapping_add(self.r[0]); let v = self.r8(io, a)?; self.w8(io, a, v & d)?; }
                    0xE => { let a = self.gbr.wrapping_add(self.r[0]); let v = self.r8(io, a)?; self.w8(io, a, v ^ d)?; }
                    0xF => { let a = self.gbr.wrapping_add(self.r[0]); let v = self.r8(io, a)?; self.w8(io, a, v | d)?; }
                    _ => unreachable!(),
                }
            }
            0xD => { let a = (next.wrapping_add(2) & !3).wrapping_add((op & 0xFF) * 4); self.r[n] = self.r32(io, a)?; }
            0xE => self.r[n] = sext8(op),
            0xF => self.fpu(io, op, pc, n, m)?,
            _ => unreachable!(),
        }
        Ok(())
    }

    fn group4<M: Mmio>(&mut self, io: &mut M, op: u32, pc: u32, n: usize, m: usize, in_slot: bool) -> Result<(), Fault> {
        let illegal = Fault::Illegal { pc, op };
        let next = pc.wrapping_add(2);
        match op & 0xF {
            0xC => { self.r[n] = shad(self.r[n], self.r[m]); return Ok(()); }
            0xD => { self.r[n] = shld(self.r[n], self.r[m]); return Ok(()); }
            0xF => { return self.mac_w(io, n, m); }
            _ => {}
        }
        let low = op & 0xFF;
        match low {
            0x00 => { self.set_t(self.r[n] >> 31 != 0); self.r[n] <<= 1; }
            0x01 => { self.set_t(self.r[n] & 1 != 0); self.r[n] >>= 1; }
            0x02 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.mach)?; self.r[n] = a; }
            0x03 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.sr)?; self.r[n] = a; }
            0x04 => { let t = self.r[n] >> 31; self.r[n] = (self.r[n] << 1) | t; self.set_t(t != 0); }
            0x05 => { let t = self.r[n] & 1; self.r[n] = (self.r[n] >> 1) | (t << 31); self.set_t(t != 0); }
            0x06 => { let a = self.r[n]; self.mach = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x07 => { let a = self.r[n]; self.sr = self.r32(io, a)? & SR_MASK; self.r[n] = a.wrapping_add(4); }
            0x08 => self.r[n] <<= 2,
            0x09 => self.r[n] >>= 2,
            0x0A => self.mach = self.r[n],
            0x0B => { let t = self.r[n]; self.pr = pc.wrapping_add(4); self.branch_delayed(io, pc, t, in_slot, op)?; self.traced_call(); return Ok(()); } // JSR
            0x0E => self.sr = self.r[n] & SR_MASK,
            0x10 => { self.r[n] = self.r[n].wrapping_sub(1); let t = self.r[n] == 0; self.set_t(t); }
            0x11 => { let t = self.r[n] as i32 >= 0; self.set_t(t); }
            0x12 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.macl)?; self.r[n] = a; }
            0x13 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.gbr)?; self.r[n] = a; }
            0x15 => { let t = self.r[n] as i32 > 0; self.set_t(t); }
            0x16 => { let a = self.r[n]; self.macl = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x17 => { let a = self.r[n]; self.gbr = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x18 => self.r[n] <<= 8,
            0x19 => self.r[n] >>= 8,
            0x1A => self.macl = self.r[n],
            0x1B => { let a = self.r[n]; let v = self.r8(io, a)?; self.set_t(v == 0); self.w8(io, a, v | 0x80)?; }
            0x1E => self.gbr = self.r[n],
            0x20 => { self.set_t(self.r[n] >> 31 != 0); self.r[n] <<= 1; }
            0x21 => { self.set_t(self.r[n] & 1 != 0); self.r[n] = ((self.r[n] as i32) >> 1) as u32; }
            0x22 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.pr)?; self.r[n] = a; }
            0x23 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.vbr)?; self.r[n] = a; }
            0x24 => { let t = self.r[n] >> 31; self.r[n] = (self.r[n] << 1) | self.t() as u32; self.set_t(t != 0); }
            0x25 => { let t = self.r[n] & 1; self.r[n] = (self.r[n] >> 1) | ((self.t() as u32) << 31); self.set_t(t != 0); }
            0x26 => { let a = self.r[n]; self.pr = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x27 => { let a = self.r[n]; self.vbr = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x28 => self.r[n] <<= 16,
            0x29 => self.r[n] >>= 16,
            0x2A => self.pr = self.r[n],
            0x2B => { let t = self.r[n]; return self.branch_delayed(io, pc, t, in_slot, op); } // JMP
            0x2E => self.vbr = self.r[n],
            0x4A => self.tbr = self.r[n],
            0x4B => { if in_slot { return Err(Fault::IllegalSlot { pc, op }); } self.pr = next; self.pc = self.r[n]; self.traced_call(); } // JSR/N
            0x52 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.fpul)?; self.r[n] = a; }
            0x56 => { let a = self.r[n]; self.fpul = self.r32(io, a)?; self.r[n] = a.wrapping_add(4); }
            0x5A => self.fpul = self.r[n],
            0x62 => { let a = self.r[n].wrapping_sub(4); self.w32(io, a, self.fpscr)?; self.r[n] = a; }
            0x66 => { let a = self.r[n]; self.fpscr = self.r32(io, a)? & FPSCR_MASK; self.r[n] = a.wrapping_add(4); }
            0x6A => self.fpscr = self.r[n] & FPSCR_MASK,
            0x80 => self.r[n] = self.r[0].wrapping_mul(self.r[n]), // MULR
            0x81 => { let v = self.r[n]; if v > 0xFF { self.r[n] = 0xFF; self.sr |= SR_CS; } } // CLIPU.B
            0x85 => { let v = self.r[n]; if v > 0xFFFF { self.r[n] = 0xFFFF; self.sr |= SR_CS; } } // CLIPU.W
            0x91 => { let v = self.r[n] as i32; if v > 127 { self.r[n] = 127; self.sr |= SR_CS } else if v < -128 { self.r[n] = (-128i32) as u32; self.sr |= SR_CS } }
            0x95 => { let v = self.r[n] as i32; if v > 32767 { self.r[n] = 32767; self.sr |= SR_CS } else if v < -32768 { self.r[n] = (-32768i32) as u32; self.sr |= SR_CS } }
            0x84 => { if self.r[0] == 0 { return Err(Fault::DivideByZero { pc }); } self.r[n] /= self.r[0]; } // DIVU
            0x94 => { if self.r[0] == 0 { return Err(Fault::DivideByZero { pc }); } self.r[n] = (self.r[n] as i32).wrapping_div(self.r[0] as i32) as u32; } // DIVS
            0x8B => { let a = self.r[n]; self.w8(io, a, self.r[0])?; self.r[n] = a.wrapping_add(1); }
            0x9B => { let a = self.r[n]; self.w16(io, a, self.r[0])?; self.r[n] = a.wrapping_add(2); }
            0xAB => { let a = self.r[n]; self.w32(io, a, self.r[0])?; self.r[n] = a.wrapping_add(4); }
            0xCB => { let a = self.r[n].wrapping_sub(1); self.r[0] = sext8(self.r8(io, a)?); self.r[n] = a; }
            0xDB => { let a = self.r[n].wrapping_sub(2); self.r[0] = sext16(self.r16(io, a)?); self.r[n] = a; }
            0xEB => { let a = self.r[n].wrapping_sub(4); self.r[0] = self.r32(io, a)?; self.r[n] = a; }
            0xE1 | 0xE5 => {
                // STBANK R0,@Rn / LDBANK @Rm,R0: the selector's bits 15-7 give the bank and 6-2 the entry; other bits are
                // ignored. A bank or entry that does not exist reads as 0 and ignores writes (undefined in the manual).
                // The firmware reaches this when an interrupt nests inside the RTOS entry before IMASK is raised
                // (the interrupted code resumes with a stale R4); on the chip the stray value only reaches GBR, which the
                // handlers reload themselves.
                let sel = self.r[n];
                let (bn, en) = (((sel >> 7) & 511) as usize, ((sel >> 2) & 31) as usize);
                if bn >= 15 || en >= 20 {
                    self.bank_anomalies += 1;
                    if low == 0xE5 { self.r[0] = 0; }
                    return Ok(());
                }
                let b = &mut self.banks[bn];
                let slot: &mut u32 = match en { 0..=14 => &mut b.r[en], 15 => &mut b.mach, 16 => &mut b.vto, 17 => &mut b.pr, 18 => &mut b.gbr, _ => &mut b.macl };
                if low == 0xE1 { *slot = self.r[0] } else { self.r[0] = *slot }
            }
            0xF0 => { for i in (n..=15).rev() { let v = if i == 15 { self.pr } else { self.r[i] }; let a = self.r[15].wrapping_sub(4); self.w32(io, a, v)?; self.r[15] = a; } } // MOVMU.L Rm,@-R15
            0xF1 => { for i in (0..=n).rev() { let v = if i == 15 { self.pr } else { self.r[i] }; let a = self.r[15].wrapping_sub(4); self.w32(io, a, v)?; self.r[15] = a; } } // MOVML.L Rm,@-R15
            0xF4 => { for i in n..=15 { let a = self.r[15]; let v = self.r32(io, a)?; self.r[15] = a.wrapping_add(4); if i == 15 { self.pr = v } else { self.r[i] = v } } } // MOVMU.L @R15+,Rn
            0xF5 => { for i in 0..=n { let a = self.r[15]; let v = self.r32(io, a)?; self.r[15] = a.wrapping_add(4); if i == 15 { self.pr = v } else { self.r[i] = v } } } // MOVML.L @R15+,Rn
            _ => return Err(illegal),
        }
        Ok(())
    }

    fn resbank<M: Mmio>(&mut self, io: &mut M) -> Result<(), Fault> {
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

    fn mac_l<M: Mmio>(&mut self, io: &mut M, n: usize, m: usize) -> Result<(), Fault> {
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

    fn mac_w<M: Mmio>(&mut self, io: &mut M, n: usize, m: usize) -> Result<(), Fault> {
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
    fn op32_disp12<M: Mmio>(&mut self, io: &mut M, op: u32, pc: u32) -> Result<(), Fault> {
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
    fn op32_bitops<M: Mmio>(&mut self, io: &mut M, op: u32, pc: u32) -> Result<(), Fault> {
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

    fn fpu<M: Mmio>(&mut self, io: &mut M, op: u32, pc: u32, n: usize, m: usize) -> Result<(), Fault> {
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

    fn fload<M: Mmio>(&mut self, io: &mut M, n: usize, a: u32, sz: bool) -> Result<(), Fault> {
        if sz { self.fr[n & 14] = self.r32(io, a)?; self.fr[n | 1] = self.r32(io, a.wrapping_add(4))?; } else { self.fr[n] = self.r32(io, a)?; }
        Ok(())
    }
    fn fstore<M: Mmio>(&mut self, io: &mut M, m: usize, a: u32, sz: bool) -> Result<(), Fault> {
        if sz { self.w32(io, a, self.fr[m & 14])?; self.w32(io, a.wrapping_add(4), self.fr[m | 1])?; } else { self.w32(io, a, self.fr[m])?; }
        Ok(())
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
