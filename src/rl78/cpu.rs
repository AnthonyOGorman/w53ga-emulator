//! RL78-S3 CPU core (RL78/G13 R5F100LLA, the unit's companion controller), written from the RL78 Family
//! User's Manual: Software (R01US0015EJ0230) instruction maps (tables 5-9..5-12) and operation list (table 5-7).
//!
//! Address space (R5F100xL): code flash 00000-7FFFF, 2nd SFR F0000-F07FF, data flash F1000-F2FFF,
//! mirror of code flash F3000-F7EFF (PMC.MAA selects 0xxxx/1xxxx), RAM F7F00-FFEDF, general registers
//! FFEE0-FFEFF (4 banks, memory mapped), SFR FFF00-FFFFF (SP FFFF8, PSW FFFFA, CS FFFFC, ES FFFFD, PMC FFFFE).

pub const PSW_CY: u8 = 0x01;
pub const PSW_AC: u8 = 0x10;
pub const PSW_Z: u8 = 0x40;
pub const PSW_IE: u8 = 0x80;

const SPL: u32 = 0xFFFF8;
#[allow(dead_code)]
const PSW: u32 = 0xFFFFA;
#[allow(dead_code)]
const CS: u32 = 0xFFFFC;
#[allow(dead_code)]
const ES: u32 = 0xFFFFD;
#[allow(dead_code)]
const PMC: u32 = 0xFFFFE;

/// Peripheral side of the bus: SFR (FFF00-FFFFF except CPU registers) and 2nd SFR (F0000-F07FF) areas.
pub trait Rl78Bus {
    fn read(&mut self, addr: u32) -> u8;
    fn write(&mut self, addr: u32, v: u8);
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum Rl78Fault { Illegal { pc: u32, bytes: [u8; 4] }, Fetch { pc: u32 }, Write { pc: u32, addr: u32 } }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Rl78 {
    pub flash: Vec<u8>,
    pub data_flash: Vec<u8>,
    /// RAM F7F00-FFEFF including the general-register banks.
    pub ram: Vec<u8>,
    /// SFR images (written values) for FFF00-FFFFF and F0000-F07FF, consulted by the bus model.
    pub pc: u32,
    pub halted: bool,
    pub stopped: bool,
    /// Pending ES prefix for the current instruction.
    es_prefix: bool,
    pub steps: u64,
    pub cur_pc: u32,
    regs_hi: [u8; 8], // FFFF8..FFFFF: SPL SPH PSW (resv) CS ES PMC MEM
    /// fCLK cycles taken by the last instruction (RL78-S3 timing: branches, calls, returns and divides cost more).
    #[serde(skip)]
    pub cycles: u32,
}

const RAM_BASE: u32 = 0xF7F00;
const RAM_END: u32 = 0xFFF00;

type R<T> = Result<T, Rl78Fault>;

impl Rl78 {
    pub fn new(flash: Vec<u8>) -> Rl78 {
        let mut c = Rl78 { flash, data_flash: vec![0xFF; 0x2000], ram: vec![0; (RAM_END - RAM_BASE) as usize], pc: 0, halted: false, stopped: false,
            es_prefix: false, steps: 0, cur_pc: 0, regs_hi: [0; 8], cycles: 1 };
        c.regs_hi[2] = 0x06; // PSW after reset: ISP = 3
        c.regs_hi[5] = 0x0F; // ES after reset
        c.pc = c.flash_word(0) as u32;
        c
    }
    fn flash_word(&self, a: u32) -> u16 { u16::from_le_bytes([self.flash[a as usize], self.flash[a as usize + 1]]) }

    // ---------- memory ----------
    #[inline]
    pub fn read8<B: Rl78Bus>(&mut self, bus: &mut B, a: u32) -> u8 {
        let a = a & 0xFFFFF;
        if (RAM_BASE..0xFFF00).contains(&a) { return self.ram[(a - RAM_BASE) as usize]; }
        match a {
            0..=0x7FFFF => self.flash[a as usize],
            0xF1000..=0xF2FFF => self.data_flash[(a - 0xF1000) as usize],
            0xF3000..=0xF7EFF => { let base = if self.regs_hi[6] & 1 != 0 { 0x10000 } else { 0 }; self.flash[(base + (a & 0xFFFF)) as usize] }
            RAM_BASE..=0xFFEFF => self.ram[(a - RAM_BASE) as usize],
            0xFFFF8..=0xFFFFF => self.regs_hi[(a - 0xFFFF8) as usize],
            _ => bus.read(a),
        }
    }
    #[inline]
    pub fn write8<B: Rl78Bus>(&mut self, bus: &mut B, a: u32, v: u8) -> R<()> {
        let a = a & 0xFFFFF;
        if (RAM_BASE..0xFFF00).contains(&a) { self.ram[(a - RAM_BASE) as usize] = v; return Ok(()); }
        match a {
            RAM_BASE..=0xFFEFF => self.ram[(a - RAM_BASE) as usize] = v,
            0xFFFF8..=0xFFFFF => { self.regs_hi[(a - 0xFFFF8) as usize] = if a == SPL { v & 0xFE } else { v }; }
            0..=0x7FFFF | 0xF1000..=0xF2FFF | 0xF3000..=0xF7EFF => return Err(Rl78Fault::Write { pc: self.cur_pc, addr: a }),
            _ => bus.write(a, v),
        }
        Ok(())
    }
    pub fn read16<B: Rl78Bus>(&mut self, bus: &mut B, a: u32) -> u16 { let a = a & !1; (self.read8(bus, a) as u16) | ((self.read8(bus, a + 1) as u16) << 8) }
    pub fn write16<B: Rl78Bus>(&mut self, bus: &mut B, a: u32, v: u16) -> R<()> { let a = a & !1; self.write8(bus, a, v as u8)?; self.write8(bus, a + 1, (v >> 8) as u8) }

    // ---------- registers ----------
    pub fn psw(&self) -> u8 { self.regs_hi[2] }
    pub fn set_psw(&mut self, v: u8) { self.regs_hi[2] = v; }
    pub fn sp(&self) -> u16 { u16::from_le_bytes([self.regs_hi[0], self.regs_hi[1]]) }
    pub fn set_sp(&mut self, v: u16) { self.regs_hi[0] = v as u8 & 0xFE; self.regs_hi[1] = (v >> 8) as u8; }
    pub fn cs(&self) -> u8 { self.regs_hi[4] & 0x0F }
    pub fn es(&self) -> u8 { self.regs_hi[5] & 0x0F }
    fn bank_base(&self) -> u32 { let p = self.psw(); let rb = ((p >> 4) & 2) | ((p >> 3) & 1); 0xFFEF8 - 8 * rb as u32 }
    /// r: 0 X, 1 A, 2 C, 3 B, 4 E, 5 D, 6 L, 7 H.
    pub fn r(&self, i: usize) -> u8 { self.ram[(self.bank_base() - RAM_BASE) as usize + i] }
    pub fn set_r(&mut self, i: usize, v: u8) { let o = (self.bank_base() - RAM_BASE) as usize + i; self.ram[o] = v; }
    /// rp: 0 AX, 1 BC, 2 DE, 3 HL.
    pub fn rp(&self, i: usize) -> u16 { (self.r(2 * i) as u16) | ((self.r(2 * i + 1) as u16) << 8) }
    pub fn set_rp(&mut self, i: usize, v: u16) { self.set_r(2 * i, v as u8); self.set_r(2 * i + 1, (v >> 8) as u8); }
    fn a(&self) -> u8 { self.r(1) }
    fn set_a(&mut self, v: u8) { self.set_r(1, v) }
    fn cy(&self) -> bool { self.psw() & PSW_CY != 0 }
    fn z(&self) -> bool { self.psw() & PSW_Z != 0 }
    fn set_flag(&mut self, f: u8, on: bool) { let p = self.psw(); self.set_psw(if on { p | f } else { p & !f }); }

    // ---------- fetch ----------
    #[inline(always)]
    fn fetch<B: Rl78Bus>(&mut self, bus: &mut B) -> R<u8> {
        if self.pc < 0x80000 { let v = self.flash[self.pc as usize]; self.pc += 1; return Ok(v); }
        if self.pc >= 0x80000 && !(RAM_BASE..0xFFEE0).contains(&self.pc) && !(0xF0800..0xF1000).contains(&self.pc) { return Err(Rl78Fault::Fetch { pc: self.pc }); }
        let v = self.read8(bus, self.pc);
        self.pc = (self.pc + 1) & 0xFFFFF;
        Ok(v)
    }
    fn fetch16<B: Rl78Bus>(&mut self, bus: &mut B) -> R<u16> { let l = self.fetch(bus)? as u16; Ok(l | ((self.fetch(bus)? as u16) << 8)) }

    // ---------- effective addresses ----------
    fn base(&self) -> u32 { if self.es_prefix { (self.es() as u32) << 16 } else { 0xF0000 } }
    fn abs16(&self, a: u16) -> u32 { self.base() | a as u32 }
    fn saddr(b: u8) -> u32 { if b < 0x20 { 0xFFF00 | b as u32 } else { 0xFFE00 | b as u32 } }
    fn sfr(b: u8) -> u32 { 0xFFF00 | b as u32 }
    fn sp_off(&self, d: u8) -> u32 { 0xF0000 | (self.sp().wrapping_add(d as u16) as u32) }
    fn ind(&self, rp: usize, d: u16) -> u32 { (self.base() & 0xF0000) | (self.rp(rp).wrapping_add(d) as u32) }
    fn based(&self, idx: u16, w: u16) -> u32 { (self.base() & 0xF0000) | (w.wrapping_add(idx) as u32) }

    // ---------- arithmetic ----------
    fn alu8(&mut self, op: u8, x: u8, y: u8) -> u8 {
        // op: 0 ADD, 1 ADDC, 2 SUB, 3 SUBC, 4 CMP, 5 AND, 6 OR, 7 XOR
        let c = self.cy() as u16;
        let (res, cy, ac) = match op {
            0 | 1 => { let ci = if op == 1 { c } else { 0 }; let s = x as u16 + y as u16 + ci; (s as u8, s > 0xFF, (x & 15) as u16 + (y & 15) as u16 + ci > 15) }
            2 | 3 | 4 => { let bi = if op == 3 { c } else { 0 }; let d = (x as i16) - (y as i16) - bi as i16; (d as u8, d < 0, ((x & 15) as i16) - ((y & 15) as i16) - (bi as i16) < 0) }
            5 => (x & y, self.cy(), self.psw() & PSW_AC != 0),
            6 => (x | y, self.cy(), self.psw() & PSW_AC != 0),
            _ => (x ^ y, self.cy(), self.psw() & PSW_AC != 0),
        };
        self.set_flag(PSW_Z, res == 0);
        if op <= 4 { self.set_flag(PSW_CY, cy); self.set_flag(PSW_AC, ac); }
        res
    }
    fn alu16(&mut self, op: u8, x: u16, y: u16) -> u16 {
        // op: 0 ADDW, 2 SUBW, 4 CMPW
        let (res, cy, ac) = if op == 0 { let s = x as u32 + y as u32; (s as u16, s > 0xFFFF, (x & 0xFF) + (y & 0xFF) > 0xFF) }
            else { let d = x as i32 - y as i32; (d as u16, d < 0, ((x & 0xFF) as i32) < ((y & 0xFF) as i32)) };
        self.set_flag(PSW_Z, res == 0); self.set_flag(PSW_CY, cy); self.set_flag(PSW_AC, ac);
        res
    }
    fn incdec8(&mut self, v: u8, inc: bool) -> u8 {
        let r = if inc { v.wrapping_add(1) } else { v.wrapping_sub(1) };
        self.set_flag(PSW_Z, r == 0);
        self.set_flag(PSW_AC, if inc { v & 15 == 15 } else { v & 15 == 0 });
        r
    }

    // ---------- stack ----------
    fn push8<B: Rl78Bus>(&mut self, bus: &mut B, v: u8) -> R<()> { let s = self.sp().wrapping_sub(1); self.set_sp_raw(s); self.write8(bus, 0xF0000 | s as u32, v) }
    fn set_sp_raw(&mut self, v: u16) { self.regs_hi[0] = v as u8; self.regs_hi[1] = (v >> 8) as u8; }
    fn push_pc<B: Rl78Bus>(&mut self, bus: &mut B, ret: u32, psw: Option<u8>) -> R<()> {
        let sp = self.sp();
        self.write8(bus, 0xF0000 | sp.wrapping_sub(1) as u32, psw.unwrap_or(0))?;
        self.write8(bus, 0xF0000 | sp.wrapping_sub(2) as u32, (ret >> 16) as u8 & 0x0F)?;
        self.write8(bus, 0xF0000 | sp.wrapping_sub(3) as u32, (ret >> 8) as u8)?;
        self.write8(bus, 0xF0000 | sp.wrapping_sub(4) as u32, ret as u8)?;
        self.set_sp(sp.wrapping_sub(4));
        Ok(())
    }
    /// Return from a CALL (used by high-level emulation of boot-ROM routines).
    pub fn ret<B: Rl78Bus>(&mut self, bus: &mut B) { self.pc = self.pop_pc(bus, false); }
    fn pop_pc<B: Rl78Bus>(&mut self, bus: &mut B, with_psw: bool) -> u32 {
        let sp = self.sp();
        let l = self.read8(bus, 0xF0000 | sp as u32) as u32;
        let h = self.read8(bus, 0xF0000 | sp.wrapping_add(1) as u32) as u32;
        let s = self.read8(bus, 0xF0000 | sp.wrapping_add(2) as u32) as u32 & 0x0F;
        if with_psw { let p = self.read8(bus, 0xF0000 | sp.wrapping_add(3) as u32); self.set_psw(p); }
        self.set_sp(sp.wrapping_add(4));
        (s << 16) | (h << 8) | l
    }

    /// Vectored interrupt / BRK acceptance: push PSW and PC, IE=0, ISP=level, jump through the vector.
    pub fn interrupt<B: Rl78Bus>(&mut self, bus: &mut B, vector: u32, level: u8) -> R<()> {
        let psw = self.psw();
        self.push_pc(bus, self.pc, Some(psw))?;
        self.set_psw((psw & !(PSW_IE | 0x06)) | ((level & 3) << 1));
        self.pc = self.flash_word(vector) as u32;
        self.halted = false; self.stopped = false;
        Ok(())
    }
    pub fn isp(&self) -> u8 { (self.psw() >> 1) & 3 }
    pub fn ie(&self) -> bool { self.psw() & PSW_IE != 0 }

    // ---------- instruction length (for the SKxx skips) ----------
    pub fn insn_len<B: Rl78Bus>(&mut self, bus: &mut B, a: u32) -> u32 {
        let b0 = self.read8(bus, a);
        if b0 == 0x11 { return 1 + self.insn_len(bus, a + 1); }
        let b1 = self.read8(bus, a + 1);
        match b0 {
            0x61 => match b1 { 0x09 | 0x29 | 0x49 | 0x59 | 0x69 | 0x79 | 0x89 | 0xA8 | 0xAB | 0xAD | 0xAF | 0xB8 | 0xCE | 0xDE | 0xC3 | 0xD3 => 3, 0xAA => 4, _ => 2 },
            0x71 => { let c = b1 & 0x0F; if b1 & 0x80 != 0 { 2 } else if c == 0 || c == 8 { 4 } else { 3 } }
            0x31 => match b1 & 0x0F { 0 | 2 | 4 => 4, 1 | 3 | 5 => 3, _ => 2 },
            0xCE if b1 == 0xFB => 3,
            _ => LEN1[b0 as usize] as u32,
        }
    }

    // ---------- execution ----------
    pub fn step<B: Rl78Bus>(&mut self, bus: &mut B) -> R<()> {
        self.steps += 1;
        self.cycles = 1;
        if self.halted || self.stopped { return Ok(()); }
        self.cur_pc = self.pc;
        self.es_prefix = false;
        let mut op = self.fetch(bus)?;
        if op == 0x11 { self.es_prefix = true; op = self.fetch(bus)?; }
        let r = self.exec(bus, op);
        self.es_prefix = false;
        r
    }

    fn illegal<B: Rl78Bus>(&mut self, bus: &mut B) -> Rl78Fault {
        let p = self.cur_pc;
        Rl78Fault::Illegal { pc: p, bytes: [self.read8(bus, p), self.read8(bus, p + 1), self.read8(bus, p + 2), self.read8(bus, p + 3)] }
    }

    fn skip<B: Rl78Bus>(&mut self, bus: &mut B, cond: bool) { if cond { let l = self.insn_len(bus, self.pc); self.pc = (self.pc + l) & 0xFFFFF; } }
    fn rel8<B: Rl78Bus>(&mut self, bus: &mut B, cond: bool) -> R<()> {
        self.cycles = if cond { 4 } else { 2 };
        let d = self.fetch(bus)? as i8;
        if cond { self.pc = (self.pc as i32 + d as i32) as u32 & 0xFFFFF; }
        Ok(())
    }

    fn exec<B: Rl78Bus>(&mut self, bus: &mut B, op: u8) -> R<()> {
        let lo = op & 0x0F;
        let hi = op >> 4;
        match op {
            0x00 => {}
            // ADDW/SUBW/CMPW AX,src ; MOVW rp,AX etc. (row 0..4, cols 1..7)
            0x01 | 0x03 | 0x05 | 0x07 | 0x23 | 0x25 | 0x27 | 0x43 | 0x45 | 0x47 => {
                let src = match lo { 1 => self.rp(0), 3 => self.rp(1), 5 => self.rp(2), _ => self.rp(3) };
                let k = match hi { 0 => 0, 2 => 2, _ => 4 };
                let r = self.alu16(k, self.rp(0), src);
                if k != 4 { self.set_rp(0, r); }
            }
            0x02 | 0x22 | 0x42 => { let a = self.fetch16(bus)?; let ea = self.abs16(a); let v = self.read16(bus, ea); let k = hi; let r = self.alu16(k, self.rp(0), v); if k != 4 { self.set_rp(0, r); } }
            0x04 | 0x24 | 0x44 => { let v = self.fetch16(bus)?; let k = hi; let r = self.alu16(k, self.rp(0), v); if k != 4 { self.set_rp(0, r); } }
            0x06 | 0x26 | 0x46 => { let b = self.fetch(bus)?; let v = self.read16(bus, Self::saddr(b)); let k = hi; let r = self.alu16(k, self.rp(0), v); if k != 4 { self.set_rp(0, r); } }
            0x10 | 0x20 => { let b = self.fetch(bus)? as u16; let sp = self.sp(); self.set_sp(if op == 0x10 { sp.wrapping_add(b) } else { sp.wrapping_sub(b) }); }
            0x12 => { let v = self.rp(0); self.set_rp(1, v); }
            0x13 => { let v = self.rp(1); self.set_rp(0, v); }
            0x14 => { let v = self.rp(0); self.set_rp(2, v); }
            0x15 => { let v = self.rp(2); self.set_rp(0, v); }
            0x16 => { let v = self.rp(0); self.set_rp(3, v); }
            0x17 => { let v = self.rp(3); self.set_rp(0, v); }
            0x30 | 0x32 | 0x34 | 0x36 => { let v = self.fetch16(bus)?; self.set_rp((lo / 2) as usize, v); }
            0x33 | 0x35 | 0x37 => { let i = ((lo - 1) / 2) as usize; let (a, b) = (self.rp(0), self.rp(i)); self.set_rp(0, b); self.set_rp(i, a); }
            0x08 => { let (a, x) = (self.r(1), self.r(0)); self.set_r(1, x); self.set_r(0, a); }
            // MOV via word[B]/[C]/[BC]
            0x09 => { let w = self.fetch16(bus)?; let ea = self.based(self.r(3) as u16, w); let v = self.read8(bus, ea); self.set_a(v); }
            0x18 => { let w = self.fetch16(bus)?; let ea = self.based(self.r(3) as u16, w); let a = self.a(); self.write8(bus, ea, a)?; }
            0x19 => { let w = self.fetch16(bus)?; let b = self.fetch(bus)?; let ea = self.based(self.r(3) as u16, w); self.write8(bus, ea, b)?; }
            0x28 => { let w = self.fetch16(bus)?; let ea = self.based(self.r(2) as u16, w); let a = self.a(); self.write8(bus, ea, a)?; }
            0x29 => { let w = self.fetch16(bus)?; let ea = self.based(self.r(2) as u16, w); let v = self.read8(bus, ea); self.set_a(v); }
            0x38 => { let w = self.fetch16(bus)?; let b = self.fetch(bus)?; let ea = self.based(self.r(2) as u16, w); self.write8(bus, ea, b)?; }
            0x39 => { let w = self.fetch16(bus)?; let b = self.fetch(bus)?; let ea = self.based(self.rp(1), w); self.write8(bus, ea, b)?; }
            0x48 => { let w = self.fetch16(bus)?; let ea = self.based(self.rp(1), w); let a = self.a(); self.write8(bus, ea, a)?; }
            0x49 => { let w = self.fetch16(bus)?; let ea = self.based(self.rp(1), w); let v = self.read8(bus, ea); self.set_a(v); }
            0x58 | 0x68 | 0x78 => { let w = self.fetch16(bus)?; let idx = match op { 0x58 => self.r(3) as u16, 0x68 => self.r(2) as u16, _ => self.rp(1) }; let ea = self.based(idx, w); let v = self.rp(0); self.write16(bus, ea, v)?; }
            0x59 | 0x69 | 0x79 => { let w = self.fetch16(bus)?; let idx = match op { 0x59 => self.r(3) as u16, 0x69 => self.r(2) as u16, _ => self.rp(1) }; let ea = self.based(idx, w); let v = self.read16(bus, ea); self.set_rp(0, v); }
            // 8-bit ALU groups: cols a..f of rows 0..7
            0x0A | 0x1A | 0x2A | 0x3A | 0x4A | 0x5A | 0x6A | 0x7A => {
                let b = self.fetch(bus)?; let v = self.fetch(bus)?; let ea = Self::saddr(b); let x = self.read8(bus, ea); let r = self.alu8(hi, x, v); if hi != 4 { self.write8(bus, ea, r)?; }
            }
            _ if hi <= 7 && lo >= 0x0B => {
                let src = match lo {
                    0x0B => { let b = self.fetch(bus)?; self.read8(bus, Self::saddr(b)) }
                    0x0C => self.fetch(bus)?,
                    0x0D => { let ea = self.ind(3, 0); self.read8(bus, ea) }
                    0x0E => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); self.read8(bus, ea) }
                    _ => { let a = self.fetch16(bus)?; let ea = self.abs16(a); self.read8(bus, ea) }
                };
                let r = self.alu8(hi, self.a(), src);
                if hi != 4 { self.set_a(r); }
            }
            0x40 => { let a = self.fetch16(bus)?; let v = self.fetch(bus)?; let ea = self.abs16(a); let x = self.read8(bus, ea); self.alu8(4, x, v); }
            0x41 => { let v = self.fetch(bus)?; self.regs_hi[5] = v & 0x0F; }
            0x50..=0x57 => { let v = self.fetch(bus)?; let i = [0, 1, 2, 3, 4, 5, 6, 7][lo as usize]; self.set_r(i, v); }
            0x60 => { let v = self.r(0); self.set_a(v); }
            0x62..=0x67 => { let v = self.r(lo as usize); self.set_a(v); }
            0x70 => { let v = self.a(); self.set_r(0, v); }
            0x72..=0x77 => { let v = self.a(); self.set_r(lo as usize, v); }
            0x61 => { let op2 = self.fetch(bus)?; return self.exec2(bus, op2); }
            0x71 => { let op2 = self.fetch(bus)?; return self.exec3(bus, op2); }
            0x31 => { let op2 = self.fetch(bus)?; return self.exec4(bus, op2); }
            0x80..=0x87 => { let i = (lo & 7) as usize; let v = self.r(i); let r = self.incdec8(v, true); self.set_r(i, r); }
            0x90..=0x97 => { let i = (lo & 7) as usize; let v = self.r(i); let r = self.incdec8(v, false); self.set_r(i, r); }
            0x88 => { let d = self.fetch(bus)?; let ea = self.sp_off(d); let v = self.read8(bus, ea); self.set_a(v); }
            0x89 => { let ea = self.ind(2, 0); let v = self.read8(bus, ea); self.set_a(v); }
            0x8A => { let d = self.fetch(bus)? as u16; let ea = self.ind(2, d); let v = self.read8(bus, ea); self.set_a(v); }
            0x8B => { let ea = self.ind(3, 0); let v = self.read8(bus, ea); self.set_a(v); }
            0x8C => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let v = self.read8(bus, ea); self.set_a(v); }
            0x8D => { let b = self.fetch(bus)?; let v = self.read8(bus, Self::saddr(b)); self.set_a(v); }
            0x8E => { let b = self.fetch(bus)?; let v = self.read8(bus, Self::sfr(b)); self.set_a(v); }
            0x8F => { let a = self.fetch16(bus)?; let ea = self.abs16(a); let v = self.read8(bus, ea); self.set_a(v); }
            0x98 => { let d = self.fetch(bus)?; let ea = self.sp_off(d); let a = self.a(); self.write8(bus, ea, a)?; }
            0x99 => { let ea = self.ind(2, 0); let a = self.a(); self.write8(bus, ea, a)?; }
            0x9A => { let d = self.fetch(bus)? as u16; let ea = self.ind(2, d); let a = self.a(); self.write8(bus, ea, a)?; }
            0x9B => { let ea = self.ind(3, 0); let a = self.a(); self.write8(bus, ea, a)?; }
            0x9C => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let a = self.a(); self.write8(bus, ea, a)?; }
            0x9D => { let b = self.fetch(bus)?; let a = self.a(); self.write8(bus, Self::saddr(b), a)?; }
            0x9E => { let b = self.fetch(bus)?; let a = self.a(); self.write8(bus, Self::sfr(b), a)?; }
            0x9F => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let a = self.a(); self.write8(bus, ea, a)?; }
            0xA0 | 0xB0 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let v = self.read8(bus, ea); let r = self.incdec8(v, op == 0xA0); self.write8(bus, ea, r)?; }
            0xA4 | 0xB4 => { let b = self.fetch(bus)?; let ea = Self::saddr(b); let v = self.read8(bus, ea); let r = self.incdec8(v, op == 0xA4); self.write8(bus, ea, r)?; }
            0xA1 | 0xA3 | 0xA5 | 0xA7 | 0xB1 | 0xB3 | 0xB5 | 0xB7 => { let i = ((lo - 1) / 2) as usize; let v = self.rp(i); self.set_rp(i, if hi == 0xA { v.wrapping_add(1) } else { v.wrapping_sub(1) }); }
            0xA2 | 0xB2 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let v = self.read16(bus, ea); self.write16(bus, ea, if op == 0xA2 { v.wrapping_add(1) } else { v.wrapping_sub(1) })?; }
            0xA6 | 0xB6 => { let b = self.fetch(bus)?; let ea = Self::saddr(b); let v = self.read16(bus, ea); self.write16(bus, ea, if op == 0xA6 { v.wrapping_add(1) } else { v.wrapping_sub(1) })?; }
            0xA8..=0xAF => { let ea = self.word_ea(bus, lo)?; let v = self.read16(bus, ea); self.set_rp(0, v); }
            0xB8..=0xBF => { let ea = self.word_ea(bus, lo)?; let v = self.rp(0); self.write16(bus, ea, v)?; }
            0xC0 | 0xC2 | 0xC4 | 0xC6 => { let sp = self.sp(); let v = self.read16(bus, 0xF0000 | sp as u32); self.set_rp((lo / 2) as usize, v); self.set_sp(sp.wrapping_add(2)); }
            0xC1 | 0xC3 | 0xC5 | 0xC7 => { let v = self.rp((lo / 2) as usize); self.push8(bus, (v >> 8) as u8)?; self.push8(bus, v as u8)?; }
            0xC8 => { let d = self.fetch(bus)?; let v = self.fetch(bus)?; let ea = self.sp_off(d); self.write8(bus, ea, v)?; }
            0xC9 => { let b = self.fetch(bus)?; let v = self.fetch16(bus)?; self.write16(bus, Self::saddr(b), v)?; }
            0xCA => { let d = self.fetch(bus)? as u16; let v = self.fetch(bus)?; let ea = self.ind(2, d); self.write8(bus, ea, v)?; }
            0xCB => { let b = self.fetch(bus)?; let v = self.fetch16(bus)?; self.write16(bus, Self::sfr(b), v)?; }
            0xCC => { let d = self.fetch(bus)? as u16; let v = self.fetch(bus)?; let ea = self.ind(3, d); self.write8(bus, ea, v)?; }
            0xCD => { let b = self.fetch(bus)?; let v = self.fetch(bus)?; self.write8(bus, Self::saddr(b), v)?; }
            0xCE => {
                let b = self.fetch(bus)?; let v = self.fetch(bus)?;
                if b == 0xFB { return self.muldiv(bus, v); }
                self.write8(bus, Self::sfr(b), v)?;
            }
            0xCF => { let ad = self.fetch16(bus)?; let v = self.fetch(bus)?; let ea = self.abs16(ad); self.write8(bus, ea, v)?; }
            0xD0..=0xD3 => { let v = self.r([0, 1, 2, 3][lo as usize]); self.cmp0(v); }
            0xD4 => { let b = self.fetch(bus)?; let v = self.read8(bus, Self::saddr(b)); self.cmp0(v); }
            0xD5 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let v = self.read8(bus, ea); self.cmp0(v); }
            0xD6 => { let r = self.a() as u16 * self.r(0) as u16; self.set_rp(0, r); }
            0xD7 => { self.pc = self.pop_pc(bus, false); self.cycles = 6; }
            0xD8 | 0xE8 | 0xF8 => { let b = self.fetch(bus)?; let v = self.read8(bus, Self::saddr(b)); self.set_r(match hi { 0xD => 0, 0xE => 3, _ => 2 }, v); }
            0xD9 | 0xE9 | 0xF9 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let v = self.read8(bus, ea); self.set_r(match hi { 0xD => 0, 0xE => 3, _ => 2 }, v); }
            0xDA | 0xEA | 0xFA => { let b = self.fetch(bus)?; let v = self.read16(bus, Self::saddr(b)); self.set_rp((hi - 0xC) as usize, v); }
            0xDB | 0xEB | 0xFB => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); let v = self.read16(bus, ea); self.set_rp((hi - 0xC) as usize, v); }
            0xDC => { let c = self.cy(); self.rel8(bus, c)?; }
            0xDD => { let c = self.z(); self.rel8(bus, c)?; }
            0xDE => { let c = !self.cy(); self.rel8(bus, c)?; }
            0xDF => { let c = !self.z(); self.rel8(bus, c)?; }
            0xE0..=0xE3 => self.set_r([0, 1, 2, 3][lo as usize], 1),
            0xE4 => { let b = self.fetch(bus)?; self.write8(bus, Self::saddr(b), 1)?; }
            0xE5 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); self.write8(bus, ea, 1)?; }
            0xE6 => self.set_rp(0, 1),
            0xE7 => self.set_rp(1, 1),
            0xEC => { let l = self.fetch16(bus)? as u32; let h = self.fetch(bus)? as u32; self.pc = ((h & 0x0F) << 16) | l; self.cycles = 3; }
            0xED => { let a = self.fetch16(bus)? as u32; self.pc = a; self.cycles = 3; }
            0xEE => { let d = self.fetch16(bus)? as i16; self.pc = (self.pc as i32 + d as i32) as u32 & 0xFFFFF; self.cycles = 3; }
            0xEF => { let c = true; self.rel8(bus, c)?; self.cycles = 3; }
            0xF0..=0xF3 => self.set_r([0, 1, 2, 3][lo as usize], 0),
            0xF4 => { let b = self.fetch(bus)?; self.write8(bus, Self::saddr(b), 0)?; }
            0xF5 => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); self.write8(bus, ea, 0)?; }
            0xF6 => self.set_rp(0, 0),
            0xF7 => self.set_rp(1, 0),
            0xFC => { let l = self.fetch16(bus)? as u32; let h = self.fetch(bus)? as u32; let t = ((h & 0x0F) << 16) | l; let ret = self.pc; self.push_pc(bus, ret, None)?; self.pc = t; self.cycles = 3; }
            0xFD => { let a = self.fetch16(bus)? as u32; let ret = self.pc; self.push_pc(bus, ret, None)?; self.pc = a; self.cycles = 3; }
            0xFE => { let d = self.fetch16(bus)? as i16; let ret = self.pc; let t = (self.pc as i32 + d as i32) as u32 & 0xFFFFF; self.push_pc(bus, ret, None)?; self.pc = t; self.cycles = 3; }
            _ => return Err(self.illegal(bus)),
        }
        Ok(())
    }

    /// Word operand for MOVW AX,x / x,AX (rows A/B cols 8-F).
    fn word_ea<B: Rl78Bus>(&mut self, bus: &mut B, lo: u8) -> R<u32> {
        Ok(match lo {
            8 => { let d = self.fetch(bus)?; self.sp_off(d) }
            9 => self.ind(2, 0),
            0xA => { let d = self.fetch(bus)? as u16; self.ind(2, d) }
            0xB => self.ind(3, 0),
            0xC => { let d = self.fetch(bus)? as u16; self.ind(3, d) }
            0xD => { let b = self.fetch(bus)?; Self::saddr(b) }
            0xE => { let b = self.fetch(bus)?; Self::sfr(b) }
            _ => { let a = self.fetch16(bus)?; self.abs16(a) }
        })
    }

    fn cmp0(&mut self, v: u8) { self.set_flag(PSW_Z, v == 0); self.set_flag(PSW_AC, false); self.set_flag(PSW_CY, false); }

    /// CE FB xx: MULHU/MULH/DIVHU/DIVWU/MACHU/MACH (RL78-S3).
    fn muldiv<B: Rl78Bus>(&mut self, bus: &mut B, k: u8) -> R<()> {
        self.cycles = match k { 0x03 => 9, 0x0B => 17, 0x05 | 0x06 => 3, _ => 2 };
        match k {
            0x01 => { let r = self.rp(0) as u32 * self.rp(1) as u32; self.set_rp(0, r as u16); self.set_rp(1, (r >> 16) as u16); }
            0x02 => { let r = (self.rp(0) as i16 as i32 * self.rp(1) as i16 as i32) as u32; self.set_rp(0, r as u16); self.set_rp(1, (r >> 16) as u16); }
            0x03 => { let (n, d) = (self.rp(0), self.rp(2)); let (q, rem) = if d == 0 { (0xFFFF, n) } else { (n / d, n % d) }; self.set_rp(0, q); self.set_rp(2, rem); }
            0x0B => {
                let n = ((self.rp(1) as u32) << 16) | self.rp(0) as u32; let d = ((self.rp(3) as u32) << 16) | self.rp(2) as u32;
                let (q, rem) = if d == 0 { (0xFFFF_FFFF, n) } else { (n / d, n % d) };
                self.set_rp(0, q as u16); self.set_rp(1, (q >> 16) as u16); self.set_rp(2, rem as u16); self.set_rp(3, (rem >> 16) as u16);
            }
            0x05 | 0x06 => {
                let macr = (self.read16(bus, 0xFFFF0) as u32) | ((self.read16(bus, 0xFFFF2) as u32) << 16);
                let (r, cy, ac) = if k == 0x05 {
                    let p = self.rp(0) as u64 * self.rp(1) as u64; let s = macr as u64 + p; (s as u32, s > 0xFFFF_FFFF, false)
                } else {
                    let p = self.rp(0) as i16 as i64 * self.rp(1) as i16 as i64; let s = macr as i32 as i64 + p;
                    (s as u32, s > i32::MAX as i64 || s < i32::MIN as i64, (s as i32) < 0)
                };
                self.write16(bus, 0xFFFF0, r as u16)?; self.write16(bus, 0xFFFF2, (r >> 16) as u16)?;
                self.set_flag(PSW_CY, cy); self.set_flag(PSW_AC, ac);
            }
            _ => return Err(self.illegal(bus)),
        }
        Ok(())
    }

    /// 2nd map (prefix 0x61).
    fn exec2<B: Rl78Bus>(&mut self, bus: &mut B, op: u8) -> R<()> {
        let (hi, lo) = (op >> 4, op & 15);
        match op {
            0x00..=0x7F if lo <= 7 => { // OP r,A  (col0 X,A; col1 A,A; 2 C,A ...)
                let i = lo as usize; let x = self.r(i); let r = self.alu8(hi, x, self.a()); if hi != 4 { self.set_r(i, r); }
            }
            0x00..=0x7F if lo >= 8 && lo != 9 => { // OP A,r (col8 A,X; col a A,C; b A,B ...)
                let i = if lo == 8 { 0 } else { (lo - 8) as usize }; let r = self.alu8(hi, self.a(), self.r(i)); if hi != 4 { self.set_a(r); }
            }
            0x09 | 0x29 | 0x49 => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let v = self.read16(bus, ea); let k = hi; let r = self.alu16(k, self.rp(0), v); if k != 4 { self.set_rp(0, r); } }
            0x59 | 0x69 => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let v = self.read8(bus, ea); let r = self.incdec8(v, op == 0x59); self.write8(bus, ea, r)?; }
            0x79 | 0x89 => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let v = self.read16(bus, ea); self.write16(bus, ea, if op == 0x79 { v.wrapping_add(1) } else { v.wrapping_sub(1) })?; }
            0x80 | 0x90 | 0xA0 | 0xB0 | 0xC0 | 0xD0 | 0xE0 | 0xF0 | 0x82 | 0x92 | 0xA2 | 0xB2 | 0xC2 | 0xD2 | 0xE2 | 0xF2 => {
                let idx = if lo == 0 { self.r(3) } else { self.r(2) } as u16;
                let ea = self.ind(3, idx); let v = self.read8(bus, ea); let k = hi - 8; let r = self.alu8(k, self.a(), v); if k != 4 { self.set_a(r); }
            }
            0x84..=0x87 | 0x94..=0x97 | 0xA4..=0xA7 | 0xB4..=0xB7 | 0xC4..=0xC7 | 0xD4..=0xD7 | 0xE4..=0xE7 | 0xF4..=0xF7 => {
                let t = 0x80 + ((lo - 4) as u32) * 0x10 + ((hi - 8) as u32) * 2;
                let target = self.flash_word(t) as u32; let ret = self.pc; self.push_pc(bus, ret, None)?; self.pc = target; self.cycles = 5;
            }
            0x8A..=0x8F => { let i = (lo - 8) as usize; let (a, x) = (self.a(), self.r(i)); self.set_a(x); self.set_r(i, a); }
            0xA8 => { let b = self.fetch(bus)?; let ea = Self::saddr(b); self.xch_mem(bus, ea)?; }
            0xA9 => { let ea = self.ind(3, self.r(2) as u16); self.xch_mem(bus, ea)?; }
            0xAA => { let ad = self.fetch16(bus)?; let ea = self.abs16(ad); self.xch_mem(bus, ea)?; }
            0xAB => { let b = self.fetch(bus)?; self.xch_mem(bus, Self::sfr(b))?; }
            0xAC => { let ea = self.ind(3, 0); self.xch_mem(bus, ea)?; }
            0xAD => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); self.xch_mem(bus, ea)?; }
            0xAE => { let ea = self.ind(2, 0); self.xch_mem(bus, ea)?; }
            0xAF => { let d = self.fetch(bus)? as u16; let ea = self.ind(2, d); self.xch_mem(bus, ea)?; }
            0xB8 => { let b = self.fetch(bus)?; let v = self.read8(bus, Self::saddr(b)); self.regs_hi[5] = v & 0x0F; }
            0xB9 => { let ea = self.ind(3, self.r(3) as u16); self.xch_mem(bus, ea)?; }
            0xC3 => { let c = !self.cy() && !self.z(); self.rel8(bus, c)?; } // BH
            0xD3 => { let c = self.cy() || self.z(); self.rel8(bus, c)?; } // BNH
            0xC8 => { let c = self.cy(); self.skip(bus, c); }
            0xD8 => { let c = !self.cy(); self.skip(bus, c); }
            0xE8 => { let c = self.z(); self.skip(bus, c); }
            0xF8 => { let c = !self.z(); self.skip(bus, c); }
            0xE3 => { let c = !self.cy() && !self.z(); self.skip(bus, c); }
            0xF3 => { let c = self.cy() || self.z(); self.skip(bus, c); }
            0xC9 => { let ea = self.ind(3, self.r(3) as u16); let v = self.read8(bus, ea); self.set_a(v); }
            0xD9 => { let ea = self.ind(3, self.r(3) as u16); let a = self.a(); self.write8(bus, ea, a)?; }
            0xE9 => { let ea = self.ind(3, self.r(2) as u16); let v = self.read8(bus, ea); self.set_a(v); }
            0xF9 => { let ea = self.ind(3, self.r(2) as u16); let a = self.a(); self.write8(bus, ea, a)?; }
            0xCA | 0xDA | 0xEA | 0xFA => { let rp = match op { 0xCA => 0, 0xDA => 1, 0xEA => 2, _ => 3 }; let t = ((self.cs() as u32) << 16) | self.rp(rp) as u32; let ret = self.pc; self.push_pc(bus, ret, None)?; self.pc = t; self.cycles = 3; }
            0xCB => { self.pc = ((self.cs() as u32) << 16) | self.rp(0) as u32; self.cycles = 3; }
            0xCC => { let ret = self.pc; self.pc = ret; return self.interrupt(bus, 0x7E, self.isp()).map(|_| ()); } // BRK
            0xCD => { let sp = self.sp(); let p = self.read8(bus, 0xF0000 | sp.wrapping_add(1) as u32); self.set_psw(p); self.set_sp(sp.wrapping_add(2)); self.cycles = 3; }
            0xDD => { let p = self.psw(); self.push8(bus, p)?; self.push8(bus, 0)?; }
            0xCE => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let x = self.r(0); self.write8(bus, ea, x)?; self.set_flag(PSW_Z, x == 0); self.set_flag(PSW_CY, self.a() == 0 || x == 0); }
            0xDE => { let d = self.fetch(bus)? as u16; let ea = self.ind(3, d); let m = self.read8(bus, ea); let x = self.r(0); self.set_flag(PSW_Z, x == m); self.set_flag(PSW_CY, self.a() == 0 || x == 0); }
            0xCF | 0xDF | 0xEF | 0xFF => { let rb = (hi - 0xC) as u8; let p = self.psw() & !0x28; self.set_psw(p | ((rb & 2) << 4) | ((rb & 1) << 3)); }
            0xDB => { let a = self.a(); self.set_flag(PSW_CY, a & 1 != 0); self.set_a(a.rotate_right(1)); }
            0xEB => { let a = self.a(); self.set_flag(PSW_CY, a & 0x80 != 0); self.set_a(a.rotate_left(1)); }
            0xFB => { let a = self.a(); let c = self.cy() as u8; self.set_flag(PSW_CY, a & 1 != 0); self.set_a((a >> 1) | (c << 7)); }
            0xDC => { let a = self.a(); let c = self.cy() as u8; self.set_flag(PSW_CY, a & 0x80 != 0); self.set_a((a << 1) | c); }
            0xEE | 0xFE => { let i = if op == 0xEE { 0 } else { 1 }; let v = self.rp(i); let c = self.cy() as u16; self.set_flag(PSW_CY, v & 0x8000 != 0); self.set_rp(i, (v << 1) | c); }
            0xEC => { // RETB
                let sp = self.sp();
                self.pc = self.pop_pc(bus, true); let _ = sp; self.cycles = 6;
            }
            0xFC => { self.pc = self.pop_pc(bus, true); self.cycles = 6; } // RETI
            0xED => { self.halted = true; }
            0xFD => { self.stopped = true; }
            _ => return Err(self.illegal(bus)),
        }
        Ok(())
    }

    fn xch_mem<B: Rl78Bus>(&mut self, bus: &mut B, ea: u32) -> R<()> { let m = self.read8(bus, ea); let a = self.a(); self.write8(bus, ea, a)?; self.set_a(m); Ok(()) }

    /// 3rd map (prefix 0x71): bit manipulation.
    fn exec3<B: Rl78Bus>(&mut self, bus: &mut B, op: u8) -> R<()> {
        let bit = (op >> 4) & 7;
        let m = 1u8 << bit;
        let low = op & 0x80 == 0;
        let c = op & 0x0F;
        if !low && c == 0 { // SET1 CY / NOT1 CY
            match bit { 0 => self.set_flag(PSW_CY, true), 4 => { let v = !self.cy(); self.set_flag(PSW_CY, v); } _ => return Err(self.illegal(bus)) }
            return Ok(());
        }
        if !low && c == 8 { if bit == 0 { self.set_flag(PSW_CY, false); return Ok(()); } return Err(self.illegal(bus)); }
        // operand address (or A register)
        enum Opnd { Mem(u32), A }
        let opnd = match (low, c) {
            (true, 0) | (true, 8) => { let a = self.fetch16(bus)?; Opnd::Mem(self.abs16(a)) }
            (true, 1..=7) => { let b = self.fetch(bus)?; Opnd::Mem(Self::saddr(b)) }
            (true, _) => { let b = self.fetch(bus)?; Opnd::Mem(Self::sfr(b)) }
            (false, 1..=7) => Opnd::Mem(self.ind(3, 0)),
            (false, _) => Opnd::A,
        };
        let get = |s: &mut Self, bus: &mut B, o: &Opnd| -> u8 { match o { Opnd::Mem(a) => s.read8(bus, *a), Opnd::A => s.a() } };
        let v = get(self, bus, &opnd);
        let b = v & m != 0;
        let kind = match c { 0 | 2 | 0xA => 1, 8 | 3 | 0xB => 2, 1 | 9 => 3, 4 | 0xC => 4, 5 | 0xD => 5, 6 | 0xE => 6, _ => 7 };
        let nv = match kind { 1 => Some(v | m), 2 => Some(v & !m), 3 => Some(if self.cy() { v | m } else { v & !m }), _ => None };
        match kind { 4 => self.set_flag(PSW_CY, b), 5 => { let x = self.cy() & b; self.set_flag(PSW_CY, x) } 6 => { let x = self.cy() | b; self.set_flag(PSW_CY, x) } 7 => { let x = self.cy() ^ b; self.set_flag(PSW_CY, x) } _ => {} }
        if let Some(nv) = nv { match opnd { Opnd::Mem(a) => self.write8(bus, a, nv)?, Opnd::A => self.set_a(nv) } }
        Ok(())
    }

    /// 4th map (prefix 0x31): bit test-and-branch, shifts.
    fn exec4<B: Rl78Bus>(&mut self, bus: &mut B, op: u8) -> R<()> {
        let (hi, c) = (op >> 4, op & 0x0F);
        match c {
            0..=5 => {
                let bit = hi & 7;
                let m = 1u8 << bit;
                let ea = match (hi < 8, c) {
                    (true, 0 | 2 | 4) => { let b = self.fetch(bus)?; Some(Self::saddr(b)) }
                    (true, _) => None,
                    (false, 0 | 2 | 4) => { let b = self.fetch(bus)?; Some(Self::sfr(b)) }
                    (false, _) => Some(self.ind(3, 0)),
                };
                let v = match ea { Some(a) => self.read8(bus, a), None => self.a() };
                let set = v & m != 0;
                let d = self.fetch(bus)? as i8;
                let take = match c { 0 | 1 => set, 2 | 3 => set, _ => !set };
                if (c == 0 || c == 1) && set { match ea { Some(a) => self.write8(bus, a, v & !m)?, None => self.set_a(v & !m) } }
                if take { self.pc = (self.pc as i32 + d as i32) as u32 & 0xFFFFF; }
                self.cycles = if take { 5 } else { 3 };
            }
            7..=0xB if hi >= 1 && hi <= 7 => {
                let n = hi as u32;
                let (idx, kind) = match c { 7 => (2usize, 0), 8 => (3, 0), 9 => (1, 0), 0xA => (1, 1), _ => (1, 2) };
                let v = self.r(idx);
                let (r, cy) = match kind {
                    0 => ((v as u32) << n, (v as u32 >> (8 - n)) & 1 != 0),
                    1 => ((v as u32) >> n, (v as u32 >> (n - 1)) & 1 != 0),
                    _ => (((v as i8 as i32) >> n) as u32, ((v as i8 as i32) >> (n - 1)) & 1 != 0),
                };
                self.set_r(idx, r as u8); self.set_flag(PSW_CY, cy);
            }
            0xC..=0xF if hi >= 1 => {
                let n = hi as u32;
                let i = if c == 0xC { 1 } else { 0 };
                let v = self.rp(i);
                let (r, cy) = match c {
                    0xC | 0xD => ((v as u32) << n, (v as u32 >> (16 - n)) & 1 != 0),
                    0xE => ((v as u32) >> n, (v as u32 >> (n - 1)) & 1 != 0),
                    _ => (((v as i16 as i32) >> n) as u32, ((v as i16 as i32) >> (n - 1)) & 1 != 0),
                };
                self.set_rp(i, r as u16); self.set_flag(PSW_CY, cy);
            }
            _ => return Err(self.illegal(bus)),
        }
        Ok(())
    }
}

/// Instruction lengths for the 1st map (prefix-free), from the operation list.
const LEN1: [u8; 256] = {
    let mut t = [1u8; 256];
    let mut i = 0;
    while i < 256 {
        let (hi, lo) = (i >> 4, i & 15);
        let mut l = 1;
        if hi <= 7 && lo >= 0xA {
            l = match lo { 0xA => 3, 0xB => 2, 0xC => 2, 0xD => 1, 0xE => 2, _ => 3 };
        }
        t[i] = l;
        i += 1;
    }
    // explicit entries
    let list: [(usize, u8); 112] = [
        (0x02, 3), (0x04, 3), (0x06, 2), (0x09, 3), (0x10, 2), (0x18, 3), (0x19, 4), (0x20, 2), (0x22, 3), (0x24, 3), (0x26, 2),
        (0x28, 3), (0x29, 3), (0x30, 3), (0x32, 3), (0x34, 3), (0x36, 3), (0x38, 4), (0x39, 4), (0x40, 4), (0x41, 2), (0x42, 3),
        (0x44, 3), (0x46, 2), (0x48, 3), (0x49, 3), (0x50, 2), (0x51, 2), (0x52, 2), (0x53, 2), (0x54, 2), (0x55, 2), (0x56, 2),
        (0x57, 2), (0x58, 3), (0x59, 3), (0x68, 3), (0x69, 3), (0x78, 3), (0x79, 3), (0x88, 2), (0x8A, 2), (0x8C, 2), (0x8D, 2),
        (0x8E, 2), (0x8F, 3), (0x98, 2), (0x9A, 2), (0x9C, 2), (0x9D, 2), (0x9E, 2), (0x9F, 3), (0xA0, 3), (0xA2, 3), (0xA4, 2),
        (0xA6, 2), (0xA8, 2), (0xAA, 2), (0xAC, 2), (0xAD, 2), (0xAE, 2), (0xAF, 3), (0xB0, 3), (0xB2, 3), (0xB4, 2), (0xB6, 2),
        (0xB8, 2), (0xBA, 2), (0xBC, 2), (0xBD, 2), (0xBE, 2), (0xBF, 3), (0xC8, 3), (0xC9, 4), (0xCA, 3), (0xCB, 4), (0xCC, 3),
        (0xCD, 3), (0xCE, 3), (0xCF, 4), (0xD4, 2), (0xD5, 3), (0xD8, 2), (0xD9, 3), (0xDA, 2), (0xDB, 3), (0xDC, 2), (0xDD, 2),
        (0xDE, 2), (0xDF, 2), (0xE4, 2), (0xE5, 3), (0xE8, 2), (0xE9, 3), (0xEA, 2), (0xEB, 3), (0xEC, 4), (0xED, 3), (0xEE, 3),
        (0xEF, 2), (0xF4, 2), (0xF5, 3), (0xF8, 2), (0xF9, 3), (0xFA, 2), (0xFB, 3), (0xFC, 4), (0xFD, 3), (0xFE, 3), (0x0A, 3),
        (0x0F, 3), (0x1F, 3),
    ];
    let mut k = 0;
    while k < list.len() { t[list[k].0] = list[k].1; k += 1; }
    t
};
