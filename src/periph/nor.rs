//! CS0 NOR flash (32 MiB, x16) command interface: AA@AAA/55@554
//! unlock, word program (A0), write buffer (25 / count / data / 29), sector erase (80, AA, 55, 30), reset (F0) and the
//! PPB (C0) / DYB (E0) protection command sets. Completion is synchronous (polling sees stable array data).
//! The array itself is `Memory::flash`; changes are queued here and applied by `Board::tick_with`.

pub const SIZE: usize = 0x200_0000;
pub const SECTOR: usize = 0x2_0000;

#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
enum Mode { Array, Ppb, Dyb }
#[derive(Clone, Copy, PartialEq, Eq, Debug, serde::Serialize, serde::Deserialize)]
enum Phase { Idle, Unlock1, Unlock2, EraseUnlock1, EraseUnlock2, EraseConfirm, WordProgram, BufferCount, BufferData, BufferConfirm, DybProgram, DybExit, PpbProgram, PpbErase, PpbExit }

/// A change to apply to the array: program (AND) a 16-bit word, or erase a sector.
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub enum Change { Program(usize, u16), Erase(usize) }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Nor {
    mode: Mode,
    phase: Phase,
    target: usize,
    remaining: u32,
    buffered: Vec<(usize, u16)>,
    #[serde(with = "serde_big_array::BigArray")]
    ppb: [bool; 256],
    #[serde(with = "serde_big_array::BigArray")]
    dyb: [bool; 256],
    ppb_erase_complete: bool,
    pub pending: Vec<Change>,
    pub bus_writes: u64,
    pub programs: u64,
    pub erases: u64,
    pub ignored: u64,
}

impl Default for Nor {
    fn default() -> Self {
        Nor { mode: Mode::Array, phase: Phase::Idle, target: 0, remaining: 0, buffered: Vec::new(), ppb: [false; 256], dyb: [false; 256],
            ppb_erase_complete: false, pending: Vec::new(), bus_writes: 0, programs: 0, erases: 0, ignored: 0 }
    }
}

impl Nor {
    pub fn handles(a: u32) -> bool { (a & 0xDFFF_FFFF) < SIZE as u32 }

    pub fn in_command_mode(&self) -> bool { self.mode != Mode::Array }

    /// Value read in a protection mode (None = normal array read).
    pub fn mode_read(&self, a: u32) -> Option<u16> {
        let sector = (a as usize & (SIZE - 1)) / SECTOR;
        match self.mode {
            Mode::Array => None,
            Mode::Dyb => Some(if self.dyb[sector] { 0 } else { 1 }),
            Mode::Ppb => Some(if self.ppb_erase_complete { 9 } else if self.ppb[sector] { 0 } else { 1 }),
        }
    }

    /// A CPU store to the flash window. Wider stores are split into x16 bus cycles (big-endian); byte stores and
    /// anything that does not form a valid command sequence make the chip return to read-array mode, as on hardware.
    pub fn write(&mut self, a: u32, n: usize, v: u64) -> Result<(), String> {
        if std::env::var_os("AR_NOR_LOG").is_some() { eprintln!("[nor] write {a:08x}/{n} = {v:x} (mode {:?} phase {:?})", self.mode, self.phase); }
        if n == 4 && a & 1 == 0 { self.write(a, 2, v >> 16)?; return self.write(a + 2, 2, v & 0xFFFF); }
        if n != 2 || a & 1 != 0 { self.ignored(a, n, v); return Ok(()); }
        match self.command(a, v) { Ok(()) => Ok(()), Err(e) => { self.ignored(a, n, v); if self.ignored <= 4 { eprintln!("[nor] {e} -> read-array mode"); } Ok(()) } }
    }

    fn ignored(&mut self, a: u32, n: usize, v: u64) {
        self.ignored += 1;
        if self.ignored <= 4 { eprintln!("[nor] stray store {a:08x}/{n} = {v:x} ignored"); }
        self.mode = Mode::Array; self.phase = Phase::Idle; self.buffered.clear();
    }

    fn command(&mut self, a: u32, v: u64) -> Result<(), String> {
        let at = a as usize & (SIZE - 1);
        let value = v as u16;
        self.bus_writes += 1;
        if value == 0xF0 && self.phase == Phase::Idle { self.mode = Mode::Array; self.ppb_erase_complete = false; return Ok(()); }
        let sector = at / SECTOR;
        match self.mode {
            Mode::Dyb => {
                match self.phase {
                    Phase::DybProgram => { if value > 1 || sector != self.target / SECTOR { return Err("DYB payload/range".into()); } self.dyb[sector] = value == 0; self.phase = Phase::Idle; }
                    Phase::DybExit => { if value != 0 { return Err("DYB exit confirm".into()); } self.mode = Mode::Array; self.phase = Phase::Idle; }
                    _ if value == 0xA0 => { self.target = at; self.phase = Phase::DybProgram; }
                    _ if value == 0x90 => self.phase = Phase::DybExit,
                    _ => return Err(format!("DYB command {value:04x} at {at:08x}")),
                }
                return Ok(());
            }
            Mode::Ppb => {
                match self.phase {
                    Phase::PpbProgram => { if value != 0 || sector != self.target / SECTOR { return Err("PPB program payload".into()); } self.ppb[sector] = true; self.ppb_erase_complete = false; self.phase = Phase::Idle; }
                    Phase::PpbErase => { if value != 0x30 || at != 0 { return Err("PPB erase confirm".into()); } self.ppb = [false; 256]; self.ppb_erase_complete = true; self.phase = Phase::Idle; }
                    Phase::PpbExit => { if value != 0 { return Err("PPB exit confirm".into()); } self.mode = Mode::Array; self.phase = Phase::Idle; self.ppb_erase_complete = false; }
                    _ if value == 0xA0 => { self.target = at; self.phase = Phase::PpbProgram; }
                    _ if value == 0x80 && at == 0 => self.phase = Phase::PpbErase,
                    _ if value == 0x90 => self.phase = Phase::PpbExit,
                    _ => return Err(format!("PPB command {value:04x} at {at:08x}")),
                }
                return Ok(());
            }
            Mode::Array => {}
        }
        self.phase = match self.phase {
            Phase::Idle => { if at == 0xAAA && value == 0xAA { Phase::Unlock1 } else { return Err(format!("NOR command {value:04x}@{at:x} without AA@AAA unlock")); } }
            Phase::Unlock1 => { if at == 0x554 && value == 0x55 { Phase::Unlock2 } else { return Err("NOR unlock requires 55@554".into()); } }
            Phase::Unlock2 => match (at, value) {
                (0xAAA, 0xF0) => { self.mode = Mode::Array; self.ppb_erase_complete = false; Phase::Idle }
                (0xAAA, 0xE0) => { self.mode = Mode::Dyb; Phase::Idle }
                (0xAAA, 0xC0) => { self.mode = Mode::Ppb; self.ppb_erase_complete = false; Phase::Idle }
                (0xAAA, 0x80) => Phase::EraseUnlock1,
                (0xAAA, 0xA0) => Phase::WordProgram,
                (_, 0x25) => { self.target = at; self.buffered.clear(); Phase::BufferCount }
                _ => return Err(format!("NOR unlocked command {value:04x}@{at:x}")),
            },
            Phase::EraseUnlock1 => { if at == 0xAAA && value == 0xAA { Phase::EraseUnlock2 } else { return Err("NOR erase unlock1".into()); } }
            Phase::EraseUnlock2 => { if at == 0x554 && value == 0x55 { Phase::EraseConfirm } else { return Err("NOR erase unlock2".into()); } }
            Phase::EraseConfirm => {
                if value != 0x30 { return Err("NOR erase confirm".into()); }
                if self.ppb[sector] || self.dyb[sector] { return Err(format!("NOR erase of protected sector {sector}")); }
                self.pending.push(Change::Erase(sector * SECTOR)); self.erases += 1; Phase::Idle
            }
            Phase::WordProgram => { self.program(at, value)?; Phase::Idle }
            Phase::BufferCount => {
                if value >= 32 || sector != self.target / SECTOR { return Err("NOR buffer count".into()); }
                self.remaining = value as u32 + 1; Phase::BufferData
            }
            Phase::BufferData => {
                if sector != self.target / SECTOR || self.buffered.first().is_some_and(|f| f.0 / 64 != at / 64) || self.buffered.iter().any(|b| b.0 == at) { return Err("NOR buffer data boundary".into()); }
                self.buffered.push((at, value)); self.remaining -= 1;
                if self.remaining == 0 { Phase::BufferConfirm } else { Phase::BufferData }
            }
            Phase::BufferConfirm => {
                if value != 0x29 || sector != self.target / SECTOR { return Err("NOR buffer confirm".into()); }
                for (a, v) in std::mem::take(&mut self.buffered) { self.program(a, v)?; }
                Phase::Idle
            }
            p => return Err(format!("NOR phase {p:?}")),
        };
        Ok(())
    }

    fn program(&mut self, at: usize, value: u16) -> Result<(), String> {
        let sector = at / SECTOR;
        if self.ppb[sector] || self.dyb[sector] { return Err(format!("NOR program of protected sector {sector}")); }
        self.pending.push(Change::Program(at, value));
        self.programs += 1;
        Ok(())
    }

    /// Apply queued changes to the array (big-endian x16 words).
    pub fn apply(&mut self, flash: &mut [u8]) -> Result<(), String> {
        for c in self.pending.drain(..) {
            match c {
                Change::Erase(a) => flash[a..a + SECTOR].fill(0xFF),
                Change::Program(a, v) => {
                    let old = u16::from_be_bytes([flash[a], flash[a + 1]]);
                    if old & v != v { return Err(format!("NOR program {a:08x}: {old:04x} -> {v:04x} needs erase")); }
                    flash[a..a + 2].copy_from_slice(&v.to_be_bytes());
                }
            }
        }
        Ok(())
    }
}
