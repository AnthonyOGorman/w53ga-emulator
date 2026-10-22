//! SH7269 address map used by the AR-W53GA firmware (big-endian).
//!
//! Plain memory is held in flat arrays; everything else goes to the MMIO bus.
//! Regions and aliases:
//! - CS0 NOR flash at 0x0000_0000, uncached mirror 0x2000_0000 (32 MiB dump).
//! - SDRAM window 0x1C00_0000..0x1C28_0000 (2.5 MiB used), uncached mirror 0x3C00_0000.
//! - On-chip high-speed RAM 0xFFF8_0000..0xFFF9_0000 (64 KiB).

pub const FLASH_SIZE: usize = 32 << 20;
pub const SDRAM_BASE: u32 = 0x1C00_0000;
pub const SDRAM_SIZE: usize = 0x0028_0000;
pub const ORAM_BASE: u32 = 0xFFF8_0000;
pub const ORAM_SIZE: usize = 0x0001_0000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Region {
    Flash(usize),
    Sdram(usize),
    Oram(usize),
    Mmio,
}

#[inline]
pub fn region(a: u32) -> Region {
    match a {
        0x0000_0000..=0x01FF_FFFF => Region::Flash(a as usize),
        0x2000_0000..=0x21FF_FFFF => Region::Flash((a - 0x2000_0000) as usize),
        0x1C00_0000..=0x1C27_FFFF => Region::Sdram((a - SDRAM_BASE) as usize),
        0x3C00_0000..=0x3C27_FFFF => Region::Sdram((a - 0x3C00_0000) as usize),
        0xFFF8_0000..=0xFFF8_FFFF => Region::Oram((a - ORAM_BASE) as usize),
        _ => Region::Mmio,
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Memory {
    /// The NOR chip is in a command mode (protection-bit reads): flash data reads go to the board's NOR model.
    #[serde(skip)]
    pub flash_cmd_mode: bool,
    /// Counts changes to the flash array made by the NOR model (programming, erasing), so that decoded code is checked.
    #[serde(skip)]
    pub code_gen: u64,
    pub flash: Vec<u8>,
    pub sdram: Vec<u8>,
    pub oram: Vec<u8>,
}

impl Memory {
    pub fn new(flash: Vec<u8>) -> Self {
        assert_eq!(flash.len(), FLASH_SIZE, "flash dump must be 32 MiB");
        Memory { flash_cmd_mode: false, code_gen: 0, flash, sdram: vec![0; SDRAM_SIZE], oram: vec![0; ORAM_SIZE] }
    }

    /// Slice for `n` bytes of plain memory at `a`, or None for MMIO / region-crossing accesses.
    #[inline]
    fn plain(&self, a: u32, n: usize) -> Option<&[u8]> {
        match region(a) {
            Region::Flash(o) if o + n <= FLASH_SIZE => Some(&self.flash[o..o + n]),
            Region::Sdram(o) if o + n <= SDRAM_SIZE => Some(&self.sdram[o..o + n]),
            Region::Oram(o) if o + n <= ORAM_SIZE => Some(&self.oram[o..o + n]),
            _ => None,
        }
    }

    #[inline]
    fn plain_mut(&mut self, a: u32, n: usize) -> Option<&mut [u8]> {
        match region(a) {
            // NOR flash writes are command sequences, handled by the flash model on the MMIO path.
            Region::Sdram(o) if o + n <= SDRAM_SIZE => Some(&mut self.sdram[o..o + n]),
            Region::Oram(o) if o + n <= ORAM_SIZE => Some(&mut self.oram[o..o + n]),
            _ => None,
        }
    }

    #[inline]
    pub fn read_plain(&self, a: u32, n: usize) -> Option<u64> {
        if self.flash_cmd_mode && matches!(region(a), Region::Flash(_)) { return None; }
        let s = self.plain(a, n)?;
        Some(match n {
            1 => s[0] as u64,
            2 => u16::from_be_bytes([s[0], s[1]]) as u64,
            4 => u32::from_be_bytes([s[0], s[1], s[2], s[3]]) as u64,
            _ => s.iter().fold(0u64, |v, &b| (v << 8) | b as u64),
        })
    }

    /// Data read fast path for an aligned 1-, 2- or 4-byte access: SDRAM (either mirror), and flash while it is in
    /// array mode. None: not one of those (use `read_plain`).
    #[inline(always)]
    pub fn read_fast(&self, a: u32, n: usize) -> Option<u32> {
        let o = (a & 0xDFFF_FFFF).wrapping_sub(SDRAM_BASE) as usize;
        let s = if o < SDRAM_SIZE { &self.sdram[o..] }
            else if a & 0xDFFF_FFFF < FLASH_SIZE as u32 && !self.flash_cmd_mode { &self.flash[(a & 0x01FF_FFFF) as usize..] }
            else { return None };
        Some(match n { 1 => s[0] as u32, 2 => u16::from_be_bytes([s[0], s[1]]) as u32, _ => u32::from_be_bytes([s[0], s[1], s[2], s[3]]) })
    }

    /// Data write fast path for an aligned 1-, 2- or 4-byte access to SDRAM (either mirror). False: not SDRAM (use
    /// `write_plain`).
    #[inline(always)]
    pub fn write_fast(&mut self, a: u32, n: usize, v: u32) -> bool {
        let o = (a & 0xDFFF_FFFF).wrapping_sub(SDRAM_BASE) as usize;
        if o >= SDRAM_SIZE { return false; }
        let s = &mut self.sdram[o..];
        match n { 1 => s[0] = v as u8, 2 => s[..2].copy_from_slice(&(v as u16).to_be_bytes()), _ => s[..4].copy_from_slice(&v.to_be_bytes()) }
        true
    }

    /// Instruction fetch fast path (flash and on-chip RAM), falling back to the general lookup.
    #[inline(always)]
    pub fn fetch16(&self, a: u32) -> Option<u16> {
        if a < 0x0200_0000 { let i = a as usize; return Some(u16::from_be_bytes([self.flash[i], self.flash[i + 1]])); }
        self.read_plain(a, 2).map(|v| v as u16)
    }

    #[inline]
    pub fn write_plain(&mut self, a: u32, n: usize, v: u64) -> bool {
        match self.plain_mut(a, n) {
            Some(s) => {
                match n {
                    1 => s[0] = v as u8,
                    2 => s.copy_from_slice(&(v as u16).to_be_bytes()),
                    4 => s.copy_from_slice(&(v as u32).to_be_bytes()),
                    _ => for (i, b) in s.iter_mut().enumerate() { *b = (v >> (8 * (n - 1 - i))) as u8; },
                }
                true
            }
            None => false,
        }
    }

    /// Raw byte store used by checkpoint loading and external (DMA/peripheral) writes.
    pub fn store_bytes(&mut self, a: u32, bytes: &[u8]) -> bool {
        match self.plain_mut(a, bytes.len()) {
            Some(s) => {
                s.copy_from_slice(bytes);
                true
            }
            None => false,
        }
    }

    /// Borrow `n` bytes of plain memory (no copy).
    pub fn slice(&self, a: u32, n: usize) -> Option<&[u8]> { self.plain(a, n) }

    pub fn load_bytes(&self, a: u32, n: usize) -> Option<Vec<u8>> {
        self.plain(a, n).map(|s| s.to_vec())
    }
}
