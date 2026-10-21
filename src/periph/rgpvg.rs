//! The 2D/vector graphics core used by the RGAH library (0xE810_0000..0xE813_0000), as far as its start-up needs it.
//! Graphics core start-up model: the firmware's
//! RGAH initialisation uploads a 74-slot shader list and then runs one clear list; each list ends in a VAD_TRAP that
//! raises pending status bit 2 (STATUS 0xE810_4018), signalled as RGPVG_INT1 (vector 186, IPR11 bits 3..0) when
//! enabled in IER (0xE810_4014), and acknowledged by writing the bit to 0xE810_401C. Shaders are not executed and no
//! pixels are produced here: drawing goes through the RGAH API, rendered in software (`gfx`).
//! Reads other than STATUS return documented assumptions: the bank-select word 0xE810_4024 reads back
//! its last written value (initially 0), every other word reads 0.

use std::collections::BTreeMap;

pub const BASE: u32 = 0xE810_0000;
pub const END: u32 = 0xE813_0000;
const SUBMIT: u32 = 0xE810_2000;
const STATUS: u32 = 0xE810_4018;
const ACK: u32 = 0xE810_401C;
const IER: u32 = 0xE810_4014;
pub const VECTOR: u32 = 186;

#[derive(Default, Clone, Debug)]
pub struct Rgpvg {
    pub regs: BTreeMap<u32, u32>,
    pub pending: u32,
    /// Lists submitted (start address written to 0xE810_2000).
    pub submits: u32,
    /// Register accesses by the CPU and the last one (address, write), for runtime HLE detection (`crate::detect`).
    pub accesses: u64,
    pub last: (u32, bool),
}

impl Rgpvg {
    pub fn handles(a: u32) -> bool { (BASE..END).contains(&a) }

    pub fn read(&mut self, a: u32, n: usize) -> Result<u64, String> {
        self.accesses += 1;
        self.last = (a, false);
        if n != 4 || a & 3 != 0 { return Err(format!("graphics core read {a:08x}/{n}")); }
        Ok(match a {
            STATUS => self.pending as u64,
            0xE810_4024 => self.regs.get(&a).copied().unwrap_or(0) as u64,
            _ => 0,
        })
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> Result<(), String> {
        if n != 4 || a & 3 != 0 { return Err(format!("graphics core write {a:08x}/{n}")); }
        self.accesses += 1;
        self.last = (a, true);
        let v = v as u32;
        self.regs.insert(a, v);
        match a {
            SUBMIT => { self.submits += 1; self.pending |= 2; } // the list runs to its final VAD_TRAP
            ACK => self.pending &= !v,
            _ => {}
        }
        Ok(())
    }

    /// RGPVG_INT1 request (pending and enabled).
    pub fn irq(&self) -> bool { self.pending & self.regs.get(&IER).copied().unwrap_or(0) & 2 != 0 }
}
