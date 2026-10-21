//! SFR / 2nd-SFR register file for the RL78/G13 with access logging; peripherals are layered on top.

use super::cpu::Rl78Bus;
use std::collections::BTreeMap;

pub struct SfrBus {
    pub sfr: [u8; 0x100],
    pub sfr2: Vec<u8>,
    /// address -> (reads, writes, last value written)
    pub log: BTreeMap<u32, (u64, u64, u8)>,
    pub logging: bool,
}

impl Default for SfrBus {
    fn default() -> Self { SfrBus { sfr: [0; 0x100], sfr2: vec![0; 0x800], log: BTreeMap::new(), logging: true } }
}

impl SfrBus {
    fn cell(&mut self, a: u32) -> Option<&mut u8> {
        match a { 0xFFF00..=0xFFFFF => Some(&mut self.sfr[(a - 0xFFF00) as usize]), 0xF0000..=0xF07FF => Some(&mut self.sfr2[(a - 0xF0000) as usize]), _ => None }
    }
}

impl Rl78Bus for SfrBus {
    fn read(&mut self, a: u32) -> u8 {
        if self.logging { self.log.entry(a).or_insert((0, 0, 0)).0 += 1; }
        self.cell(a).map_or(0xFF, |c| *c)
    }
    fn write(&mut self, a: u32, v: u8) {
        if self.logging { let e = self.log.entry(a).or_insert((0, 0, 0)); e.1 += 1; e.2 = v; }
        if let Some(c) = self.cell(a) { *c = v; }
    }
}
