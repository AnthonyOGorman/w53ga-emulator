//! NEC infrared remote: schedule of falling edges of the demodulated receiver output on MTU1 TIOC1A.
//! Timing (absolute P0 ticks, P0 = 100/3 MHz): leader 9 + 4.5 ms, bit 0 = 1.125 ms, bit 1 = 2.25 ms, 32 bits LSB
//! first (customer 0x84E4, command, ~command), repeat frames (9 + 2.25 ms) every 108 ms while held. Decoded by the
//! MTU1 TGIA interrupt handler; the release is the MTU1 overflow ~126 ms after the last edge.
//!
//! Remote command bytes the unit's remote sends: 0x00, 0x01, 0x02, 0x08, 0x09, 0x0A, 0x10,
//! 0x11, 0x12, 0x64, 0xFF -> key codes 1..10, 15.

use std::collections::VecDeque;

pub const CUSTOMER: u32 = 0x84E4;

#[inline]
pub fn us(us: u64) -> u64 { us * 100 / 3 }

/// Falling-edge times for one press starting at `start`, held for `repeats` repeat frames.
pub fn press_edges(start: u64, command: u8, repeats: u32) -> Vec<u64> {
    let mut e = vec![start];
    let mut t = start + us(13_500);
    e.push(t);
    let word = CUSTOMER | (command as u32) << 16 | ((!command) as u32) << 24;
    for i in 0..32 { t += if (word >> i) & 1 != 0 { us(2_250) } else { us(1_125) }; e.push(t); }
    for r in 1..=repeats as u64 { let r0 = start + r * us(108_000); e.push(r0); e.push(r0 + us(11_250)); }
    e
}

#[derive(Default, serde::Serialize, serde::Deserialize)]
pub struct IrRemote {
    pub edges: VecDeque<u64>,
    /// Earliest step a new press may start (previous press edges + release overflow).
    pub busy_until: u64,
    pub presses: u64,
}

impl IrRemote {
    /// Queue a press at or after `now`; returns its start step.
    pub fn press(&mut self, now: u64, command: u8, repeats: u32) -> u64 {
        let start = now.max(self.busy_until) + 1;
        let e = press_edges(start, command, repeats);
        self.busy_until = e.last().unwrap() + us(130_000);
        self.edges.extend(e);
        self.presses += 1;
        start
    }
    #[inline]
    /// An edge scheduled at or before `step` (batched ticks cover several steps).
    pub fn edge_at(&mut self, step: u64) -> bool {
        if self.edges.front().is_some_and(|&e| e <= step) { self.edges.pop_front(); true } else { false }
    }
}
