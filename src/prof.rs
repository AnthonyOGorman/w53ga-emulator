//! Minimal sampling profiler: the emulator marks its current phase in an atomic, a sampler thread histograms it.

use std::sync::atomic::{AtomicU8, Ordering};

pub static PHASE: AtomicU8 = AtomicU8::new(0);
pub const CPU: u8 = 1;
pub const BOARD: u8 = 2;
pub const RL78: u8 = 3;
pub const DISPLAY: u8 = 4;
pub const DMA: u8 = 5;
pub const OTHER: u8 = 6;
pub const IRQ: u8 = 7;
pub const SERIAL: u8 = 8;
pub const TIMERS: u8 = 9;
pub const AUDIO: u8 = 10;
pub const NAMES: [&str; 11] = ["idle", "sh-cpu", "board", "rl78", "display", "dma", "other", "irq-scan", "serial", "timers", "audio"];

#[inline(always)]
pub fn set(p: u8) { PHASE.store(p, Ordering::Relaxed); }
