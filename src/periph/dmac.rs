//! Direct memory access controller (SH7269 DMAC, 16 channels).
//!
//! Registers per channel at 0xFFFE_1000 + 16n: SAR, DAR, DMATCR, CHCR. DMARS0..7 at 0xFFFE_1300 + 4k select the
//! peripheral request for channel pairs (even channel = low byte, odd = high byte). DMAOR stays a board latch.
//! Model: cycle-steal, one transfer unit per step for the whole controller, fixed priority (lowest channel first).
//! Peripheral requests are supplied by the board (`Request`); a channel whose selected request source is not
//! modelled simply never sees a request. HE at half count, TE at zero; DEI (vector 108+4n) when IE, HEI (109+4n)
//! when HIE. CHCR flags HE/TE clear by read-1-then-write-0.

pub const BASE: u32 = 0xFFFE_1000;
pub const DMARS_BASE: u32 = 0xFFFE_1300;
pub const RELOAD_BASE: u32 = 0xFFFE_1100;
const RLDSAR: u32 = 1 << 29;
const RLDDAR: u32 = 1 << 28;
const TEMASK: u32 = 1 << 20;
pub const DE: u32 = 1;
pub const TE: u32 = 2;
pub const IE: u32 = 4;
pub const HIE: u32 = 0x4_0000;
pub const HE: u32 = 0x8_0000;
const FLAGS: u32 = HE | TE;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Channel {
    pub sar: u32, pub dar: u32, pub tcr: u32, pub chcr: u32,
    /// Reload registers RSAR/RDAR/RDMATCR (0xFFFE_1100 + 16n).
    pub rsar: u32, pub rdar: u32, pub rtcr: u32,
    pub seen: u32,
    /// DMATCR at the arming store (for the HE half-count threshold).
    pub arm_count: u32,
    pub units: u64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Dmac {
    pub ch: [Channel; 16],
    pub dmars: [u16; 8],
    pub units: u64,
    /// Bit c set when channel c has DE=1, TE=0 (refreshed on CHCR writes and transfers).
    armed: u16,
    /// Bit c set when channel c asserts DEI or HEI.
    irq: u16,
}

/// One transfer unit the board must perform: read `size` bytes at `src` then write them to `dst`.
#[derive(Clone, Copy, Debug)]
pub struct Unit { pub channel: usize, pub src: u32, pub dst: u32, pub size: usize, pub saf: bool, pub daf: bool }

impl Dmac {
    pub fn new() -> Dmac { Dmac { ch: [Channel::default(); 16], dmars: [0; 8], units: 0, armed: 0, irq: 0 } }

    /// Recompute the armed/irq masks (after loading state or any CHCR change).
    pub fn refresh(&mut self) {
        self.armed = 0; self.irq = 0;
        for (c, x) in self.ch.iter().enumerate() {
            let streaming = x.chcr & TEMASK != 0 && x.chcr & (RLDSAR | RLDDAR) != 0;
            if x.chcr & DE != 0 && (x.chcr & TE == 0 || streaming) { self.armed |= 1 << c; }
            if x.chcr & (IE | TE) == (IE | TE) || x.chcr & (HIE | HE) == (HIE | HE) { self.irq |= 1 << c; }
        }
    }

    pub fn handles(a: u32, n: usize) -> bool {
        n > 0 && ((a >= BASE && a as u64 + n as u64 <= BASE as u64 + 0x100) || (a >= RELOAD_BASE && a as u64 + n as u64 <= RELOAD_BASE as u64 + 0x100)
            || (a >= DMARS_BASE && a as u64 + n as u64 <= DMARS_BASE as u64 + 0x20))
    }

    pub fn selector(&self, c: usize) -> u8 { let w = self.dmars[c / 2]; if c & 1 == 1 { (w >> 8) as u8 } else { w as u8 } }

    pub fn read(&mut self, a: u32, n: usize) -> Result<u64, String> {
        if a >= DMARS_BASE {
            let k = ((a - DMARS_BASE) / 4) as usize;
            if (a - DMARS_BASE) % 4 != 0 || n != 2 { return Err(format!("DMARS access {a:08x}/{n}")); }
            return Ok(self.dmars[k] as u64);
        }
        let reload = a >= RELOAD_BASE;
        let base = if reload { RELOAD_BASE } else { BASE };
        let (c, reg, lane) = (((a - base) / 16) as usize, ((a - base) % 16) / 4, (a - base) % 4);
        if lane as usize + n > 4 || a as usize % n != 0 { return Err(format!("DMAC misaligned {a:08x}/{n}")); }
        let ch = &mut self.ch[c];
        let v = match (reload, reg) { (false, 0) => ch.sar, (false, 1) => ch.dar, (false, 2) => ch.tcr, (false, _) => ch.chcr,
            (true, 0) => ch.rsar, (true, 1) => ch.rdar, (true, 2) => ch.rtcr, (true, _) => return Err(format!("DMAC reserved {a:08x}")) };
        let shift = (4 - lane as usize - n) * 8;
        let mask = if n == 4 { u32::MAX } else { (1u32 << (n * 8)) - 1 };
        if reg == 3 { ch.seen |= ((v >> shift) & mask) << shift & FLAGS; }
        Ok(((v >> shift) & mask) as u64)
    }

    pub fn write(&mut self, a: u32, n: usize, value: u64) -> Result<(), String> {
        if a >= DMARS_BASE {
            let k = ((a - DMARS_BASE) / 4) as usize;
            if (a - DMARS_BASE) % 4 != 0 || n != 2 { return Err(format!("DMARS access {a:08x}/{n}")); }
            for c in [2 * k, 2 * k + 1] {
                let new_sel = if c & 1 == 1 { (value >> 8) as u8 } else { value as u8 };
                if self.ch[c].chcr & DE != 0 && new_sel != self.selector(c) { return Err(format!("DMARS change for armed channel {c}")); }
            }
            self.dmars[k] = value as u16;
            return Ok(());
        }
        let reload = a >= RELOAD_BASE;
        let base = if reload { RELOAD_BASE } else { BASE };
        let (c, reg, lane) = (((a - base) / 16) as usize, ((a - base) % 16) / 4, (a - base) % 4);
        if lane as usize + n > 4 || a as usize % n != 0 { return Err(format!("DMAC misaligned {a:08x}/{n}")); }
        let shift = (4 - lane as usize - n) * 8;
        if reload {
            let lm: u32 = (if n == 4 { u32::MAX } else { (1u32 << (n * 8)) - 1 }) << shift;
            let w = (value as u32) << shift;
            let ch = &mut self.ch[c];
            match reg { 0 => ch.rsar = (ch.rsar & !lm) | (w & lm), 1 => ch.rdar = (ch.rdar & !lm) | (w & lm), 2 => ch.rtcr = ((ch.rtcr & !lm) | (w & lm)) & 0x00FF_FFFF, _ => return Err(format!("DMAC reserved {a:08x}")) }
            return Ok(());
        }
        let lm: u32 = (if n == 4 { u32::MAX } else { (1u32 << (n * 8)) - 1 }) << shift;
        let w = (value as u32) << shift;
        let ch = &mut self.ch[c];
        match reg {
            0 => ch.sar = (ch.sar & !lm) | (w & lm),
            1 => ch.dar = (ch.dar & !lm) | (w & lm),
            2 => ch.tcr = ((ch.tcr & !lm) | (w & lm)) & 0x00FF_FFFF,
            _ => {
                let clear = (ch.seen & lm) & !w;
                let flags = (ch.chcr & FLAGS) & !clear;
                let config = ((ch.chcr & !lm) | (w & lm)) & !FLAGS;
                let was = ch.chcr & DE != 0;
                ch.chcr = config | flags;
                ch.seen &= !lm;
                if !was && ch.chcr & DE != 0 { ch.arm_count = ch.tcr; }
                self.refresh();
            }
        }
        Ok(())
    }

    /// Is channel `c` enabled to transfer (DE=1, TE=0, DMAOR.DME=1, no AE/NMIF)?
    pub fn enabled(&self, c: usize, dmaor: u64) -> bool {
        let ch = &self.ch[c];
        let streaming = ch.chcr & TEMASK != 0 && ch.chcr & (RLDSAR | RLDDAR) != 0;
        ch.chcr & DE != 0 && (ch.chcr & TE == 0 || streaming) && dmaor & 1 != 0 && dmaor & 6 == 0
    }

    /// Choose the highest-priority channel with an active request (`request(channel, selector)`), commit the
    /// register side of one unit and return it for the board to move the data.
    /// Any channel with DE=1 and TE=0 (cheap pre-check for the per-step path).
    #[inline]
    pub fn any_armed(&self) -> bool { self.armed != 0 }

    /// Channels currently able to transfer (bit mask).
    pub fn armed_mask(&self) -> u16 { self.armed }

    /// Any DEI/HEI level asserted.
    #[inline]
    pub fn any_irq(&self) -> bool { self.irq != 0 }

    pub fn next_unit(&mut self, dmaor: u64, request: impl Fn(usize, u8) -> bool) -> Result<Option<Unit>, String> {
        for c in 0..16 {
            if !self.enabled(c, dmaor) { continue; }
            let chcr = self.ch[c].chcr;
            let rs = (chcr >> 8) & 15;
            let active = match rs {
                0b1000 => request(c, self.selector(c)),
                0b0100 => true, // auto request
                _ => return Err(format!("DMA channel {c}: request mode RS={rs:04b} not modelled")),
            };
            if !active { continue; }
            let size = match (chcr >> 3) & 3 { 0 => 1, 1 => 2, 2 => 4, _ => 16 };
            let ch = &mut self.ch[c];
            let unit = Unit { channel: c, src: ch.sar, dst: ch.dar, size, saf: false, daf: false };
            let step = |mode: u32, v: u32| match mode { 0 => v, 1 => v.wrapping_add(size as u32), 2 => v.wrapping_sub(size as u32), _ => v };
            let unit = Unit { saf: chcr & (1 << 25) != 0, daf: chcr & (1 << 26) != 0, ..unit };
            ch.sar = step((chcr >> 12) & 3, ch.sar);
            ch.dar = step((chcr >> 14) & 3, ch.dar);
            ch.tcr = ch.tcr.wrapping_sub(1) & 0x00FF_FFFF;
            if ch.tcr <= ch.arm_count / 2 { ch.chcr |= HE; }
            if ch.tcr == 0 {
                ch.chcr |= TE;
                // Reload (manual 11.3.5-11.3.7): SAR/DAR and DMATCR take the reload registers at transfer end.
                if chcr & (RLDSAR | RLDDAR) != 0 {
                    if chcr & RLDSAR != 0 { ch.sar = ch.rsar; }
                    if chcr & RLDDAR != 0 { ch.dar = ch.rdar; }
                    ch.tcr = ch.rtcr;
                    ch.arm_count = ch.tcr;
                }
            }
            ch.units += 1;
            self.units += 1;
            self.refresh();
            return Ok(Some(unit));
        }
        Ok(None)
    }

    /// Pending DMA interrupts as (vector, channel).
    pub fn requests(&self) -> impl Iterator<Item = (u32, usize)> + '_ {
        (0..16).flat_map(move |c| {
            let x = self.ch[c].chcr;
            let dei = (x & (IE | TE) == (IE | TE)).then_some((108 + 4 * c as u32, c));
            let hei = (x & (HIE | HE) == (HIE | HE)).then_some((109 + 4 * c as u32, c));
            [dei, hei].into_iter().flatten()
        })
    }

    /// IPR register and shift for channel `c` (IPR06..IPR09, four channels each, highest nibble first).
    pub fn ipr(c: usize) -> (u32, u32) { (0xFFFE_0C00 + 2 * (c / 4) as u32, 12 - 4 * (c % 4) as u32) }
}
