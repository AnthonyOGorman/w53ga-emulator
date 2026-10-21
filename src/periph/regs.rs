//! Generic on-chip register latches with their access behaviours:
//! read returns `value & read_mask`; writes merge through `mask`; flag registers follow the SH7269
//! read-1-then-write-0 clearing rules.

use crate::checkpoint::MmioLatch;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Behavior { Rw, Ro, W1c, CmtCsr, MtuStatus, DmaChcr, Dmaor, GpioData, Wdt, StartupScratch }

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Latch {
    pub addr: u32,
    pub size: u32,
    pub name: String,
    pub value: u64,
    pub mask: u64,
    pub read_mask: u64,
    pub behavior: Behavior,
    pub cmf_read: bool,
    pub status_read: u64,
    pub direction: u32,
    pub pin_mask: u64,
    pub pin_value: u64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Latches {
    pub regs: Vec<Latch>,
    by_byte: HashMap<u32, usize>,
    /// Dense index for 0xFFFE_0000..0xFFFF_0000 (most on-chip registers): latch index + 1, 0 = none.
    dense: Vec<u16>,
}
const DENSE_BASE: u32 = 0xFFFE_0000;
const DENSE_LEN: usize = 0x1_0000;

impl Latches {
    pub fn from_checkpoint(src: &[MmioLatch]) -> Result<Latches, String> {
        let mut regs = Vec::new();
        for l in src {
            let behavior = match l.behavior.as_str() {
                "rw" => Behavior::Rw, "ro" => Behavior::Ro, "w1c" => Behavior::W1c, "cmt_csr" => Behavior::CmtCsr,
                "mtu_status" => Behavior::MtuStatus, "dma_chcr" => Behavior::DmaChcr, "dmaor" => Behavior::Dmaor,
                "gpio_data" => Behavior::GpioData, "startup_scratch" => Behavior::StartupScratch,
                b if b.starts_with("wdt_") => Behavior::Wdt,
                b => return Err(format!("register {} behaviour {b}", l.name)),
            };
            regs.push(Latch { addr: l.addr as u32, size: l.size, name: l.name.clone(), value: l.value, mask: l.mask, read_mask: l.read_mask,
                behavior, cmf_read: l.cmf_read, status_read: l.status_read, direction: l.direction as u32, pin_mask: l.pin_mask, pin_value: l.pin_value });
        }
        let mut by_byte = HashMap::new();
        let mut dense = vec![0u16; DENSE_LEN];
        for (i, r) in regs.iter().enumerate() {
            for b in 0..r.size {
                by_byte.insert(r.addr + b, i);
                let a = r.addr + b;
                if (DENSE_BASE..DENSE_BASE + DENSE_LEN as u32).contains(&a) { dense[(a - DENSE_BASE) as usize] = i as u16 + 1; }
            }
        }
        Ok(Latches { regs, by_byte, dense })
    }

    /// Index of the latch fully containing [a, a+n).
    #[inline]
    fn index(&self, a: u32) -> Option<usize> {
        if (DENSE_BASE..DENSE_BASE + DENSE_LEN as u32).contains(&a) { let d = self.dense[(a - DENSE_BASE) as usize]; return if d == 0 { None } else { Some(d as usize - 1) }; }
        self.by_byte.get(&a).copied()
    }

    pub fn find(&self, a: u32, n: usize) -> Option<usize> {
        let i = self.index(a)?;
        let r = &self.regs[i];
        if a + n as u32 <= r.addr + r.size { Some(i) } else { None }
    }
    #[inline]
    pub fn value(&self, addr: u32) -> u64 { self.index(addr).map_or(0, |i| self.regs[i].value) }
    pub fn set(&mut self, addr: u32, v: u64) { if let Some(i) = self.index(addr) { self.regs[i].value = v; } }

    fn selected_mask(r: &Latch, a: u32, n: usize) -> u64 {
        (0..n).fold(0, |m, i| m | 0xFF << ((r.size as i64 - 1 - (a - r.addr) as i64 - i as i64) * 8))
    }

    pub fn read(&mut self, i: usize, a: u32, n: usize) -> u64 {
        let sel = Self::selected_mask(&self.regs[i], a, n);
        let dir_value = if self.regs[i].behavior == Behavior::GpioData { self.value(self.regs[i].direction) } else { 0 };
        let r = &mut self.regs[i];
        match r.behavior {
            Behavior::CmtCsr => if r.value & 0x80 != 0 { r.cmf_read = true },
            Behavior::MtuStatus => r.status_read |= r.value & 0x3F & sel,
            Behavior::DmaChcr => r.status_read |= r.value & 0x80002 & sel,
            Behavior::Dmaor => r.status_read |= r.value & 6 & sel,
            Behavior::Wdt => if r.value & 0x8000 != 0 { r.status_read |= 0x80 },
            _ => {}
        }
        let v = if r.behavior == Behavior::GpioData {
            let dir = dir_value & r.pin_mask;
            (r.value & dir) | (r.pin_value & !dir & r.pin_mask)
        } else { r.value & r.read_mask };
        let shift = (r.size as i64 - (a - r.addr) as i64 - n as i64) * 8;
        (v >> shift) & if n == 8 { u64::MAX } else { (1u64 << (8 * n)) - 1 }
    }

    /// Returns the previous value.
    pub fn write(&mut self, i: usize, a: u32, n: usize, data: u64) -> u64 {
        let r = &mut self.regs[i];
        let shift = (r.size as i64 - (a - r.addr) as i64 - n as i64) * 8;
        let sm = Self::selected_mask(r, a, n);
        let selected = (data << shift) & sm;
        let wm = r.mask & sm;
        let before = r.value;
        match r.behavior {
            Behavior::DmaChcr | Behavior::Dmaor => {
                let flags = if r.behavior == Behavior::DmaChcr { 0x80002 } else { 6 };
                let mut saved = r.value & flags;
                saved &= !(r.status_read & sm & !selected);
                r.value = (r.value & !wm) | (selected & wm);
                r.value = (r.value & !flags) | saved;
                r.status_read &= !sm;
            }
            Behavior::MtuStatus => { r.value &= !(r.status_read & !selected & wm); r.status_read = 0; }
            Behavior::CmtCsr => {
                let mut flag = r.value & 0x80;
                if selected & 0x80 == 0 && r.cmf_read { flag = 0; }
                r.value = (r.value & !wm) | (selected & wm);
                r.value = (r.value & !0x80) | flag;
                r.cmf_read = false;
            }
            Behavior::Ro => {}
            Behavior::W1c => r.value &= !(selected & wm),
            Behavior::Wdt => { /* keyed watchdog writes: the firmware never starts the WDT; latch only */ }
            Behavior::Rw | Behavior::GpioData | Behavior::StartupScratch => r.value = (r.value & !wm) | (selected & wm),
        }
        before
    }
}
