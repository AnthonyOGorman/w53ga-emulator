//! Watchdog timer (0xFFFE_0000): WTCSR, WTCNT, WRCSR. Writes are keyed 16-bit stores (0xA5xx to WTCSR, 0x5Axx to
//! WTCNT; WRCSR: 0xA500 clears WOVF, 0x5Axx sets RSTE/RSTS), reads are 8-bit. The counter runs on P0 divided by the
//! WTCSR.CKS clock; in watchdog mode (WT/IT = 1) an overflow sets WRCSR.WOVF and, with RSTE, resets the chip (power-on
//! reset when RSTS = 0, manual reset otherwise). The boot loader uses this to restart the unit after a firmware update;
//! the main firmware leaves the watchdog stopped. Interval-timer interrupts are not modelled (the firmware does not use
//! them).

pub const BASE: u32 = 0xFFFE_0000;
/// P0 clocks per count for WTCSR.CKS 0..7 (SH7268/SH7269 manual, WDT clock select).
const DIV: [u64; 8] = [1, 64, 128, 256, 512, 1024, 4096, 16384];

#[derive(Default, Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Wdt {
    pub wtcsr: u8,
    pub wtcnt: u8,
    pub wrcsr: u8,
    phase: u64,
    /// A reset requested by an overflow (true = power-on reset), taken by the machine.
    #[serde(skip)]
    pub reset: Option<bool>,
}

impl Wdt {
    /// Register values after a power-on reset (WTCSR 0x18, WRCSR 0x1F).
    pub fn power_on() -> Wdt { Wdt { wtcsr: 0x18, wtcnt: 0, wrcsr: 0x1F, phase: 0, reset: None } }

    pub fn handles(a: u32) -> bool { (BASE..BASE + 6).contains(&a) }

    pub fn read(&self, a: u32, n: usize) -> u64 {
        let v = match a - BASE { 0 => self.wtcsr, 2 => self.wtcnt, 4 => self.wrcsr, _ => 0 } as u64;
        if n == 2 { v << 8 } else { v }
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> Result<(), String> {
        if n != 2 { return Err(format!("WDT write {a:08x}/{n}: keyed 16-bit writes only")); }
        let (key, low) = ((v >> 8) as u8, v as u8);
        match (a - BASE, key) {
            (0, 0xA5) => { self.wtcsr = (self.wtcsr & 0x80 & low) | (low & 0x7F); if self.wtcsr & 0x20 == 0 { self.phase = 0; } }
            (2, 0x5A) => self.wtcnt = low,
            (4, 0xA5) if low == 0 => self.wrcsr &= !0x80,
            (4, 0x5A) => self.wrcsr = (self.wrcsr & 0x80) | (low & 0x60) | 0x1F,
            _ => return Err(format!("WDT write {a:08x} = {v:04x}: wrong key")),
        }
        Ok(())
    }

    pub fn tick(&mut self, p0: u64) {
        if self.wtcsr & 0x20 == 0 { return; }
        self.phase += p0;
        let div = DIV[(self.wtcsr & 7) as usize];
        while self.phase >= div {
            self.phase -= div;
            self.wtcnt = self.wtcnt.wrapping_add(1);
            if self.wtcnt == 0 {
                if self.wtcsr & 0x40 != 0 {
                    self.wrcsr |= 0x80;
                    if self.wrcsr & 0x40 != 0 { self.reset = Some(self.wrcsr & 0x20 == 0); self.wtcsr &= !0x20; return; }
                } else { self.wtcsr |= 0x80; }
            }
        }
    }
}
