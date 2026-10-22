//! RTC (0xFFFE_6000): BCD calendar, R64CNT, clocked from EXTAL
//! (P0 x 2/5) through the RFRH/RFRL divider. Alarm/periodic/carry interrupts are not modelled (writes enabling
//! them are errors).

use serde::Deserialize;
use std::collections::BTreeMap;

pub const BASE: u32 = 0xFFFE_6000;

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawRtc { values: BTreeMap<String, u32>, known: Vec<u32>, p0_ticks: u64, extal_remainder: u64, divider_phase: u64, pulses128: u64, seconds: u64 }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Rtc {
    /// Register values by offset (0..0x2E); `present` marks implemented offsets, `known` initialized ones.
    #[serde(with = "serde_big_array::BigArray")]
    values: [u32; 0x30],
    #[serde(with = "serde_big_array::BigArray")]
    present: [bool; 0x30],
    #[serde(with = "serde_big_array::BigArray")]
    known: [bool; 0x30],
    pub p0_ticks: u64, extal_remainder: u64, divider_phase: u64, pub pulses128: u64, pub seconds: u64,
}

type R<T> = Result<T, String>;

fn decimal(mut b: u32) -> R<u32> { let (mut n, mut m) = (0, 1); while b != 0 { let d = b & 15; if d > 9 { return Err("RTC invalid BCD".into()); } n += d * m; m *= 10; b >>= 4; } Ok(n) }
fn bcd(mut n: u32) -> u32 { let (mut r, mut s) = (0, 0); loop { r |= (n % 10) << s; n /= 10; s += 4; if n == 0 { break; } } r }
fn month_days(y: u32, m: u32) -> u32 { if m == 2 { if y % 4 == 0 && (y % 100 != 0 || y % 400 == 0) { 29 } else { 28 } } else { [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31][m as usize] } }

impl Rtc {
    pub fn from_raw(x: RawRtc) -> R<Rtc> {
        let (mut values, mut present, mut known) = ([0u32; 0x30], [false; 0x30], [false; 0x30]);
        for (k, v) in x.values { let o = k.parse::<usize>().map_err(|e| e.to_string())?; values[o] = v; present[o] = true; }
        for o in x.known { known[o as usize] = true; }
        Ok(Rtc { values, present, known, p0_ticks: x.p0_ticks, extal_remainder: x.extal_remainder, divider_phase: x.divider_phase, pulses128: x.pulses128, seconds: x.seconds })
    }
    #[inline] fn v(&self, a: u32) -> u32 { self.values[a as usize] }
    #[inline] fn put(&mut self, a: u32, v: u32) { self.values[a as usize] = v; self.known[a as usize] = true; }
    fn wide(a: u32) -> bool { matches!(a, 0xE | 0x20 | 0x2A | 0x2C) }
    pub fn handles(&self, a: u32, n: usize) -> bool { let x = a.wrapping_sub(BASE); (x as usize) < 0x30 && self.present[x as usize] && n == if Self::wide(x) { 2 } else { 1 } }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        if !self.handles(a, n) { return Err(format!("RTC register/width {a:08x}/{n}")); }
        let x = a - BASE;
        if !self.known[x as usize] { return Err(format!("RTC uninitialized field +{x:x}")); }
        Ok(self.v(x) as u64)
    }

    pub fn write(&mut self, a: u32, n: usize, value: u64) -> R<()> {
        if !self.handles(a, n) { return Err(format!("RTC register/width {a:08x}/{n}")); }
        let (x, value) = (a - BASE, value as u32);
        match x {
            0 => return Err("R64CNT is read-only".into()),
            0x1C => { if value & 0x18 != 0 { return Err("RTC carry/alarm interrupt not modelled".into()); } let cur = self.v(x); self.put(x, (value & 0x80) | (cur & value & 1)); }
            0x1E => {
                if value & 0xF0 != 0 { return Err("RTC periodic interrupt not modelled".into()); }
                if value & 4 != 0 { return Err("RTC 30-second adjustment not modelled".into()); }
                if value & 2 != 0 { self.divider_phase = 0; self.put(0, 0); self.put(0x1C, 0); for z in [0x10, 0x12, 0x14, 0x16, 0x18, 0x1A, 0x20, 0x24] { self.put(z, 0); } }
                self.put(x, value & 9);
            }
            0x24 => { if value & 0x80 != 0 { return Err("RTC year alarm not modelled".into()); } self.put(x, 0); }
            0x26 => {
                if value & 3 > 1 { return Err("RTC prohibited clock source".into()); }
                if self.v(0x1E) & 1 != 0 && value & 3 != self.v(x) & 3 { return Err("RTC source switch while running".into()); }
                self.put(x, value & 3);
            }
            0x2A => self.put(x, value & 0x8007),
            0x2C => self.put(x, value),
            0x10..=0x20 => { if value & 0x80 != 0 && x != 0x20 { return Err("RTC enabled alarm not modelled".into()); } self.put(x, value); }
            _ => {
                if self.v(0x1E) & 1 != 0 { return Err("RTC calendar write requires START=0".into()); }
                let mask = match x { 6 => 0x3F, 8 => 7, 0xA => 0x3F, 0xC => 0x1F, 0xE => 0xFFFF, _ => 0x7F };
                let next = value & mask;
                let d = decimal(next)?;
                let ok = match x { 8 => next <= 6, 2 | 4 => d <= 59, 6 => d <= 23, 0xA => (1..=31).contains(&d), 0xC => (1..=12).contains(&d), _ => d <= 9999 };
                if !ok { return Err("RTC calendar field out of range".into()); }
                self.put(x, next);
            }
        }
        Ok(())
    }

    fn second(&mut self) -> R<()> {
        let (mut s, mut mi, mut h, mut d, mut m, mut y) = (decimal(self.v(2))?, decimal(self.v(4))?, decimal(self.v(6))?, decimal(self.v(0xA))?, decimal(self.v(0xC))?, decimal(self.v(0xE))?);
        if d > month_days(y, m) { return Err("RTC impossible date".into()); }
        s += 1;
        if s == 60 { s = 0; mi += 1; if mi == 60 { mi = 0; h += 1; if h == 24 { h = 0; let wd = (self.v(8) + 1) % 7; self.put(8, wd); d += 1; if d > month_days(y, m) { d = 1; m += 1; if m == 13 { m = 1; y = (y + 1) % 10000; } } } } }
        self.put(2, bcd(s)); self.put(4, bcd(mi)); self.put(6, bcd(h)); self.put(0xA, bcd(d)); self.put(0xC, bcd(m)); self.put(0xE, bcd(y));
        self.seconds += 1;
        Ok(())
    }

    pub fn tick_p0(&mut self, ticks: u64) -> R<()> {
        self.p0_ticks += ticks;
        if self.v(0x26) == 0 { return Ok(()); }
        // EXTAL = P0 x 2/5: count without dividing on the per-step path.
        self.extal_remainder += ticks * 2;
        let extal = self.extal_remainder / 5;
        self.extal_remainder %= 5;
        if extal == 0 { return Ok(()); }
        let divisor = (((self.v(0x2A) & 7) as u64) << 16) | self.v(0x2C) as u64;
        if divisor == 0 { return Err("RTC zero divider".into()); }
        self.divider_phase += extal * if self.v(0x2A) & 0x8000 != 0 { 2 } else { 1 };
        let pulses = self.divider_phase / divisor;
        self.divider_phase %= divisor;
        self.pulses128 += pulses;
        let total = self.v(0) as u64 + pulses;
        self.put(0, (total & 127) as u32);
        if self.v(0x1E) & 1 != 0 { for _ in 0..total / 128 { self.second()?; } }
        Ok(())
    }

    /// Calendar as (year, month, day, hour, minute, second) for the front end.
    pub fn calendar(&self) -> Option<(u32, u32, u32, u32, u32, u32)> {
        Some((decimal(self.v(0xE)).ok()?, decimal(self.v(0xC)).ok()?, decimal(self.v(0xA)).ok()?, decimal(self.v(6)).ok()?, decimal(self.v(4)).ok()?, decimal(self.v(2)).ok()?))
    }
}
