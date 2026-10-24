//! Emulated OBD-II adapter (the maker's OBD-II accessory) on SCIF6.
//!
//! Link protocol (as the unit speaks it): both directions use
//! frames `A0 A2 L d[0..L] S B0 B3` with S = (L + sum d) & 0xFF. The first payload byte of a reply is its type:
//! 0x10 presence (answer to 0x40; the real adapter's is 10 01 01: hybrid connector, RO-117), 0x05 configuration
//! (answer to 0x32/0x39; 0x30 maker, 0x33 security and 0x37 G sensor are written without an answer, as the real
//! adapter did, and read back with 0x32), 0x06 diagnosis status (0x34), 0x04 car information (0x31, then streamed;
//! 0x42 or 0x35 stops it), 0x07 detection result (0x38), 0x80 error. The adapter is powered by the car all the time
//! and keeps its settings: maker 1, security 0 as the original stored them on the real unit. As the real adapter, it
//! answers 0x31 with error 1 (80 00 01) unless 0x33 and 0x30 were written since the greeting (the original writes
//! both at every start). Its 0x05 as the real one's: 05 00 maker 04 03 security 00 90 G 00 00 00.
//!
//! Car information (type 0x04, 19 bytes): [2] speed km/h, [3..5] fuel rate
//! (x1.035), [5..7] engine speed x4 rpm, [7] throttle 100/255 %, [8] manifold pressure kPa+101 (boost), [9] engine
//! load 100/255 %, [10] battery 1.75 x - 3.25 V... (unused here), [11] fuel pressure /3 kPa, [12..14] fuel used,
//! [14] intake air +40 C, [15] oil +40 C (0xFF = not available), [16] coolant +40 C (shown as エンジン水温), [17..19] spare.
//! Unavailable values are 0xFF (0xFFFF for words, 0 for the percentage bytes).
//!
//! Switching the car off: the real adapter then sends an error frame (type 0x80, code 2 at byte 2) and keeps the unit
//! powered until the unit asks it to cut the power (0x41, then 0x42). `AR_OBD2_OFF_AT=S` sends that error frame at
//! emulated second S (the power is not cut here; the unit's 0x41/0x42 show in `AR_OBD_LOG`). `AR_OBD2_SILENT_AT=S`
//! instead just stops the car information at second S (no error frame).

use std::collections::VecDeque;

const P0_HZ: f64 = 100.0e6 / 3.0;

#[derive(Clone, Debug)]
pub struct CarState {
    pub speed_kmh: f64,
    pub rpm: f64,
    pub coolant_c: f64,
    pub intake_c: f64,
    pub throttle_pct: f64,
    pub load_pct: f64,
    /// Manifold pressure above atmosphere (kPa; negative = vacuum).
    pub boost_kpa: f64,
    pub battery_v: f64,
    /// Follow the GPS speed and derive engine speed from it (simple 5-speed gearbox).
    pub follow_gps: bool,
}

impl Default for CarState {
    fn default() -> Self {
        CarState { speed_kmh: 0.0, rpm: 750.0, coolant_c: 88.0, intake_c: 30.0, throttle_pct: 12.0, load_pct: 25.0, boost_kpa: -40.0,
            battery_v: 14.2, follow_gps: true }
    }
}

impl CarState {
    /// Engine speed for a road speed: idle at standstill, otherwise the gear that keeps 1500-2800 rpm.
    pub fn rpm_for(speed: f64) -> f64 {
        if speed < 1.0 { return 750.0; }
        let ratios = [125.0, 70.0, 47.0, 36.0, 29.0]; // rpm per km/h
        let limits = [20.0, 40.0, 60.0, 80.0, f64::MAX];
        let g = limits.iter().position(|&l| speed < l).unwrap_or(4);
        (speed * ratios[g]).max(900.0)
    }
}

pub struct Obd2 {
    pub car: CarState,
    /// Index into SCIF6 `tx_log` up to which the unit's bytes have been parsed.
    seen: usize,
    frame: Vec<u8>,
    out: VecDeque<(u64, Vec<u8>)>,
    pub maker: u8,
    pub security: u8,
    pub gsensor: u8,
    pub streaming: bool,
    next_info: u64,
    pub commands: Vec<u8>,
    pub log: bool,
    pub frames_in: u64,
    pub frames_out: u64,
    /// `AR_OBD2_OFF_AT`: when the car is switched off (P0 ticks), once.
    pub off_at: Option<u64>,
    /// `AR_OBD2_SILENT_AT`: when the car information stops without an error frame (P0 ticks), once.
    pub silent_at: Option<u64>,
    /// 0x33 / 0x30 written since the last greeting (0x31 needs both).
    sec_set: bool,
    maker_set: bool,
}

fn frame(payload: &[u8]) -> Vec<u8> {
    let l = payload.len() as u8;
    let sum = payload.iter().fold(l, |s, &b| s.wrapping_add(b));
    let mut f = vec![0xA0, 0xA2, l];
    f.extend_from_slice(payload);
    f.extend([sum, 0xB0, 0xB3]);
    f
}

impl Obd2 {
    pub fn new(tx_seen: usize) -> Obd2 {
        let secs = |name: &str| std::env::var(name).ok().and_then(|v| v.parse::<f64>().ok()).map(|s| (s * P0_HZ) as u64);
        Obd2 { car: CarState::default(), seen: tx_seen, frame: Vec::new(), out: VecDeque::new(), maker: 1, security: 0, gsensor: 6, sec_set: false, maker_set: false,
            streaming: false, next_info: 0, commands: Vec::new(), log: std::env::var_os("AR_OBD_LOG").is_some(), frames_in: 0, frames_out: 0,
            off_at: secs("AR_OBD2_OFF_AT"), silent_at: secs("AR_OBD2_SILENT_AT") }
    }

    fn reply(&mut self, now: u64, payload: &[u8]) {
        // The adapter answers within a few milliseconds.
        self.out.push_back((now + (P0_HZ * 0.010) as u64, frame(payload)));
    }

    fn config(&self) -> Vec<u8> { vec![0x05, 0x00, self.maker, 0x04, 0x03, self.security, 0x00, 0x90, self.gsensor, 0x00, 0x00, 0x00] }

    pub fn car_info(&self) -> Vec<u8> {
        let c = &self.car;
        let b = |v: f64| v.round().clamp(0.0, 254.0) as u8;
        let pct = |v: f64| (v * 255.0 / 100.0).round().clamp(1.0, 255.0) as u8;
        let rpm = (c.rpm * 4.0).round().clamp(0.0, 65534.0) as u16;
        let mut p = vec![0u8; 19];
        p[0] = 0x04;
        p[1] = 0x01;
        p[2] = b(c.speed_kmh);
        p[3] = 0xFF; p[4] = 0xFF;
        p[5] = (rpm >> 8) as u8; p[6] = rpm as u8;
        p[7] = pct(c.throttle_pct);
        p[8] = b(c.boost_kpa + 101.0);
        p[9] = pct(c.load_pct);
        p[10] = ((c.battery_v + 3.25) / 1.75).round().clamp(1.0, 255.0) as u8;
        p[11] = 0xFF;
        p[12] = 0xFF; p[13] = 0xFF;
        p[14] = b(c.intake_c + 40.0);
        p[15] = 0xFF;
        p[16] = b(c.coolant_c + 40.0);
        p[17] = 0xFF; p[18] = 0xFF;
        p
    }

    fn command(&mut self, now: u64, d: &[u8]) {
        let Some(&cmd) = d.first() else { return };
        self.frames_in += 1;
        if self.commands.len() < 4096 { self.commands.push(cmd); }
        if self.log { eprintln!("[obd2] {:.3}s unit -> adapter {:02x?}", now as f64 / P0_HZ, d); }
        match cmd {
            0x40 => { self.sec_set = false; self.maker_set = false; self.reply(now, &[0x10, 0x01, 0x01]) }
            0x42 => { self.streaming = false; }
            0x30 => { self.maker = d.get(1).copied().unwrap_or(0); self.maker_set = true; }
            0x33 => { self.security = d.get(1).copied().unwrap_or(0); self.sec_set = true; }
            0x37 => { self.gsensor = d.get(1).copied().unwrap_or(0); }
            0x32 | 0x39 => { let c = self.config(); self.reply(now, &c); }
            0x34 => self.reply(now, &[0x06, 0x00, 0x00, 0x00]),
            0x31 if !(self.sec_set && self.maker_set) => self.reply(now, &[0x80, 0x00, 0x01]),
            0x31 => { self.streaming = true; self.next_info = now + (P0_HZ * 0.010) as u64; }
            0x38 => { let mut p = vec![0u8; 0x24]; p[0] = 0x07; self.reply(now, &p); }
            0x35 => { self.streaming = false; }
            _ => {}
        }
    }

    /// Exchange bytes with SCIF6: parse what the unit transmitted, deliver due replies and the car-information stream.
    pub fn advance(&mut self, now: u64, scif: &mut crate::periph::scif::Scif) {
        if scif.tx_log.len() > self.seen {
            let new: Vec<u8> = scif.tx_log[self.seen..].to_vec();
            self.seen = scif.tx_log.len();
            for b in new {
                if self.frame.is_empty() && b != 0xA0 { continue; }
                if self.frame.len() == 1 && b != 0xA2 { self.frame.clear(); if b == 0xA0 { self.frame.push(b); } continue; }
                self.frame.push(b);
                if self.frame.len() >= 3 {
                    let l = self.frame[2] as usize;
                    if self.frame.len() == l + 6 {
                        let f = std::mem::take(&mut self.frame);
                        let sum = f[3..3 + l].iter().fold(f[2], |s, &x| s.wrapping_add(x));
                        if sum == f[3 + l] { self.command(now, &f[3..3 + l]); } else if self.log { eprintln!("[obd2] bad checksum {f:02x?}"); }
                    }
                }
            }
        }
        if self.off_at.is_some_and(|t| now >= t) {
            self.off_at = None;
            self.streaming = false;
            if self.log { eprintln!("[obd2] car switched off: error frame, code 2"); }
            self.out.push_back((now, frame(&[0x80, 0x00, 0x02, 0x00])));
        }
        if self.silent_at.is_some_and(|t| now >= t) {
            self.silent_at = None;
            self.streaming = false;
            if self.log { eprintln!("[obd2] car switched off: car information stops (no error frame)"); }
        }
        if self.streaming && now >= self.next_info {
            self.next_info = now + (P0_HZ * 0.200) as u64;
            if self.car.follow_gps { self.car.rpm = CarState::rpm_for(self.car.speed_kmh); }
            let p = self.car_info();
            self.out.push_back((now, frame(&p)));
        }
        while self.out.front().is_some_and(|(t, _)| *t <= now) {
            let (_, f) = self.out.pop_front().unwrap();
            if self.log && f.get(3) != Some(&0x04) { eprintln!("[obd2] {:.3}s adapter -> unit {:02x?}", now as f64 / P0_HZ, f); }
            self.frames_out += 1;
            scif.queue_rx(&f);
        }
    }
}
