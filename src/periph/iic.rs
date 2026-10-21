//! IIC2 (SH7269 RIIC channel 2, 0xFFFE_E800) byte engine,
//! with the inertial sensor on the bus.
//!
//! Controller timing is functional (P0 ticks): START 84, transmit-buffer handoff 1, byte 756, STOP 84.
//! The sensor is an MPU-6500-compatible register file at 7-bit address 0x69 (wire 0xD2/0xD3): writes set the
//! register pointer then auto-increment data; reads auto-increment. Other addresses (the absent pressure sensors
//! the firmware probes) NACK. Initial values (fixture): WHO_AM_I (0x75) = 0x70, register 0x00
//! = 0, and a stationary level sample (+1 g on Z, 21 C, no rotation) in 0x3B..0x48; tests/UI can change them.

use serde::Deserialize;
use std::collections::BTreeMap;

pub const BASE: u32 = 0xFFFE_E800;
pub const IIC1_BASE: u32 = 0xFFFE_E400;
const START_P0: u64 = 84;
const HANDOFF_P0: u64 = 1;
const BYTE_P0: u64 = 756;
const STOP_P0: u64 = 84;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum Pending { None, Start, Load, Tx, Rx, RxMulti, Stop }

impl Pending {
    fn parse(s: &str) -> Result<Pending, String> {
        Ok(match s { "none" => Pending::None, "start" => Pending::Start, "load" => Pending::Load, "tx" => Pending::Tx, "rx" => Pending::Rx,
            "rx-multi" => Pending::RxMulti, "stop" => Pending::Stop, x => return Err(format!("IIC pending {x}")) })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum Phase { Idle, Pointer, Data, Read, Nack }

/// Register-file I2C device (MPU-6500 compatible inertial sensor).
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Sensor {
    #[serde(with = "serde_big_array::BigArray")]
    pub regs: [u8; 128],
    phase: Phase,
    ptr: u8,
    pub address7: u8,
}

impl Sensor {
    /// Plain register-file device (e.g. the audio DAC at 0x12: write-only from the firmware's point of view).
    pub fn register_file(address7: u8) -> Sensor { Sensor { regs: [0; 128], phase: Phase::Idle, ptr: 0, address7 } }

    pub fn stationary() -> Sensor {
        let mut regs = [0u8; 128];
        regs[0x75] = 0x70;
        let sample = [0x00u8, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]; // AX AY AZ T GX GY GZ
        regs[0x3B..0x49].copy_from_slice(&sample);
        Sensor { regs, phase: Phase::Idle, ptr: 0, address7: 0x69 }
    }
    fn address(&mut self, wire: u8, repeated: bool) -> bool {
        if wire >> 1 != self.address7 { return false; }
        if wire & 1 == 0 { self.phase = Phase::Pointer; } else { self.phase = Phase::Read; }
        let _ = repeated;
        true
    }
    fn write_byte(&mut self, v: u8) -> Result<bool, String> {
        match self.phase {
            Phase::Pointer => { self.ptr = v & 0x7F; self.phase = Phase::Data; }
            Phase::Data => { self.regs[self.ptr as usize] = v; self.ptr = (self.ptr + 1) & 0x7F; }
            p => return Err(format!("sensor write in phase {p:?}")),
        }
        Ok(true)
    }
    fn read_byte(&mut self) -> Result<u8, String> {
        if self.phase != Phase::Read { return Err(format!("sensor read in phase {:?}", self.phase)); }
        let v = self.regs[self.ptr as usize];
        self.ptr = (self.ptr + 1) & 0x7F;
        Ok(v)
    }
    fn stop(&mut self) { self.phase = Phase::Idle; }
}

fn minus_one_i64() -> i64 { -1 }

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawPeer { phase: String, #[serde(default = "minus_one_i64")] pending_register: i64, #[serde(default)] configured: BTreeMap<String, i64>, #[serde(default)] stationary_cursor: i64, #[serde(default)] stationary_prefix: i64 }

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawIic {
    r: [u32; 9], flags: u32, seen_flags: u32, ackbr: u32, buffer: i32, shift: i32, busy: bool, scl_high: bool, sda_released: bool,
    next_address: bool, repeated: bool, addressed: bool, read_direction: bool, rx_started: bool, rx_available: bool,
    #[serde(default)] multi_rx_final_nack: bool, #[serde(default)] multi_rx_transaction: bool, #[serde(default)] multi_rx_completed: u32,
    #[serde(default = "minus_one")] multi_rx_launched_ack: i32,
    pending: String, now: u64, due: u64, reads: u64, writes: u64, starts: u64, stops: u64, transmitted: u64, received: u64,
    #[serde(default)] peer: Option<RawPeer>, #[serde(default)] multibyte: bool,
}
fn minus_one() -> i32 { -1 }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Iic2 {
    r: [u32; 9], flags: u32, seen_flags: u32, ackbr: u32, buffer: i32, shift: i32, busy: bool, scl_high: bool, sda_released: bool,
    next_address: bool, repeated: bool, addressed: bool, read_direction: bool, rx_started: bool, rx_available: bool,
    multi_final_nack: bool, multi_transaction: bool, multi_completed: u32, multi_launched_ack: i32,
    pending: Pending, now: u64, due: u64,
    pub reads: u64, pub writes: u64, pub starts: u64, pub stops: u64, pub transmitted: u64, pub received: u64,
    multibyte: bool,
    pub base: u32,
    /// Devices on the bus; `cur` is the one addressed in the current transfer.
    pub devices: Vec<Sensor>,
    cur: Option<usize>,
}

type R<T> = Result<T, String>;

impl Iic2 {
    /// IIC2 (inertial sensor bus) from a start state.
    pub fn from_raw(x: RawIic) -> R<Iic2> { Self::from_raw_with(x, BASE, Sensor::stationary()) }

    /// IIC1 (audio DAC bus at 7-bit 0x12) from a start state.
    pub fn iic1_from_raw(x: RawIic) -> R<Iic2> { Self::from_raw_with(x, IIC1_BASE, Sensor::register_file(0x12)) }

    /// IIC2 (inertial sensor) at power-on.
    pub fn iic2_reset() -> Iic2 {
        let x: RawIic = serde_json::from_value(serde_json::json!({
            "r": [0, 0x20, 0, 0, 0, 0, 0xff, 0xff, 0], "flags": 0, "seenFlags": 0, "ackbr": 0, "buffer": -1, "shift": -1, "busy": false,
            "sclHigh": true, "sdaReleased": true, "nextAddress": false, "repeated": false, "addressed": false, "readDirection": false,
            "rxStarted": false, "rxAvailable": false, "pending": "none", "now": 0, "due": 0, "reads": 0, "writes": 0, "starts": 0, "stops": 0,
            "transmitted": 0, "received": 0, "multibyte": true })).expect("IIC2 reset state");
        Self::from_raw_with(x, BASE, Sensor::stationary()).expect("IIC2 reset")
    }

    /// IIC1 at power-on (initial fields: released lines, idle controller).
    pub fn iic1_reset() -> Iic2 {
        let x: RawIic = serde_json::from_value(serde_json::json!({
            "r": [0, 0x20, 0, 0, 0, 0, 0xff, 0xff, 0], "flags": 0, "seenFlags": 0, "ackbr": 0, "buffer": -1, "shift": -1, "busy": false,
            "sclHigh": true, "sdaReleased": true, "nextAddress": false, "repeated": false, "addressed": false, "readDirection": false,
            "rxStarted": false, "rxAvailable": false, "pending": "none", "now": 0, "due": 0, "reads": 0, "writes": 0, "starts": 0, "stops": 0,
            "transmitted": 0, "received": 0 })).expect("IIC1 reset state");
        Self::from_raw_with(x, IIC1_BASE, Sensor::register_file(0x12)).expect("IIC1 reset")
    }

    fn from_raw_with(x: RawIic, base: u32, mut sensor: Sensor) -> R<Iic2> {
        if let Some(p) = &x.peer {
            for (k, v) in &p.configured { sensor.regs[k.parse::<usize>().map_err(|e| e.to_string())? & 0x7F] = *v as u8; }
            sensor.phase = match p.phase.as_str() {
                "idle" => Phase::Idle, "write-register" => Phase::Pointer, "write-value" | "write-complete" | "prefix-complete" => Phase::Data,
                "read-ready" | "read-complete" => Phase::Read, "address-NACK" => Phase::Nack, ph => return Err(format!("sensor phase {ph}")),
            };
            if p.stationary_cursor >= 0 && p.stationary_prefix >= 0 { sensor.ptr = (0x3B + p.stationary_cursor) as u8; }
            else if p.pending_register >= 0 { sensor.ptr = p.pending_register as u8; }
        }
        let cur = if sensor.phase != Phase::Idle && sensor.phase != Phase::Nack { Some(0) } else { None };
        Ok(Iic2 {
            r: x.r, flags: x.flags, seen_flags: x.seen_flags, ackbr: x.ackbr, buffer: x.buffer, shift: x.shift, busy: x.busy, scl_high: x.scl_high,
            sda_released: x.sda_released, next_address: x.next_address, repeated: x.repeated, addressed: x.addressed, read_direction: x.read_direction,
            rx_started: x.rx_started, rx_available: x.rx_available, multi_final_nack: x.multi_rx_final_nack, multi_transaction: x.multi_rx_transaction,
            multi_completed: x.multi_rx_completed, multi_launched_ack: x.multi_rx_launched_ack, pending: Pending::parse(&x.pending)?, now: x.now, due: x.due,
            reads: x.reads, writes: x.writes, starts: x.starts, stops: x.stops, transmitted: x.transmitted, received: x.received,
            multibyte: x.multibyte, base, devices: vec![sensor], cur,
        })
    }

    pub fn handles(&self, a: u32, n: usize) -> bool { n > 0 && a < self.base + 0x400 && a + n as u32 > self.base }

    fn offset(&self, a: u32, n: usize) -> R<usize> {
        if n != 1 { return Err("IIC registers require byte access".into()); }
        let o = a.wrapping_sub(self.base);
        if o > 8 { return Err(format!("unimplemented IIC2 register {a:08x}")); }
        Ok(o as usize)
    }
    fn clear_flags(&mut self, m: u32) { self.flags &= !m; self.seen_flags &= !m; }
    fn transmitting(&self) -> bool { self.r[0] & 0xB0 == 0xB0 }
    fn receiving(&self) -> bool { self.r[0] & 0xB0 == 0xA0 }
    fn timing(&self) -> R<()> {
        if self.r[0] & 15 != 4 || self.r[8] != 0 || self.r[5] & 1 != 0 || self.r[2] & 0x80 != 0 { return Err("byte engine requires original I2C/MSB/CKS4/NF0 configuration".into()); }
        Ok(())
    }
    fn schedule(&mut self, p: Pending, delay: u64) { self.pending = p; self.due = self.now + delay; }
    /// The model only starts multi-byte receives while a stationary-bank prefix is active (register 0x3B..0x48).
    fn multibyte_prefix_active(&self) -> bool {
        self.multibyte && self.cur.is_some_and(|i| { let d = &self.devices[i]; d.address7 == 0x69 && (0x3B..0x49).contains(&d.ptr) && d.phase == Phase::Read })
    }
    /// Inertial sensor register file (for front-end tilt/motion input), if on this bus.
    pub fn sensor(&mut self) -> Option<&mut Sensor> { self.devices.iter_mut().find(|d| d.address7 == 0x69) }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        let o = self.offset(a, n)?;
        let value = match o {
            1 => {
                // While a start, byte or stop condition is in progress the bus is busy (BBSY = 1 until the stop
                // condition completes). The SDA/SCL monitor bits are mid-transition then; they read as the last
                // settled levels. Software polls BBSY here before starting a transfer.
                let busy = self.busy || self.pending != Pending::None;
                0x55 | (self.r[1] & 2) | if busy { 0x80 } else { 0 } | if self.sda_released { 0x20 } else { 0 } | if self.scl_high { 8 } else { 0 }
            }
            2 => {
                if matches!(self.pending, Pending::Tx | Pending::Rx | Pending::RxMulti) { return Err("mid-frame bit count unknown".into()); }
                (self.r[2] & 0x87) | 0x38
            }
            3 => (self.r[3] & 0xFD) | self.ackbr,
            4 => self.flags,
            7 => self.read_icdrr()?,
            _ => self.r[o],
        };
        if o == 4 { self.seen_flags |= self.flags; }
        self.reads += 1;
        Ok(value as u64)
    }

    fn read_icdrr(&mut self) -> R<u32> {
        if self.rx_available {
            if self.multi_transaction {
                if self.flags & 0x20 == 0 { return Err("ICDRR occupancy without RDRF".into()); }
                if self.multi_final_nack {
                    if self.busy || self.pending != Pending::None { return Err("final NACK byte drained before STOP".into()); }
                } else {
                    if !self.busy || self.pending != Pending::None || self.multi_launched_ack != -1 { return Err("next receive launch requires idle active bus".into()); }
                    if self.multi_completed >= 14 { return Err("multibyte read exceeds bank".into()); }
                    if self.r[3] & 1 != 0 && self.multi_completed < 1 { return Err("first received byte must be ACKed".into()); }
                }
                let v = self.r[7];
                self.rx_available = false;
                self.clear_flags(0x20);
                if self.multi_final_nack { self.multi_transaction = false; }
                else { self.multi_launched_ack = (self.r[3] & 1) as i32; self.schedule(Pending::RxMulti, BYTE_P0); }
                Ok(v)
            } else {
                if self.busy || self.pending != Pending::None { return Err("buffered RX drained before STOP".into()); }
                if self.r[0] & 0x40 == 0 { return Err("continuous receive unsupported".into()); }
                let v = self.r[7];
                self.rx_available = false;
                self.clear_flags(0x20);
                Ok(v)
            }
        } else {
            if !self.receiving() || !self.busy || !self.addressed || !self.read_direction || self.rx_started || self.pending != Pending::None {
                return Err("ICDRR read is neither first dummy nor buffered drain".into());
            }
            self.timing()?;
            let v = self.r[7];
            if self.multibyte_prefix_active() {
                if self.r[0] & 0x40 == 0 { return Err("RCVD0 receive mode unsupported".into()); }
                if self.r[3] & 1 != 0 { return Err("first multibyte receive must begin with ACKBT=0".into()); }
                self.rx_started = true; self.multi_transaction = true; self.multi_completed = 0; self.multi_final_nack = false; self.multi_launched_ack = 0;
                self.schedule(Pending::RxMulti, BYTE_P0);
            } else {
                if self.r[0] & 0x40 == 0 || self.r[3] & 1 == 0 { return Err("single-byte dummy requires RCVD1 and ACKBT1".into()); }
                self.rx_started = true;
                self.clear_flags(0x20);
                self.schedule(Pending::Rx, BYTE_P0);
            }
            Ok(v)
        }
    }

    pub fn write(&mut self, a: u32, n: usize, data: u64) -> R<()> {
        let o = self.offset(a, n)?;
        let v = (data & 0xFF) as u32;
        match o {
            7 => return Err("ICDRR is receive-only".into()),
            0 => {
                if self.pending != Pending::None && ((v ^ self.r[0]) & 0xBF) != 0 { return Err("mode change during pending event".into()); }
                if self.busy && v & 0x80 == 0 { return Err("disabling active bus".into()); }
                if v & 0x10 != 0 && self.r[0] & 0x10 == 0 { self.flags |= 0x80; }
                self.r[0] = v;
            }
            1 => {
                if v & 5 != 5 { return Err("ICCR2 reserved bits".into()); }
                let rst = v & 2 != 0;
                if rst && (self.busy || self.pending != Pending::None) { return Err("active-bus reset".into()); }
                if v & 0x10 == 0 && v & 0x20 == 0 { return Err("manual SDA drive unsupported".into()); }
                if v & 0x40 == 0 && self.pending == Pending::Stop {
                    // A bus recovery routine writes ICCR2 three times in a row (BBSY/SCP cleared, then SCP, then both); the
                    // later writes arrive while its STOP condition is still being generated and the bus ends idle.
                    if std::env::var_os("AR_IIC_LOG").is_some() { eprintln!("[iic {:08x}] bus command {v:02x} during STOP: ignored", self.base); }
                } else if v & 0x40 == 0 {
                    self.timing()?;
                    if rst || self.r[1] & 2 != 0 || (!self.transmitting() && v & 0x80 != 0) { return Err("START requires released reset and master transmitter".into()); }
                    if self.pending != Pending::None || self.buffer != -1 || self.shift != -1 { return Err("bus command during pending byte event".into()); }
                    if v & 0x80 != 0 {
                        if self.rx_available { return Err("restart with unread RX buffer".into()); }
                        self.repeated = self.busy; self.next_address = true; self.addressed = false; self.read_direction = false; self.rx_started = false; self.ackbr = 0;
                        self.schedule(Pending::Start, START_P0);
                    } else {
                        if !self.busy || self.r[0] & 0xA0 != 0xA0 { return Err("STOP requires enabled master and active transfer".into()); }
                        if self.multi_transaction && (!self.multi_final_nack || !self.rx_available || self.pending != Pending::None) {
                            // Software error recovery (after a bus-busy or byte timeout) switches
                            // to master transmit and issues STOP in the middle of a multi-byte read: the read is
                            // abandoned and the bus freed.
                            if self.pending != Pending::None { return Err("multibyte STOP during a byte transfer".into()); }
                            if std::env::var_os("AR_IIC_LOG").is_some() { eprintln!("[iic {:08x}] STOP aborts multi-byte read after {} bytes", self.base, self.multi_completed); }
                            self.multi_transaction = false; self.multi_final_nack = false; self.multi_launched_ack = -1;
                            self.rx_available = false; self.clear_flags(0x20);
                        }
                        self.schedule(Pending::Stop, STOP_P0);
                    }
                }
                if rst { self.r[2] &= 0x80; self.scl_high = true; self.sda_released = true; }
                self.r[1] = (self.r[1] & 0x20) | (v & 2);
            }
            2 => {
                if v & 0x40 != 0 || v & 0x30 != 0x30 { return Err("ICMR fixed bits".into()); }
                if v & 0x80 != 0 { return Err("LSB-first unsupported".into()); }
                if v & 8 == 0 && self.pending != Pending::None { return Err("BC programming during transfer".into()); }
                if v & 8 == 0 && v & 7 != 0 && self.scl_high { return Err("nonzero BC requires SCL low".into()); }
                self.r[2] = (v & 0x80) | if v & 8 == 0 { v & 7 } else { self.r[2] & 7 };
            }
            3 => {
                if v & 0xFC != 0 { return Err("IIC IRQ enables outside polling contract".into()); }
                if self.multibyte && self.pending == Pending::RxMulti && (v & 1) as i32 != self.multi_launched_ack { return Err("ACKBT change after launch".into()); }
                self.r[3] = v & 0xFD;
            }
            4 => { let clear = self.seen_flags & !v; self.clear_flags(clear); self.seen_flags = 0; }
            5 => { if v & 1 != 0 { return Err("clocked-synchronous format unsupported".into()); } self.r[5] = v; }
            6 => {
                let active = self.transmitting() && self.r[1] & 2 == 0;
                if active {
                    self.timing()?;
                    if !self.busy && self.pending != Pending::Start { return Err("ICDRT without START".into()); }
                    if self.buffer != -1 || matches!(self.pending, Pending::Stop | Pending::Rx) { return Err("transmit buffer overflow".into()); }
                }
                self.r[6] = v;
                self.clear_flags(0xC0);
                if active { self.buffer = v as i32; if self.pending == Pending::None { self.schedule(Pending::Load, HANDOFF_P0); } }
            }
            8 => {
                if v & 0xEC != 0 { return Err("NF2CYC reserved bits".into()); }
                if self.busy || self.pending != Pending::None { return Err("filter change during transfer".into()); }
                self.r[8] = v;
            }
            _ => self.r[o] = v,
        }
        self.writes += 1;
        Ok(())
    }

    fn complete(&mut self) -> R<()> {
        match self.pending {
            Pending::Start => {
                self.busy = true; self.scl_high = false; self.sda_released = true; self.starts += 1; self.flags |= 0x80;
                if self.buffer != -1 { self.clear_flags(0x80); self.schedule(Pending::Load, HANDOFF_P0); } else { self.pending = Pending::None; self.due = 0; }
            }
            Pending::Load => {
                if self.buffer < 0 { return Err("missing ICDRT handoff byte".into()); }
                self.shift = self.buffer; self.buffer = -1; self.flags |= 0x80;
                self.schedule(Pending::Tx, BYTE_P0);
            }
            Pending::Tx => {
                let value = self.shift as u8;
                let ack;
                if self.next_address {
                    if !self.repeated { self.cur = None; }
                    let found = self.devices.iter().position(|d| d.address7 == value >> 1);
                    ack = match found { Some(i) => { self.cur = Some(i); self.devices[i].address(value, self.repeated) } None => { self.cur = None; false } };
                    self.addressed = ack; self.read_direction = value & 1 != 0; self.next_address = false;
                    if ack && self.read_direction && self.multibyte_prefix_active() {
                        self.multi_transaction = false; self.multi_final_nack = false; self.multi_completed = 0; self.multi_launched_ack = -1; self.rx_started = false; self.rx_available = false;
                    }
                } else {
                    if !self.addressed || self.read_direction { return Err("data TX without acknowledged write address".into()); }
                    ack = match self.cur { Some(i) => self.devices[i].write_byte(value)?, None => return Err("data TX without device".into()) };
                }
                self.ackbr = if ack { 0 } else { 2 };
                if std::env::var_os("AR_IIC_LOG").is_some() { eprintln!("[iic {:08x}] {} {value:02x} {}", self.base, if self.next_address || self.cur.is_none() && !self.addressed { "addr" } else { "data" }, if ack { "ACK" } else { "NACK" }); }
                self.transmitted += 1; self.r[2] &= 0x80; self.shift = -1; self.scl_high = false; self.sda_released = true;
                if self.buffer != -1 { self.schedule(Pending::Load, HANDOFF_P0); }
                else { self.pending = Pending::None; self.due = 0; if self.flags & 0x80 != 0 { self.flags |= 0x40; } }
            }
            Pending::Rx => {
                if self.r[3] & 1 == 0 { return Err("single-byte receive requires ACKBT1".into()); }
                self.r[7] = match self.cur { Some(i) => self.devices[i].read_byte()? as u32, None => return Err("receive without device".into()) };
                self.rx_available = true; self.received += 1; self.r[2] &= 0x80; self.flags |= 0x20; self.scl_high = false; self.sda_released = true;
                self.pending = Pending::None; self.due = 0;
            }
            Pending::RxMulti => {
                if !self.multi_transaction || self.multi_launched_ack < 0 || (self.r[3] & 1) as i32 != self.multi_launched_ack { return Err("multibyte ACKBT launch mismatch".into()); }
                if self.rx_available || self.flags & 0x20 != 0 { return Err("ICDRR overwrite before drain".into()); }
                if self.multi_completed >= 14 { return Err("multibyte receive exceeds bank".into()); }
                self.r[7] = match self.cur { Some(i) => self.devices[i].read_byte()? as u32, None => return Err("receive without device".into()) };
                self.rx_available = true; self.received += 1; self.multi_completed += 1;
                self.multi_final_nack = self.multi_launched_ack == 1; self.multi_launched_ack = -1;
                self.r[2] &= 0x80; self.flags |= 0x20; self.scl_high = false; self.sda_released = true;
                self.pending = Pending::None; self.due = 0;
            }
            Pending::Stop => {
                if let Some(i) = self.cur.take() { self.devices[i].stop(); }
                self.busy = false; self.scl_high = true; self.sda_released = true; self.r[2] = (self.r[2] & 0x80) | 7; self.flags |= 8; self.stops += 1;
                self.pending = Pending::None; self.due = 0;
            }
            Pending::None => return Err("no scheduled IIC event".into()),
        }
        Ok(())
    }

    pub fn advance_p0(&mut self, ticks: u64) -> R<()> {
        let target = self.now + ticks;
        let mut bound = 0;
        while self.pending != Pending::None && self.due <= target {
            bound += 1;
            if bound > 8 { return Err("unbounded IIC event chain".into()); }
            self.now = self.due;
            self.complete()?;
        }
        self.now = target;
        Ok(())
    }
}
