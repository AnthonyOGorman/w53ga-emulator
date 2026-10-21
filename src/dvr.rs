//! Emulated drive recorder (the maker's dash cam accessory) on SCIF1: its control link.
//!
//! The unit polls with `$CSEVT,<seq hex>,<state>,<key>*CS` (state 'N' normal, 'C' while it shows the recorder's
//! menu); the recorder echoes the sequence number with its own state ('N' normal, 'C' menu/communication mode,
//! 'E' event recording). After the first answered poll the unit asks `$CSSYS,...` and expects
//! `$CSSYS,<version>,<model>`; versions above 20 enable event-recording control. The unit also forwards its GPS fix
//! as `$GPRMC`/`$GPGGA`, which the recorder only logs. Sentences end in `*hh\r\n` with the NMEA XOR checksum.
//!
//! The camera picture is a separate NTSC composite signal into the SH7269 video decoder (see `Board::camera`).

use std::collections::VecDeque;

const P0_HZ: f64 = 100.0e6 / 3.0;

pub struct Dvr {
    seen: usize,
    line: Vec<u8>,
    out: VecDeque<(u64, Vec<u8>)>,
    /// State character in the recorder's replies.
    pub state: u8,
    pub version: u32,
    pub model: String,
    pub polls: u64,
    pub keys: Vec<String>,
    pub gps_sentences: u64,
    pub log: bool,
}

fn sentence(body: &str) -> Vec<u8> {
    let cs = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${body}*{cs:02X}\r\n").into_bytes()
}

impl Dvr {
    pub fn new(tx_seen: usize) -> Dvr {
        Dvr { seen: tx_seen, line: Vec::new(), out: VecDeque::new(), state: b'N', version: 30, model: "EMU-DVR".into(), polls: 0,
            keys: Vec::new(), gps_sentences: 0, log: std::env::var_os("AR_DVR_LOG").is_some() }
    }

    fn reply(&mut self, now: u64, s: Vec<u8>) { self.out.push_back((now + (P0_HZ * 0.020) as u64, s)); }

    fn on_line(&mut self, now: u64, line: &str) {
        if self.log { eprintln!("[dvr] unit -> recorder {line}"); }
        let body = line.trim_start_matches('$').split('*').next().unwrap_or("");
        let f: Vec<&str> = body.split(',').collect();
        match f.first().copied() {
            Some("CSEVT") => {
                self.polls += 1;
                let seq = f.get(1).copied().unwrap_or("00");
                if let Some(k) = f.get(3) { if !k.is_empty() && self.keys.len() < 1000 { self.keys.push(k.to_string()); } }
                let s = sentence(&format!("CSEVT,{seq},{},", self.state as char));
                self.reply(now, s);
            }
            Some("CSSYS") => { let s = sentence(&format!("CSSYS,{},{}", self.version, self.model)); self.reply(now, s); }
            Some(x) if x.starts_with("GP") || x.starts_with("GN") => self.gps_sentences += 1,
            _ => {}
        }
    }

    /// Exchange bytes with SCIF1.
    pub fn advance(&mut self, now: u64, scif: &mut crate::periph::scif::Scif) {
        if scif.tx_log.len() > self.seen {
            let new: Vec<u8> = scif.tx_log[self.seen..].to_vec();
            self.seen = scif.tx_log.len();
            for b in new {
                if b == b'$' { self.line.clear(); }
                self.line.push(b);
                if b == b'\r' || b == b'\n' || self.line.len() > 200 {
                    let l = String::from_utf8_lossy(&self.line).trim().to_string();
                    self.line.clear();
                    if l.starts_with('$') { self.on_line(now, &l); }
                }
            }
        }
        while self.out.front().is_some_and(|(t, _)| *t <= now) {
            let (_, s) = self.out.pop_front().unwrap();
            if self.log { eprintln!("[dvr] recorder -> unit {}", String::from_utf8_lossy(&s).trim()); }
            scif.queue_rx(&s);
        }
    }
}
