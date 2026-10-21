//! The wireless LAN module on SD host 1 (SDIO), emulated at its host interface: the SDIO function registers and CIS the
//! unit's driver uses, and the module's configuration protocol ("WID" messages),
//! recovered from that driver. The module's own firmware is not executed: like the real boot ROM, the emulated module
//! asks the host to download it (embedded in the unit's flash; a 32-byte "WLFW" header with the LE32 image size), checks
//! each chunk's checksum, then reports that it
//! started. Scanning, joining and data then follow the driver's protocol; Ethernet frames go to `usernet` (a user-mode
//! network behind a virtual access point) or anywhere else through the `Link` trait.
//!
//! Transfers (host side, all on SDIO function 1):
//!   CMD53 block write to address 7: a host frame - LE16 header (length in bits 11-0, type in 15-12: 1 data, 4 config)
//!     then the payload; the module toggles bit 4 of its status byte (common CIS + 0xF6, low nibble 2 = ready) per frame.
//!   Interrupt: register 6 reads non-zero while a module frame waits; writing 1 to register 5 acknowledges it and the
//!     frame is then read with a CMD53 block read from address 8 (type 2 data, type 3 config).
//! Config messages: 'Q'/'W' from the host, 'R' (response), 'I' (information) and 'N' (network) from the module:
//!   [type][seq][LE16 length][LE16 WID][length byte (LE16 for WIDs 0x4xxx)][value]...

use std::collections::{BTreeMap, VecDeque};

const P0_HZ: f64 = 100.0e6 / 3.0;
/// Common CIS (fn0 0x1000) and function-1 CIS (0x1100). The driver reads the module status at common CIS + 0xF6.
const CIS0: u32 = 0x1000;
const CIS1: u32 = 0x1100;
/// SDIO manufacturer / card ID (as the module reports in its CIS).
const MANF: u16 = 0x032C;
const CARD: u16 = 0x0120;
const CHUNK: u32 = 1024;

fn log() -> bool { std::env::var_os("AR_WLAN_LOG").is_some() }

/// A virtual access point the module can see and join.
#[derive(Clone, Debug)]
pub struct Ap {
    pub ssid: String,
    pub bssid: [u8; 6],
    pub channel: u8,
    /// Empty = open network, otherwise WPA2-PSK (AES) with this passphrase ("*": any passphrase is accepted, e.g. to
    /// stand in for an access point already registered in the unit whose key is not known).
    pub passphrase: String,
    pub rssi: i8,
}

impl Ap {
    fn security(&self) -> u8 { if self.passphrase.is_empty() { 0 } else { 0x31 } } // 11i mode: enc + WPA2 + AES
}

/// Where the module's Ethernet frames go once it is associated.
pub trait Link: Send {
    fn input(&mut self, now: u64, frame: &[u8]);
    fn output(&mut self, now: u64) -> Option<Vec<u8>>;
    fn associated(&mut self, _mac: [u8; 6]) {}
    fn disassociated(&mut self) {}
}

#[derive(PartialEq, Debug, Clone, Copy)]
enum Boot { Rom, Header, Body { next: u32, total: u32 }, Done, Running }

pub struct Wlan {
    pub mac: [u8; 6],
    pub aps: Vec<Ap>,
    pub link: Option<Box<dyn Link>>,
    cccr: BTreeMap<u32, u8>,
    f1: [u8; 16],
    status_toggle: u8,
    /// Module -> host frames (SDIO header included) and the time they become available.
    out: VecDeque<(u64, Vec<u8>)>,
    /// Interrupt raised (register 6) for the frame at the head of `out`; after the acknowledge it is `staged`.
    signalled: bool,
    staged: Option<Vec<u8>>,
    boot: Boot,
    image: Vec<u8>,
    wids: BTreeMap<u16, Vec<u8>>,
    seq: u8,
    scan_done_at: Option<u64>,
    survey_part: u8,
    join_at: Option<(u64, Option<usize>)>,
    pub connected: Option<usize>,
    pub frames_in: u64,
    pub frames_out: u64,
    now: u64,
}

impl Wlan {
    pub fn new(mac: [u8; 6], aps: Vec<Ap>) -> Wlan {
        let mut w = Wlan {
            mac, aps, link: None, cccr: BTreeMap::new(), f1: [0; 16], status_toggle: 0, out: VecDeque::new(), signalled: false,
            staged: None, boot: Boot::Rom, image: Vec::new(), wids: BTreeMap::new(), seq: 0, scan_done_at: None, survey_part: 0,
            join_at: None, connected: None, frames_in: 0, frames_out: 0, now: 0,
        };
        w.reset();
        w
    }

    /// Card reset (power-up, CCCR I/O abort RES): function registers cleared, back in the boot ROM.
    fn reset(&mut self) {
        self.cccr.clear();
        for (a, v) in [(0x00u32, 0x32u8), (0x01, 0x02), (0x08, 0x13), (0x09, CIS0 as u8), (0x0A, (CIS0 >> 8) as u8), (0x0B, 0), (0x12, 0x01), (0x13, 0x01)] { self.cccr.insert(a, v); }
        for (a, v) in [(0x100u32, 0x07u8), (0x109, CIS1 as u8), (0x10A, (CIS1 >> 8) as u8), (0x10B, 0), (0x110, 0), (0x111, 0)] { self.cccr.insert(a, v); }
        self.f1 = [0; 16];
        self.out.clear(); self.signalled = false; self.staged = None;
        self.boot = Boot::Rom; self.image.clear(); self.wids.clear(); self.scan_done_at = None; self.join_at = None;
        if self.connected.take().is_some() { if let Some(l) = &mut self.link { l.disassociated(); } }
    }

    fn cis(&self, a: u32) -> u8 {
        let manfid = [0x20, 4, MANF as u8, (MANF >> 8) as u8, CARD as u8, (CARD >> 8) as u8];
        let t: Vec<u8> = if a >= CIS1 {
            [&manfid[..], &[0x21, 2, 0x0C, 0x00], &[0xFF]].concat()
        } else {
            [&manfid[..], &[0x21, 2, 0x0C, 0x00], &[0xFF]].concat()
        };
        let base = if a >= CIS1 { CIS1 } else { CIS0 };
        t.get((a - base) as usize).copied().unwrap_or(0xFF)
    }

    /// CMD52 (IO_RW_DIRECT). Returns the register value after the access.
    pub fn cmd52(&mut self, func: u8, addr: u32, write: bool, data: u8) -> u8 {
        match func {
            0 => {
                if addr == CIS0 + 0xF6 { return 2 | (self.status_toggle << 4); }
                if (CIS0..CIS0 + 0x200).contains(&addr) { return self.cis(addr); }
                if write {
                    match addr {
                        0x02 => { self.cccr.insert(0x02, data & 2); self.cccr.insert(0x03, data & 2); }
                        0x06 => if data & 8 != 0 { if log() { eprintln!("[wlan] card reset"); } self.reset(); },
                        0x03 | 0x05 => {}
                        _ => { self.cccr.insert(addr, data); }
                    }
                }
                if addr == 0x05 { return if self.irq_line_raw() { 2 } else { 0 }; }
                self.cccr.get(&addr).copied().unwrap_or(0)
            }
            1 => {
                let r = (addr & 15) as usize;
                if write {
                    if r == 5 && data & 1 != 0 && self.signalled { self.signalled = false; self.staged = self.out.pop_front().map(|x| x.1); }
                    else if r != 5 && r != 6 { self.f1[r] = data; }
                }
                if r == 6 { return self.signalled as u8; }
                self.f1[r]
            }
            _ => 0,
        }
    }

    /// CMD53 read from function 1.
    pub fn read_data(&mut self, func: u8, addr: u32, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        if func == 1 && addr == 8 {
            if let Some(f) = self.staged.take() { let n = f.len().min(len); v[..n].copy_from_slice(&f[..n]); self.frames_out += 1; }
        }
        v
    }

    /// CMD53 write to function 1.
    pub fn write_data(&mut self, func: u8, addr: u32, data: &[u8]) {
        if func != 1 || addr != 7 || data.len() < 2 { return; }
        self.status_toggle ^= 1;
        let len = (u16::from_le_bytes([data[0], data[1]]) & 0x0FFF) as usize;
        let kind = data[1] >> 4;
        let body = &data[2..(2 + len).min(data.len())];
        match kind {
            1 => { self.frames_in += 1; if self.connected.is_some() { let now = self.now; if let Some(l) = &mut self.link { l.input(now, body); } } }
            4 => self.config(body),
            k => if log() { eprintln!("[wlan] host frame type {k} len {len}"); },
        }
    }

    /// The card's interrupt line (function 1 pending, enabled in CCCR IEN with the master enable).
    fn irq_line_raw(&self) -> bool { self.signalled }
    pub fn irq_line(&self) -> bool {
        let ien = self.cccr.get(&0x04).copied().unwrap_or(0);
        self.signalled && ien & 3 == 3
    }

    pub fn advance(&mut self, now: u64) {
        self.now = now;
        if let Some(t) = self.scan_done_at { if now >= t { self.scan_done_at = None; self.wids.insert(0x001E, vec![0]); self.survey_part = 0; } }
        if let Some((t, index)) = self.join_at { if now >= t { self.join_at = None; self.join(index); } }
        if self.connected.is_some() {
            while let Some(f) = self.link.as_mut().and_then(|l| l.output(now)) {
                if self.out.len() > 64 { break; } // the host is not keeping up: the frame is dropped, as on the air
                let mut v = Vec::with_capacity(f.len() + 2);
                v.extend(((f.len() as u16 & 0x0FFF) | 0x2000).to_le_bytes());
                v.extend(f);
                self.out.push_back((now, v));
            }
        }
        if !self.signalled && self.staged.is_none() && self.out.front().is_some_and(|x| x.0 <= now) { self.signalled = true; }
    }

    fn send(&mut self, delay_s: f64, msg: Vec<u8>) {
        let mut v = Vec::with_capacity(msg.len() + 2);
        v.extend(((msg.len() as u16 & 0x0FFF) | 0x3000).to_le_bytes());
        v.extend(msg);
        self.out.push_back((self.now + (delay_s * P0_HZ) as u64, v));
    }

    /// A module message: kind 'R'/'I'/'N', the WID and its value (length encoding by WID type).
    fn message(&mut self, kind: u8, seq: u8, wid: u16, value: &[u8]) -> Vec<u8> {
        let mut m = vec![kind, seq, 0, 0];
        m.extend(wid.to_le_bytes());
        if wid >> 12 == 4 { m.extend((value.len() as u16).to_le_bytes()); } else { m.push(value.len() as u8); }
        m.extend(value);
        let n = m.len() as u16;
        m[2..4].copy_from_slice(&n.to_le_bytes());
        m
    }

    fn info(&mut self, delay_s: f64, wid: u16, value: &[u8]) {
        self.seq = self.seq.wrapping_add(1);
        let m = self.message(b'I', self.seq, wid, value);
        self.send(delay_s, m);
    }

    fn config(&mut self, m: &[u8]) {
        if m.len() < 6 { return; }
        let (kind, seq, wid) = (m[0], m[1], u16::from_le_bytes([m[4], m[5]]));
        match kind {
            b'Q' => {
                let v = self.query(wid);
                if log() { eprintln!("[wlan] Q {wid:04x} -> {:02x?}", &v[..v.len().min(48)]); }
                let r = self.message(b'R', seq, wid, &v);
                self.send(0.0002, r);
            }
            b'W' => {
                let value: Vec<u8> = if wid == 0x30F0 {
                    let n = u16::from_le_bytes([m.get(6).copied().unwrap_or(0), m.get(7).copied().unwrap_or(0)]) as usize;
                    m.get(8..8 + n + 1).map(|x| x.to_vec()).unwrap_or_default()
                } else {
                    let n = m.get(6).copied().unwrap_or(0) as usize;
                    m.get(7..7 + n).map(|x| x.to_vec()).unwrap_or_default()
                };
                if log() && wid != 0x30F0 { eprintln!("[wlan] W {wid:04x} = {:02x?}", &value[..value.len().min(48)]); }
                // Boot ROM requests (download, reset, start) get no status reply; everything else does.
                let reply = !matches!(wid, 0x00F1 | 0x00F2 | 0x30F0 | 0x0012);
                self.write_wid(wid, value);
                if reply { let r = self.message(b'R', seq, 0x0005, &[1]); self.send(0.0002, r); }
            }
            k => if log() { eprintln!("[wlan] message type {k:02x}"); },
        }
    }

    fn request_download(&mut self, addr: u32, len: u32) {
        let mut v = addr.to_le_bytes().to_vec();
        v.extend((len as u16).to_le_bytes());
        let m = self.message(b'R', 0, 0x00F0, &v);
        self.send(0.0001, m);
    }

    fn write_wid(&mut self, wid: u16, value: Vec<u8>) {
        let now = self.now;
        match wid {
            0x00F2 => { // reset request: the boot ROM asks for the firmware, header first
                self.boot = Boot::Header; self.image.clear();
                self.request_download(0, 32);
            }
            0x30F0 => {
                let (data, sum) = value.split_at(value.len().saturating_sub(1));
                let ok = sum.first().is_some_and(|&s| data.iter().fold(0u8, |a, &b| a.wrapping_add(b)) == s);
                if !ok { if log() { eprintln!("[wlan] download checksum error at {}", self.image.len()); } }
                self.image.extend(data);
                match self.boot {
                    Boot::Header => {
                        let total = if self.image.len() >= 8 && &self.image[..4] == b"WLFW" { u32::from_le_bytes(self.image[4..8].try_into().unwrap()) + 32 } else { 32 };
                        if log() { eprintln!("[wlan] firmware header {:02x?}: image {total} bytes", &self.image[..self.image.len().min(16)]); }
                        self.boot = Boot::Body { next: 32, total };
                    }
                    Boot::Body { next, total } => { self.boot = Boot::Body { next: next + data.len() as u32, total }; }
                    _ => {}
                }
                if let Boot::Body { next, total } = self.boot {
                    if next < total { self.request_download(next, CHUNK.min(total - next)); }
                    else {
                        self.boot = Boot::Done;
                        if log() { eprintln!("[wlan] firmware downloaded ({} bytes)", self.image.len()); }
                        let m = self.message(b'R', 0, 0x00F1, &[]);
                        self.send(0.0001, m);
                    }
                }
            }
            0x00F1 => { self.boot = Boot::Running; self.info(0.05, 0x003D, &[1]); } // firmware started
            0x001E => {
                if value.first() == Some(&1) && self.scan_done_at.is_none() { self.scan_done_at = Some(now + (0.3 * P0_HZ) as u64); self.join_at = None; }
                self.wids.insert(wid, if self.scan_done_at.is_some() { vec![1] } else { value });
                return;
            }
            0x0000 => {
                if value.first().is_some_and(|&b| b & 0x80 != 0) { self.disconnect(); }
                else if self.wids.get(&0x3000).is_some_and(|s| !s.is_empty()) && self.scan_done_at.is_none() { self.join_at = Some((now + (0.4 * P0_HZ) as u64, None)); }
            }
            0x0020 => { self.join_at = Some((now + (0.4 * P0_HZ) as u64, value.first().map(|&i| i as usize))); }
            _ => {}
        }
        self.wids.insert(wid, value);
    }

    fn disconnect(&mut self) {
        if self.connected.take().is_some() {
            if let Some(l) = &mut self.link { l.disassociated(); }
            self.info(0.01, 0x0005, &[0]);
        }
    }

    fn join(&mut self, index: Option<usize>) {
        let ssid = self.wids.get(&0x3000).cloned().unwrap_or_default();
        let ap = match index { Some(i) => (i < self.aps.len()).then_some(i), None => self.aps.iter().position(|a| a.ssid == "*" || a.ssid.as_bytes() == &ssid[..]) };
        let mode = self.wids.get(&0x000C).and_then(|v| v.first().copied()).unwrap_or(0);
        let psk = self.wids.get(&0x3008).cloned().unwrap_or_default();
        let ok = ap.is_some_and(|i| {
            let a = &self.aps[i];
            if a.ssid == "*" { true } else if a.passphrase.is_empty() { mode == 0 } else { mode & 1 != 0 && mode & 2 == 0 && (a.passphrase == "*" || psk == a.passphrase.as_bytes() || psk.len() == 64) }
        });
        if log() { eprintln!("[wlan] join {:?} ssid {:?} mode {mode:02x}: {}", index, String::from_utf8_lossy(&ssid), if ok { "associated" } else { "failed" }); }
        if let (true, Some(i)) = (ok, ap) {
            self.connected = Some(i);
            let mac = self.mac;
            if let Some(l) = &mut self.link { l.associated(mac); }
            self.info(0.0, 0x0005, &[1]);
        } else {
            self.info(0.0, 0x0005, &[0]);
        }
    }

    fn query(&mut self, wid: u16) -> Vec<u8> {
        match wid {
            0x3001 => b"W53GA-EMU WLAN 1".to_vec(),
            0x300B => b"W53GA-EMU".to_vec(),
            0x300F => b"1.0".to_vec(),
            0x300C => self.mac.to_vec(),
            0x001F => vec![self.connected.map_or(0x9C, |i| self.aps[i].rssi as u8)],
            0x00B2 => vec![54],
            0x001E => vec![self.scan_done_at.is_some() as u8],
            0x0043 => vec![0],
            0x3012 => {
                // Site survey results, read in two parts: [entry bytes][part index][44-byte entries]
                let part = self.survey_part; self.survey_part ^= 1;
                let mut v = vec![0u8, part];
                if part == 0 {
                    for a in self.aps.iter().take(5) {
                        let mut e = [0u8; 44];
                        let s = a.ssid.as_bytes(); e[..s.len().min(32)].copy_from_slice(&s[..s.len().min(32)]);
                        e[0x21] = 0; e[0x22] = a.channel; e[0x23] = a.security();
                        e[0x24..0x2A].copy_from_slice(&a.bssid); e[0x2A] = a.rssi as u8;
                        v.extend(e);
                    }
                }
                v[0] = (v.len() - 2) as u8;
                v
            }
            w => self.wids.get(&w).cloned().unwrap_or_else(|| match w >> 12 { 0 => vec![0], 1 => vec![0, 0], 2 => vec![0; 4], _ => Vec::new() }),
        }
    }

    pub fn debug(&self) -> String {
        format!("boot {:?} image {} B, connected {:?}, frames host->module {} module->host {}, queued {}", self.boot, self.image.len(),
            self.connected.map(|i| self.aps[i].ssid.clone()), self.frames_in, self.frames_out, self.out.len())
    }
}

/// The module with one virtual access point (`ssid`, open when `passphrase` is empty) whose network is bridged to the
/// host's internet connection (`usernet`).
pub fn with_internet(ssid: &str, passphrase: &str) -> Wlan { with_host(ssid, passphrase, crate::usernet::UserNet::with_default_host()) }

/// The module with one virtual access point whose network is `net`.
pub fn with_host(ssid: &str, passphrase: &str, net: crate::usernet::UserNet) -> Wlan {
    let ap = Ap { ssid: ssid.to_string(), bssid: [0x02, 0x52, 0x45, 0x41, 0x00, 0x01], channel: 6, passphrase: passphrase.to_string(), rssi: -45 };
    let mut w = Wlan::new([0x02, 0xCE, 0x11, 0x57, 0xA0, 0x01], vec![ap]);
    w.link = Some(Box::new(net));
    w
}

/// `--wifi` value: "SSID" or "SSID:passphrase" (default "W53GA-NET", open; passphrase "*" accepts any key). SSID "*":
/// any network the unit joins by name is this access point, with any security and key (the unit's registered network,
/// without naming it; scans list it as "*").
/// A "#MAC" suffix (e.g. "#00:11:22:33:44:55") gives the module that hardware address instead of the emulator's own:
/// some online services identify the unit by it.
pub fn from_option(spec: Option<&str>) -> Wlan {
    let s = spec.filter(|s| !s.starts_with("--")).unwrap_or("W53GA-NET");
    let (ssid, pass, mac) = parse_spec(s);
    let mut w = with_internet(ssid, pass);
    if let Some(m) = mac { w.mac = m; }
    w
}

/// "SSID[:PASSPHRASE][#MAC]" -> (SSID, passphrase, MAC).
pub fn parse_spec(s: &str) -> (&str, &str, Option<[u8; 6]>) {
    let (ap, mac) = s.rsplit_once('#').map_or((s, None), |(a, m)| (a, parse_mac(m)));
    let (ssid, pass) = ap.split_once(':').unwrap_or((ap, ""));
    (ssid, pass, mac)
}

/// "00:11:22:33:44:55" (also '-' separated or plain hex).
pub fn parse_mac(s: &str) -> Option<[u8; 6]> {
    let h: String = s.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if h.len() != 12 { return None; }
    let mut m = [0u8; 6];
    for i in 0..6 { m[i] = u8::from_str_radix(&h[2 * i..2 * i + 2], 16).ok()?; }
    Some(m)
}
