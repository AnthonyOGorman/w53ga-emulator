//! User-mode networking behind the emulated Wi-Fi access point (like QEMU's "user" network): the unit gets an address
//! from a built-in DHCP server, its DNS queries are answered by the host's resolver, and its TCP connections and UDP
//! datagrams are carried by ordinary host sockets, so it reaches the real internet without a virtual adapter or admin
//! rights. The unit's TCP stack talks to this proxy, which keeps one host connection per unit connection.
//!
//!   10.0.2.2  gateway (and DHCP server; TCP to it reaches this computer's 127.0.0.1)   10.0.2.3  DNS   10.0.2.15  the unit
//!
//! Host I/O goes through `HostNet`; `ThreadedHost` (native builds) uses std sockets on worker threads so the emulator
//! never blocks. Without a host backend (WebAssembly for now) connections are refused.

use std::collections::{HashMap, VecDeque};
use std::net::{Ipv4Addr, SocketAddrV4};

const P0_HZ: f64 = 100.0e6 / 3.0;
const GW: [u8; 4] = [10, 0, 2, 2];
const DNS: [u8; 4] = [10, 0, 2, 3];
const CLIENT: [u8; 4] = [10, 0, 2, 15];
const GW_MAC: [u8; 6] = [0x52, 0x55, 0x0A, 0x00, 0x02, 0x02];
const MSS: usize = 1400;

fn log() -> bool { std::env::var_os("AR_NET_LOG").is_some() }

/// Events from the host side.
pub enum Event {
    Connected(u64),
    Refused(u64),
    Data(u64, Vec<u8>),
    Eof(u64),
    Datagram(u64, SocketAddrV4, Vec<u8>),
    Resolved(u64, Vec<Ipv4Addr>),
}

/// Host sockets. Every call returns at once; results arrive through `poll`.
pub trait HostNet: Send {
    fn connect(&mut self, id: u64, to: SocketAddrV4);
    fn send(&mut self, id: u64, data: Vec<u8>);
    /// No more data from the unit (half close); the host may still send.
    fn shutdown(&mut self, id: u64);
    fn close(&mut self, id: u64);
    fn datagram(&mut self, id: u64, to: SocketAddrV4, data: Vec<u8>);
    fn resolve(&mut self, id: u64, name: String);
    fn poll(&mut self) -> Vec<Event>;
}

/// No host networking: every connection is refused, names do not resolve.
pub struct NoHost(Vec<Event>);
impl HostNet for NoHost {
    fn connect(&mut self, id: u64, _: SocketAddrV4) { self.0.push(Event::Refused(id)); }
    fn send(&mut self, _: u64, _: Vec<u8>) {}
    fn shutdown(&mut self, _: u64) {}
    fn close(&mut self, _: u64) {}
    fn datagram(&mut self, _: u64, _: SocketAddrV4, _: Vec<u8>) {}
    fn resolve(&mut self, id: u64, _: String) { self.0.push(Event::Resolved(id, Vec::new())); }
    fn poll(&mut self) -> Vec<Event> { std::mem::take(&mut self.0) }
}

/// Host requests as plain data (for a relay that executes them elsewhere).
#[derive(Debug, Clone)]
pub enum Cmd { Connect(u64, SocketAddrV4), Send(u64, Vec<u8>), Shutdown(u64), Close(u64), Datagram(u64, SocketAddrV4, Vec<u8>), Resolve(u64, String) }

/// A host backend that only queues the requests and hands out events given to it: the browser front end carries both
/// to `webserve`'s `/net` relay, which runs them on real sockets (`relay_apply`).
#[derive(Clone, Default)]
pub struct QueueHost { pub cmds: std::sync::Arc<std::sync::Mutex<Vec<Cmd>>>, pub events: std::sync::Arc<std::sync::Mutex<Vec<Event>>> }

impl HostNet for QueueHost {
    fn connect(&mut self, id: u64, to: SocketAddrV4) { self.cmds.lock().unwrap().push(Cmd::Connect(id, to)); }
    fn send(&mut self, id: u64, data: Vec<u8>) { self.cmds.lock().unwrap().push(Cmd::Send(id, data)); }
    fn shutdown(&mut self, id: u64) { self.cmds.lock().unwrap().push(Cmd::Shutdown(id)); }
    fn close(&mut self, id: u64) { self.cmds.lock().unwrap().push(Cmd::Close(id)); }
    fn datagram(&mut self, id: u64, to: SocketAddrV4, data: Vec<u8>) { self.cmds.lock().unwrap().push(Cmd::Datagram(id, to, data)); }
    fn resolve(&mut self, id: u64, name: String) { self.cmds.lock().unwrap().push(Cmd::Resolve(id, name)); }
    fn poll(&mut self) -> Vec<Event> { std::mem::take(&mut *self.events.lock().unwrap()) }
}

fn b64() -> base64::engine::GeneralPurpose { base64::engine::general_purpose::STANDARD }

/// Relay wire format (JSON): requests `[{"op":"connect","id":1,"to":"1.2.3.4:80"}, {"op":"send","id":1,"data":B64}, ...]`.
pub fn cmds_to_json(cmds: &[Cmd]) -> String {
    use base64::Engine;
    let v: Vec<serde_json::Value> = cmds.iter().map(|c| match c {
        Cmd::Connect(id, to) => serde_json::json!({"op": "connect", "id": id, "to": to.to_string()}),
        Cmd::Send(id, d) => serde_json::json!({"op": "send", "id": id, "data": b64().encode(d)}),
        Cmd::Shutdown(id) => serde_json::json!({"op": "shutdown", "id": id}),
        Cmd::Close(id) => serde_json::json!({"op": "close", "id": id}),
        Cmd::Datagram(id, to, d) => serde_json::json!({"op": "datagram", "id": id, "to": to.to_string(), "data": b64().encode(d)}),
        Cmd::Resolve(id, n) => serde_json::json!({"op": "resolve", "id": id, "name": n}),
    }).collect();
    serde_json::Value::Array(v).to_string()
}

/// Events in the relay wire format: `[{"ev":"connected","id":1}, {"ev":"data","id":1,"data":B64}, ...]`.
pub fn events_from_json(s: &str) -> Vec<Event> {
    use base64::Engine;
    let Ok(serde_json::Value::Array(v)) = serde_json::from_str::<serde_json::Value>(s) else { return Vec::new() };
    v.iter().filter_map(|e| {
        let id = e["id"].as_u64()?;
        let data = || e["data"].as_str().and_then(|d| b64().decode(d).ok()).unwrap_or_default();
        Some(match e["ev"].as_str()? {
            "connected" => Event::Connected(id), "refused" => Event::Refused(id), "eof" => Event::Eof(id), "data" => Event::Data(id, data()),
            "datagram" => Event::Datagram(id, e["from"].as_str()?.parse().ok()?, data()),
            "resolved" => Event::Resolved(id, e["ips"].as_array()?.iter().filter_map(|x| x.as_str()?.parse().ok()).collect()),
            _ => return None,
        })
    }).collect()
}

/// Server side of the relay: run the JSON requests on `host` and return the events it has produced, as JSON.
pub fn relay_apply(host: &mut dyn HostNet, cmds: &str) -> String {
    use base64::Engine;
    if let Ok(serde_json::Value::Array(v)) = serde_json::from_str::<serde_json::Value>(cmds) {
        for c in &v {
            let (Some(op), Some(id)) = (c["op"].as_str(), c["id"].as_u64()) else { continue };
            let to = c["to"].as_str().and_then(|t| t.parse::<SocketAddrV4>().ok());
            let data = c["data"].as_str().and_then(|d| b64().decode(d).ok()).unwrap_or_default();
            match op {
                "connect" => if let Some(to) = to { host.connect(id, to) },
                "send" => host.send(id, data),
                "shutdown" => host.shutdown(id),
                "close" => host.close(id),
                "datagram" => if let Some(to) = to { host.datagram(id, to, data) },
                "resolve" => if let Some(n) = c["name"].as_str() { host.resolve(id, n.to_string()) },
                _ => {}
            }
        }
    }
    let ev: Vec<serde_json::Value> = host.poll().into_iter().map(|e| match e {
        Event::Connected(id) => serde_json::json!({"ev": "connected", "id": id}),
        Event::Refused(id) => serde_json::json!({"ev": "refused", "id": id}),
        Event::Eof(id) => serde_json::json!({"ev": "eof", "id": id}),
        Event::Data(id, d) => serde_json::json!({"ev": "data", "id": id, "data": b64().encode(d)}),
        Event::Datagram(id, from, d) => serde_json::json!({"ev": "datagram", "id": id, "from": from.to_string(), "data": b64().encode(d)}),
        Event::Resolved(id, ips) => serde_json::json!({"ev": "resolved", "id": id, "ips": ips.iter().map(|i| i.to_string()).collect::<Vec<_>>()}),
    }).collect();
    serde_json::Value::Array(ev).to_string()
}

#[cfg(not(target_arch = "wasm32"))]
pub use threaded::ThreadedHost;

#[cfg(not(target_arch = "wasm32"))]
mod threaded {
    use super::{Event, HostNet};
    use std::collections::HashMap;
    use std::io::{Read, Write};
    use std::net::{Ipv4Addr, Shutdown, SocketAddr, SocketAddrV4, TcpStream, ToSocketAddrs, UdpSocket};
    use std::sync::mpsc::{channel, Receiver, Sender};
    use std::time::Duration;

    enum Cmd { Data(Vec<u8>), Shutdown, Close }

    /// Host sockets on worker threads (one per connection or UDP flow).
    pub struct ThreadedHost { tx: Sender<Event>, rx: Receiver<Event>, tcp: HashMap<u64, Sender<Cmd>>, udp: HashMap<u64, UdpSocket> }

    impl Default for ThreadedHost { fn default() -> Self { let (tx, rx) = channel(); ThreadedHost { tx, rx, tcp: HashMap::new(), udp: HashMap::new() } } }

    impl HostNet for ThreadedHost {
        fn connect(&mut self, id: u64, to: SocketAddrV4) {
            let (ctx, crx) = channel::<Cmd>();
            self.tcp.insert(id, ctx);
            let ev = self.tx.clone();
            std::thread::spawn(move || {
                let s = match TcpStream::connect_timeout(&SocketAddr::V4(to), Duration::from_secs(10)) { Ok(s) => s, Err(_) => { let _ = ev.send(Event::Refused(id)); return; } };
                let _ = s.set_nodelay(true);
                let _ = ev.send(Event::Connected(id));
                let mut r = match s.try_clone() { Ok(r) => r, Err(_) => { let _ = ev.send(Event::Eof(id)); return; } };
                let ev2 = ev.clone();
                std::thread::spawn(move || {
                    let mut b = vec![0u8; 16384];
                    loop { match r.read(&mut b) { Ok(0) | Err(_) => { let _ = ev2.send(Event::Eof(id)); break; } Ok(n) => { if ev2.send(Event::Data(id, b[..n].to_vec())).is_err() { break; } } } }
                });
                let mut w = s;
                for c in crx {
                    match c {
                        Cmd::Data(d) => if w.write_all(&d).is_err() { break; },
                        Cmd::Shutdown => { let _ = w.shutdown(Shutdown::Write); }
                        Cmd::Close => { let _ = w.shutdown(Shutdown::Both); break; }
                    }
                }
            });
        }
        fn send(&mut self, id: u64, data: Vec<u8>) { if let Some(c) = self.tcp.get(&id) { let _ = c.send(Cmd::Data(data)); } }
        fn shutdown(&mut self, id: u64) { if let Some(c) = self.tcp.get(&id) { let _ = c.send(Cmd::Shutdown); } }
        fn close(&mut self, id: u64) { if let Some(c) = self.tcp.remove(&id) { let _ = c.send(Cmd::Close); } self.udp.remove(&id); }
        fn datagram(&mut self, id: u64, to: SocketAddrV4, data: Vec<u8>) {
            if !self.udp.contains_key(&id) {
                let Ok(s) = UdpSocket::bind(("0.0.0.0", 0)) else { return };
                let _ = s.set_read_timeout(Some(Duration::from_secs(120)));
                if let Ok(r) = s.try_clone() {
                    let ev = self.tx.clone();
                    std::thread::spawn(move || {
                        let mut b = vec![0u8; 2048];
                        while let Ok((n, SocketAddr::V4(from))) = r.recv_from(&mut b) { if ev.send(Event::Datagram(id, from, b[..n].to_vec())).is_err() { break; } }
                    });
                }
                self.udp.insert(id, s);
            }
            if let Some(s) = self.udp.get(&id) { let _ = s.send_to(&data, to); }
        }
        fn resolve(&mut self, id: u64, name: String) {
            let ev = self.tx.clone();
            std::thread::spawn(move || {
                let v: Vec<Ipv4Addr> = (name.as_str(), 0).to_socket_addrs().map(|a| a.filter_map(|x| match x { SocketAddr::V4(v) => Some(*v.ip()), _ => None }).collect()).unwrap_or_default();
                let _ = ev.send(Event::Resolved(id, v));
            });
        }
        fn poll(&mut self) -> Vec<Event> { self.rx.try_iter().collect() }
    }
}

/// Log the first line of an HTTP request or response.
fn first_line(dir: &str, remote: SocketAddrV4, d: &[u8]) {
    let line: String = d.iter().take_while(|&&b| b != 13 && b != 10).take(160).map(|&b| if (32..127).contains(&b) { b as char } else { '.' }).collect();
    eprintln!("[net] {dir} {remote} {line}");
}

fn sum16(data: &[u8], mut acc: u32) -> u32 {
    let mut i = 0;
    while i + 1 < data.len() { acc += u16::from_be_bytes([data[i], data[i + 1]]) as u32; i += 2; }
    if i < data.len() { acc += (data[i] as u32) << 8; }
    acc
}
fn fold(mut acc: u32) -> u16 { while acc >> 16 != 0 { acc = (acc & 0xFFFF) + (acc >> 16); } !(acc as u16) }

#[derive(PartialEq, Debug)]
enum St { Connecting, SynRcvd, Open }

struct Tcp {
    id: u64, st: St, unit_port: u16, remote: SocketAddrV4,
    rcv_nxt: u32, snd_una: u32, snd_nxt: u32, iss: u32,
    unit_wnd: u32, mss: usize,
    /// Host data from `snd_una` on (sent and unsent).
    buf: VecDeque<u8>,
    host_eof: bool, fin_sent: bool, unit_fin: bool,
    progress: u64,
    /// Bytes unit -> host and host -> unit (for the log).
    up: u64, down: u64,
}

pub struct UserNet {
    host: Box<dyn HostNet>,
    client_mac: [u8; 6],
    out: VecDeque<Vec<u8>>,
    tcp: HashMap<(u16, SocketAddrV4), Tcp>,
    udp: HashMap<(u16, SocketAddrV4), u64>,
    dns: HashMap<u64, (Vec<u8>, u16)>,
    by_id: HashMap<u64, (u16, SocketAddrV4)>,
    udp_by_id: HashMap<u64, u16>,
    next_id: u64,
    ip_id: u16,
    pub stats: [u64; 4],
}

impl UserNet {
    pub fn new(host: Box<dyn HostNet>) -> UserNet {
        UserNet { host, client_mac: [0xFF; 6], out: VecDeque::new(), tcp: HashMap::new(), udp: HashMap::new(), dns: HashMap::new(), by_id: HashMap::new(),
            udp_by_id: HashMap::new(), next_id: 1, ip_id: 1, stats: [0; 4] }
    }
    /// Host networking where available (native builds), otherwise none.
    pub fn with_default_host() -> UserNet {
        #[cfg(not(target_arch = "wasm32"))]
        { UserNet::new(Box::new(ThreadedHost::default())) }
        #[cfg(target_arch = "wasm32")]
        { UserNet::new(Box::new(NoHost(Vec::new()))) }
    }

    fn eth(&mut self, dst: [u8; 6], ethertype: u16, payload: &[u8]) {
        let mut f = Vec::with_capacity(14 + payload.len());
        f.extend(dst); f.extend(GW_MAC); f.extend(ethertype.to_be_bytes()); f.extend(payload);
        self.out.push_back(f);
    }

    fn ip(&mut self, src: [u8; 4], dst: [u8; 4], proto: u8, l4: &[u8]) {
        let mut h = vec![0x45, 0, 0, 0, 0, 0, 0x40, 0, 64, proto, 0, 0];
        h[2..4].copy_from_slice(&((20 + l4.len()) as u16).to_be_bytes());
        h[4..6].copy_from_slice(&self.ip_id.to_be_bytes()); self.ip_id = self.ip_id.wrapping_add(1);
        h.extend(src); h.extend(dst);
        let c = fold(sum16(&h, 0)); h[10..12].copy_from_slice(&c.to_be_bytes());
        h.extend(l4);
        let mac = if dst == [255; 4] { [0xFF; 6] } else { self.client_mac };
        self.eth(mac, 0x0800, &h);
    }

    fn l4_checksum(src: [u8; 4], dst: [u8; 4], proto: u8, seg: &[u8]) -> u16 {
        let mut p = Vec::with_capacity(12); p.extend(src); p.extend(dst); p.push(0); p.push(proto); p.extend((seg.len() as u16).to_be_bytes());
        fold(sum16(seg, sum16(&p, 0)))
    }

    fn udp_out(&mut self, src: SocketAddrV4, dst: SocketAddrV4, data: &[u8]) {
        let mut u = Vec::with_capacity(8 + data.len());
        u.extend(src.port().to_be_bytes()); u.extend(dst.port().to_be_bytes()); u.extend(((8 + data.len()) as u16).to_be_bytes()); u.extend([0, 0]); u.extend(data);
        let c = Self::l4_checksum(src.ip().octets(), dst.ip().octets(), 17, &u);
        u[6..8].copy_from_slice(&(if c == 0 { 0xFFFF } else { c }).to_be_bytes());
        self.ip(src.ip().octets(), dst.ip().octets(), 17, &u);
    }

    fn tcp_out(&mut self, remote: SocketAddrV4, unit_port: u16, seq: u32, ack: u32, flags: u8, data: &[u8], syn_mss: bool) {
        let opts: &[u8] = if syn_mss { &[2, 4, (MSS >> 8) as u8, MSS as u8] } else { &[] };
        let hl = 20 + opts.len();
        let mut t = Vec::with_capacity(hl + data.len());
        t.extend(remote.port().to_be_bytes()); t.extend(unit_port.to_be_bytes()); t.extend(seq.to_be_bytes()); t.extend(ack.to_be_bytes());
        t.push(((hl / 4) as u8) << 4); t.push(flags); t.extend(65535u16.to_be_bytes()); t.extend([0, 0, 0, 0]); t.extend(opts); t.extend(data);
        let c = Self::l4_checksum(remote.ip().octets(), CLIENT, 6, &t);
        t[16..18].copy_from_slice(&c.to_be_bytes());
        self.ip(remote.ip().octets(), CLIENT, 6, &t);
    }

    fn arp(&mut self, p: &[u8]) {
        if p.len() < 28 || u16::from_be_bytes([p[6], p[7]]) != 1 { return; }
        let (sha, spa, tpa) = (&p[8..14], &p[14..18], &p[24..28]);
        self.client_mac.copy_from_slice(sha);
        if tpa[..3] != CLIENT[..3] || tpa == CLIENT || tpa == spa { return; }
        let mut r = vec![0, 1, 8, 0, 6, 4, 0, 2];
        r.extend(GW_MAC); r.extend(tpa); r.extend(sha); r.extend(spa);
        let dst: [u8; 6] = sha.try_into().unwrap();
        self.eth(dst, 0x0806, &r);
    }

    fn dhcp(&mut self, d: &[u8]) {
        if d.len() < 240 || d[0] != 1 || d[236..240] != [99, 130, 83, 99] { return; }
        let mut kind = 0;
        let mut i = 240;
        while i + 1 < d.len() && d[i] != 255 { if d[i] == 0 { i += 1; continue; } let l = d[i + 1] as usize; if d[i] == 53 && l >= 1 { kind = d[i + 2]; } i += 2 + l; }
        let reply = match kind { 1 => 2, 3 => 5, _ => return };
        let mut r = vec![0u8; 240];
        r[0] = 2; r[1] = 1; r[2] = 6; r[4..8].copy_from_slice(&d[4..8]); r[10..12].copy_from_slice(&d[10..12]);
        r[16..20].copy_from_slice(&CLIENT); r[20..24].copy_from_slice(&GW); r[28..44].copy_from_slice(&d[28..44]);
        r[236..240].copy_from_slice(&[99, 130, 83, 99]);
        r.extend([53, 1, reply, 54, 4]); r.extend(GW); r.extend([51, 4, 0, 1, 0x51, 0x80, 1, 4, 255, 255, 255, 0, 3, 4]); r.extend(GW);
        r.extend([6, 4]); r.extend(DNS); r.push(255);
        if log() { eprintln!("[net] DHCP {} -> 10.0.2.15", if reply == 2 { "offer" } else { "ack" }); }
        self.udp_out(SocketAddrV4::new(GW.into(), 67), SocketAddrV4::new(Ipv4Addr::BROADCAST, 68), &r);
    }

    fn dns_query(&mut self, src_port: u16, q: &[u8]) {
        if q.len() < 12 { return; }
        let mut name = Vec::new();
        let mut i = 12;
        while i < q.len() && q[i] != 0 { let l = q[i] as usize; if i + 1 + l > q.len() { return; } name.push(String::from_utf8_lossy(&q[i + 1..i + 1 + l]).into_owned()); i += 1 + l; }
        let end = i + 5;
        if end > q.len() { return; }
        let qtype = u16::from_be_bytes([q[i + 1], q[i + 2]]);
        let id = self.next_id; self.next_id += 1;
        self.dns.insert(id, (q[..end].to_vec(), src_port));
        let name = name.join(".");
        if log() { eprintln!("[net] DNS {name} (type {qtype})"); }
        if qtype == 1 { self.host.resolve(id, name); } else { self.dns_answer(id, Vec::new(), false); }
    }

    fn dns_answer(&mut self, id: u64, ips: Vec<Ipv4Addr>, nx: bool) {
        let Some((q, port)) = self.dns.remove(&id) else { return };
        let mut r = q.clone();
        r[2] = 0x81 | (q[2] & 1); r[3] = if nx && ips.is_empty() { 0x83 } else { 0x80 };
        r[6..8].copy_from_slice(&(ips.len() as u16).to_be_bytes()); r[8..12].copy_from_slice(&[0, 0, 0, 0]);
        for ip in &ips { r.extend([0xC0, 12, 0, 1, 0, 1, 0, 0, 0, 60, 0, 4]); r.extend(ip.octets()); }
        if log() { eprintln!("[net] DNS answer {ips:?}"); }
        self.udp_out(SocketAddrV4::new(DNS.into(), 53), SocketAddrV4::new(CLIENT.into(), port), &r);
    }

    fn icmp(&mut self, src: [u8; 4], dst: [u8; 4], p: &[u8]) {
        if p.len() < 8 || p[0] != 8 || (dst != GW && dst != DNS) { return; }
        let mut r = p.to_vec(); r[0] = 0; r[2] = 0; r[3] = 0;
        let c = fold(sum16(&r, 0)); r[2..4].copy_from_slice(&c.to_be_bytes());
        self.ip(dst, src, 1, &r);
    }

    fn tcp_in(&mut self, now: u64, dst: [u8; 4], seg: &[u8]) {
        if seg.len() < 20 { return; }
        let (sport, dport) = (u16::from_be_bytes([seg[0], seg[1]]), u16::from_be_bytes([seg[2], seg[3]]));
        let seq = u32::from_be_bytes(seg[4..8].try_into().unwrap());
        let ack = u32::from_be_bytes(seg[8..12].try_into().unwrap());
        let hl = ((seg[12] >> 4) as usize * 4).max(20).min(seg.len());
        let flags = seg[13];
        let wnd = u16::from_be_bytes([seg[14], seg[15]]) as u32;
        let data = &seg[hl..];
        let remote = SocketAddrV4::new(dst.into(), dport);
        let key = (sport, remote);
        const FIN: u8 = 1; const SYN: u8 = 2; const RST: u8 = 4; const ACK: u8 = 16;
        if flags & RST != 0 { if let Some(c) = self.tcp.remove(&key) { self.host.close(c.id); self.by_id.remove(&c.id); } return; }
        if flags & SYN != 0 && flags & ACK == 0 {
            if self.tcp.contains_key(&key) { return; } // retransmitted SYN while connecting
            let mut mss = 536;
            let mut i = 20;
            while i < hl { match seg[i] { 0 => break, 1 => i += 1, k => { let l = seg.get(i + 1).copied().unwrap_or(2).max(2) as usize; if k == 2 && l == 4 && i + 4 <= hl { mss = u16::from_be_bytes([seg[i + 2], seg[i + 3]]) as usize; } i += l; } } }
            let id = self.next_id; self.next_id += 1;
            let iss = (now as u32).wrapping_mul(2654435761);
            if log() { eprintln!("[net] TCP connect {remote} (unit port {sport})"); }
            self.tcp.insert(key, Tcp { id, st: St::Connecting, unit_port: sport, remote, rcv_nxt: seq.wrapping_add(1), snd_una: iss, snd_nxt: iss, iss, unit_wnd: wnd, mss: mss.min(MSS),
                buf: VecDeque::new(), host_eof: false, fin_sent: false, unit_fin: false, progress: now, up: 0, down: 0 });
            self.by_id.insert(id, key);
            // The gateway is this computer (as in QEMU): a server here is reachable from the unit at 10.0.2.2.
            let to = if remote.ip().octets() == GW { SocketAddrV4::new(Ipv4Addr::LOCALHOST, remote.port()) } else { remote };
            self.host.connect(id, to);
            self.stats[0] += 1;
            return;
        }
        let Some(mut c) = self.tcp.remove(&key) else {
            if flags & ACK != 0 { self.tcp_out(remote, sport, ack, 0, RST, &[], false); }
            return;
        };
        c.unit_wnd = wnd;
        if flags & ACK != 0 {
            if c.st == St::SynRcvd && ack == c.iss.wrapping_add(1) { c.st = St::Open; c.snd_una = ack; c.progress = now; }
            let acked = ack.wrapping_sub(c.snd_una);
            if c.st == St::Open && acked > 0 && acked <= c.snd_nxt.wrapping_sub(c.snd_una) {
                let data_acked = (acked as usize).min(c.buf.len());
                c.buf.drain(..data_acked);
                c.snd_una = ack; c.progress = now;
            }
        }
        let mut reply = false;
        if !data.is_empty() || flags & FIN != 0 {
            if seq == c.rcv_nxt {
                if !data.is_empty() {
                    if log() { first_line("->", c.remote, data); }
                    self.host.send(c.id, data.to_vec()); c.rcv_nxt = c.rcv_nxt.wrapping_add(data.len() as u32); self.stats[1] += data.len() as u64; c.up += data.len() as u64;
                }
                if flags & FIN != 0 && !c.unit_fin { c.unit_fin = true; c.rcv_nxt = c.rcv_nxt.wrapping_add(1); self.host.shutdown(c.id); }
            }
            reply = true;
        }
        if reply { self.tcp_out(c.remote, c.unit_port, c.snd_nxt, c.rcv_nxt, ACK, &[], false); }
        if c.unit_fin && c.fin_sent && c.snd_una == c.snd_nxt {
            if log() { eprintln!("[net] TCP closed {} (sent {} B, received {} B)", c.remote, c.up, c.down); }
            self.host.close(c.id); self.by_id.remove(&c.id);
            return;
        }
        self.tcp.insert(key, c);
    }

    fn udp_in(&mut self, src: [u8; 4], dst: [u8; 4], p: &[u8]) {
        if p.len() < 8 { return; }
        let (sport, dport) = (u16::from_be_bytes([p[0], p[1]]), u16::from_be_bytes([p[2], p[3]]));
        let len = (u16::from_be_bytes([p[4], p[5]]) as usize).clamp(8, p.len());
        let data = &p[8..len];
        if dport == 67 { return self.dhcp(data); }
        if dst == DNS && dport == 53 { return self.dns_query(sport, data); }
        if src != CLIENT { return; }
        let to = SocketAddrV4::new(dst.into(), dport);
        let id = match self.udp.get(&(sport, to)) { Some(&id) => id, None => { let id = self.next_id; self.next_id += 1; self.udp.insert((sport, to), id); self.udp_by_id.insert(id, sport); id } };
        self.host.datagram(id, to, data.to_vec());
    }

    fn host_events(&mut self, now: u64) {
        for e in self.host.poll() {
            match e {
                Event::Resolved(id, ips) => { let nx = ips.is_empty(); self.dns_answer(id, ips, nx); }
                Event::Datagram(id, from, data) => if let Some(&port) = self.udp_by_id.get(&id) { self.udp_out(from, SocketAddrV4::new(CLIENT.into(), port), &data); },
                Event::Connected(id) | Event::Refused(id) | Event::Data(id, _) | Event::Eof(id) => {
                    let Some(key) = self.by_id.get(&id).copied() else { continue };
                    let Some(mut c) = self.tcp.remove(&key) else { continue };
                    match e {
                        Event::Connected(_) => { c.st = St::SynRcvd; c.snd_nxt = c.iss.wrapping_add(1); c.progress = now; self.tcp_out(c.remote, c.unit_port, c.iss, c.rcv_nxt, 0x12, &[], true); }
                        Event::Refused(_) => {
                            if log() { eprintln!("[net] TCP {} refused", c.remote); }
                            self.tcp_out(c.remote, c.unit_port, 0, c.rcv_nxt, 0x14, &[], false); self.by_id.remove(&id); continue;
                        }
                        Event::Data(_, d) => { if log() && c.down == 0 { first_line("<-", c.remote, &d); } self.stats[2] += d.len() as u64; c.down += d.len() as u64; c.buf.extend(d); }
                        Event::Eof(_) => c.host_eof = true,
                        _ => {}
                    }
                    self.tcp.insert(key, c);
                }
            }
        }
        // Send what the unit's window allows; go back to the oldest unacknowledged byte if nothing moved for 0.5 s.
        let keys: Vec<_> = self.tcp.keys().copied().collect();
        for key in keys {
            let mut c = self.tcp.remove(&key).unwrap();
            if c.st == St::Open {
                if c.snd_nxt != c.snd_una && now.saturating_sub(c.progress) > (0.5 * P0_HZ) as u64 { c.snd_nxt = c.snd_una; c.fin_sent = false; c.progress = now; self.stats[3] += 1; }
                loop {
                    let in_flight = c.snd_nxt.wrapping_sub(c.snd_una) as usize;
                    let sent = in_flight.min(c.buf.len());
                    let room = (c.unit_wnd as usize).min(32768).saturating_sub(in_flight);
                    let n = (c.buf.len() - sent).min(c.mss).min(room);
                    if n == 0 { break; }
                    let d: Vec<u8> = c.buf.range(sent..sent + n).copied().collect();
                    self.tcp_out(c.remote, c.unit_port, c.snd_nxt, c.rcv_nxt, 0x18, &d, false);
                    c.snd_nxt = c.snd_nxt.wrapping_add(n as u32);
                }
                if c.host_eof && !c.fin_sent && c.snd_nxt.wrapping_sub(c.snd_una) as usize == c.buf.len() {
                    self.tcp_out(c.remote, c.unit_port, c.snd_nxt, c.rcv_nxt, 0x11, &[], false);
                    c.snd_nxt = c.snd_nxt.wrapping_add(1); c.fin_sent = true;
                }
            }
            self.tcp.insert(key, c);
        }
    }

    pub fn debug(&self) -> String {
        format!("TCP connections opened {}, open now {}, bytes unit->net {} net->unit {}, retransmissions {}", self.stats[0], self.tcp.len(), self.stats[1], self.stats[2], self.stats[3])
    }
}

impl crate::wlan::Link for UserNet {
    fn input(&mut self, now: u64, f: &[u8]) {
        if f.len() < 14 { return; }
        let ethertype = u16::from_be_bytes([f[12], f[13]]);
        let p = &f[14..];
        match ethertype {
            0x0806 => self.arp(p),
            0x0800 if p.len() >= 20 && p[0] >> 4 == 4 => {
                self.client_mac.copy_from_slice(&f[6..12]);
                let ihl = (p[0] & 15) as usize * 4;
                let total = (u16::from_be_bytes([p[2], p[3]]) as usize).clamp(ihl, p.len());
                let (src, dst): ([u8; 4], [u8; 4]) = (p[12..16].try_into().unwrap(), p[16..20].try_into().unwrap());
                let l4 = &p[ihl..total];
                match p[9] { 1 => self.icmp(src, dst, l4), 6 => self.tcp_in(now, dst, l4), 17 => self.udp_in(src, dst, l4), _ => {} }
            }
            _ => {}
        }
    }
    fn output(&mut self, now: u64) -> Option<Vec<u8>> {
        if self.out.is_empty() { self.host_events(now); }
        self.out.pop_front()
    }
}
