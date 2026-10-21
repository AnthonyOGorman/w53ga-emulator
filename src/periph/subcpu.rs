//! Companion MCU (R5F100) link on SCIF7: frame codec recovered from the original parser
//! and a scripted peer.
//!
//! Wire frame: seq, LE16 len, seq^FF, len^FFFF, then body = LE16 event, LE16 mode/state, u8 status,
//! [main->peer only: 64-byte mirror], u8 command, LE16 len, payload; then CRC16 (reflected 0x8408, init FFFF)
//! over the body, low byte first. Peer->main frames are len+16 bytes, main->peer len+80.

use std::collections::VecDeque;

pub fn crc16(data: &[u8]) -> u16 {
    let mut v: u16 = 0xFFFF;
    for &b in data { v ^= b as u16; for _ in 0..8 { v = if v & 1 != 0 { (v >> 1) ^ 0x8408 } else { v >> 1 }; } }
    v
}

/// Encode a controller -> main frame.
pub fn encode(seq: u8, cmd: u8, payload: &[u8], event: u16, state: u16, status: u8) -> Vec<u8> {
    let n = payload.len() as u16;
    let mut f = vec![seq, n as u8, (n >> 8) as u8, seq ^ 0xFF, !n as u8, (!n >> 8) as u8];
    let mut body = Vec::new();
    body.extend(event.to_le_bytes()); body.extend(state.to_le_bytes()); body.push(status);
    body.push(cmd); body.extend(n.to_le_bytes()); body.extend(payload);
    let c = crc16(&body);
    f.extend(&body); f.extend(c.to_le_bytes());
    f
}

/// A decoded main -> controller frame.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MainFrame { pub seq: u8, pub event: u16, pub state: u16, pub status: u8, pub command: u8, pub payload: Vec<u8> }

/// Try to decode one main -> controller frame at the start of `d`; returns (frame, bytes consumed).
pub fn decode_main(d: &[u8]) -> Option<Result<(MainFrame, usize), usize>> {
    if d.len() < 6 { return None; }
    let (seq, len) = (d[0], u16::from_le_bytes([d[1], d[2]]));
    if seq == 0 || d[3] != seq ^ 0xFF || u16::from_le_bytes([d[4], d[5]]) != !len { return Some(Err(1)); }
    let total = len as usize + 80;
    if d.len() < total { return None; }
    let body = &d[6..total - 2];
    if crc16(body) != u16::from_le_bytes([d[total - 2], d[total - 1]]) { return Some(Err(1)); }
    let f = MainFrame { seq, event: u16::from_le_bytes([body[0], body[1]]), state: u16::from_le_bytes([body[2], body[3]]), status: body[4],
        command: body[69], payload: body[72..72 + len as usize].to_vec() };
    Some(Ok((f, total)))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Trigger { AtStep(u64), AfterReply }

/// Scripted controller: queued frames released at a step or after the next main reply.
#[derive(Default, serde::Serialize, serde::Deserialize)]
pub struct Peer {
    pub seq: u8,
    pub queue: VecDeque<(Trigger, u8, Vec<u8>)>,
    cursor: usize,
    pub received: Vec<MainFrame>,
    replies_at_head: usize,
    pub sent: u64,
}

impl Peer {
    pub fn new(next_seq: u8) -> Peer { Peer { seq: next_seq, ..Default::default() } }

    pub fn push(&mut self, t: Trigger, cmd: u8, payload: Vec<u8>) {
        if self.queue.is_empty() { self.replies_at_head = self.received.len(); }
        self.queue.push_back((t, cmd, payload));
    }

    /// Parse new transmitted bytes; return frames to put on the wire now.
    pub fn poll(&mut self, now: u64, tx_log: &[u8]) -> Vec<Vec<u8>> {
        while self.cursor < tx_log.len() {
            match decode_main(&tx_log[self.cursor..]) {
                None => break,
                Some(Err(skip)) => self.cursor += skip,
                Some(Ok((f, n))) => { self.cursor += n; self.received.push(f); }
            }
        }
        let mut out = Vec::new();
        while let Some((t, cmd, payload)) = self.queue.front().cloned() {
            let ready = match t { Trigger::AtStep(s) => now >= s, Trigger::AfterReply => self.received.len() > self.replies_at_head };
            if !ready { break; }
            self.queue.pop_front();
            let f = encode(self.seq, cmd, &payload, 0, 0, 0);
            if std::env::var_os("AR_PEER_LOG").is_some() { eprintln!("[peer] step {now}: send seq {} cmd {cmd:02x} {:02x?}", self.seq, f); }
            out.push(f);
            self.seq = if self.seq == 255 { 1 } else { self.seq + 1 };
            self.sent += 1;
            self.replies_at_head = self.received.len();
        }
        out
    }

    /// Skip already transmitted bytes (e.g. traffic before the peer was attached).
    pub fn sync_cursor(&mut self, tx_log: &[u8]) { self.cursor = 0; self.received.clear(); let _ = self.poll(0, tx_log); self.queue.clear(); }
}

/// A decoded controller -> main frame (no mirror block).
#[derive(Clone, Debug)]
pub struct PeerFrame { pub seq: u8, pub event: u16, pub state: u16, pub status: u8, pub command: u8, pub payload: Vec<u8> }

/// Try to decode one controller -> main frame at the start of `d` (wire length L+16).
pub fn decode_peer(d: &[u8]) -> Option<Result<(PeerFrame, usize), usize>> {
    if d.len() < 6 { return None; }
    let (seq, len) = (d[0], u16::from_le_bytes([d[1], d[2]]));
    if seq == 0 || d[3] != seq ^ 0xFF || u16::from_le_bytes([d[4], d[5]]) != !len { return Some(Err(1)); }
    let total = len as usize + 16;
    if d.len() < total { return None; }
    let body = &d[6..total - 2];
    if crc16(body) != u16::from_le_bytes([d[total - 2], d[total - 1]]) { return Some(Err(1)); }
    Some(Ok((PeerFrame { seq, event: u16::from_le_bytes([body[0], body[1]]), state: u16::from_le_bytes([body[2], body[3]]), status: body[4],
        command: body[5], payload: body[8..8 + len as usize].to_vec() }, total)))
}
