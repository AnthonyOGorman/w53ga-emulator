//! SCIF (FIFO serial) channel, asynchronous 8N1 subset +
//! `Scif7ClockAdapter`. SCIF7 (0xE800_A800) carries the link to the companion MCU.
//! Clock: one ETU (bit period) every 16 x (SCBRR+1) P0 ticks (P1 = 2 x P0, CKS = 00); a byte takes 10 ETU.
//! Received bytes come from `queue_rx` (the board's peer model); transmitted bytes collect in `tx_log`.

use base64::Engine;
use serde::Deserialize;
use std::collections::VecDeque;

pub const DR: u32 = 1;
pub const RDF: u32 = 2;
pub const BRK: u32 = 16;
pub const TDFE: u32 = 32;
pub const TEND: u32 = 64;
pub const ER: u32 = 128;
pub const TE: u32 = 32;
pub const RE: u32 = 16;
pub const RIE: u32 = 64;
pub const TIE: u32 = 128;
pub const REIE: u32 = 8;
const CLEARABLE: u32 = ER | TEND | TDFE | BRK | RDF | DR;

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawSerial {
    smr: u32, brr: u32, scr: u32, fsr: u32, fcr: u32, sptr: u32, lsr: u32, emr: u32, fsr_seen: u32, lsr_seen: u32,
    rx_idle_etu: u32, rx_shift_etu: u32, tx_shift: i32, tx_shift_etu: u32, etu: u64, rx_received: u64, rx_lost: u64, tx_ignored: u64,
    rx_line_high: bool, rx: Vec<u32>, tx: Vec<u32>, pending: Vec<u32>, #[serde(default)] captured: Option<String>,
}

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawScif7 { serial: RawSerial, p0_ticks: u64, phase_p0: u64 }

/// Channel adapter (SCIF1, SCIF6): child serial core + SCEMR.ABCS in the adapter.
#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawAdapter { child: RawSerial, emr: u32, phase_p0: u64, p0_ticks: u64 }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Scif {
    pub base: u32,
    pub vectors: [u32; 4], // BRI, ERI, RXI, TXI
    smr: u32, brr: u32, scr: u32, fsr: u32, fcr: u32, sptr: u32, lsr: u32, emr: u32, fsr_seen: u32, lsr_seen: u32,
    rx_idle_etu: u32, rx_shift_etu: u32, tx_shift: i32, tx_shift_etu: u32,
    pub etu: u64, pub rx_received: u64, pub rx_lost: u64, pub tx_ignored: u64, rx_line_high: bool,
    rx: VecDeque<u32>, tx: VecDeque<u32>, pending: VecDeque<u32>,
    pub tx_log: Vec<u8>,
    pub p0_ticks: u64, phase_p0: u64,
    /// SCEMR.ABCS: 8 (ABCS=1) or 16 base clocks per bit.
    abcs: u32,
    /// Transport active: the clock only runs once SCSCR enabled TE or RE.
    pub transport_active: bool,
    /// TDFE also clears when a CPU write fills the FIFO above the trigger (the manual's DMAC clearing rule) instead of
    /// only after read-1/write-0. The firmware's SCIF7 transmit handler never acknowledges TDFE, so under the manual's
    /// rule TXI is re-requested continuously while it sends; the handler then finds the FIFO full and returns. The
    /// bytes sent and their timing are the same either way; eliding those redundant requests keeps the RTOS idle and
    /// the emulator in real time. Set by the front ends (--strict-tdfe keeps the manual's rule); off for replay.
    #[serde(skip)]
    pub tdfe_auto_clear: bool,
}

type R<T> = Result<T, String>;

impl Scif {
    pub fn scif7_from_raw(x: RawScif7, transport_active: bool) -> R<Scif> {
        Self::from_serial(x.serial, 0xE800_A800, [286, 287, 288, 289], 0, x.p0_ticks, x.phase_p0, transport_active)
    }

    pub fn from_adapter(x: RawAdapter, base: u32, vectors: [u32; 4], transport_active: bool) -> R<Scif> {
        Self::from_serial(x.child, base, vectors, x.emr, x.p0_ticks, x.phase_p0, transport_active)
    }

    fn from_serial(s: RawSerial, base: u32, vectors: [u32; 4], abcs: u32, p0_ticks: u64, phase_p0: u64, transport_active: bool) -> R<Scif> {
        let tx_log = match &s.captured { Some(c) => base64::engine::general_purpose::STANDARD.decode(c).map_err(|e| e.to_string())?, None => Vec::new() };
        Ok(Scif {
            base, vectors, abcs,
            smr: s.smr, brr: s.brr, scr: s.scr, fsr: s.fsr, fcr: s.fcr, sptr: s.sptr, lsr: s.lsr, emr: s.emr, fsr_seen: s.fsr_seen, lsr_seen: s.lsr_seen,
            rx_idle_etu: s.rx_idle_etu, rx_shift_etu: s.rx_shift_etu, tx_shift: s.tx_shift, tx_shift_etu: s.tx_shift_etu, etu: s.etu,
            rx_received: s.rx_received, rx_lost: s.rx_lost, tx_ignored: s.tx_ignored, rx_line_high: s.rx_line_high,
            rx: s.rx.into(), tx: s.tx.into(), pending: s.pending.into(), tx_log, p0_ticks, phase_p0, transport_active, tdfe_auto_clear: false,
        })
    }

    /// Power-on state ( idle-high receive line, no peer data).
    pub fn reset(base: u32, vectors: [u32; 4]) -> Scif {
        Scif { base, vectors, abcs: 0, smr: 0, brr: 255, scr: 0, fsr: TEND | TDFE, fcr: 0, sptr: 0x50, lsr: 0, emr: 0, fsr_seen: 0, lsr_seen: 0,
            rx_idle_etu: 0, rx_shift_etu: 0, tx_shift: -1, tx_shift_etu: 0, etu: 0, rx_received: 0, rx_lost: 0, tx_ignored: 0, rx_line_high: true,
            rx: VecDeque::new(), tx: VecDeque::new(), pending: VecDeque::new(), tx_log: Vec::new(), p0_ticks: 0, phase_p0: 0, transport_active: false, tdfe_auto_clear: false }
    }

    pub fn handles(&self, a: u32, n: usize) -> bool { n > 0 && a >= self.base && a + n as u32 <= self.base + 0x2A }
    fn rtrg(&self) -> usize { [1, 4, 8, 14][((self.fcr >> 6) & 3) as usize] }
    fn ttrg(&self) -> usize { [8, 4, 2, 0][((self.fcr >> 4) & 3) as usize] }
    fn levels(&mut self) {
        if self.rx.len() >= self.rtrg() { self.fsr |= RDF; }
        if self.tx.len() <= self.ttrg() { self.fsr |= TDFE; }
        if self.scr & TE == 0 { self.fsr |= TEND; }
    }
    fn offset(&self, a: u32, n: usize) -> R<u32> {
        if !self.handles(a, n) { return Err(format!("SCIF address {a:08x}")); }
        let off = a - self.base;
        if off % 4 != 0 { return Err("unaligned SCIF register".into()); }
        let want = if matches!(off, 4 | 12 | 20) { 1 } else { 2 };
        if n != want { return Err(format!("SCIF register width +{off:x}/{n}")); }
        Ok(off)
    }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        let off = self.offset(a, n)?;
        self.levels();
        let v = match off {
            0 => self.smr, 4 => self.brr, 8 => self.scr,
            12 => return Err("SCFTDR is write-only".into()),
            16 => { self.fsr_seen |= self.fsr & CLEARABLE; self.fsr }
            20 => self.rx.pop_front().ok_or("empty SCFRDR read is undefined")?,
            24 => self.fcr,
            28 => ((self.tx.len() as u32) << 8) | self.rx.len() as u32,
            32 => (self.sptr & !1) | self.rx_line_high as u32,
            36 => { self.lsr_seen |= self.lsr; self.lsr }
            40 => self.abcs,
            _ => return Err(format!("SCIF register read +{off:x}")),
        };
        Ok(v as u64)
    }

    fn quiet(&self) -> bool { self.rx.is_empty() && self.tx.is_empty() && self.pending.is_empty() && self.tx_shift < 0 && self.rx_shift_etu == 0 }

    pub fn write(&mut self, a: u32, n: usize, value: u64) -> R<()> {
        let off = self.offset(a, n)?;
        let v = value as u32;
        match off {
            0 => { if v != 0 { return Err("only asynchronous SCSMR=0".into()); } self.smr = v; }
            4 => {
                if self.scr & (TE | RE) != 0 { return Err("BRR change requires TE=RE=0".into()); }
                if !self.quiet() { return Err("BRR change requires empty FIFOs".into()); }
                self.brr = v; self.phase_p0 = 0;
            }
            8 => {
                if v & !0xF8 != 0 { return Err("SCSCR clock/reserved bits".into()); }
                if self.scr & TE != 0 && v & TE == 0 { self.tx_shift = -1; self.tx_shift_etu = 0; self.fsr |= TEND; }
                self.scr = v;
                self.transport_active = v & 0x30 != 0;
            }
            12 => {
                if self.fcr & 4 != 0 || self.tx.len() == 16 { self.tx_ignored += 1; } else { self.tx.push_back(v & 0xFF); }
                if self.tdfe_auto_clear && self.tx.len() > self.ttrg() { self.fsr &= !TDFE; }
            }
            16 => {
                let mut clear = self.fsr_seen & !v & CLEARABLE;
                if self.rx.len() >= self.rtrg() { clear &= !RDF; }
                if !self.rx.is_empty() { clear &= !DR; }
                if self.tx.len() <= self.ttrg() { clear &= !TDFE; }
                if self.tx.is_empty() && self.tx_shift < 0 { clear &= !TEND; }
                self.fsr &= !clear; self.fsr_seen = 0;
            }
            20 => return Err("SCFRDR is read-only".into()),
            24 => {
                if v & !0x7F6 != 0 { return Err("SCFCR loopback/modem modes".into()); }
                self.fcr = v;
                if v & 2 != 0 { self.rx.clear(); self.rx_idle_etu = 0; }
                if v & 4 != 0 { self.tx.clear(); }
            }
            28 => return Err("SCFDR is read-only".into()),
            32 => {
                if v & 0xFFFC != 0x0050 { return Err("SCSPTR must preserve reset latches".into()); }
                if v & 2 != 0 && self.scr & TE != 0 { return Err("break output requires TE=0".into()); }
                self.sptr = v;
            }
            36 => { self.lsr &= !(self.lsr_seen & !v & 1); self.lsr_seen = 0; }
            40 => {
                if v > 1 { return Err("only SCEMR BGDM=0, ABCS=0/1".into()); }
                if !self.quiet() || self.scr & (TE | RE) != 0 { return Err("SCEMR change requires a quiet disabled channel".into()); }
                self.abcs = v; self.emr = 0; self.phase_p0 = 0;
            }
            _ => return Err(format!("SCIF register write +{off:x}")),
        }
        self.levels();
        Ok(())
    }

    /// Bytes from the peer (companion MCU) enter the receive shifter in order.
    /// Bytes arriving on RxD. While the channel has never been enabled (no TE/RE yet) nothing samples the line, so
    /// they are lost, as when the peer starts talking before this side has set up its UART.
    pub fn queue_rx(&mut self, bytes: &[u8]) {
        if !self.transport_active { self.rx_lost += bytes.len() as u64; return; }
        self.pending.extend(bytes.iter().map(|&b| b as u32));
    }
    pub fn rx_pending(&self) -> usize { self.pending.len() }

    fn receive(&mut self, v: u32) {
        self.rx_idle_etu = 0;
        if self.scr & RE == 0 || self.fcr & 2 != 0 || self.lsr & 1 != 0 { self.rx_lost += 1; return; }
        if self.rx.len() == 16 { self.lsr |= 1; self.rx_lost += 1; return; }
        self.rx.push_back(v); self.rx_received += 1; self.levels();
    }

    fn advance_etu(&mut self, count: u64) {
        for _ in 0..count {
            self.etu += 1;
            if !self.pending.is_empty() {
                self.rx_shift_etu += 1;
                if self.rx_shift_etu == 10 { self.rx_shift_etu = 0; let b = self.pending.pop_front().unwrap(); self.receive(b); }
            } else if !self.rx.is_empty() {
                self.rx_idle_etu += 1;
                if self.rx_idle_etu >= 15 && self.rx.len() < self.rtrg() { self.fsr |= DR; }
            }
            if self.scr & TE != 0 {
                if self.tx_shift < 0 { if let Some(b) = self.tx.pop_front() { self.tx_shift = b as i32; self.tx_shift_etu = 0; } }
                if self.tx_shift >= 0 {
                    self.tx_shift_etu += 1;
                    if self.tx_shift_etu == 10 { self.tx_log.push(self.tx_shift as u8); self.tx_shift = -1; self.tx_shift_etu = 0; if self.tx.is_empty() { self.fsr |= TEND; } }
                }
            }
            self.levels();
        }
    }

    pub fn advance_p0(&mut self, ticks: u64) {
        if !self.transport_active { return; }
        self.p0_ticks += ticks;
        let period = if self.abcs == 1 { 8 } else { 16 } * (self.brr as u64 + 1);
        self.phase_p0 += ticks;
        if self.phase_p0 >= period {
            let due = self.phase_p0 / period;
            self.phase_p0 %= period;
            self.advance_etu(due);
        }
    }

    pub fn debug(&self) -> String {
        format!("scr {:02x} fsr {:04x} fcr {:04x} brr {} tx {} rx {} pending {} active {}", self.scr, self.fsr, self.fcr, self.brr, self.tx.len(), self.rx.len(), self.pending.len(), self.transport_active)
    }

    /// TXI request level (TIE and TDFE): also the DMA transmit request.
    pub fn txi_level(&self) -> bool { self.scr & TIE != 0 && self.fsr & TDFE != 0 }

    /// DMAC-activated SCFTDR byte write: TDFE clears once the FIFO holds more than the trigger count.
    pub fn dma_write_tdr(&mut self, v: u8) -> R<()> {
        if self.scr & TE == 0 || !self.txi_level() || self.fcr & 4 != 0 || self.tx.len() == 16 { return Err("DMA write to SCFTDR without TXI request".into()); }
        self.tx.push_back(v as u32);
        if self.tx.len() > self.ttrg() { self.fsr &= !TDFE; }
        Ok(())
    }

    /// Requested vectors in selection order (BRI, ERI, RXI, TXI).
    pub fn requests(&self) -> [Option<u32>; 4] {
        let (fsr, scr) = (self.fsr, self.scr);
        let mut out = [None; 4];
        if scr & (RIE | REIE) != 0 {
            if fsr & BRK != 0 || self.lsr & 1 != 0 { out[0] = Some(self.vectors[0]); }
            if fsr & ER != 0 { out[1] = Some(self.vectors[1]); }
        }
        if scr & RIE != 0 && fsr & (RDF | DR) != 0 { out[2] = Some(self.vectors[2]); }
        if scr & TIE != 0 && fsr & TDFE != 0 { out[3] = Some(self.vectors[3]); }
        out
    }
}
