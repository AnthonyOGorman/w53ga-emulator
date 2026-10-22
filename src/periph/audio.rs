//! Audio path: sampling rate converter SRC0 (0xFFFE_7000) and serial sound interface SSI0 (0xFFFF_0000),
//! written from the SH7268/69 manual (sections 42 and 40).
//!
//! SRC0: 8-entry input FIFO, 16-entry output FIFO, IINT/OINT/UDF/OVF/FLF/CEF flags, IDEI/ODFI DMA requests and
//! interrupts. Conversion is functional linear interpolation at the IFS->OFS ratio (not the hardware FIR); the
//! number of output units after a flush follows the manual's formula (table 42.4).
//! SSI0: 8-entry transmit FIFO of 32-bit entries (upper 16 bits then lower 16 bits = one stereo frame in the
//! firmware's 16-bit stereo format), TDE request/flag, underflow (TUIRQ), SSIF0 interrupt level. The serial clock
//! is modelled as one frame per `p0_per_frame` P0 ticks (fs from the SRC output rate, default 44.1 kHz).

pub const SRC_BASE: u32 = 0xFFFE_7000;
pub const SSI_BASE: u32 = 0xFFFF_0000;
const P0_HZ: f64 = 100.0e6 / 3.0;

fn ifs_hz(ifs: u32) -> Option<f64> { Some(match ifs { 0 => 8000.0, 1 => 11025.0, 2 => 12000.0, 4 => 16000.0, 5 => 22050.0, 6 => 24000.0, 8 => 32000.0, 9 => 44100.0, 10 => 48000.0, _ => return None }) }
fn ofs_hz(ofs: u32) -> Option<f64> { Some(match ofs { 0 => 44100.0, 1 => 48000.0, 2 => 32000.0, 4 => 8000.0, 5 => 16000.0, _ => return None }) }
/// Table 42.4 n for (OFS, IFS).
fn table_n(ofs: u32, ifs: u32) -> u32 {
    let row: [u32; 9] = match ofs { 0 => [38, 40, 40, 43, 48, 48, 43, 0, 63], 1 => [38, 40, 40, 43, 48, 48, 43, 32, 0], 2 => [40, 37, 40, 48, 40, 48, 0, 48, 63], _ => [0, 0, 0, 0, 0, 0, 0, 63, 0] };
    let col = match ifs { 0 => 0, 1 => 1, 2 => 2, 4 => 3, 5 => 4, 6 => 5, 8 => 6, 9 => 7, 10 => 8, _ => return 1 };
    row[col].max(1)
}

#[derive(Default, serde::Serialize, serde::Deserialize)]
pub struct Src {
    idctrl: u32, odctrl: u32, ctrl: u32,
    flags: u32,       // CEF(5) UDF(3) OVF(2) IINT(1) OINT(0); FLF kept separately
    seen: u32,
    flf: bool,
    pub inp: std::collections::VecDeque<(i16, i16)>,
    pub out: std::collections::VecDeque<(i16, i16)>,
    last_out: u32,
    phase: f64,
    a: Option<(i16, i16)>,
    b: Option<(i16, i16)>,
    units_in: u64, units_out: u64, flush_target: Option<u64>,
    pub converted: u64,
    /// A tick while quiescent has run and nothing has touched the converter since (`tick_quiescent`).
    #[serde(skip)]
    settled: bool,
}

type R<T> = Result<T, String>;

impl Src {
    pub fn new() -> Src { Src { flags: 2, ..Default::default() } }
    pub fn handles(a: u32, n: usize) -> bool { n > 0 && a >= SRC_BASE && a + n as u32 <= SRC_BASE + 0x10 }
    pub fn enabled(&self) -> bool { self.ctrl & 0x1000 != 0 }
    fn ifs(&self) -> u32 { (self.ctrl >> 4) & 15 }
    fn ofs(&self) -> u32 { self.ctrl & 7 }
    pub fn out_rate(&self) -> f64 { ofs_hz(self.ofs()).unwrap_or(44100.0) }
    fn iftrg(&self) -> usize { [0, 2, 4, 6][(self.idctrl & 3) as usize] }
    fn oftrg(&self) -> usize { [1, 4, 8, 12][(self.odctrl & 3) as usize] }
    fn clear_all(&mut self) {
        self.inp.clear(); self.out.clear(); self.flags = 2; self.seen = 0; self.flf = false;
        self.phase = 0.0; self.a = None; self.b = None; self.units_in = 0; self.units_out = 0; self.flush_target = None;
    }
    fn levels(&mut self) {
        if self.inp.len() <= self.iftrg() { self.flags |= 2; }
        if self.out.len() >= self.oftrg() { self.flags |= 1; }
    }
    pub fn status(&self) -> u32 {
        ((self.out.len() as u32 & 31) << 11) | ((self.inp.len() as u32 & 15) << 7) | (self.flags & 0x2F) | if self.flf { 0x10 } else { 0 }
    }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        self.settled = false;
        let o = a - SRC_BASE;
        Ok(match (o, n) {
            (0, 4) => 0, (0, 2) | (2, 2) => 0,
            (4, 4) => self.read_od() as u64,
            (4, 2) => (self.read_od() >> 16) as u64,
            (8, 2) => self.idctrl as u64,
            (0xA, 2) => self.odctrl as u64,
            (0xC, 2) => (self.ctrl & 0x3CF7) as u64,
            (0xE, 2) => { let s = self.status(); self.seen |= s & 0x2F; s as u64 }
            _ => return Err(format!("SRC access {a:08x}/{n}")),
        })
    }

    fn read_od(&mut self) -> u32 {
        match self.out.pop_front() {
            Some((l, r)) => {
                let (mut hi, mut lo) = (l as u16 as u32, r as u16 as u32);
                if self.odctrl & 0x400 != 0 { std::mem::swap(&mut hi, &mut lo); }
                let mut v = (hi << 16) | lo;
                if self.odctrl & 0x200 != 0 { v = ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF); }
                self.last_out = v;
                if self.ctrl & 0x400 == 0 { self.flags &= !4; } // OVF auto-clears when OVEN=0 and space appears
                if self.out.len() < self.oftrg() { self.flags &= !1; }
                self.check_end();
                v
            }
            None => { if self.enabled() { self.flags |= 8; } self.last_out }
        }
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> R<()> {
        self.settled = false;
        let o = a - SRC_BASE;
        let v = v as u32;
        match (o, n) {
            (0, 4) | (0, 2) => self.write_id(if n == 2 { v << 16 } else { v }),
            (8, 2) => { if self.enabled() && (v ^ self.idctrl) & 0x200 != 0 { return Err("IED change while SRCEN=1".into()); } self.idctrl = v & 0x303; }
            (0xA, 2) => { if self.enabled() && (v ^ self.odctrl) & 0x600 != 0 { return Err("OCH/OED change while SRCEN=1".into()); } self.odctrl = v & 0x703; }
            (0xC, 2) => {
                let was = self.enabled();
                if v & 0x100 != 0 { self.clear_all(); }
                let new_ctrl = v & 0x3CF7;
                if !was && new_ctrl & 0x1000 != 0 {
                    if ifs_hz((new_ctrl >> 4) & 15).is_none() || ofs_hz(new_ctrl & 7).is_none() { return Err(format!("SRC prohibited rate setting {new_ctrl:04x}")); }
                    self.clear_all();
                }
                if was && new_ctrl & 0x1000 != 0 && (new_ctrl ^ self.ctrl) & 0xF7 != 0 { return Err("IFS/OFS change while SRCEN=1".into()); }
                self.ctrl = new_ctrl;
                if v & 0x200 != 0 && self.enabled() && self.flush_target.is_none() {
                    let (ifs, ofs) = (self.ifs(), self.ofs());
                    let n = table_n(ofs, ifs) as f64;
                    let (fi, fo) = (ifs_hz(ifs).unwrap(), ofs_hz(ofs).unwrap());
                    let total_in = self.units_in as f64;
                    let target = if total_in > 0.0 { (((total_in * n - 1.0) * fo / (fi * n)).floor() + 1.0) as u64 } else { 0 };
                    self.flush_target = Some(target.max(self.units_out));
                    self.flf = true;
                }
            }
            (0xE, 2) => { let clear = self.seen & !v & 0x2F; self.flags &= !clear; self.seen = 0; self.levels(); }
            _ => return Err(format!("SRC write {a:08x}/{n}")),
        }
        Ok(())
    }

    fn write_id(&mut self, mut v: u32) {
        if self.inp.len() >= 8 { return; }
        if self.idctrl & 0x200 != 0 { v = ((v & 0x00FF_00FF) << 8) | ((v >> 8) & 0x00FF_00FF); }
        self.inp.push_back(((v >> 16) as i16, v as i16));
        self.units_in += 1;
        if self.inp.len() > self.iftrg() { self.flags &= !2; }
    }

    fn check_end(&mut self) {
        if let Some(t) = self.flush_target {
            if self.units_out >= t && self.inp.is_empty() {
                self.flf = false;
                if self.out.is_empty() { self.flags |= 0x20; self.flush_target = None; }
            }
        }
    }

    /// Convert available input into output: linear interpolation between samples `a` and `b` at `phase` in [0,1),
    /// advancing by IFS/OFS per output (functional resampler; runs faster than audio rate, as the hardware does).
    /// No input, no sample in conversion and no flush: a tick only re-derives flags, so one tick stands for many.
    pub fn quiescent(&self) -> bool { self.inp.is_empty() && self.b.is_none() && self.flush_target.is_none() }

    pub fn tick(&mut self) {
        self.settled = false;
        if !self.enabled() { return; }
        let (fi, fo) = (ifs_hz(self.ifs()).unwrap_or(44100.0), ofs_hz(self.ofs()).unwrap_or(44100.0));
        let step = fi / fo;
        for _ in 0..2 {
            if let Some(t) = self.flush_target { if self.units_out >= t { self.inp.clear(); self.check_end(); break; } }
            if self.out.len() >= 16 {
                if self.flags & 4 == 0 && (!self.inp.is_empty() || self.flush_target.is_some()) { self.flags |= 4; }
                break;
            }
            if self.ctrl & 0x400 != 0 && self.flags & 4 != 0 { break; }
            if self.a.is_none() { self.a = self.inp.pop_front(); }
            let Some(a) = self.a else { break };
            if self.b.is_none() { self.b = self.inp.pop_front().or(if self.flush_target.is_some() { Some(a) } else { None }); }
            let Some(b) = self.b else { break };
            let f = self.phase;
            let lerp = |x: i16, y: i16| (x as f64 + (y as f64 - x as f64) * f).round().clamp(-32768.0, 32767.0) as i16;
            self.out.push_back((lerp(a.0, b.0), lerp(a.1, b.1)));
            self.units_out += 1;
            self.converted += 1;
            self.phase += step;
            while self.phase >= 1.0 { self.phase -= 1.0; self.a = self.b; self.b = self.inp.pop_front(); }
        }
        if self.inp.len() <= self.iftrg() { self.flags |= 2; }
        self.levels();
        self.check_end();
    }

    /// A tick while `quiescent`. Such a tick only sets the IINT/OINT flags from the FIFO levels, so once it has run, the
    /// next one changes nothing until the converter is read or written: it is then not repeated.
    pub fn tick_quiescent(&mut self) {
        if self.settled { return; }
        self.tick();
        self.settled = true;
    }

    /// DMA request lines: IDEI (0x41: IEN && IINT), ODFI (0x42: OEN && OINT).
    pub fn idei(&self) -> bool { self.enabled() && self.idctrl & 0x100 != 0 && self.flags & 2 != 0 }
    pub fn odfi(&self) -> bool { self.enabled() && self.odctrl & 0x100 != 0 && self.flags & 1 != 0 }
    /// Interrupt requests (vector, pending) for OVF0..IDEI0 = 341..345.
    pub fn irqs(&self) -> [(u32, bool); 5] {
        let c = self.ctrl;
        [(341, c & 0x400 != 0 && self.flags & 4 != 0), (342, c & 0x800 != 0 && self.flags & 8 != 0), (343, c & 0x2000 != 0 && self.flags & 0x20 != 0),
         (344, self.odfi()), (345, self.idei())]
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Ssi {
    cr: u32, fcr: u32, tdmr: u32,
    sr_flags: u32, sr_seen: u32,
    idst: bool, iirq: bool,
    fifo: std::collections::VecDeque<u32>,
    tde: bool, tde_seen: bool,
    phase: f64,
    pub p0_per_frame: f64,
    /// Played stereo frames (L, R); drained by the host audio output.
    pub played: std::collections::VecDeque<(i16, i16)>,
    pub frames: u64, pub underflows: u64,
    /// Optional full capture of played frames (for WAV dumps).
    pub capture: Option<Vec<(i16, i16)>>,
}

impl Ssi {
    /// State seeded from a start state (`ssiTx`: SSICR, SSIFCR, TEN running, FIFO depth).
    pub fn new(cr: u32, fcr: u32, fifo_len: usize, ten_running: bool) -> Ssi {
        let mut s = Ssi { cr, fcr, tdmr: 0, sr_flags: 0, sr_seen: 0, idst: !ten_running, iirq: !ten_running, fifo: std::iter::repeat_n(0, fifo_len).collect(),
            tde: false, tde_seen: false, phase: 0.0, p0_per_frame: P0_HZ / 44100.0, played: Default::default(), frames: 0, underflows: 0, capture: None };
        s.levels();
        s
    }
    pub fn handles(a: u32, n: usize) -> bool { n > 0 && a >= SSI_BASE && a + n as u32 <= SSI_BASE + 0x24 }
    fn ten(&self) -> bool { self.cr & 2 != 0 }
    fn threshold(&self) -> usize { [7, 6, 4, 2][((self.fcr >> 6) & 3) as usize] }
    fn levels(&mut self) { if self.fifo.len() <= self.threshold() { self.tde = true; } }
    fn status(&self) -> u32 { self.sr_flags | 2 | if self.iirq { 0x0200_0000 } else { 0 } | if self.idst { 1 } else { 0 } }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        if n != 4 { return Err(format!("SSI access {a:08x}/{n}")); }
        Ok(match a - SSI_BASE {
            0x00 => self.cr as u64,
            0x04 => { self.sr_seen |= self.sr_flags; self.status() as u64 }
            0x10 => self.fcr as u64,
            0x14 => { if self.tde { self.tde_seen = true; } (((self.fifo.len() as u32 & 15) << 24) | if self.tde { 0x1_0000 } else { 0 }) as u64 }
            0x1C => 0,
            0x20 => self.tdmr as u64,
            o => return Err(format!("SSI read +{o:x}")),
        })
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> R<()> {
        if n != 4 { return Err(format!("SSI access {a:08x}/{n}")); }
        let v = v as u32;
        match a - SSI_BASE {
            0x00 => {
                if v & 1 != 0 { return Err("SSI receive (REN) not modelled".into()); }
                let (was, now) = (self.ten(), v & 2 != 0);
                self.cr = v;
                if !was && now { self.idst = false; self.iirq = false; }
                if was && !now { self.idst = true; self.iirq = true; }
            }
            0x04 => { let clear = self.sr_seen & !v & 0x3C00_0000; self.sr_flags &= !clear; self.sr_seen = 0; }
            0x10 => { if v & 2 != 0 { self.fifo.clear(); } self.fcr = v & 0xFF; self.levels(); }
            0x14 => { if self.tde_seen && v & 0x1_0000 == 0 && self.fifo.len() > self.threshold() { self.tde = false; } self.tde_seen = false; }
            0x18 => self.push(v),
            0x20 => self.tdmr = v,
            o => return Err(format!("SSI write +{o:x}")),
        }
        Ok(())
    }

    /// SSIFTDR write (CPU or DMA): TDE clears once the FIFO holds more than the trigger count.
    pub fn push(&mut self, v: u32) {
        if self.fifo.len() < 8 { self.fifo.push_back(v); }
        if self.fifo.len() > self.threshold() { self.tde = false; }
    }

    pub fn tick(&mut self, p0: u64) {
        if !self.ten() { return; }
        self.phase += p0 as f64;
        if self.phase < self.p0_per_frame { return; }
        self.phase -= self.p0_per_frame;
        let frame = match self.fifo.pop_front() {
            Some(w) => if self.cr & 8 == 0 { ((w >> 16) as i16, w as i16) } else { (0, 0) },
            None => { self.underflows += 1; self.sr_flags |= 0x2000_0000; (0, 0) }
        };
        self.played.push_back(frame);
        if let Some(c) = &mut self.capture { c.push(frame); }
        if self.played.len() > 1 << 16 { self.played.pop_front(); }
        self.frames += 1;
        self.levels();
    }

    /// DMA request (selector 0x21): TEN && TIE && TDE.
    pub fn tx_request(&self) -> bool { self.ten() && self.fcr & 8 != 0 && self.tde }
    /// SSIF0 (vector 224) level: enabled error/idle flags.
    pub fn irq(&self) -> bool {
        let s = self.status();
        (s & self.cr & 0x3C00_0000) != 0 || (self.iirq && self.cr & 0x0200_0000 != 0)
    }
}
