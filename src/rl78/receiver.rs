//! The controller board's receivers as the controller sees them.
//!
//! Radio: a PLL synthesizer programmed with 24-bit words over three port pins (P2.3 clock, P2.4 data, P2.5 latch; the
//! controller sends a setup word (reference divider, top byte 05) and then a channel word, and raises P2.6 once the
//! receiver is tuned). Its signal level is read on ANI1.
//!
//! Radar: the microwave oscillator is swept by a 10-bit DAC on port pins (bits 0-1 on P0.5/P0.6, bits 2-9 on P7,
//! written together by the controller), P3.0 powers the front end, a comparator on P3.1 goes low while the detector
//! output is above its threshold, and the detector output itself is read on ANI0. The controller sweeps until the
//! comparator fires, scans around that point for the peak and measures it above the noise floor it calibrated.
//! A radar signal shows as a peak at one sweep position (`RADAR_AT`), its height set by the radar's strength.

/// Sweep position of the emulated radar signal, its width (steps) and height at full strength (fraction of the
/// A/D range above the noise floor), and the comparator threshold above the floor.
const RADAR_AT: f64 = 340.0;
const RADAR_WIDTH: f64 = 4.0;
const RADAR_PEAK: f64 = 0.45;
const COMPARATOR: f64 = 0.03;

#[derive(Default, Clone, serde::Serialize, serde::Deserialize)]
pub struct Receiver {
    shift: u32,
    /// Last setup word (reference divider) and channel word latched into the synthesizer.
    pub setup: u32,
    pub channel: u32,
    /// P2.6: set by the controller after tuning (receiver enabled / measurement window).
    pub enabled: bool,
    /// Radar sweep DAC output (0-1023).
    pub sweep: u16,
    /// Strength of the radar signal at the antenna (0-1), set from the radar scene.
    #[serde(default)]
    pub radar: f64,
    #[serde(skip)]
    log: Option<bool>,
}

impl Receiver {
    pub fn logging(&mut self) -> bool { *self.log.get_or_insert_with(|| std::env::var_os("AR_RX_LOG").is_some()) }

    /// A write to port 2's output latch (old and new levels).
    pub fn port2(&mut self, now: u64, old: u8, new: u8) {
        let rose = |b: u8| new & b != 0 && old & b == 0;
        if rose(0x08) { self.shift = ((self.shift << 1) | ((new >> 4) & 1) as u32) & 0xFF_FFFF; }
        if rose(0x20) {
            let w = self.shift;
            if w >> 16 == 0x05 { self.setup = w; } else { self.channel = w; }
            if self.logging() { eprintln!("[rx] {now:>12} latch {w:06x}"); }
        }
        if (new ^ old) & 0x40 != 0 {
            self.enabled = new & 0x40 != 0;
            if self.logging() { eprintln!("[rx] {now:>12} P2.6 {} (setup {:06x} channel {:06x})", self.enabled as u8, self.setup, self.channel); }
        }
    }

    /// Radar detector output above the noise floor (fraction of the A/D range) at the current sweep position, with
    /// the front end powered (P3.0) or not.
    pub fn detector(&self, powered: bool) -> f64 {
        if !powered || self.radar <= 0.0 { return 0.0; }
        let d = (self.sweep as f64 - RADAR_AT) / RADAR_WIDTH;
        self.radar * RADAR_PEAK * (-d * d).exp()
    }

    /// The comparator output on P3.1 (low while the detector output is above its threshold).
    pub fn comparator_low(&self, powered: bool) -> bool { self.detector(powered) > COMPARATOR }

    /// Port 0 and port 7 output latches changed: the radar sweep DAC follows P0.5/P0.6 and P7.
    pub fn sweep_pins(&mut self, p0: u8, p7: u8) { self.sweep = ((p7 as u16) << 2) | ((p0 >> 5) & 3) as u16; }
}
