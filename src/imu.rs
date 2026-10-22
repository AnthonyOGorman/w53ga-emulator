//! Measurement model of the unit's inertial sensor (MPU-6500 compatible, IIC2 7-bit address 0x69): what the chip in
//! a car-mounted unit measures while the emulated vehicle drives (`crate::gnss`), written into its output registers
//! (0x3B..0x40 accelerometer, 0x43..0x48 gyro, big-endian) at the sample rate the program configures.
//!
//! Axes, as the unit's program reads the chip: the board stands upright facing the driver, so the chip's Y axis points
//! down, Z backwards and X to the left. The program takes -Y as vertical (its display-rotation check expects about
//! +1 g there), -Z as longitudinal (integrated into a speed and calibrated against the GPS speed, positive when
//! accelerating) and -X as lateral, and the gyro's Y rate as yaw (positive when turning right, integrated into the
//! course it dead-reckons with). Scaling follows ACCEL_CONFIG / GYRO_CONFIG (the program sets ±2 g and ±250 °/s) and
//! the sample rate SMPLRT_DIV / CONFIG (the program sets 20 Hz).
//!
//! Besides gravity, speed changes, cornering and grade, the vertical axis carries road vibration growing with the
//! speed: the program estimates the speed from it while the GPS has no fix, and takes a car without vibration for
//! parked. The vibration is high-passed noise (successive samples differ, their averages hardly), scaled so that the
//! program's vibration speed estimate comes out near the true speed.
//!
//! Diagnostics (environment): AR_IMU=off writes no samples (the power-on sample stays), AR_IMU=legacy writes the
//! stationary sample this emulator used before (+1 g on Z) once; AR_IMU_VIB=IDLE,PER_KMH sets the vibration (mg).

use crate::gnss::{Gnss, Motion};

const P0_HZ: f64 = 100.0e6 / 3.0;
const G: f64 = 9.80665;
/// Road vibration, standard deviation of the high-passed vertical acceleration (mg): idle (engine running) + per km/h.
/// Calibrated against the unit's program: with this gain its vibration speed estimate (which bounds its dead-reckoned
/// speed to [estimate - 10, estimate + 20] km/h without a fix) comes out at about 0.85 to 1.1 times the true speed
/// from 20 to 100 km/h.
const VIB_IDLE_MG: f64 = 0.8;
const VIB_PER_KMH_MG: f64 = 1.0;
/// Share of that vibration on the longitudinal and lateral axes.
const VIB_OTHER: f64 = 0.3;
/// Gyro noise (°/s): base + per km/h.
const GYRO_NOISE: f64 = 0.03;
const GYRO_NOISE_PER_KMH: f64 = 0.002;

#[derive(Clone, Default)]
pub struct Imu {
    /// P0 time of the next sample.
    next: u64,
    /// Previous sample time and motion (rates are differences between samples).
    prev: Option<(u64, Motion)>,
    rng: u64,
    /// Previous white-noise samples (vertical, longitudinal, lateral) of the high-passed vibration.
    hp: [f64; 3],
    /// Vibration (idle mg, mg per km/h); AR_IMU_VIB=IDLE,PER_KMH overrides it (calibration).
    vib: Option<(f64, f64)>,
    /// AR_IMU: 0 = motion, 1 = off, 2 = legacy stationary sample.
    mode: Option<u8>,
}

/// Output sample period (P0 clocks) from SMPLRT_DIV and the DLPF setting in CONFIG.
fn period(regs: &[u8; 128]) -> u64 {
    let dlpf = regs[0x1A] & 7;
    let base = if dlpf == 0 || dlpf == 7 { 8000.0 } else { 1000.0 };
    (P0_HZ / (base / (1.0 + regs[0x19] as f64))) as u64
}

fn wrap180(a: f64) -> f64 { (a + 180.0).rem_euclid(360.0) - 180.0 }

impl Imu {
    fn uniform(&mut self) -> f64 {
        if self.rng == 0 { self.rng = 0x9E37_79B9_7F4A_7C15; }
        self.rng ^= self.rng >> 12; self.rng ^= self.rng << 25; self.rng ^= self.rng >> 27;
        ((self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 { let (u, v) = (self.uniform(), self.uniform()); (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos() }

    /// Whether a new sample is due at `now` (P0 clocks).
    #[inline]
    pub fn due(&self, now: u64) -> bool { now >= self.next }

    /// Write the sample due at `now` into the chip's registers: the vehicle's motion from `gnss`, or a car standing
    /// with its engine running.
    pub fn update(&mut self, now: u64, regs: &mut [u8; 128], gnss: Option<&mut Gnss>) {
        if now < self.next { return; }
        let mode = *self.mode.get_or_insert_with(|| match std::env::var("AR_IMU").as_deref() { Ok("off") => 1, Ok("legacy") => 2, _ => 0 });
        if mode != 0 {
            self.next = u64::MAX;
            if mode == 2 { regs[0x3B..0x49].copy_from_slice(&[0, 0, 0, 0, 0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0]); }
            return;
        }
        self.next = now + period(regs).max(1);
        let (vib_idle, vib_kmh) = *self.vib.get_or_insert_with(|| std::env::var("AR_IMU_VIB").ok()
            .and_then(|v| { let (a, b) = v.split_once(',')?; Some((a.trim().parse().ok()?, b.trim().parse().ok()?)) })
            .unwrap_or((VIB_IDLE_MG, VIB_PER_KMH_MG)));
        let m = gnss.map(|g| g.motion(now)).unwrap_or_default();
        // rates from the previous sample (none after a gap, e.g. a resumed state)
        let (accel, yaw, pitch_rate) = match self.prev {
            Some((t0, p)) if now > t0 && now - t0 < (P0_HZ * 0.5) as u64 => {
                let dt = (now - t0) as f64 / P0_HZ;
                ((m.speed - p.speed) / dt, wrap180(m.heading - p.heading) / dt, (m.pitch - p.pitch) / dt)
            }
            _ => (0.0, 0.0, 0.0),
        };
        self.prev = Some((now, m));
        let kmh = m.speed * 3.6;
        let vib = vib_idle + vib_kmh * kmh;
        let n = [self.gauss(), self.gauss(), self.gauss()];
        let hp: Vec<f64> = (0..3).map(|k| (n[k] - self.hp[k]) / std::f64::consts::SQRT_2).collect();
        self.hp = n;
        // specific force in the vehicle's axes (mg): forward, right, up
        let mg = 1000.0 / G;
        let fwd = (accel + G * m.pitch.sin()) * mg + VIB_OTHER * vib * hp[1];
        let right = m.speed * yaw.to_radians() * mg + VIB_OTHER * vib * hp[2];
        let up = 1000.0 * m.pitch.cos() + vib * hp[0];
        let gn = GYRO_NOISE + GYRO_NOISE_PER_KMH * kmh;
        let (gx, gy, gz) = (-pitch_rate.to_degrees() + gn * self.gauss(), yaw + gn * self.gauss(), gn * self.gauss());
        // chip axes: X left, Y down, Z backwards
        let acc_lsb = (16384 >> ((regs[0x1C] >> 3) & 3)) as f64 / 1000.0;
        let gyro_lsb = 131.072 / (1 << ((regs[0x1B] >> 3) & 3)) as f64;
        let word = |v: f64| ((v.round().clamp(-32768.0, 32767.0) as i16) as u16).to_be_bytes();
        for (k, v) in [-right * acc_lsb, -up * acc_lsb, -fwd * acc_lsb].into_iter().enumerate() { regs[0x3B + 2 * k..0x3D + 2 * k].copy_from_slice(&word(v)); }
        for (k, v) in [gx * gyro_lsb, gy * gyro_lsb, gz * gyro_lsb].into_iter().enumerate() { regs[0x43 + 2 * k..0x45 + 2 * k].copy_from_slice(&word(v)); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn word(r: &[u8; 128], a: usize) -> i16 { i16::from_be_bytes([r[a], r[a + 1]]) }

    #[test]
    fn standing_level() {
        let mut r = [0u8; 128];
        r[0x19] = 0x31; r[0x1A] = 5; // 20 Hz as the unit's program sets it
        let mut imu = Imu::default();
        imu.update(0, &mut r, None);
        assert!((word(&r, 0x3D) + 16384).abs() < 50, "Y = {}", word(&r, 0x3D)); // -1 g on Y (pointing down)
        assert!(word(&r, 0x3B).abs() < 20 && word(&r, 0x3F).abs() < 20);
        assert!(!imu.due(1) && imu.due((P0_HZ / 20.0) as u64));
    }

    #[test]
    fn turning_right_reads_positive_yaw() {
        let mut g = Gnss::new(vec![(35.0, 135.0), (35.009, 135.0), (35.009, 135.011)], 36.0);
        g.start_after = 0;
        let mut r = [0u8; 128];
        r[0x19] = 0x31; r[0x1A] = 5;
        let mut imu = Imu::default();
        let step = (P0_HZ / 20.0) as u64;
        let (mut yaw_sum, mut lat_max) = (0.0, 0i16);
        for k in 0..20 * 200 {
            imu.update(k * step, &mut r, Some(&mut g));
            yaw_sum += word(&r, 0x45) as f64 / 131.072 / 20.0;
            lat_max = lat_max.min(word(&r, 0x3B)); // X points left: a right turn pushes it negative
        }
        assert!((yaw_sum - 90.0).abs() < 3.0, "integrated yaw {yaw_sum}");
        assert!(lat_max < -1000, "lateral {lat_max}");
    }
}
