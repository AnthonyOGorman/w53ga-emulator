//! Emulated GNSS receiver (Furuno eRide-style NMEA module) on the controller's UART0: a 1 Hz position fix along a
//! route of waypoints, as standard NMEA 0183 sentences, plus replies to the controller's `$PERD...` configuration.
//!
//! The route is a list of (latitude, longitude[, altitude]) waypoints driven by a simple vehicle: it stands for the
//! first seconds after power-on (engine just started), then its speed follows the set speed with limited acceleration
//! and braking, and corners are driven as smooth curves (quadratic Bézier between the tangent points), so position,
//! course, speed and the inertial sensor (`crate::imu`) tell one consistent story.
//!
//! The sky can be blocked (a tunnel): the receiver keeps sending its sentences once a second, with the time running
//! on, but without a fix (RMC status V, GGA quality 0, no satellites used, satellites in view without signal), while
//! the vehicle drives on. When the sky clears the receiver tracks again and reports a fix a couple of seconds later.

/// Seconds per peripheral clock of the SH-2A side (time base of the companion link).
const P0_HZ: f64 = 100.0e6 / 3.0;
/// Pulling away / braking of the emulated vehicle (m/s²).
const ACCEL: f64 = 2.0;
const BRAKE: f64 = 3.0;
/// Lateral acceleration the corner radii are designed for at the set speed (m/s²), and the radius limits (m).
const LAT_ACCEL: f64 = 2.5;
const R_MIN: f64 = 15.0;
const R_MAX: f64 = 500.0;
/// After the sky clears: seconds of tracking without a fix (a hot receiver re-acquiring).
const REACQUIRE_S: f64 = 2.0;
/// Altitude reported for routes without altitudes (m).
const DEFAULT_ALT: f64 = 120.0;
/// The vehicle stands for the first seconds after power-on, as when the engine has just been started: the unit zeroes
/// its G-sensor then (about 2.5 to 10.5 s after power-on) and would take a moving car's acceleration for a tilted
/// mounting, which it later integrates as braking whenever the GPS has no fix.
const DEPART_S: f64 = 15.0;

#[derive(Clone, Copy, Debug)]
pub struct Fix { pub lat: f64, pub lon: f64, pub speed_kmh: f64, pub heading: f64, pub alt: f64 }

/// The vehicle's state for the inertial sensor: speed (m/s), course (degrees, clockwise from north), pitch (radians,
/// nose up positive).
#[derive(Clone, Copy, Debug, Default)]
pub struct Motion { pub speed: f64, pub heading: f64, pub pitch: f64 }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Gnss {
    pub waypoints: Vec<(f64, f64)>,
    /// Set speed (km/h); the vehicle's actual speed follows it (`last_speed`).
    pub speed_kmh: f64,
    /// UTC date/time at emulated time zero: (year, month, day, hour, minute, second).
    pub epoch: (u32, u32, u32, u32, u32, u32),
    /// Talker for position sentences ("GN" multi-GNSS or "GP").
    pub talker: String,
    next_report: u64,
    rx: Vec<u8>,
    pub commands: Vec<String>,
    pub reports: u64,
    pub log: bool,
    /// Output enabled (the controller powers and configures the receiver first).
    pub start_after: u64,
    /// The vehicle's current speed (km/h), for accessories that follow the drive (OBD-II, camera).
    #[serde(skip)]
    pub last_speed: f64,
    /// Distance travelled along the route (m), integrated from the speed so speed changes do not jump the position.
    #[serde(skip)]
    pub travelled: f64,
    #[serde(skip)]
    last_t: Option<f64>,
    /// Stop at the last waypoint instead of driving the route back and forth.
    #[serde(skip)]
    pub hold_at_end: bool,
    /// Altitude of each waypoint (m); empty: a flat route at the default altitude.
    #[serde(skip)]
    pub altitudes: Vec<f64>,
    /// Sky blocked by hand (a tunnel): no fix while set.
    #[serde(skip)]
    pub cut: bool,
    /// Sky blocked during these spans of emulated time (seconds since power-on, as in frame names and the status).
    #[serde(skip)]
    pub cut_spans: Vec<(f64, f64)>,
    /// Emulated second until which the receiver re-acquires after the sky cleared.
    #[serde(skip)]
    reacquire_until: f64,
    #[serde(skip)]
    was_blocked: bool,
    /// Vehicle speed (m/s).
    #[serde(skip)]
    v: f64,
    #[serde(skip)]
    path: Option<Path>,
}

/// Route from "lat,lon;lat,lon;..." or "@FILE" (one "lat,lon" per line, '#' comments).
pub fn parse_route(spec: &str) -> Result<Vec<(f64, f64)>, String> { parse_route_alt(spec).map(|(w, _)| w) }

/// Route with optional altitudes: points "lat,lon" or "lat,lon,alt" (metres); the altitudes are returned only when
/// every point has one.
pub fn parse_route_alt(spec: &str) -> Result<(Vec<(f64, f64)>, Vec<f64>), String> {
    let text = match spec.strip_prefix('@') { Some(path) => std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?, None => spec.to_string() };
    let (mut out, mut alts) = (Vec::new(), Vec::new());
    let points = text.lines().map(|l| l.split('#').next().unwrap_or("")).flat_map(|l| l.split(';')).map(str::trim);
    for p in points.filter(|l| !l.is_empty()) {
        let v: Vec<&str> = p.split(',').map(str::trim).collect();
        if v.len() < 2 || v.len() > 3 { return Err(format!("route point \"{p}\" is not lat,lon or lat,lon,alt")); }
        let num = |s: &str| s.parse::<f64>().map_err(|e| format!("{p}: {e}"));
        out.push((num(v[0])?, num(v[1])?));
        if let Some(a) = v.get(2) { alts.push(num(a)?); }
    }
    if alts.len() != out.len() { alts.clear(); }
    Ok((out, alts))
}

/// "FROM-TO[;FROM-TO...]" in emulated seconds.
pub fn parse_spans(spec: &str) -> Result<Vec<(f64, f64)>, String> {
    spec.split(';').map(str::trim).filter(|s| !s.is_empty()).map(|s| {
        let (a, b) = s.split_once('-').ok_or_else(|| format!("\"{s}\" is not FROM-TO (seconds)"))?;
        let (a, b) = (a.trim().parse::<f64>().map_err(|e| format!("{s}: {e}"))?, b.trim().parse::<f64>().map_err(|e| format!("{s}: {e}"))?);
        if b <= a { return Err(format!("\"{s}\": the end must come after the start")); }
        Ok((a, b))
    }).collect()
}

/// A corner of the driven path: it runs from `l1` metres before to `l2` metres after the vertex `v` (at route distance
/// `s`), between the tangent points `a` and `b`. Bézier corners carry the position along the curve; sharp turns
/// (U-turns) and the corner where a closed path starts (turned entirely before it, so the drive starts on course) only
/// turn the course.
#[derive(Clone, Debug)]
struct Corner { s: f64, l1: f64, l2: f64, a: (f64, f64), v: (f64, f64), b: (f64, f64), din: (f64, f64), dout: (f64, f64), turn: f64, bezier: bool }

/// The path the vehicle drives, in metres east/north of the first waypoint: the waypoints (hold at the end), the
/// waypoints closed into a loop (a route ending where it starts), or the waypoints and back again (back and forth).
#[derive(Clone, Debug)]
struct Path { key: (usize, u64, bool), lat0: f64, lon0: f64, k_lon: f64, pts: Vec<(f64, f64)>, alt: Vec<f64>, cum: Vec<f64>, total: f64, closed: bool, corners: Vec<Corner> }

fn key(wp: &[(f64, f64)], alts: &[f64], hold: bool) -> (usize, u64, bool) {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for (a, b) in wp { a.to_bits().hash(&mut h); b.to_bits().hash(&mut h); }
    for a in alts { a.to_bits().hash(&mut h); }
    (wp.len(), h.finish(), hold)
}

fn hdg(d: (f64, f64)) -> f64 { (d.0.atan2(d.1).to_degrees() + 360.0) % 360.0 }
fn wrap180(a: f64) -> f64 { let x = (a + 180.0).rem_euclid(360.0) - 180.0; if x == -180.0 { 180.0 } else { x } }

impl Path {
    fn build(wp: &[(f64, f64)], alts: &[f64], hold: bool, set_speed_kmh: f64) -> Path {
        let (lat0, lon0) = wp[0];
        let k_lon = 111_320.0 * lat0.to_radians().cos();
        let xy = |p: (f64, f64)| ((p.1 - lon0) * k_lon, (p.0 - lat0) * 111_320.0);
        let alt_of = |i: usize| alts.get(i).copied().unwrap_or(DEFAULT_ALT);
        let (mut pts, mut alt): (Vec<(f64, f64)>, Vec<f64>) = (wp.iter().map(|&p| xy(p)).collect(), (0..wp.len()).map(alt_of).collect());
        let dist = |a: (f64, f64), b: (f64, f64)| ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        let looped = !hold && dist(pts[0], *pts.last().unwrap()) < 20.0;
        let closed = !hold;
        if looped {
            if dist(pts[0], *pts.last().unwrap()) < 0.01 { pts.pop(); alt.pop(); }
        } else if !hold {
            for i in (1..wp.len() - 1).rev() { pts.push(pts[i]); alt.push(alt[i]); }
        }
        // drop zero-length legs
        let mut i = 1;
        while i < pts.len() { if dist(pts[i - 1], pts[i]) < 0.01 { pts.remove(i); alt.remove(i); } else { i += 1; } }
        let n = pts.len();
        let mut cum = vec![0.0];
        for i in 1..n { cum.push(cum[i - 1] + dist(pts[i - 1], pts[i])); }
        let total = cum[n - 1] + if closed && n > 1 { dist(pts[n - 1], pts[0]) } else { 0.0 };
        let v = set_speed_kmh / 3.6;
        let r = (v * v / LAT_ACCEL).clamp(R_MIN, R_MAX);
        let mut corners = Vec::new();
        if n >= 2 {
            let seg = |i: usize| -> ((f64, f64), f64) {
                let (a, b) = (pts[i % n], pts[(i + 1) % n]);
                let l = dist(a, b);
                (((b.0 - a.0) / l, (b.1 - a.1) / l), l)
            };
            let verts: Vec<usize> = if closed { (0..n).collect() } else { (1..n - 1).collect() };
            for j in verts {
                let (din, lin) = seg((j + n - 1) % n);
                let (dout, lout) = seg(j);
                let turn = wrap180(hdg(dout) - hdg(din));
                if turn.abs() < 0.5 || lin < 0.01 || lout < 0.01 { continue; }
                let smooth = turn.abs() <= 120.0;
                let want = if smooth { r * (turn.abs().to_radians() / 2.0).tan() } else { R_MIN };
                let l = want.min(0.45 * lin).min(0.45 * lout);
                if l < 0.05 { continue; }
                let vtx = pts[j];
                let (l1, l2, bezier) = if j == 0 { (2.0 * l, 0.0, false) } else { (l, l, smooth) };
                corners.push(Corner { s: cum[j], l1, l2, a: (vtx.0 - din.0 * l, vtx.1 - din.1 * l), v: vtx, b: (vtx.0 + dout.0 * l, vtx.1 + dout.1 * l), din, dout, turn, bezier });
            }
        }
        Path { key: (0, 0, hold), lat0, lon0, k_lon, pts, alt, cum, total, closed, corners }
    }

    /// Position (m), course (deg), altitude (m) and grade (rise per metre) after `s` metres of the path (wrapped).
    fn at(&self, s: f64) -> ((f64, f64), f64, f64, f64) {
        let n = self.pts.len();
        if n < 2 || self.total <= 0.0 { return (self.pts[0], 0.0, self.alt[0], 0.0); }
        let s = if self.closed { s.rem_euclid(self.total) } else { s.clamp(0.0, self.total) };
        // polyline position
        let i = match self.cum.binary_search_by(|c| c.partial_cmp(&s).unwrap()) { Ok(i) => i, Err(i) => i - 1 }.min(if self.closed { n - 1 } else { n - 2 });
        let (a, b) = (self.pts[i], self.pts[(i + 1) % n]);
        let l = if i + 1 < n { self.cum[i + 1] - self.cum[i] } else { self.total - self.cum[i] };
        let f = if l > 0.0 { ((s - self.cum[i]) / l).clamp(0.0, 1.0) } else { 0.0 };
        let mut pos = (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f);
        let mut course = hdg((b.0 - a.0, b.1 - a.1));
        let (aa, ab) = (self.alt[i], self.alt[(i + 1) % n]);
        let alt = aa + (ab - aa) * f;
        let grade = if l > 0.0 { (ab - aa) / l } else { 0.0 };
        for c in &self.corners {
            // distance from the vertex, the path's ends wrapping round for a closed path
            let mut d = s - c.s;
            if self.closed { if d > self.total / 2.0 { d -= self.total; } else if d < -self.total / 2.0 { d += self.total; } }
            if d < -c.l1 || d >= c.l2 { continue; }
            let u = (d + c.l1) / (c.l1 + c.l2);
            if c.bezier {
                let (p, q, r) = ((1.0 - u) * (1.0 - u), 2.0 * u * (1.0 - u), u * u);
                pos = (p * c.a.0 + q * c.v.0 + r * c.b.0, p * c.a.1 + q * c.v.1 + r * c.b.1);
                course = hdg(((1.0 - u) * c.din.0 + u * c.dout.0, (1.0 - u) * c.din.1 + u * c.dout.1));
            } else {
                course = (hdg(c.din) + c.turn * u).rem_euclid(360.0);
            }
            break;
        }
        (pos, course, alt, grade)
    }

    fn latlon(&self, p: (f64, f64)) -> (f64, f64) { (self.lat0 + p.1 / 111_320.0, self.lon0 + p.0 / self.k_lon) }
}

impl Gnss {
    pub fn new(waypoints: Vec<(f64, f64)>, speed_kmh: f64) -> Gnss {
        Gnss { waypoints, speed_kmh, epoch: (2026, 10, 19, 3, 0, 0), talker: "GN".into(), next_report: 0, rx: Vec::new(), commands: Vec::new(),
            reports: 0, log: std::env::var_os("AR_GNSS_LOG").is_some(), start_after: (P0_HZ * 1.0) as u64, last_speed: 0.0, travelled: 0.0, last_t: None, hold_at_end: false,
            altitudes: Vec::new(), cut: false, cut_spans: Vec::new(), reacquire_until: 0.0, was_blocked: false, v: 0.0, path: None }
    }

    /// New route from the start (e.g. "drive from here to there"); the speed stays unless given. The vehicle keeps
    /// its speed (it does not stop to turn onto the new route).
    pub fn reroute(&mut self, waypoints: Vec<(f64, f64)>, speed_kmh: Option<f64>) {
        self.waypoints = waypoints;
        self.altitudes.clear();
        self.travelled = 0.0;
        self.path = None;
        if let Some(v) = speed_kmh { self.speed_kmh = v; }
    }

    /// The driven path for the current waypoints (rebuilt when they change).
    fn path(&mut self) -> Option<&Path> {
        if self.waypoints.len() < 2 { return None; }
        let k = key(&self.waypoints, &self.altitudes, self.hold_at_end);
        if self.path.as_ref().map(|p| p.key) != Some(k) {
            let mut p = Path::build(&self.waypoints, &self.altitudes, self.hold_at_end, self.speed_kmh);
            p.key = k;
            self.path = Some(p);
        }
        self.path.as_ref()
    }

    /// Current fix (distance travelled so far, the vehicle's speed).
    pub fn current(&self) -> Fix {
        let mut f = self.fix_at_distance(self.travelled);
        f.speed_kmh = if self.waypoints.len() < 2 { 0.0 } else { self.v * 3.6 };
        f
    }

    /// Position, speed and heading `t` seconds after start at the set speed.
    pub fn fix_at(&self, t: f64) -> Fix { self.fix_at_distance(t * self.speed_kmh / 3.6) }

    /// Position, (set) speed and course after `dist` metres: round and round a route that ends where it starts (within
    /// 20 m), back and forth along any other, or held at the end.
    pub fn fix_at_distance(&self, dist: f64) -> Fix {
        let wp = &self.waypoints;
        if wp.len() < 2 { let (lat, lon) = wp.first().copied().unwrap_or((35.0, 135.0)); return Fix { lat, lon, speed_kmh: 0.0, heading: 0.0, alt: self.altitudes.first().copied().unwrap_or(DEFAULT_ALT) }; }
        let built;
        let path = match &self.path {
            Some(p) if p.key == key(wp, &self.altitudes, self.hold_at_end) => p,
            _ => { built = Path::build(wp, &self.altitudes, self.hold_at_end, self.speed_kmh); &built }
        };
        let (pos, heading, alt, _) = path.at(dist);
        let (lat, lon) = path.latlon(pos);
        let speed_kmh = if self.hold_at_end && dist >= path.total { 0.0 } else { self.speed_kmh };
        Fix { lat, lon, speed_kmh, heading, alt }
    }

    /// Drive the vehicle on to `now` (P0 clocks): its speed follows the set speed with limited acceleration and
    /// braking (and brakes to stop at the end of a route that holds there).
    fn drive_to(&mut self, now: u64) {
        let depart = self.start_after.max((DEPART_S * P0_HZ) as u64);
        if now < depart || self.waypoints.len() < 2 { self.last_speed = 0.0; return; }
        let t = (now - depart) as f64 / P0_HZ;
        let Some(mut last) = self.last_t else { self.last_t = Some(t); return };
        if t <= last { return; }
        let (total, hold) = match self.path() { Some(p) => (p.total, !p.closed), None => return };
        while last < t {
            let h = (t - last).min(0.05);
            let mut target = self.speed_kmh.max(0.0) / 3.6;
            if hold { target = target.min((2.0 * BRAKE * (total - self.travelled).max(0.0)).sqrt()); }
            let v0 = self.v;
            self.v = (v0 + (target - v0).clamp(-BRAKE * h, ACCEL * h)).max(0.0);
            self.travelled += (v0 + self.v) / 2.0 * h;
            if hold && self.travelled >= total { self.travelled = total; self.v = 0.0; }
            last += h;
        }
        self.last_t = Some(t);
        self.last_speed = self.v * 3.6;
    }

    /// The vehicle's motion at `now` (P0 clocks), for the inertial sensor.
    pub fn motion(&mut self, now: u64) -> Motion {
        self.drive_to(now);
        let travelled = self.travelled;
        let v = self.v;
        match self.path() {
            Some(p) if p.pts.len() >= 2 => { let (_, heading, _, grade) = p.at(travelled); Motion { speed: v, heading, pitch: grade.atan() } }
            _ => Motion::default(),
        }
    }

    /// Whether the sky is blocked (a tunnel) at emulated second `t`.
    pub fn sky_blocked(&self, t: f64) -> bool { self.cut || self.cut_spans.iter().any(|&(a, b)| t >= a && t < b) }

    /// Whether the receiver has a fix at emulated second `t` (sky clear and re-acquired).
    pub fn has_fix(&self, t: f64) -> bool { !self.sky_blocked(t) && t >= self.reacquire_until }

    /// Bytes from the controller (configuration sentences).
    pub fn on_rx(&mut self, data: &[u8], out: &mut Vec<u8>) {
        self.rx.extend_from_slice(data);
        while let Some(i) = self.rx.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.rx.drain(..=i).collect();
            let s = String::from_utf8_lossy(&line).trim().to_string();
            if s.is_empty() { continue; }
            if self.log { eprintln!("[gnss] <- {s}"); }
            if s.starts_with("$PERDSYS,VERSION") { out.extend(sentence("PERDSYS,VERSION,OPUS7_SFLASH_ES1,ENP632A1229F,FURUNO_GN87F,ENP632A1229F").bytes()); }
            else if let Some(body) = s.strip_prefix('$').and_then(|b| b.split('*').next()) { out.extend(sentence(&format!("PERDACK,{},0", body.split(',').take(2).collect::<Vec<_>>().join(","))).bytes()); }
            self.commands.push(s);
        }
    }

    /// Emit the sentences due at emulated time `now` (P0 clocks).
    pub fn tick(&mut self, now: u64, out: &mut Vec<u8>) {
        if now < self.start_after || now < self.next_report { return; }
        if self.next_report == 0 { self.next_report = now; }
        self.next_report = now.max(self.next_report) + P0_HZ as u64;
        let t = (now - self.start_after) as f64 / P0_HZ;
        self.drive_to(now);
        let f = self.current();
        let secs = now as f64 / P0_HZ;
        let blocked = self.sky_blocked(secs);
        if blocked { self.was_blocked = true; } else if self.was_blocked { self.was_blocked = false; self.reacquire_until = secs + REACQUIRE_S; }
        let fix = self.has_fix(secs);
        let (y, mo, d, h, mi, s) = utc(self.epoch, t as u64);
        let time = format!("{h:02}{mi:02}{s:02}.00");
        let date = format!("{d:02}{mo:02}{:02}", y % 100);
        let tk = self.talker.as_str();
        let mut text = String::new();
        if fix {
            let (lat, ns) = nmea_angle(f.lat, 2, 'N', 'S');
            let (lon, ew) = nmea_angle(f.lon, 3, 'E', 'W');
            let knots = f.speed_kmh / 1.852;
            let alt = f.alt;
            text += &sentence(&format!("{tk}RMC,{time},A,{lat},{ns},{lon},{ew},{knots:.1},{:.1},{date},,,A", f.heading));
            text += &sentence(&format!("{tk}GGA,{time},{lat},{ns},{lon},{ew},1,09,0.9,{alt:.1},M,36.0,M,,"));
            text += &sentence(&format!("{tk}GNS,{time},{lat},{ns},{lon},{ew},AA,09,0.9,{alt:.1},36.0,,"));
        } else {
            // no position: the time runs on from the receiver's clock
            text += &sentence(&format!("{tk}RMC,{time},V,,,,,,,{date},,,N"));
            text += &sentence(&format!("{tk}GGA,{time},,,,,0,00,,,M,,M,,"));
            text += &sentence(&format!("{tk}GNS,{time},,,,,NN,00,,,,,"));
        }
        // satellites in view; with the sky blocked their positions are known (almanac) but there is no signal
        let sv = |s: &str| if blocked { strip_snr(s) } else { s.to_string() };
        text += &sentence(&sv("GPGSV,3,1,09,02,45,050,42,05,60,120,45,12,30,200,38,13,15,300,35"));
        text += &sentence(&sv("GPGSV,3,2,09,15,70,030,47,18,25,250,36,24,40,090,41,25,10,320,30"));
        text += &sentence(&sv("GPGSV,3,3,09,29,55,170,44"));
        // GLONASS satellites: the controller waits for a complete GSV cycle per configured constellation (GPS + GLONASS)
        text += &sentence(&sv("GLGSV,2,1,06,65,35,060,40,66,62,140,44,72,20,220,36,73,48,300,42"));
        text += &sentence(&sv("GLGSV,2,2,06,80,15,010,33,81,55,250,43"));
        if self.log { eprint!("[gnss] -> {text}"); }
        out.extend(text.bytes());
        self.reports += 1;
    }
}

/// A GSV body with the signal-to-noise fields emptied (satellites in view, none received).
fn strip_snr(body: &str) -> String {
    let f: Vec<&str> = body.split(',').collect();
    f.iter().enumerate().map(|(i, x)| if i >= 4 && (i - 4) % 4 == 3 { "" } else { x }).collect::<Vec<_>>().join(",")
}

/// `$body*CS\r\n` with the NMEA XOR checksum.
pub fn sentence(body: &str) -> String {
    let cs = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${body}*{cs:02X}\r\n")
}

/// ddmm.mmmm / dddmm.mmmm and hemisphere.
fn nmea_angle(v: f64, deg_digits: usize, pos: char, neg: char) -> (String, char) {
    let a = v.abs();
    let deg = a.floor();
    let min = (a - deg) * 60.0;
    (format!("{:0w$}{:07.4}", deg as u32, min, w = deg_digits), if v >= 0.0 { pos } else { neg })
}

/// Calendar after `secs` seconds from `epoch` (simple day roll-over, months of the given year).
fn utc(e: (u32, u32, u32, u32, u32, u32), secs: u64) -> (u32, u32, u32, u32, u32, u32) {
    let total = e.3 as u64 * 3600 + e.4 as u64 * 60 + e.5 as u64 + secs;
    let days = (total / 86400) as u32;
    let rem = total % 86400;
    let (mut y, mut mo, mut d) = (e.0, e.1, e.2 + days);
    loop {
        let ml = match mo { 2 => if y % 4 == 0 { 29 } else { 28 }, 4 | 6 | 9 | 11 => 30, _ => 31 };
        if d <= ml { break; }
        d -= ml; mo += 1;
        if mo > 12 { mo = 1; y += 1; }
    }
    (y, mo, d, (rem / 3600) as u32, ((rem / 60) % 60) as u32, (rem % 60) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snr_stripped() {
        assert_eq!(strip_snr("GPGSV,3,3,09,29,55,170,44"), "GPGSV,3,3,09,29,55,170,");
        assert_eq!(strip_snr("GLGSV,2,2,06,80,15,010,33,81,55,250,43"), "GLGSV,2,2,06,80,15,010,,81,55,250,");
    }

    #[test]
    fn spans() {
        assert_eq!(parse_spans("70-100;130.5-140").unwrap(), vec![(70.0, 100.0), (130.5, 140.0)]);
        assert!(parse_spans("100-70").is_err());
        assert!(parse_spans("x").is_err());
    }

    #[test]
    fn tunnel_blocks_the_fix_and_the_drive_goes_on() {
        let mut g = Gnss::new(vec![(35.0, 135.0), (35.1, 135.0)], 72.0);
        g.start_after = 0;
        g.cut_spans = vec![(40.0, 50.0)];
        let mut text = std::collections::BTreeMap::new();
        for s in 0..60u64 {
            let mut out = Vec::new();
            g.tick((s as f64 * P0_HZ) as u64 + 1, &mut out);
            text.insert(s, String::from_utf8(out).unwrap());
        }
        let rmc = |s: u64| text[&s].lines().find(|l| l.contains("RMC")).unwrap().to_string();
        assert!(rmc(39).contains(",A,") && text[&39].contains(",1,09,"));
        assert!(rmc(45).contains(",V,,,,,,,") && text[&45].contains("GGA") && text[&45].contains(",0,00,"), "{}", text[&45]);
        assert!(text[&45].contains("GPGSV,3,3,09,29,55,170,*"), "satellites in view without signal");
        assert!(rmc(51).contains(",V,") && text[&51].contains("GPGSV,3,3,09,29,55,170,44"), "re-acquiring");
        assert!(rmc(53).contains(",A,"));
        // the car drove on through the tunnel: 72 km/h for 10 s is 200 m
        let lat = |s: &str| { let f: Vec<&str> = s.split(',').collect(); f[3][..2].parse::<f64>().unwrap() + f[3][2..].parse::<f64>().unwrap() / 60.0 };
        assert!(((lat(&rmc(53)) - lat(&rmc(39))) * 111_320.0 - 14.0 * 20.0).abs() < 5.0);
    }

    #[test]
    fn corner_course_is_continuous() {
        // an L-shaped route: 1 km north, then 1 km east; the course turns smoothly from 0 to 90 degrees
        let g = Gnss::new(vec![(35.0, 135.0), (35.009, 135.0), (35.009, 135.011)], 50.0);
        let mut last = g.fix_at_distance(0.0).heading;
        assert!(wrap180(last).abs() < 0.5, "starts on course north, not {last}");
        let mut max_step: f64 = 0.0;
        for k in 1..1900 { // up to the U-turn at the far end (driven back and forth)
            let h = g.fix_at_distance(k as f64).heading;
            max_step = max_step.max(wrap180(h - last).abs());
            last = h;
        }
        assert!(max_step < 3.0, "course jumps {max_step} degrees per metre");
        let f = g.fix_at_distance(1500.0);
        assert!((f.heading - 90.0).abs() < 1.0 && (f.lat - 35.009).abs() < 1e-5);
    }

    #[test]
    fn vehicle_accelerates_and_brakes() {
        let mut g = Gnss::new(vec![(35.0, 135.0), (35.1, 135.0)], 72.0);
        g.start_after = 0;
        let at = |s: f64| ((DEPART_S + s) * P0_HZ) as u64; // the car stands until DEPART_S after power-on
        g.drive_to(at(-1.0));
        assert_eq!(g.last_speed, 0.0);
        g.drive_to(at(0.0));
        g.drive_to(at(5.0));
        assert!((g.last_speed - 36.0).abs() < 0.5, "{}", g.last_speed); // 2 m/s² for 5 s
        g.drive_to(at(20.0));
        assert!((g.last_speed - 72.0).abs() < 0.01);
        g.speed_kmh = 0.0;
        g.drive_to(at(23.0));
        assert!((g.last_speed - 72.0 + 3.0 * 3.0 * 3.6).abs() < 0.5, "{}", g.last_speed);
    }
}
