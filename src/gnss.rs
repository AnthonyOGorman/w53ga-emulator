//! Emulated GNSS receiver (Furuno eRide-style NMEA module) on the controller's UART0: a 1 Hz position fix along a
//! route of waypoints, as standard NMEA 0183 sentences, plus replies to the controller's `$PERD...` configuration.
//!
//! The route is a list of (latitude, longitude) waypoints in degrees driven at a constant speed; the fix advances in
//! emulated time, so the map follows it.

/// Seconds per peripheral clock of the SH-2A side (time base of the companion link).
const P0_HZ: f64 = 100.0e6 / 3.0;

#[derive(Clone, Copy, Debug)]
pub struct Fix { pub lat: f64, pub lon: f64, pub speed_kmh: f64, pub heading: f64 }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Gnss {
    pub waypoints: Vec<(f64, f64)>,
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
    /// Speed of the last reported fix (km/h), for accessories that follow the drive (OBD-II).
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
}

/// Route from "lat,lon;lat,lon;..." or "@FILE" (one "lat,lon" per line, '#' comments).
pub fn parse_route(spec: &str) -> Result<Vec<(f64, f64)>, String> {
    let text = match spec.strip_prefix('@') { Some(path) => std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?, None => spec.to_string() };
    let mut out = Vec::new();
    let points = text.lines().map(|l| l.split('#').next().unwrap_or("")).flat_map(|l| l.split(';')).map(str::trim);
    for p in points.filter(|l| !l.is_empty()) {
        let (a, b) = p.split_once(',').ok_or_else(|| format!("route point \"{p}\" is not lat,lon"))?;
        out.push((a.trim().parse::<f64>().map_err(|e| format!("{p}: {e}"))?, b.trim().parse::<f64>().map_err(|e| format!("{p}: {e}"))?));
    }
    Ok(out)
}

impl Gnss {
    pub fn new(waypoints: Vec<(f64, f64)>, speed_kmh: f64) -> Gnss {
        Gnss { waypoints, speed_kmh, epoch: (2026, 10, 19, 3, 0, 0), talker: "GN".into(), next_report: 0, rx: Vec::new(), commands: Vec::new(),
            reports: 0, log: std::env::var_os("AR_GNSS_LOG").is_some(), start_after: (P0_HZ * 1.0) as u64, last_speed: 0.0, travelled: 0.0, last_t: None, hold_at_end: false }
    }

    /// New route from the start (e.g. "drive from here to there"); the speed stays unless given.
    pub fn reroute(&mut self, waypoints: Vec<(f64, f64)>, speed_kmh: Option<f64>) {
        self.waypoints = waypoints;
        self.travelled = 0.0;
        if let Some(v) = speed_kmh { self.speed_kmh = v; }
    }

    /// Current fix (distance travelled so far).
    pub fn current(&self) -> Fix { self.fix_at_distance(self.travelled) }

    /// Position, speed and heading `t` seconds after start at the current speed.
    pub fn fix_at(&self, t: f64) -> Fix { self.fix_at_distance(t * self.speed_kmh / 3.6) }

    /// Position, speed and heading after `dist` metres (ping-pong along the waypoints, or held at the end; a route
    /// that ends where it starts, within 20 m, is driven round and round).
    pub fn fix_at_distance(&self, dist: f64) -> Fix {
        let wp = &self.waypoints;
        if wp.len() < 2 { let (lat, lon) = wp.first().copied().unwrap_or((35.0, 135.0)); return Fix { lat, lon, speed_kmh: 0.0, heading: 0.0 }; }
        let leg = |a: (f64, f64), b: (f64, f64)| -> f64 {
            let (dy, dx) = ((b.0 - a.0) * 111_320.0, (b.1 - a.1) * 111_320.0 * a.0.to_radians().cos());
            (dx * dx + dy * dy).sqrt()
        };
        let total: f64 = wp.windows(2).map(|w| leg(w[0], w[1])).sum();
        if total <= 0.0 { return Fix { lat: wp[0].0, lon: wp[0].1, speed_kmh: 0.0, heading: 0.0 }; }
        if self.hold_at_end && dist >= total { let (lat, lon) = *wp.last().unwrap(); let b = self.fix_at_distance(total - 0.01); return Fix { lat, lon, speed_kmh: 0.0, heading: b.heading }; }
        let looped = leg(wp[0], *wp.last().unwrap()) < 20.0;
        let mut d = if looped { dist % total } else { dist % (2.0 * total) };
        let forward = d <= total;
        if !forward { d = 2.0 * total - d; }
        for w in wp.windows(2) {
            let l = leg(w[0], w[1]);
            if d <= l || std::ptr::eq(w.as_ptr(), wp[wp.len() - 2..].as_ptr()) {
                let f = if l > 0.0 { (d / l).min(1.0) } else { 0.0 };
                let (a, b) = if forward { (w[0], w[1]) } else { (w[1], w[0]) };
                let (lat, lon) = if forward { (a.0 + (b.0 - a.0) * f, a.1 + (b.1 - a.1) * f) } else { (b.0 + (a.0 - b.0) * f, b.1 + (a.1 - b.1) * f) };
                let (dy, dx) = ((b.0 - a.0), (b.1 - a.1) * a.0.to_radians().cos());
                let heading = (dx.atan2(dy).to_degrees() + 360.0) % 360.0;
                return Fix { lat, lon, speed_kmh: self.speed_kmh, heading };
            }
            d -= l;
        }
        let (lat, lon) = *wp.last().unwrap();
        Fix { lat, lon, speed_kmh: 0.0, heading: 0.0 }
    }

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
        self.travelled += self.speed_kmh / 3.6 * (t - self.last_t.unwrap_or(t)).max(0.0);
        self.last_t = Some(t);
        let f = self.current();
        self.last_speed = f.speed_kmh;
        let (y, mo, d, h, mi, s) = utc(self.epoch, t as u64);
        let time = format!("{h:02}{mi:02}{s:02}.00");
        let date = format!("{d:02}{mo:02}{:02}", y % 100);
        let (lat, ns) = nmea_angle(f.lat, 2, 'N', 'S');
        let (lon, ew) = nmea_angle(f.lon, 3, 'E', 'W');
        let knots = f.speed_kmh / 1.852;
        let tk = self.talker.as_str();
        let mut text = String::new();
        text += &sentence(&format!("{tk}RMC,{time},A,{lat},{ns},{lon},{ew},{knots:.1},{:.1},{date},,,A", f.heading));
        text += &sentence(&format!("{tk}GGA,{time},{lat},{ns},{lon},{ew},1,09,0.9,120.0,M,36.0,M,,"));
        text += &sentence(&format!("{tk}GNS,{time},{lat},{ns},{lon},{ew},AA,09,0.9,120.0,36.0,,"));
        text += &sentence("GPGSV,3,1,09,02,45,050,42,05,60,120,45,12,30,200,38,13,15,300,35");
        text += &sentence("GPGSV,3,2,09,15,70,030,47,18,25,250,36,24,40,090,41,25,10,320,30");
        text += &sentence("GPGSV,3,3,09,29,55,170,44");
        // GLONASS satellites: the controller waits for a complete GSV cycle per configured constellation (GPS + GLONASS)
        text += &sentence("GLGSV,2,1,06,65,35,060,40,66,62,140,44,72,20,220,36,73,48,300,42");
        text += &sentence("GLGSV,2,2,06,80,15,010,33,81,55,250,43");
        if self.log { eprint!("[gnss] -> {text}"); }
        out.extend(text.bytes());
        self.reports += 1;
    }
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
