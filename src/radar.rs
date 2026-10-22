//! Radar transmitters around the emulated unit, for its radar receiver (see `rl78::receiver`).
//!
//! The scene gives the receiver one number, the strength of the strongest radar it can hear (0 = none, 1 = the
//! strongest the detector shows), from fixed transmitters placed on the map (heard within a range, louder when
//! closer), from time windows, and from a manual level set by the user interface.
//!
//! Spec (`--radar`), entries separated by `;`:
//!   `LAT,LON[,RANGE_M]`        a speed-trap radar at that place (default range 1000 m)
//!   `@FROM-TO[:STRENGTH]`      a radar heard from FROM to TO emulated seconds (default strength 0.7)

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum Source {
    Fixed { lat: f64, lon: f64, range_m: f64 },
    Window { from_s: f64, to_s: f64, strength: f64 },
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Scene {
    pub sources: Vec<Source>,
    /// Strength set by the user interface (window key, web page); the receiver hears the strongest of it and the
    /// scene's sources.
    pub manual: f64,
}

/// Distance in metres between two positions (spherical earth).
pub fn distance_m(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (la1, la2) = (a.0.to_radians(), b.0.to_radians());
    let (dla, dlo) = (la2 - la1, (b.1 - a.1).to_radians());
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    2.0 * 6_371_000.0 * h.sqrt().asin()
}

impl Scene {
    pub fn parse(spec: &str) -> Result<Scene, String> {
        let mut sources = Vec::new();
        for e in spec.split(';').map(str::trim).filter(|e| !e.is_empty()) {
            let num = |s: &str| s.trim().parse::<f64>().map_err(|_| format!("--radar: not a number: {s}"));
            if let Some(w) = e.strip_prefix('@') {
                let (range, strength) = w.split_once(':').map_or((w, None), |(a, b)| (a, Some(b)));
                let (a, b) = range.split_once('-').ok_or(format!("--radar: {e}: expected @FROM-TO[:STRENGTH]"))?;
                sources.push(Source::Window { from_s: num(a)?, to_s: num(b)?, strength: strength.map(num).transpose()?.unwrap_or(0.7).clamp(0.0, 1.0) });
            } else {
                let p: Vec<&str> = e.split(',').collect();
                if p.len() < 2 || p.len() > 3 { return Err(format!("--radar: {e}: expected LAT,LON[,RANGE_M]")); }
                sources.push(Source::Fixed { lat: num(p[0])?, lon: num(p[1])?, range_m: p.get(2).map(|s| num(s)).transpose()?.unwrap_or(1000.0) });
            }
        }
        Ok(Scene { sources, manual: 0.0 })
    }

    /// Strength the receiver hears at emulated time `t_s` and the unit's position (if it has one).
    pub fn strength(&self, t_s: f64, pos: Option<(f64, f64)>) -> f64 {
        let mut s = self.manual;
        for src in &self.sources {
            let v = match *src {
                Source::Window { from_s, to_s, strength } => if (from_s..to_s).contains(&t_s) { strength } else { 0.0 },
                // fades in over the range: weakest at its edge, full strength from 100 m in
                Source::Fixed { lat, lon, range_m } => pos.map_or(0.0, |p| {
                    let d = distance_m(p, (lat, lon));
                    if d >= range_m { 0.0 } else { ((range_m - d) / (range_m - 100.0).max(1.0)).clamp(0.0, 1.0) }
                }),
            };
            s = s.max(v);
        }
        s.clamp(0.0, 1.0)
    }
}
