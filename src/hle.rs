//! Optional high-level emulation (HLE) of software routines, configured by a profile.
//!
//! The emulator models hardware; it knows nothing about the program running on it. Two things are nevertheless
//! better handled at the level of the software, and both are optional and data-driven:
//!
//! - **2D graphics** (`rgah`): the SH7269's vector graphics processor runs undocumented microcode, so it is not
//!   emulated. Software reaches it through the Renesas RGAH 2D API; when a profile names the API's entry points in
//!   the loaded image, calls to `fillRect`, `drawImage`, `bitBlt`, `bitBltFixedColor` and `finish` are rendered in
//!   software instead (`crate::gfx`). Without it those calls return without drawing.
//! - **Idle skipping** (`idle_loop`): the program counter of an RTOS idle loop. While the CPU sits there with nothing
//!   deliverable, only the peripherals advance (a speed optimisation; the result is the same).
//!
//! A profile is JSON. Every entry may carry a `check`: the SHA-256 of `check_len` bytes of the image at that address,
//! so a profile is only applied to the image it was written for (a hash, not the bytes, so profiles contain no code):
//!
//! ```json
//! { "name": "example", "idle_loop": { "addr": "0x0001234", "check": "<sha256 hex>" },
//!   "rgah": { "fill_rect": { "addr": "0x..." }, "draw_image": { "addr": "0x..." }, "bit_blt": { "addr": "0x..." },
//!             "bit_blt_fixed_color": { "addr": "0x..." }, "finish": { "addr": "0x..." } } }
//! ```
//!
//! A profile is given with `--hle FILE` (native) or the web page's profile picker. Profiles placed in `profiles/`
//! before building are built in and chosen automatically when all their checks match the flash image. The repository
//! ships no profile: entry points are facts about a particular program, which the user supplies.
//! `w53ga-headless --make-profile` writes one from addresses the user knows.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Bytes hashed by a `check` unless the entry says otherwise.
pub const DEFAULT_CHECK_LEN: u32 = 32;

/// Built-in profiles (JSON text): whatever `profiles/*.json` held at build time (none in the repository).
const BUILTIN: &[&str] = crate::embedded::PROFILES;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct Entry {
    /// Address as a hex string ("0x1234") or a number.
    #[serde(with = "hex_addr")]
    pub addr: u32,
    /// SHA-256 (hex) of `check_len` bytes of the flash image at `addr`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub check: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub check_len: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct Rgah {
    pub fill_rect: Entry,
    pub draw_image: Entry,
    pub bit_blt: Entry,
    pub bit_blt_fixed_color: Entry,
    pub finish: Entry,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct Profile {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub idle_loop: Option<Entry>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rgah: Option<Rgah>,
}

/// The RGAH API functions a profile can intercept.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RgahFn { FillRect, DrawImage, BitBlt, BitBltFixedColor, Finish }

/// Entry points resolved from a profile, as the machine uses them.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Hooks {
    pub idle_loop: Option<u32>,
    /// (address, function), sorted by address.
    pub rgah: Vec<(u32, u8)>,
    /// Lowest and highest RGAH entry point (fast range test before the lookup).
    pub rgah_range: Option<(u32, u32)>,
    pub profile: Option<String>,
}

impl RgahFn {
    pub fn from_u8(v: u8) -> Option<RgahFn> {
        Some(match v { 0 => RgahFn::FillRect, 1 => RgahFn::DrawImage, 2 => RgahFn::BitBlt, 3 => RgahFn::BitBltFixedColor, 4 => RgahFn::Finish, _ => return None })
    }
    pub fn name(self) -> &'static str {
        match self { RgahFn::FillRect => "fillRect", RgahFn::DrawImage => "drawImage", RgahFn::BitBlt => "bitBlt", RgahFn::BitBltFixedColor => "bitBltFixedColor", RgahFn::Finish => "finish" }
    }
}

impl Hooks {
    /// The function intercepted at `pc`, if any.
    #[inline]
    pub fn rgah_at(&self, pc: u32) -> Option<RgahFn> {
        let (lo, hi) = self.rgah_range?;
        if pc < lo || pc > hi { return None; }
        self.rgah.iter().find(|(a, _)| *a == pc).and_then(|(_, f)| RgahFn::from_u8(*f))
    }
}

impl Entry {
    fn verify(&self, flash: &[u8]) -> Result<(), String> {
        let Some(want) = &self.check else { return Ok(()) };
        let n = self.check_len.unwrap_or(DEFAULT_CHECK_LEN) as usize;
        let a = self.addr as usize;
        let bytes = flash.get(a..a + n).ok_or_else(|| format!("{:#x}: outside the flash image", self.addr))?;
        let got = hex(&Sha256::digest(bytes));
        if got.eq_ignore_ascii_case(want.trim()) { Ok(()) } else { Err(format!("{:#x}: check does not match this flash image", self.addr)) }
    }

    /// An entry for `addr` with a check computed from `flash` (for writing profiles).
    pub fn with_check(flash: &[u8], addr: u32) -> Option<Entry> {
        let a = addr as usize;
        let bytes = flash.get(a..a + DEFAULT_CHECK_LEN as usize)?;
        Some(Entry { addr, check: Some(hex(&Sha256::digest(bytes))), check_len: None })
    }
}

impl Profile {
    pub fn parse(json: &str) -> Result<Profile, String> { serde_json::from_str(json).map_err(|e| format!("HLE profile: {e}")) }

    pub fn load(path: &std::path::Path) -> Result<Profile, String> {
        Self::parse(&std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?)
    }

    fn entries(&self) -> Vec<&Entry> {
        let mut v: Vec<&Entry> = self.idle_loop.iter().collect();
        if let Some(r) = &self.rgah { v.extend([&r.fill_rect, &r.draw_image, &r.bit_blt, &r.bit_blt_fixed_color, &r.finish]); }
        v
    }

    /// Check every entry against the flash image.
    pub fn verify(&self, flash: &[u8]) -> Result<(), String> {
        for e in self.entries() { e.verify(flash).map_err(|m| format!("profile {}: {m}", self.name))?; }
        Ok(())
    }

    /// The machine-side hooks of this profile.
    pub fn hooks(&self) -> Hooks {
        let mut rgah = Vec::new();
        if let Some(r) = &self.rgah {
            for (k, e) in [&r.fill_rect, &r.draw_image, &r.bit_blt, &r.bit_blt_fixed_color, &r.finish].into_iter().enumerate() { rgah.push((e.addr, k as u8)); }
        }
        rgah.sort();
        let rgah_range = if rgah.is_empty() { None } else { Some((rgah[0].0, rgah[rgah.len() - 1].0)) };
        Hooks { idle_loop: self.idle_loop.as_ref().map(|e| e.addr), rgah, rgah_range, profile: Some(self.name.clone()) }
    }
}

/// The built-in profiles.
pub fn builtin() -> Vec<Profile> { BUILTIN.iter().filter_map(|j| Profile::parse(j).ok()).collect() }

/// The first built-in profile whose checks all match `flash`.
pub fn detect(flash: &[u8]) -> Option<Profile> { builtin().into_iter().find(|p| p.verify(flash).is_ok()) }

fn hex(b: &[u8]) -> String { b.iter().map(|x| format!("{x:02x}")).collect() }

mod hex_addr {
    use serde::{Deserialize, Deserializer, Serializer};
    pub fn serialize<S: Serializer>(v: &u32, s: S) -> Result<S::Ok, S::Error> { s.serialize_str(&format!("{v:#x}")) }
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<u32, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum A { N(u32), S(String) }
        match A::deserialize(d)? {
            A::N(n) => Ok(n),
            A::S(s) => {
                let t = s.trim().trim_start_matches("0x").trim_start_matches("0X").replace('_', "");
                u32::from_str_radix(&t, 16).map_err(serde::de::Error::custom)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image() -> Vec<u8> { (0..4096u32).map(|i| (i * 7 + 3) as u8).collect() }

    #[test]
    fn builtin_profiles_parse() {
        for j in BUILTIN { Profile::parse(j).expect("built-in profile parses"); }
        assert_eq!(builtin().len(), BUILTIN.len());
    }

    #[test]
    fn addresses_parse_as_hex_strings_or_numbers() {
        let p = Profile::parse(r#"{"name":"t","idle_loop":{"addr":"0x00_10"},"rgah":{"fill_rect":{"addr":16},"draw_image":{"addr":"0x20"},
            "bit_blt":{"addr":"0x30"},"bit_blt_fixed_color":{"addr":"0x40"},"finish":{"addr":"0x50"}}}"#).unwrap();
        assert_eq!(p.idle_loop.as_ref().unwrap().addr, 0x10);
        let h = p.hooks();
        assert_eq!(h.idle_loop, Some(0x10));
        assert_eq!(h.rgah_range, Some((0x10, 0x50)));
        assert_eq!(h.rgah_at(0x30), Some(RgahFn::BitBlt));
        assert_eq!(h.rgah_at(0x31), None);
        assert_eq!(h.rgah_at(0x8), None);
    }

    #[test]
    fn checks_bind_a_profile_to_its_image() {
        let img = image();
        let e = Entry::with_check(&img, 0x100).unwrap();
        let p = Profile { name: "t".into(), idle_loop: Some(e), ..Default::default() };
        assert!(p.verify(&img).is_ok());
        let mut other = img.clone();
        other[0x105] ^= 1;
        assert!(p.verify(&other).is_err());
        // Round trip through JSON keeps the check.
        let back = Profile::parse(&serde_json::to_string(&p).unwrap()).unwrap();
        assert_eq!(back, p);
    }

    #[test]
    fn no_builtin_profile_matches_an_unrelated_image() {
        assert!(detect(&image()).is_none());
        assert!(detect(&[]).is_none());
    }
}
