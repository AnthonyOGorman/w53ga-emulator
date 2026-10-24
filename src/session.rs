//! Shared command-line set-up of the native front ends (`w53ga`, `w53ga-headless`): which files make up the unit and
//! which accessories are plugged in.

use crate::machine::Machine;
use std::path::{Path, PathBuf};

/// Options common to the native front ends.
#[derive(Default, Debug, Clone)]
pub struct Options {
    /// 32 MiB NOR flash image of the main board.
    pub flash: Option<PathBuf>,
    /// Resume this saved state instead of switching on.
    pub state: Option<PathBuf>,
    /// Power-on state file (instead of the one built in).
    pub power_on_state: Option<PathBuf>,
    /// Controller code flash (1 MiB image or update package).
    pub rl78: Option<PathBuf>,
    /// Controller's serial flash image (otherwise an empty, provisioned chip).
    pub rl78_spi: Option<PathBuf>,
    /// SD card folder, extra files (card path, host file), keep the folder unchanged, card CID.
    pub sd: Option<PathBuf>,
    pub sd_add: Vec<(String, PathBuf)>,
    /// Added files whose final content goes back to the host file at exit (`--sd-persist`; also in `sd_add`).
    pub sd_persist: Vec<(String, PathBuf)>,
    pub sd_readonly: bool,
    pub sd_cid: Option<String>,
    /// HLE profile: None = automatic (built-in profiles), Some("none") = off, Some(path).
    pub hle: Option<String>,
    pub obd2: bool,
    pub dvr: bool,
    /// Wi-Fi access point "SSID[:PASSPHRASE][#MAC]" (Some("") = defaults).
    pub wifi: Option<String>,
    /// GPS route "lat,lon;lat,lon;..." or "@FILE", speed in km/h.
    pub gps: Option<String>,
    pub gps_speed: Option<f64>,
    /// Radar transmitters for the radar receiver (see `radar::Scene::parse`).
    pub radar: Option<String>,
    /// Spans of emulated time (seconds) with the GPS sky blocked, like a tunnel: no fix while the drive goes on.
    pub gps_cut: Vec<(f64, f64)>,
}

pub const OPTIONS_HELP: &str = "\
  --flash FILE            32 MiB NOR flash image of the unit (required)
  --rl78 FILE             controller code flash: 1 MiB image or an update package carrying one (to switch on)
  --state FILE            resume a saved state instead of switching on (the flash image is still read)
  --sd DIR                SD card folder (served as a FAT32 SDHC card; changes written back on exit)
  --sd-readonly           keep the software's SD card writes in this session only; the folder is never changed
  --sd-add CARD=HOST      add a host file to the card at CARD (e.g. FOLDER/FILE.BIN=path), never written back
  --sd-persist CARD=HOST  add a host file to the card like --sd-add, and write what the software left in it back to
                          the host file on exit (also with --sd-readonly): settings kept from run to run
  --sd-cid HEX            the card's CID (32 hex digits), for data bound to a particular card
  --rl78-spi FILE         the controller's serial flash contents (default: empty, provisioned chip)
  --power-on-state FILE   SH7269 power-on reset state (when this build has none built in)
  --hle FILE|none         HLE profile (default: a built-in profile whose checks match the flash image, else
                          entry points detected at run time; none: no HLE)
  --obd2                  plug in the OBD-II adapter
  --dvr                   plug in the drive recorder and its camera
  --wifi [SSID[:PASS]]    fit the Wi-Fi module with one virtual access point bridged to this computer's network
  --gps ROUTE             drive the GNSS receiver along \"lat,lon;lat,lon;...\" or @FILE (one point per line;
                          \"lat,lon,alt\" with altitudes in metres for hills)
  --gps-speed KMH         route speed (default 50)
  --gps-cut FROM-TO[;..]  block the GPS sky (a tunnel) from FROM to TO emulated seconds: no fix, the drive goes on
  --radar SPEC            radar the unit can hear, entries separated by ';': LAT,LON[,RANGE_M] a radar at that
                          place (default range 1000 m, stronger when closer), @FROM-TO[:STRENGTH] a radar from
                          FROM to TO emulated seconds (strength 0-1, default 0.7)";

impl Options {
    /// Parse the options above from `args` (other arguments are left for the caller).
    pub fn parse(args: &[String]) -> Result<Options, String> {
        let opt = |name: &str| -> Option<String> { args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).filter(|v| !v.starts_with("--")).cloned() };
        let has = |name: &str| args.iter().any(|a| a == name);
        let mut o = Options {
            flash: opt("--flash").map(PathBuf::from),
            state: opt("--state").map(PathBuf::from),
            power_on_state: opt("--power-on-state").map(PathBuf::from),
            rl78: opt("--rl78").map(PathBuf::from),
            rl78_spi: opt("--rl78-spi").map(PathBuf::from),
            sd: opt("--sd").map(PathBuf::from),
            sd_readonly: has("--sd-readonly"),
            sd_cid: opt("--sd-cid"),
            hle: opt("--hle"),
            obd2: has("--obd2"),
            dvr: has("--dvr"),
            wifi: if has("--wifi") { Some(opt("--wifi").unwrap_or_default()) } else { None },
            gps: opt("--gps"),
            radar: opt("--radar"),
            gps_speed: opt("--gps-speed").map(|s| s.parse().map_err(|_| format!("--gps-speed: not a number: {s}"))).transpose()?,
            gps_cut: opt("--gps-cut").map(|s| crate::gnss::parse_spans(&s).map_err(|e| format!("--gps-cut: {e}"))).transpose()?.unwrap_or_default(),
            ..Default::default()
        };
        for (i, a) in args.iter().enumerate() {
            if a == "--sd-add" || a == "--sd-persist" {
                let spec = args.get(i + 1).ok_or(format!("{a} needs CARD=HOST"))?;
                let (c, h) = spec.split_once('=').ok_or(format!("{a} needs CARD=HOST"))?;
                o.sd_add.push((c.to_string(), PathBuf::from(h)));
                if a == "--sd-persist" { o.sd_persist.push((c.to_string(), PathBuf::from(h))); }
            }
        }
        Ok(o)
    }

    /// Build the machine these options describe (messages for the user are printed).
    pub fn build(&self) -> Result<Machine, String> {
        if let Some(p) = &self.power_on_state { crate::checkpoint::set_power_on_state(std::fs::read(p).map_err(|e| format!("{}: {e}", p.display()))?)?; }
        let flash_path = self.flash.as_ref().ok_or("--flash FILE is required (a 32 MiB dump of the unit's NOR flash)")?;
        let flash = std::fs::read(flash_path).map_err(|e| format!("{}: {e}", flash_path.display()))?;
        let mut m = match &self.state {
            Some(s) => {
                let mut m = Machine::load_state(s)?;
                if m.cpu.mem.flash != flash { println!("note: {} differs from the flash image saved in the state; using {}", flash_path.display(), flash_path.display()); }
                m.cpu.mem.flash = flash;
                if let Some(root) = &self.sd { self.reattach_sd(&mut m, root)?; }
                m
            }
            None => {
                let rl78 = self.rl78.as_ref().ok_or("--rl78 FILE is required to switch the unit on (the controller's code flash image)")?;
                let mut m = Machine::power_on(flash)?;
                let image = crate::rl78::companion::Companion::firmware_image(&std::fs::read(rl78).map_err(|e| format!("{}: {e}", rl78.display()))?, 0)?;
                let mut c = crate::rl78::companion::Companion::new(image);
                if let Some(p) = &self.rl78_spi { c.g13.spi = crate::rl78::spiflash::SpiFlash::from_image(std::fs::read(p).map_err(|e| format!("{}: {e}", p.display()))?); }
                if let Some(s) = &m.board.scif7 { c.sync(s); }
                m.board.companion = Some(Box::new(c));
                if let Some(root) = &self.sd {
                    let c = crate::checkpoint::Checkpoint::power_on()?;
                    let skipped = m.board.attach_sd_folder(&c, root, &self.sd_add)?;
                    println!("SD card: {} ({} files skipped: not 8.3 names{})", root.display(), skipped.len(), if self.sd_add.is_empty() { String::new() } else { format!(", {} extra files", self.sd_add.len()) });
                    for s in skipped.iter().take(10) { println!("  skipped {s}"); }
                } else {
                    println!("no SD card (--sd DIR)");
                }
                m
            }
        };
        if let Some(cid) = &self.sd_cid { m.board.set_card_cid(crate::periph::sdhi::parse_cid(cid)?)?; }
        if self.sd_readonly { if let Some(s) = &mut m.board.sdhi { s.card.keep_folder = true; } }
        m.configure_realtime();
        self.apply_profile(&mut m)?;
        if self.obd2 { let seen = m.board.scif6.as_ref().map_or(0, |s| s.tx_log.len()); m.board.obd2 = Some(crate::obd2::Obd2::new(seen)); }
        if self.dvr { let seen = m.board.scif1.as_ref().map_or(0, |s| s.tx_log.len()); m.board.dvr = Some(crate::dvr::Dvr::new(seen)); m.board.camera = true; }
        if let Some(w) = &self.wifi { m.board.sdhi1.card = Some(Box::new(crate::wlan::from_option(if w.is_empty() { None } else { Some(w.as_str()) }))); }
        if let Some(c) = m.board.companion.as_mut() {
            c.refresh_debug_env();
            if let Some(r) = &self.gps {
                let (wp, alts) = crate::gnss::parse_route_alt(r)?;
                let mut g = crate::gnss::Gnss::new(wp, self.gps_speed.unwrap_or(50.0));
                g.start_after = c.now() + 3_333_333;
                g.set_altitudes(alts);
                g.cut_spans = self.gps_cut.clone();
                c.gnss = Some(g);
            }
            if let Some(r) = &self.radar { c.radar = crate::radar::Scene::parse(r)?; }
        }
        Ok(m)
    }

    fn reattach_sd(&self, m: &mut Machine, root: &Path) -> Result<(), String> {
        let (vol, _) = crate::fat32::Volume::build(root, &self.sd_add)?;
        m.board.replace_sd_volume(vol)?;
        if let Some(s) = &mut m.board.sdhi { s.card.root = Some(root.to_path_buf()); }
        Ok(())
    }

    /// HLE hooks, in this order: the profile given with --hle, else a built-in profile matching the flash image, else
    /// runtime detection (`crate::detect`). `--hle none` turns HLE off.
    fn apply_profile(&self, m: &mut Machine) -> Result<(), String> {
        let p = match self.hle.as_deref() {
            Some("none") => {
                println!("HLE: off (2D graphics calls are not rendered; see README, \"HLE profiles\")");
                m.disable_hle();
                return Ok(());
            }
            Some(path) => {
                let p = crate::hle::Profile::load(Path::new(path))?;
                p.verify(&m.cpu.mem.flash)?;
                Some(p)
            }
            None => crate::hle::detect(&m.cpu.mem.flash),
        };
        match &p {
            Some(p) => println!("HLE profile: {}", p.name),
            None => println!("HLE profile: none; entry points are detected at run time"),
        }
        m.set_profile(p.as_ref());
        Ok(())
    }
}
