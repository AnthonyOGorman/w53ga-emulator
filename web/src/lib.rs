//! WebAssembly front end: the unchanged emulator crate driven from a browser Web Worker through a small C-style API.
//!
//! JavaScript owns the files, all of which stay in the browser: it passes the flash image and either the controller
//! image (power-on start) or a saved state as byte arrays, lists the SD folder's files (card path and size) and answers
//! `js_read` synchronously (FileReaderSync in the worker) when the emulated software reads the card. What the software
//! writes to the card can be fetched as file changes (`emu_sd_changes`) and written back to the folder by the page.
//! Strings and byte arrays cross the boundary through `alloc`ed buffers; results are fetched with `buf_ptr`/`buf_len`.

use std::cell::RefCell;
use w53ga_emu::machine::Machine;
use w53ga_emu::sdsync::Change;

#[link(wasm_import_module = "env")]
extern "C" {
    /// Read `len` bytes at `offset` of the SD file `path` into `dst`; 0 on success.
    fn js_read(path: *const u8, path_len: usize, offset: f64, dst: *mut u8, len: usize) -> i32;
}

const P0_HZ: f64 = 100.0e6 / 3.0;
const W: usize = 400;
const H: usize = 240;

struct Emu {
    m: Machine,
    frame: Vec<u32>,
    sd: Vec<(String, u64, bool)>,
    /// Started from power-on: the SD card is attached fresh (a saved state already holds the card's host state).
    power_on: bool,
    /// The Wi-Fi module's network requests and events, carried to and from the local server's /net relay by the worker.
    net: Option<w53ga_emu::usernet::QueueHost>,
    /// Card changes from the last `emu_sd_changes`, fetched one by one with `emu_sd_change_data`.
    changes: Vec<Change>,
}

thread_local! {
    static EMU: RefCell<Option<Emu>> = const { RefCell::new(None) };
    static BUF: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
    /// HLE profile chosen by the user (JSON), applied to every machine created afterwards.
    static PROFILE: RefCell<Option<w53ga_emu::hle::Profile>> = const { RefCell::new(None) };
}

fn set_buf(b: Vec<u8>) -> usize { let n = b.len(); BUF.with(|x| *x.borrow_mut() = b); n }
fn err(e: impl std::fmt::Display) -> i32 { set_buf(e.to_string().into_bytes()); -1 }
fn with<R>(f: impl FnOnce(&mut Emu) -> R) -> Option<R> { EMU.with(|e| e.borrow_mut().as_mut().map(f)) }
unsafe fn bytes<'a>(p: *const u8, n: usize) -> &'a [u8] { if n == 0 { &[] } else { std::slice::from_raw_parts(p, n) } }
unsafe fn text(p: *const u8, n: usize) -> String { String::from_utf8_lossy(bytes(p, n)).into_owned() }

/// Allocate `len` bytes for JavaScript to fill (freed with `dealloc`).
#[no_mangle]
pub extern "C" fn alloc(len: usize) -> *mut u8 { let mut v = vec![0u8; len.max(1)]; let p = v.as_mut_ptr(); std::mem::forget(v); p }

/// # Safety
/// `p`/`len` must come from `alloc`.
#[no_mangle]
pub unsafe extern "C" fn dealloc(p: *mut u8, len: usize) { drop(Vec::from_raw_parts(p, len.max(1), len.max(1))); }

/// Result buffer of the last call that produced bytes or an error message.
#[no_mangle]
pub extern "C" fn buf_ptr() -> *const u8 { BUF.with(|b| b.borrow().as_ptr()) }
#[no_mangle]
pub extern "C" fn buf_len() -> usize { BUF.with(|b| b.borrow().len()) }

/// 1 if this build can switch a unit on (a power-on state is built in or was supplied), else 0.
#[no_mangle]
pub extern "C" fn emu_power_on_available() -> i32 { w53ga_emu::checkpoint::power_on_available() as i32 }

/// Supply the power-on state (gzip-compressed JSON, `data/power-on.json.gz`). 0, or -1 with the message.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_set_power_on_state(p: *const u8, n: usize) -> i32 {
    match w53ga_emu::checkpoint::set_power_on_state(bytes(p, n).to_vec()) { Ok(()) => 0, Err(e) => err(e) }
}

/// Use an HLE profile (JSON) for machines created from now on; an empty string goes back to automatic choice among
/// the built-in profiles. 0, or -1 with the message.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_set_profile(p: *const u8, n: usize) -> i32 {
    let json = text(p, n);
    if json.trim().is_empty() { PROFILE.with(|x| *x.borrow_mut() = None); return 0; }
    match w53ga_emu::hle::Profile::parse(&json) { Ok(pr) => { PROFILE.with(|x| *x.borrow_mut() = Some(pr)); 0 } Err(e) => err(e) }
}

/// Apply the chosen (or a matching built-in) HLE profile to the machine. Err if a chosen profile does not match the
/// flash image.
fn apply_profile(m: &mut Machine) -> Result<(), String> {
    let chosen = PROFILE.with(|x| x.borrow().clone());
    let p = match chosen { Some(p) => { p.verify(&m.cpu.mem.flash)?; Some(p) } None => w53ga_emu::hle::detect(&m.cpu.mem.flash) };
    m.set_profile(p.as_ref());
    Ok(())
}

fn install(m: Machine, power_on: bool) {
    EMU.with(|e| *e.borrow_mut() = Some(Emu { m, frame: vec![0; W * H], sd: Vec::new(), power_on, net: None, changes: Vec::new() }));
}

/// Load a saved state (as written by `emu_save_state` or the desktop front ends). 0 on success, -1 with the message.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_load_state(p: *const u8, n: usize) -> i32 {
    match Machine::load_state_from(bytes(p, n)) {
        Ok(mut m) => { m.configure_realtime(); install(m, false); 0 }
        Err(e) => err(e),
    }
}

/// Switch the unit on: the flash image and the controller image (a 1 MiB image or an update package carrying one),
/// from power-on reset; the SD card follows with `emu_sd_add`/`emu_sd_commit`. 0 on success, -1 with the message.
/// # Safety
/// `f`/`r` must point to `fl`/`rl` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_power_on(f: *const u8, fl: usize, r: *const u8, rl: usize) -> i32 {
    let image = match w53ga_emu::rl78::companion::Companion::firmware_image(bytes(r, rl), 0) { Ok(i) => i, Err(e) => return err(e) };
    match Machine::power_on(bytes(f, fl).to_vec()) {
        Ok(mut m) => {
            m.configure_realtime();
            if let Err(e) = apply_profile(&mut m) { return err(e); }
            let mut comp = w53ga_emu::rl78::companion::Companion::new(image);
            if let Some(s) = &m.board.scif7 { comp.sync(s); }
            m.board.companion = Some(Box::new(comp));
            install(m, true);
            0
        }
        Err(e) => err(e),
    }
}

/// Use the user's flash image with a loaded state. 0 if it is the one the state was saved with, 1 if it differs (used
/// anyway), -1 on error. The HLE profile is chosen again for this image.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_set_flash(p: *const u8, n: usize) -> i32 {
    let data = bytes(p, n);
    with(|e| {
        let f = &mut e.m.cpu.mem.flash;
        if f.len() != data.len() { return err(format!("flash image is {} bytes, expected {}", data.len(), f.len())); }
        let same = f[..] == data[..];
        f.copy_from_slice(data);
        if let Err(x) = apply_profile(&mut e.m) { return err(x); }
        if same { 0 } else { 1 }
    }).unwrap_or_else(|| err("no state loaded"))
}

/// Start a new SD card file list.
#[no_mangle]
pub extern "C" fn emu_sd_clear() { with(|e| e.sd.clear()); }

/// Add a card file (path relative to the card root, e.g. "FOLDER/FILE.BIN"); `extra` places it after the others.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_sd_add(p: *const u8, n: usize, size: f64, extra: i32) { let path = text(p, n); with(|e| e.sd.push((path, size as u64, extra != 0))); }

/// Build the card from the listed files and serve it through `js_read`. 0 on success; skipped names in the buffer.
#[no_mangle]
pub extern "C" fn emu_sd_commit() -> i32 {
    w53ga_emu::periph::sdhi::set_file_reader(Some(Box::new(|path: &str, off: u64, buf: &mut [u8]| {
        let r = unsafe { js_read(path.as_ptr(), path.len(), off as f64, buf.as_mut_ptr(), buf.len()) };
        if r == 0 { Ok(()) } else { Err(format!("SD read {path} @{off} failed")) }
    })));
    with(|e| match w53ga_emu::fat32::Volume::from_list(&e.sd) {
        Ok((vol, skipped)) => {
            let r = if e.power_on {
                w53ga_emu::checkpoint::Checkpoint::power_on().and_then(|c| e.m.board.attach_sd_volume(&c, vol))
            } else { e.m.board.replace_sd_volume(vol) };
            match r { Ok(()) => { set_buf(skipped.join("\n").into_bytes()); 0 } Err(x) => err(x) }
        }
        Err(x) => err(x),
    }).unwrap_or_else(|| err("no state loaded"))
}

/// Give the card a CID (32 hex digits; empty keeps the current one). 0, or -1 with the message.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_sd_cid(p: *const u8, n: usize) -> i32 {
    let s = text(p, n);
    if s.trim().is_empty() { return 0; }
    match w53ga_emu::periph::sdhi::parse_cid(&s) {
        Ok(cid) => with(|e| match e.m.board.set_card_cid(cid) { Ok(()) => 0, Err(x) => err(x) }).unwrap_or_else(|| err("no state loaded")),
        Err(x) => err(x),
    }
}

/// What the software changed on the card since it was built, as JSON in the buffer:
/// `[{"op":"mkdir"|"write"|"delete","path":"...","existed":bool,"size":N}, ...]`; the bytes of change `i` come from
/// `emu_sd_change_data(i)`. Returns the number of changes, or -1 with the message.
#[no_mangle]
pub extern "C" fn emu_sd_changes() -> i32 {
    with(|e| {
        let Some(s) = e.m.board.sdhi.as_mut() else { return err("no SD card") };
        match w53ga_emu::sdsync::changes(&mut s.card) {
            Ok(c) => {
                let list: Vec<serde_json::Value> = c.iter().map(|x| match x {
                    Change::Mkdir(p) => serde_json::json!({"op": "mkdir", "path": p}),
                    Change::Write { path, data, existed } => serde_json::json!({"op": "write", "path": path, "existed": existed, "size": data.len()}),
                    Change::Delete(p) => serde_json::json!({"op": "delete", "path": p}),
                }).collect();
                let n = c.len() as i32;
                e.changes = c;
                set_buf(serde_json::Value::Array(list).to_string().into_bytes());
                n
            }
            Err(x) => err(x),
        }
    }).unwrap_or_else(|| err("no state loaded"))
}

/// The new content of change `i` from the last `emu_sd_changes` in the buffer; returns its length.
#[no_mangle]
pub extern "C" fn emu_sd_change_data(i: usize) -> usize {
    with(|e| match e.changes.get(i) { Some(Change::Write { data, .. }) => set_buf(data.clone()), _ => set_buf(Vec::new()) }).unwrap_or(0)
}

/// Run for `ms` milliseconds of emulated time. 0, or -1 with the stop reason.
#[no_mangle]
pub extern "C" fn emu_run_ms(ms: f64) -> i32 {
    with(|e| match e.m.run((ms / 1000.0 * P0_HZ) as u64) { Ok(()) => 0, Err(x) => err(format!("{x:?}")) }).unwrap_or_else(|| err("no state loaded"))
}

/// The panel in landscape as 400 x 240 pixels 0x00RRGGBB.
#[no_mangle]
pub extern "C" fn emu_render() -> *const u32 {
    with(|e| { e.m.render_landscape(&mut e.frame, W, H); e.frame.as_ptr() }).unwrap_or(std::ptr::null())
}

/// Remote button (NEC command: up 0x00, down 0x01, left 0x02, right 0x08, ENT 0x09, BACK 0x0A, ...).
#[no_mangle]
pub extern "C" fn emu_key(cmd: u32) { with(|e| { let now = e.m.steps; e.m.board.ir.press(now, cmd as u8, 0); }); }

/// GPS route "lat,lon;lat,lon;..." at `speed` km/h (`hold`: stop at the end instead of driving back and forth).
/// A single point places the vehicle there.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_gps_route(p: *const u8, n: usize, speed: f64, hold: i32) -> i32 {
    let s = text(p, n);
    let wp: Vec<(f64, f64)> = s.split(';').filter_map(|q| { let (a, b) = q.split_once(',')?; Some((a.trim().parse().ok()?, b.trim().parse().ok()?)) }).collect();
    if wp.is_empty() { return err("no waypoints"); }
    with(|e| {
        let Some(c) = e.m.board.companion.as_mut() else { return err("no controller") };
        match c.gnss.as_mut() {
            Some(g) => g.reroute(wp, Some(speed)),
            None => { let mut g = w53ga_emu::gnss::Gnss::new(wp, speed); g.start_after = c.now() + (P0_HZ / 10.0) as u64; c.gnss = Some(g); }
        }
        if let Some(g) = c.gnss.as_mut() { g.hold_at_end = hold != 0; }
        0
    }).unwrap_or_else(|| err("no state loaded"))
}

/// Change the GPS speed (km/h) without moving the vehicle.
#[no_mangle]
pub extern "C" fn emu_gps_speed(speed: f64) { with(|e| if let Some(g) = e.m.board.companion.as_mut().and_then(|c| c.gnss.as_mut()) { g.speed_kmh = speed.max(0.0); }); }

/// Plug the OBD-II adapter / drive recorder in or out.
#[no_mangle]
pub extern "C" fn emu_accessories(obd2: i32, dvr: i32) {
    with(|e| {
        let b = &mut e.m.board;
        if (obd2 != 0) != b.obd2.is_some() { b.obd2 = if obd2 != 0 { Some(w53ga_emu::obd2::Obd2::new(b.scif6.as_ref().map_or(0, |s| s.tx_log.len()))) } else { None }; }
        if (dvr != 0) != b.dvr.is_some() { b.dvr = if dvr != 0 { Some(w53ga_emu::dvr::Dvr::new(b.scif1.as_ref().map_or(0, |s| s.tx_log.len()))) } else { None }; }
        b.camera = dvr != 0;
    });
}

/// Fit the wireless LAN module with a virtual access point "SSID[:passphrase][#MAC]" ("*" accepts any key), whose
/// network reaches the internet through the local server's /net relay; an empty string removes the module.
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_wifi(p: *const u8, n: usize) {
    let spec = text(p, n);
    with(|e| {
        if spec.is_empty() { e.m.board.sdhi1.card = None; e.net = None; return; }
        let q = w53ga_emu::usernet::QueueHost::default();
        let (ssid, pass, mac) = w53ga_emu::wlan::parse_spec(&spec);
        let mut w = w53ga_emu::wlan::with_host(ssid, pass, w53ga_emu::usernet::UserNet::new(Box::new(q.clone())));
        if let Some(m) = mac { w.mac = m; }
        e.m.board.sdhi1.card = Some(Box::new(w));
        e.net = Some(q);
    });
}

/// Network requests queued since the last call, as relay JSON in the buffer (length returned; 0 when Wi-Fi is off).
#[no_mangle]
pub extern "C" fn emu_net_take() -> usize {
    let cmds = with(|e| e.net.as_ref().map(|q| std::mem::take(&mut *q.cmds.lock().unwrap()))).flatten().unwrap_or_default();
    set_buf(w53ga_emu::usernet::cmds_to_json(&cmds).into_bytes())
}

/// Events from the relay (JSON).
/// # Safety
/// `p` must point to `n` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn emu_net_give(p: *const u8, n: usize) {
    let ev = w53ga_emu::usernet::events_from_json(&text(p, n));
    with(|e| if let Some(q) = &e.net { q.events.lock().unwrap().extend(ev); });
}

/// Engine values reported by the OBD-II adapter (`follow_gps`: road speed from the GPS, engine speed from the gearbox).
#[no_mangle]
#[allow(clippy::too_many_arguments)]
pub extern "C" fn emu_obd(speed: f64, rpm: f64, coolant: f64, intake: f64, throttle: f64, load: f64, boost: f64, battery: f64, follow_gps: i32) {
    with(|e| if let Some(o) = e.m.board.obd2.as_mut() {
        let c = &mut o.car;
        c.follow_gps = follow_gps != 0;
        if !c.follow_gps { c.speed_kmh = speed; c.rpm = rpm; }
        c.coolant_c = coolant; c.intake_c = intake; c.throttle_pct = throttle; c.load_pct = load; c.boost_kpa = boost; c.battery_v = battery;
    });
}

/// Audio produced since the last call: interleaved stereo i16 at 44.1 kHz in the buffer; returns the frame count.
#[no_mangle]
pub extern "C" fn emu_audio() -> usize {
    with(|e| {
        let Some(s) = e.m.board.ssi.as_mut() else { return 0 };
        let frames: Vec<(i16, i16)> = s.played.drain(..).collect();
        let mut b = Vec::with_capacity(frames.len() * 4);
        for (l, r) in &frames { b.extend(l.to_le_bytes()); b.extend(r.to_le_bytes()); }
        set_buf(b);
        frames.len()
    }).unwrap_or(0)
}

/// Status as JSON in the buffer (time, GPS fix, accessories, Wi-Fi, HLE profile, SD card writes).
#[no_mangle]
pub extern "C" fn emu_status() -> usize {
    let s = with(|e| {
        let m = &e.m;
        let gps = m.board.companion.as_ref().and_then(|c| c.gnss.as_ref()).map(|g| { let f = g.current(); serde_json::json!({"lat": f.lat, "lon": f.lon, "speed": g.speed_kmh, "heading": f.heading}) });
        let obd = m.board.obd2.as_ref().map(|o| serde_json::json!({"streaming": o.streaming, "rpm": o.car.rpm, "speed": o.car.speed_kmh}));
        let wifi = m.board.sdhi1.card.as_ref().map(|w| serde_json::json!({"connected": w.connected.map(|i| w.aps[i].ssid.clone()), "frames": w.frames_in + w.frames_out}));
        let sd_written = m.board.sdhi.as_ref().map_or(0, |s| s.card.overlay.len());
        serde_json::json!({"t": m.steps as f64 / P0_HZ, "gps": gps, "obd2": obd, "dvr": m.board.dvr.as_ref().map(|d| d.polls > 0), "camera": m.board.camera, "wifi": wifi,
            "profile": m.hooks.profile, "sdWritten": sd_written})
    }).unwrap_or(serde_json::json!({"error": "no state"}));
    set_buf(s.to_string().into_bytes())
}

/// Save the complete state into the buffer; returns its length (0 on failure, with the message in the buffer).
#[no_mangle]
pub extern "C" fn emu_save_state() -> usize {
    with(|e| { let mut v = Vec::new(); match e.m.save_state_to(&mut v) { Ok(()) => set_buf(v), Err(x) => { err(x); 0 } } }).unwrap_or(0)
}
