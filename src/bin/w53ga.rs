//! Desktop front end: a window with the unit's screen, the keyboard as the IR remote, audio out and a GPS feed,
//! paced to real time.
//!
//! usage: w53ga --flash FLASH.bin --rl78 CONTROLLER.bin --sd SDCARD-FOLDER [options]
//!        w53ga --flash FLASH.bin --state SAVED.state [--sd SDCARD-FOLDER] [options]
//!
//! Keys: arrows = remote up/down/left/right, Enter = ENT, Backspace = BACK, F1..F5 = the other remote buttons
//! (codes 0x10, 0x11, 0x12, 0x64, 0xFF), +/- = GPS speed ±10 km/h, G = GPS stop/go, R = radar off/weak/medium/strong,
//! T = tunnel (GPS sky blocked, the drive goes on), P = pause, S = screenshot (BMP), F9 = save state, Esc = quit.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use minifb::{Key, KeyRepeat, Window, WindowOptions};
use std::collections::VecDeque;
use std::path::Path;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use w53ga_emu::machine::{rotate_ccw, write_bmp};
use w53ga_emu::session::{Options, OPTIONS_HELP};

const P0_HZ: f64 = 100.0e6 / 3.0;
const W: usize = 400;
const H: usize = 240;

fn usage() -> ! {
    eprintln!("usage: w53ga --flash FLASH.bin --rl78 CONTROLLER.bin [--sd FOLDER] [options]\n       w53ga --flash FLASH.bin --state SAVED.state [options]\n\n{OPTIONS_HELP}\n  --scale N               window scale (default 2)\n  --unpaced               run as fast as possible instead of real time\n  --save-flash FILE       on exit, write the NOR flash as it is then (e.g. after a software update)\n\nKeys: arrows/Enter/Backspace = remote, F1-F5 = other remote buttons, +/- GPS speed, G GPS stop/go, R radar off/weak/medium/strong, T tunnel,\n      P pause, S screenshot, F9 save state, Esc quit.");
    std::process::exit(2)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 || args.iter().any(|a| a == "-h" || a == "--help") { usage(); }
    let opt = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    let has = |name: &str| args.iter().any(|a| a == name);
    let scale: usize = opt("--scale").map_or(2, |s| s.parse().unwrap_or(2)).clamp(1, 8);
    let options = Options::parse(&args).unwrap_or_else(|e| { eprintln!("{e}"); usage() });
    let mut m = options.build().unwrap_or_else(|e| { eprintln!("error: {e}"); std::process::exit(1) });
    let paced = !has("--unpaced");
    println!("running (arrows/Enter/Backspace = remote, F1-F5 buttons, +/- GPS speed, G GPS stop/go, T tunnel, P pause, S screenshot, F9 save state, Esc quit)");

    // Audio: SSI0 frames are pushed into a shared queue; the output callback resamples to the device rate.
    let audio: Arc<Mutex<VecDeque<(i16, i16)>>> = Arc::new(Mutex::new(VecDeque::new()));
    let _stream = start_audio(audio.clone());

    let mut window = Window::new("AR-W53GA emulator", W * scale, H * scale, WindowOptions::default()).expect("window");
    window.set_target_fps(0);
    let mut buf = vec![0u32; W * H];
    let mut paused = false;
    let mut status = String::new();
    let (mut shot, mut saves) = (0, 0);
    let mut gps_stopped_speed: Option<f64> = None;
    // Real-time pacing: emulated time is kept in step with the host clock (catching up at most 100 ms at a time).
    let mut clock = Instant::now();
    let mut emu_at_clock = m.steps;
    let mut last_title = Instant::now();
    let mut last_present = Instant::now() - Duration::from_secs(1);
    let (mut title_steps, mut title_t) = (m.steps, Instant::now());
    while window.is_open() && !window.is_key_down(Key::Escape) {
        for k in window.get_keys_pressed(KeyRepeat::No) {
            let cmd = match k {
                Key::Up => Some(0x00), Key::Down => Some(0x01), Key::Left => Some(0x02), Key::Right => Some(0x08),
                Key::Enter => Some(0x09), Key::Backspace => Some(0x0A), Key::F1 => Some(0x10), Key::F2 => Some(0x11),
                Key::F3 => Some(0x12), Key::F4 => Some(0x64), Key::F5 => Some(0xFF), _ => None,
            };
            if let Some(cmd) = cmd { m.board.ir.press(m.steps, cmd, 0); }
            match k {
                Key::P => { paused = !paused; clock = Instant::now(); emu_at_clock = m.steps; }
                Key::S => if let Some((w, h, px)) = m.compose_rgba() {
                    let (w, h, px) = rotate_ccw(w, h, &px);
                    let name = format!("screenshot-{shot:03}.bmp");
                    match write_bmp(Path::new(&name), w, h, &px) { Ok(()) => println!("saved {name}"), Err(e) => println!("screenshot not saved: {e}") }
                    shot += 1;
                },
                Key::F9 => { let name = format!("w53ga-{saves:03}.state"); match m.save_state(Path::new(&name)) { Ok(()) => println!("saved {name}"), Err(e) => println!("save failed: {e}") } saves += 1; }
                Key::Equal | Key::NumPadPlus | Key::Minus | Key::NumPadMinus | Key::G => if let Some(g) = m.board.companion.as_mut().and_then(|c| c.gnss.as_mut()) {
                    match k {
                        Key::G => { if let Some(v) = gps_stopped_speed.take() { g.speed_kmh = v; } else { gps_stopped_speed = Some(g.speed_kmh); g.speed_kmh = 0.0; } }
                        Key::Minus | Key::NumPadMinus => g.speed_kmh = (g.speed_kmh - 10.0).max(0.0),
                        _ => g.speed_kmh += 10.0,
                    }
                    println!("GPS speed {} km/h", g.speed_kmh);
                },
                // a radar the unit can hear, as if a speed trap were ahead: off -> weak -> medium -> strong -> off
                Key::R => if let Some(c) = m.board.companion.as_mut() {
                    let r = &mut c.radar.manual;
                    *r = match *r { x if x < 0.3 => 0.35, x if x < 0.6 => 0.7, x if x < 0.9 => 1.0, _ => 0.0 };
                    println!("radar {}", if *r == 0.0 { "off".to_string() } else { format!("strength {:.0}%", *r * 100.0) });
                },
                Key::T => if let Some(g) = m.board.companion.as_mut().and_then(|c| c.gnss.as_mut()) {
                    g.cut = !g.cut;
                    println!("{}", if g.cut { "tunnel: GPS sky blocked (no fix), the drive goes on" } else { "tunnel exit: GPS sky clear" });
                },
                _ => {}
            }
        }
        if !paused && status.is_empty() {
            // Emulate up to the host clock (or one 1/60 s slice when unpaced), bounded by a 25 ms host budget.
            let target = if paced { emu_at_clock + (clock.elapsed().as_secs_f64() * P0_HZ) as u64 } else { m.steps + (P0_HZ / 60.0) as u64 };
            if target > m.steps + (P0_HZ / 10.0) as u64 { emu_at_clock = m.steps; clock = Instant::now(); } // fell behind: drop the backlog
            let budget = Instant::now() + Duration::from_millis(25);
            while m.steps < target && Instant::now() < budget {
                let chunk = 50_000.min(target - m.steps);
                if let Err(e) = m.run(chunk) {
                    status = format!("STOPPED at step {}: {e:?}", m.steps);
                    println!("{status}\n  PC {:08x} PR {:08x}\n  recent:{}", m.cpu.pc, m.cpu.pr, m.dump_history());
                    match m.save_state(Path::new("w53ga-stopped.state")) {
                        Ok(()) => println!("machine state saved as w53ga-stopped.state (it contains your flash image and RAM: do not share it publicly)"),
                        Err(e) => println!("state not saved: {e}"),
                    }
                    break;
                }
            }
            if let Some(s) = &mut m.board.ssi {
                let mut q = audio.lock().unwrap();
                q.extend(s.played.drain(..));
                while q.len() > 44100 / 4 { q.pop_front(); }
            }
        }
        // Present at most 60 frames per second; between frames only poll input.
        if last_present.elapsed() >= Duration::from_micros(16_000) {
            m.render_landscape(&mut buf, W, H);
            window.update_with_buffer(&buf, W, H).expect("present");
            last_present = Instant::now();
        } else {
            window.update();
        }
        if paced && !paused {
            let ahead = m.steps as f64 / P0_HZ - (emu_at_clock as f64 / P0_HZ + clock.elapsed().as_secs_f64());
            if ahead > 0.002 { std::thread::sleep(Duration::from_secs_f64(ahead.min(0.02))); }
        }
        if last_title.elapsed() > Duration::from_millis(500) {
            let speed = (m.steps - title_steps) as f64 / P0_HZ / title_t.elapsed().as_secs_f64();
            let mut extra = m.board.companion.as_ref().and_then(|c| c.gnss.as_ref()).map_or(String::new(), |g| format!("  GPS {} km/h{}", g.speed_kmh, if g.sky_blocked(m.steps as f64 / P0_HZ) { " (tunnel: no fix)" } else { "" }));
            if let Some(c) = &m.board.companion { if c.g13.rx.radar > 0.0 { extra += &format!("  RADAR {:.0}%", c.g13.rx.radar * 100.0); } }
            if let Some(o) = &m.board.obd2 { extra += &format!("  OBD {:.0} rpm{}", o.car.rpm, if o.streaming { "" } else { " (idle)" }); }
            if let Some(d) = &m.board.dvr { extra += &format!("  DVR {}", if d.polls > 0 { "linked" } else { "waiting" }); }
            if let Some(w) = &m.board.sdhi1.card { extra += &format!("  Wi-Fi {}", w.connected.map_or("off-line".into(), |i| w.aps[i].ssid.clone())); }
            window.set_title(&format!("AR-W53GA emulator  {speed:.2}x real time  {:.1}s{extra}{}", m.steps as f64 / P0_HZ, if paused { "  [paused]" } else if !status.is_empty() { "  [stopped]" } else { "" }));
            last_title = Instant::now(); title_steps = m.steps; title_t = Instant::now();
        }
    }
    w53ga_emu::machine::mmio_log_flush();
    // What the software wrote to the SD card goes back to the SD folder (--sd-readonly: never).
    w53ga_emu::sdsync::sync_and_report(m.board.sdhi.as_mut().map(|s| &mut s.card));
    if let Some(p) = opt("--save-flash") {
        if m.board.nor.erases + m.board.nor.programs == 0 { println!("flash unchanged; {p} not written"); }
        else { match std::fs::write(&p, &m.cpu.mem.flash) { Ok(()) => println!("flash image written to {p} ({} erases, {} programs)", m.board.nor.erases, m.board.nor.programs), Err(e) => println!("flash image not written: {e}") } }
    }
}

fn start_audio(queue: Arc<Mutex<VecDeque<(i16, i16)>>>) -> Option<cpal::Stream> {
    let host = cpal::default_host();
    let dev = host.default_output_device()?;
    let cfg = dev.default_output_config().ok()?;
    let rate = cfg.sample_rate().0 as f64;
    let channels = cfg.channels() as usize;
    let step = 44100.0 / rate;
    let mut phase = 0.0f64;
    let mut cur = (0i16, 0i16);
    let err = |e| eprintln!("audio error: {e}");
    let stream = match cfg.sample_format() {
        cpal::SampleFormat::F32 => dev.build_output_stream(&cfg.into(), move |data: &mut [f32], _| {
            let mut q = queue.lock().unwrap();
            for frame in data.chunks_mut(channels) {
                phase += step;
                while phase >= 1.0 { phase -= 1.0; cur = q.pop_front().unwrap_or((0, 0)); }
                for (k, s) in frame.iter_mut().enumerate() { *s = (if k == 0 { cur.0 } else { cur.1 }) as f32 / 32768.0; }
            }
        }, err, None).ok()?,
        cpal::SampleFormat::I16 => dev.build_output_stream(&cfg.into(), move |data: &mut [i16], _| {
            let mut q = queue.lock().unwrap();
            for frame in data.chunks_mut(channels) {
                phase += step;
                while phase >= 1.0 { phase -= 1.0; cur = q.pop_front().unwrap_or((0, 0)); }
                for (k, s) in frame.iter_mut().enumerate() { *s = if k == 0 { cur.0 } else { cur.1 }; }
            }
        }, err, None).ok()?,
        _ => return None,
    };
    stream.play().ok()?;
    Some(stream)
}
