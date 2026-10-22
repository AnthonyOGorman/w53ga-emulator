//! Headless front end: runs the unit without a window for a set time, then writes screenshots, a saved state, audio,
//! or diagnostics. Useful for scripting, regression checks and producing screenshots.
//!
//! usage: w53ga-headless --flash FLASH.bin (--rl78 CONTROLLER.bin | --state SAVED.state) [--sd FOLDER] --seconds S [options]
//!        w53ga-headless --flash FLASH.bin --make-profile OUT.json --name NAME [--idle ADDR] [--rgah A,B,C,D,E]

use std::path::{Path, PathBuf};
use std::time::Instant;
use w53ga_emu::machine::{rotate_ccw, write_bmp, Machine};
use w53ga_emu::session::{Options, OPTIONS_HELP};

const P0_HZ: f64 = 100.0e6 / 3.0;

fn usage() -> ! {
    eprintln!("usage: w53ga-headless --flash FLASH.bin (--rl78 CONTROLLER.bin | --state SAVED.state) --seconds S [options]\n\n{OPTIONS_HELP}\n  --seconds S             emulated time to run (default 10)\n  --frame FILE.bmp        write the screen at the end (landscape, as the unit shows it)\n  --every S               with --frame: also write FILE-<t>.bmp every S emulated seconds\n  --ir CODE@S,...         remote presses: command code (hex) at emulated second S, e.g. 09@60,01@62\n  --save-state FILE       save the machine at the end (contains the flash image and RAM: keep it private)\n  --wav FILE              record the audio output\n  --dump-mem ADDR,LEN,FILE  write main-CPU memory (hex address and length) at the end\n  --save-flash FILE       write the NOR flash as it is at the end\n  --dump-rspi FILE        write the bytes sent on RSPI0 and RSPI1 (hex, one line per channel), e.g. the panel set-up\n  --dump-vdc FILE         write every VDC4 register (address, name, shadow and active value) at the end\n\nDiagnostics (environment): AR_MMIO_LOG=FILE logs every MMIO store; AR_LINK_LOG=1 (or full) logs the controller link.\n\nProfile writer (checks computed from the flash image; addresses in hex):\n  w53ga-headless --flash FLASH.bin --make-profile OUT.json --name NAME [--idle ADDR] [--rgah FILL,DRAW,BLT,BLTFC,FINISH]");
    std::process::exit(2)
}

fn hex(s: &str) -> Result<u32, String> { u32::from_str_radix(s.trim().trim_start_matches("0x").replace('_', "").as_str(), 16).map_err(|_| format!("not a hex number: {s}")) }

fn make_profile(args: &[String], opt: &dyn Fn(&str) -> Option<String>) -> Result<(), String> {
    use w53ga_emu::hle::{Entry, Profile, Rgah};
    let flash_path = opt("--flash").ok_or("--flash FILE is required")?;
    let flash = std::fs::read(&flash_path).map_err(|e| format!("{flash_path}: {e}"))?;
    let out = opt("--make-profile").ok_or("--make-profile OUT.json")?;
    let entry = |a: u32| Entry::with_check(&flash, a).ok_or_else(|| format!("{a:#x} is outside the flash image"));
    let rgah = match opt("--rgah") {
        Some(list) => {
            let a: Vec<u32> = list.split(',').map(hex).collect::<Result<_, _>>()?;
            if a.len() != 5 { return Err("--rgah needs five addresses: fillRect, drawImage, bitBlt, bitBltFixedColor, finish".into()); }
            Some(Rgah { fill_rect: entry(a[0])?, draw_image: entry(a[1])?, bit_blt: entry(a[2])?, bit_blt_fixed_color: entry(a[3])?, finish: entry(a[4])? })
        }
        None => None,
    };
    let idle_loop = opt("--idle").map(|a| hex(&a).and_then(entry)).transpose()?;
    let p = Profile { name: opt("--name").unwrap_or_else(|| "profile".into()), description: None, idle_loop, rgah };
    let _ = args;
    std::fs::write(&out, serde_json::to_string_pretty(&p).map_err(|e| e.to_string())? + "\n").map_err(|e| format!("{out}: {e}"))?;
    println!("wrote {out}");
    Ok(())
}

fn save_frame(m: &Machine, path: &Path) {
    let Some((w, h, px)) = m.compose_rgba() else { println!("no display layer active yet; {} not written", path.display()); return };
    let (w, h, px) = rotate_ccw(w, h, &px);
    match write_bmp(path, w, h, &px) { Ok(()) => println!("frame {} ({w}x{h}) at t={:.2}s", path.display(), m.steps as f64 / P0_HZ), Err(e) => println!("{}: {e}", path.display()) }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 || args.iter().any(|a| a == "-h" || a == "--help") { usage(); }
    let opt = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    if opt("--make-profile").is_some() {
        if let Err(e) = make_profile(&args, &opt) { eprintln!("error: {e}"); std::process::exit(1); }
        return;
    }
    let options = Options::parse(&args).unwrap_or_else(|e| { eprintln!("{e}"); usage() });
    let mut m = options.build().unwrap_or_else(|e| { eprintln!("error: {e}"); std::process::exit(1) });
    let seconds: f64 = opt("--seconds").map_or(10.0, |s| s.parse().unwrap_or_else(|_| usage()));
    let frame = opt("--frame").map(PathBuf::from);
    let every: Option<f64> = opt("--every").map(|s| s.parse().unwrap_or_else(|_| usage()));
    let mut presses: Vec<(u64, u8)> = opt("--ir").map(|s| s.split(',').map(|p| {
        let (c, at) = p.split_once('@').unwrap_or_else(|| usage());
        let t: f64 = at.parse().unwrap_or_else(|_| usage());
        ((t * P0_HZ) as u64, u8::from_str_radix(c.trim_start_matches("0x"), 16).unwrap_or_else(|_| usage()))
    }).collect()).unwrap_or_default();
    presses.sort();
    let wav = opt("--wav").map(PathBuf::from);
    if wav.is_some() { if let Some(s) = &mut m.board.ssi { s.capture = Some(Vec::new()); } }

    let t0 = Instant::now();
    let start = m.steps;
    let end = start + (seconds * P0_HZ) as u64;
    let every_steps = every.map(|e| ((e * P0_HZ) as u64).max(1));
    let mut next_frame = every_steps.map_or(u64::MAX, |e| start + e);
    let mut result = Ok(());
    while m.steps < end {
        while let Some(&(at, cmd)) = presses.first() { if at <= m.steps { m.board.ir.press(m.steps, cmd, 0); println!("remote 0x{cmd:02x} at t={:.2}s", m.steps as f64 / P0_HZ); presses.remove(0); } else { break; } }
        let next_press = presses.first().map_or(u64::MAX, |p| p.0);
        let stop = end.min(next_frame).min(next_press.max(m.steps + 1));
        result = m.run(stop - m.steps);
        if result.is_err() { break; }
        if m.steps >= next_frame {
            if let Some(f) = &frame { let stem = f.file_stem().unwrap_or_default().to_string_lossy(); save_frame(&m, &f.with_file_name(format!("{stem}-{:07.2}.bmp", m.steps as f64 / P0_HZ))); }
            next_frame += every_steps.unwrap_or(u64::MAX);
        }
    }
    let secs = t0.elapsed().as_secs_f64();
    w53ga_emu::machine::mmio_log_flush();
    let done = (m.steps - start) as f64 / P0_HZ;
    println!("ran {done:.2} s of emulated time in {secs:.2} s ({:.2}x real time), {} interrupts", done / secs.max(1e-9), m.interrupts);
    if let Err(e) = &result { println!("STOPPED at t={:.3}s pc {:08x}: {e:?}\n  recent:{}", m.steps as f64 / P0_HZ, m.cpu.pc, m.dump_history()); }
    if m.gfx_hle && m.hooks.rgah_range.is_some() { println!("2D graphics: {} fills, {} images, {} blits, {} masked, {} pixels", m.gfx.fills, m.gfx.images, m.gfx.blits, m.gfx.masks, m.gfx.pixels); }
    if let Some(f) = m.detected() {
        let at = |a: Option<u32>, t: u64| a.map_or("not seen".to_string(), |a| format!("{a:#x} (t={:.3}s)", t as f64 / P0_HZ));
        let names = ["fill_rect", "draw_image", "bit_blt", "bit_blt_fixed_color", "finish"];
        let rgah: Vec<String> = names.iter().enumerate().map(|(k, n)| format!("{n} {}", at(f.rgah[k], f.rgah_at[k]))).collect();
        println!("HLE detected at run time: idle_loop {}; {}", at(f.idle_loop, f.idle_at), rgah.join(", "));
    }
    if let Some(f) = &frame { save_frame(&m, f); }
    w53ga_emu::sdsync::sync_and_report(m.board.sdhi.as_mut().map(|s| &mut s.card));
    if let Some(p) = opt("--save-state") { match m.save_state(Path::new(&p)) { Ok(()) => println!("state saved: {p}"), Err(e) => println!("state not saved: {e}") } }
    if let Some(spec) = opt("--dump-mem") {
        let p: Vec<&str> = spec.splitn(3, ',').collect();
        if p.len() == 3 {
            match (hex(p[0]), hex(p[1])) {
                (Ok(a), Ok(n)) => {
                    let bytes: Vec<u8> = (0..n).map(|k| m.cpu.mem.load_bytes(a + k, 1).map_or(0, |b| b[0])).collect();
                    match std::fs::write(p[2], &bytes) { Ok(()) => println!("memory {a:08x}+{n:x} written to {}", p[2]), Err(e) => println!("{}: {e}", p[2]) }
                }
                _ => println!("--dump-mem ADDR,LEN,FILE (hex)"),
            }
        }
    }
    if let Some(p) = opt("--dump-rspi") {
        let txt: String = (0..2).map(|c| format!("{c}: {}\n", m.board.rspi_tx[c].iter().map(|b| format!("{b:02x}")).collect::<String>())).collect();
        match std::fs::write(&p, txt) { Ok(()) => println!("RSPI transmit log written to {p} ({} / {} bytes)", m.board.rspi_tx[0].len(), m.board.rspi_tx[1].len()), Err(e) => println!("{p}: {e}") }
    }
    if let (Some(p), Some(d)) = (opt("--dump-vdc"), &m.board.display) {
        let txt: String = d.vdc.regs.iter().map(|r| format!("{:08x} {:<12} {:<8} shadow {:08x} active {:08x}\n", r.address, r.name, r.group, r.shadow, r.active)).collect();
        match std::fs::write(&p, txt) { Ok(()) => println!("VDC4 registers written to {p}"), Err(e) => println!("{p}: {e}") }
    }
    if let Some(p) = opt("--save-flash") { match std::fs::write(&p, &m.cpu.mem.flash) { Ok(()) => println!("flash image written to {p}"), Err(e) => println!("{p}: {e}") } }
    if let (Some(path), Some(s)) = (&wav, &m.board.ssi) {
        if let Some(c) = &s.capture {
            let fs = (P0_HZ / s.p0_per_frame).round() as u32;
            let data = (c.len() * 4) as u32;
            let mut f = Vec::with_capacity(44 + c.len() * 4);
            f.extend(b"RIFF"); f.extend((36 + data).to_le_bytes()); f.extend(b"WAVEfmt "); f.extend(16u32.to_le_bytes()); f.extend(1u16.to_le_bytes()); f.extend(2u16.to_le_bytes());
            f.extend(fs.to_le_bytes()); f.extend((fs * 4).to_le_bytes()); f.extend(4u16.to_le_bytes()); f.extend(16u16.to_le_bytes()); f.extend(b"data"); f.extend(data.to_le_bytes());
            for (l, r) in c { f.extend(l.to_le_bytes()); f.extend(r.to_le_bytes()); }
            match std::fs::write(path, f) { Ok(()) => println!("wrote {} ({fs} Hz stereo, {:.2} s)", path.display(), c.len() as f64 / fs as f64), Err(e) => println!("{}: {e}", path.display()) }
        }
    }
    if result.is_err() { std::process::exit(1); }
}
