//! The RL78/G13 controller on its own (no main CPU): disassemble or run a controller image and report faults, hot PCs,
//! serial output and SFR accesses. A development tool for the RL78 core and G13 peripheral models.
//!
//! usage: rl78-run <image.bin> [--half 0|1] [--steps N] [--dis ADDR,COUNT] [--trace FROM,TO] [--trace-pc ADDR] [--watch ADDR,LEN]
//!   image: a 512 KiB code flash image, a 1 MiB two-bank image (--half picks the bank) or an update package with an
//!   88-byte header in front of the 1 MiB image. Addresses are hex.

use std::collections::HashMap;
use w53ga_emu::rl78::{companion::Companion, dis::dis, g13::G13, Rl78};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 || args.iter().any(|a| a == "-h" || a == "--help") {
        eprintln!("usage: rl78-run <image.bin> [--half 0|1] [--steps N] [--dis ADDR,COUNT] [--trace FROM,TO] [--trace-pc ADDR] [--watch ADDR,LEN]");
        std::process::exit(2);
    }
    let arg = |n: &str| args.iter().position(|a| a == n).and_then(|i| args.get(i + 1)).cloned();
    let half: usize = arg("--half").map_or(0, |s| s.parse().expect("--half"));
    let steps: u64 = arg("--steps").map_or(1_000_000, |s| s.replace('_', "").parse().expect("--steps"));
    let d = std::fs::read(&args[1]).unwrap_or_else(|e| { eprintln!("{}: {e}", args[1]); std::process::exit(1) });
    let flash = if d.len() == 0x80000 { d } else { Companion::firmware_image(&d, half).unwrap_or_else(|e| { eprintln!("{e}"); std::process::exit(1) }) };
    if let Some(r) = arg("--dis") {
        let (a, n) = r.split_once(',').expect("--dis ADDR,COUNT");
        let (mut a, n) = (u32::from_str_radix(a.trim_start_matches("0x"), 16).expect("hex address"), n.parse::<u32>().expect("count"));
        let rd = |x: u32| flash.get(x as usize).copied().unwrap_or(0xFF);
        for _ in 0..n { let (t, l) = dis(&rd, a); println!("{a:05x}: {:<12} {t}", (0..l).map(|i| format!("{:02x}", rd(a + i))).collect::<String>()); a += l; }
        return;
    }
    let mut cpu = Rl78::new(flash);
    let mut bus = G13::default();
    let names: HashMap<u32, String> = include_str!("../rl78/sfr_names.tsv").lines().filter_map(|l| { let (a, n) = l.split_once('\t')?; Some((u32::from_str_radix(a, 16).ok()?, n.to_string())) }).collect();
    if let Some(w) = arg("--watch") { let (a, n) = w.split_once(',').expect("--watch ADDR,LEN"); let a = u32::from_str_radix(a, 16).expect("hex"); bus.watch = Some((a, a + u32::from_str_radix(n, 16).expect("hex"))); }
    let mut hot: HashMap<u32, u64> = HashMap::new();
    println!("reset -> {:05x}", cpu.pc);
    let mut fault = None;
    let trace = arg("--trace").map(|t| { let (a, b) = t.split_once(',').expect("--trace FROM,TO"); (a.parse::<u64>().expect("step"), b.parse::<u64>().expect("step")) });
    let trace_pc = arg("--trace-pc").map(|t| u32::from_str_radix(&t, 16).expect("hex"));
    let mut tracing_left = 0u64;
    for _ in 0..steps {
        if Some(cpu.pc) == trace_pc { tracing_left = 200; }
        if trace.is_some_and(|(a, b)| cpu.steps >= a && cpu.steps < b) || tracing_left > 0 {
            tracing_left = tracing_left.saturating_sub(1);
            let f = &cpu.flash;
            let rd = |x: u32| f.get(x as usize).copied().unwrap_or(0xFF);
            let (t, _) = dis(&rd, cpu.pc);
            println!("{:>9} {:05x} {:<28} AX={:04x} BC={:04x} DE={:04x} HL={:04x} SP={:04x} PSW={:02x} ES={:x} CS={:x}", cpu.steps, cpu.pc, t, cpu.rp(0), cpu.rp(1), cpu.rp(2), cpu.rp(3), cpu.sp(), cpu.psw(), cpu.es(), cpu.cs());
        }
        *hot.entry(cpu.pc).or_default() += 1;
        bus.cur_pc = cpu.pc;
        if let Err(e) = bus.step(&mut cpu) { fault = Some(e); break; }
        bus.tick(cpu.cycles.max(1) as u64);
        if cpu.stopped { println!("STOP at step {} pc {:05x}", cpu.steps, cpu.pc); break; }
    }
    println!("steps {} pc {:05x} psw {:02x} sp {:04x} fault {:?}", cpu.steps, cpu.pc, cpu.psw(), cpu.sp(), fault);
    let mut h: Vec<_> = hot.into_iter().collect();
    h.sort_by_key(|x| std::cmp::Reverse(x.1));
    println!("hot PCs: {}", h.iter().take(16).map(|(p, n)| format!("{p:05x}:{n}")).collect::<Vec<_>>().join(" "));
    println!("interrupts: {}", bus.irq_count.iter().enumerate().filter(|x| *x.1 > 0).map(|(n, c)| format!("vec {:02x}:{c}", 4 + 2 * n)).collect::<Vec<_>>().join(" "));
    for k in 0..4 { let t = bus.uart_take_tx(k); if !t.is_empty() { println!("UART{k} tx {} bytes: {:02x?}", t.len(), &t[..t.len().min(200)]); } }
    println!("SFR accesses (addr name reads writes last):");
    for (a, (r, w, v)) in &bus.log { println!("  {a:05x} {:<12} r{r:<8} w{w:<6} {v:02x}", names.get(a).map_or("", |s| s.as_str())); }
    let rd = |x: u32| cpu.flash[x as usize & 0x7FFFF];
    let mut a = cpu.pc.saturating_sub(0x10);
    for _ in 0..12 { let (t, l) = dis(&rd, a); println!("{a:05x}: {t}"); a += l; }
}
