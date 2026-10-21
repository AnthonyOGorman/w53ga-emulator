//! Runtime HLE detection (`w53ga_emu::detect`) on a small hand-assembled program, without any software from the unit:
//! a "library" with a drawing call, a driver function that writes the graphics core's command port and a `finish`
//! that then waits forever (as the real one does, the graphics core not being emulated), called by a main program.
//! The same program is placed at two different addresses: detection finds it wherever it is.

use w53ga_emu::detect::Found;
use w53ga_emu::hle::{Entry, Profile, Rgah};
use w53ga_emu::machine::Machine;
use w53ga_emu::mem::FLASH_SIZE;

const PIXELS: u32 = 0x1C00_1000;
const MARKER: u32 = 0x1C00_0100;
const STACK: u32 = 0x1C20_0000;

/// MOV.L @(disp,PC),Rn at `at` loading the literal at `lit`.
fn movl_pc(at: u32, lit: u32, rn: u16) -> u16 {
    let base = (at & !3) + 4;
    assert!(lit >= base && lit % 4 == 0 && (lit - base) / 4 < 256);
    0xD000 | (rn << 8) | ((lit - base) / 4) as u16
}

struct Image { flash: Vec<u8> }

impl Image {
    fn new() -> Image { Image { flash: vec![0; FLASH_SIZE] } }
    fn w16(&mut self, a: u32, v: u16) { self.flash[a as usize..a as usize + 2].copy_from_slice(&v.to_be_bytes()); }
    fn w32(&mut self, a: u32, v: u32) { self.flash[a as usize..a as usize + 4].copy_from_slice(&v.to_be_bytes()); }
    fn code(&mut self, at: u32, ops: &[u16]) { for (i, &op) in ops.iter().enumerate() { self.w16(at + 2 * i as u32, op); } }
    fn words(&mut self, at: u32, ws: &[u32]) { for (i, &w) in ws.iter().enumerate() { self.w32(at + 4 * i as u32, w); } }
}

/// Addresses of the program's parts when it is placed at `base`.
struct Layout { main: u32, fill: u32, finish: u32, driver: u32 }

fn program(base: u32) -> (Vec<u8>, Layout) {
    let l = Layout { main: base, fill: base + 0x100, finish: base + 0x140, driver: base + 0x180 };
    let (bmp, pt, eff) = (base + 0x200, base + 0x220, base + 0x230);
    let mut im = Image::new();
    im.words(0, &[l.main, STACK]);
    // main: fillRect(bmp, &pt, 8, 8, color, 255, &effect); finish(); *MARKER = 0x600DF00D; for (;;);
    let lit = base + 0x40;
    let m = l.main;
    im.code(m, &[
        movl_pc(m, lit, 4), movl_pc(m + 2, lit + 4, 5), 0xE608, 0xE708,
        movl_pc(m + 8, lit + 8, 0), 0x2F06, movl_pc(m + 12, lit + 12, 0), 0x2F06, movl_pc(m + 16, lit + 16, 0), 0x2F06,
        movl_pc(m + 20, lit + 20, 1), 0x410B, 0x0009, 0x7F0C,
        movl_pc(m + 28, lit + 24, 1), 0x410B, 0x0009,
        movl_pc(m + 34, lit + 28, 1), movl_pc(m + 36, lit + 32, 0), 0x2102, 0xAFFE, 0x0009,
    ]);
    im.words(lit, &[bmp, pt, eff, 255, 0xFFFF_0000, l.fill, l.finish, MARKER, 0x600D_F00D]);
    // fillRect: driver(command); return 0
    let f = l.fill;
    im.code(f, &[0x4F22, movl_pc(f + 2, f + 0x10, 1), movl_pc(f + 4, f + 0x14, 4), 0x410B, 0x0009, 0x4F26, 0x000B, 0xE000]);
    im.words(f + 0x10, &[l.driver, 0xA002_0010]);
    // finish: driver(0); then wait forever for the graphics core
    let x = l.finish;
    im.code(x, &[0x4F22, movl_pc(x + 2, x + 0x10, 1), 0xE400, 0x410B, 0x0009, 0xAFFE, 0x0009]);
    im.words(x + 0x10, &[l.driver]);
    // driver: *COMMAND_PORT = r4
    let d = l.driver;
    im.code(d, &[movl_pc(d, d + 8, 1), 0x2142, 0x000B, 0x0009]);
    im.words(d + 8, &[0xE810_4000]);
    // a 16x16 RGB565 frame buffer in RAM, the point (2, 3), an effect record (copy)
    im.words(bmp, &[16, 16, 0, 0, 16, 16, 0, PIXELS]);
    im.words(pt, &[2, 3]);
    im.words(eff, &[0, 0, 0, 0]);
    (im.flash, l)
}

fn machine(base: u32) -> (Machine, Layout) {
    let (flash, l) = program(base);
    let mut m = Machine::power_on(flash).expect("power on");
    m.gfx_hle = true;
    m.set_profile(None);
    (m, l)
}

fn pixel(m: &Machine, x: u32, y: u32) -> u64 { m.cpu.mem.read_plain(PIXELS + (y * 16 + x) * 2, 2).unwrap() }

#[test]
fn detection_finds_the_library_wherever_it_is_and_nothing_is_lost() {
    for base in [0x0001_0000, 0x0123_4000] {
        let (mut m, l) = machine(base);
        m.run(5_000).expect("runs");
        assert_eq!(m.cpu.mem.read_plain(MARKER, 4), Some(0x600D_F00D), "finish returned (base {base:#x})");
        let found = m.detected().expect("detection on").clone();
        assert_eq!(found.rgah[0], Some(l.fill), "fillRect at base {base:#x}");
        assert_eq!(found.rgah[4], Some(l.finish), "finish at base {base:#x}");
        assert_eq!(found.rgah[1..4], [None, None, None], "no other call was made");
        assert_eq!(found.range, Some((l.fill, l.driver)));
        // the very first call was drawn: an 8x8 red square at (2, 3)
        assert_eq!(pixel(&m, 2, 3), 0xF800);
        assert_eq!(pixel(&m, 9, 10), 0xF800);
        assert_eq!(pixel(&m, 1, 3), 0);
        assert_eq!(pixel(&m, 10, 10), 0);
        assert_eq!(m.gfx.fills, 1);
        assert_eq!(m.board.gpu.accesses, 0, "the exploration left the graphics core untouched");
    }
}

#[test]
fn detection_is_kept_in_saved_states() {
    let (mut m, l) = machine(0x0004_0000);
    m.run(5_000).expect("runs");
    let mut state = Vec::new();
    m.save_state_to(&mut state).expect("save");
    let mut back = Machine::load_state_from(&state[..]).expect("load");
    back.set_profile(None);
    assert_eq!(back.detected(), m.detected());
    assert_eq!(back.hooks.rgah_at(l.fill), Some(w53ga_emu::hle::RgahFn::FillRect));
}

#[test]
fn a_profile_overrides_detection() {
    let (mut m, l) = machine(0x0004_0000);
    let e = |a| Entry { addr: a, check: None, check_len: None };
    let p = Profile { name: "given".into(), rgah: Some(Rgah { fill_rect: e(0x10), draw_image: e(0x20), bit_blt: e(0x30), bit_blt_fixed_color: e(0x40), finish: e(0x50) }), ..Default::default() };
    m.set_profile(Some(&p));
    assert!(m.detected().is_none());
    assert!(m.cpu.trace.is_none());
    m.run(5_000).expect("runs");
    assert!(m.detected().is_none(), "nothing is detected while a profile is in use");
    assert_eq!(m.hooks.rgah_at(l.fill), None);
    assert_ne!(m.cpu.mem.read_plain(MARKER, 4), Some(0x600D_F00D), "without the right hooks finish never returns");
    m.disable_hle();
    assert!(m.detected().is_none() && m.hooks.rgah.is_empty());
}

#[test]
fn nothing_is_detected_without_drawing_calls() {
    // The same program, but the frame buffer descriptor is not a valid bitmap (unknown pixel format): no call matches.
    let (mut flash, _) = program(0x0001_0000);
    flash[0x0001_0200 + 24..0x0001_0200 + 28].copy_from_slice(&9u32.to_be_bytes());
    let mut m = Machine::power_on(flash).expect("power on");
    m.gfx_hle = true;
    m.set_profile(None);
    m.run(5_000).expect("runs");
    assert_eq!(m.detected(), Some(&Found::default()));
    assert!(m.hooks.rgah.is_empty());
}
