//! Tests of the emulator's building blocks that need no software from the unit: the SH-2A and RL78 cores on tiny
//! hand-assembled programs, the controller link codec, NMEA output, SD card helpers and the Wi-Fi option parser.

use w53ga_emu::cpu::{Cpu, Mmio, SR_T};
use w53ga_emu::mem::{Memory, FLASH_SIZE};

/// Bus without peripherals: reads return 0, writes are recorded.
#[derive(Default)]
struct NoIo { writes: Vec<(u32, usize, u64)>, ibnr: u16 }
impl Mmio for NoIo {
    fn read(&mut self, _pc: u32, _a: u32, _n: usize) -> u64 { 0 }
    fn write(&mut self, _pc: u32, a: u32, n: usize, v: u64) { self.writes.push((a, n, v)); }
    fn ibnr(&mut self) -> u16 { self.ibnr }
    fn set_ibnr(&mut self, v: u16) { self.ibnr = v; }
}

/// A CPU with `code` (big-endian 16-bit words) at 0x1000 of an otherwise empty flash, PC there.
fn sh2a(code: &[u16]) -> Cpu {
    let mut flash = vec![0u8; FLASH_SIZE];
    for (i, w) in code.iter().enumerate() { flash[0x1000 + 2 * i..0x1002 + 2 * i].copy_from_slice(&w.to_be_bytes()); }
    let mut c = Cpu::new(Memory::new(flash));
    c.pc = 0x1000;
    c
}

fn run_to(c: &mut Cpu, io: &mut NoIo, pc: u32, max: usize) {
    for _ in 0..max { if c.pc == pc { return; } c.step(io).expect("no fault"); }
    panic!("did not reach {pc:#x} (pc {:#x})", c.pc);
}

#[test]
fn sh2a_counted_loop() {
    // mov #0,r0; mov #10,r1; loop: add r1,r0; dt r1; bf loop; nop
    let mut c = sh2a(&[0xE000, 0xE10A, 0x301C, 0x4110, 0x8BFC, 0x0009]);
    let mut io = NoIo::default();
    run_to(&mut c, &mut io, 0x100A, 100);
    assert_eq!(c.r[0], 55);
    assert_eq!(c.r[1], 0);
    assert!(c.sr & SR_T != 0, "DT sets T when the count reaches zero");
}

#[test]
fn sh2a_delay_slot_runs_before_the_branch_target() {
    // bra +2 (to 0x1008); slot: mov #5,r1; (skipped) mov #0x63,r1; nop; target: add #1,r1
    let mut c = sh2a(&[0xA002, 0xE105, 0xE163, 0x0009, 0x7101, 0x0009]);
    let mut io = NoIo::default();
    run_to(&mut c, &mut io, 0x100A, 10);
    assert_eq!(c.r[1], 6);
}

#[test]
fn sh2a_pc_relative_load_store_and_sign_extension() {
    // mov.l @(1,pc),r2 (= 0x1C000010); mov #0x7B,r1; mov.l r1,@r2; mov.l @r2,r0; data; mov #-1,r3
    let mut c = sh2a(&[0xD201, 0xE17B, 0x2212, 0x6022, 0x1C00, 0x0010, 0xE3FF, 0x0009]);
    let mut io = NoIo::default();
    for _ in 0..4 { c.step(&mut io).unwrap(); }
    assert_eq!(c.r[2], 0x1C00_0010);
    assert_eq!(c.r[0], 0x7B);
    assert_eq!(c.mem.load_bytes(0x1C00_0010, 4).unwrap(), vec![0, 0, 0, 0x7B]);
    // the uncached mirror sees the same SDRAM
    assert_eq!(c.mem.read_plain(0x3C00_0010, 4), Some(0x7B));
    c.pc = 0x100C;
    c.step(&mut io).unwrap();
    assert_eq!(c.r[3], 0xFFFF_FFFF);
    assert!(io.writes.is_empty(), "no MMIO traffic for plain memory");
}

#[test]
fn sh2a_peripheral_stores_go_to_the_bus() {
    // mov.l @(1,pc),r2 (= 0xFFFE0800); mov #1,r1; mov.w r1,@r2; nop; data
    let mut c = sh2a(&[0xD201, 0xE101, 0x2211, 0x0009, 0xFFFE, 0x0800]);
    let mut io = NoIo::default();
    for _ in 0..3 { c.step(&mut io).unwrap(); }
    assert_eq!(io.writes, vec![(0xFFFE_0800, 2, 1)]);
}

#[test]
fn sh2a_misaligned_access_faults() {
    // mov #1,r2; mov.l @r2,r0
    let mut c = sh2a(&[0xE201, 0x6022]);
    let mut io = NoIo::default();
    c.step(&mut io).unwrap();
    assert!(matches!(c.step(&mut io), Err(w53ga_emu::cpu::Fault::Address { addr: 1, size: 4, write: false, .. })));
}

#[test]
fn rl78_reset_vector_and_arithmetic() {
    use w53ga_emu::rl78::{g13::G13, Rl78};
    let mut flash = vec![0xFFu8; 0x80000];
    flash[0..2].copy_from_slice(&0x00D8u16.to_le_bytes());
    // MOV A,#2AH; ADD A,#05H; NOP
    flash[0xD8..0xDD].copy_from_slice(&[0x51, 0x2A, 0x0C, 0x05, 0x00]);
    let mut cpu = Rl78::new(flash);
    assert_eq!(cpu.pc, 0xD8);
    let mut bus = G13::default();
    bus.step(&mut cpu).unwrap();
    bus.step(&mut cpu).unwrap();
    assert_eq!(cpu.r(1), 0x2F, "A");
    assert_eq!(cpu.pc, 0xDC);
    let rd = |a: u32| cpu.flash[a as usize];
    let (text, len) = w53ga_emu::rl78::dis::dis(&rd, 0xD8);
    assert_eq!(len, 2, "{text}");
}

#[test]
fn controller_link_frames_round_trip() {
    use w53ga_emu::periph::subcpu::{crc16, decode_peer, encode};
    let f = encode(7, 0x55, &[1, 2, 3, 4], 0x0102, 0x0304, 0x05);
    assert_eq!(f.len(), 4 + 16);
    let (frame, used) = decode_peer(&f).expect("complete").expect("valid");
    assert_eq!(used, f.len());
    assert_eq!((frame.seq, frame.command, frame.event, frame.state, frame.status), (7, 0x55, 0x0102, 0x0304, 0x05));
    assert_eq!(frame.payload, vec![1, 2, 3, 4]);
    let mut bad = f.clone();
    *bad.last_mut().unwrap() ^= 0xFF;
    assert!(matches!(decode_peer(&bad), Some(Err(_))), "a CRC error is rejected");
    assert!(decode_peer(&f[..10]).is_none(), "a partial frame waits for more bytes");
    assert_eq!(crc16(b"123456789"), 0x6F91, "CRC-16/X-25 without the final inversion");
}

#[test]
fn nmea_sentences_carry_their_checksum() {
    let s = w53ga_emu::gnss::sentence("GPGGA,123519,4807.038,N");
    assert!(s.starts_with("$GPGGA,") && s.ends_with("\r\n"));
    let (body, cs) = s.trim_end().trim_start_matches('$').split_once('*').unwrap();
    assert_eq!(u8::from_str_radix(cs, 16).unwrap(), body.bytes().fold(0, |a, b| a ^ b));
}

#[test]
fn gps_routes_parse() {
    let r = w53ga_emu::gnss::parse_route("35.68,139.76; 35.69 , 139.77").unwrap();
    assert_eq!(r, vec![(35.68, 139.76), (35.69, 139.77)]);
    assert!(w53ga_emu::gnss::parse_route("35.68").is_err());
}

#[test]
fn card_ids() {
    use w53ga_emu::periph::sdhi::{crc7, parse_cid, DEFAULT_CID};
    assert_eq!(DEFAULT_CID[15], (crc7(&DEFAULT_CID[..15]) << 1) | 1);
    let hex: String = DEFAULT_CID.iter().map(|b| format!("{b:02x}")).collect();
    assert_eq!(parse_cid(&hex).unwrap(), DEFAULT_CID);
    assert_eq!(parse_cid(&hex[..30]).unwrap(), DEFAULT_CID, "the CRC byte is completed");
    assert_eq!(parse_cid(&hex.to_uppercase()).unwrap(), DEFAULT_CID);
    assert!(parse_cid("12").is_err());
    assert!(parse_cid(&"zz".repeat(16)).is_err());
    // CRC7 of CMD0's argument bytes (0x40 00 00 00 00) is 0x4A
    assert_eq!(crc7(&[0x40, 0, 0, 0, 0]), 0x4A);
}

#[test]
fn card_volume_keeps_short_names_and_skips_long_ones() {
    let files = vec![("FOLDER/FILE.BIN".to_string(), 1000u64, false), ("A long name.txt".to_string(), 10, false), ("ROOT.TXT".to_string(), 0, false)];
    let (vol, skipped) = w53ga_emu::fat32::Volume::from_list(&files).unwrap();
    assert_eq!(skipped, vec!["A long name.txt".to_string()]);
    assert!(vol.sector_count > 0);
}

#[test]
fn wifi_option_parsing() {
    use w53ga_emu::wlan::{parse_mac, parse_spec};
    assert_eq!(parse_spec("Home:secret#00:11:22:33:44:55"), ("Home", "secret", Some([0, 0x11, 0x22, 0x33, 0x44, 0x55])));
    assert_eq!(parse_spec("Open"), ("Open", "", None));
    assert_eq!(parse_mac("00-11-22-33-44-5"), None);
}

#[test]
fn power_on_needs_a_full_size_flash_image() {
    let e = w53ga_emu::machine::Machine::power_on(vec![0; 1024]).err().expect("rejected");
    assert!(e.contains("32 MiB"), "{e}");
}

#[test]
fn controller_images_are_recognised_by_size() {
    use w53ga_emu::rl78::companion::Companion;
    let mut two_banks = vec![0u8; 0x100000];
    two_banks[0x80000] = 0xAB;
    assert_eq!(Companion::firmware_image(&two_banks, 1).unwrap()[0], 0xAB);
    let mut package = vec![0u8; 88];
    package.extend(&two_banks);
    assert_eq!(Companion::firmware_image(&package, 1).unwrap()[0], 0xAB);
    assert!(Companion::firmware_image(&[0; 100], 0).is_err());
}
