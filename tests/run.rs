//! `Machine::run` (instructions in batches through `Cpu::run`, flash code from decoded blocks) must do exactly what
//! calling `Machine::step` repeatedly does, including when the code in flash changes: reprogrammed through the NOR
//! command interface by the program itself, or edited from outside between runs.

use w53ga_emu::machine::Machine;
use w53ga_emu::mem::FLASH_SIZE;

const ACC: u32 = 0x1C00_0100;
const FUNC: u32 = 0x2000;

/// MOV.L @(disp,PC),Rn at `at` loading the literal at `lit`.
fn movl_pc(at: u32, lit: u32, rn: u16) -> u16 {
    let base = (at & !3) + 4;
    assert!(lit >= base && lit % 4 == 0 && (lit - base) / 4 < 256);
    0xD000 | (rn << 8) | ((lit - base) / 4) as u16
}

/// A program that calls FUNC (mov #1,r0; rts) 50 times adding r0 to ACC, with a device read and a 32-bit instruction in
/// the loop, then reprograms FUNC's first word to mov #0,r0 through the NOR command interface and calls it 50 times
/// more (BF/S loop), then spins.
fn image() -> Vec<u8> {
    let mut f = vec![0u8; FLASH_SIZE];
    let mut w16 = |a: u32, v: u16| f[a as usize..a as usize + 2].copy_from_slice(&v.to_be_bytes());
    let lit = 0x1100;
    let code: Vec<(u32, u16)> = vec![
        (0x1000, movl_pc(0x1000, lit, 2)), (0x1002, movl_pc(0x1002, lit + 4, 3)), (0x1004, movl_pc(0x1004, lit + 8, 9)), (0x1006, 0xE800),
        // loop 1
        (0x1008, 0x430B), (0x100A, 0x0009), (0x100C, 0x6122), (0x100E, 0x310C), (0x1010, 0x2212), (0x1012, 0x6A91),
        (0x1014, 0x0B50), (0x1016, 0x4321), (0x1018, 0x7801), (0x101A, 0x6083), (0x101C, 0x8832), (0x101E, 0x8BF3),
        // word program FUNC <- 0xE000 (AA@AAA, 55@554, A0@AAA, data)
        (0x1020, movl_pc(0x1020, lit + 12, 4)), (0x1022, movl_pc(0x1022, lit + 16, 5)), (0x1024, movl_pc(0x1024, lit + 20, 0)),
        (0x1026, movl_pc(0x1026, lit + 24, 1)), (0x1028, movl_pc(0x1028, lit + 28, 6)), (0x102A, movl_pc(0x102A, lit + 32, 7)),
        (0x102C, 0x2401), (0x102E, 0x2511), (0x1030, 0x2461), (0x1032, 0x2371), (0x1034, 0xE800),
        // loop 2
        (0x1036, 0x430B), (0x1038, 0x0009), (0x103A, 0x6122), (0x103C, 0x310C), (0x103E, 0x2212), (0x1040, 0x7801),
        (0x1042, 0x6083), (0x1044, 0x8832), (0x1046, 0x8FF6), (0x1048, 0x7B01), (0x104A, 0xAFFE), (0x104C, 0x0009),
        // FUNC
        (FUNC, 0xE001), (FUNC + 2, 0x000B), (FUNC + 4, 0x0009),
    ];
    for (a, v) in code { w16(a, v); }
    let mut w32 = |a: u32, v: u32| f[a as usize..a as usize + 4].copy_from_slice(&v.to_be_bytes());
    w32(0, 0x1000);
    w32(4, 0x1C20_0000);
    for (k, v) in [ACC, FUNC, 0xFFFE_C004, 0xAAA, 0x554, 0xAA, 0x55, 0xA0, 0xE000].into_iter().enumerate() { w32(lit + 4 * k as u32, v); }
    f
}

fn machine() -> Machine {
    let mut m = Machine::power_on(image()).expect("power on");
    m.configure_realtime();
    m.disable_hle();
    m
}

fn same(a: &Machine, b: &Machine) {
    assert_eq!((a.steps, a.cpu.pc, a.cpu.r, a.cpu.sr, a.cpu.pr), (b.steps, b.cpu.pc, b.cpu.r, b.cpu.sr, b.cpu.pr));
    assert_eq!((a.history, a.hist_pos), (b.history, b.hist_pos));
    assert_eq!(a.board.mmio_accesses, b.board.mmio_accesses);
    assert_eq!(a.cpu.mem.sdram, b.cpu.mem.sdram);
    assert_eq!(a.cpu.mem.flash, b.cpu.mem.flash);
}

fn acc(m: &Machine) -> u64 { m.cpu.mem.read_plain(ACC, 4).unwrap() }

#[test]
fn run_is_step_by_step_even_when_flash_changes() {
    let (mut fast, mut slow) = (machine(), machine());
    let n = 32 * 40;
    fast.run(n).expect("runs");
    while slow.steps < n { slow.step().expect("steps"); }
    same(&fast, &slow);
    assert_eq!(acc(&fast), 50, "the reprogrammed function returns 0");
    assert_eq!(fast.board.nor.programs, 1);

    // edit the function from outside (as a front end replacing the image would) and run loop 2 again
    for m in [&mut fast, &mut slow] {
        m.cpu.mem.flash[FUNC as usize..FUNC as usize + 2].copy_from_slice(&0xE003u16.to_be_bytes());
        m.cpu.pc = 0x1034;
    }
    fast.run(n).expect("runs");
    let end = slow.steps + n;
    while slow.steps < end { slow.step().expect("steps"); }
    same(&fast, &slow);
    assert_eq!(acc(&fast), 50 + 150, "the edited function returns 3");
}

const COUNTER: u32 = 0x1C00_0200;
const SCRATCH: u32 = 0x1C00_0300;

/// A program that waits in a loop for a timer interrupt (CMT0, every 201 x 8 P0 clocks) to change a counter in RAM,
/// then counts the change and waits again. The loop pushes and pops PR (a store that writes what is already there
/// once the loop runs). With `noisy`, the loop also increments a word in RAM each pass (registers still repeat): it must
/// not be fast-forwarded.
fn waiting_image(noisy: bool) -> Vec<u8> {
    let mut f = vec![0u8; FLASH_SIZE];
    let lit = 0x1100;
    let mut code: Vec<(u32, u16)> = vec![
        // set-up: VBR, CMT module on (STBCR7 bit 2), CMCOR0 = 200, CMCSR0 = CMIE | P0/8, IPR12 CMT0 level 7, CMSTR = 1
        (0x1000, movl_pc(0x1000, lit, 0)), (0x1002, 0x402E),
        (0x1004, movl_pc(0x1004, lit + 4, 1)), (0x1006, 0x6010), (0x1008, 0xC9FB), (0x100A, 0x2100),
        (0x100C, movl_pc(0x100C, lit + 8, 1)), (0x100E, movl_pc(0x100E, lit + 12, 0)), (0x1010, 0x2101),
        (0x1012, movl_pc(0x1012, lit + 16, 1)), (0x1014, 0xE040), (0x1016, 0x2101),
        (0x1018, movl_pc(0x1018, lit + 20, 1)), (0x101A, movl_pc(0x101A, lit + 24, 0)), (0x101C, 0x2101),
        (0x101E, movl_pc(0x101E, lit + 28, 1)), (0x1020, 0xE001), (0x1022, 0x2101),
        // r8 = &COUNTER, r9 = last value seen, r10 = changes seen, r12 = &SCRATCH; SR.IMASK = 0
        (0x1024, movl_pc(0x1024, lit + 32, 8)), (0x1026, 0xE900), (0x1028, 0xEA00), (0x102A, movl_pc(0x102A, lit + 36, 12)),
        (0x102C, 0xE000), (0x102E, 0x400E),
        // ISR: clear CMF (read, then write 0x40), COUNTER += 1, RESBANK, RTE
        (0x1400, movl_pc(0x1400, 0x1420, 1)), (0x1402, 0x6011), (0x1404, 0xE040), (0x1406, 0x2101),
        (0x1408, movl_pc(0x1408, 0x1424, 1)), (0x140A, 0x6012), (0x140C, 0x7001), (0x140E, 0x2102), (0x1410, 0x005B), (0x1412, 0x002B), (0x1414, 0x0009),
    ];
    let wait = if noisy {
        // L: sts.l pr,@-r15; lds.l @r15+,pr; mov.l @r12,r3; add #1,r3; mov.l r3,@r12; mov #0,r3; mov.l @r8,r2; cmp/eq r9,r2; bt L
        vec![0x4F22, 0x4F26, 0x63C2, 0x7301, 0x2C32, 0xE300, 0x6282, 0x3290, 0x89F6]
    } else {
        // L: sts.l pr,@-r15; lds.l @r15+,pr; mov.l @r8,r2; cmp/eq r9,r2; bt L
        vec![0x4F22, 0x4F26, 0x6282, 0x3290, 0x89FA]
    };
    let mut a = 0x1030;
    for w in &wait { code.push((a, *w)); a += 2; }
    // mov r2,r9; add #1,r10; bra L; nop
    let back = ((0x1030i32 - (a as i32 + 4 + 4)) / 2) as u16 & 0xFFF;
    code.extend([(a, 0x6923), (a + 2, 0x7A01), (a + 4, 0xA000 | back), (a + 6, 0x0009)]);
    for (a, v) in code { f[a as usize..a as usize + 2].copy_from_slice(&v.to_be_bytes()); }
    let mut w32 = |a: u32, v: u32| f[a as usize..a as usize + 4].copy_from_slice(&v.to_be_bytes());
    w32(0, 0x1000);
    w32(4, 0x1C20_0000);
    for (k, v) in [0x1800, 0xFFFE_0418, 0xFFFE_C006, 200, 0xFFFE_C002, 0xFFFE_0C0C, 0x7000, 0xFFFE_C000, COUNTER, SCRATCH].into_iter().enumerate() { w32(lit + 4 * k as u32, v); }
    w32(0x1420, 0xFFFE_C002);
    w32(0x1424, COUNTER);
    w32(0x1800 + 188 * 4, 0x1400);
    f
}

fn waiting(noisy: bool) -> Machine {
    let mut m = Machine::power_on(waiting_image(noisy)).expect("power on");
    m.configure_realtime();
    m.disable_hle();
    m
}

#[test]
fn busy_wait_fast_forward_is_step_by_step() {
    for noisy in [false, true] {
        let (mut fast, mut slow) = (waiting(noisy), waiting(noisy));
        let n = 32 * 4000;
        // in several runs, as a front end would call it
        for _ in 0..4 { fast.run(n / 4).expect("runs"); }
        while slow.steps < n { slow.step().expect("steps"); }
        same(&fast, &slow);
        assert_eq!((fast.interrupts, fast.cpu.cur_pc), (slow.interrupts, slow.cpu.cur_pc));
        assert_eq!(fast.cpu.mem.oram, slow.cpu.mem.oram);
        let seen = fast.cpu.r[10];
        assert!(seen > 50, "the program saw {seen} timer interrupts");
        assert_eq!(fast.cpu.mem.read_plain(COUNTER, 4), Some(fast.interrupts));
        let (_, found, skipped) = fast.spin_stats();
        if noisy {
            assert_eq!(found, 0, "a loop that changes memory is not fast-forwarded");
            assert!(fast.cpu.mem.read_plain(SCRATCH, 4).unwrap() > 1000);
        } else {
            assert!(found > 40 && skipped > n * 4, "{found} loops fast-forwarded, {skipped} instructions skipped");
        }
    }
}
