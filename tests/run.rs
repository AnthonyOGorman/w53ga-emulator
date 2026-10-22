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
