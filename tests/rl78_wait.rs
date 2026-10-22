//! The controller's busy-wait fast-forward (`Companion`'s `spin_*`) must leave the controller exactly as executing every
//! instruction does: a program waits in a loop for a timer interrupt handler to set a flag in RAM.

use w53ga_emu::rl78::companion::Companion;

/// TAU0 channel 0 as an interval timer (fCLK, 1000 clocks) with INTTM00 unmasked; the main loop waits for the handler
/// to set a word in RAM, clears it, counts it in another word and waits again. With `noisy`, the wait loop also
/// increments a word in RAM each pass (its registers still repeat): it must not be fast-forwarded.
fn image(noisy: bool) -> Vec<u8> {
    let mut f = vec![0xFFu8; 0x80000];
    let mut put = |a: usize, b: &[u8]| f[a..a + b.len()].copy_from_slice(b);
    put(0x0000, &[0x00, 0x01]); // reset vector -> 0x0100
    put(0x002C, &[0x00, 0x02]); // INTTM00 (flag 20) -> 0x0200
    put(0x0100, &[
        0xCB, 0xF8, 0x00, 0xFE, // MOVW SP,#0xFE00
        0x30, 0x00, 0x00,       // MOVW AX,#0
        0xBF, 0xB6, 0x01,       // MOVW !TPS0,AX (fCLK)
        0xBF, 0x90, 0x01,       // MOVW !TMR00,AX (interval mode)
        0xCB, 0x18, 0xE7, 0x03, // MOVW TDR00,#999
        0xCE, 0xE6, 0xEF,       // MOV MK1L,#0xEF (TMMK00 = 0)
        0x30, 0x01, 0x00,       // MOVW AX,#1
        0xBF, 0xB2, 0x01,       // MOVW !TS0,AX (start channel 0)
        0x71, 0x7A, 0xFA,       // EI
    ]);
    // 0x011D: wait
    let wait: &[u8] = if noisy {
        &[0xF6, 0xA2, 0x04, 0xE0, 0x42, 0x00, 0xE0, 0xDD, 0xF7, 0xF6, 0xBF, 0x00, 0xE0, 0xA2, 0x02, 0xE0, 0xEF, 0xEB] // ... INCW !0xE004 ...
    } else {
        // CLRW AX; CMPW AX,!0xE000; BZ $0x011D; CLRW AX; MOVW !0xE000,AX; INCW !0xE002; BR $0x011D
        &[0xF6, 0x42, 0x00, 0xE0, 0xDD, 0xFA, 0xF6, 0xBF, 0x00, 0xE0, 0xA2, 0x02, 0xE0, 0xEF, 0xF1]
    };
    put(0x011D, wait);
    put(0x0200, &[0xE6, 0xBF, 0x00, 0xE0, 0x61, 0xFC]); // ONEW AX; MOVW !0xE000,AX; RETI
    f
}

fn word(c: &Companion, a: u32) -> u16 { let o = (a - 0xF7F00) as usize; u16::from_le_bytes([c.cpu.ram[o], c.cpu.ram[o + 1]]) }

#[test]
fn controller_busy_wait_fast_forward_is_step_by_step() {
    for noisy in [false, true] {
        let (mut fast, mut slow) = (Companion::new(image(noisy)), Companion::new(image(noisy)));
        slow.spin_skip = false;
        fast.spin_skip = true;
        for k in 0..200_000 {
            fast.advance(32, None);
            slow.advance(32, None);
            // the same at every point, not only at the end (a fast-forward stops in the middle of a pass)
            assert_eq!((fast.cpu.pc, fast.cpu.cur_pc, fast.cpu.cycles, fast.cpu.steps, fast.g13.cycles), (slow.cpu.pc, slow.cpu.cur_pc, slow.cpu.cycles, slow.cpu.steps, slow.g13.cycles), "after {k} calls");
            if k % 997 == 0 { assert_eq!(bincode::serialize(&fast.cpu).unwrap(), bincode::serialize(&slow.cpu).unwrap(), "after {k} calls"); }
        }
        assert_eq!(bincode::serialize(&fast.cpu).unwrap(), bincode::serialize(&slow.cpu).unwrap(), "CPU, RAM and registers");
        assert_eq!((fast.cpu.steps, fast.cpu.pc, fast.cpu.cur_pc), (slow.cpu.steps, slow.cpu.pc, slow.cpu.cur_pc));
        let g = |c: &Companion| bincode::serialize(&(&c.g13.sfr[..], &c.g13.sfr2, c.g13.cycles, &c.g13.tau, &c.g13.sau, &c.g13.irq_count[..])).unwrap();
        assert_eq!(g(&fast), g(&slow), "peripherals");
        assert!(fast.fault.is_none() && slow.fault.is_none());
        let seen = word(&fast, 0xFE002);
        assert!(seen > 3000, "the program saw {seen} timer interrupts");
        let (_, found, skipped) = fast.spin_stats();
        if noisy {
            assert_eq!(found, 0, "a loop that changes RAM is not fast-forwarded");
            assert!(word(&fast, 0xFE004) > 1000);
        } else {
            assert!(found > 1000 && skipped > fast.cpu.steps / 2, "{found} loops fast-forwarded, {skipped} of {} instructions not executed", fast.cpu.steps);
        }
        assert_eq!(slow.spin_stats().1, 0);
    }
}
