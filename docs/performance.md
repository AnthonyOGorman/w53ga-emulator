# Performance

How fast the emulator runs, what makes it fast, how a speed change is checked, and what could make it faster still.
None of the speed-ups alters what is emulated: the emulated machine ends up in exactly the same state as without them,
instruction for instruction and tick for tick (see [Checking that a change is exact](#checking-that-a-change-is-exact)).

## Speed

The seven [scenarios](#scenarios) before (`dd5a5b7`) and after (`36c3b06`) the speed work, run one after the other on
the same machine under the same load:

| Scenario | Before | After | Speed-up |
|---|---:|---:|---:|
| a: power-on, boot, map (60 s) | 0.77x | 1.58x | 2.0x |
| b: map driving (60 s) | 0.90x | 1.65x | 1.8x |
| c: dense Tokyo map (60 s) | 0.35x | 0.75x | 2.2x |
| d: from-scratch firmware (15 s) | 0.31x | 4.59x | 14.6x |
| e: as a, HLE detected at run time (60 s) | 0.79x | 1.27x | 1.6x |
| f: as a, with the controller's GPS database (60 s) | 1.01x | 1.82x | 1.8x |
| g: as c, with the GPS database (60 s) | 0.33x | 0.76x | 2.3x |

"x real time" is emulated seconds per second of wall time, from the emulator's own `ran X s of emulated time in Y s`
line; start-up and writing the outputs are not included. The machine (Ryzen 7 7700X, Windows 11) was busy with other
programs during these runs. On a quieter machine the same builds ran 15-30% faster: the dense Tokyo map at
0.95-0.96x, map driving at about 2.0x, the scratch firmware at 5.0x.

**Browser.** The WebAssembly build gains less: power-on and 100 s of driving went from 0.74x to 0.94x, run in Node 24
(the V8 engine of Chrome and Edge) the way the page's worker runs it.

### Scenarios

Deterministic headless runs (`w53ga-headless`) with the unit's own files: the 32 MiB flash dump (main software 1.15),
the controller program `I4PGM015.bin`, the SD card folder (`--sd-readonly`, with the card's CID) and the v1.15 HLE
profile (except in e).

| Scenario | What runs |
|---|---|
| a | Power on, boot to the map, map with a GPS route (Tottori, 50 km/h), 60 s |
| b | From a state saved at the end of a: driving on along the same route, 60 s |
| c | From the same state: the dense map of central Tokyo (`web/www/routes/tokyo-palace-loop.txt`, 40 km/h), 60 s |
| d | The from-scratch firmware with its boot chooser, remote keys at 3.0 and 3.5 s, 15 s |
| e | As a, without a profile: HLE entry points and the idle loop detected at run time, 60 s |
| f | As a, with the controller's serial flash holding the GPS warning database (`--rl78-spi`), 60 s |
| g | From a state saved at the end of f: Tokyo as in c, with the database, 60 s |

## What makes it fast

- **SH-2A interpreter.** Every 16-bit opcode is decoded once into its own small handler. Flash code runs from decoded
  blocks, which are checked against flash again after NOR programming. Instructions run back to back up to the next
  peripheral tick (`Cpu::run`), with direct paths into SDRAM, flash and on-chip RAM. This roughly doubles the speed
  when the CPU is busy (c, g).
- **RL78 controller.** Its per-instruction path is inlined into the companion's run loop, with its clock budget in a
  register.
- **Busy-wait fast-forward** on both CPUs ([below](#busy-wait-fast-forward)). It makes the scratch firmware 15x faster
  (about 95% of its SH-2A instructions and 60% of the controller's were such loops), and skips the boot loader's timer
  waits and about 12% of the controller's instructions while booting.
- **Idle loop.** On the map, the unit's RTOS idle loop named by the HLE profile is skipped: with nothing deliverable,
  only the peripherals advance until an interrupt is due.
- **Peripherals.** The VDC4 latches its layers at Vsync without building strings, the SRC does not repeat its idle
  ticks, and the RTC divides instead of looping.
- **Saved states.** The serializer is buffered in front of gzip: saving went from about 12 s to under 0.5 s, and
  loading is faster too (same format). This is what F9 in the window and the browser's state save wait for.

## Busy-wait fast-forward

A loop that provably repeats until something outside it changes is not executed pass by pass. The peripherals are
ticked as usual, and the CPU is given the exact state it would have reached.

### SH-2A (`Machine::spin_*`)

**Looking for a loop.** After a tick, the machine may look for a loop starting at the current instruction. It runs
through `step`, so nothing is skipped while looking. It needs a pass of at most 1024 steps that returns to the same
registers, followed by one more pass recorded with a store check.

**When a loop qualifies.** All of these must hold:
- the recorded pass returns to the same registers;
- every store writes the value already in memory;
- SR.IMASK stays the same;
- no device is accessed, except reads of plain register latches, whose only read effect (noting status flags as read)
  repeats exactly;
- no interrupt is taken and no device writes memory;
- no excluded instruction runs: register banks, RESBANK, SLEEP, TRAPA, RTE, LDC to SR.

Every further pass is then the same computation on the same inputs.

**Fast-forwarding.** The peripherals are ticked exactly as `run_fast` would tick them, without executing the loop,
until one of these happens: an interrupt above SR.IMASK is pending, a device writes memory, a latch the loop reads
changes, a reset or fault is pending, or the run ends. The CPU then gets the recorded registers of that point in the
pass, and the PC history and the MMIO access count are advanced as executing would have. Failed attempts back off, up
to 4096 ticks between attempts.

### RL78 (`Companion::spin_*`)

The controller has the same mechanism, with its state kept exact at every instruction:
- **The check.** Before each instruction, the same check `service` makes decides whether an interrupt is due.
- **The loop.** A pass may make no SFR access, store nothing new to RAM, and keep IE and ISP unchanged.
- **Applying it.** Each instruction's recorded effect (registers, PC, cur_pc, clocks) is applied and the clocks are
  ticked. Whole passes are applied at once where neither a peripheral event nor the end of the clock budget falls
  inside.

### Turning it off

Both fast-forwards are on by default. `W53GA_SPIN=0` turns them off, for comparison or debugging. With
`W53GA_SPIN_STATS=1`, `w53ga-headless` reports how many loops were fast-forwarded and how many instructions were not
executed. They stay off while runtime HLE detection is still tracing calls, and while a watchpoint or a debug hook is
set.

### Firmware without an idle loop

The from-scratch firmware has no RTOS idle loop for the idle-loop skip: its main loop polls, in RAM, the controller
link's receive ring, the remote's queue and the millisecond counter, and redraws every 250 ms. Skipping at a fixed loop
head would delay the work it does when data is waiting. The fast-forward needs no known loop: it checks every time that
the loop is repeating identically, and stops at the tick where anything the loop depends on changes.

## Checking that a change is exact

A speed change must leave every run byte for byte the same. Run the scenarios with the build before and after the
change, and with `W53GA_SPIN=0`, and compare:
- the standard output, minus the timing line and file paths;
- the error output with `AR_LINK_LOG=1` (or `full`): the controller link log, every frame with its time;
- every frame written with `--frame` and `--every`;
- the flash image (`--save-flash`) and memory dumps (`--dump-mem`);
- the saved end state (`--save-state`), field by field: saved states contain hash maps, so their bytes differ from run
  to run even when the machine is the same.

`cargo test` includes run-equivalence tests for the fast-forwards (`tests/run.rs`, `tests/rl78_wait.rs`): programs
that wait for a timer interrupt, or poll a timer flag, must end in the same state as when run step by step, and loops
that change memory must not be fast-forwarded. Removing any one of the safety checks makes them fail. The WebAssembly
front end must still build (`cargo check -p w53ga-web --target wasm32-unknown-unknown`).

## Where the time goes now

Shares of host time inside `Machine::run`, from a sampling profiler:

| Scenario | SH-2A | RL78 controller | Other peripherals |
|---|---:|---:|---:|
| b (map; the idle loop is 79% of the time) | ~40% | ~33% | ~25% (display ~4%, interrupt priority scan ~3%, SCIF ~3%) |
| c, g (dense Tokyo map) | ~60% | ~19% | ~15% |
| d (scratch firmware) | ~30% (incl. the fast-forward's checks) | ~25% | ~40% |

**SH-2A.** The interpreter costs about 4 ns per instruction. On the dense map the firmware executes about 160 M SH-2A
instructions per emulated second. That is real drawing work (pixel loops), so no loop skipping applies.

**RL78 controller.** The controller runs about 14 M instructions per emulated second and never halts. On the map it
reads its serial flash continuously, waiting for each byte in a polling loop with a timeout counter. It costs 8-12 ns
per instruction.

## Ideas not done, with estimated gains

| Idea | Estimated gain | Effort, risk |
|---|---|---|
| **SH-2A JIT**: translate flash blocks to x86-64 ([study below](#jit-study)) | Dense map ~0.95x → 1.5-1.7x natively. The SH-2A drops to ~1 ns per instruction; the RL78 and the peripherals then dominate, capping the gain at ~2.2x. | 8-11 person-weeks. x86-64 only, no gain in the browser, and a second implementation of every instruction to keep exact. |
| **Threaded, pre-decoded SH-2A interpreter**: specialised handlers with decoded operands, PC history recorded per block, successor links between blocks | 1.4-1.9x on SH-2A instruction cost in a prototype; +20-30% on the dense map natively, +10-15% in the browser | 2-3 person-weeks, portable. Every memory-access handler changes. |
| **WebAssembly code generation** for the browser build | +50-80% browser speed | 5-8 person-weeks on top of a JIT design |
| **RL78 pre-decoded instruction cache** | 1.3-1.5x on the controller → +5-10% on the map and Tokyo | 1-2 person-weeks |
| **Countdown polling loops**. These change a counter on each pass, so the identical-state fast-forward does not apply. Examples: the boot loader's delay loops at 0x1185EE and 0x40926 (~10% of SH-2A instructions while booting); the controller's serial-flash wait at 0x48043 (~45% of its instructions on the map). | +10-15% on the map scenarios | Needs a sound way to show that the counter only feeds tests against zero; none found that is exact in general. |
| **Event-driven peripheral ticks**: skip ticks at which no model has anything due | +10% on idle-heavy scenarios (the controller runs at every tick anyway) | Every model needs an exact "next event"; high risk. |
| Cheaper interrupt priority scan (`Board::best_request`, run after every tick) | ~1.5% | small |
| PC history per block instead of per instruction in the SH-2A loop | ≤5% on Tokyo (inconclusive under machine load) | small change to the `Machine::history` API |

### JIT study

Measured on the dense map with an instrumented build:
- **Code and blocks:** 99.98% of instructions come from flash blocks. The average executed block is 5.8 instructions,
  and 90% of blocks are followed by the block they followed last time.
- **Run lengths:** `Cpu::run` returns about every 136 instructions: 71% of returns are budget ends and 29% are device
  accesses.
- **Instruction mix:** 12% are FPU operations (DN always set, round-to-zero rare); branches are about 15%.

Prototypes of the strategies, run on a map-like synthetic loop with an exact state comparison, in ns per instruction:

| Strategy | Native | Browser (wasm, Node 24) |
|---|---:|---:|
| Interpreter now | 3.9-4.3 | 7.4 |
| Threaded code | 2.7-3.1 | 5.6 |
| Tail-call threaded | 2.2-2.3 | 5.7 |
| JIT-shaped code | 0.7-1.1 | 1.2-1.4 |

**Design.** A JIT would keep the interpreter as the exact fallback for every slow path: device accesses, faults, rare
instructions, and RAM code (0.02%). Generated code would exit *before* any instruction it does not handle, so stop
points, faults and the "stop after a device access" rule stay exactly as now. Translated code would live outside the
saved state.
