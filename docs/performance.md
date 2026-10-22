# Performance

How fast the emulator runs, what the `perf` branch changed to make it faster, how each change was checked, and what
was not done. None of these changes alters what is emulated. After every change the emulated machine is in exactly
the same state as before, instruction for instruction and tick for tick (see [Correctness gate](#correctness-gate)).

## Summary

The seven [scenarios](#scenarios) were run one after the other on the same machine under the same background load,
with the baseline (`dd5a5b7`, the merge base of `perf` and `firmware-diagnostics`) and with the current head
(`36c3b06`):

| Scenario | Baseline wall s | x real time | Now wall s | x real time | Speed-up |
|---|---:|---:|---:|---:|---:|
| a: power-on, boot, map (60 s) | 77.6 | 0.77x | 38.0 | 1.58x | 2.0x |
| b: map driving (60 s) | 66.8 | 0.90x | 36.3 | 1.65x | 1.8x |
| c: dense Tokyo map (60 s) | 173.6 | 0.35x | 79.9 | 0.75x | 2.2x |
| d: from-scratch firmware (15 s) | 47.9 | 0.31x | 3.3 | 4.59x | 14.6x |
| e: as a, HLE detected at run time (60 s) | 76.0 | 0.79x | 47.3 | 1.27x | 1.6x |
| f: as a, with the controller's GPS database (60 s) | 59.6 | 1.01x | 33.0 | 1.82x | 1.8x |
| g: as c, with the GPS database (60 s) | 182.5 | 0.33x | 78.7 | 0.76x | 2.3x |

"x real time" is emulated seconds per second of wall time, taken from the emulator's own `ran X s of emulated time in
Y s` line. Start-up and the writing of outputs are not included.

**The machine was loaded during these runs.** It is a Ryzen 7 7700X under Windows 11, and a game, a browser and other
emulator runs were using the CPU (see [All runs](#all-runs)). With less load the same builds ran 15-30% faster:

- this session's builds ran the dense Tokyo map at 0.95-0.96x and map driving at about 2.0x;
- the scratch firmware ran at 5.0x in a single run.

The previous session's unloaded baseline figures were 0.94x for a, 0.94x for b, 0.35x for c and 0.32x for d.

Where the speed-up comes from:

- **SH-2A interpreter.** This is the previous session's work plus three small changes here:
  - opcodes are decoded once into a handler table;
  - instructions run in batches between peripheral ticks;
  - flash code runs from decoded blocks;
  - there are direct paths into SDRAM, flash and on-chip RAM;
  - each instruction does less bookkeeping.

  It roughly doubles the speed when the CPU is busy (c, g).
- **Busy-wait fast-forward**, for both the SH-2A and the RL78.
  - **How it works:** a loop that provably repeats until an interrupt arrives is not executed pass by pass. The
    peripherals are ticked as usual, and the CPU is put in the exact state it would have reached.
  - **Scratch firmware (d):** this is what makes it 15x faster. About 95% of its SH-2A instructions and 60% of the
    controller's were such loops.
  - **Boot scenarios (a, e, f):** it skips the boot loader's timer waits and about 12% of the controller's
    instructions.
  - **The unit's own firmware on the map:** its RTOS idle loop was already skipped.
- **Peripherals:** the VDC4 no longer builds strings at every Vsync, the SRC's idle ticks are not repeated, and the RTC
  divides instead of looping.
- **Saved states:** saving the machine took about 12 s and now takes under 0.5 s, and loading is faster too. This is
  not part of the "x real time" figures, but it is what the window front end's F9 and the browser's state save wait
  for.

## Scenarios

All runs are deterministic headless runs (`w53ga-headless`) using the unit's own files:

- the unit's 32 MiB flash dump (main software 1.15);
- the controller program `I4PGM015.bin`;
- the SD card folder, always mounted with `--sd-readonly` and given the card's CID;
- the v1.15 HLE profile (every scenario except e).

Each run writes a frame every 5 s (every 1 s in d), then the flash image, an SDRAM dump and the saved state at the end.
`AR_LINK_LOG=1` is set, so every controller link frame goes to the error output with its time.

| Scenario | What runs |
|---|---|
| a | Power on, boot to the map, map with a GPS route (Tottori, 50 km/h), 60 s |
| b | From a state saved at the end of a: driving on along the same route, 60 s |
| c | From the same state: driving through the dense map of central Tokyo (`web/www/routes/tokyo-palace-loop.txt`, 40 km/h), 60 s |
| d | The from-scratch firmware with its boot chooser (the `w53ga-scratch` development flash image), remote keys at 3.0 and 3.5 s, 15 s |
| e | As a, without a profile: HLE entry points and the idle loop detected at run time, 60 s |
| f | As a, with the controller's serial flash holding the GPS warning database (`--rl78-spi controller-gpsdb.spi`), 60 s |
| g | From a state saved at the end of f: Tokyo as in c, with the database, 60 s |

Scenarios a-e are the previous session's, unchanged. Their references were made then by the baseline binary, and the
baseline rebuilt now reproduces them exactly.

Scenarios f and g were added in this session, with the controller database installed. Without it, the controller reads
an empty chip. Their references were made with the baseline binary.

## Correctness gate

Every commit was checked against the baseline on all seven scenarios before it was made.

**Byte-identical outputs.** The following must match exactly:
- the standard output, minus the timing line and file paths;
- the error output, including the complete controller link log (every frame with its P0 time);
- every frame written during the run;
- the flash image;
- the SDRAM dump;
- the saved end state, compared field by field with `statecmp`. Every memory is compared by hash, plus every other
  field of both CPUs and all peripheral models; HashMap order does not matter.

**Other checks:**
- `cargo test --release` passes, including the run-equivalence tests below.
- The WebAssembly front end still compiles for `wasm32-unknown-unknown` (`cargo check -p w53ga-web`).
- For changes to the SH-2A core, the previous session's differential test finds no difference. It runs every 16-bit
  opcode from random CPU states against a copy of the original interpreter, with accesses aimed at SDRAM, on-chip RAM,
  flash, flash in command mode and MMIO.

**Tests for the busy-wait fast-forwards** (`tests/run.rs`, `tests/rl78_wait.rs`):
- Programs that wait for a timer interrupt, or poll a timer flag, end in the same state with the fast-forward as step
  by step.
- The controller's state is compared after every call.
- Loops that change memory are not fast-forwarded.
- Each safety check was removed once to confirm the tests then fail.

The six commits inherited from the previous session were each rebuilt and gated the same way. All seven scenarios came
out identical, including the two new ones.

## The changes

### Inherited from the previous session (gated again here)

| Commit | Change | Measured (same load as the baseline) |
|---|---|---|
| `875fd9d` | Every 16-bit opcode decoded once into its own small handler; data accesses to SDRAM and array-mode flash go straight to memory | c 0.35x → 0.39x, d 0.31x → 0.40x |
| `6ad97b8` | Instructions run back to back up to the next peripheral tick (`Cpu::run`), returning to the machine's per-instruction checks only when one could apply | c → 0.56x, b → 1.36x, a → 1.09x |
| `d989e0a` | The RL78 per-instruction path inlined into the companion's loop | not measured separately |
| `b60c3f4` | Decoded blocks of flash code: the checks are made once per block; blocks are checked against flash again after NOR programming and on every `Machine::run` | not measured separately |
| `3a08582` | One handler per FPU opcode group; delay slots inline; slow memory paths moved out of the handlers | not measured separately |
| `7734d57` | Idle ticks without entering the CPU loop | the six together: c 0.82x (lightly loaded machine) |

### This session

Each change below was measured against the previous build in alternating runs (the A/B rows under
[All runs](#all-runs)).

| Commit | Change | Measured |
|---|---|---|
| `c64f3f8` | On-chip RAM, where the map's pixel loops keep tables, served from the fast data paths | c +3.5% |
| `8a5a0b4` | Small RL78 helpers inlined; the controller's clock budget kept in a register | b +5% (4 pairs); RL78-only benchmark 8.8 → 8.1 ns per instruction |
| `0338091` | SH-2A run loop keeps the PC history position in a register | c +5% (2 pairs) |
| `dfb4b16` | Busy-wait fast-forward for the SH-2A (below) | d 0.56x → 1.41x |
| `170c849` | ...also for loops that read plain register latches (the boot loader's timer waits) | d 1.41x → about 2.5x (measured in parallel runs); boot scenarios: 224 M instructions per 60 s not executed |
| `ab11f53` | `Vdc4::vsync_edge` no longer clones and formats register group names at every Vsync (thousands of allocations per frame) | b +5% (2 pairs) |
| `2c71235` | Saved-state serializer buffered in front of gzip (bincode wrote every byte of the memories with its own call into the compressor); same format, either way compatible | saving a map state 12.4 s → under 0.4 s; loading 0.95 s → under 0.4 s |
| `bd5ead7` | Fast-forward history without a division per entry | part of the next |
| `3550015` | Busy-wait fast-forward for the RL78 controller (below) | d → 5.0x (single run); 129 M of 220 M controller instructions not executed in d, 95 M per 60 s while booting |
| `21dc064` | SRC: a quiescent tick is not repeated until the converter is read or written; RTC: EXTAL count divided instead of a subtraction loop | not measured separately (both were about 2.5% and 1% of the map scenario's time) |
| `36c3b06` | `w53ga`: the F key switches between real-time pacing and full speed (`--unpaced` at start-up) | front end only |

### Busy-wait fast-forward: SH-2A (`Machine::spin_*`)

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

**Fast-forwarding.** The peripherals are ticked exactly as `run_fast` would tick them, without executing the loop. This
continues until one of these happens:
- an interrupt above SR.IMASK is pending;
- a device writes memory;
- a latch the loop reads changes;
- a reset or fault is pending;
- the run ends.

The CPU then gets the recorded registers of that point in the pass. The PC history and the MMIO access count are
advanced as executing would have. Failed attempts back off, up to 4096 ticks between attempts.

### Busy-wait fast-forward: RL78 (`Companion::spin_*`)

The controller has the same mechanism, with its state kept exact at every instruction:
- **The check.** Before each instruction, the same check `service` makes decides whether an interrupt is due.
- **The loop.** A pass may make no SFR access, store nothing new to RAM, and keep IE and ISP unchanged.
- **Applying it.** Each instruction's recorded effect (registers, PC, cur_pc, clocks) is applied and the clocks are
  ticked. Whole passes are applied at once where neither a peripheral event nor the end of the clock budget falls
  inside.

### Turning it off

Both fast-forwards are on by default. `W53GA_SPIN=0` turns them off, for comparison or debugging. With
`W53GA_SPIN_STATS=1`, `w53ga-headless` reports how many loops were fast-forwarded and how many instructions were not
executed.

They stay off while any of these is active:
- runtime HLE detection still tracing calls;
- a watchpoint;
- a debug hook.

### Why runtime idle-loop detection does not apply to the from-scratch firmware

**A profile is in use.** Scenario d passes the v1.15 HLE profile, as the scratch firmware's tools do, because its checks
match the image (the scratch package leaves the original firmware in place behind the boot chooser). A profile turns
runtime detection off. The idle loop the profile names (0xFD95A) belongs to the original firmware, which the scratch
firmware never runs.

**There is no RTOS idle loop to find.** The scratch firmware's main loop polls, in RAM, the controller link's receive
ring, the remote's queue and the millisecond counter, and redraws every 250 ms. Skipping at a fixed loop head whenever
no interrupt is pending would delay the work that loop does when data is waiting.

**The busy-wait fast-forward covers it instead.** It does not need a known idle loop: it checks every time that the
loop is repeating identically, and stops at the tick where anything the loop depends on changes. That now covers the
scratch firmware's main loop, its `delay_ms`, the boot loader's timer waits and the controller's own wait loops.

## Where the time goes now

Shares of host time inside `Machine::run`, from a sampling profiler on the current code:

| Scenario | SH-2A | RL78 controller | Other peripherals |
|---|---:|---:|---:|
| b (map; the idle loop is 79% of the time) | ~40% | ~33% | ~25% (display ~4%, interrupt priority scan ~3%, SCIF ~3%) |
| c, g (dense Tokyo map) | ~60% | ~19% | ~15% |
| d (scratch firmware) | ~30% (incl. the fast-forward's checks) | ~25% | ~40% |

**SH-2A.** The interpreter costs about 4 ns per instruction. On the dense map the firmware executes about
160 M SH-2A instructions per emulated second. That is real drawing work (pixel loops), so the map runs close to real
time and no loop skipping applies.

**RL78 controller.** The controller runs about 14 M instructions per emulated second and never halts. On the map it
reads its serial flash continuously, waiting for each byte in a polling loop with a timeout counter. It costs 8-12 ns
per instruction (less in isolation, more alongside the SH-2A).

## Ideas not done, with estimated gains

| Idea | Estimated gain | Effort, risk |
|---|---|---|
| **SH-2A JIT**: translate flash blocks to x86-64. A feasibility study with prototypes was made this session (summary below). | Dense map ~0.95x → 1.5-1.7x natively. The SH-2A drops to ~1 ns per instruction; the RL78 and the peripherals then dominate, capping the gain at ~2.2x. | 8-11 person-weeks. x86-64 only, no gain in the browser, and a second implementation of every instruction to keep exact. |
| **Threaded, pre-decoded SH-2A interpreter**: specialised handlers with decoded operands, PC history recorded per block, successor links between blocks. | 1.4-1.9x on SH-2A instruction cost in the prototype; +20-30% on the dense map natively, +10-15% in the browser. | 2-3 person-weeks, portable. Every memory-access handler changes, so they would need the differential test and the gate again. |
| **WebAssembly code generation** for the browser build | +50-80% browser speed | 5-8 person-weeks on top of a JIT design |
| **RL78 pre-decoded instruction cache** | 1.3-1.5x on the controller → +5-10% on the map and Tokyo | 1-2 person-weeks |
| **Countdown polling loops**. These change a counter on each pass, so the identical-state fast-forward does not apply. Examples: the boot loader's delay loops at 0x1185EE and 0x40926 (~10% of SH-2A instructions while booting); the controller's serial-flash wait at 0x48043 (~45% of its instructions on the map). | +10-15% on the map scenarios | Needs a sound way to show that the counter only feeds tests against zero; none found that is exact in general. |
| **Event-driven peripheral ticks**: skip ticks at which no model has anything due. | +10% on idle-heavy scenarios (the controller runs at every tick anyway) | Every model needs an exact "next event"; high risk. |
| Cheaper interrupt priority scan (`Board::best_request`, run after every tick) | ~1.5% | small |
| PC history per block instead of per instruction in the SH-2A loop | ≤5% on Tokyo (measured: inconclusive under machine load) | small change to the `Machine::history` API |
| Not revalidating decoded blocks on every `Machine::run` (the window front end calls it every 50,000 P0 clocks) | measured: no difference | - |

### JIT study summary

A separate study measured the dense map with an instrumented build:
- **Code and blocks:** 99.98% of instructions come from flash blocks. The average executed block is 5.8 instructions,
  and 90% of blocks are followed by the block they followed last time.
- **Run lengths:** `Cpu::run` returns about every 136 instructions: 71% of returns are budget ends and 29% are device
  accesses.
- **Instruction mix:** 12% are FPU operations (DN always set, round-to-zero rare); branches are about 15%.

**Prototypes** of the strategies, run on a map-like synthetic loop with an exact state comparison, measured in ns per
instruction:

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

The prototypes are in the session's scratch folder, not in the repository.

## Measurement method

Each scenario runs as a separate process, one at a time. The records are:
- the emulator's own `ran X s of emulated time in Y s` line;
- the process's CPU time, scaled to the run part;
- the other load on the machine at the time (not listed here).

A/B comparisons alternate the two builds. The reference outputs, gate scripts, `statecmp`, the CPU differential test,
an RL78-only benchmark and the sampling profiler are in the session's scratch folder, not in the repository.

A full timing series of every commit on every scenario was started but stopped after the first three builds: about two
hours of machine time for figures that the background load made unreliable anyway. The table below lists every
sequential run that was made.

## All runs

"series": one build after the other, every scenario in turn. "A/B": alternating runs of two builds on one scenario.
`36c3b06` is the current head; `wtN` builds of the working tree are listed under the commit whose code they were.

| Run | Build | Scenario | Wall s | x real time | x (CPU time) |
|---|---|---|---:|---:|---:|
| series | `dd5a5b7` | a | 77.59 | 0.77x | 0.81x |
| series | `dd5a5b7` | b | 66.83 | 0.90x | 0.93x |
| series | `dd5a5b7` | c | 173.60 | 0.35x | 0.36x |
| series | `dd5a5b7` | d | 47.89 | 0.31x | 0.33x |
| series | `dd5a5b7` | e | 75.95 | 0.79x | 0.84x |
| series | `dd5a5b7` | f | 59.59 | 1.01x | 1.04x |
| series | `dd5a5b7` | g | 182.54 | 0.33x | 0.35x |
| series | `875fd9d` | a | 81.30 | 0.74x | 0.78x |
| series | `875fd9d` | b | 70.72 | 0.85x | 0.91x |
| series | `875fd9d` | c | 153.42 | 0.39x | 0.41x |
| series | `875fd9d` | d | 37.44 | 0.40x | 0.42x |
| series | `875fd9d` | e | 66.22 | 0.91x | 0.95x |
| series | `875fd9d` | f | 55.81 | 1.08x | 1.12x |
| series | `875fd9d` | g | 148.42 | 0.40x | 0.42x |
| series | `6ad97b8` | a | 55.20 | 1.09x | 1.14x |
| series | `6ad97b8` | b | 44.11 | 1.36x | 1.39x |
| series | `6ad97b8` | c | 106.69 | 0.56x | 0.60x |
| series | `6ad97b8` | d | 28.38 | 0.53x | 0.57x |
| series | `6ad97b8` | e | 58.56 | 1.02x | 1.09x |
| series | `6ad97b8` | f | 48.08 | 1.25x | 1.31x |
| series | `36c3b06` | a | 38.00 | 1.58x | 1.65x |
| series | `36c3b06` | b | 36.33 | 1.65x | 1.73x |
| series | `36c3b06` | c | 79.86 | 0.75x | 0.77x |
| series | `36c3b06` | d | 3.27 | 4.59x | 4.87x |
| series | `36c3b06` | e | 47.26 | 1.27x | 1.31x |
| series | `36c3b06` | f | 32.98 | 1.82x | 1.89x |
| series | `36c3b06` | g | 78.67 | 0.76x | 0.79x |
| A/B | `7734d57` | c | 73.13 | 0.82x | 0.83x |
| A/B | `c64f3f8` | c | 70.74 | 0.85x | 0.86x |
| A/B | `c64f3f8` | b | 30.37 | 1.98x | 2.00x |
| A/B | `8a5a0b4` | b | 30.36 | 1.98x | 2.00x |
| A/B | `c64f3f8` | b | 32.56 | 1.84x | 1.86x |
| A/B | `8a5a0b4` | b | 30.45 | 1.97x | 2.00x |
| A/B | `c64f3f8` | b | 31.51 | 1.90x | 1.93x |
| A/B | `8a5a0b4` | b | 28.95 | 2.07x | 2.10x |
| A/B | `c64f3f8` | b | 31.73 | 1.89x | 1.91x |
| A/B | `8a5a0b4` | b | 30.32 | 1.98x | 2.00x |
| A/B | `8a5a0b4` | c | 66.85 | 0.90x | 0.90x |
| A/B | `0338091` | c | 62.31 | 0.96x | 0.97x |
| A/B | `8a5a0b4` | c | 64.84 | 0.93x | 0.93x |
| A/B | `0338091` | c | 63.29 | 0.95x | 0.96x |
| A/B | `0338091` | d | 26.74 | 0.56x | 0.58x |
| A/B | `0338091` | c | 81.86 | 0.73x | 0.76x |
| A/B | `dfb4b16` | d | 10.67 | 1.41x | 1.46x |
| A/B | `dfb4b16` | c | 74.61 | 0.80x | 0.83x |
| A/B | `170c849` | b | 36.17 | 1.66x | 1.71x |
| A/B | `ab11f53` | b | 34.10 | 1.76x | 1.81x |
| A/B | `170c849` | b | 35.00 | 1.71x | 1.76x |
| A/B | `ab11f53` | b | 33.78 | 1.78x | 1.84x |
| A/B | `2c71235` | c | 80.48 | 0.75x | 0.78x |
| A/B | experiment: `2c71235` without the PC history | c | 74.03 | 0.81x | 0.84x |
| A/B | `2c71235` | c | 76.36 | 0.79x | 0.82x |
| A/B | experiment: `2c71235` without the PC history | c | 77.88 | 0.77x | 0.81x |
