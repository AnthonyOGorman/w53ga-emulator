# Contributing

Thank you for helping. Bug reports, accuracy fixes, new peripheral models, front-end improvements and documentation are
all welcome.

## The one rule: no manufacturer code or data

This project emulates **hardware**. It must never contain software or data from the unit or its maker:

- no flash dumps, controller images, update packages, SD card contents, saved states (`.state` files contain the flash
  image and RAM), map, sound, font or graphics data, and no byte arrays, tables or strings copied out of them;
- no code that recognises or patches a particular program by its bytes, and no symbol files for it. Program-specific
  entry points belong in user-supplied HLE profiles (`src/hle.rs`), which hold addresses and SHA-256 checks only;
- no personal data: Wi-Fi names and keys, card IDs, MAC addresses, home locations, API keys.

Pull requests that add any of these will be closed. When in doubt, ask in an issue first. Screenshots of what a unit
displays are fine in issues; please crop out anything personal.

## Building and testing

```sh
cargo build --release          # native front ends (needs ALSA/X11/Wayland dev packages on Linux, see CI)
cargo test                     # unit and integration tests; none need any file from a unit
sh scripts/build-web.sh        # WebAssembly build into web/www (rustup target add wasm32-unknown-unknown)
```

Tests must not depend on files from a unit. Test the CPU cores with hand-assembled instruction sequences (see
`tests/cores.rs`) and peripherals with synthetic register traffic.

## Style

- `cargo fmt` is not enforced (the code uses long lines on purpose); keep the surrounding style.
- Model hardware from the Renesas manuals and from observed behaviour, and say in a comment where a behaviour comes
  from. A model that meets an unsupported configuration should stop the run with a clear message rather than guess.
- Keep the web front end dependency-free (plain HTML, CSS and JavaScript); it must work as static files.
- Do not fake progress: no forced program counters, flags or memory to get past a problem. Fix the model instead.

## Reporting bugs

Include the emulator version (commit), the front end (native or browser and version), the options used, and what you
saw. Logs are welcome; never attach dumps, saved states or SD card files.

By contributing you agree that your contributions are licensed under the project's licence (MIT OR Apache-2.0).
