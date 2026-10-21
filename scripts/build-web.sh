#!/usr/bin/env sh
# Build the browser version: compiles the emulator to WebAssembly and places it next to web/www/index.html.
# Needs the wasm target once: rustup target add wasm32-unknown-unknown
# Then serve web/www with any static file server, e.g.  cargo run --release --bin w53ga-serve
# or  python3 -m http.server -d web/www 8053  and open http://127.0.0.1:8053/
set -eu
cd "$(dirname "$0")/.."
cargo build -p w53ga-web --target wasm32-unknown-unknown --profile wasm-release
target_dir="${CARGO_TARGET_DIR:-target}"
cp "$target_dir/wasm32-unknown-unknown/wasm-release/w53ga_web.wasm" web/www/w53ga_web.wasm
echo "web/www/w53ga_web.wasm ($(wc -c < web/www/w53ga_web.wasm) bytes) is ready; serve web/www and open it in a browser."
