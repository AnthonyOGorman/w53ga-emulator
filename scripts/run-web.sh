#!/usr/bin/env sh
# Build the browser version and serve it on http://127.0.0.1:8053/ (local only).
set -eu
cd "$(dirname "$0")/.."
sh scripts/build-web.sh
cargo run --release --bin w53ga-serve -- --port 8053 --root web/www
