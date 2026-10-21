@echo off
rem Build the browser version: compiles the emulator to WebAssembly and places it next to web\www\index.html.
rem Needs the wasm target once: rustup target add wasm32-unknown-unknown
rem Then serve web\www, e.g.  cargo run --release --bin w53ga-serve   and open http://127.0.0.1:8053/
setlocal
pushd "%~dp0.."
cargo build -p w53ga-web --target wasm32-unknown-unknown --profile wasm-release || (popd & exit /b 1)
set TD=%CARGO_TARGET_DIR%
if "%TD%"=="" set TD=target
copy /y "%TD%\wasm32-unknown-unknown\wasm-release\w53ga_web.wasm" "web\www\w53ga_web.wasm" >nul || (popd & exit /b 1)
echo web\www\w53ga_web.wasm is ready; serve web\www and open it in a browser.
popd
