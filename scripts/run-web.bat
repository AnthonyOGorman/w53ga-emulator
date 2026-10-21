@echo off
rem Build the browser version and serve it on http://127.0.0.1:8053/ (local only), then open it.
setlocal
pushd "%~dp0.."
call "%~dp0build-web.bat" || (popd & exit /b 1)
cargo build --release --bin w53ga-serve || (popd & exit /b 1)
set TD=%CARGO_TARGET_DIR%
if "%TD%"=="" set TD=target
start "" http://127.0.0.1:8053/
"%TD%\release\w53ga-serve.exe" --port 8053 --root web\www
popd
