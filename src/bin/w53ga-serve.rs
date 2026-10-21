//! Tiny local web server for the browser front end (standard library only). Serves the static page in `web/www`
//! (with the WebAssembly build placed there by the build script) and listens on 127.0.0.1 only.
//!
//! The page works from any static file server; this one adds one optional extra: `POST /net?session=N`, a relay for
//! the emulated Wi-Fi module's network. Browsers cannot open raw sockets, so the page sends the socket requests of its
//! user-mode network here and gets the events back; they run on this computer's sockets, exactly as in the desktop
//! emulator. Nothing else is ever sent to the server: flash images and SD card files are read by the page itself.
//!
//! usage: w53ga-serve [--port N] [--root DIR]     (defaults: 8053, web/www)

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, Mutex};
use w53ga_emu::usernet::{relay_apply, ThreadedHost};

type Sessions = Arc<Mutex<HashMap<String, ThreadedHost>>>;

fn content_type(p: &Path) -> &'static str {
    match p.extension().and_then(|e| e.to_str()).unwrap_or("") {
        "html" => "text/html; charset=utf-8",
        "js" | "mjs" => "text/javascript; charset=utf-8",
        "css" => "text/css; charset=utf-8",
        "wasm" => "application/wasm",
        "json" => "application/json",
        "png" => "image/png",
        "svg" => "image/svg+xml",
        "ico" => "image/x-icon",
        "txt" | "md" => "text/plain; charset=utf-8",
        _ => "application/octet-stream",
    }
}

/// The file under `root` for a request path, refusing anything that would leave it.
fn resolve(root: &Path, url: &str) -> Option<PathBuf> {
    let path = url.split(['?', '#']).next().unwrap_or("/");
    let rel = Path::new(path.trim_start_matches('/'));
    if rel.components().any(|c| !matches!(c, Component::Normal(_))) { return None; }
    let p = root.join(rel);
    if p.is_dir() { Some(p.join("index.html")) } else { Some(p) }
}

fn reply(s: &mut TcpStream, status: &str, ctype: &str, body: &[u8]) -> std::io::Result<()> {
    write!(s, "HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nCache-Control: no-cache\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n", body.len())?;
    s.write_all(body)
}

fn serve(mut s: TcpStream, root: &Path, sessions: Sessions) -> std::io::Result<()> {
    let mut r = BufReader::new(s.try_clone()?);
    let mut line = String::new();
    r.read_line(&mut line)?;
    let mut it = line.split_whitespace();
    let (method, url) = (it.next().unwrap_or("GET").to_string(), it.next().unwrap_or("/").to_string());
    let mut length = 0usize;
    loop {
        let mut h = String::new();
        if r.read_line(&mut h)? == 0 || h.trim().is_empty() { break; }
        if let Some((k, v)) = h.split_once(':') { if k.eq_ignore_ascii_case("content-length") { length = v.trim().parse().unwrap_or(0); } }
    }
    if method == "POST" && url.starts_with("/net") {
        let mut body = vec![0u8; length.min(64 << 20)];
        r.read_exact(&mut body)?;
        let session = url.split("session=").nth(1).unwrap_or("0").split('&').next().unwrap_or("0").to_string();
        let out = {
            let mut m = sessions.lock().unwrap();
            let host = m.entry(session).or_default();
            relay_apply(host, &String::from_utf8_lossy(&body))
        };
        return reply(&mut s, "200 OK", "application/json", out.as_bytes());
    }
    if method != "GET" && method != "HEAD" { return reply(&mut s, "405 Method Not Allowed", "text/plain", b"method not allowed"); }
    let Some(p) = resolve(root, &url).filter(|p| p.is_file()) else { return reply(&mut s, "404 Not Found", "text/plain", b"not found") };
    let mut f = std::fs::File::open(&p)?;
    let len = f.metadata()?.len();
    write!(s, "HTTP/1.1 200 OK\r\nContent-Type: {}\r\nContent-Length: {len}\r\nCache-Control: no-cache\r\nX-Content-Type-Options: nosniff\r\nConnection: close\r\n\r\n", content_type(&p))?;
    if method == "HEAD" { return Ok(()); }
    let mut buf = vec![0u8; 1 << 16];
    loop { let n = f.read(&mut buf)?; if n == 0 { break; } s.write_all(&buf[..n])?; }
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let opt = |name: &str| args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned();
    if args.iter().any(|a| a == "-h" || a == "--help") { println!("usage: w53ga-serve [--port N] [--root DIR]   (defaults: 8053, web/www)"); return; }
    // `w53ga-serve 8080` also works.
    let port: u16 = opt("--port").or_else(|| args.get(1).filter(|a| !a.starts_with('-')).cloned()).and_then(|p| p.parse().ok()).unwrap_or(8053);
    let root = PathBuf::from(opt("--root").unwrap_or_else(|| "web/www".into()));
    if !root.join("index.html").is_file() { eprintln!("{} has no index.html (run from the repository root, or pass --root)", root.display()); std::process::exit(1); }
    if !root.join("w53ga_web.wasm").is_file() { eprintln!("note: {} has no w53ga_web.wasm yet; build it first (scripts/build-web.sh or scripts\\build-web.bat)", root.display()); }
    let l = match TcpListener::bind(("127.0.0.1", port)) { Ok(l) => l, Err(e) => { eprintln!("cannot listen on 127.0.0.1:{port}: {e}"); std::process::exit(1); } };
    println!("AR-W53GA emulator (web): http://127.0.0.1:{port}/   serving {}   (Ctrl+C to stop)", root.display());
    let sessions: Sessions = Arc::default();
    let root = Arc::new(root);
    for s in l.incoming().flatten() {
        let (ss, rr) = (sessions.clone(), root.clone());
        std::thread::spawn(move || { let _ = serve(s, &rr, ss); });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_stay_inside_the_root() {
        let root = Path::new("www");
        assert_eq!(resolve(root, "/app.js?v=1"), Some(PathBuf::from("www/app.js")));
        assert_eq!(resolve(root, "/../secret"), None);
        assert_eq!(resolve(root, "/a/../../b"), None);
    }
}
