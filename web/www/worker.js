// Emulator worker: runs the WebAssembly build of the emulator paced to real time, and answers its SD card reads
// synchronously from the files the user chose (FileReaderSync only exists in workers). With a read/write folder
// (File System Access API) it also writes the unit's card changes back when asked. Nothing leaves the browser, except
// the optional Wi-Fi relay requests to the local server the page was loaded from.
'use strict';

const W = 400, H = 240;
let ex = null;
const mem = () => new Uint8Array(ex.memory.buffer);
const files = new Map();          // card path -> File
const fromFolder = new Set();     // card paths that came from the chosen folder
let dir = null;                   // FileSystemDirectoryHandle (read/write folders only)
const reader = new FileReaderSync();
const enc = new TextEncoder(), dec = new TextDecoder();
let running = false, paused = false, emuMs = 0, wallStart = 0, lastFrame = 0, lastStatus = 0, statusWall = 0, statusEmu = 0, speedFactor = 0;
let wifi = false, netBusy = false, netWarned = false;
const session = Math.floor(Math.random() * 1e12).toString(36);

function put(bytes) { const p = ex.alloc(bytes.length); mem().set(bytes, p); return p; }
function buf() { return mem().slice(ex.buf_ptr(), ex.buf_ptr() + ex.buf_len()); }
function text() { return dec.decode(buf()); }
function str(s, fn, ...rest) { const b = enc.encode(s); const p = put(b); const r = fn(p, b.length, ...rest); ex.dealloc(p, b.length); return r; }
function bytesCall(ab, fn) { const u = new Uint8Array(ab); const p = put(u); const r = fn(p, u.length); ex.dealloc(p, u.length); return r; }

async function load() {
  const env = {
    js_read(pathPtr, pathLen, offset, dst, len) {
      try {
        const path = dec.decode(new Uint8Array(ex.memory.buffer, pathPtr, pathLen));
        const f = files.get(path);
        if (!f) return 1;
        const ab = reader.readAsArrayBuffer(f.slice(offset, offset + len));
        new Uint8Array(ex.memory.buffer, dst, len).set(new Uint8Array(ab));
        return 0;
      } catch (e) { return 2; }
    },
  };
  const resp = await fetch('w53ga_web.wasm');
  if (!resp.ok) throw new Error('w53ga_web.wasm is missing next to index.html: build it first (scripts/build-web.sh or scripts\\build-web.bat).');
  const { instance } = await WebAssembly.instantiate(await resp.arrayBuffer(), { env });
  ex = instance.exports;
  postMessage({ type: 'ready', powerOnAvailable: ex.emu_power_on_available() === 1 });
}
load().catch((e) => postMessage({ type: 'loadError', message: String(e.message || e) }));

// Wi-Fi: the emulated network's socket requests go to the local server's /net relay (one request in flight at a time).
function netPump() {
  if (!wifi || netBusy) return;
  ex.emu_net_take();
  const cmds = text();
  netBusy = true;
  fetch('net?session=' + session, { method: 'POST', body: cmds })
    .then((r) => { if (!r.ok) throw new Error(); return r.text(); }).then((t) => { if (t.length > 2) str(t, ex.emu_net_give); })
    .catch(() => {
      // No relay (a static host such as GitHub Pages): the module finds no network. Say so once.
      if (!netWarned) { netWarned = true; postMessage({ type: 'error', message: 'Wi-Fi: no network here. The emulated Wi-Fi needs the local server w53ga-serve (see README, "Quick start: in the browser"); everything else works.' }); }
    }).finally(() => { netBusy = false; });
}

function loop() {
  if (!running) return;
  const now = performance.now();
  if (!paused) {
    // Keep emulated time with the wall clock; never more than 50 ms behind (drop the backlog when slower).
    const wall = now - wallStart;
    if (wall - emuMs > 50) { wallStart += wall - emuMs - 50; }
    const t0 = performance.now();
    while (emuMs < performance.now() - wallStart && performance.now() - t0 < 30) {
      const step = Math.min(5, performance.now() - wallStart - emuMs);
      if (ex.emu_run_ms(step) !== 0) { running = false; postMessage({ type: 'stopped', reason: text() }); return; }
      emuMs += step;
    }
    netPump();
    if (ex.emu_audio()) postMessage({ type: 'audio', pcm: buf().buffer });
  }
  if (now - lastFrame > 33) {
    lastFrame = now;
    const p = ex.emu_render();
    const px = new Uint32Array(ex.memory.buffer, p, W * H);
    const rgba = new Uint8ClampedArray(W * H * 4);
    for (let i = 0; i < px.length; i++) { const v = px[i]; rgba[i * 4] = v >> 16; rgba[i * 4 + 1] = (v >> 8) & 255; rgba[i * 4 + 2] = v & 255; rgba[i * 4 + 3] = 255; }
    postMessage({ type: 'frame', rgba: rgba.buffer }, [rgba.buffer]);
  }
  if (now - lastStatus > 250) {
    // speed: emulated time per wall time over the last second or so
    if (now - statusWall > 1000) { speedFactor = paused ? 0 : (emuMs - statusEmu) / (now - statusWall); statusWall = now; statusEmu = emuMs; }
    lastStatus = now;
    ex.emu_status();
    const s = JSON.parse(text());
    s.speedFactor = speedFactor;
    postMessage({ type: 'status', status: s });
  }
  setTimeout(loop, 4);
}

function attachCard(list) {
  files.clear(); fromFolder.clear(); ex.emu_sd_clear();
  for (const f of list) { files.set(f.path, f.file); fromFolder.add(f.path.toLowerCase()); str(f.path, ex.emu_sd_add, f.file.size, 0); }
  if (ex.emu_sd_commit() !== 0) throw new Error('SD card: ' + text());
  return text();
}

// Write what the software changed on the card back into the chosen folder.
async function saveCard() {
  if (!dir) throw new Error('the SD card folder is read-only here (choose it with “Choose folder…” in a browser with the File System Access API)');
  const n = ex.emu_sd_changes();
  if (n < 0) throw new Error(text());
  const list = JSON.parse(text());
  const lines = [];
  const parent = async (path, create) => {
    const parts = path.split('/'); let d = dir;
    for (const p of parts.slice(0, -1)) d = await d.getDirectoryHandle(p, { create });
    return [d, parts[parts.length - 1]];
  };
  for (let i = 0; i < list.length; i++) {
    const c = list[i];
    if (c.op === 'mkdir') {
      const [d, name] = await parent(c.path, true);
      await d.getDirectoryHandle(name, { create: true });
      lines.push('folder ' + c.path);
    } else if (c.op === 'write') {
      if (c.existed && !fromFolder.has(c.path.toLowerCase())) continue;
      ex.emu_sd_change_data(i);
      const data = buf();
      const [d, name] = await parent(c.path, true);
      const h = await d.getFileHandle(name, { create: true });
      const old = files.get(c.path);
      if (old && old.size === data.length) { try { const cur = new Uint8Array(await old.arrayBuffer()); if (cur.every((b, k) => b === data[k])) continue; } catch (_) { /* changed on disk: rewrite */ } }
      const w = await h.createWritable();
      await w.write(data); await w.close();
      files.set(c.path, await h.getFile());   // later reads of this file see the new content
      fromFolder.add(c.path.toLowerCase());
      lines.push((c.existed ? 'updated ' : 'added ') + c.path);
    } else if (c.op === 'delete') {
      if (!fromFolder.has(c.path.toLowerCase())) continue;
      const [d, name] = await parent(c.path, false);
      await d.removeEntry(name);
      lines.push('deleted ' + c.path);
    }
  }
  return lines;
}

onmessage = async (e) => {
  const m = e.data;
  try {
    if (!ex && m.cmd !== 'init') return;
    switch (m.cmd) {
      case 'init': {
        if (!ex) throw new Error('the emulator is not loaded');
        running = false;
        if (m.poweron && bytesCall(m.poweron, ex.emu_set_power_on_state) !== 0) throw new Error('power-on state: ' + text());
        if (str(m.profile || '', ex.emu_set_profile) !== 0) throw new Error('HLE profile: ' + text());
        let note = '';
        if (m.mode === 'power-on') {
          const pf = put(new Uint8Array(m.flash)), pr = put(new Uint8Array(m.rl78));
          const r = ex.emu_power_on(pf, m.flash.byteLength, pr, m.rl78.byteLength);
          ex.dealloc(pf, m.flash.byteLength); ex.dealloc(pr, m.rl78.byteLength);
          if (r !== 0) throw new Error(text());
        } else {
          if (bytesCall(m.state, ex.emu_load_state) !== 0) throw new Error('saved state: ' + text());
          const r = bytesCall(m.flash, ex.emu_set_flash);
          if (r < 0) throw new Error('flash image: ' + text());
          if (r === 1) note = 'Your flash image differs from the one the state was saved with; using yours.';
        }
        dir = m.dir || null;
        const skipped = m.files.length || m.mode === 'power-on' ? attachCard(m.files) : '';
        if (m.cid && str(m.cid, ex.emu_sd_cid) !== 0) throw new Error(text());
        ex.emu_accessories(m.obd2 ? 1 : 0, m.dvr ? 1 : 0);
        str(m.wifi || '', ex.emu_wifi); wifi = !!m.wifi;
        if (m.route) str(m.route, ex.emu_gps_route, m.speed, m.hold === false ? 0 : 1);
        if (m.radar) ex.emu_radar(m.radar);
        ex.emu_status();
        const profile = JSON.parse(text()).profile;
        running = true; paused = false; emuMs = 0; wallStart = performance.now(); statusWall = wallStart; statusEmu = 0;
        postMessage({ type: 'started', mode: m.mode, note, card: m.files.length > 0, cid: !!m.cid, skipped: skipped.split('\n').filter(Boolean).slice(0, 5).join(', '), profile });
        loop();
        break;
      }
      case 'key': ex.emu_key(m.code); break;
      case 'route': if (str(m.route, ex.emu_gps_route, m.speed, m.hold ? 1 : 0) !== 0) postMessage({ type: 'error', message: 'GPS: ' + text() }); break;
      case 'speed': ex.emu_gps_speed(m.speed); break;
      case 'radar': ex.emu_radar(m.strength); break;
      case 'gpscut': ex.emu_gps_cut(m.on ? 1 : 0); break;
      case 'accessories': ex.emu_accessories(m.obd2 ? 1 : 0, m.dvr ? 1 : 0); break;
      case 'wifi': str(m.spec, ex.emu_wifi); wifi = !!m.spec; break;
      case 'obd': ex.emu_obd(m.speed, m.rpm, m.coolant, m.intake, m.throttle, m.load, m.boost, m.battery, m.follow ? 1 : 0); break;
      case 'pause': paused = m.on; if (!paused) { wallStart = performance.now() - emuMs; statusWall = performance.now(); statusEmu = emuMs; } break;
      case 'save': { const n = ex.emu_save_state(); postMessage(n ? { type: 'saved', state: buf().buffer } : { type: 'saved', state: null, error: text() }); break; }
      case 'sdsave': { const lines = await saveCard(); postMessage({ type: 'sdsaved', lines }); break; }
    }
  } catch (err) { postMessage({ type: 'error', message: String(err.message || err) }); }
};
