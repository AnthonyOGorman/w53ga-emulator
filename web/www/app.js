// AR-W53GA emulator, browser front end (main thread): file selection, controls and display. The emulator itself runs
// in worker.js (WebAssembly). Every file the user chooses is read locally by this page; nothing is uploaded.
'use strict';

const $ = (id) => document.getElementById(id);
const MiB = 1 << 20;
const FLASH_SIZE = 32 * MiB;
const RL78_SIZES = [MiB, MiB + 88];
const W = 400, H = 240;

const files = { flash: null, rl78: null, state: null, profile: null, poweron: null };
let sd = null;            // { mode: 'rw' | 'ro', name, handle, files: [{ path, file }] }
let powerOnAvailable = false, wasmReady = false;
let running = false, paused = false, started = false, startedMode = '';
let lastStatus = null, sdDirty = false;
let heading = 0, car = null;
let pendingRoute = null;  // { pts, hold }: a route chosen before the unit is switched on, driven from the start
let gpsCut = false;

// ---------------------------------------------------------------- worker
const worker = new Worker('worker.js');
const ctx = $('screen').getContext('2d', { alpha: false });
const image = ctx.createImageData(W, H);

worker.onmessage = (e) => {
  const m = e.data;
  switch (m.type) {
    case 'ready':
      wasmReady = true; powerOnAvailable = m.powerOnAvailable;
      $('row-poweron').hidden = powerOnAvailable;
      if (!powerOnAvailable) $('row-poweron').closest('details').open = true;
      refresh();
      break;
    case 'loadError': msg(m.message, 'err'); break;
    case 'frame': image.data.set(new Uint8ClampedArray(m.rgba)); ctx.putImageData(image, 0, 0); break;
    case 'audio': audio && audio.push(new Int16Array(m.pcm)); break;
    case 'status': onStatus(m.status); break;
    case 'started':
      started = true; running = true; paused = false; startedMode = m.mode;
      showCut(false);
      $('overlay').classList.add('hidden');
      const routeNote = pendingRoute ? `Driving the loaded route (${pendingRoute.pts.length} points).` : '';
      pendingRoute = null;
      msg([m.mode === 'power-on' ? 'Switched on: the unit boots from reset (about 45 s to the map).' : 'State resumed.', routeNote, m.note, m.profile ? `HLE profile: ${m.profile}.` : 'HLE off: 2D graphics calls are not drawn.', m.skipped ? `Skipped (not 8.3 names): ${m.skipped}` : '', m.mode === 'power-on' && m.card && !m.cid ? 'No card CID given: the unit will report an SD card error and the map will have no data (see README, "Card CID").' : ''].filter(Boolean).join(' '), 'ok');
      refresh();
      break;
    case 'stopped': running = false; msg('Stopped: ' + m.reason, 'err'); refresh(); break;
    case 'error': msg(m.message, 'err'); refresh(); break;
    case 'saved': if (m.state) download(new Blob([m.state]), `w53ga-${stamp()}.state`); else msg('State not saved: ' + (m.error || 'nothing running'), 'err'); break;
    case 'sdsaved': sdDirty = false; msg(m.lines.length ? `SD card: ${m.lines.length} change(s) written to “${sd.name}”: ${m.lines.slice(0, 6).join(', ')}${m.lines.length > 6 ? ', …' : ''}` : 'SD card: nothing to write.', 'ok'); refresh(); break;
  }
};
worker.onerror = (e) => msg('Worker error: ' + (e.message || e), 'err');

// ---------------------------------------------------------------- messages, helpers
function msg(text, kind) { const el = $('msg'); el.textContent = text || ''; el.className = 'msg' + (kind ? ' ' + kind : ''); }
function stamp() { return new Date().toISOString().replace(/[:T]/g, '-').slice(0, 19); }
function size(n) { return n >= MiB ? (n / MiB).toFixed(n % MiB ? 1 : 0) + ' MiB' : n >= 1024 ? Math.round(n / 1024) + ' KiB' : n + ' B'; }
function download(blob, name) { const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name; document.body.append(a); a.click(); a.remove(); setTimeout(() => URL.revokeObjectURL(a.href), 10000); }
function setVal(id, text, ok) { const el = $(id); el.textContent = text; el.classList.toggle('ok', !!ok); el.title = text; }

// ---------------------------------------------------------------- choosing files
function classify(f) {
  const n = f.name.toLowerCase();
  if (f.size === FLASH_SIZE) return 'flash';
  if (RL78_SIZES.includes(f.size)) return 'rl78';
  if (n.endsWith('.state')) return 'state';
  if (n.endsWith('.json')) return 'profile';
  if (n.endsWith('.gz')) return 'poweron';
  return null;
}
function useFile(kind, f) {
  if (!f) return;
  if (kind === 'flash' && f.size !== FLASH_SIZE) { msg(`${f.name} is ${size(f.size)}; a flash image is exactly 32 MiB.`, 'err'); return; }
  if (kind === 'rl78' && !RL78_SIZES.includes(f.size)) { msg(`${f.name} is ${f.size} bytes; the controller image is 1 MiB (1048576 bytes) or an update package carrying one (1048664 bytes).`, 'err'); return; }
  files[kind] = f;
  const label = { flash: 'val-flash', rl78: 'val-rl78', state: 'val-state', profile: 'val-profile', poweron: 'val-poweron' }[kind];
  setVal(label, `${f.name} · ${size(f.size)}`, true);
  if (kind === 'state') $('row-flash').scrollIntoView({ block: 'nearest' });
  refresh();
}
for (const k of Object.keys(files)) { const input = $('in-' + k); if (input) input.addEventListener('change', () => useFile(k, input.files[0])); }

async function walk(dir, prefix, out) {
  for await (const [name, h] of dir.entries()) {
    const path = prefix ? `${prefix}/${name}` : name;
    if (h.kind === 'file') out.push({ path, file: await h.getFile() });
    else await walk(h, path, out);
  }
  return out;
}
async function useDirHandle(handle) {
  let mode = 'ro';
  try { mode = (await handle.requestPermission({ mode: 'readwrite' })) === 'granted' ? 'rw' : 'ro'; } catch (_) { /* read-only */ }
  msg(`Reading the folder “${handle.name}”…`);
  const list = await walk(handle, '', []);
  setSd({ mode, name: handle.name, handle: mode === 'rw' ? handle : null, files: list });
}
function setSd(s) {
  sd = s;
  const total = s.files.reduce((a, f) => a + f.file.size, 0);
  setVal('val-sd', `${s.name} · ${s.files.length} files · ${size(total)} · ${s.mode === 'rw' ? 'read/write' : 'read-only'}`, true);
  $('sd-mode-note').textContent = s.mode === 'rw'
    ? 'read/write: what the unit writes can be saved back to this folder'
    : 'read-only: the unit\'s writes last for this session only';
  // A controller update package on the card can be used as the controller image.
  if (!files.rl78) { const c = s.files.find((x) => x.file.size === MiB + 88); if (c) { files.rl78 = c.file; setVal('val-rl78', `${c.path} (from the card) · ${size(c.file.size)}`, true); } }
  msg('');
  refresh();
}
$('btn-sd').addEventListener('click', async () => {
  if (window.showDirectoryPicker) {
    try { await useDirHandle(await window.showDirectoryPicker({ id: 'w53ga-sd', mode: 'readwrite' })); }
    catch (e) { if (e.name !== 'AbortError') msg('Folder not opened: ' + e.message, 'err'); }
  } else $('in-sd').click();
});
$('in-sd').addEventListener('change', () => {
  const list = [...$('in-sd').files].map((f) => ({ path: f.webkitRelativePath.split('/').slice(1).join('/'), file: f })).filter((x) => x.path);
  const name = $('in-sd').files[0] ? $('in-sd').files[0].webkitRelativePath.split('/')[0] : 'folder';
  if (list.length) setSd({ mode: 'ro', name, handle: null, files: list });
});

// drag and drop: files are recognised by size and name; a folder becomes the SD card
let dragDepth = 0;
addEventListener('dragenter', (e) => { if (e.dataTransfer && [...e.dataTransfer.types].includes('Files')) { dragDepth++; $('dropzone').hidden = false; e.preventDefault(); } });
addEventListener('dragleave', () => { if (--dragDepth <= 0) { dragDepth = 0; $('dropzone').hidden = true; } });
addEventListener('dragover', (e) => e.preventDefault());
addEventListener('drop', async (e) => {
  e.preventDefault(); dragDepth = 0; $('dropzone').hidden = true;
  const items = [...e.dataTransfer.items].filter((i) => i.kind === 'file');
  const handles = items.map((i) => (i.getAsFileSystemHandle ? i.getAsFileSystemHandle() : null));
  const entries = items.map((i) => (i.webkitGetAsEntry ? i.webkitGetAsEntry() : null));
  const plain = [...e.dataTransfer.files];
  for (let k = 0; k < items.length; k++) {
    const h = handles[k] ? await handles[k] : null;
    if (h && h.kind === 'directory') { await useDirHandle(h); continue; }
    if (!h && entries[k] && entries[k].isDirectory) { setSd({ mode: 'ro', name: entries[k].name, handle: null, files: await readEntryDir(entries[k], '') }); continue; }
    const f = h ? await h.getFile() : plain[k];
    const kind = f && classify(f);
    if (kind) useFile(kind, f); else if (f) msg(`Not recognised: ${f.name} (${size(f.size)}). A flash image is 32 MiB; use the file buttons for other files.`, 'err');
  }
});
function readEntryDir(dir, prefix) {
  return new Promise((resolve) => {
    const out = [], reader = dir.createReader();
    const next = () => reader.readEntries(async (batch) => {
      if (!batch.length) return resolve(out);
      for (const en of batch) {
        const path = prefix ? `${prefix}/${en.name}` : en.name;
        if (en.isFile) out.push({ path, file: await new Promise((r) => en.file(r)) });
        else out.push(...await readEntryDir(en, path));
      }
      next();
    });
    next();
  });
}

// ---------------------------------------------------------------- start, pause, save
function refresh() {
  const haveFlash = !!files.flash;
  $('btn-power').disabled = !wasmReady || !haveFlash || !files.rl78 || (!powerOnAvailable && !files.poweron);
  $('btn-resume').disabled = !wasmReady || !haveFlash || !files.state;
  for (const id of ['btn-pause', 'btn-shot', 'btn-save']) $(id).disabled = !started;
  $('btn-pause').textContent = paused ? 'Continue' : 'Pause';
  $('btn-sdsave').disabled = !(started && sd && sd.mode === 'rw');
  $('btn-power').textContent = started ? 'Power on again' : 'Power on';
  const st = $('st-state');
  st.className = 'pill ' + (!started ? '' : !running ? 'stopped' : paused ? 'paused' : 'on');
  st.lastElementChild.textContent = !started ? 'Off' : !running ? 'Stopped' : paused ? 'Paused' : 'Running';
  $('led').className = 'led ' + (!started ? '' : !running ? 'stopped' : paused ? 'paused' : 'on');
  if (!started) {
    const need = [];
    if (!haveFlash) need.push('your flash image');
    if (!files.rl78 && !files.state) need.push('the controller image');
    if (!sd && !files.state) need.push('your SD card folder');
    if (!powerOnAvailable && !files.poweron && !files.state) need.push('the power-on state (More files)');
    if (!wasmReady) need.push('the emulator to finish loading');
    $('overlay').querySelector('.big').textContent = haveFlash ? 'Ready when you are' : 'Drop your flash image here';
    $('overlay').lastElementChild.innerHTML = need.length ? 'Still needed: ' + need.join(', ') + '.' : 'Press <b>Power on</b> to switch the unit on.';
  }
}

async function start(mode) {
  try {
    if (!files.flash) return msg('Choose your flash image first.', 'err');
    if (mode === 'power-on' && !sd && !confirmNoCard()) return;
    // settings first, as they are now; then the files (reading them takes a moment)
    const init = {
      cmd: 'init', mode, files: sd ? sd.files : [], dir: sd && sd.mode === 'rw' ? sd.handle : null,
      cid: $('in-cid').value.trim(),
      obd2: $('acc-obd2').checked, dvr: $('acc-dvr').checked, wifi: $('acc-wifi').checked ? $('in-ssid').value : '',
      route: pendingRoute ? pendingRoute.pts.join(';') : car ? `${car[0]},${car[1]}` : '35.6812,139.7671',
      hold: pendingRoute ? pendingRoute.hold : true, speed: +$('gps-speed').value,
      radar: +$('radar-strength').value / 100,
    };
    init.flash = await files.flash.arrayBuffer();
    init.profile = files.profile ? await files.profile.text() : '';
    init.poweron = files.poweron ? await files.poweron.arrayBuffer() : null;
    if (mode === 'power-on') init.rl78 = await files.rl78.arrayBuffer();
    else init.state = await files.state.arrayBuffer();
    msg(mode === 'power-on' ? 'Switching on…' : 'Loading the state…');
    startAudio();
    const transfer = [init.flash, init.rl78, init.state, init.poweron].filter(Boolean);
    worker.postMessage(init, transfer);
    sdDirty = false;
  } catch (e) { msg(String(e.message || e), 'err'); }
}
let noCardOk = false;
function confirmNoCard() {
  if (noCardOk) return true;
  noCardOk = true;
  msg('No SD card folder chosen: the unit will start without a card (press Power on again to go ahead).', 'err');
  return false;
}
$('btn-power').addEventListener('click', () => start('power-on'));
$('btn-resume').addEventListener('click', () => start('state'));
$('btn-pause').addEventListener('click', togglePause);
function togglePause() { if (!started) return; paused = !paused; worker.postMessage({ cmd: 'pause', on: paused }); refresh(); }
$('btn-save').addEventListener('click', () => { worker.postMessage({ cmd: 'save' }); msg('Saving the state (it contains your flash image and RAM: keep the file private)…'); });
$('btn-shot').addEventListener('click', screenshot);
function screenshot() {
  if (!started) return;
  $('screen').toBlob((b) => b && download(b, `w53ga-${stamp()}.png`), 'image/png');
}
$('btn-sdsave').addEventListener('click', () => { worker.postMessage({ cmd: 'sdsave' }); msg('Writing SD card changes…'); });
addEventListener('beforeunload', (e) => { if (started && sd && sd.mode === 'rw' && sdDirty) { e.preventDefault(); e.returnValue = ''; } });

// ---------------------------------------------------------------- status
function onStatus(s) {
  lastStatus = s;
  const f = s.speedFactor || 0;
  $('speedtxt').textContent = paused ? 'paused' : `${f.toFixed(2)}× real time`;
  const fill = $('speedfill');
  fill.style.width = Math.min(100, f * 100) + '%';
  fill.style.background = f >= 0.95 ? 'var(--ok)' : f >= 0.7 ? 'var(--warn)' : 'var(--bad)';
  $('st-time').textContent = `${s.t.toFixed(1)} s`;
  const gps = $('st-gps');
  gps.hidden = !s.gps;
  if (s.gps) {
    const sky = s.gps.cut ? ' · <span class="cut">no fix (tunnel)</span>' : s.gps.fix === false ? ' · <span class="cut">acquiring</span>' : '';
    gps.innerHTML = `GPS <b>${s.gps.lat.toFixed(4)}, ${s.gps.lon.toFixed(4)}</b> · ${(s.gps.actual ?? s.gps.speed).toFixed(0)} km/h${sky}`;
    const prev = car;
    car = [s.gps.lat, s.gps.lon];
    if (carMarker) { carMarker.setLatLng(car); follow(prev); }
    $('gps-cut-note').textContent = s.gps.cut ? 'In the tunnel: no GPS fix, the car drives on' : s.gps.fix === false ? 'Out of the tunnel: re-acquiring…' : 'GPS sky clear';
  }
  const acc = [];
  if (s.obd2) acc.push(`OBD ${s.obd2.streaming ? Math.round(s.obd2.rpm) + ' rpm' : 'connecting'}`);
  if (s.dvr !== null && s.dvr !== undefined) acc.push(`DVR ${s.dvr ? 'linked' : 'waiting'}`);
  if (s.wifi) acc.push(`Wi-Fi ${s.wifi.connected ? s.wifi.connected : 'idle'}`);
  $('st-acc').hidden = !acc.length; $('st-acc').textContent = acc.join(' · ');
  const sdp = $('st-sd');
  sdp.hidden = !sd;
  if (sd) {
    sdDirty = s.sdWritten > 0;
    sdp.textContent = s.sdWritten ? `SD: ${s.sdWritten} sectors written${sd.mode === 'rw' ? '' : ' (session only)'}` : 'SD: no writes';
  }
}

// ---------------------------------------------------------------- display scale (crisp whole-number scaling)
let scaleChoice = +(localStorage.getItem('w53ga.scale') || 0);
function applyScale() {
  const stage = document.querySelector('.stage');
  const avail = Math.max(W, stage.clientWidth - 40);
  const availH = Math.max(H, innerHeight - 260);
  const fit = Math.max(1, Math.min(Math.floor(avail / W), Math.floor(availH / H)));
  const s = scaleChoice ? Math.min(scaleChoice, Math.max(1, Math.floor(avail / W))) : fit;
  const c = $('screen');
  c.style.width = W * s + 'px'; c.style.height = H * s + 'px';
  document.querySelectorAll('.scale button').forEach((b) => b.classList.toggle('on', +b.dataset.scale === scaleChoice));
}
document.querySelectorAll('.scale button').forEach((b) => b.addEventListener('click', () => { scaleChoice = +b.dataset.scale; try { localStorage.setItem('w53ga.scale', scaleChoice); } catch (_) { } applyScale(); }));
new ResizeObserver(applyScale).observe(document.querySelector('.stage'));
addEventListener('resize', applyScale);

// ---------------------------------------------------------------- remote
const keyCodes = { ArrowUp: 0x00, ArrowDown: 0x01, ArrowLeft: 0x02, ArrowRight: 0x08, Enter: 0x09, Backspace: 0x0A, F1: 0x10, F2: 0x11, F3: 0x12, F4: 0x64, F5: 0xFF };
document.querySelectorAll('[data-k]').forEach((b) => b.addEventListener('click', () => started && worker.postMessage({ cmd: 'key', code: +b.dataset.k })));
addEventListener('keydown', (e) => {
  const t = e.target;
  if ((t.tagName === 'INPUT' && !['range', 'checkbox', 'file'].includes(t.type)) || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA') return;
  if (!started || e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key in keyCodes) { e.preventDefault(); if (!e.repeat) worker.postMessage({ cmd: 'key', code: keyCodes[e.key] }); }
  else if (e.key === 'p' || e.key === 'P') togglePause();
  else if (e.key === 's' || e.key === 'S') screenshot();
  else if (e.key === 't' || e.key === 'T') toggleCut();
});

// ---------------------------------------------------------------- accessories
['acc-obd2', 'acc-dvr'].forEach((id) => $(id).addEventListener('change', () => started && worker.postMessage({ cmd: 'accessories', obd2: $('acc-obd2').checked, dvr: $('acc-dvr').checked })));
$('acc-wifi').addEventListener('change', () => { $('wifi-row').hidden = !$('acc-wifi').checked; sendWifi(); });
$('in-ssid').addEventListener('change', sendWifi);
function sendWifi() { if (started) worker.postMessage({ cmd: 'wifi', spec: $('acc-wifi').checked ? $('in-ssid').value : '' }); }

// ---------------------------------------------------------------- GPS
document.querySelectorAll('.seg button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.seg button').forEach((x) => x.classList.toggle('on', x === b));
  document.querySelectorAll('.tab').forEach((p) => { p.hidden = p.dataset.pane !== b.dataset.tab; });
  if (b.dataset.tab === 'map' && map) map.invalidateSize();   // its size may have changed while hidden
}));
function speed() { return +$('gps-speed').value; }
// tunnel: the GPS sky is blocked (no fix) while the drive goes on
function showCut(on) {
  gpsCut = on;
  const b = $('gps-cut');
  b.classList.toggle('on', on); b.setAttribute('aria-pressed', on); b.textContent = on ? 'Leave tunnel' : 'Enter tunnel';
}
function toggleCut() {
  if (!started) return msg('Start the unit first.', 'err');
  showCut(!gpsCut);
  worker.postMessage({ cmd: 'gpscut', on: gpsCut });
}
$('gps-cut').addEventListener('click', toggleCut);
function sendRoute(text, hold) {
  const pts = text.split(/\r?\n/).map((l) => l.split('#')[0]).flatMap((l) => l.split(';')).map((p) => p.trim()).filter((p) => /^-?[\d.]+\s*,\s*-?[\d.]+(\s*,\s*-?[\d.]+)?$/.test(p));
  if (!pts.length) return msg('The route has no points.', 'err');
  showRoute(pts);
  if (!started) { pendingRoute = { pts, hold }; msg(`Route of ${pts.length} points loaded: the unit drives it once you switch it on.`, 'ok'); return; }
  worker.postMessage({ cmd: 'route', route: pts.join(';'), speed: speed(), hold });
  msg(`Driving a route of ${pts.length} points at ${speed()} km/h.`, 'ok');
}
$('route-sample').addEventListener('change', async () => {
  const v = $('route-sample').value;
  if (!v) return;
  try { sendRoute(await (await fetch(v)).text(), false); } catch (e) { msg('Route not loaded: ' + e.message, 'err'); }
});
// cleared after each choice, so that choosing the same file again loads it again
$('in-route').addEventListener('change', async () => { const f = $('in-route').files[0]; $('in-route').value = ''; if (f) sendRoute(await f.text(), false); });
$('radar-strength').addEventListener('input', () => {
  const v = +$('radar-strength').value;
  $('radar-strength-v').textContent = v ? `${v} %` : 'off';
  started && worker.postMessage({ cmd: 'radar', strength: v / 100 });
});
$('gps-speed').addEventListener('input', () => { $('gps-speed-v').textContent = `${speed()} km/h`; started && worker.postMessage({ cmd: 'speed', speed: speed() }); });
// manual driving: a straight line ahead in the chosen heading, re-aimed on every turn
function ahead(p, deg, km) {
  const r = 6371, d = km / r, b = deg * Math.PI / 180, la = p[0] * Math.PI / 180, lo = p[1] * Math.PI / 180;
  const la2 = Math.asin(Math.sin(la) * Math.cos(d) + Math.cos(la) * Math.sin(d) * Math.cos(b));
  const lo2 = lo + Math.atan2(Math.sin(b) * Math.sin(d) * Math.cos(la), Math.cos(d) - Math.sin(la) * Math.sin(la2));
  return [la2 * 180 / Math.PI, lo2 * 180 / Math.PI];
}
function steer(delta) {
  heading = (heading + delta + 360) % 360;
  $('drv-heading').textContent = `${heading}°`;
  if (!started || !car) return;
  const to = ahead(car, heading, 20);
  worker.postMessage({ cmd: 'route', route: `${car[0]},${car[1]};${to[0]},${to[1]}`, speed: speed(), hold: true });
}
$('drv-left').addEventListener('click', () => steer(-15));
$('drv-right').addEventListener('click', () => steer(15));
$('drv-go').addEventListener('click', () => {
  const p = [+$('drv-lat').value, +$('drv-lon').value];
  if (!isFinite(p[0]) || !isFinite(p[1])) return msg('Enter a latitude and longitude in degrees.', 'err');
  car = p;
  if (started) worker.postMessage({ cmd: 'route', route: `${p[0]},${p[1]}`, speed: speed(), hold: true });
  steer(0);
});
// optional OpenStreetMap view (Leaflet and tiles are fetched only after the user asks for it)
let map = null, carMarker = null, target = null, routeLine = null, routePts = null;
// the loaded route on the map, the whole of it in view
function showRoute(pts) { routePts = pts.map((p) => p.split(',').slice(0, 2).map(Number)); if (map) drawRoute(); }
function drawRoute() {
  if (routeLine) routeLine.remove();
  routeLine = L.polyline(routePts, { color: '#2563eb', weight: 3, opacity: 0.5 }).addTo(map);
  map.fitBounds(routeLine.getBounds(), { padding: [20, 20] });
}
// keep the car in view when it drives (or jumps) out of it; not while the map has been moved away from the car
function follow(prev) {
  if (!map || !prev) return;
  const b = map.getBounds();
  if (b.contains(prev) && !b.contains(car)) map.panTo(car);
}
$('map-enable').addEventListener('click', async () => {
  $('map-enable').disabled = true;
  try {
    await new Promise((res, rej) => {
      const css = document.createElement('link'); css.rel = 'stylesheet'; css.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css'; document.head.append(css);
      const js = document.createElement('script'); js.src = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'; js.onload = res; js.onerror = () => rej(new Error('could not load Leaflet')); document.head.append(js);
    });
    $('map').hidden = false; $('map-enable').hidden = true;
    const at = car || [35.6812, 139.7671];
    map = L.map('map').setView(at, 14);
    L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '© OpenStreetMap contributors' }).addTo(map);
    carMarker = L.circleMarker(at, { radius: 7, color: '#2563eb', fillOpacity: .9 }).addTo(map);
    if (routePts) drawRoute();
    map.on('click', (e) => {
      if (!started) return msg('Start the unit first.', 'err');
      const p = [e.latlng.lat, e.latlng.lng];
      if (e.originalEvent.shiftKey || !car) { worker.postMessage({ cmd: 'route', route: `${p[0]},${p[1]}`, speed: speed(), hold: true }); car = p; carMarker.setLatLng(p); }
      else worker.postMessage({ cmd: 'route', route: `${car[0]},${car[1]};${p[0]},${p[1]}`, speed: speed(), hold: true });
      if (target) target.remove();
      target = L.circleMarker(p, { radius: 5, color: '#dc2626' }).addTo(map);
    });
  } catch (e) { msg('Map not available: ' + e.message, 'err'); $('map-enable').disabled = false; }
});

// ---------------------------------------------------------------- OBD-II values
const obd = { speed: [0, 0, 180, 'km/h'], rpm: [750, 0, 8000, 'rpm'], coolant: [88, -40, 130, '°C'], intake: [30, -40, 80, '°C'], throttle: [12, 0, 100, '%'], load: [25, 0, 100, '%'], boost: [-40, -100, 150, 'kPa'], battery: [14.2, 9, 16, 'V'] };
for (const [k, [v, lo, hi, unit]] of Object.entries(obd)) {
  const l = document.createElement('label'); l.textContent = k; l.htmlFor = 'obd-' + k;
  const r = Object.assign(document.createElement('input'), { type: 'range', min: lo, max: hi, step: k === 'battery' ? 0.1 : 1, value: v, id: 'obd-' + k });
  const o = document.createElement('output'); o.textContent = `${v} ${unit}`;
  r.addEventListener('input', () => { o.textContent = `${r.value} ${unit}`; sendObd(); });
  $('obd-grid').append(l, r, o);
}
$('obd-follow').addEventListener('change', sendObd);
function sendObd() {
  if (!started) return;
  const v = (k) => +$('obd-' + k).value;
  worker.postMessage({ cmd: 'obd', speed: v('speed'), rpm: v('rpm'), coolant: v('coolant'), intake: v('intake'), throttle: v('throttle'), load: v('load'), boost: v('boost'), battery: v('battery'), follow: $('obd-follow').checked });
}

// ---------------------------------------------------------------- audio (44.1 kHz stereo from the unit's SSI)
let audio = null;
function startAudio() {
  if (audio) return;
  try {
    const ac = new AudioContext();
    const q = []; let qi = 0, phase = 0;
    const step = 44100 / ac.sampleRate;
    const node = ac.createScriptProcessor(2048, 0, 2);
    node.onaudioprocess = (ev) => {
      const Lc = ev.outputBuffer.getChannelData(0), Rc = ev.outputBuffer.getChannelData(1);
      for (let i = 0; i < Lc.length; i++) {
        while (q.length && qi >= q[0].length / 2) { q.shift(); qi = 0; }
        const c = q[0];
        Lc[i] = c ? c[qi * 2] / 32768 : 0; Rc[i] = c ? c[qi * 2 + 1] / 32768 : 0;
        phase += step; while (phase >= 1) { phase -= 1; qi++; }
      }
    };
    node.connect(ac.destination);
    audio = { push: (pcm) => { q.push(pcm); while (q.length > 40) q.shift(); } };
  } catch (_) { audio = null; }
}

applyScale();
refresh();
