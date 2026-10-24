// Mocap map editor: polls /api/state, draws markers/car/obstacles/occupancy on a
// canvas, and turns marker selections into obstacles via the REST API.
'use strict';

const POLL_MS = 100;
const MATCH_TOL = 0.05;       // m, re-associate selected markers across polls
const PICK_PX = 10;           // click radius for marker picking
const MIN_MARKERS = { polygon: 3, polyline: 2, loop: 3 };
const KIND_COLORS = { polygon: '#e04d33', polyline: '#f0a51a', loop: '#3389f2' };

const $ = (id) => document.getElementById(id);
const canvas = $('view');
const ctx = canvas.getContext('2d');

let data = null;                 // latest /api/state snapshot
let occ = { version: -1, img: null, grid: null };
let listVersion = -1;
let selection = [];              // [{x, y, z, id, missing}] in click order
let kind = 'polygon';
let preview = null;
let view = { cx: 0, cy: 0, scale: 60 };  // world centre [m], px per m
let viewInitialised = false;
let drag = null;
let cursor = null;

// ---- helpers ------------------------------------------------------------

function w2s(x, y) {
  return [canvas.clientWidth / 2 + (x - view.cx) * view.scale,
          canvas.clientHeight / 2 - (y - view.cy) * view.scale];
}

function s2w(px, py) {
  return [view.cx + (px - canvas.clientWidth / 2) / view.scale,
          view.cy - (py - canvas.clientHeight / 2) / view.scale];
}

async function api(method, path, body) {
  const opts = { method, headers: {} };
  if (body !== undefined) {
    opts.headers['Content-Type'] = 'application/json';
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(path, opts);
  const json = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(json.error || `${method} ${path}: HTTP ${res.status}`);
  return json;
}

function toast(msg, isError = false) {
  const el = $('toast');
  el.textContent = msg;
  el.className = isError ? 'error' : '';
}

const fmt = (v, d = 2) => (v === null || v === undefined ? '—' : Number(v).toFixed(d));

// ---- polling ------------------------------------------------------------

async function poll() {
  try {
    data = await api('GET', '/api/state');
    reassociateSelection();
    if (data.version !== occ.version) await loadOccupancy();
    if (data.version !== listVersion) {
      renderObstacleList();
      fillGridForm();
      listVersion = data.version;
    }
    if (!viewInitialised) { fitView(); viewInitialised = true; }
    renderStatus();
  } catch (err) {
    $('st-markers').textContent = 'node unreachable';
  }
  draw();
  setTimeout(poll, POLL_MS);
}

async function loadOccupancy() {
  const o = await api('GET', '/api/occupancy');
  const g = o.grid;
  const bytes = Uint8Array.from(atob(o.data), (c) => c.charCodeAt(0));
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  for (let r = 0; r < g.height; r++) {
    const dst = (g.height - 1 - r) * g.width;   // grid row 0 = min y = image bottom
    for (let c = 0; c < g.width; c++) {
      const v = (bytes[r * g.width + c] << 24) >> 24;  // int8
      const i = (dst + c) * 4;
      if (v >= 100) { img.data[i] = 30; img.data[i + 1] = 35; img.data[i + 2] = 50; img.data[i + 3] = 150; }
      else if (v < 0) { img.data[i] = 128; img.data[i + 1] = 128; img.data[i + 2] = 128; img.data[i + 3] = 60; }
    }
  }
  octx.putImageData(img, 0, 0);
  occ = { version: o.version, img: off, grid: g };
}

function reassociateSelection() {
  for (const s of selection) {
    let best = null;
    let bestD = MATCH_TOL;
    for (const m of data.markers) {
      const d = Math.hypot(m.x - s.x, m.y - s.y, m.z - s.z);
      if (d <= bestD) { best = m; bestD = d; }
    }
    s.missing = !best;
    if (best) Object.assign(s, { x: best.x, y: best.y, z: best.z, id: best.id });
  }
}

// ---- status / lists / forms --------------------------------------------

function renderStatus() {
  const nEx = data.markers.filter((m) => m.excluded).length;
  const age = data.markers_age === null ? 'no data' : `${fmt(data.markers_age * 1000, 0)} ms ago`;
  $('st-markers').textContent = `${data.markers.length} (${nEx} on car) · ${age}`;
  const c = data.car;
  $('st-car').textContent = c
    ? `(${fmt(c.x)}, ${fmt(c.y)}) ${fmt(c.yaw * 180 / Math.PI, 0)}° · ${fmt(c.age * 1000, 0)} ms`
    : 'not tracked';
  $('st-up').textContent = data.up_axis;
  const note = $('st-note');
  let warn = data.load_note || '';
  if (data.raw_range) {
    const [rx, ry, rz] = data.raw_range;
    $('st-spread').textContent = `x ${fmt(rx)} · y ${fmt(ry)} · z ${fmt(rz)} m`;
    // Markers mostly sit near the floor, so the up axis usually has the smallest spread.
    if (data.markers.length >= 4) {
      const flat = ry < rz ? 'y' : 'z';
      if (flat !== data.up_axis && Math.min(ry, rz) < 0.5 * Math.max(rx, ry, rz)) {
        warn += `${warn ? ' ' : ''}Raw ${flat.toUpperCase()} has the smallest spread; check up_axis (currently ${data.up_axis}).`;
      }
    }
  } else {
    $('st-spread').textContent = '—';
  }
  note.hidden = !warn;
  note.textContent = warn;
  $('sel-count').textContent = `${selection.length} selected`;
}

function renderObstacleList() {
  const ul = $('obstacles');
  ul.innerHTML = '';
  if (!data.obstacles.length) {
    ul.innerHTML = '<li class="muted">none yet</li>';
    return;
  }
  for (const ob of data.obstacles) {
    const li = document.createElement('li');
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = KIND_COLORS[ob.kind] || '#888';
    const nameBox = document.createElement('div');
    const name = document.createElement('input');
    name.value = ob.name;
    name.addEventListener('change', () => patchObstacle(ob.id, { name: name.value }));
    const meta = document.createElement('div');
    meta.className = 'meta';
    meta.textContent = `${ob.kind} · ${ob.vertices.length} pts`
      + (ob.width ? ` · w ${fmt(ob.width)}` : '');
    nameBox.append(name, meta);
    const infl = document.createElement('input');
    infl.type = 'number';
    infl.step = '0.01';
    infl.min = '0';
    infl.value = ob.inflation;
    infl.title = 'inflation [m]';
    infl.style.width = '64px';
    infl.addEventListener('change', () => patchObstacle(ob.id, { inflation: Number(infl.value) }));
    const del = document.createElement('button');
    del.textContent = '✕';
    del.title = 'delete';
    del.addEventListener('click', async () => {
      if (!confirm(`Delete obstacle "${ob.name}"?`)) return;
      try { await api('DELETE', `/api/obstacles/${ob.id}`); toast(`deleted ${ob.name}`); }
      catch (e) { toast(e.message, true); }
    });
    li.append(sw, nameBox, infl, del);
    ul.append(li);
  }
}

async function patchObstacle(id, fields) {
  try { await api('PATCH', `/api/obstacles/${id}`, fields); toast('updated'); }
  catch (e) { toast(e.message, true); listVersion = -1; }
}

function fillGridForm() {
  const g = data.grid;
  const set = (id, v) => { if (document.activeElement !== $(id)) $(id).value = +v.toFixed(3); };
  set('g-res', g.resolution);
  set('g-ox', g.origin_x);
  set('g-oy', g.origin_y);
  set('g-sx', g.width * g.resolution);
  set('g-sy', g.height * g.resolution);
  $('grid-msg').textContent = `${g.width} × ${g.height} cells`
    + (data.map_file ? ` · ${data.map_file}` : '');
}

// ---- obstacle creation ---------------------------------------------------

function obstacleBody() {
  return {
    kind,
    order: $('order').value || undefined,
    name: $('name').value.trim() || undefined,
    width: kind === 'polygon' ? undefined : Number($('width').value),
    inflation: Number($('inflation').value),
    points: selection.map(({ x, y, z, id }) => ({ x, y, z, id })),
  };
}

let previewTimer = null;
function schedulePreview() {
  clearTimeout(previewTimer);
  previewTimer = setTimeout(async () => {
    const need = MIN_MARKERS[kind];
    if (selection.length < need) {
      preview = null;
      $('preview-msg').textContent = selection.length
        ? `select at least ${need} markers for a ${kind}` : '';
      draw();
      return;
    }
    try {
      preview = await api('POST', '/api/preview', obstacleBody());
      $('preview-msg').textContent = `preview: ${preview.vertices.length} vertices, numbers show the order`;
    } catch (e) {
      preview = null;
      $('preview-msg').textContent = e.message;
    }
    draw();
  }, 150);
}

async function createObstacle() {
  try {
    const ob = await api('POST', '/api/obstacles', obstacleBody());
    toast(`created ${ob.name}`);
    selection = [];
    preview = null;
    $('name').value = '';
    $('preview-msg').textContent = '';
  } catch (e) {
    toast(e.message, true);
  }
}

function setKind(k) {
  kind = k;
  for (const b of document.querySelectorAll('#kind button')) b.classList.toggle('on', b.dataset.kind === k);
  $('width-label').style.visibility = k === 'polygon' ? 'hidden' : 'visible';
  schedulePreview();
}

// ---- drawing --------------------------------------------------------------

function resize() {
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(canvas.clientWidth * dpr);
  canvas.height = Math.round(canvas.clientHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}

function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawMetricGrid() {
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  const steps = [0.05, 0.1, 0.25, 0.5, 1, 2, 5, 10, 20];
  const step = steps.find((s) => s * view.scale >= 40) || 50;
  const [x0, y1] = s2w(0, 0);
  const [x1, y0] = s2w(W, H);
  ctx.lineWidth = 1;
  ctx.font = '10px system-ui';
  ctx.fillStyle = css('--muted');
  for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
    const [sx] = w2s(x, 0);
    ctx.strokeStyle = Math.abs(x) < 1e-9 ? css('--muted') : css('--line');
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, H); ctx.stroke();
    ctx.fillText(+x.toFixed(2), sx + 2, H - 4);
  }
  for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
    const [, sy] = w2s(0, y);
    ctx.strokeStyle = Math.abs(y) < 1e-9 ? css('--muted') : css('--line');
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
    ctx.fillText(+y.toFixed(2), 2, sy - 2);
  }
}

function drawOccupancy() {
  const g = occ.grid;
  if (!g) return;
  const [sx, sy] = w2s(g.origin_x, g.origin_y + g.height * g.resolution);
  const w = g.width * g.resolution * view.scale;
  const h = g.height * g.resolution * view.scale;
  if ($('v-occ').checked && occ.img) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(occ.img, sx, sy, w, h);
  }
  ctx.strokeStyle = css('--accent');
  ctx.setLineDash([6, 4]);
  ctx.strokeRect(sx, sy, w, h);
  ctx.setLineDash([]);
}

function tracePath(verts, closed) {
  ctx.beginPath();
  verts.forEach((v, i) => {
    const [x, y] = w2s(v[0], v[1]);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  if (closed) ctx.closePath();
}

function drawObstacle(ob, isPreview) {
  const color = KIND_COLORS[ob.kind] || '#888';
  const closed = ob.kind !== 'polyline';
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  if (isPreview) ctx.setLineDash([5, 4]);
  tracePath(ob.vertices, closed);
  if (ob.kind === 'polygon') {
    ctx.fillStyle = color + (isPreview ? '22' : '40');
    ctx.fill();
    ctx.lineWidth = 2;
  } else {
    ctx.lineWidth = Math.max(2, ob.width * view.scale);
    ctx.globalAlpha = isPreview ? 0.35 : 0.55;
  }
  ctx.strokeStyle = color;
  ctx.stroke();
  ctx.globalAlpha = 1;
  ctx.setLineDash([]);
  if (isPreview) {
    ctx.fillStyle = css('--text');
    ctx.font = 'bold 11px system-ui';
    ob.vertices.forEach((v, i) => {
      const [x, y] = w2s(v[0], v[1]);
      ctx.fillText(String(i + 1), x + 7, y + 12);
    });
  }
}

function drawCar(c) {
  const [x, y] = w2s(c.x, c.y);
  const L = Math.max(14, 0.45 * view.scale);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-c.yaw);
  ctx.beginPath();
  ctx.moveTo(L * 0.6, 0);
  ctx.lineTo(-L * 0.4, L * 0.3);
  ctx.lineTo(-L * 0.4, -L * 0.3);
  ctx.closePath();
  ctx.fillStyle = '#21a366cc';
  ctx.fill();
  ctx.strokeStyle = '#167a4b';
  ctx.stroke();
  ctx.restore();
}

function draw() {
  const W = canvas.clientWidth;
  const H = canvas.clientHeight;
  ctx.clearRect(0, 0, W, H);
  if ($('v-lines').checked) drawMetricGrid();
  if (!data) return;
  drawOccupancy();
  for (const ob of data.obstacles) drawObstacle(ob, false);
  if (preview) drawObstacle(preview, true);

  for (const m of data.markers) {
    const [x, y] = w2s(m.x, m.y);
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, 2 * Math.PI);
    ctx.fillStyle = m.excluded ? '#8a8f99' : css('--text');
    ctx.fill();
  }
  selection.forEach((s, i) => {
    const [x, y] = w2s(s.x, s.y);
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, 2 * Math.PI);
    ctx.strokeStyle = s.missing ? '#d33' : css('--accent');
    ctx.lineWidth = 2;
    ctx.stroke();
    if (!preview) {
      ctx.fillStyle = css('--accent');
      ctx.font = 'bold 11px system-ui';
      ctx.fillText(String(i + 1), x + 9, y - 6);
    }
  });
  if (data.car) drawCar(data.car);

  if (drag && drag.box) {
    ctx.strokeStyle = css('--accent');
    ctx.setLineDash([4, 3]);
    ctx.strokeRect(drag.x0, drag.y0, drag.x - drag.x0, drag.y - drag.y0);
    ctx.setLineDash([]);
  }
}

function fitView() {
  const xs = [];
  const ys = [];
  if (data.grid) {
    const g = data.grid;
    xs.push(g.origin_x, g.origin_x + g.width * g.resolution);
    ys.push(g.origin_y, g.origin_y + g.height * g.resolution);
  }
  for (const m of data.markers) { xs.push(m.x); ys.push(m.y); }
  if (!xs.length) return;
  const [xmin, xmax, ymin, ymax] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  view.cx = (xmin + xmax) / 2;
  view.cy = (ymin + ymax) / 2;
  view.scale = 0.9 * Math.min(canvas.clientWidth / Math.max(xmax - xmin, 0.5),
                              canvas.clientHeight / Math.max(ymax - ymin, 0.5));
  draw();
}

// ---- interaction ------------------------------------------------------------

function toggleMarkerAt(px, py) {
  let best = null;
  let bestD = PICK_PX;
  for (const m of data.markers) {
    const [x, y] = w2s(m.x, m.y);
    const d = Math.hypot(x - px, y - py);
    if (d <= bestD) { best = m; bestD = d; }
  }
  if (!best) return;
  if (best.excluded) { toast('that marker belongs to the car', true); return; }
  const i = selection.findIndex((s) => Math.hypot(s.x - best.x, s.y - best.y, s.z - best.z) <= MATCH_TOL);
  if (i >= 0) selection.splice(i, 1);
  else selection.push({ x: best.x, y: best.y, z: best.z, id: best.id, missing: false });
  schedulePreview();
}

function boxSelect(x0, y0, x1, y1) {
  const [lx, hx] = [Math.min(x0, x1), Math.max(x0, x1)];
  const [ly, hy] = [Math.min(y0, y1), Math.max(y0, y1)];
  for (const m of data.markers) {
    const [x, y] = w2s(m.x, m.y);
    if (m.excluded || x < lx || x > hx || y < ly || y > hy) continue;
    if (selection.some((s) => Math.hypot(s.x - m.x, s.y - m.y, s.z - m.z) <= MATCH_TOL)) continue;
    selection.push({ x: m.x, y: m.y, z: m.z, id: m.id, missing: false });
  }
  schedulePreview();
}

canvas.addEventListener('mousedown', (e) => {
  drag = { x0: e.offsetX, y0: e.offsetY, x: e.offsetX, y: e.offsetY,
           box: e.shiftKey, moved: false, cx: view.cx, cy: view.cy };
});

window.addEventListener('mousemove', (e) => {
  const r = canvas.getBoundingClientRect();
  const px = e.clientX - r.left;
  const py = e.clientY - r.top;
  cursor = s2w(px, py);
  $('hud-cursor').textContent = `x ${fmt(cursor[0])}, y ${fmt(cursor[1])}`;
  if (!drag) return;
  drag.x = px;
  drag.y = py;
  if (Math.hypot(px - drag.x0, py - drag.y0) > 3) drag.moved = true;
  if (!drag.box && drag.moved) {
    view.cx = drag.cx - (px - drag.x0) / view.scale;
    view.cy = drag.cy + (py - drag.y0) / view.scale;
  }
  draw();
});

window.addEventListener('mouseup', () => {
  if (!drag) return;
  if (data) {
    if (drag.box && drag.moved) boxSelect(drag.x0, drag.y0, drag.x, drag.y);
    else if (!drag.moved) toggleMarkerAt(drag.x0, drag.y0);
  }
  drag = null;
  draw();
});

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const [wx, wy] = s2w(e.offsetX, e.offsetY);
  view.scale = Math.min(5000, Math.max(2, view.scale * Math.exp(-e.deltaY * 0.0015)));
  const [nx, ny] = s2w(e.offsetX, e.offsetY);
  view.cx += wx - nx;
  view.cy += wy - ny;
  draw();
}, { passive: false });

for (const b of document.querySelectorAll('#kind button')) {
  b.addEventListener('click', () => setKind(b.dataset.kind));
}
for (const id of ['order', 'width', 'inflation']) $(id).addEventListener('input', schedulePreview);
$('btn-create').addEventListener('click', createObstacle);
$('btn-clear').addEventListener('click', () => { selection = []; schedulePreview(); });

$('btn-grid').addEventListener('click', async () => {
  const res = Number($('g-res').value);
  try {
    await api('PUT', '/api/grid', {
      resolution: res,
      origin_x: Number($('g-ox').value),
      origin_y: Number($('g-oy').value),
      width: Math.ceil(Number($('g-sx').value) / res - 1e-9),
      height: Math.ceil(Number($('g-sy').value) / res - 1e-9),
    });
    toast('grid updated');
  } catch (e) { toast(e.message, true); }
});

$('btn-fit').addEventListener('click', async () => {
  try {
    await api('POST', '/api/grid/fit', { margin: Number($('g-margin').value), resolution: Number($('g-res').value) });
    toast('grid fitted to obstacles');
  } catch (e) { toast(e.message, true); }
});

$('btn-save').addEventListener('click', async () => {
  try {
    const r = await api('POST', '/api/save', {});
    toast(`saved ${r.json} (+ ${r.pgm.split('/').pop()}, ${r.yaml.split('/').pop()})`);
  } catch (e) { toast(e.message, true); }
});

$('v-occ').addEventListener('change', draw);
$('v-lines').addEventListener('change', draw);
$('btn-fitview').addEventListener('click', () => data && fitView());
window.addEventListener('resize', resize);

setKind('polygon');
resize();
poll();
