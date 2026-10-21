(function () {
  'use strict';
  const G = window.ClimbGeometry;
  const $ = id => document.getElementById(id);
  const canvas = $('scene'), ctx = canvas.getContext('2d');
  const plot = $('force-plot'), pctx = plot.getContext('2d');
  const palette = ['#79cbbb', '#e3a16e', '#b4a5cf', '#d2ca88'];
  const limbLabels = {left_hand: 'Left hand', right_hand: 'Right hand', left_foot: 'Left foot', right_foot: 'Right foot'};
  const phases = {initialization: 'Starting stance', initial: 'Starting stance', initial_settle: 'Settling the start', start: 'Starting stance', reachability: 'Reachability check', planning: 'Planning the reach', dynamic: 'Controlled transition', transition: 'Controlled transition', settle: 'Settling the stance', complete: 'Attempt complete', failure: 'Attempt stopped'};
  const state = {demos: [], demo: null, time: 0, playing: false, speed: 1, index: 0, last: null, view: 'orbit', azimuth: -.34, elevation: .13, zoom: 1, forceSeries: [], peak: 0};
  const camera = {target: [0, .1, 1.3], azimuth: -.34, elevation: .13, scale: 180};
  let width = 800, height = 440, plotWidth = 270, plotHeight = 85, dirty = true, uiIndex = -1;
  let drag = null;

  const human = value => String(value || '').replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
  const phaseName = value => phases[value] || human(value) || 'Recorded motion';
  const duration = () => state.demo?.frames.at(-1)?.t || 0;
  const rgba = (hex, alpha) => `rgba(${parseInt(hex.slice(1, 3), 16)},${parseInt(hex.slice(3, 5), 16)},${parseInt(hex.slice(5, 7), 16)},${alpha})`;
  const point = p => G.project(p, camera, width, height);
  function make(tag, className, content) { const node = document.createElement(tag); if (className) node.className = className; if (content !== undefined) node.textContent = content; return node; }

  function normalizeDemo(data) {
    if (data?.frames?.length === 0) {
      if (data.success !== false || typeof data.route !== 'string' || !Array.isArray(data.outcomes) || !data.outcomes.some(o => o.passed === false) || !Array.isArray(data.geometry?.holds)) throw new Error('An empty trajectory must include a failed result and its diagnostic.');
      // Exercise the common schema checks without inventing a frame for display.
      // This temporary validation-only transform is discarded immediately.
      if (!Array.isArray(data.bodyNames)) throw new Error('The failed trajectory must name its model bodies.');
      G.validateTrajectory({...data, frames: [{t: 0, pos: data.bodyNames.map(() => [0, 0, 0]), quat: data.bodyNames.map(() => [1, 0, 0, 0])}]});
    } else G.validateTrajectory(data);
    return {...data, limbs: data.limbs || ['left_hand', 'right_hand', 'left_foot', 'right_foot'], outcomes: data.outcomes || []};
  }

  function gripInfo(frame, i) {
    const anchored = Boolean(frame.grips?.[i]);
    let hold = state.demo.geometry.holds.find(h => h.name === frame.holdNames?.[i]);
    if (!hold && frame.anchors?.[i]) hold = state.demo.geometry.holds.find(h => G.norm(G.sub(h.pos, frame.anchors[i])) < .025);
    const force = frame.forces?.[i];
    const load = Array.isArray(force) && force.every(Number.isFinite) ? G.norm(force) : null;
    const cap = hold?.maxGripN;
    return {anchored, hold, force, load, cap, ratio: Number.isFinite(load) && cap > 0 ? load / cap : null};
  }

  function createReadouts() {
    $('grip-readouts').replaceChildren(...state.demo.limbs.map((limb, i) => {
      const row = make('div', 'grip-row'); row.style.setProperty('--limb', palette[i % 4]); row.dataset.limb = limb;
      const top = make('div', 'grip-topline'), name = make('span', 'limb-name', limbLabels[limb] || human(limb)); name.prepend(make('i', 'limb-dot'));
      top.append(name, make('span', 'grip-value'));
      const meter = make('div', 'grip-meter'); meter.append(make('span'));
      const foot = make('div', 'grip-footnote'); foot.append(make('span', 'hold-name'), make('span', 'grip-state'));
      row.append(top, meter, foot); return row;
    }));
  }

  function selectDemo(index) {
    state.demo = state.demos[index]; if (!state.demo) return;
    state.time = 0; state.playing = false; state.index = 0; uiIndex = -1; state.peak = 0;
    const d = state.demo;
    $('replay-origin').textContent = d.freshRun ? 'FRESH MUJOCO RUN' : 'RECORDED EXAMPLE';
    $('route-select').value = String(index);
    $('timeline').max = String(duration()); $('timeline').value = '0';
    ['play', 'restart', 'timeline'].forEach(id => $(id).disabled = !d.frames.length);
    $('scene-message').hidden = Boolean(d.frames.length);
    if (!d.frames.length) $('scene-message').replaceChildren(make('strong', '', 'No motion was recorded.'), make('p', '', 'The starting pose could not be established. See the attempt diagnostic below, then adjust the route in the editor.'));
    $('result-badge').textContent = d.success ? 'CONTROLLER COMPLETED' : 'NOT COMPLETED';
    $('result-badge').classList.toggle('failed', !d.success);
    const metadata = d.metadata || d;
    const mass = d.massKg ?? metadata.massKg, step = d.timestep ?? metadata.timestep;
    $('model-stats').textContent = `${Number.isFinite(mass) ? mass.toFixed(1) : '—'} kg · ${Number.isFinite(step) ? (step * 1000).toFixed(0) : '—'} ms step`;
    state.forceSeries = d.limbs.map((_, i) => d.frames.map(f => { const info = gripInfo(f, i); const ratio = info.anchored ? info.ratio : 0; if (Number.isFinite(ratio)) state.peak = Math.max(state.peak, ratio); return ratio; }));
    $('peak-load').replaceChildren(document.createTextNode(state.forceSeries.some(series => series.some(v => v > 0)) ? String(Math.round(state.peak * 100)) : '—'), make('small', '', '%'));
    $('plot-duration').textContent = `0 → ${duration().toFixed(2)} s`;
    createReadouts(); renderVerdict(); resetCamera();
    // Open on a real supported stance after initial settling. Restart still begins at t=0.
    const preview = d.frames.filter(f => f.t <= 2.8 && f.grips?.every(Boolean) && (f.moveIndex < 0 || /^initial/.test(f.phase || ''))).at(-1);
    if (preview) { state.time = preview.t; state.index = G.frameIndex(d.frames, state.time); }
    updateUI(); dirty = true;
  }

  function renderVerdict() {
    const d = state.demo, failure = d.outcomes.find(o => !o.passed);
    const reason = failure?.reason || '';
    let title, description;
    if (d.success) { title = 'Model/controller completed.'; description = 'This attempt passed its reachability, transition, and settle checks within the simplified model and configured grip limits. Wall clearance is reported separately.'; }
    else if (failure?.phase === 'reachability') { title = failure.detail?.category === 'geometric_reach_limit' ? 'Beyond the current reach.' : 'No target pose found.'; description = 'The reachability check rejected the target from the current stance. This controlled attempt stopped before that transition.'; }
    else if (failure?.phase === 'initialization') { title = 'The starting stance fails.'; description = 'The model could not establish a valid starting configuration. The exact initialization diagnostic is recorded below.'; }
    else if (/grip|friction|force|cone|overload|strength/i.test(reason)) { title = 'A grip reaches its limit.'; description = 'A recorded grip load violated a configured force rating or friction constraint. Inspect the force history and exact diagnostic.'; }
    else { title = 'The controller stops here.'; description = 'The controlled transition did not pass. A controller or planning failure does not establish that the move is physically impossible.'; }
    $('verdict-title').textContent = title; $('verdict-description').textContent = description;
    $('failure-detail').hidden = !failure;
    if (failure) {
      const fields = {force_n: ['Force', 'N'], normal_n: ['Normal', 'N'], tangential_n: ['Tangential', 'N'], time_s: ['At', 's'], distance_m: ['Target distance', 'm'], max_reach_m: ['Max reach', 'm'], residual_m: ['Residual', 'm'], error_m: ['Tracking error', 'm']};
      const detail = typeof failure.detail === 'string' ? failure.detail : Object.entries(failure.detail || {}).filter(([key, value]) => fields[key] && Number.isFinite(value)).map(([key, value]) => `${fields[key][0]} ${value.toFixed(fields[key][1] === 'N' ? 1 : 3)} ${fields[key][1]}`).join(' · ');
      $('failure-detail').replaceChildren(document.createTextNode(reason));
      if (detail) { $('failure-detail').append(make('br'), document.createTextNode(detail)); }
    }
    $('outcome-count').textContent = `${d.outcomes.filter(o => o.passed).length} / ${d.outcomes.length} CHECKS PASSED`;
    $('outcomes').replaceChildren(...d.outcomes.map(o => {
      const li = make('li', `outcome${o.passed ? '' : ' failed'}`);
      const main = make('div', 'outcome-main');
      main.append(make('strong', '', String(o.move || 'Starting stance').replace(/_/g, ' ').replace(/\s*->\s*/g, ' → ')), make('span', '', phaseName(o.phase) + (o.passed && o.reason ? ` · ${o.reason}` : '')));
      li.append(make('span', 'outcome-icon', o.passed ? '✓' : '×'), main, make('span', 'outcome-tag', o.passed ? 'PASS' : 'STOP')); return li;
    }));
    if (!d.outcomes.length) $('outcomes').append(make('li', 'outcome', 'No outcome diagnostics were included in this record.'));
  }

  function resetCamera(view = 'orbit') {
    state.view = view; state.zoom = 1;
    state.azimuth = view === 'front' ? 0 : view === 'side' ? Math.PI / 2 - .02 : -.34;
    state.elevation = view === 'front' || view === 'side' ? 0 : .13;
    const positions = [...state.demo.geometry.holds.map(h => h.pos), ...(state.demo.frames[0]?.pos || [[0, 0, 1]])];
    const xs = positions.map(p => p[0]), zs = positions.map(p => p[2]);
    const minX = Math.min(...xs) - .25, maxX = Math.max(...xs) + .25, minZ = Math.min(...zs) - .25, maxZ = Math.max(...zs) + .32;
    camera.target = [(minX + maxX) / 2, .14, (minZ + maxZ) / 2];
    camera.spanX = Math.max(maxX - minX, 1.6); camera.spanZ = Math.max(maxZ - minZ, 1.9);
    updateCameraButtons(); dirty = true;
  }

  function updateCameraButtons() {
    document.querySelectorAll('[data-view]').forEach(b => { const active = b.dataset.view === state.view; b.classList.toggle('active', active); b.setAttribute('aria-pressed', String(active)); });
  }

  function updateUI() {
    const d = state.demo; if (!d) return;
    const frame = d.frames[state.index];
    if (!frame) {
      $('frame-count').textContent = 'NO RECORDED FRAMES'; $('phase-label').textContent = 'Initialization stopped'; $('move-label').textContent = 'See the diagnostic below'; $('wall-warning').hidden = true;
      $('grip-readouts').querySelectorAll('.grip-value').forEach(node => { node.textContent = '—'; });
      $('contact-count').textContent = '—'; $('com-height').textContent = '—'; $('time-readout').textContent = '0.00 / 0.00 s'; $('play-state').textContent = 'NO RECORD';
      return;
    }
    $('timeline').value = String(state.time);
    $('time-readout').replaceChildren(document.createTextNode(state.time.toFixed(2) + ' '), make('span', '', `/ ${duration().toFixed(2)} s`));
    $('play-state').textContent = state.playing ? 'PLAYING' : state.time >= duration() && duration() ? 'END OF RECORD' : 'PAUSED';
    $('play').setAttribute('aria-label', state.playing ? 'Pause trajectory' : 'Play trajectory');
    $('play-icon').setAttribute('d', state.playing ? 'M5 3h4v14H5zM12 3h4v14h-4z' : 'M6 3 17 10 6 17Z');
    if (uiIndex === state.index) return;
    uiIndex = state.index;
    $('frame-count').textContent = `FRAME ${String(state.index + 1).padStart(3, '0')} / ${d.frames.length}`;
    $('phase-label').textContent = phaseName(frame.phase);
    const move = Number.isInteger(frame.moveIndex) && frame.moveIndex >= 0 ? (d.moves?.[frame.moveIndex] || d.outcomes.filter(o => o.move && o.move !== '(starting stance)')[frame.moveIndex]?.move) : null;
    const moveText = typeof move === 'string' ? move.replace(/_/g, ' ').replace('->', '→') : move ? `${human(move.limb)} → ${move.hold}` : '';
    $('move-label').textContent = Number.isInteger(frame.moveIndex) && frame.moveIndex >= 0 ? `MOVE ${String(frame.moveIndex + 1).padStart(2, '0')}${moveText ? ' · ' + moveText : ''}` : 'Establishing support on the wall';
    const rows = $('grip-readouts').children;
    d.limbs.forEach((_, i) => {
      const info = gripInfo(frame, i), row = rows[i];
      row.classList.toggle('released', !info.anchored); row.classList.toggle('overloaded', info.anchored && info.ratio > 1);
      row.querySelector('.grip-value').replaceChildren(document.createTextNode(info.load === null ? '—' : String(Math.round(info.load))), make('small', '', ` / ${info.cap ? Math.round(info.cap) : '—'} N`));
      row.querySelector('.grip-meter span').style.width = `${Math.min(100, (info.ratio || 0) * 100)}%`;
      row.querySelector('.hold-name').textContent = info.hold ? `${info.hold.name} · ${info.hold.type}` : 'NO HOLD DATA';
      row.querySelector('.grip-state').textContent = info.anchored ? info.ratio === null ? 'ANCHORED' : `${Math.round(info.ratio * 100)}% · ANCHORED` : 'RELEASED';
    });
    $('contact-count').replaceChildren(document.createTextNode(String(frame.grips?.filter(Boolean).length ?? 0)), make('small', '', `/ ${d.limbs.length}`));
    $('com-height').replaceChildren(document.createTextNode(Number.isFinite(frame.com?.[2]) ? frame.com[2].toFixed(2) : '—'), make('small', '', 'm'));
    const currentClearance = frame.wallClearanceM;
    const minClearance = d.minimumWallClearanceM ?? d.metadata?.minimumWallClearanceM;
    const intersectsNow = Number.isFinite(currentClearance) && currentClearance < -.001;
    const intersectsRun = Number.isFinite(minClearance) && minClearance < -.001;
    $('wall-warning').hidden = !intersectsNow && !intersectsRun;
    if (intersectsNow || intersectsRun) $('wall-warning').textContent = `Wall intersection${intersectsNow ? '' : ' in run'}: ${(-(intersectsNow ? currentClearance : minClearance) * 100).toFixed(1)} cm · collision response not modeled`;
  }

  function fitCanvas(target, context) {
    const rect = target.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
    target.width = Math.round(rect.width * dpr); target.height = Math.round(rect.height * dpr);
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    return [rect.width, rect.height];
  }

  function line(a, b, color, lineWidth = 1, dash = []) {
    const p = point(a), q = point(b); ctx.beginPath(); ctx.setLineDash(dash); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.stroke(); ctx.setLineDash([]);
  }

  function drawWall() {
    const wall = state.demo.geometry.wall || {pos: [0, -.05, 2], size: [3, .05, 3]}, [x, y0, z] = wall.pos, [sx, sy, sz] = wall.size, y = y0 + sy;
    const corners = [[x - sx, y, z - sz], [x + sx, y, z - sz], [x + sx, y, z + sz], [x - sx, y, z + sz]].map(point);
    ctx.beginPath(); corners.forEach((p, i) => i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)); ctx.closePath(); ctx.fillStyle = '#e8ebe1'; ctx.fill();
    ctx.save(); ctx.clip();
    for (let gx = Math.ceil((x - sx) * 4) / 4; gx <= x + sx; gx += .25) line([gx, y, z - sz], [gx, y, z + sz], Math.abs(gx % 1) < .001 ? '#ced7c8' : '#dce2d5', .65);
    for (let gz = Math.ceil((z - sz) * 4) / 4; gz <= z + sz; gz += .25) line([x - sx, y, gz], [x + sx, y, gz], Math.abs(gz % 1) < .001 ? '#cbd6c5' : '#dce2d5', .65);
    ctx.restore();
    const ruleX = camera.target[0] - Math.min(camera.spanX * .55, 1.05);
    ctx.font = '8px ui-monospace, Menlo, monospace'; ctx.fillStyle = '#8b9a85'; ctx.textAlign = 'right';
    for (let gz = 0; gz <= 5; gz += .5) { const p = point([ruleX, y, gz]); if (p.y > 108 && p.y < height - 48) { line([ruleX + .025, y, gz], [ruleX + .07, y, gz], '#a9b8a2', 1); ctx.fillText(gz.toFixed(1) + ' m', p.x - 4, p.y + 3); } }
    ctx.textAlign = 'left';
  }

  function sphere(p, radius, color, alpha = 1) {
    const r = Math.max(1, radius * camera.scale);
    const gradient = ctx.createRadialGradient(p.x - r * .35, p.y - r * .4, r * .03, p.x + r * .15, p.y + r * .2, r * 1.2);
    gradient.addColorStop(0, color); gradient.addColorStop(1, '#34484a');
    ctx.globalAlpha = alpha; ctx.fillStyle = gradient; ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.fill(); ctx.globalAlpha = 1;
    ctx.strokeStyle = '#18333288'; ctx.lineWidth = .7; ctx.stroke();
    ctx.beginPath(); ctx.arc(p.x - r * .25, p.y - r * .32, r * .22, Math.PI, Math.PI * 1.8); ctx.strokeStyle = '#ffffff45'; ctx.lineWidth = 1; ctx.stroke();
  }

  function capsule(a, b, radius, color) {
    const p = point(a), q = point(b), r = Math.max(1, radius * camera.scale), dx = q.x - p.x, dy = q.y - p.y, length = Math.hypot(dx, dy), nx = length ? -dy / length : 1, ny = length ? dx / length : 0;
    ctx.lineCap = 'round'; ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.strokeStyle = '#1c3738'; ctx.lineWidth = r * 2 + 1.7; ctx.stroke();
    const gradient = ctx.createLinearGradient(p.x - nx * r, p.y - ny * r, p.x + nx * r, p.y + ny * r); gradient.addColorStop(0, '#365254'); gradient.addColorStop(.35, color); gradient.addColorStop(.65, color); gradient.addColorStop(1, '#476063'); ctx.strokeStyle = gradient; ctx.lineWidth = r * 2; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x - nx * r * .3, p.y - ny * r * .3); ctx.lineTo(q.x - nx * r * .3, q.y - ny * r * .3); ctx.strokeStyle = '#f1f7e535'; ctx.lineWidth = Math.max(.7, r * .16); ctx.stroke(); ctx.lineCap = 'butt';
  }

  function drawBody(frame) {
    const shapes = [];
    state.demo.geometry.bodies.forEach(body => {
      const i = state.demo.bodyNames.indexOf(body.name); if (i < 0 || !frame.pos[i] || !frame.quat[i]) return;
      const bp = frame.pos[i], bq = frame.quat[i];
      body.geoms.forEach(geom => {
        const center = G.transform(bp, bq, geom); const color = body.name === 'torso' ? '#b8c8bd' : body.name.endsWith('_L') ? '#b0cdc3' : '#cad0bb';
        if (geom.type === 'capsule') { const ends = G.capsuleEndpoints(bp, bq, geom); shapes.push({depth: point(center).depth, draw: () => capsule(ends[0], ends[1], geom.size[0], color)}); }
        else if (geom.type === 'sphere') shapes.push({depth: point(center).depth, draw: () => sphere(point(center), geom.size[0], color)});
        else if (geom.type === 'box') {
          const corners = [-1, 1].flatMap(x => [-1, 1].flatMap(y => [-1, 1].map(z => G.transform(bp, bq, geom, [x * geom.size[0], y * geom.size[1], z * geom.size[2]]))));
          [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]].forEach((face, faceIndex) => {
            const ps = face.map(k => point(corners[k])); shapes.push({depth: ps.reduce((n, p) => n + p.depth, 0) / 4, draw: () => { ctx.beginPath(); ps.forEach((p, k) => k ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y)); ctx.closePath(); ctx.fillStyle = ['#78988e','#aec6b7','#729187','#c0d2be','#8ba99b','#adc4af'][faceIndex]; ctx.fill(); ctx.strokeStyle = '#425d55'; ctx.lineWidth = .7; ctx.stroke(); }});
          });
        }
      });
    });
    shapes.sort((a, b) => a.depth - b.depth).forEach(s => s.draw());
  }

  function drawHolds(frame) {
    state.demo.geometry.holds.forEach(hold => {
      const p = point(hold.pos), r = Math.max(3, (hold.radius || .04) * camera.scale);
      const attached = state.demo.limbs.findIndex((_, i) => frame.grips?.[i] && gripInfo(frame, i).hold?.name === hold.name);
      ctx.beginPath(); ctx.ellipse(p.x + 2, p.y + 3, r * 1.4, r * 1.2, 0, 0, Math.PI * 2); ctx.fillStyle = '#314a3e16'; ctx.fill();
      if (attached >= 0) { ctx.beginPath(); ctx.arc(p.x, p.y, r + 6, 0, Math.PI * 2); ctx.strokeStyle = rgba(palette[attached], .8); ctx.lineWidth = 1.1; ctx.stroke(); }
      sphere(p, hold.radius || .04, hold.type === 'jug' ? '#dba477' : hold.type === 'sloper' ? '#82aaa5' : hold.type === 'crimp' ? '#b3a4c7' : '#95aa8b');
      ctx.font = '8px ui-monospace, Menlo, monospace'; ctx.fillStyle = '#74856e'; ctx.fillText(hold.name, p.x + r + 7, p.y + 3);
    });
  }

  function drawForces(frame) {
    if (!$('show-forces').checked) return;
    state.demo.limbs.forEach((_, i) => {
      const info = gripInfo(frame, i); if (!info.anchored || !info.force || !frame.anchors?.[i] || !info.load) return;
      const anchor = frame.anchors[i], p = point(anchor), q = point(G.add(anchor, G.scale(info.force, .00055)));
      const dx = q.x - p.x, dy = q.y - p.y, length = Math.hypot(dx, dy); if (length < 2) return;
      const angle = Math.atan2(dy, dx), color = palette[i];
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(q.x, q.y); ctx.stroke();
      ctx.fillStyle = color; ctx.beginPath(); ctx.moveTo(q.x, q.y); ctx.lineTo(q.x - 6 * Math.cos(angle - .45), q.y - 6 * Math.sin(angle - .45)); ctx.lineTo(q.x - 6 * Math.cos(angle + .45), q.y - 6 * Math.sin(angle + .45)); ctx.closePath(); ctx.fill();
      ctx.beginPath(); ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2); ctx.fill();
    });
  }

  function drawCOM(frame) {
    if (!$('show-com').checked || !frame.com) return;
    const p = point(frame.com), wallY = (state.demo.geometry.wall?.pos?.[1] || -.05) + (state.demo.geometry.wall?.size?.[1] || .05);
    line(frame.com, [frame.com[0], wallY, frame.com[2]], '#496d6570', 1, [3, 4]);
    ctx.beginPath(); ctx.arc(p.x, p.y, 6, 0, Math.PI * 2); ctx.fillStyle = '#f6f6e8'; ctx.fill(); ctx.strokeStyle = '#325d51'; ctx.lineWidth = 1.2; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x - 9, p.y); ctx.lineTo(p.x + 9, p.y); ctx.moveTo(p.x, p.y - 9); ctx.lineTo(p.x, p.y + 9); ctx.stroke();
    ctx.fillStyle = '#2d554b'; ctx.font = '8px ui-monospace, Menlo, monospace'; ctx.fillText('COM', p.x + 14, p.y + 3);
  }

  function drawAxes() {
    const origin = {x: width - 52, y: height - 63};
    const basis = G.cameraBasis(camera.azimuth, camera.elevation);
    [[1,0,0,'X','#a66d4f'],[0,1,0,'Y','#7b956b'],[0,0,1,'Z','#5f8589']].forEach(axis => { const vector = axis.slice(0, 3), x = origin.x + G.dot(vector, basis.right) * 23, y = origin.y - G.dot(vector, basis.up) * 23; ctx.strokeStyle = axis[4]; ctx.lineWidth = 1.1; ctx.beginPath(); ctx.moveTo(origin.x, origin.y); ctx.lineTo(x, y); ctx.stroke(); ctx.fillStyle = axis[4]; ctx.font = '7px ui-monospace, Menlo, monospace'; ctx.fillText(axis[3], x + 4, y + 2); });
    ctx.fillStyle = '#71836d'; ctx.beginPath(); ctx.arc(origin.x, origin.y, 2, 0, Math.PI * 2); ctx.fill();
  }

  function renderScene() {
    if (!state.demo) return;
    camera.azimuth = state.azimuth; camera.elevation = state.elevation;
    camera.scale = Math.min((width - 80) / camera.spanX, (height - 94) / camera.spanZ) * state.zoom;
    ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#eeeee5'; ctx.fillRect(0, 0, width, height);
    drawWall(); const frame = state.demo.frames[state.index];
    if (!frame) { drawAxes(); return; }
    // Shadows use the same recorded body centers and radii; they are a visual depth cue only.
    ctx.save(); ctx.globalAlpha = .13;
    state.demo.geometry.bodies.forEach(body => { const i = state.demo.bodyNames.indexOf(body.name); if (i < 0) return; body.geoms.filter(g => g.type === 'capsule').forEach(g => { const ends = G.capsuleEndpoints(frame.pos[i], frame.quat[i], g).map(p => [p[0] + .06, 0, p[2] - .04]); const a = point(ends[0]), b = point(ends[1]); ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.strokeStyle = '#6a7d62'; ctx.lineWidth = g.size[0] * camera.scale * 2.4; ctx.lineCap = 'round'; ctx.stroke(); }); });
    ctx.restore(); ctx.lineCap = 'butt'; drawHolds(frame); drawBody(frame); drawForces(frame); drawCOM(frame); drawAxes();
  }

  function renderPlot() {
    if (!state.demo) return;
    pctx.clearRect(0, 0, plotWidth, plotHeight);
    const left = 25, right = plotWidth - 2, top = 5, bottom = plotHeight - 5, max = Math.max(1.15, state.peak * 1.13), end = duration() || 1;
    const x = t => left + t / end * (right - left), y = n => bottom - n / max * (bottom - top);
    pctx.font = '7px ui-monospace, Menlo, monospace'; pctx.textAlign = 'right';
    [0, .5, 1].forEach(value => { pctx.strokeStyle = value === 1 ? '#718b7466' : '#31484a'; pctx.setLineDash(value === 1 ? [3,3] : []); pctx.beginPath(); pctx.moveTo(left, y(value)); pctx.lineTo(right, y(value)); pctx.lineWidth = .7; pctx.stroke(); pctx.fillStyle = '#789385'; pctx.fillText(`${value * 100}`, left - 5, y(value) + 2); }); pctx.setLineDash([]);
    state.forceSeries.forEach((values, i) => {
      const trace = (endIndex, alpha, lineWidth) => { pctx.beginPath(); let connected = false; for (let j = 0; j <= endIndex; j++) { if (!Number.isFinite(values[j])) { connected = false; continue; } const px = x(state.demo.frames[j].t), py = y(values[j]); if (connected) pctx.lineTo(px, py); else pctx.moveTo(px, py); connected = true; } pctx.strokeStyle = rgba(palette[i], alpha); pctx.lineWidth = lineWidth; pctx.stroke(); };
      trace(values.length - 1, .3, 1); trace(state.index, .95, 1.4);
      if (Number.isFinite(values[state.index])) { pctx.beginPath(); pctx.arc(x(state.time), y(values[state.index]), 2.3, 0, Math.PI * 2); pctx.fillStyle = palette[i]; pctx.fill(); }
    });
    pctx.beginPath(); pctx.moveTo(x(state.time), top); pctx.lineTo(x(state.time), bottom); pctx.strokeStyle = '#ecedda70'; pctx.lineWidth = .7; pctx.stroke(); pctx.textAlign = 'left';
  }

  function setTime(time) { if (!state.demo?.frames.length) return; state.time = Math.max(0, Math.min(duration(), time)); state.index = G.frameIndex(state.demo.frames, state.time); updateUI(); dirty = true; }
  function togglePlayback() { if (!state.demo?.frames.length) return; if (state.time >= duration()) setTime(0); state.playing = !state.playing; state.last = null; updateUI(); }
  function tick(now) {
    if (state.playing && state.demo) {
      if (state.last !== null) setTime(state.time + Math.min((now - state.last) / 1000, .1) * state.speed);
      if (state.time >= duration()) { state.playing = false; updateUI(); }
    }
    state.last = now;
    if (dirty) { renderScene(); renderPlot(); dirty = false; }
    requestAnimationFrame(tick);
  }

  $('play').addEventListener('click', togglePlayback);
  $('restart').addEventListener('click', () => { if (state.demo) { state.playing = false; setTime(0); } });
  $('timeline').addEventListener('input', event => { state.playing = false; if (state.demo) setTime(Number(event.target.value)); });
  $('speed').addEventListener('change', event => { state.speed = Number(event.target.value); });
  $('route-select').addEventListener('change', event => selectDemo(Number(event.target.value)));
  document.querySelectorAll('[data-view]').forEach(button => button.addEventListener('click', () => { if (state.demo) resetCamera(button.dataset.view); }));
  $('reset-view').addEventListener('click', () => { if (state.demo) resetCamera(); });
  ['show-forces', 'show-com'].forEach(id => $(id).addEventListener('change', () => { dirty = true; }));
  canvas.addEventListener('pointerdown', e => { drag = {x: e.clientX, y: e.clientY, azimuth: state.azimuth, elevation: state.elevation}; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener('pointermove', e => { if (!drag) return; state.azimuth = Math.max(-1.48, Math.min(1.48, drag.azimuth + (e.clientX - drag.x) * .006)); state.elevation = Math.max(-.4, Math.min(.7, drag.elevation + (e.clientY - drag.y) * .004)); state.view = 'orbit'; updateCameraButtons(); dirty = true; });
  const endDrag = () => { drag = null; }; canvas.addEventListener('pointerup', endDrag); canvas.addEventListener('pointercancel', endDrag);
  canvas.addEventListener('wheel', e => { e.preventDefault(); state.zoom = Math.max(.6, Math.min(2.3, state.zoom * Math.exp(-e.deltaY * .001))); dirty = true; }, {passive: false});
  canvas.addEventListener('keydown', e => { if (e.code === 'Space') { e.preventDefault(); togglePlayback(); } else if (e.code === 'ArrowRight' || e.code === 'ArrowLeft') { e.preventDefault(); if (state.demo) { state.playing = false; setTime(state.time + (e.code === 'ArrowRight' ? 1 : -1) * .1); } } });
  document.addEventListener('keydown', e => { if (e.code === 'Space' && e.target === document.body) { e.preventDefault(); togglePlayback(); } });
  document.addEventListener('visibilitychange', () => { if (document.hidden && state.playing) { state.playing = false; updateUI(); } });
  $('import-trigger').addEventListener('keydown', e => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); $('import-file').click(); } });
  $('import-file').addEventListener('change', async e => {
    const file = e.target.files?.[0]; if (!file) return;
    try { if (file.size > 50 * 1024 * 1024) throw new Error('Choose a trajectory under 50 MB.'); const demo = normalizeDemo(JSON.parse(await file.text())); state.demos.push(demo); const option = make('option', '', human(demo.route || file.name)); option.value = String(state.demos.length - 1); $('route-select').append(option); $('scene-message').hidden = true; ['play', 'restart', 'timeline'].forEach(id => $(id).disabled = false); selectDemo(state.demos.length - 1); $('import-status').textContent = `Loaded ${file.name}. Playback is local to this browser.`; }
    catch (error) { $('import-status').textContent = error.message; }
    e.target.value = '';
  });
  function resize() { [width, height] = fitCanvas(canvas, ctx); [plotWidth, plotHeight] = fitCanvas(plot, pctx); dirty = true; }
  window.ClimbReplay = {
    loadTrajectory(data) {
      const demo = {...normalizeDemo(data), freshRun: true}; state.demos.push(demo);
      const option = make('option', '', `${human(demo.route)} · edited run`); option.value = String(state.demos.length - 1); $('route-select').append(option);
      selectDemo(state.demos.length - 1); return demo;
    },
    pause() { state.playing = false; updateUI(); },
    resize,
    invalidate() { state.playing = false; $('result-badge').textContent = 'PREVIOUS RUN · ROUTE EDITED'; $('result-badge').classList.add('failed'); }
  };
  new ResizeObserver(resize).observe($('viewport')); new ResizeObserver(resize).observe(plot.parentElement);
  try {
    const source = window.CLIMB_DEMOS || [];
    state.demos = source.map(normalizeDemo);
    if (!state.demos.length) throw new Error('No trajectory data is bundled yet. Generate docs/demo/data.js, or load an exported trajectory JSON from the workflow below.');
    $('route-select').replaceChildren(...state.demos.map((demo, i) => { const option = make('option', '', human(demo.route)); option.value = String(i); return option; }));
    selectDemo(0);
  } catch (error) {
    $('scene-message').hidden = false; $('scene-message').append(make('strong', '', 'Ready for a recorded climb.'), make('p', '', error.message));
    $('route-select').replaceChildren(make('option', '', 'No trajectory loaded'));
    ['play', 'restart', 'timeline'].forEach(id => $(id).disabled = true);
  }
  resize(); requestAnimationFrame(tick);
})();
