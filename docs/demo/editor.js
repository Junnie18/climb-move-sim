(function () {
  'use strict';
  const E = window.ClimbEditorState, $ = id => document.getElementById(id);
  const names = {left_hand: 'Left hand', right_hand: 'Right hand', left_foot: 'Left foot', right_foot: 'Right foot'};
  const initials = {left_hand: 'LH', right_hand: 'RH', left_foot: 'LF', right_foot: 'RF'};
  const colors = {left_hand: '#397f76', right_hand: '#b36b43', left_foot: '#86709d', right_foot: '#8c873b'};
  const holdColors = {jug: '#c58c62', crimp: '#aa8dba', sloper: '#71a49c', foothold: '#96a377'};
  const canvas = $('planner'), ctx = canvas.getContext('2d');
  const templates = (window.CLIMB_DEMOS || []).flatMap(d => { try { return [{name: d.route, route: E.fromDemo(d)}]; } catch { return []; } });
  if (!templates.length) { $('editor-status').textContent = 'No editable route templates were found. Load the bundled demo data to use the editor.'; $('run-simulation').disabled = true; return; }
  const store = E.create(templates[0].route);
  let mode = 'start', tool = 'holds', selectedLimb = 'left_hand', selectedHold = store.route.holds[0].name;
  let width = 800, height = 600, drag = null, busy = false, runId = 0, zoom = 1, importedRoute = null;
  const camera = {x: 0, z: 1.3, scale: 360};
  const hostedDefault = location.hostname === 'junnie18.github.io' && /^\/climb-move-sim(?:\/|$)/.test(location.pathname) ? 'https://ascent-climbing-lab.vercel.app' : '';
  const defaultSimulatorUrl = window.CLIMB_SIMULATOR_URL || hostedDefault;
  let simulatorUrl = defaultSimulatorUrl;
  try { simulatorUrl = localStorage.getItem('ascent.simulatorUrl') || defaultSimulatorUrl; } catch { /* Private browsing still uses the configured default. */ }
  $('simulator-url').value = simulatorUrl;
  const make = (tag, className, text) => { const n = document.createElement(tag); if (className) n.className = className; if (text !== undefined) n.textContent = text; return n; };
  const project = pos => ({x: width / 2 + (pos[0] - camera.x) * camera.scale * zoom, y: height / 2 - (pos[2] - camera.z) * camera.scale * zoom});
  const unproject = point => [camera.x + (point.x - width / 2) / (camera.scale * zoom), .05, camera.z - (point.y - height / 2) / (camera.scale * zoom)];
  const mouse = event => { const r = canvas.getBoundingClientRect(); return {x: event.clientX - r.left, y: event.clientY - r.top}; };
  const clamp = (n, low, high) => Math.max(low, Math.min(high, n));
  const currentRoute = () => drag?.route || store.route;
  const holdsByName = route => Object.fromEntries(route.holds.map(h => [h.name, h]));
  function endpointMap(route) { const holds = holdsByName(route); return Object.fromEntries(Object.entries(E.assignments(route, mode === 'moves')).map(([limb, hold]) => [limb, holds[hold].pos])); }
  function handlePoint(limb, position) { const p = project(position); return {x: p.x + (limb.startsWith('left') ? -31 : 31), y: p.y + (limb.endsWith('foot') ? 15 : 0)}; }
  function setStatus(title, text, kind = 'draft') {
    $('editor-verdict-title').textContent = title; $('editor-status').textContent = text;
    $('editor-verdict').className = `editor-verdict ${kind}`;
  }
  function invalidate(message = 'Your route changed. Run a new simulation to test this version.') {
    $('view-result').hidden = true; window.ClimbReplay?.invalidate();
    setStatus(busy ? 'Route changed during simulation' : 'Ready to test your route', busy ? 'The running result will be discarded. Run again when it finishes to test these edits.' : message);
  }
  function commit(route, message) {
    try { if (store.commit(route)) invalidate(message); renderControls(); draw(); return true; }
    catch (error) { setStatus('Check this edit', error.message, 'error'); renderControls(); draw(); return false; }
  }
  function edit(mutate, message) { const route = store.route; mutate(route); return commit(route, message); }
  function showTab(tab) {
    const editing = tab === 'editor'; $('editor-stage').hidden = !editing; $('replay-stage').hidden = editing;
    ['editor', 'replay'].forEach(name => { $(name + '-tab').classList.toggle('active', name === tab); $(name + '-tab').setAttribute('aria-pressed', String(name === tab)); });
    if (editing) { window.ClimbReplay?.pause(); resize(); } else requestAnimationFrame(() => window.ClimbReplay?.resize());
  }
  function setMode(next) { mode = next; tool = 'limbs'; renderControls(); draw(); }
  function fit() {
    const positions = store.route.holds.map(h => h.pos), xs = positions.map(p => p[0]), zs = positions.map(p => p[2]);
    const minX = Math.min(...xs) - .35, maxX = Math.max(...xs) + .35, minZ = Math.min(...zs) - .22, maxZ = Math.max(...zs) + .32;
    camera.x = (minX + maxX) / 2; camera.z = (minZ + maxZ) / 2;
    camera.scale = Math.min((width - 100) / Math.max(maxX - minX, 1.2), (height - 110) / Math.max(maxZ - minZ, 1.3)); zoom = 1; draw();
  }
  function selectOptions(select, options, value) {
    select.replaceChildren(...options.map(({value, label}) => { const n = make('option', '', label); n.value = value; return n; })); select.value = value;
  }
  function renderControls() {
    const route = store.route, holds = holdsByName(route), assigned = E.assignments(route, mode === 'moves');
    if (!holds[selectedHold]) selectedHold = route.holds[0].name;
    $('editor-name').value = route.name; $('editor-undo').disabled = !store.canUndo;
    $('edit-revision').textContent = `DRAFT ${String(store.revision + 1).padStart(2, '0')}`;
    ['holds', 'limbs'].forEach(t => { $('tool-' + t).classList.toggle('active', tool === t); $('tool-' + t).setAttribute('aria-pressed', String(tool === t)); });
    ['start', 'moves'].forEach(m => { $('mode-' + m).classList.toggle('active', mode === m); $('mode-' + m).setAttribute('aria-pressed', String(mode === m)); });
    $('assignment-help').textContent = mode === 'start' ? 'Drag a limb handle onto a hold to set its starting contact.' : 'The figure shows the end of your sequence. Drag a limb onto a new hold to append a move.';
    $('planner-context').textContent = tool === 'holds' ? 'Drag a hold to reshape the route.' : `${mode === 'start' ? 'Set the start' : 'Add the next move'}: drag a limb, or select one and click a hold.`;
    $('assign-limb').textContent = mode === 'start' ? 'Set contact' : 'Add move';
    $('limb-controls').replaceChildren(...E.LIMBS.map(limb => {
      const b = make('button', `limb-control${selectedLimb === limb ? ' selected' : ''}`); b.type = 'button'; b.style.setProperty('--limb', colors[limb]); b.setAttribute('aria-pressed', String(selectedLimb === limb));
      b.append(make('span', 'limb-initial', initials[limb]), make('span', '', names[limb]), make('small', '', assigned[limb]));
      b.addEventListener('click', () => { selectedLimb = limb; tool = 'limbs'; renderControls(); draw(); }); return b;
    }));
    const options = route.holds.map(h => ({value: h.name, label: h.name}));
    selectOptions($('assignment-target'), options, selectedHold); selectOptions($('selected-hold'), options, selectedHold);
    const hold = holds[selectedHold], defaults = E.DEFAULTS[hold.type];
    $('hold-x').value = Number(hold.pos[0].toFixed(3)); $('hold-z').value = Number(hold.pos[2].toFixed(3)); $('hold-type').value = hold.type;
    $('hold-cap').value = hold.max_grip_n ?? defaults.max_grip_n; $('hold-friction').value = hold.friction ?? defaults.friction;
    const inStart = Object.values(route.start).includes(selectedHold), inMoves = route.moves.some(m => m.hold === selectedHold);
    $('delete-hold').disabled = inStart || inMoves || route.holds.length <= 4;
    $('delete-hold').title = inStart || inMoves ? 'Reassign contacts and remove moves using this hold before deleting it.' : 'Remove this hold';
    $('add-hold').disabled = route.holds.length >= 40;
    $('move-count').textContent = route.moves.length; $('clear-moves').disabled = !route.moves.length; $('empty-moves').hidden = Boolean(route.moves.length);
    $('move-list').replaceChildren(...route.moves.map((move, index) => {
      const li = make('li', 'planned-move'), label = make('span', 'planned-move-label'); li.style.setProperty('--limb', colors[move.limb]);
      label.append(make('strong', '', names[move.limb]), make('small', '', move.hold));
      const controls = make('span', 'move-actions');
      [['↑', 'Move earlier', -1], ['↓', 'Move later', 1], ['×', 'Remove move', 0]].forEach(([text, action, direction]) => {
        const b = make('button', '', text); b.setAttribute('aria-label', `${action}: ${names[move.limb]} to ${move.hold}`); b.title = action;
        b.disabled = direction < 0 && index === 0 || direction > 0 && index === route.moves.length - 1;
        b.addEventListener('click', () => edit(r => { if (!direction) r.moves.splice(index, 1); else [r.moves[index], r.moves[index + direction]] = [r.moves[index + direction], r.moves[index]]; })); controls.append(b);
      });
      li.append(make('span', 'move-number', String(index + 1).padStart(2, '0')), label, controls); return li;
    }));
  }
  function assign(limb, holdName) {
    const route = store.route; selectedLimb = limb; selectedHold = holdName;
    const assigned = E.assignments(route, mode === 'moves');
    if (assigned[limb] === holdName) { renderControls(); draw(); return; }
    if (mode === 'start') route.start[limb] = holdName;
    else { if (route.moves.length >= 12) { setStatus('Sequence is full', 'Use at most 12 moves in one interactive simulation.', 'error'); return; } route.moves.push({limb, hold: holdName}); }
    commit(route, mode === 'start' ? 'Starting contact updated. Run the simulation to check this pose.' : `Move ${route.moves.length} added. Run the simulation to test the complete sequence.`);
  }
  function line(a, b, color, lineWidth = 1, dash = []) {
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.strokeStyle = color; ctx.lineWidth = lineWidth; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
  }
  function circle(point, radius, fill, stroke, lineWidth = 1) { ctx.beginPath(); ctx.arc(point.x, point.y, radius, 0, Math.PI * 2); if (fill) { ctx.fillStyle = fill; ctx.fill(); } if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lineWidth; ctx.stroke(); } }
  function draw() {
    if (!width || !height) return;
    const route = currentRoute(), endpoints = endpointMap(route), holds = holdsByName(route), assigned = E.assignments(route, mode === 'moves');
    ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#edeee5'; ctx.fillRect(0, 0, width, height);
    const s = camera.scale * zoom, min = unproject({x: 0, y: height}), max = unproject({x: width, y: 0});
    const grid = s < 120 ? .5 : .25;
    for (let x = Math.floor(min[0] / grid) * grid; x <= max[0]; x += grid) { const p = project([x, 0, 0]); line({x: p.x, y: 0}, {x: p.x, y: height}, Math.abs(x % 1) < .001 ? '#cfd6c8' : '#dce1d5', .7); }
    for (let z = Math.floor(min[2] / grid) * grid; z <= max[2]; z += grid) { const p = project([0, 0, z]); line({x: 0, y: p.y}, {x: width, y: p.y}, Math.abs(z % 1) < .001 ? '#cfd6c8' : '#dce1d5', .7); if (p.y > 110 && p.y < height - 45) { ctx.font = '9px ui-monospace, monospace'; ctx.fillStyle = '#8c9988'; ctx.fillText(`${z.toFixed(2)} m`, 18, p.y - 5); } }
    $('planner-scale').textContent = `${grid} m GRID`;
    // This deliberately schematic figure depicts contacts, not inverse kinematics.
    const handX = (endpoints.left_hand[0] + endpoints.right_hand[0]) / 2, footX = (endpoints.left_foot[0] + endpoints.right_foot[0]) / 2;
    const handZ = (endpoints.left_hand[2] + endpoints.right_hand[2]) / 2, footZ = (endpoints.left_foot[2] + endpoints.right_foot[2]) / 2;
    const hip = [(handX + footX) / 2, .05, footZ + .45], shoulder = [handX * .6 + footX * .4, .05, Math.max(footZ + .72, handZ - .22)];
    const hipP = project(hip), shoulderP = project(shoulder);
    line(hipP, shoulderP, '#c3cdbf', clamp(s * .14, 16, 42)); line(hipP, shoulderP, '#859b8c', 1, [4, 4]);
    circle(project([shoulder[0], .05, shoulder[2] + .16]), clamp(s * .075, 12, 24), '#d4dccd', '#99aa97');
    E.LIMBS.forEach(limb => {
      const side = limb.startsWith('left') ? -1 : 1, upper = limb.endsWith('hand'), base = upper ? shoulder : hip;
      const origin = project([base[0] + side * (upper ? .13 : .09), .05, base[2]]), end = project(endpoints[limb]);
      const mid = {x: (origin.x + end.x) / 2 + side * Math.min(22, s * .055), y: (origin.y + end.y) / 2 + (upper ? 22 : -5)};
      line(origin, mid, '#a9bbaa', clamp(s * .04, 5, 12)); line(mid, end, '#a9bbaa', clamp(s * .035, 4, 10)); circle(mid, 3, '#f0f1e8', '#8da08d');
    });
    // The planned move path connects successive hold locations for each limb.
    const previous = {...route.start}; route.moves.forEach((move, index) => {
      const a = project(holds[previous[move.limb]].pos), b = project(holds[move.hold].pos); previous[move.limb] = move.hold;
      line(a, b, colors[move.limb] + '77', 1.5, [3, 4]);
      const offset = move.limb.startsWith('left') ? -1 : 1;
      ctx.fillStyle = colors[move.limb]; ctx.font = '8px ui-monospace, monospace'; ctx.textAlign = offset < 0 ? 'right' : 'left'; ctx.fillText(String(index + 1), b.x + offset * 14, b.y - 9);
    }); ctx.textAlign = 'left';
    route.holds.forEach(hold => {
      const p = project(hold.pos), selected = hold.name === selectedHold, attached = E.LIMBS.filter(l => assigned[l] === hold.name), radius = clamp((hold.radius ?? E.DEFAULTS[hold.type].radius) * s, 5, 24);
      if (selected) circle(p, Math.max(13, radius + 4), '#ffffff55', '#365b50', 1.5);
      if (attached.length) circle(p, 11, null, colors[attached[0]] + '88', 1.5);
      circle({x: p.x + 1, y: p.y + 2}, 7, '#344a3d20');
      circle(p, hold.type === 'foothold' ? 5.5 : 7, holdColors[hold.type], '#69806b', .8);
      circle({x: p.x - 1.5, y: p.y - 1.5}, 1.5, '#f8f5e9aa');
      if (selected) { ctx.fillStyle = '#345748'; ctx.font = '10px ui-monospace, monospace'; const left = p.x > width * .7; ctx.textAlign = left ? 'right' : 'left'; ctx.fillText(hold.name, p.x + (left ? -19 : 19), p.y - 11); ctx.textAlign = 'left'; }
    });
    E.LIMBS.forEach(limb => {
      const p = project(endpoints[limb]), h = handlePoint(limb, endpoints[limb]), selected = selectedLimb === limb && tool === 'limbs';
      line(p, h, colors[limb] + '99', 1, [2, 3]); circle(h, selected ? 16 : 14, selected ? colors[limb] : '#f6f5eb', colors[limb], selected ? 2 : 1.4);
      ctx.fillStyle = selected ? '#fffdf1' : colors[limb]; ctx.font = 'bold 9px ui-monospace, monospace'; ctx.textAlign = 'center'; ctx.fillText(initials[limb], h.x, h.y + 3); ctx.textAlign = 'left';
    });
    if (drag?.kind === 'limb') {
      const start = handlePoint(drag.limb, endpoints[drag.limb]), target = nearestHold(drag.point, route, 25);
      line(start, drag.point, colors[drag.limb], 1.5, [5, 5]); circle(drag.point, 17, colors[drag.limb] + '22', colors[drag.limb], 1.5);
      if (target) circle(project(target.pos), 20, '#ffffff44', colors[drag.limb], 2);
    }
  }
  function nearestHold(point, route, distance = 16) {
    return route.holds.map(h => ({hold: h, distance: Math.hypot(project(h.pos).x - point.x, project(h.pos).y - point.y)})).filter(p => p.distance <= distance).sort((a, b) => a.distance - b.distance)[0]?.hold;
  }
  function resize() {
    const rect = canvas.getBoundingClientRect(); if (!rect.width || !rect.height) return;
    width = rect.width; height = rect.height; const ratio = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); draw();
  }
  canvas.addEventListener('pointerdown', event => {
    if (event.button !== 0) return; const point = mouse(event), route = store.route, endpoints = endpointMap(route);
    canvas.setPointerCapture(event.pointerId);
    if (tool === 'limbs') {
      const limb = E.LIMBS.find(l => { const h = handlePoint(l, endpoints[l]); return Math.hypot(h.x - point.x, h.y - point.y) < 19; });
      if (limb) { selectedLimb = limb; drag = {kind: 'limb', limb, point}; renderControls(); draw(); return; }
      const hold = nearestHold(point, route); if (hold) { drag = {kind: 'assign', hold: hold.name, point}; return; }
    } else {
      const hold = nearestHold(point, route); if (hold) { selectedHold = hold.name; drag = {kind: 'hold', hold: hold.name, route, point}; renderControls(); draw(); return; }
    }
    drag = {kind: 'pan', point, x: camera.x, z: camera.z};
  });
  canvas.addEventListener('pointermove', event => {
    const point = mouse(event);
    if (!drag) { canvas.style.cursor = nearestHold(point, store.route) ? (tool === 'holds' ? 'grab' : 'pointer') : 'default'; return; }
    if (drag.kind === 'hold') {
      const hold = drag.route.holds.find(h => h.name === drag.hold), position = unproject(point), snap = $('snap-grid').checked ? .01 : .001;
      hold.pos[0] = clamp(Math.round(position[0] / snap) * snap, -3, 3); hold.pos[2] = clamp(Math.round(position[2] / snap) * snap, .05, 5);
      $('hold-x').value = hold.pos[0].toFixed(3); $('hold-z').value = hold.pos[2].toFixed(3);
    } else if (drag.kind === 'limb') drag.point = point;
    else if (drag.kind === 'pan') { camera.x = drag.x - (point.x - drag.point.x) / (camera.scale * zoom); camera.z = drag.z + (point.y - drag.point.y) / (camera.scale * zoom); }
    draw();
  });
  canvas.addEventListener('pointerup', event => {
    if (!drag) return; const ended = drag; drag = null;
    if (ended.kind === 'hold') commit(ended.route);
    else if (ended.kind === 'assign') assign(selectedLimb, ended.hold);
    else if (ended.kind === 'limb') { const target = nearestHold(mouse(event), store.route, 25); if (target) assign(ended.limb, target.name); else setStatus('Choose a hold', 'Drop the limb on a hold. Its previous assignment has been kept.'); }
    draw();
  });
  canvas.addEventListener('pointercancel', () => { drag = null; renderControls(); draw(); });
  canvas.addEventListener('lostpointercapture', () => { if (drag) { drag = null; renderControls(); draw(); } });
  canvas.addEventListener('wheel', event => { event.preventDefault(); const p = mouse(event), before = unproject(p); zoom = clamp(zoom * Math.exp(-event.deltaY * .0015), .4, 4); const after = unproject(p); camera.x += before[0] - after[0]; camera.z += before[2] - after[2]; draw(); }, {passive: false});
  canvas.addEventListener('keydown', event => {
    if (event.key === 'Escape') { drag = null; renderControls(); draw(); }
    if (tool === 'holds' && ['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown'].includes(event.key)) {
      event.preventDefault(); const amount = event.shiftKey ? .05 : .01;
      edit(r => { const h = r.holds.find(h => h.name === selectedHold), axis = /Left|Right/.test(event.key) ? 0 : 2; h.pos[axis] = Number((h.pos[axis] + (/Right|Up/.test(event.key) ? amount : -amount)).toFixed(3)); });
    }
  });
  $('editor-tab').addEventListener('click', () => showTab('editor')); $('replay-tab').addEventListener('click', () => showTab('replay'));
  $('tool-holds').addEventListener('click', () => { tool = 'holds'; renderControls(); draw(); }); $('tool-limbs').addEventListener('click', () => { tool = 'limbs'; renderControls(); draw(); });
  $('mode-start').addEventListener('click', () => setMode('start')); $('mode-moves').addEventListener('click', () => setMode('moves'));
  $('planner-fit').addEventListener('click', fit);
  $('editor-template').replaceChildren(...templates.map((t, i) => { const n = make('option', '', t.name); n.value = i; return n; }));
  $('editor-template').addEventListener('change', e => { store.replace(e.target.value === 'imported' ? importedRoute : templates[Number(e.target.value)].route); selectedHold = store.route.holds[0].name; invalidate(); renderControls(); fit(); });
  $('editor-name').addEventListener('change', e => edit(r => { r.name = e.target.value.trim(); }));
  $('editor-undo').addEventListener('click', () => { store.undo(); invalidate(); renderControls(); draw(); });
  $('editor-reset').addEventListener('click', () => { store.reset(); invalidate('Template restored. Run a fresh simulation, or continue editing.'); renderControls(); fit(); });
  $('clear-moves').addEventListener('click', () => edit(r => { r.moves = []; }));
  $('selected-hold').addEventListener('change', e => { selectedHold = e.target.value; renderControls(); draw(); });
  $('assignment-target').addEventListener('change', e => { selectedHold = e.target.value; renderControls(); draw(); });
  $('assign-limb').addEventListener('click', () => assign(selectedLimb, $('assignment-target').value));
  [['hold-x', 0], ['hold-z', 2]].forEach(([id, axis]) => $(id).addEventListener('change', e => edit(r => { r.holds.find(h => h.name === selectedHold).pos[axis] = e.target.value === '' ? NaN : Number(e.target.value); })));
  $('hold-type').addEventListener('change', e => edit(r => { const h = r.holds.find(h => h.name === selectedHold); h.type = e.target.value; Object.assign(h, E.clone(E.DEFAULTS[h.type])); }));
  [['hold-cap', 'max_grip_n'], ['hold-friction', 'friction']].forEach(([id, key]) => $(id).addEventListener('change', e => edit(r => { r.holds.find(h => h.name === selectedHold)[key] = Number(e.target.value); })));
  $('delete-hold').addEventListener('click', () => edit(r => { r.holds = r.holds.filter(h => h.name !== selectedHold); }));
  $('add-hold').addEventListener('click', () => {
    const route = store.route; let count = route.holds.length + 1, name = `hold_${count}`; while (route.holds.some(h => h.name === name)) name = `hold_${++count}`;
    route.holds.push({name, type: 'jug', pos: [Number(clamp(camera.x + .35, -3, 3).toFixed(2)), .05, Number(clamp(camera.z + .35, .05, 5).toFixed(2))]}); selectedHold = name; tool = 'holds'; commit(route);
  });
  $('editor-export').addEventListener('click', () => {
    const route = store.route, blob = new Blob([JSON.stringify(route, null, 2) + '\n'], {type: 'application/json'}), url = URL.createObjectURL(blob), a = document.createElement('a');
    a.href = url; a.download = (route.name.replace(/[^a-z0-9_-]+/gi, '-').replace(/^-|-$/g, '') || 'climbing-route') + '.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  $('editor-import').addEventListener('click', () => $('editor-import-file').click());
  $('editor-import-file').addEventListener('change', async event => {
    const file = event.target.files?.[0]; if (!file) return;
    try {
      if (file.size > 128 * 1024) throw new Error('Choose a route JSON under 128 KiB. Trajectory files belong in the replay workflow.');
      const route = E.validate(JSON.parse(await file.text())); store.replace(route); importedRoute = E.clone(route);
      selectedHold = route.holds[0].name; selectedLimb = 'left_hand'; mode = 'start'; tool = 'holds';
      const existing = $('editor-template').querySelector('[data-imported]'); if (existing) existing.remove();
      const option = make('option', '', `${route.name} · imported`); option.value = 'imported'; option.dataset.imported = 'true'; $('editor-template').append(option); $('editor-template').value = 'imported';
      invalidate(`Loaded ${file.name}. Run a fresh simulation to test this route.`); renderControls(); fit();
    } catch (error) { setStatus('Could not import this route', error instanceof SyntaxError ? 'The file is not valid JSON. Export a route from this editor, then try importing it.' : error.message, 'error'); }
    finally { event.target.value = ''; }
  });
  function endpoint(path) {
    const input = simulatorUrl.trim().replace(/\/$/, '');
    if (input) { const url = new URL(input); if (!['http:', 'https:'].includes(url.protocol)) throw new Error('Use an http or https simulator URL.'); if (url.username || url.password || url.search || url.hash) throw new Error('Use the simulator base URL without credentials, query parameters or a fragment.'); return input + path; }
    if (location.protocol === 'file:') throw new Error('Open the local app at http://127.0.0.1:8765, or enter a hosted simulator URL in Simulator connection.');
    return path;
  }
  async function checkConnection() {
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 5000);
    $('connection-status').textContent = 'Checking simulator…';
    try {
      const response = await fetch(endpoint('/api/health'), {signal: controller.signal});
      if (!response.ok) throw new Error('No simulator is responding at this address.');
      const data = await response.json(); if (data?.status !== 'ok' || !/mujoco/i.test(data?.engine || '')) throw new Error('This address did not return a MuJoCo simulator health check.');
      $('connection-label').textContent = 'Simulator connected'; $('connection-status').textContent = 'Connected. Edited routes run on the Python simulator.';
    } catch { $('connection-label').textContent = 'Connect simulator'; $('connection-status').textContent = 'Editing works here. To calculate fresh results, open the local Python app or enter its hosted URL.'; }
    finally { clearTimeout(timer); }
  }
  $('save-connection').addEventListener('click', () => {
    simulatorUrl = $('simulator-url').value.trim(); try { endpoint('/api/health'); try { localStorage.setItem('ascent.simulatorUrl', simulatorUrl); } catch {} checkConnection(); }
    catch (error) { $('connection-status').textContent = error.message; }
  });
  $('run-simulation').addEventListener('click', async () => {
    if (busy) return;
    const token = store.beginRun(), request = ++runId, controller = new AbortController(); let timer;
    try {
      E.validate(token.route); const url = endpoint('/api/simulate'); busy = true; $('run-simulation').disabled = true; $('run-simulation').querySelector('span').textContent = 'Simulating…'; $('view-result').hidden = true;
      setStatus('Running your route in MuJoCo', 'Solving the starting pose, then checking each transition, support force and grip constraint. This can take a minute.', 'running');
      timer = setTimeout(() => controller.abort(), 180000);
      const response = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({route: token.route, frameEvery: 20}), signal: controller.signal});
      let data; try { data = await response.json(); } catch { throw new Error('No simulator API is available here. Open Simulator connection to connect the local app or a hosted server.'); }
      if (!response.ok) throw new Error(data.error || (response.status === 429 ? 'The simulator is busy. Try again when the current run finishes.' : 'The simulator could not run this route.'));
      if (!data.trajectory || typeof data.trajectory.success !== 'boolean') throw new Error('The simulator returned no valid trajectory.');
      if (request !== runId || token.revision !== store.revision) { setStatus('Edits kept, older result discarded', 'The route changed while the simulation was running. Run again to test the current version.'); return; }
      window.ClimbReplay.loadTrajectory(data.trajectory);
      store.accept(token, data.trajectory);
      const failure = data.trajectory.outcomes?.find(o => !o.passed);
      setStatus(data.trajectory.success ? 'Model/controller completed' : 'This attempt did not complete', data.trajectory.success ? 'Your edited sequence passed the configured checks. Inspect the motion, grip loads and wall-clearance limitations in the replay.' : (failure?.reason || 'Inspect the execution trace to see where the controller stopped.'), data.trajectory.success ? 'passed' : 'failed');
      $('view-result').hidden = false; showTab('replay'); $('replay-stage').scrollIntoView({block: 'start'});
    } catch (error) {
      if (request !== runId || token.revision !== store.revision) { setStatus('Edits kept, older result discarded', 'The previous run ended after the route changed. Run again to test the current version.'); return; }
      setStatus('Simulation unavailable', error.name === 'AbortError' ? 'The request timed out. The server may still be finishing its run. Try again shortly.' : error.message === 'Failed to fetch' || error instanceof TypeError ? 'Could not reach the Python simulator. Check Simulator connection, then run again. Your edits are saved in this tab.' : error.message, 'error');
      if (error instanceof TypeError) $('connection-label').textContent = 'Connect simulator';
    } finally { clearTimeout(timer); busy = false; $('run-simulation').disabled = false; $('run-simulation').querySelector('span').textContent = 'Run simulation'; }
  });
  $('view-result').addEventListener('click', () => showTab('replay'));
  new ResizeObserver(resize).observe(canvas.parentElement);
  renderControls(); resize(); fit(); checkConnection();
})();
