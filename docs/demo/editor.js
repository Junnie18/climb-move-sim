(function () {
  'use strict';
  const E = window.ClimbEditorState, $ = id => document.getElementById(id);
  const names = {left_hand: 'Left hand', right_hand: 'Right hand', left_foot: 'Left foot', right_foot: 'Right foot'};
  const initials = {left_hand: 'LH', right_hand: 'RH', left_foot: 'LF', right_foot: 'RF'};
  const colors = {left_hand: '#397f76', right_hand: '#b36b43', left_foot: '#86709d', right_foot: '#8c873b'};
  const holdColors = {jug: '#c58c62', crimp: '#aa8dba', sloper: '#71a49c', foothold: '#96a377'};
  const canvas = $('planner'), ctx = canvas.getContext('2d');
  const templates = (window.CLIMB_DEMOS || []).flatMap(d => { try { return [{name: d.route, route: E.fromDemo(d)}]; } catch { return []; } });
  if (!templates.length) { $('editor-status').textContent = 'No editable route templates were found. Load the bundled demo data to use the editor.'; $('run-simulation').disabled = true; $('test-pose').disabled = true; return; }
  const starter = E.starter(templates[0].route); templates.unshift({name: 'Blank wall · four starting contacts', route: starter});
  const store = E.create(starter);
  let mode = 'moves', tool = 'limbs', selectedLimb = 'left_hand', selectedHold = 'target_left';
  let width = 800, height = 600, drag = null, busy = false, runId = 0, zoom = 1, importedRoute = null;
  let poseScopeRevision = 0, activeCheckType = null;
  let preview = null, previewRevision = -1, previewTimer, previewController;
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
  function holdLabel(route, name) {
    const starting = E.LIMBS.find(limb => route.start[limb] === name);
    if (starting) return names[starting] + ' start';
    return 'Hold ' + (route.holds.filter(h => !Object.values(route.start).includes(h.name)).findIndex(h => h.name === name) + 1);
  }
  function endpointMap(route) { const holds = holdsByName(route); return Object.fromEntries(Object.entries(E.assignments(route, mode === 'moves')).map(([limb, hold]) => [limb, holds[hold].pos])); }
  function handlePoint(limb, position) { const p = project(position); return {x: p.x + (limb.startsWith('left') ? -31 : 31), y: p.y + (limb.endsWith('foot') ? 15 : 0)}; }
  function setStatus(title, text, kind = 'draft') {
    $('editor-verdict-title').textContent = title; $('editor-status').textContent = text;
    $('editor-verdict').className = `editor-verdict ${kind}`;
  }
  function invalidate(message = 'Your route changed. Run a new simulation to test this version.') {
    $('view-result').hidden = true; $('replay-tab').disabled = true; window.ClimbReplay?.invalidate();
    setStatus(busy ? 'Route changed during simulation' : 'Ready to test your route', busy ? 'The running result will be discarded. Run again when it finishes to test these edits.' : message);
  }
  function commit(route, message) {
    try { if (store.commit(route)) { invalidate(message); schedulePreview(); } renderControls(); draw(); return true; }
    catch (error) { setStatus('Check this edit', error.message, 'error'); renderControls(); draw(); return false; }
  }
  function edit(mutate, message) { const route = store.route; mutate(route); return commit(route, message); }
  function showTab(tab) {
    const editing = tab === 'editor'; $('editor-stage').hidden = !editing; $('replay-stage').hidden = editing;
    ['editor', 'replay'].forEach(name => { $(name + '-tab').classList.toggle('active', name === tab); $(name + '-tab').setAttribute('aria-pressed', String(name === tab)); });
    if (editing) { window.ClimbReplay?.pause(); resize(); } else window.ClimbReplay?.resize();
  }
  function setMode(next) {
    if (next !== mode) {
      poseScopeRevision++;
      if (activeCheckType === 'pose' || !busy && store.result?.checkType === 'pose') {
        $('view-result').hidden = true; $('replay-tab').disabled = true; window.ClimbReplay?.invalidate();
        setStatus('Displayed pose changed', busy ? 'The running pose result will be discarded. Test again when it finishes.' : 'Test this pose again to check these contacts. Your planned moves are unchanged.');
      }
    }
    mode = next; tool = 'limbs'; renderControls(); draw(); schedulePreview();
  }
  function fit() {
    const positions = store.route.holds.map(h => h.pos), xs = positions.map(p => p[0]), zs = positions.map(p => p[2]);
    const minX = Math.min(...xs) - .5, maxX = Math.max(...xs) + .5, minZ = Math.min(...zs) - .25, maxZ = Math.max(...zs) + .4;
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
    $('assignment-help').textContent = mode === 'start' ? 'Editing starting contacts. Switch back to Add moves before planning your next reach.' : 'Drag a labeled hand or foot onto a hold. Each drop adds one move.';
    $('planner-context').textContent = mode === 'start' ? 'Editing the starting pose' : route.moves.length ? `${route.moves.length} move${route.moves.length === 1 ? '' : 's'} planned · Drag another limb to continue` : 'Try dragging the left hand to hold 1';
    $('run-simulation').hidden = !route.moves.length;
    if (!busy) { $('run-simulation').querySelector('span').textContent = `Test ${route.moves.length} move${route.moves.length === 1 ? '' : 's'}`; $('test-pose').querySelector('span').textContent = 'Test this pose'; }
    $('assign-limb').textContent = mode === 'start' ? 'Set contact' : 'Add move';
    $('limb-controls').replaceChildren(...E.LIMBS.map(limb => {
      const b = make('button', `limb-control${selectedLimb === limb ? ' selected' : ''}`); b.type = 'button'; b.style.setProperty('--limb', colors[limb]); b.setAttribute('aria-pressed', String(selectedLimb === limb));
      b.append(make('span', 'limb-initial', initials[limb]), make('span', '', names[limb]), make('small', '', holdLabel(route, assigned[limb])));
      b.addEventListener('click', () => { selectedLimb = limb; tool = 'limbs'; renderControls(); draw(); }); return b;
    }));
    const options = route.holds.map(h => ({value: h.name, label: holdLabel(route, h.name)}));
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
      label.append(make('strong', '', names[move.limb] + ' to ' + holdLabel(route, move.hold).toLowerCase()), make('small', '', `${Math.round(E.moveDistance(route, index) * 100)} cm from the previous contact`));
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
  function drawPreview(scale) {
    if (!preview) return;
    const G = window.ClimbGeometry; ctx.globalAlpha = previewRevision === store.revision && !drag ? 1 : .35;
    preview.geometry.bodies.forEach(body => {
      const i = preview.bodyNames.indexOf(body.name); if (i < 0) return;
      const color = body.name === 'torso' ? '#809e91' : body.name.endsWith('_L') ? '#83b0a1' : '#b5aa87';
      body.geoms.forEach(geom => {
        const center = project(G.transform(preview.pos[i], preview.quat[i], geom));
        if (geom.type === 'capsule') {
          const ends = G.capsuleEndpoints(preview.pos[i], preview.quat[i], geom).map(project);
          ctx.lineCap = 'round'; line(ends[0], ends[1], '#405e51', Math.max(4, geom.size[0] * scale * 2 + 1)); line(ends[0], ends[1], color, Math.max(3, geom.size[0] * scale * 2 - 1)); ctx.lineCap = 'butt';
        } else if (geom.type === 'sphere') circle(center, Math.max(3, geom.size[0] * scale), color, '#496454');
        else if (geom.type === 'box') {
          const points = [[-1,0,-1],[1,0,-1],[1,0,1],[-1,0,1]].map(p => project(G.transform(preview.pos[i], preview.quat[i], geom, p.map((v,k) => v * geom.size[k]))));
          ctx.beginPath(); points.forEach((p,k) => k ? ctx.lineTo(p.x,p.y) : ctx.moveTo(p.x,p.y)); ctx.closePath(); ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = '#496454'; ctx.stroke();
        }
      });
    });
    ctx.globalAlpha = 1;
    if (previewRevision === store.revision) preview.limbs.forEach((limb, i) => { if (preview.residuals[i] > .0001 && preview.targets[i]) line(project(preview.sites[i]), project(preview.targets[i]), '#b45e39', 2, [4, 4]); });
  }
  function draw() {
    if (!width || !height) return;
    const route = currentRoute(), endpoints = endpointMap(route), holds = holdsByName(route), assigned = E.assignments(route, mode === 'moves');
    ctx.clearRect(0, 0, width, height); ctx.fillStyle = '#edeee5'; ctx.fillRect(0, 0, width, height);
    const s = camera.scale * zoom, min = unproject({x: 0, y: height}), max = unproject({x: width, y: 0});
    const grid = s < 120 ? .5 : .25;
    for (let x = Math.floor(min[0] / grid) * grid; x <= max[0]; x += grid) { const p = project([x, 0, 0]); line({x: p.x, y: 0}, {x: p.x, y: height}, Math.abs(x % 1) < .001 ? '#cfd6c8' : '#dce1d5', .7); }
    for (let z = Math.floor(min[2] / grid) * grid; z <= max[2]; z += grid) { const p = project([0, 0, z]); line({x: 0, y: p.y}, {x: width, y: p.y}, Math.abs(z % 1) < .001 ? '#cfd6c8' : '#dce1d5', .7); if (p.y > 110 && p.y < height - 45) { ctx.font = '9px ui-monospace, monospace'; ctx.fillStyle = '#8c9988'; ctx.fillText(`${z.toFixed(2)} m`, 18, p.y - 5); } }
    $('planner-scale').textContent = `${grid} m GRID`;
    drawPreview(s);
    // The planned move path connects successive hold locations for each limb.
    const previous = {...route.start}; route.moves.forEach((move, index) => {
      const a = project(holds[previous[move.limb]].pos), b = project(holds[move.hold].pos); previous[move.limb] = move.hold;
      line(a, b, colors[move.limb], index === route.moves.length - 1 ? 3 : 1.5, [4, 5]);
      const offset = move.limb.startsWith('left') ? -1 : 1;
      ctx.fillStyle = colors[move.limb]; ctx.font = 'bold 12px system-ui, sans-serif'; ctx.textAlign = offset < 0 ? 'right' : 'left'; ctx.fillText(`Move ${index + 1}`, b.x + offset * 25, b.y - 15);
    }); ctx.textAlign = 'left';
    route.holds.forEach(hold => {
      const p = project(hold.pos), selected = hold.name === selectedHold, attached = E.LIMBS.filter(l => assigned[l] === hold.name), radius = clamp((hold.radius ?? E.DEFAULTS[hold.type].radius) * s, 5, 24);
      if (selected) circle(p, Math.max(13, radius + 4), '#ffffff55', '#365b50', 1.5);
      if (attached.length) circle(p, 11, null, colors[attached[0]] + '88', 1.5);
      circle({x: p.x + 1, y: p.y + 2}, 7, '#344a3d20');
      circle(p, hold.type === 'foothold' ? 9 : 12, holdColors[hold.type], '#57705e', 1.2);
      circle({x: p.x - 1.5, y: p.y - 1.5}, 1.5, '#f8f5e9aa');
      if (!Object.values(route.start).includes(hold.name)) { ctx.fillStyle = '#fffdf1'; ctx.font = 'bold 11px system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.fillText(holdLabel(route, hold.name).replace('Hold ', ''), p.x, p.y + 4); ctx.textAlign = 'left'; }
    });
    E.LIMBS.forEach(limb => {
      const p = project(endpoints[limb]), h = handlePoint(limb, endpoints[limb]), selected = selectedLimb === limb && tool === 'limbs';
      line(p, h, colors[limb] + '99', 1, [2, 3]); circle(h, 18, selected ? colors[limb] : '#fffdf4', colors[limb], selected ? 2.5 : 1.5);
      ctx.fillStyle = selected ? '#fffdf1' : colors[limb]; ctx.font = 'bold 10px system-ui, sans-serif'; ctx.textAlign = 'center'; ctx.fillText(initials[limb], h.x, h.y + 3); ctx.textAlign = 'left';
      ctx.fillStyle = '#365345'; ctx.font = '12px system-ui, sans-serif'; ctx.textAlign = limb.startsWith('left') ? 'right' : 'left'; ctx.fillText(names[limb], h.x + (limb.startsWith('left') ? -25 : 25), h.y + 4); ctx.textAlign = 'left';
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
    const limb = E.LIMBS.find(l => { const h = handlePoint(l, endpoints[l]); return Math.hypot(h.x - point.x, h.y - point.y) < 22; });
    if (limb) { selectedLimb = limb; drag = {kind: 'limb', limb, point}; renderControls(); draw(); return; }
    const hold = nearestHold(point, route, 18); if (hold) { selectedHold = hold.name; drag = {kind: 'hold', hold: hold.name, route, point}; renderControls(); draw(); return; }
    drag = {kind: 'pan', point, x: camera.x, z: camera.z};
  });
  canvas.addEventListener('pointermove', event => {
    const point = mouse(event);
    if (!drag) { const endpoints = endpointMap(store.route); canvas.style.cursor = nearestHold(point, store.route) || E.LIMBS.some(l => {const p = handlePoint(l,endpoints[l]); return Math.hypot(point.x-p.x,point.y-p.y)<22;}) ? 'grab' : 'default'; return; }
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
  $('editor-tab').addEventListener('click', () => showTab('editor')); $('replay-tab').addEventListener('click', () => showTab('replay')); $('back-to-editor').addEventListener('click', () => showTab('editor'));
  $('tool-holds').addEventListener('click', () => { tool = 'holds'; renderControls(); draw(); }); $('tool-limbs').addEventListener('click', () => { tool = 'limbs'; renderControls(); draw(); });
  $('mode-start').addEventListener('click', () => setMode('start')); $('mode-moves').addEventListener('click', () => setMode('moves'));
  $('planner-fit').addEventListener('click', fit);
  $('editor-template').replaceChildren(...templates.map((t, i) => { const n = make('option', '', t.name); n.value = i; return n; }));
  $('editor-template').addEventListener('change', e => { store.replace(e.target.value === 'imported' ? importedRoute : templates[Number(e.target.value)].route); selectedHold = store.route.holds[0].name; mode = 'moves'; invalidate(); renderControls(); fit(); schedulePreview(); });
  $('editor-name').addEventListener('change', e => edit(r => { r.name = e.target.value.trim(); }));
  $('editor-undo').addEventListener('click', () => { store.undo(); invalidate(); renderControls(); draw(); schedulePreview(); });
  $('editor-reset').addEventListener('click', () => { store.reset(); mode = 'moves'; invalidate('Wall restored. Drag a limb to start a new sequence.'); renderControls(); fit(); schedulePreview(); });
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
      selectedHold = route.holds[0].name; selectedLimb = 'left_hand'; mode = 'moves'; tool = 'limbs';
      const existing = $('editor-template').querySelector('[data-imported]'); if (existing) existing.remove();
      const option = make('option', '', `${route.name} · imported`); option.value = 'imported'; option.dataset.imported = 'true'; $('editor-template').append(option); $('editor-template').value = 'imported';
      invalidate(`Loaded ${file.name}. Run a fresh simulation to test this route.`); renderControls(); fit(); schedulePreview();
    } catch (error) { setStatus('Could not import this route', error instanceof SyntaxError ? 'The file is not valid JSON. Export a route from this editor, then try importing it.' : error.message, 'error'); }
    finally { event.target.value = ''; }
  });
  function endpoint(path) {
    const input = simulatorUrl.trim().replace(/\/$/, '');
    if (input) { const url = new URL(input); if (!['http:', 'https:'].includes(url.protocol)) throw new Error('Use an http or https simulator URL.'); if (url.username || url.password || url.search || url.hash) throw new Error('Use the simulator base URL without credentials, query parameters or a fragment.'); return input + path; }
    if (location.protocol === 'file:') throw new Error('Open the local app at http://127.0.0.1:8765, or enter a hosted simulator URL in Simulator connection.');
    return path;
  }
  function schedulePreview() {
    clearTimeout(previewTimer); previewController?.abort(); previewRevision = -1;
    $('preview-status').textContent = 'Updating body pose… faded geometry is the previous pose. This checks reach only, not grip or stability.';
    draw();
    const revision = store.revision, requestedMode = mode;
    previewTimer = setTimeout(async () => {
      const controller = new AbortController(); previewController = controller;
      const timer = setTimeout(() => controller.abort(), 20000), route = store.route;
      route.start = E.assignments(route, mode === 'moves'); route.moves = [];
      try {
        const response = await fetch(endpoint('/api/preview'), {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({route}), signal: controller.signal});
        const data = await response.json(), p = data.preview;
        if (!response.ok || !p) throw new Error(data.error || 'Pose preview unavailable');
        window.ClimbGeometry.validateTrajectory({...p, route: 'Pose preview', success: false, frames: [{t: 0, pos: p.pos, quat: p.quat}], outcomes: []});
        if (p.limbs?.length !== 4 || p.sites?.length !== 4 || p.targets?.length !== 4 || p.residuals?.length !== 4 || !Number.isFinite(p.maxResidualM) || p.residuals.some(n => !Number.isFinite(n)) || p.sites.some(v => !Array.isArray(v) || v.length !== 3 || !v.every(Number.isFinite)) || p.targets.some(v => v !== null && (!Array.isArray(v) || v.length !== 3 || !v.every(Number.isFinite)))) throw new Error('Invalid pose preview');
        if (revision !== store.revision || requestedMode !== mode || controller.signal.aborted) return;
        preview = p; previewRevision = revision;
        const gap = p.maxResidualM < .01 ? `${(p.maxResidualM * 1000).toFixed(1)} mm` : `${(p.maxResidualM * 100).toFixed(1)} cm`;
        const poseName = mode === 'moves' && store.route.moves.length ? 'Planned end pose' : 'Starting body pose';
        $('preview-status').textContent = p.converged ? `${poseName} found. Reach only so far. Test this pose for support, or test the moves to check how the model gets there.` : `The pose solver has not matched all contacts. Largest gap: ${gap}. Dashed links show the remaining gaps. Try adjusting the holds; this is not a physical impossibility verdict.`;
        $('preview-status').classList.toggle('preview-warning', !p.converged); draw();
      } catch {
        if (revision !== store.revision || requestedMode !== mode || controller !== previewController) return;
        $('preview-status').textContent = 'Body preview unavailable. Faded geometry is the previous pose; markers show your intended contacts. You can still edit and run the simulation.';
        $('preview-status').classList.add('preview-warning'); draw();
      } finally { clearTimeout(timer); }
    }, 300);
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
    simulatorUrl = $('simulator-url').value.trim(); try { endpoint('/api/health'); try { localStorage.setItem('ascent.simulatorUrl', simulatorUrl); } catch {} checkConnection(); schedulePreview(); }
    catch (error) { $('connection-status').textContent = error.message; }
  });
  async function runSimulation(checkType) {
    if (busy) return;
    const token = store.beginRun(), request = ++runId, controller = new AbortController(), started = Date.now(); let timer, elapsedTimer;
    const requestedScopeRevision = poseScopeRevision;
    const isStale = () => request !== runId || token.revision !== store.revision || checkType === 'pose' && requestedScopeRevision !== poseScopeRevision;
    const poseScope = mode === 'moves' && token.route.moves.length ? 'final' : 'starting';
    if (checkType === 'pose') { token.route.start = E.assignments(token.route, mode === 'moves'); token.route.moves = []; }
    try {
      E.validate(token.route); const url = endpoint('/api/simulate'); busy = true; activeCheckType = checkType;
      ['run-example', 'run-simulation', 'test-pose'].forEach(id => $(id).disabled = true);
      $(checkType === 'pose' ? 'test-pose' : 'run-simulation').querySelector('span').textContent = 'Simulating…'; $('view-result').hidden = true;
      const showElapsed = () => { if (!isStale()) setStatus(`Simulating · ${Math.floor((Date.now() - started) / 1000)}s elapsed`, checkType === 'pose' ? `Checking whether the ${poseScope} contacts support the model through settling. Your planned moves are kept.` : 'Calculating the body motion and contact forces. The animation starts when the result is ready.', 'running'); };
      showElapsed(); elapsedTimer = setInterval(showElapsed, 1000);
      timer = setTimeout(() => controller.abort(), 180000);
      const response = await fetch(url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({route: token.route, frameEvery: 20}), signal: controller.signal});
      let data; try { data = await response.json(); } catch { throw new Error('No simulator API is available here. Open Simulator connection to connect the local app or a hosted server.'); }
      if (!response.ok) throw new Error(data.error || (response.status === 429 ? 'The simulator is busy. Try again when the current run finishes.' : 'The simulator could not run this route.'));
      if (!data.trajectory || typeof data.trajectory.success !== 'boolean') throw new Error('The simulator returned no valid trajectory.');
      if (isStale()) { setStatus('Edits kept, older result discarded', 'The route or displayed pose changed while the simulation was running. Run again to test the current version.'); return; }
      data.trajectory.checkType = checkType;
      if (checkType === 'pose') data.trajectory.poseScope = poseScope;
      window.ClimbReplay.loadTrajectory(data.trajectory, {autoplay: true});
      store.accept(token, data.trajectory);
      const result = E.describeResult(data.trajectory); setStatus(result.title, result.text, result.kind);
      $('replay-tab').disabled = false; $('view-result').hidden = false; showTab('replay');
    } catch (error) {
      if (isStale()) { setStatus('Edits kept, older result discarded', 'The previous run ended after the route or displayed pose changed. Run again to test the current version.'); return; }
      setStatus('Simulation unavailable', error.name === 'AbortError' ? 'The request timed out. The server may still be finishing its run. Try again shortly.' : error.message === 'Failed to fetch' || error instanceof TypeError ? 'Could not reach the Python simulator. Check Simulator connection, then run again. Your edits are saved in this tab.' : error.message, 'error');
      if (error instanceof TypeError) $('connection-label').textContent = 'Connect simulator';
    } finally { clearTimeout(timer); clearInterval(elapsedTimer); busy = false; activeCheckType = null; ['run-example', 'run-simulation', 'test-pose'].forEach(id => $(id).disabled = false); renderControls(); }
  }
  $('test-pose').addEventListener('click', () => runSimulation('pose'));
  $('run-simulation').addEventListener('click', () => runSimulation('moves'));
  $('run-example').addEventListener('click', () => {
    if (busy) return;
    const route = E.clone(starter); route.moves = [{limb: 'left_hand', hold: 'target_left'}];
    store.replace(starter); store.commit(route); mode = 'moves'; selectedLimb = 'left_hand'; selectedHold = 'target_left'; $('editor-template').value = '0';
    invalidate(); renderControls(); fit(); schedulePreview(); $('run-simulation').click();
  });
  $('view-result').addEventListener('click', () => showTab('replay'));
  new ResizeObserver(resize).observe(canvas.parentElement);
  renderControls(); resize(); fit(); checkConnection(); schedulePreview();
})();
