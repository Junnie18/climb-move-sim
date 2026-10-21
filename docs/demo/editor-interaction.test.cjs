const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const E = require('./editor-state.js');
const G = require('./geometry.js');

// Exercise the shipped event handlers, including hit testing and async result
// handling. This intentionally does not replace a browser layout/render test.
class Element {
  constructor(tagName = 'div', attrs = {}) {
    this.tagName = tagName; this.attrs = {...attrs}; this.children = [];
    this.listeners = {}; this.dataset = {}; this.style = {setProperty() {}};
    this.hidden = Object.hasOwn(attrs, 'hidden'); this.disabled = Object.hasOwn(attrs, 'disabled');
    this.checked = Object.hasOwn(attrs, 'checked'); this.value = attrs.value || '';
    this.className = attrs.class || ''; this._text = '';
    for (const [key, value] of Object.entries(attrs)) if (key.startsWith('data-')) this.dataset[key.slice(5).replace(/-([a-z])/g, (_, c) => c.toUpperCase())] = value;
    this.classList = {
      contains: name => this.className.split(/\s+/).includes(name),
      toggle: (name, force) => {
        const next = new Set(this.className.split(/\s+/).filter(Boolean));
        if (force ?? !next.has(name)) next.add(name); else next.delete(name);
        this.className = [...next].join(' ');
      },
      add: name => this.classList.toggle(name, true),
      remove: name => this.classList.toggle(name, false),
    };
  }
  append(...nodes) { for (const node of nodes) { node.parentElement = this; this.children.push(node); } }
  prepend(...nodes) { for (const node of nodes) node.parentElement = this; this.children.unshift(...nodes); }
  replaceChildren(...nodes) { this._text = ''; this.children = []; this.append(...nodes); }
  set textContent(value) { this._text = String(value); this.children = []; }
  get textContent() { return this._text + this.children.map(n => n.textContent).join(''); }
  setAttribute(key, value) { this.attrs[key] = String(value); }
  getAttribute(key) { return this.attrs[key] ?? null; }
  addEventListener(type, handler) { (this.listeners[type] ||= []).push(handler); }
  async dispatch(type, detail = {}) {
    const event = {target: this, preventDefault() {}, button: 0, pointerId: 1, ...detail};
    for (const handler of this.listeners[type] || []) await handler(event);
  }
  click() { return this.dispatch('click'); }
  setPointerCapture() {}
  scrollIntoView() {}
  getBoundingClientRect() { return {left: 0, top: 0, width: 800, height: 600}; }
  matches(selector) {
    if (selector.startsWith('.')) return this.classList.contains(selector.slice(1));
    if (selector.startsWith('#')) return this.attrs.id === selector.slice(1);
    const attr = /^\[([^=\]]+)(?:=["']?([^"'\]]+)["']?)?\]$/.exec(selector);
    if (attr) return Object.hasOwn(this.attrs, attr[1]) && (attr[2] === undefined || this.attrs[attr[1]] === attr[2]);
    return this.tagName === selector;
  }
  querySelectorAll(selector) {
    const parts = selector.trim().split(/\s+/), found = [];
    const visit = node => {
      for (const child of node.children) {
        if (child.matches(parts[0])) {
          if (parts.length === 1) found.push(child);
          else found.push(...child.querySelectorAll(parts.slice(1).join(' ')));
        }
        visit(child);
      }
    };
    visit(this); return found;
  }
  querySelector(selector) { return this.querySelectorAll(selector)[0] || null; }
  remove() { this.parentElement.children = this.parentElement.children.filter(node => node !== this); }
}

function documentFromHTML() {
  const document = new Element('document'), stack = [document], ids = new Map();
  const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
  for (const match of html.matchAll(/<!--[\s\S]*?-->|<![^>]*>|<\/?[^>]+>/g)) {
    const tag = match[0];
    if (/^<!/.test(tag)) continue;
    if (/^<\//.test(tag)) { if (stack.length > 1) stack.pop(); continue; }
    const name = /^<([\w-]+)/.exec(tag)?.[1]; if (!name) continue;
    const attrs = {};
    for (const attr of tag.slice(name.length + 1).matchAll(/([\w:-]+)(?:\s*=\s*"([^"]*)"|\s*=\s*'([^']*)')?/g)) attrs[attr[1]] = attr[2] ?? attr[3] ?? '';
    const element = new Element(name, attrs); stack.at(-1).append(element);
    if (attrs.id) ids.set(attrs.id, element);
    if (!/^(?:area|base|br|col|embed|hr|img|input|link|meta|param|source|track|wbr)$/.test(name) && !/\/>$/.test(tag)) stack.push(element);
  }
  document.getElementById = id => ids.get(id) || null;
  document.createElement = tag => new Element(tag);
  document.createTextNode = text => { const node = new Element('#text'); node.textContent = text; return node; };
  document.body = document.querySelector('body');
  return document;
}

function canvasContext() {
  const labels = new Map(), calls = [];
  return new Proxy({
    labels, calls,
    fillText(text, x, y) { labels.set(String(text), {x, y}); calls.push({method: 'fillText', args: [text, x, y]}); },
    measureText(text) { return {width: String(text).length * 6}; },
    createLinearGradient() { return {addColorStop() {}}; },
    createRadialGradient() { return {addColorStop() {}}; },
  }, {get(target, key) { return key in target ? target[key] : (...args) => { calls.push({method: key, args}); }; }});
}

function fixture() {
  const route = JSON.parse(fs.readFileSync(path.join(__dirname, '../../data/routes/showcase/01_supported_steps.json'), 'utf8'));
  const frames = [0, 2.8, 3, 3.5, 4].map(t => ({
    t, pos: [[0, .1, 1.2]], quat: [[1, 0, 0, 0]], grips: [true, true, true, true],
    holdNames: Object.values(route.start), forces: E.LIMBS.map(() => [0, 0, 100]),
    phase: t < 3 ? 'initial_settle' : 'dynamic', moveIndex: t < 3 ? -1 : 0,
    com: [0, .1, 1.2],
  }));
  return {route: route.name, routeDefinition: route, success: true, bodyNames: ['torso'], limbs: [...E.LIMBS],
    frames, geometry: {bodies: [{name: 'torso', geoms: []}], holds: route.holds.map(h => ({...h, maxGripN: 1400}))},
    outcomes: [{passed: true, phase: 'settle', move: 'left_hand -> hand_step_0', reason: 'Held'}]};
}

function boot({viewer = false, simulate} = {}) {
  const document = documentFromHTML(), contexts = new Map(), callbacks = [], requests = [];
  for (const canvas of document.querySelectorAll('canvas')) {
    const context = canvasContext(); contexts.set(canvas.attrs.id, context); canvas.getContext = () => context;
  }
  let store;
  const window = {ClimbEditorState: {...E, create(route) { store = E.create(route); return store; }}, ClimbGeometry: G, CLIMB_DEMOS: [fixture()], devicePixelRatio: 1};
  window.addEventListener = () => {};
  const replayLoads = [], replayResizes = [];
  window.ClimbReplay = {pause() {}, resize() { replayResizes.push(true); }, invalidate() {}, loadTrajectory(...args) { replayLoads.push(args); }};
  const context = vm.createContext({window, document, location: {hostname: 'localhost', pathname: '/', protocol: 'http:'},
    localStorage: {getItem() { return null; }, setItem() {}}, URL, Blob, AbortController, console,
    setTimeout() { return 1; }, clearTimeout() {}, setInterval() { return 2; }, clearInterval() {}, performance: {now: () => 0},
    requestAnimationFrame(callback) { callbacks.push(callback); return callbacks.length; },
    ResizeObserver: class { observe() {} },
    fetch: async (url, options = {}) => {
      requests.push({url, options});
      if (url.endsWith('/api/health')) return {ok: true, json: async () => ({status: 'ok', engine: 'MuJoCo'})};
      if (url.endsWith('/api/preview')) return {ok: true, json: async () => ({error: 'Preview is not needed for this interaction test'})};
      const trajectory = simulate ? await simulate(JSON.parse(options.body)) : fixture();
      return {ok: true, json: async () => ({trajectory})};
    },
  });
  const load = file => vm.runInContext(fs.readFileSync(path.join(__dirname, file), 'utf8'), context, {filename: file});
  if (viewer) load('viewer.js'); else load('editor.js');
  const get = id => { const element = document.getElementById(id); assert.ok(element, `Missing HTML element ${id}`); return element; };
  function handle(limb) {
    const label = {left_hand: 'LH', right_hand: 'RH', left_foot: 'LF', right_foot: 'RF'}[limb];
    const point = contexts.get('planner').labels.get(label); assert.ok(point, `Missing rendered ${label} handle`);
    return {x: point.x, y: point.y - 3};
  }
  function holdPoint(name) {
    const route = store.route, assigned = E.assignments(route), left = route.holds.find(h => h.name === assigned.left_hand), right = route.holds.find(h => h.name === assigned.right_hand);
    const lh = handle('left_hand'), rh = handle('right_hand');
    const scale = (rh.x - 31 - (lh.x + 31)) / (right.pos[0] - left.pos[0]);
    const hold = route.holds.find(h => h.name === name); assert.ok(hold, `Missing hold ${name}`);
    return {x: lh.x + 31 + (hold.pos[0] - left.pos[0]) * scale, y: lh.y - (hold.pos[2] - left.pos[2]) * scale};
  }
  async function drag(from, to) {
    const canvas = get('planner');
    await canvas.dispatch('pointerdown', {clientX: from.x, clientY: from.y});
    await canvas.dispatch('pointermove', {clientX: to.x, clientY: to.y});
    await canvas.dispatch('pointerup', {clientX: to.x, clientY: to.y});
  }
  return {get, store, window, handle, holdPoint, drag, requests, replayLoads, replayResizes, callbacks, contexts};
}

test('first direct limb drag queues exactly one move while preserving the starting stance', async () => {
  const app = boot(), originalStart = app.store.route.start;
  assert.equal(app.store.route.moves.length, 0, 'Opening the editor must not queue hidden example moves');
  await app.drag(app.handle('left_hand'), app.holdPoint('target_left'));
  assert.deepEqual(app.store.route.moves, [{limb: 'left_hand', hold: 'target_left'}]);
  assert.deepEqual(app.store.route.start, originalStart);
  assert.equal(app.get('move-count').textContent, '1');
});

test('limb handles still work after choosing the move-holds tool', async () => {
  const app = boot();
  await app.get('tool-holds').click();
  await app.drag(app.handle('right_hand'), app.holdPoint('target_right'));
  assert.deepEqual(app.store.route.moves, [{limb: 'right_hand', hold: 'target_right'}]);
});

test('dragging a hold moves its position without adding a limb move', async () => {
  const app = boot(), original = app.store.route.holds.find(h => h.name === 'target_left').pos;
  const point = app.holdPoint('target_left');
  await app.drag(point, {x: point.x - 35, y: point.y - 20});
  assert.notDeepEqual(app.store.route.holds.find(h => h.name === 'target_left').pos, original);
  assert.equal(app.store.route.moves.length, 0);
});

test('dropping a limb away from a hold keeps both the stance and sequence unchanged', async () => {
  const app = boot(), before = app.store.route;
  await app.drag(app.handle('left_hand'), {x: 20, y: 20});
  assert.deepEqual(app.store.route, before);
  assert.match(app.get('editor-status').textContent, /hold/i);
});

test('a cancelled hold drag leaves the route unchanged', async () => {
  const app = boot(), before = app.store.route, point = app.holdPoint('target_left');
  await app.get('planner').dispatch('pointerdown', {clientX: point.x, clientY: point.y});
  await app.get('planner').dispatch('pointermove', {clientX: point.x + 50, clientY: point.y});
  await app.get('planner').dispatch('pointercancel');
  assert.deepEqual(app.store.route, before);
});

test('Run submits the edited sequence and shows its returned result', async () => {
  const app = boot();
  await app.drag(app.handle('left_hand'), app.holdPoint('target_left'));
  await app.get('run-simulation').click();
  const sent = app.requests.find(request => request.url.endsWith('/api/simulate'));
  assert.ok(sent, 'Clicking Run should call the simulation API');
  assert.deepEqual(JSON.parse(sent.options.body).route.moves, [{limb: 'left_hand', hold: 'target_left'}]);
  assert.equal(app.replayLoads.length, 1);
  assert.equal(app.get('replay-stage').hidden, false);
  assert.ok(app.replayResizes.length > 0, 'Showing the replay must size and paint it without waiting for an animation frame');
  assert.equal(app.get('run-simulation').disabled, false);
});

test('Run an example submits one move, and Reset restores an empty sequence', async () => {
  const app = boot();
  await app.get('run-example').click();
  const sent = app.requests.find(request => request.url.endsWith('/api/simulate'));
  assert.ok(sent, 'The example must actually run, not just populate the editor');
  assert.deepEqual(JSON.parse(sent.options.body).route.moves, [{limb: 'left_hand', hold: 'target_left'}]);
  await app.get('editor-reset').click();
  assert.equal(app.store.route.moves.length, 0);
});

test('an edit during simulation discards the old result and keeps the new draft', async () => {
  let resolve;
  const response = new Promise(done => { resolve = done; });
  const app = boot({simulate: () => response});
  const run = app.get('run-simulation').click();
  await app.drag(app.handle('left_hand'), app.holdPoint('target_left'));
  resolve(fixture()); await run;
  assert.equal(app.replayLoads.length, 0);
  assert.equal(app.store.route.moves.length, 1);
  assert.match(app.get('editor-status').textContent, /changed|current|again/i);
});

test('a fresh simulation starts playing the move; recorded examples stay paused', async () => {
  const app = boot({viewer: true});
  assert.equal(app.get('play-state').textContent, 'PAUSED');
  app.window.ClimbReplay.loadTrajectory(fixture());
  assert.equal(app.get('play-state').textContent, 'PLAYING');
  assert.ok(Number(app.get('timeline').value) >= 2.8, 'Skip the initial settling wait for a new result');
  assert.ok(Number(app.get('timeline').value) < 4, 'Start before the end of the motion');
  const initialTime = Number(app.get('timeline').value);
  app.callbacks.shift()(0); app.callbacks.shift()(100);
  assert.ok(Number(app.get('timeline').value) > initialTime, 'Animation frames must advance the fresh result');
  await app.get('restart').click();
  assert.equal(Number(app.get('timeline').value), 0, 'Restart must retain the full recorded timeline');
  await app.get('route-select').dispatch('change', {target: {value: '0'}});
  assert.equal(app.get('play-state').textContent, 'PAUSED');
});

test('opening a fresh result paints the scene and force plot without waiting for animation frames', () => {
  const app = boot({viewer: true}), scene = app.contexts.get('scene'), plot = app.contexts.get('force-plot');
  scene.calls.length = 0; plot.calls.length = 0;
  app.window.ClimbReplay.loadTrajectory(fixture());
  assert.ok(scene.calls.some(call => call.method === 'arc'), 'Loading a visible trajectory should immediately draw its holds');
  assert.ok(plot.calls.some(call => call.method === 'stroke'), 'Loading a visible trajectory should immediately draw its force traces');
  scene.calls.length = 0; plot.calls.length = 0;
  app.window.ClimbReplay.resize();
  // Background/inactive browser tabs may defer requestAnimationFrame callbacks.
  // Data loading and showing the canvas must still paint a real first frame.
  assert.ok(scene.calls.some(call => call.method === 'arc'), 'The recorded holds should be drawn synchronously');
  assert.ok(scene.calls.some(call => call.method === 'stroke'), 'The scene should contain visible geometry, not only a cleared background');
  assert.ok(plot.calls.some(call => call.method === 'lineTo'), 'Force traces should be drawn synchronously');
  assert.ok(plot.calls.some(call => call.method === 'stroke'), 'Force traces must be stroked');
});

test('an initialization failure displays its diagnostic without inventing a replay', () => {
  const app = boot({viewer: true}), failure = fixture();
  failure.success = false; failure.frames = [];
  failure.outcomes = [{passed: false, phase: 'initialization', reason: 'Starting holds are out of reach', detail: {category: 'geometric_reach_limit'}}];
  app.window.ClimbReplay.loadTrajectory(failure);
  assert.equal(app.get('scene-message').hidden, false);
  assert.equal(app.get('play').disabled, true);
  assert.equal(app.get('play-state').textContent, 'NO RECORD');
  assert.match(app.get('failure-detail').textContent, /out of reach/);
});
