const test = require('node:test');
const assert = require('node:assert/strict');
const E = require('./editor-state.js');
const route = () => ({name: 'Example', holds: E.LIMBS.map((limb, i) => ({name: limb, type: i < 2 ? 'jug' : 'foothold', pos: [(i % 2 ? 1 : -1) * .2, .05, i < 2 ? 1.55 : .75]})), start: Object.fromEntries(E.LIMBS.map(limb => [limb, limb])), moves: []});
test('edits invalidate a completed verdict and reject in-flight results for an older revision', () => {
  const s = E.create(route()), first = s.beginRun(); assert.equal(s.accept(first, {success: true}), true);
  const next = s.route; next.holds[0].pos[2] += .1; s.commit(next);
  assert.equal(s.result, null); assert.equal(s.accept(first, {success: true}), false);
  const second = s.beginRun(); assert.equal(s.accept(second, {success: false}), true); assert.equal(s.result.success, false);
});
test('undo preserves original start, advances revision and never revives an old verdict', () => {
  const original = route(), s = E.create(original), token = s.beginRun(), next = s.route;
  next.start.left_hand = 'right_hand'; s.commit(next); s.undo();
  assert.deepEqual(s.route, original); assert.equal(s.revision, 2); assert.equal(s.accept(token, {}), false);
});
test('start assignments and planned final assignments remain separate', () => {
  const r = route(); r.moves.push({limb: 'left_hand', hold: 'right_hand'});
  assert.equal(E.assignments(r, false).left_hand, 'left_hand'); assert.equal(E.assignments(r).left_hand, 'right_hand');
});
test('route state cannot be mutated outside commit or through an in-flight snapshot', () => {
  const s = E.create(route()), snapshot = s.route, token = s.beginRun();
  snapshot.holds[0].pos[2] = 3; token.route.name = 'Modified';
  assert.equal(s.route.holds[0].pos[2], 1.55); assert.equal(s.route.name, 'Example');
});
test('invalid edits fail before replacing the current route', () => {
  const s = E.create(route()), next = s.route; next.holds[0].pos[0] = NaN;
  assert.throws(() => s.commit(next), /finite/); assert.equal(s.revision, 0);
  next.holds[0].pos[0] = 0; next.start.left_hand = 'missing'; assert.throws(() => s.commit(next), /Assign/);
});
test('reset restores the template and invalidates pending simulation', () => {
  const s = E.create(route()), next = s.route; next.moves = [{limb: 'left_hand', hold: 'right_hand'}]; s.commit(next);
  const pending = s.beginRun(); s.reset(); assert.equal(s.route.moves.length, 0); assert.equal(s.accept(pending, {}), false);
});
test('malformed hold types, normals and moves fail with actionable validation errors', () => {
  const r = route(); r.holds[0].type = '__proto__'; assert.throws(() => E.validate(r), /supported hold type/);
  r.holds[0].type = 'jug'; r.holds[0].normal = [0, 0, 0]; assert.throws(() => E.validate(r), /nonzero finite/);
  delete r.holds[0].normal; r.moves = [null]; assert.throws(() => E.validate(r), /valid single-limb moves/);
});
test('starter keeps the four starting contacts and two separated targets, with no inherited moves', () => {
  const source = route(); source.holds.push({name: 'old_target', type: 'jug', pos: [0, .05, 1.6]});
  source.moves.push({limb: 'left_hand', hold: 'old_target'});
  const starter = E.starter(source);
  assert.equal(starter.holds.length, 6); assert.deepEqual(starter.start, source.start); assert.deepEqual(starter.moves, []);
  assert.equal(starter.holds.some(h => h.name === 'old_target'), false);
  assert.ok(Math.abs(starter.holds.find(h => h.name === 'target_left').pos[2] - 1.75) < 1e-10);
  assert.equal(source.moves.length, 1);
});
test('move distances use the previous contact of that limb, not always the starting hold', () => {
  const r = E.starter(route()); r.moves = [{limb: 'left_hand', hold: 'target_left'}, {limb: 'left_hand', hold: 'left_hand'}];
  assert.ok(Math.abs(E.moveDistance(r, 0) - .2) < 1e-10);
  assert.ok(Math.abs(E.moveDistance(r, 1) - .2) < 1e-10);
});
test('result explanations distinguish unreachable targets, grip failures, and planner failures', () => {
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'reachability', detail: {category: 'geometric_reach_limit'}}]}).title, /reach/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'settle', detail: {category: 'grip_strength_exceeded'}}]}).title, /grip/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'reachability', detail: {category: 'ik_nonconvergence'}}]}).title, /pose/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'initialization', detail: {category: 'ik_nonconvergence'}}]}).title, /starting/i);
  assert.match(E.describeResult({success: true, outcomes: []}).title, /stance/i);
  assert.match(E.describeResult({success: true, outcomes: [{passed: true, move: '(starting stance)', phase: 'initialization', detail: {category: 'completed'}}]}).title, /stance/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'settle', detail: {category: 'contact_cone_exceeded', limb: 'left_foot'}}]}).title, /reached.*left foot/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'initialization', detail: {category: 'initial_ik_nonconvergence'}}]}).text, /stability has not been tested/i);
  assert.match(E.describeResult({success: false, outcomes: [{passed: false, phase: 'initialization', detail: {category: 'numerical_instability'}}]}).text, /numerical failure/i);
  assert.match(E.describeResult({success: true, outcomes: [{passed: true, move: '(starting stance)', phase: 'initialization'}]}).text, /No climbing moves were tested/);
});

test('planner failures stay unresolved instead of declaring a human move physically impossible', () => {
  for (const category of ['geometric_reach_limit', 'ik_nonconvergence', 'initial_ik_nonconvergence', 'numerical_instability']) {
    const explanation = E.describeResult({success: false, outcomes: [{passed: false, phase: 'planning', detail: {category}}]});
    assert.equal(explanation.kind, 'unresolved', category);
  }
  const reach = E.describeResult({success: false, outcomes: [{passed: false, phase: 'reachability', detail: {category: 'geometric_reach_limit'}}]});
  assert.match(reach.text, /torso|body shift/i);
  assert.match(reach.text, /pose/i);
});

test('every pose result identifies the tested starting or final contacts independently of editor view', () => {
  for (const [poseScope, label] of [['starting', 'Starting pose'], ['final', 'Final pose']]) {
    const success = E.describeResult({success: true, checkType: 'pose', poseScope, outcomes: []});
    assert.ok(success.title.startsWith(label));
    assert.doesNotMatch(success.text, /displayed contacts/i);
    for (const category of ['initial_ik_nonconvergence', 'numerical_instability', 'grip_strength_exceeded', 'contact_cone_exceeded']) {
      const failure = E.describeResult({success: false, checkType: 'pose', poseScope, outcomes: [{passed: false, phase: 'initialization', detail: {category}}]});
      assert.ok(failure.title.startsWith(label), category);
    }
  }
});
