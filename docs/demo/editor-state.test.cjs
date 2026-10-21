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
