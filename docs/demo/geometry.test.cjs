/* Run with: node --test docs/demo/geometry.test.cjs */
const {test} = require('node:test');
const assert = require('node:assert/strict');
const G = require('./geometry.js');
function near(actual, expected) { actual.forEach((n, i) => assert.ok(Math.abs(n - expected[i]) < 1e-9, `${actual} != ${expected}`)); }
test('body and local geom rotations compose for actual capsule endpoints', () => {
  const q = [Math.SQRT1_2, 0, Math.SQRT1_2, 0];
  const geom = {pos: [0, 0, 1], quat: q, size: [.1, .5, 0]};
  const [a, b] = G.capsuleEndpoints([1, 2, 3], q, geom);
  near(a, [2, 2, 3.5]); near(b, [2, 2, 2.5]);
});
test('front and side projection preserve vertical world axis', () => {
  for (const azimuth of [0, Math.PI / 2]) {
    const camera = {target: [0, 0, 0], azimuth, elevation: 0, scale: 100};
    const p = G.project([0, 0, 1], camera, 400, 300);
    assert.equal(p.x, 200); assert.equal(p.y, 50);
  }
  near(Object.values(G.project([1, 0, 0], {target: [0, 0, 0], azimuth: 0, elevation: 0, scale: 100}, 400, 300)), [300, 150, 0]);
});
test('playhead chooses last sampled frame without inventing a future state', () => {
  const frames = [{t: 0}, {t: .04}, {t: .08}, {t: .11}];
  assert.equal(G.frameIndex(frames, -.01), 0);
  assert.equal(G.frameIndex(frames, .039), 0);
  assert.equal(G.frameIndex(frames, .04), 1);
  assert.equal(G.frameIndex(frames, .1), 2);
  assert.equal(G.frameIndex(frames, 1), 3);
});
const trajectory = () => ({route: 'imported fixture', success: true, bodyNames: ['torso'], geometry: {bodies: [{name: 'torso', geoms: [{type: 'sphere', pos: [0,0,0], quat: [1,0,0,0], size: [.1,0,0]}]}], holds: []}, frames: [{t: 0, pos: [[0,0,1]], quat: [[1,0,0,0]]}]});
test('JSON imports reject invalid numeric geometry before reaching Canvas', () => {
  assert.doesNotThrow(() => G.validateTrajectory(trajectory()));
  for (const mutate of [
    data => { data.frames = []; },
    data => { data.frames[0].pos[0][0] = Infinity; },
    data => { data.frames[0].quat[0] = [0,0,0,0]; },
    data => { data.geometry.bodies[0].geoms[0].size[0] = -.1; },
    data => { data.geometry.wall = {pos: [0,0,0], size: [1e9,.1,1e9]}; },
    data => { data.frames[0].forces = [[1,2,NaN],null,null,null]; },
    data => { data.success = 'false'; },
    data => { data.limbs = 'hands'; },
    data => { data.frames.push({...data.frames[0], t: -.1}); },
  ]) { const data = trajectory(); mutate(data); assert.throws(() => G.validateTrajectory(data)); }
});
