/* Compiled MuJoCo geoms are body-local. Quaternions are scalar-first (w,x,y,z). */
(function (root) {
  'use strict';
  const add = (a, b) => a.map((v, i) => v + b[i]);
  const sub = (a, b) => a.map((v, i) => v - b[i]);
  const scale = (a, s) => a.map(v => v * s);
  const dot = (a, b) => a.reduce((v, x, i) => v + x * b[i], 0);
  const norm = a => Math.hypot(...a);
  function rotate(q, v) {
    const [w, x, y, z] = q;
    const t = [2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])];
    return [v[0] + w * t[0] + y * t[2] - z * t[1], v[1] + w * t[1] + z * t[0] - x * t[2], v[2] + w * t[2] + x * t[1] - y * t[0]];
  }
  function transform(bodyPos, bodyQuat, geom, point = [0, 0, 0]) {
    return add(bodyPos, rotate(bodyQuat, add(geom.pos, rotate(geom.quat, point))));
  }
  function capsuleEndpoints(bodyPos, bodyQuat, geom) {
    return [-1, 1].map(sign => transform(bodyPos, bodyQuat, geom, [0, 0, sign * geom.size[1]]));
  }
  function cameraBasis(azimuth, elevation) {
    const sa = Math.sin(azimuth), ca = Math.cos(azimuth), se = Math.sin(elevation), ce = Math.cos(elevation);
    return {right: [ca, -sa, 0], up: [-sa * se, -ca * se, ce], toward: [sa * ce, ca * ce, se]};
  }
  function project(point, camera, width, height) {
    const v = sub(point, camera.target), basis = cameraBasis(camera.azimuth, camera.elevation);
    return {x: width / 2 + dot(v, basis.right) * camera.scale, y: height / 2 - dot(v, basis.up) * camera.scale, depth: dot(v, basis.toward)};
  }
  function frameIndex(frames, time) {
    let low = 0, high = frames.length - 1;
    while (low < high) { const mid = Math.ceil((low + high) / 2); if (frames[mid].t <= time) low = mid; else high = mid - 1; }
    return low;
  }
  function validateTrajectory(data) {
    const fail = message => { throw new Error(message); };
    const vector = (value, length) => Array.isArray(value) && value.length === length && value.every(n => Number.isFinite(n) && Math.abs(n) <= 1e9);
    const quaternion = value => vector(value, 4) && Math.abs(norm(value) - 1) < .001;
    if (!data || typeof data.route !== 'string' || typeof data.success !== 'boolean' || !Array.isArray(data.frames) || !data.frames.length || !Array.isArray(data.bodyNames) || !data.bodyNames.length || !Array.isArray(data.geometry?.bodies) || !data.geometry.bodies.length || !Array.isArray(data.geometry.holds)) fail('Expected an exported trajectory with a route, result, nonempty frames, bodyNames, and geometry.');
    if (data.bodyNames.length > 1024 || data.geometry.holds.length > 10000) fail('This trajectory exceeds the browser viewer’s geometry limits.');
    if (data.bodyNames.some(n => typeof n !== 'string') || new Set(data.bodyNames).size !== data.bodyNames.length) fail('Body names must be unique strings.');
    const limbCount = data.limbs?.length ?? 4;
    if (data.limbs !== undefined && (!Array.isArray(data.limbs) || limbCount !== 4 || data.limbs.some(l => typeof l !== 'string') || new Set(data.limbs).size !== limbCount)) fail('The trajectory must name its four distinct limbs.');
    if (data.outcomes !== undefined && (!Array.isArray(data.outcomes) || data.outcomes.some(o => !o || typeof o.passed !== 'boolean' || typeof o.phase !== 'string' || (o.reason !== undefined && typeof o.reason !== 'string')))) fail('Outcome diagnostics must include a boolean passed value and a phase.');
    data.frames.forEach((f, index) => {
      if (!f || !Number.isFinite(f.t) || f.t < 0 || (index && f.t < data.frames[index - 1].t)) fail('Frame timestamps must be finite, nonnegative, and ordered.');
      if (!Array.isArray(f.pos) || !Array.isArray(f.quat) || f.pos.length !== data.bodyNames.length || f.quat.length !== data.bodyNames.length || f.pos.some(p => !vector(p, 3)) || f.quat.some(q => !quaternion(q))) fail('Every body transform needs a finite 3D position and a normalized (w, x, y, z) quaternion.');
      if (f.grips !== undefined && (!Array.isArray(f.grips) || f.grips.length !== limbCount || f.grips.some(g => typeof g !== 'boolean'))) fail('Grip state must contain one boolean per limb.');
      ['anchors', 'forces'].forEach(key => { if (f[key] !== undefined && (!Array.isArray(f[key]) || f[key].length !== limbCount || f[key].some(v => v !== null && !vector(v, 3)))) fail(`${key} must contain one finite 3D vector per limb.`); });
      if (f.com !== undefined && f.com !== null && !vector(f.com, 3)) fail('Center of mass must be a finite 3D vector.');
      if (f.holdNames !== undefined && (!Array.isArray(f.holdNames) || f.holdNames.length !== limbCount || f.holdNames.some(n => n !== null && typeof n !== 'string'))) fail('Hold assignments must contain one name or null per limb.');
    });
    data.geometry.bodies.forEach(body => {
      if (!body || !data.bodyNames.includes(body.name) || !Array.isArray(body.geoms)) fail('Geometry must refer to named trajectory bodies.');
      body.geoms.forEach(geom => {
        if (!geom || !['capsule', 'sphere', 'box'].includes(geom.type) || !vector(geom.pos, 3) || !quaternion(geom.quat) || !vector(geom.size, 3) || geom.size[0] <= 0 || geom.size.some(n => n < 0) || (geom.type === 'box' && geom.size.some(n => n <= 0))) fail('Geometry needs a supported shape, finite transform, and valid positive dimensions.');
      });
    });
    data.geometry.holds.forEach(hold => { if (!hold || typeof hold.name !== 'string' || !vector(hold.pos, 3) || (hold.radius !== undefined && (!Number.isFinite(hold.radius) || hold.radius <= 0 || hold.radius > 1e3)) || (hold.maxGripN !== undefined && (!Number.isFinite(hold.maxGripN) || hold.maxGripN <= 0))) fail('Holds need finite positions, positive radii, and positive force ratings.'); });
    const wall = data.geometry.wall;
    if (wall && (!vector(wall.pos, 3) || !vector(wall.size, 3) || wall.size.some(n => n <= 0 || n > 100))) fail('Wall geometry must have a finite position and positive half-dimensions no greater than 100 m.');
    return data;
  }
  const api = {add, sub, scale, dot, norm, rotate, transform, capsuleEndpoints, cameraBasis, project, frameIndex, validateTrajectory};
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ClimbGeometry = api;
})(typeof window !== 'undefined' ? window : globalThis);
