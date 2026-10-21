/* Route editing is independent from simulation. Revisions fence asynchronous results. */
(function (root) {
  'use strict';
  const LIMBS = ['left_hand', 'right_hand', 'left_foot', 'right_foot'];
  const DEFAULTS = {
    jug: {friction: 1.3, max_grip_n: 1400, radius: .06, normal: [0, 1, 0]},
    crimp: {friction: 1.1, max_grip_n: 550, radius: .03, normal: [0, 1, 0]},
    sloper: {friction: .55, max_grip_n: 900, radius: .075, normal: [0, 1, 0]},
    foothold: {friction: 1, max_grip_n: 1200, radius: .035, normal: [0, 0, 1]}
  };
  const clone = value => JSON.parse(JSON.stringify(value));
  function validate(route) {
    if (!route || typeof route.name !== 'string' || !route.name.trim()) throw new Error('Give the route a name.');
    if (!Array.isArray(route.holds) || route.holds.length < 4 || route.holds.length > 40) throw new Error('Use between 4 and 40 holds.');
    const names = new Set();
    for (const hold of route.holds) {
      if (!hold || typeof hold.name !== 'string' || !hold.name || names.has(hold.name)) throw new Error('Each hold needs a unique name.');
      names.add(hold.name);
      if (typeof hold.type !== 'string' || !Object.hasOwn(DEFAULTS, hold.type)) throw new Error('Choose a supported hold type.');
      if (!Array.isArray(hold.pos) || hold.pos.length !== 3 || !hold.pos.every(Number.isFinite)) throw new Error('Hold coordinates must be finite numbers.');
      if (Math.abs(hold.pos[0]) > 3 || hold.pos[1] < 0 || hold.pos[1] > 1 || hold.pos[2] < .05 || hold.pos[2] > 5) throw new Error('Keep holds within x ±3 m, wall offset 0–1 m, and height 0.05–5 m.');
      for (const key of ['friction', 'max_grip_n', 'radius']) if (hold[key] !== undefined && (!Number.isFinite(hold[key]) || hold[key] < 0)) throw new Error('Grip properties must be finite nonnegative numbers.');
      if (hold.friction > 4 || (hold.max_grip_n !== undefined && (hold.max_grip_n < 1 || hold.max_grip_n > 10000)) || (hold.radius !== undefined && (hold.radius < .005 || hold.radius > .3))) throw new Error('Use friction 0–4, grip caps 1–10000 N and hold radii 0.005–0.3 m.');
      if (hold.normal !== undefined && (!Array.isArray(hold.normal) || hold.normal.length !== 3 || !hold.normal.every(Number.isFinite) || Math.hypot(...hold.normal) < 1e-12)) throw new Error('A hold normal must be a nonzero finite 3D vector.');
    }
    if (!route.start || Object.keys(route.start).length !== 4 || LIMBS.some(limb => !names.has(route.start[limb]))) throw new Error('Assign all four starting limbs to a hold.');
    if (!Array.isArray(route.moves) || route.moves.length > 12 || route.moves.some(move => !move || !LIMBS.includes(move.limb) || !names.has(move.hold))) throw new Error('Use at most 12 valid single-limb moves.');
    return route;
  }
  function fromDemo(demo) {
    if (demo.routeDefinition) return clone(demo.routeDefinition);
    const frame = demo.frames?.[0];
    if (!frame?.holdNames?.every(Boolean)) throw new Error('This trajectory has no editable starting assignments.');
    const moves = [], seen = new Set();
    for (const outcome of demo.outcomes || []) {
      const match = /^([a-z_]+)\s*->\s*(.+)$/.exec(outcome.move || '');
      if (match && !seen.has(outcome.move)) { moves.push({limb: match[1], hold: match[2]}); seen.add(outcome.move); }
    }
    return {name: demo.route, holds: demo.geometry.holds.map(h => ({name: h.name, pos: clone(h.pos), type: h.type, ...(h.normal ? {normal: clone(h.normal)} : {}), ...(h.friction ? {friction: h.friction} : {}), ...(h.maxGripN ? {max_grip_n: h.maxGripN} : {}), ...(h.radius ? {radius: h.radius} : {})})), start: Object.fromEntries(LIMBS.map((limb, i) => [limb, frame.holdNames[i]])), moves};
  }
  function assignments(route, includeMoves = true) {
    const result = {...route.start};
    if (includeMoves) route.moves.forEach(move => { result[move.limb] = move.hold; });
    return result;
  }
  function starter(source) {
    const route = clone(source), contacts = new Set(Object.values(route.start));
    route.name = 'My climbing route'; route.moves = [];
    route.holds = route.holds.filter(h => contacts.has(h.name));
    for (const side of ['left', 'right']) {
      const origin = route.holds.find(h => h.name === route.start[side + '_hand']);
      route.holds.push({name: 'target_' + side, type: 'jug', pos: [origin.pos[0], origin.pos[1], Number((origin.pos[2] + .2).toFixed(3))]});
    }
    return validate(route);
  }
  function moveDistance(route, index) {
    const move = route.moves[index], previous = {...route.start};
    route.moves.slice(0, index).forEach(m => { previous[m.limb] = m.hold; });
    const a = route.holds.find(h => h.name === previous[move.limb]).pos, b = route.holds.find(h => h.name === move.hold).pos;
    return Math.hypot(...a.map((value, i) => value - b[i]));
  }
  function describeResult(data) {
    const failure = data.outcomes?.find(o => !o.passed), category = failure?.detail?.category;
    const hasMoves = data.routeDefinition ? data.routeDefinition.moves.length > 0 : data.outcomes?.some(o => o.phase !== 'initialization' && o.move !== '(starting stance)');
    if (data.success) return {title: hasMoves ? 'The model completed your moves' : 'The starting stance held', text: hasMoves ? 'The simulated climber passed the configured movement, contact and grip checks. Watch the motion below. This is a simplified model, not a guarantee for a human climber.' : 'The simulated climber held its four starting contacts through the settling check. No climbing moves were tested. Add a move to test a reach.', kind: 'passed'};
    if (category === 'numerical_instability') return {title: 'The simulation became unstable', text: 'Reset the wall or try a smaller change. This is a numerical failure, not a physical feasibility verdict.', kind: 'failed'};
    if (category === 'initial_ik_nonconvergence') return {title: 'No starting body pose was found', text: 'Move the starting holds closer together or reset the wall. The pose solver could not match all four starting contacts; stability has not been tested.', kind: 'failed'};
    if (failure?.phase === 'initialization') return {title: 'The starting pose did not hold', text: 'Try moving the starting holds closer to their original positions, or reset the wall. The model could not establish a stable starting stance.', kind: 'failed'};
    if (category === 'geometric_reach_limit') return {title: 'That hold is out of reach from this stance', text: 'Move the target closer to the hand or foot, then test again. The other contacts stay fixed during this reach.', kind: 'failed'};
    if (category === 'ik_nonconvergence') return {title: 'No suitable body pose was found', text: 'Move the target closer or adjust the other contacts. The pose solver stopped; this does not prove that a person cannot make the move.', kind: 'failed'};
    if (category === 'grip_strength_exceeded') return {title: 'A grip could not support the load', text: 'Try a closer target or change the hold’s grip strength in Advanced settings. The calculated force exceeded the configured grip limit.', kind: 'failed'};
    if (category === 'contact_cone_exceeded') {
      const limb = failure.detail?.limb?.replace(/_/g, ' '), contact = limb ? `the ${limb}` : 'a contact';
      return {title: failure.phase === 'settle' ? `The hold was reached, then support failed at ${contact}` : `The model lost support at ${contact}`, text: 'The contact needed force in a direction that its friction constraint could not support. Try a smaller move or adjust the starting stance.', kind: 'failed'};
    }
    if (category === 'contact_not_reached') return {title: 'The limb did not settle onto the hold', text: 'The controller could not reach the target closely and slowly enough to attach. Bring the target closer and test again.', kind: 'failed'};
    return {title: 'The model stopped before finishing', text: 'Try a smaller move. The detailed diagnostic below explains which controller check stopped this attempt.', kind: 'failed'};
  }
  function create(route) {
    validate(route);
    let current = clone(route), baseline = clone(route), revision = 0, history = [], result = null;
    return {
      get route() { return clone(current); }, get revision() { return revision; }, get result() { return result; }, get canUndo() { return history.length > 0; },
      commit(next) { validate(next); if (JSON.stringify(next) === JSON.stringify(current)) return false; history.push(current); if (history.length > 80) history.shift(); current = clone(next); revision++; result = null; return true; },
      undo() { if (!history.length) return false; current = history.pop(); revision++; result = null; return true; },
      reset() { history.push(current); current = clone(baseline); revision++; result = null; },
      replace(next) { validate(next); current = clone(next); baseline = clone(next); history = []; revision++; result = null; },
      beginRun() { return {revision, route: clone(current)}; },
      accept(token, trajectory) { if (token.revision !== revision) return false; result = trajectory; return true; }
    };
  }
  const api = {LIMBS, DEFAULTS, clone, validate, fromDemo, assignments, starter, moveDistance, describeResult, create};
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.ClimbEditorState = api;
})(typeof window !== 'undefined' ? window : globalThis);
