/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_web.js
 * @brief Offline Web/WebView direction button, ramp and lifecycle regressions
 */
"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const root = path.resolve(__dirname, "../../clients/web");
const {Motion, advance, stick} = require(path.join(root, "teleop.js"));
const close = (actual, expected) => assert(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`);
assert.deepEqual(stick(0.03, 0.03), {x: 0, y: 0});
close(stick(0.53, 0).x, 0.5);
close(stick(-2, 0).x, -1);
close(Math.hypot(...Object.values(stick(1, 1))), 1);
assert.deepEqual(stick(NaN, Infinity), {x: 0, y: 0});
const walk = {name: "walk", minimum: {vx: -0.8, vy: -0.5, wz: -1},
    maximum: {vx: 2, vy: 0.5, wz: 1}, actions: []};
const stand = {name: "stand", minimum: {vx: 0, vy: 0, wz: 0},
    maximum: {vx: 0, vy: 0, wz: 0}, actions: []};
const config = {slow_limits: {vx: 0.6, vy: 0.6, wz: 1}, fast_limits: {vx: 1.5, vy: 0.6, wz: 1},
    acceleration: {vx: 1, vy: 1, wz: 2}, deceleration: {vx: 2.5, vy: 0.5, wz: 2.5}};
const motion = new Motion();
assert.deepEqual(motion.step({vx: 1}, 0.05), {vx: 0, vy: 0, wz: 0}, "no catalog means no motion");
motion.configure(stand, config);
assert.deepEqual(motion.step({vx: 1}, 0.05), {vx: 0, vy: 0, wz: 0});
motion.configure(walk, config);
close(motion.limits("vx").max, 0.6); // Startup/stand must not erase mode presets.
close(motion.step({vx: 1}, 0.05).vx, 0.05);
for (let i = 0; i < 30; ++i) motion.step({vx: 1}, 0.05);
close(motion.value.vx, 0.6);
motion.mode = "fast";
close(motion.step({vx: 1}, 0.05).vx, 0.65);
for (let i = 0; i < 30; ++i) motion.step({vx: 1}, 0.05);
close(motion.value.vx, 1.5);
motion.mode = "slow";
close(motion.step({vx: 1}, 0.05).vx, 1.375); // Mode changes brake; no jump to the new cap.
close(motion.step({}, 0.05).vx, 1.25);
for (let i = 0; i < 30; ++i) motion.step({}, 0.05);
assert.equal(motion.value.vx, 0);
assert(!motion.moving);
close(advance(0.2, -1, 1, 2.5, 0.1), -0.02); // Piecewise reversal across zero.
close(advance(-0.5, 0, 1, 2.5, 0.05), -0.375);
motion.mode = "fast";
for (let i = 0; i < 50; ++i) motion.step({vx: -1, vy: 1, wz: 1}, 0.05);
close(motion.value.vx, -0.8); close(motion.value.vy, 0.5); close(motion.value.wz, 1);
motion.reset();
close(motion.step({vx: 1}, 20).vx, 0.1); // A large dt cannot produce an unlimited step.
motion.reset();

motion.configure({...walk, client_smoothing: false}, config);
motion.mode = "slow";
assert.deepEqual(motion.step({vx: 1, vy: 1, wz: 1}, 0.05), {vx: 0.6, vy: 0.5, wz: 1});
motion.mode = "fast";
close(motion.step({vx: 1}, 0.05).vx, 1.5);
close(motion.step({vx: -1}, 0.05).vx, -0.8); // Reverse target goes to the policy's own smoother.
assert.deepEqual(motion.step({}, 0.05), {vx: 0, vy: 0, wz: 0});
motion.configure({...walk, client_smoothing: false,
    minimum: {...walk.minimum, vx: -0.6}}, config);
for (const mode of ["slow", "fast"]) {
    motion.mode = mode;
    close(motion.step({vx: 1}, 0.05).vx, mode === "fast" ? 1.5 : 0.6);
    close(motion.step({vx: -1}, 0.05).vx, -0.6);
    close(motion.step({vx: -0.5}, 0.05).vx, -0.3);
}
motion.configure({...stand, client_smoothing: false}, config);
assert.deepEqual(motion.step({vx: 1, vy: 1, wz: 1}, 0.05), {vx: 0, vy: 0, wz: 0});
motion.configure(walk, config);
close(motion.step({vx: 1}, 0.05).vx, 0.05); // Switching back restores default client ramps.

// Exercise the actual app lifecycle, without browser/network/robot dependencies.
const elements = new Map(), intervals = new Map(), events = new Map(), captured = [];
let now = 100;
function element(key) {
    if (!elements.has(key)) {
        const attributes = new Map();
        elements.set(key, {value: "", dataset: {}, style: {setProperty() {}},
            classList: {toggle() {}, remove() {}, add() {}},
            querySelector: part => element(`${key} ${part}`), replaceChildren() {},
            setAttribute: (name, value) => attributes.set(name, value), getAttribute: name => attributes.get(name),
            hasPointerCapture: () => false, setPointerCapture() {}, focus() {},
            getBoundingClientRect: () => ({left: 0, top: 0, width: 146, height: 146})});
    }
    return elements.get(key);
}
const context = vm.createContext({
    document: {body: element("body"), getElementById: element, hidden: false,
        querySelector: element, querySelectorAll: selector => {
            if (selector === '[data-motion]') return [...'wsadjl'].map(key => {
                const button = element('direction-' + key); button.dataset.motion = key; return button;
            });
            if (selector === '[data-speed-mode]') return ['slow','fast'].map(mode => {
                const button = element('mode-' + mode); button.dataset.speedMode = mode; return button;
            });
            if (selector === '[data-stick]') return ['move','turn'].map(name => {
                const pad = element('stick-' + name); pad.dataset.stick = name; return pad;
            });
            return [];
        }, addEventListener() {}},
    window: {addEventListener: (name, fn) => events.set(name, fn), dispatchEvent() {}},
    CustomEvent: class { constructor(type, init) { this.type = type; this.detail = init.detail; } },
    navigator: {userAgent: "offline-test"},
    location: {hash: "", pathname: "/"}, history: {replaceState() {}},
    sessionStorage: {getItem() { return ""; }}, URLSearchParams, URL,
    performance: {now: () => now}, setInterval: (fn, ms) => intervals.set(ms, fn),
    fetch: async () => ({ok: false}), Option: function() {}, captured
});
for (const file of ["teleop.js", "app.js"])
    vm.runInContext(fs.readFileSync(path.join(root, file), "utf8"), context);
function run(source) { return vm.runInContext(source, context); }
function render(catalog, policy = "walk") {
    run(`document.body.dataset.mode = 'control'; catalog = ${JSON.stringify(catalog)};
        status = {policy: ${JSON.stringify(policy)}, request: {}, interaction: {},
          online: true, owns_control: true, state: 'RL', age_ms: 0};
        authenticated = true; lastStatusAt = performance.now(); render();`);
}
const catalog = {policies: [stand, walk], teleop: config};
render(undefined); // A delayed catalog must not poison the next render.
render(catalog, "stand");
assert.equal(element("direction-w").disabled, true);
render(catalog);
assert.equal(element("direction-w").disabled, false);
close(run('motion.limits("vx").max'), 0.6);
run("call = async (op, args) => { captured.push({op, args}); return {}; };");
const pointer = (id, x, y) => ({pointerId: id, button: 0, clientX: x, clientY: y, preventDefault() {}});
async function tick() {
    now += 50;
    run("lastStatusAt = performance.now();");
    intervals.get(50)();
    await Promise.resolve(); await Promise.resolve();
}
async function main() {
    close(Number(element("speed-vx").value), 0.6);
    element("speed-vx").value = "0.3";
    element("direction-w").onpointerdown(pointer(10, 73, 23));
    close(run("motionInput().vx"), 0.5);
    element("direction-a").onpointerdown(pointer(11, 23, 73));
    close(run("motionInput().vx"), 0.5);
    close(run("motionInput().vy"), 1); // Each button uses its own speed preset.
    element("direction-a").onpointerup(pointer(11, 23, 73));
    element("speed-vx").value = "5";
    close(run("motionInput().vx"), 1);
    element("speed-vx").value = "invalid";
    close(run("motionInput().vx"), 0);
    element("direction-w").onpointerup(pointer(10, 73, 23));
    element("mode-fast").onclick();
    close(Number(element("speed-vx").value), 1.5);
    element("mode-slow").onclick();
    close(Number(element("speed-vx").value), 0.6);
    render(catalog, "stand");
    close(Number(element("speed-vx").value), 0.6);
    render(catalog);
    close(Number(element("speed-vx").value), 0.6);
    element("direction-w").onpointerdown(pointer(1, 73, 23));
    element("direction-j").onpointerdown(pointer(2, 23, 73));
    await tick();
    close(captured.at(-1).args.vx, 0.05);
    close(captured.at(-1).args.wz, 0.1);
    await tick(); await tick();
    const previous = run("motion.value.vx");
    element("direction-w").onpointerup(pointer(1, 73, 23));
    close(run("motion.value.vx"), previous); // Release sets target, not command, to zero.
    await tick();
    assert(run("motion.value.vx") < previous);
    assert(run("motion.value.wz") > 0); // Other finger remains effective.
    element("direction-j").onpointerup(pointer(2, 23, 73));
    for (let i = 0; i < 10; ++i) await tick();
    close(captured.at(-1).args.vx, 0); close(captured.at(-1).args.wz, 0);
    element("direction-w").onpointerdown(pointer(3, 73, 23));
    await tick();
    events.get("blur")();
    close(run("motion.value.vx"), 0);
    close(captured.at(-1).args.vx, 0);
    assert.equal(run("pointers.size"), 0);
    element("direction-w").onpointerdown(pointer(4, 73, 23));
    await tick();
    const beforeStall = run("motion.value.vx");
    now += 500; intervals.get(50)();
    assert(run("motion.value.vx") >= beforeStall);
    assert(run("motion.value.vx") <= beforeStall + 0.1);
    assert.equal(run("pointers.size"), 1, "a delayed refresh preserves the held button");
    events.get("resize")?.();
    assert.equal(run("pointers.size"), 1, "window resizing must not cancel a held button");
    element("direction-w").onpointerup(pointer(4, 73, 23));
    element("direction-w").onpointerdown(pointer(5, 73, 23));
    await tick();
    events.get("keydown")({code: "Space", key: " ", target: {tagName: "BODY"}, preventDefault() {}});
    assert.equal(captured.at(-1).op, "stop");
    close(run("motion.value.vx"), 0);
    assert.equal(run("pointers.size"), 0);
    render({...catalog, policies: [stand, {...walk, client_smoothing: false}]});
    element("direction-w").onpointerdown(pointer(7, 73, 23));
    await tick();
    close(captured.at(-1).args.vx, 0.6);
    element("direction-w").onpointerup(pointer(7, 73, 23));
    await tick();
    close(captured.at(-1).args.vx, 0);
    run("endMotion(); inputMode = 'sticks';");
    const half = 73 - 146 * 0.34 * 0.53;
    element('stick-move').onpointerdown(pointer(20, 73, half));
    element('stick-turn').onpointerdown(pointer(21, half, 73));
    await tick();
    close(captured.at(-1).args.vx, 0.3); close(captured.at(-1).args.wz, 0.5);
    element('stick-move').onpointerup(pointer(20, 73, half));
    await tick();
    close(captured.at(-1).args.vx, 0); close(captured.at(-1).args.wz, 0.5);
    element('stick-turn').onpointercancel(pointer(21, half, 73));
    await tick();
    close(captured.at(-1).args.wz, 0);
    run("endMotion(); inputMode = 'buttons';");
    render(catalog);
    element("direction-w").onpointerdown(pointer(8, 73, 23));
    await tick();
    close(captured.at(-1).args.vx, 0.05);
    events.get("blur")();
    run("call = () => new Promise(() => {});");
    element("direction-w").onpointerdown(pointer(6, 73, 23));
    await tick();
    for (let i = 0; i < 22; ++i) await tick();
    close(run("motion.value.vx"), 0.6);
    assert.equal(run("pointers.size"), 1, "missing velocity replies must not cancel current input");
    console.log("Web direction button regressions passed: presets, independent axes, ramps, multitouch, stop, delayed refresh and replies");
}
main().catch(error => { console.error(error); process.exitCode = 1; });
