/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_web_connection.js
 * @brief Offline connection recovery tests using the production web application
 */
"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const root = path.resolve(__dirname, "../../clients/web");
const catalog = {robot: "offline-test", policies: [{name: "walk", actions: [],
    minimum: {vx: -1, vy: -1, wz: -1}, maximum: {vx: 1, vy: 1, wz: 1}}]};
const initialStatus = {state: "RL", policy: "walk", online: true, owns_control: false,
    owner: "", age_ms: 0, request: {}, interaction: {}, velocity: {vx: 0, vy: 0, wz: 0}};
async function flush() { for (let i = 0; i < 8; ++i) await Promise.resolve(); }

function harness() {
    let now = 100, timerId = 0;
    const timers = new Map(), elements = new Map(), events = new Map(), documentEvents = new Map(), sockets = [];
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
    class Socket {
        static CONNECTING = 0;
        static OPEN = 1;
        readyState = 0;
        bufferedAmount = 0;
        sent = [];
        sentAt = [];
        constructor() { sockets.push(this); }
        send(text) {
            assert.equal(this.readyState, Socket.OPEN);
            const request = JSON.parse(text);
            this.sent.push(request); this.sentAt.push(now);
            if (this.ackDelay !== undefined && !this.dropReplies?.has(request.op))
                addTimer(() => this.reply(request), request.op === "velocity" ? this.ackDelay : 0, false);
        }
        close() { this.readyState = 3; this.onclose?.({code: 1000}); }
        receive(packet) { this.onmessage({data: JSON.stringify(packet)}); }
        reply(request, data = {}, code = "ok") {
            this.receive({v: 1, id: request.id, ok: code === "ok", code, message: "", data});
        }
    }
    const addTimer = (fn, ms, repeat) => {
        const id = ++timerId;
        timers.set(id, {fn, due: now + ms, repeat: repeat ? ms : 0});
        return id;
    };
    const context = vm.createContext({
        document: {body: element("body"), getElementById: element, hidden: false,
            querySelector: element, querySelectorAll: selector => {
            if (selector === '[data-motion]') return [...'wsadjl'].map(key => {
                const button = element('direction-' + key); button.dataset.motion = key; return button;
            });
            if (selector === '[data-speed-mode]') return ['slow','fast'].map(mode => {
                const button = element('mode-' + mode); button.dataset.speedMode = mode; return button;
            });
            return [];
        },
            addEventListener: (name, fn) => documentEvents.set(name, fn)},
        window: {addEventListener: (name, fn) => events.set(name, fn), dispatchEvent() {}},
        CustomEvent: class { constructor(type, init) { this.type = type; this.detail = init.detail; } },
        navigator: {userAgent: "offline-test"},
        location: {hash: "#token=offline-test", pathname: "/", protocol: "http:", host: "offline.invalid"},
        history: {replaceState() {}}, sessionStorage: {getItem() { return ""; }, setItem() {}},
        URLSearchParams, URL, WebSocket: Socket, performance: {now: () => now},
        setTimeout: (fn, ms) => addTimer(fn, ms, false), clearTimeout: id => timers.delete(id),
        setInterval: (fn, ms) => addTimer(fn, ms, true),
        fetch: async () => ({ok: false}), Option: function() {}
    });
    for (const file of ["teleop.js", "app.js"])
        vm.runInContext(fs.readFileSync(path.join(root, file), "utf8"), context);
    const run = source => vm.runInContext(source, context);
    async function advance(ms) {
        const end = now + ms;
        while (true) {
            const next = [...timers].filter(([, t]) => t.due <= end).sort((a, b) => a[1].due - b[1].due)[0];
            if (!next) break;
            const [id, timer] = next;
            now = timer.due;
            if (timer.repeat) timer.due += timer.repeat;
            else timers.delete(id);
            timer.fn(); await flush();
        }
        now = end; await flush();
    }
    async function ready(socket = sockets.at(-1), code = "ok") {
        socket.readyState = Socket.OPEN; socket.onopen();
        socket.reply(socket.sent[0], {catalog, status: initialStatus}, code);
        await flush();
        return socket;
    }
    function status(changes = {}) {
        sockets.at(-1).receive({v: 1, event: "status", data: {...initialStatus, ...changes}});
    }
    function push() {
        element("direction-w").onpointerdown({pointerId: 1, button: 0, clientX: 73, clientY: 23, preventDefault() {}});
    }
    async function stall(ms) {
        now += ms;
        for (const timer of timers.values()) timer.due = Math.max(timer.due, now);
        await advance(0);
    }
    return {run, advance, stall, ready, sockets, status, push, element, events, documentEvents};
}

async function main() {
    {
        const h = harness(), socket = await h.ready();
        const initialRequests = socket.sent.length;
        assert.equal(h.element("owner").textContent, "无人持有");
        for (const [owner, label] of [["gamepad#7", "手柄"], ["web#8", "其他网页"],
                                     ["android#9", "手机 App"], ["terminal#10", "终端"]]) {
            h.status({owner});
            assert.equal(h.element("owner").textContent, `${label}已接管控制权`);
            assert.equal(h.element("motion-state").textContent, `${label}已接管控制权`);
            assert.equal(h.element("acquire").disabled, true);
            assert.equal(h.element("release").disabled, true);
        }
        h.status({owner: "gamepad#7", online: false});
        assert.equal(h.element("owner").textContent, "等待状态确认");
        h.status({owner: "gamepad#7"});
        await h.advance(1050);
        h.run("render()");
        assert.equal(h.element("owner").textContent, "等待状态确认", "stale status is not current ownership");
        h.status({owner: "<custom>#11"});
        assert.equal(h.element("owner").textContent, "<custom>#11");
        h.status({owner: "web#12", owns_control: true});
        assert.equal(h.element("owner").textContent, "本客户端已接管");
        h.status({owner: ""});
        assert.equal(h.element("owner").textContent, "无人持有");
        assert.equal(h.element("acquire").disabled, false);
        assert.equal(h.element("motion-state").textContent, "等待接管");
        assert.equal(socket.sent.length, initialRequests, "ownership display must not send commands");
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true});
        const presets = () => socket.sent.filter(r => r.op === "client_event" && r.args.event === "teleop_preset");
        assert.equal(presets().length, 0, "connection/acquire do not announce a default speed");
        h.element("mode-fast").onclick();
        assert.equal(presets().length, 1);
        assert.equal(presets()[0].args.detail, "fast");
        h.element("mode-fast").onclick();
        assert.equal(presets().length, 1, "clicking the current preset is silent");
        h.element("mode-slow").onclick();
        assert.equal(presets().length, 2);
        h.status({owns_control: false});
        h.element("mode-fast").onclick();
        assert.equal(presets().length, 2, "spectators cannot announce robot speed changes");
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true});
        h.run(`catalog.policies[0].display_name = '走跑运控'; render();`);
        assert.equal(h.element("active-policy").textContent, "走跑运控");
        assert.equal(h.element("select-policy").disabled, true, "legacy policies cannot switch in RL");
        h.run(`catalog.policies[0].switch_group = 'locomotion';
            catalog.policies.push({...catalog.policies[0], name:'peer'});
            catalog.policies.push({...catalog.policies[0], name:'other', switch_group:''}); render();`);
        h.element("policy").value = "other"; h.element("policy").onchange();
        assert.equal(h.element("select-policy").disabled, true, "different groups remain locked");
        h.element("policy").value = "peer"; h.element("policy").onchange();
        assert.equal(h.element("select-policy").disabled, false, "compatible RL handoff is selectable");
        h.push(); await h.advance(50);
        h.element("select-policy").onclick();
        assert.equal(h.run("motion.value.vx"), 0, "policy selection clears the old held speed");
        assert(socket.sent.some(r => r.op === "policy" && r.args.policy === "peer"));
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true}); h.push(); await h.advance(50);
        h.run('render = () => { throw new Error("test render failure"); }');
        h.status({owns_control: true});
        assert.equal(socket.readyState, 1, "a rendering error must not close the connection");
        assert.equal(h.run("authenticated"), true);
        assert.equal(h.run("motion.value.vx"), 0, "stop old input after a rendering failure");
        assert(socket.sent.some(r => r.op === "client_event" && r.args.event === "ui_error"));
        assert(!socket.sent.some(r => r.op === "release"));
    }
    for (const delay of [20, 80, 180, 280]) {
        const h = harness(), socket = await h.ready();
        socket.ackDelay = delay;
        h.run(`catalog.policies[0].client_smoothing = false;
            catalog.teleop = {slow_limits:{vx:.6,vy:.6,wz:1}}; render();`);
        h.element("speed-vx").value = "0.6";
        h.status({owns_control: true}); h.push(); await h.advance(50);
        assert.equal(socket.sent.filter(r => r.op === "velocity").at(-1).args.vx, 0.6);
        h.element("direction-w").onpointerup({pointerId:1, preventDefault() {}});
        await h.advance(50);
        assert.equal(socket.sent.filter(r => r.op === "velocity").at(-1).args.vx, 0);
        assert.equal(socket.sent.filter(r => r.op === "velocity").length, 2);
        console.log(`Policy-side smoothing, ACK ${delay} ms: target and release each sent on the next 50 ms tick`);
    }
    for (const delay of [20, 80, 180, 280]) {
        const h = harness(), socket = await h.ready();
        socket.ackDelay = delay;
        h.run(`catalog.teleop = {slow_limits:{vx:.6,vy:.6,wz:1},
            acceleration:{vx:1,vy:1,wz:2}, deceleration:{vx:2.5,vy:.5,wz:2.5}}; render();`);
        h.element("speed-vx").value = "0.6";
        h.status({owns_control: true}); h.push();
        for (let i = 0; i < 12; ++i) {
            h.status({owns_control: true}); await h.advance(50);
        }
        assert(Math.abs(h.run("motion.value.vx") - .6) < 1e-9, `${delay} ms replies must not slow acceleration`);
        h.element("direction-w").onpointerup({pointerId:1, preventDefault() {}});
        for (let i = 0; i < 5; ++i) {
            h.status({owns_control: true}); await h.advance(50);
        }
        assert.equal(h.run("motion.value.vx"), 0, `${delay} ms replies must not slow braking`);
        const velocities = socket.sent.filter(r => r.op === "velocity");
        assert.equal(velocities.length, 17);
        assert.equal(velocities.at(-1).args.vx, 0, "final zero is sent before older replies arrive");
        const times = socket.sentAt.filter((_, i) => socket.sent[i].op === "velocity");
        assert(times.slice(1).every((time, i) => time - times[i] === 50), "send cadence is independent of RTT");
        await h.advance(350);
        assert.equal(h.run("pending.size"), 0);
        console.log(`ACK ${delay} ms: 0->0.6 in 600 ms, release in 250 ms, velocity stream 20 Hz`);
    }
    {
        const h = harness(), socket = await h.ready();
        socket.bufferedAmount = 1024;
        h.status({owns_control: true}); h.push(); await h.advance(500);
        assert(h.run("motion.value.vx") > 0, "transport backlog must not freeze smoothing");
        h.element("direction-w").onpointerup({pointerId:1, preventDefault() {}});
        await h.advance(250);
        assert.equal(h.run("motion.value.vx"), 0);
        assert(!socket.sent.some(r => r.op === "velocity"), "do not queue intermediate velocities");
        socket.bufferedAmount = 0;
        await h.advance(50);
        const velocities = socket.sent.filter(r => r.op === "velocity");
        assert.equal(velocities.length, 1);
        assert.equal(velocities[0].args.vx, 0, "retain final zero while the transport is busy");
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true}); h.push(); await h.advance(50);
        const previous = socket.sent.find(r => r.op === "velocity");
        h.events.get("blur")(); h.push(); await h.advance(50);
        socket.reply(previous, {}, "not_ready"); await flush();
        assert(h.run("motion.value.vx") > 0, "a stale rejection must not clear a new gesture");
    }
    for (const delay of [1500, 2500, Infinity]) {
        const h = harness(), socket = await h.ready();
        socket.ackDelay = Number.isFinite(delay) ? delay : 0;
        if (!Number.isFinite(delay)) socket.dropReplies = new Set(["velocity", "renew"]);
        h.status({owns_control: true}); h.push();
        for (let i = 0; i < 80; ++i) {
            h.status({owns_control: true}); await h.advance(50);
        }
        assert.equal(h.run("motion.value.vx"), 0.4);
        assert.equal(h.run("pointers.size"), 1);
        assert(h.run("pending.size") <= 50, "timed-out request records are discarded without pausing input");
        assert.equal(socket.readyState, 1);
        h.element("direction-w").onpointerup({pointerId:1, preventDefault() {}});
        for (let i = 0; i < 8; ++i) {
            h.status({owns_control: true}); await h.advance(50);
        }
        assert.equal(socket.sent.filter(r => r.op === "velocity").at(-1).args.vx, 0);
    }
    {
        const h = harness(), socket = await h.ready();
        socket.ackDelay = 20;
        h.status({owns_control: true}); h.push(); await h.advance(100);
        const before = h.run("motion.value.vx");
        await h.stall(500);
        assert.equal(h.run("pointers.size"), 1);
        assert(h.run("motion.value.vx") >= before);
        assert(h.run("motion.value.vx") <= before + 0.1);
        h.events.get("resize")?.();
        assert.equal(h.run("pointers.size"), 1);
        assert.equal(socket.readyState, 1);
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true}); h.push(); await h.advance(50);
        socket.reply(socket.sent.find(r => r.op === "velocity"), {}, "not_ready");
        await flush();
        assert.equal(h.run("motion.value.vx"), 0, "an explicit rejection still clears rejected input");
        assert.equal(h.run("pointers.size"), 0);
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true}); h.push();
        await h.advance(50);
        const velocity = socket.sent.find(r => r.op === "velocity");
        assert(velocity.args.vx > 0);
        await h.advance(300);
        assert.equal(socket.readyState, 1, "300 ms acknowledgement delay must not close the connection");
        socket.reply(velocity);
        await flush();
        h.status({owns_control: true}); await h.advance(50);
        assert(h.run("motion.value.vx") > 0);
        assert(!socket.sent.some(r => r.op === "release"));
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true}); h.push(); await h.advance(50);
        await h.advance(1250);
        assert.equal(socket.readyState, 1, "a brief status gap pauses input without closing the page");
        assert.equal(h.run("motion.value.vx"), 0);
        assert.equal(h.run("pointers.size"), 0);
        h.status(); await h.advance(100);
        assert.equal(h.sockets.length, 1);
        assert.equal(h.run("motion.value.vx"), 0, "status recovery does not revive the old gesture");
    }
    {
        const h = harness(), old = await h.ready();
        h.status({owns_control: true}); h.push(); await h.advance(50);
        old.close(); await h.advance(500);
        const fresh = await h.ready();
        assert.equal(h.sockets.length, 2);
        old.onclose(); old.receive({v: 1, event: "status", data: {...initialStatus, owns_control: true}});
        await h.advance(100);
        assert.equal(h.run("authenticated"), true);
        assert.equal(h.run("status.owns_control"), false);
        assert.equal(h.run("motion.value.vx"), 0);
        assert(fresh.sent.every(r => r.op === "hello"), "reconnection must not replay commands or reacquire control");
    }
    {
        const h = harness();
        for (const delay of [500, 1000, 2000, 4000, 5000, 5000]) {
            h.sockets.at(-1).close();
            const before = h.sockets.length;
            await h.advance(delay - 1); assert.equal(h.sockets.length, before);
            await h.advance(1); assert.equal(h.sockets.length, before + 1);
        }
        await h.ready(); assert.equal(h.run("authenticated"), true);
    }
    {
        const h = harness();
        await h.advance(5500);
        assert.equal(h.sockets.length, 2, "a stuck connection handshake is retried");
        const socket = await h.ready();
        await h.advance(6000);
        assert.equal(socket.readyState, 3, "a silent half-open connection is eventually replaced");
        assert(h.sockets.length > 2);
    }
    {
        const h = harness(), socket = await h.ready();
        h.status({owns_control: true});
        h.run("document.hidden = true"); h.documentEvents.get("visibilitychange")();
        assert(socket.sent.some(r => r.op === "client_event" && r.args.event === "page_hidden"));
        assert(socket.sent.some(r => r.op === "release"));
        assert.equal(socket.readyState, 1, "backgrounding does not itself tear down a healthy web connection");
        socket.close(); await h.advance(30000); assert.equal(h.sockets.length, 1);
        h.run("document.hidden = false"); h.documentEvents.get("visibilitychange")();
        await h.ready(); assert.equal(h.sockets.length, 2);
        assert.equal(h.run("status.owns_control"), false);
    }
    {
        const h = harness(); await h.ready();
        h.run("window.operatorDisconnect()");
        assert(h.sockets[0].sent.some(r => r.op === "client_event" && r.args.event === "manual_disconnect"));
        h.events.get("online")(); h.events.get("pageshow")();
        await h.advance(30000);
        assert.equal(h.sockets.length, 1, "explicit disconnect suppresses automatic retries");
        // The existing Android onResume bridge also uses this entry point.
        h.run("window.operatorReconnect()"); await h.ready();
        assert.equal(h.sockets.length, 2);
    }
    {
        const h = harness(); await h.ready(h.sockets[0], "unauthorized");
        await h.advance(30000);
        assert.equal(h.sockets.length, 1, "invalid pairing must not cause an authentication retry loop");
        assert.equal(h.element("login").hidden, false);
    }
    console.log("Web connection regressions passed: latency, stale status, retry/backoff, foreground, manual disconnect, auth and no command replay");
}
main().catch(error => { console.error(error); process.exitCode = 1; });
