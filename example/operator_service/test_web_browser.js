/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_web_browser.js
 * @brief Local-only Chromium touch regressions with synthetic operator status
 */
"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs/promises");
const http = require("node:http");
const path = require("node:path");
const {chromium} = require("playwright");
const root = path.resolve(process.env.OPERATOR_WEB_TEST_ROOT ||
    path.join(__dirname, "../../build/operator_web"));
const output = process.env.OPERATOR_WEB_TEST_OUTPUT;
const assets = new Map([
    ["/", ["index.html", "text/html"]], ["/app.js", ["app.js", "text/javascript"]],
    ["/teleop.js", ["teleop.js", "text/javascript"]], ["/style.css", ["style.css", "text/css"]],
    ["/voice/panel.js", ["voice/panel.js", "text/javascript"]],
    ["/voice/panel.css", ["voice/panel.css", "text/css"]],
    ["/vendor/lucide.min.js", ["vendor/lucide.min.js", "text/javascript"]],
    ["/assets/spacemit-logo.svg", ["assets/spacemit-logo.svg", "image/svg+xml"]]
]);
const server = http.createServer(async (req, res) => {
    const asset = assets.get(req.url);
    if (!asset) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, {"Content-Type": asset[1], "Cache-Control": "no-store",
        "Content-Security-Policy": "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'"});
    res.end(await fs.readFile(path.join(root, asset[0])));
});
const catalog = {robot: "Robot · 离线界面测试", policies: [
    {name: "walk", minimum: {vx: -2, vy: -0.6, wz: -1}, maximum: {vx: 2, vy: 0.6, wz: 1}, actions: []}
], teleop: {slow_limits: {vx: 0.6, vy: 0.6, wz: 1},
    fast_limits: {vx: 1.5, vy: 0.6, wz: 1}, acceleration: {vx: 1, vy: 1, wz: 2},
    deceleration: {vx: 2.5, vy: 0.5, wz: 2.5}}};

async function exercise(browser, name, width, app, direct = false) {
    const clientCatalog = {...catalog,
        policies: catalog.policies.map(policy => ({...policy, client_smoothing: !direct}))};
    const context = await browser.newContext({viewport: {width, height: 844},
        hasTouch: true, isMobile: true, ...(app ? {userAgent: "Android SpaceMITOperatorApp"} : {})});
    const page = await context.newPage();
    const errors = [], sent = [];
    page.on("pageerror", error => errors.push(error.message));
    page.on("console", message => { if (message.text().includes("Content Security Policy")) errors.push(message.text()); });
    let socket, emit = () => {}, acknowledgementDelay = 0, publish = true, connections = 0;
    const statusTimers = new Set();
    const status = {state: "RL", policy: "walk", online: true, owns_control: false, owner: "",
        age_ms: 0, rl_hz: 50, zero_ready: true, request: {}, interaction: {}, velocity: {vx: 0, vy: 0, wz: 0}};
    await page.routeWebSocket("**/api/v1/ws", route => {
        socket = route; connections++;
        const replies = new Set();
        emit = () => { if (publish) route.send(JSON.stringify({v: 1, event: "status", data: status})); };
        route.onMessage(message => {
            const r = JSON.parse(message);
            sent.push(r);
            if (r.op === "acquire") { status.owns_control = true; status.owner = name; }
            if (r.op === "velocity") status.velocity = r.args;
            if (["release", "stop"].includes(r.op)) {
                status.owns_control = false; status.owner = ""; status.velocity = {vx: 0, vy: 0, wz: 0};
            }
            const reply = JSON.stringify({v: 1, id: r.id, ok: true, code: "ok", message: "",
                data: r.op === "hello" ? {catalog: clientCatalog, status} : {}});
            if (r.op === "velocity" && acknowledgementDelay) {
                const timer = setTimeout(() => { replies.delete(timer); route.send(reply); }, acknowledgementDelay);
                replies.add(timer);
            } else route.send(reply);
            emit();
        });
        const statusTimer = setInterval(emit, 50);
        statusTimers.add(statusTimer);
        route.onClose(() => {
            clearInterval(statusTimer); statusTimers.delete(statusTimer);
            for (const timer of replies) clearTimeout(timer);
        });
    });
    const waitVelocity = async (axis, value) => {
        await page.waitForFunction(({axis, value}) => {
            const axes = {vx: 0, vy: 1, wz: 2};
            return Math.abs(Number(document.getElementById("command-velocity").textContent.split(" / ")[axes[axis]]) - value) < 0.015;
        }, {axis, value});
    };
    try {
        await page.goto(`http://127.0.0.1:${server.address().port}/#token=offline-test`);
        await page.waitForFunction(() => document.querySelectorAll("svg.lucide").length > 0);
        await page.waitForFunction(() => document.getElementById("connection").textContent === "控制链路在线");
        assert(!await page.locator("#admin-exit").isVisible());
        status.owner = "gamepad#7"; emit();
        await page.waitForFunction(() => document.getElementById("owner").textContent === "手柄已接管控制权");
        assert(await page.locator("#acquire").isDisabled());
        await page.locator('[data-view="motion"]').click();
        await page.setViewportSize({width: 844, height: width});
        assert(await page.locator("#motion-state").isVisible());
        assert.equal(await page.locator("#motion-state").textContent(), "手柄已接管控制权");
        assert(await page.locator("#direction-w").isDisabled());
        assert(!sent.some(r => r.op === "acquire"), "spectator page does not acquire automatically");
        await page.setViewportSize({width, height: 844});
        await page.locator('[data-view="control"]').click();
        status.owner = ""; emit();
        await page.waitForFunction(() => document.getElementById("owner").textContent === "无人持有");
        await page.locator("#acquire").click();
        await page.locator('[data-view="motion"]').click();
        await page.waitForFunction(() => !document.getElementById("direction-w").disabled);
        assert.equal(sent.find(r => r.op === "hello").args.name, app ? "android" : "web");
        assert(await page.locator("#speed-limits").textContent().then(text => text.includes("0.60")));
        assert.equal(await page.locator(".speed-settings, .direction-pad").count(), 2);
        assert.equal(await page.locator(".joystick").count(), 2);
        assert(!await page.locator(".dual-sticks").isVisible());
        assert.equal(await page.locator("#stop, #stop-label, .stop-bar").count(), 0);
        assert.equal(await page.getByRole("button", {name: /^停止(移动|动作)$/}).count(), 0);
        assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        await page.locator("#speed-vx").fill("0.3");
        await page.locator("#speed-wz").fill("0.5");
        await page.locator(".direction-pad").scrollIntoViewIfNeeded();
        let left = await page.locator("#direction-w").boundingBox();
        const right = await page.locator("#direction-j").boundingBox();
        const touchPoint = (box, id) => ({id, x: box.x + box.width / 2, y: box.y + box.height / 2});
        const cdp = await context.newCDPSession(page);
        const touch = (type, points) => cdp.send("Input.dispatchTouchEvent", {type, touchPoints: points});
        await touch("touchStart", [touchPoint(left, 1), touchPoint(right, 2)]);
        await waitVelocity("vx", 0.3); await waitVelocity("wz", 0.5);
        assert(sent.some(r => r.op === "velocity" && r.args.vx > 0.2 && r.args.wz > 0.2));
        const releaseAt = sent.length;
        await touch("touchEnd", []);
        await waitVelocity("vx", 0); await waitVelocity("wz", 0);
        const release = sent.slice(releaseAt).filter(r => r.op === "velocity");
        if (direct) {
            assert(release.every(r => r.args.vx === 0.3 || r.args.vx === 0), "release must not add a client ramp");
            assert.equal(release.at(-1).args.vx, 0);
        } else {
            assert(release.some(r => r.args.vx > 0 && r.args.vx < 0.29), "release must ramp down");
        }
        await page.locator('[data-speed-mode="fast"]').click();
        left = await page.locator("#direction-w").boundingBox();
        await touch("touchStart", [touchPoint(left, 3)]);
        await waitVelocity("vx", 1.5);
        await page.locator('[data-speed-mode="slow"]').click();
        await waitVelocity("vx", 0.6);
        const reverseAt = sent.length;
        await touch("touchEnd", []);
        await page.locator('#direction-s').evaluate(button => button.scrollIntoView({block: "center"}));
        const back = await page.locator("#direction-s").boundingBox();
        await touch("touchStart", [touchPoint(back, 3)]);
        await waitVelocity("vx", -0.6);
        const reversal = sent.slice(reverseAt).filter(r => r.op === "velocity").map(r => r.args.vx);
        if (direct) assert(reversal.every(v => [0.6, 0, -0.6].includes(v)), "reversal is smoothed by the policy");
        else assert(reversal.some(v => v > 0 && v < 0.5) && reversal.some(v => v < 0 && v > -0.3));
        await page.keyboard.press("Space");
        await waitVelocity("vx", 0);
        await touch("touchEnd", []);
        assert(sent.some(r => r.op === "stop"));
        await page.locator("#acquire").click();
        await page.waitForFunction(() => !document.getElementById("direction-w").disabled);
        await page.locator('#direction-w').evaluate(button => button.scrollIntoView({block: "center"}));
        left = await page.locator("#direction-w").boundingBox();
        await touch("touchStart", [touchPoint(left, 4)]);
        await waitVelocity("vx", 0.6);
        status.fault = {source: "test", code: "offline_fault", latched: true}; emit();
        await waitVelocity("vx", 0);
        await touch("touchEnd", []);
        assert(await page.locator("#direction-w").isDisabled());
        delete status.fault; emit();
        // No previous finger gesture resumes when the fault indication disappears.
        await page.waitForTimeout(200);
        await waitVelocity("vx", 0);
        assert.equal(status.velocity.vx, 0, "fault also clears the outstanding command at the service");
        await page.locator("#acquire").click();
        await page.waitForFunction(() => !document.getElementById("direction-w").disabled);
        const originalConnections = connections;
        acknowledgementDelay = 2500;
        await touch("touchStart", [touchPoint(left, 5)]);
        await page.waitForTimeout(3000);
        assert.equal(connections, originalConnections);
        assert.equal(await page.locator("#connection").textContent(), "控制链路在线");
        assert.equal(Number((await page.locator("#command-velocity").textContent()).split(" / ")[0]), 0.6);
        await page.evaluate(() => {
            const until = performance.now() + 500;
            while (performance.now() < until) { /* Simulate a delayed browser refresh. */ }
            window.dispatchEvent(new Event("resize"));
        });
        await page.waitForTimeout(100);
        assert.equal(Number((await page.locator("#command-velocity").textContent()).split(" / ")[0]), 0.6);
        assert(!await page.locator("#message").textContent().then(text => text.includes("遥控刷新中断")));
        await touch("touchEnd", []);
        await page.waitForTimeout(350);
        assert.equal(Number((await page.locator("#command-velocity").textContent()).split(" / ")[0]), 0);
        assert.equal(sent.filter(r => r.op === "velocity").at(-1).args.vx, 0);
        acknowledgementDelay = 0;
        publish = false;
        await page.waitForTimeout(1400);
        assert.equal(connections, originalConnections);
        assert.equal(await page.locator("#connection").textContent(), "网络延迟，等待恢复");
        status.owns_control = false; status.owner = "";
        publish = true; emit();
        await page.waitForFunction(() => !document.getElementById("acquire").disabled);
        const disconnectedAt = sent.length;
        await socket.close();
        await page.waitForFunction(() => document.getElementById("connection").textContent === "控制链路在线");
        assert.equal(connections, originalConnections + 1);
        assert(!sent.slice(disconnectedAt).some(r => ["acquire", "velocity", "state", "interaction"].includes(r.op)),
            "automatic reconnect must not replay robot commands");
        await page.locator("#acquire").click();
        await page.waitForFunction(() => !document.getElementById("direction-w").disabled);
        if (output) {
            await fs.mkdir(output, {recursive: true});
            await page.screenshot({path: path.join(output, `${name}.png`), fullPage: true});
        }
        for (const viewport of [{width: 320, height: 844}, {width: 1200, height: 900}, {width: 844, height: 390}]) {
            await page.setViewportSize(viewport);
            await page.evaluate(() => scrollTo(0, 0));
            assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
            const box = await page.locator(".direction-pad").boundingBox();
            const footer = await page.locator(".command-bar").boundingBox();
            assert(box && box.width > 80);
            if (viewport.height === 390) assert(box.y + box.height <= footer.y, "landscape buttons must stay above status bar");
            if (output) await page.screenshot({path: path.join(output, `${name}-${viewport.width}.png`), fullPage: true});
        }
        await page.locator('button[data-input-mode="sticks"]').click();
        assert(!await page.locator('.direction-pad').isVisible());
        assert(!await page.locator('.speed-settings').isVisible());
        const move = await page.locator('[data-stick="move"]').boundingBox();
        const turn = await page.locator('[data-stick="turn"]').boundingBox();
        const footer = await page.locator('.command-bar').boundingBox();
        assert(move.y + move.height <= footer.y && turn.y + turn.height <= footer.y,
            'both landscape sticks must stay above status bar');
        assert(move.x + move.width < turn.x, 'left and right thumbs have separate controls');
        const moveTouch = {...touchPoint(move, 20), y: move.y + move.height * (0.5 - 0.34 * 0.53)};
        const turnTouch = {...touchPoint(turn, 21), x: turn.x + turn.width * (0.5 - 0.34 * 0.53)};
        acknowledgementDelay = 2500;
        const sticksAt = sent.length;
        await touch('touchStart', [moveTouch, turnTouch]);
        await waitVelocity('vx', 0.3); await waitVelocity('wz', 0.5);
        if (direct) {
            const first = sent.slice(sticksAt).find(r => r.op === 'velocity' && r.args.vx > 0);
            assert(Math.abs(first.args.vx - 0.3) < 0.015, 'stick target is not ramped in the webpage');
        }
        await page.waitForTimeout(2600);
        await waitVelocity('vx', 0.3); await waitVelocity('wz', 0.5);
        assert.equal(connections, originalConnections + 1, 'slow stick ACKs do not reconnect');
        if (output) await page.screenshot({path: path.join(output, `${name}-dual-sticks.png`), fullPage: true});
        await touch('touchEnd', [moveTouch]);
        await waitVelocity('vx', 0); await waitVelocity('wz', 0.5);
        await touch('touchEnd', []);
        await waitVelocity('wz', 0);
        acknowledgementDelay = 0;
        // Full left strafe; changing the view clears the held stick instead of replaying it.
        await touch('touchStart', [{...touchPoint(move, 22), x: move.x + move.width * 0.16}]);
        await waitVelocity('vy', 0.6);
        await page.locator('button[data-input-mode="buttons"]').click();
        await waitVelocity('vy', 0);
        await touch('touchEnd', []);
        assert(await page.locator('.direction-pad').isVisible());
        assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
        await page.evaluate(() => window.dispatchEvent(new Event("pagehide")));
        await page.waitForTimeout(100);
        assert(sent.some(r => r.op === "release"));
        assert.deepEqual(errors, []);
        console.log(`${name}: buttons and dual sticks, independent fingers, immediate policy targets, 2500 ms replies, layout, fault and reconnect passed`);
    } catch (error) {
        if (output) {
            await fs.mkdir(output, {recursive: true});
            await page.screenshot({path: path.join(output, `${name}-failure.png`), fullPage: true});
        }
        console.error('browser test state', await page.evaluate(() => ({
            scrollY, height: innerHeight, text: document.getElementById('command-velocity').textContent,
            motion: document.getElementById('motion-state').textContent,
            back: document.getElementById('direction-s').getBoundingClientRect().toJSON()
        })), errors);
        throw error;
    } finally {
        for (const timer of statusTimers) clearInterval(timer);
        await socket?.close();
        await context.close();
    }
}

async function main() {
    await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
    const browser = await chromium.launch({headless: true, executablePath: process.env.OPERATOR_BROWSER});
    try {
        await exercise(browser, "mobile_web", 390, false);
        await exercise(browser, "android_webview_page", 360, true);
        await exercise(browser, "mobile_web_policy_smoothing", 390, false, true);
        await exercise(browser, "android_webview_policy_smoothing", 360, true, true);
    } finally { await browser.close(); }
}
main().catch(error => { console.error(error); process.exitCode = 1; }).finally(() => server.close());
