/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file teleop.js
 * @brief Direction limits and spatial joystick input for Web and WebView
 */
"use strict";
(() => {
    const axes = ["vx", "vy", "wz"];
    const zero = () => ({vx: 0, vy: 0, wz: 0});
    const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
    const finite = (value, fallback) => Number.isFinite(value) ? value : fallback;

    class Motion {
        constructor() {
            this.mode = "slow";
            this.value = zero();
            this.configure();
        }
        configure(policy, config = {}) {
            this.config = {...config, ...policy?.teleop};
            this.minimum = zero();
            this.maximum = zero();
            for (const axis of axes) {
                this.minimum[axis] = Math.min(0, finite(policy?.minimum?.[axis], 0));
                this.maximum[axis] = Math.max(0, finite(policy?.maximum?.[axis], 0));
                this.value[axis] = clamp(this.value[axis], this.minimum[axis], this.maximum[axis]);
            }
        }
        limits(axis, mode = this.mode) {
            const fallback = Math.max(-this.minimum[axis], this.maximum[axis]) * (mode === "slow" ? 0.4 : 1);
            const cap = Math.max(0, finite(this.config[`${mode}_limits`]?.[axis], fallback));
            return {min: Math.max(this.minimum[axis], -cap), max: Math.min(this.maximum[axis], cap)};
        }
        get hasSpeedModes() {
            return axes.some(axis => {
                const slow = this.limits(axis, "slow"), fast = this.limits(axis, "fast");
                return slow.min !== fast.min || slow.max !== fast.max;
            });
        }
        step(input) {
            for (const axis of axes) {
                const normalized = clamp(finite(input[axis], 0), -1, 1);
                const limits = this.limits(axis);
                const target = normalized * (normalized >= 0 ? limits.max : -limits.min);
                this.value[axis] = target;
            }
            return {...this.value};
        }
        reset() { this.value = zero(); }
        get moving() { return axes.some(axis => this.value[axis] !== 0); }
    }
    function stick(x, y, deadzone = 0.06) {
        x = finite(x, 0); y = finite(y, 0);
        const length = Math.hypot(x, y);
        const dead = clamp(finite(deadzone, 0.06), 0, 0.49);
        if (length <= dead) return {x: 0, y: 0};
        const scale = (Math.min(length, 1) - dead) / ((1 - dead) * length);
        return {x: x * scale, y: y * scale};
    }
    const api = {Motion, stick};
    if (typeof module !== "undefined" && module.exports) module.exports = api;
    else window.OperatorTeleop = api;
})();
