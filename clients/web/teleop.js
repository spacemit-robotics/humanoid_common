/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file teleop.js
 * @brief Bounded operator input with optional client velocity ramps (Web and WebView)
 */
"use strict";
(() => {
    const axes = ["vx", "vy", "wz"];
    const zero = () => ({vx: 0, vy: 0, wz: 0});
    const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
    const finite = (value, fallback) => Number.isFinite(value) ? value : fallback;
    const positive = (value, fallback) => value > 0 && Number.isFinite(value) ? value : fallback;

    function advance(value, target, acceleration, deceleration, dt) {
        // Reversals brake to zero first, then accelerate in the opposite direction.
        if (value * target < 0) {
            const brakingTime = Math.abs(value) / deceleration;
            if (dt <= brakingTime) return value - Math.sign(value) * deceleration * dt;
            dt -= brakingTime;
            value = 0;
        }
        const rate = Math.abs(target) > Math.abs(value) ? acceleration : deceleration;
        const delta = target - value;
        return Math.abs(delta) <= rate * dt ? target : value + Math.sign(delta) * rate * dt;
    }

    class Motion {
        constructor() {
            this.mode = "slow";
            this.value = zero();
            this.configure();
        }
        configure(policy, config = {}) {
            this.config = config;
            this.clientSmoothing = policy?.client_smoothing !== false;
            this.minimum = zero();
            this.maximum = zero();
            this.acceleration = zero();
            this.deceleration = zero();
            for (const axis of axes) {
                this.minimum[axis] = Math.min(0, finite(policy?.minimum?.[axis], 0));
                this.maximum[axis] = Math.max(0, finite(policy?.maximum?.[axis], 0));
                this.acceleration[axis] = positive(config.acceleration?.[axis], axis === "wz" ? 2 : 1);
                this.deceleration[axis] = positive(config.deceleration?.[axis], axis === "wz" ? 4 : 2);
                this.value[axis] = clamp(this.value[axis], this.minimum[axis], this.maximum[axis]);
            }
        }
        limits(axis) {
            const fallback = Math.max(-this.minimum[axis], this.maximum[axis]) * (this.mode === "slow" ? 0.4 : 1);
            const cap = Math.max(0, finite(this.config[`${this.mode}_limits`]?.[axis], fallback));
            return {min: Math.max(this.minimum[axis], -cap), max: Math.min(this.maximum[axis], cap)};
        }
        step(input, dt) {
            dt = clamp(finite(dt, 0), 0, 0.1);
            for (const axis of axes) {
                const normalized = clamp(finite(input[axis], 0), -1, 1);
                const limits = this.limits(axis);
                const target = normalized * (normalized >= 0 ? limits.max : -limits.min);
                this.value[axis] = this.clientSmoothing ? advance(this.value[axis], target,
                    this.acceleration[axis], this.deceleration[axis], dt) : target;
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
    const api = {Motion, advance, stick};
    if (typeof module !== "undefined" && module.exports) module.exports = api;
    else window.OperatorTeleop = api;
})();
