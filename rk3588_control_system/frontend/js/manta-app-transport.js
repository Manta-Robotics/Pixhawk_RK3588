(function (window) {
    "use strict";

    var DEFAULT_DEVICE = { name: "MANTA ROBOTIC", model: "MANTA", serial: "2407" };
    var MOCK_DEVICES = [
        { name: "MANTA ROBOTIC", model: "MANTA", serial: "2407", rssi: -43 },
        { name: "MANTA ROBOTIC", model: "MANTA", serial: "1842", rssi: -59 },
        { name: "MANTA ROBOTIC", model: "MANTA", serial: "0931", rssi: -73 }
    ];

    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, Number(value) || 0));
    }

    function speedFromTelemetry(telemetry) {
        var velocity = telemetry && telemetry.velocity ? telemetry.velocity : {};
        return Math.hypot(Number(velocity.vx) || 0, Number(velocity.vy) || 0);
    }

    function temperatureFromTelemetry(telemetry) {
        var temperature = telemetry && telemetry.temperature ? telemetry.temperature : {};
        return Number(temperature.flightController || temperature.hostBoard || 0);
    }

    function postJson(url, body) {
        return fetch(url, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body || {})
        }).then(function (response) {
            return response.json().catch(function () { return {}; }).then(function (payload) {
                if (!response.ok || payload.success === false || payload.ok === false) {
                    throw new Error(payload.message || ("HTTP " + response.status));
                }
                return payload;
            });
        });
    }

    function TransportBase() {
        this.listeners = {};
        this.connected = false;
        this.device = DEFAULT_DEVICE;
        this.mode = "base";
        this.motionLocked = false;
    }

    TransportBase.prototype.on = function (event, handler) {
        if (!this.listeners[event]) this.listeners[event] = [];
        this.listeners[event].push(handler);
        return function () {
            var list = this.listeners[event] || [];
            this.listeners[event] = list.filter(function (item) { return item !== handler; });
        }.bind(this);
    };

    TransportBase.prototype.emit = function (event, payload) {
        (this.listeners[event] || []).forEach(function (handler) {
            try { handler(payload); } catch (error) { console.error(error); }
        });
    };

    TransportBase.prototype.log = function (level, source, message, category, messageEn) {
        this.emit("log", {
            timestamp: new Date().toISOString(),
            level: level || "INFO",
            source: source || "APP",
            message: message || "",
            messageEn: messageEn || "",
            category: category || "system"
        });
    };

    TransportBase.prototype.setMotionLocked = function (locked) {
        this.motionLocked = Boolean(locked);
        return this.motionLocked;
    };

    function MockTransport() {
        TransportBase.call(this);
        this.mode = "mock";
        this.telemetryTimer = null;
        this.vector = { x: 0, y: 0 };
        this.startedAt = Date.now();
    }

    MockTransport.prototype = Object.create(TransportBase.prototype);
    MockTransport.prototype.constructor = MockTransport;

    MockTransport.prototype.discover = function () {
        this.log("INFO", "BLE", "正在扫描附近的 Manta", "hardware", "Scanning for a nearby Manta");
        return new Promise(function (resolve) {
            setTimeout(function () { resolve(MOCK_DEVICES.map(function (device) { return Object.assign({}, device); })); }, 650);
        });
    };

    MockTransport.prototype.pair = function (pin) {
        return new Promise(function (resolve, reject) {
            setTimeout(function () {
                if (String(pin) !== "240724") reject(new Error("配对码不正确"));
                else resolve({ token: "preview-session", wifi: "MANTA-2407" });
            }, 550);
        });
    };

    MockTransport.prototype.connect = function () {
        this.connected = true;
        this.emit("hardwareStatus", { boardOnline: true, pixhawkOnline: true, imuOnline: true, motorsOnline: true, gimbalOnline: true });
        this.startedAt = Date.now();
        this.log("INFO", "LINK", "BLE 控制链路已建立", "hardware", "BLE control link established");
        this.log("INFO", "WIFI", "5 GHz 影像链路已建立，互联网保持在线", "hardware", "5 GHz video link established; internet remains online");
        this.log("INFO", "PIXHAWK", "飞控状态在线", "hardware", "Flight controller online");
        this.startTelemetry();
        return Promise.resolve({ device: DEFAULT_DEVICE });
    };

    MockTransport.prototype.startTelemetry = function () {
        clearInterval(this.telemetryTimer);
        var tick = function () {
            if (!this.connected) return;
            var elapsed = (Date.now() - this.startedAt) / 1000;
            this.emit("telemetry", {
                latency: 14 + Math.round(Math.sin(elapsed / 3) * 4 + Math.random() * 3),
                speed: Math.hypot(this.vector.x, this.vector.y) * 2.2,
                temperature: 45 + Math.sin(elapsed / 18) * 2,
                attitude: {
                    roll: Math.sin(elapsed / 4) * 2.2,
                    pitch: Math.cos(elapsed / 5) * 1.4,
                    yaw: (184 + elapsed * 0.7) % 360
                },
                gps: { satellites: 9, hdop: 1.2, fixType: 3, estimated: true },
                vector: this.vector
            });
        }.bind(this);
        tick();
        this.telemetryTimer = setInterval(tick, 500);
    };

    MockTransport.prototype.drive = function (vector) {
        var nextVector = { x: clamp(vector.x, -1, 1), y: clamp(vector.y, -1, 1) };
        if (this.motionLocked && Math.hypot(nextVector.x, nextVector.y) > 0.001) return;
        this.vector = nextVector;
        var throttle = Math.round(-this.vector.y * 100);
        var steering = Math.round(this.vector.x * 45);
        this.emit("driveAck", {
            throttle: throttle,
            steering: steering,
            leftPwm: clamp(1500 + throttle * 4 - steering * 5, 1000, 2000),
            rightPwm: clamp(1500 + throttle * 4 + steering * 5, 1000, 2000)
        });
    };

    MockTransport.prototype.emergencyStop = function () {
        this.drive({ x: 0, y: 0 });
        this.log("CRITICAL", "SAFETY", "紧急停止已触发，推进输出归零", "command", "Emergency stop triggered; thrust output is zero");
        this.emit("emergency", { active: true });
        return Promise.resolve({ success: true });
    };

    MockTransport.prototype.arm = function () {
        this.log("COMMAND", "SAFETY", "推进武装已确认", "command", "Propulsion arm confirmed");
        this.emit("armed", { armed: true });
        return Promise.resolve({ success: true, armed: true });
    };

    MockTransport.prototype.gimbal = function (action, payload) {
        var labels = { home: "云台回中", stop: "停止云台", recordStart: "开始录像", recordStop: "停止录像", click: "点击居中", face: "人脸跟踪", swimmer: "泳者跟踪" };
        var labelsEn = { home: "Center gimbal", stop: "Stop gimbal", recordStart: "Start recording", recordStop: "Stop recording", click: "Click to center", face: "Face tracking", swimmer: "Swimmer tracking" };
        this.log("COMMAND", "GIMBAL", labels[action] || action, "command", labelsEn[action] || action);
        var state = Object.assign({ connected: true, action: action, ok: true, trackingActive: action === "face" || action === "swimmer", trackMode: action === "swimmer" ? "swimmer" : "face" }, payload || {});
        if (state.trackingActive) {
            state.trackStatus = { locked: true, status: "locked", frame_w: 1920, frame_h: 1080, x: 720, y: 220, w: 330, h: 520 };
            state.lastTarget = state.trackStatus;
        }
        this.emit("gimbalState", state);
        return Promise.resolve({ success: true });
    };

    MockTransport.prototype.gimbalClick = function (dx, dy) {
        this.log("COMMAND", "GIMBAL", "点击目标 X " + dx + " / Y " + dy, "command", "Click target X " + dx + " / Y " + dy);
        return Promise.resolve({ success: true, delta: { dx: dx, dy: dy } });
    };

    MockTransport.prototype.setGimbalOsd = function (enabled) {
        this.log("COMMAND", "GIMBAL", enabled ? "显示云台 OSD" : "隐藏云台 OSD", "command", enabled ? "Gimbal OSD enabled" : "Gimbal OSD hidden");
        return Promise.resolve({ success: true, mode: enabled ? 2 : 0 });
    };

    MockTransport.prototype.recoverGimbal = function () {
        this.emit("gimbalState", { connected: true, trackingActive: false, trackMode: "face", trackStatus: { locked: false, status: "idle", message: "idle" } });
        return Promise.resolve({ connected: true });
    };

    MockTransport.prototype.startImuCalibration = function () {
        this.log("COMMAND", "PIXHAWK", "开始 IMU 六面校准", "command", "Started six-position IMU calibration");
        return Promise.resolve({ success: true, status: "STARTED" });
    };

    MockTransport.prototype.confirmImuCalibration = function (positionCode) {
        this.log("COMMAND", "PIXHAWK", "确认 IMU 校准姿态 " + Number(positionCode), "command", "Confirmed IMU calibration pose " + Number(positionCode));
        return Promise.resolve({ success: true, positionCode: Number(positionCode), status: "CONFIRMED" });
    };

    MockTransport.prototype.disconnect = function () {
        this.connected = false;
        clearInterval(this.telemetryTimer);
        this.drive({ x: 0, y: 0 });
        this.log("INFO", "LINK", "设备连接已断开", "hardware", "Device connection closed");
        return Promise.resolve();
    };

    function LiveTransport() {
        TransportBase.call(this);
        this.mode = "live";
        this.socket = null;
        this.pollTimer = null;
        this.lastDriveAt = 0;
        this.pendingDrive = null;
        this.driveTimer = null;
        this.lastLatency = 0;
        this.pollFailures = 0;
        this.gimbalTrackingActive = false;
        this.gimbalTrackMode = null;
        this.gimbalModePromise = Promise.resolve();
    }

    LiveTransport.prototype = Object.create(TransportBase.prototype);
    LiveTransport.prototype.constructor = LiveTransport;

    LiveTransport.prototype.discover = function () {
        return fetch("/health", { cache: "no-store" }).then(function (response) {
            if (!response.ok) throw new Error("未找到 Manta 控制服务");
            return [DEFAULT_DEVICE];
        });
    };

    LiveTransport.prototype.pair = function (pin) {
        if (!/^\d{6}$/.test(String(pin))) return Promise.reject(new Error("请输入六位配对码"));
        // 当前后端尚未提供配对端点；只在 UI 层验证格式，真实令牌端点上线后替换这里。
        return Promise.resolve({ token: "local-preview" });
    };

    LiveTransport.prototype.connect = function () {
        var requestStartedAt = performance.now();
        return fetch("/api/status", { cache: "no-store" }).then(function (response) {
            if (!response.ok) throw new Error("Manta 状态接口不可用");
            return response.json();
        }).then(function (payload) {
            if (!payload || payload.success === false) throw new Error(payload.message || "Manta 返回无效状态");
            this.lastLatency = performance.now() - requestStartedAt;
            this.connected = true;
            this.connectSocket();
            this.applyStatus(payload);
            this.startPolling();
            this.log("INFO", "LINK", "已连接 Manta 真实控制服务", "hardware");
            return this.openGimbalLink().then(function () { return { device: DEFAULT_DEVICE }; });
        }.bind(this));
    };

    LiveTransport.prototype.connectSocket = function () {
        if (!window.io) return;
        this.socket = window.io({ transports: ["websocket", "polling"], timeout: 5000 });
        this.socket.on("connect", function () { this.emit("connection", { connected: true, degraded: false }); }.bind(this));
        this.socket.on("telemetry_update", function (telemetry) { this.applyTelemetry(telemetry); }.bind(this));
        this.socket.on("system_state", function (state) { if (state && state.telemetry) this.applyTelemetry(state.telemetry); }.bind(this));
        this.socket.on("rover_drive_ack", function (payload) { this.emit("driveAck", payload || {}); }.bind(this));
        this.socket.on("gimbal_state", function (state) { this.applyGimbalTrackingState(state); this.emit("gimbalState", state || {}); }.bind(this));
        this.socket.on("gimbal_target", function (target) { this.emit("gimbalTarget", target || {}); }.bind(this));
        this.socket.on("gimbal_track_status", function (status) { this.emit("gimbalTrackStatus", status || {}); }.bind(this));
        this.socket.on("aircraft_armed", function () { this.emit("armed", { armed: true }); }.bind(this));
        this.socket.on("aircraft_disarmed", function () { this.emit("armed", { armed: false }); }.bind(this));
        this.socket.on("log_entry", function (entry) {
            this.log(entry.level, entry.source || "BOARD", entry.message, /motor|gimbal|pixhawk/i.test(entry.source || "") ? "hardware" : "system");
        }.bind(this));
        this.socket.on("disconnect", function () { this.emit("connection", { connected: false, degraded: true }); }.bind(this));
    };

    LiveTransport.prototype.startPolling = function () {
        clearInterval(this.pollTimer);
        this.pollTimer = setInterval(function () {
            if (!this.connected) return;
            var startedAt = performance.now();
            fetch("/api/status", { cache: "no-store" }).then(function (response) { if (!response.ok) throw new Error("HTTP " + response.status); return response.json(); }).then(function (payload) {
                this.lastLatency = performance.now() - startedAt;
                this.pollFailures = 0;
                this.applyStatus(payload);
                this.refreshGimbalState();
                this.emit("connection", { connected: true, degraded: false, hardwareOnline: this.pixhawkOnline });
            }.bind(this)).catch(function (error) {
                this.pollFailures += 1;
                this.log("WARNING", "LINK", "状态轮询失败：" + error.message, "hardware");
                if (this.pollFailures >= 2) this.emit("connection", { connected: false, degraded: true });
            }.bind(this));
        }.bind(this), 2000);
    };

    LiveTransport.prototype.applyStatus = function (payload) {
        var data = payload && payload.data ? payload.data : {};
        this.pixhawkOnline = Boolean(data.isConnected) && String(data.pixhawkStatus || "").toLowerCase() !== "disconnected";
        this.emit("hardwareStatus", { boardOnline: true, pixhawkOnline: this.pixhawkOnline, imuOnline: this.pixhawkOnline, motorsOnline: this.pixhawkOnline });
        if (data.telemetry) this.applyTelemetry(data.telemetry);
        if (data.gimbal) { this.applyGimbalTrackingState(data.gimbal); this.emit("gimbalState", data.gimbal); }
    };

    LiveTransport.prototype.refreshGimbalState = function () {
        return fetch("/api/gimbal/state", { cache: "no-store" }).then(function (response) { if (!response.ok) throw new Error("HTTP " + response.status); return response.json(); }).then(function (payload) {
            var state = payload && payload.state ? payload.state : {};
            this.applyGimbalTrackingState(state);
            this.emit("gimbalState", state);
            this.emit("hardwareStatus", { gimbalOnline: Boolean(state.connected) });
            return state;
        }.bind(this)).catch(function () {
            this.emit("hardwareStatus", { gimbalOnline: false });
            return {};
        }.bind(this));
    };

    LiveTransport.prototype.applyGimbalTrackingState = function (state) {
        state = state || {};
        this.gimbalTrackingActive = Boolean(state.trackingActive);
        if (state.trackMode === "face" || state.trackMode === "swimmer") this.gimbalTrackMode = state.trackMode;
        if (!this.gimbalTrackingActive) this.gimbalTrackMode = null;
    };

    LiveTransport.prototype.openGimbalLink = function () {
        return postJson("/api/gimbal/connect", {}).then(function () {
            return new Promise(function (resolve) { setTimeout(resolve, 180); });
        }).then(function () {
            return this.refreshGimbalState();
        }.bind(this)).catch(function (error) {
            this.emit("hardwareStatus", { gimbalOnline: false });
            this.log("WARNING", "GIMBAL", "云台串口连接失败：" + error.message, "hardware", "Gimbal serial connection failed: " + error.message);
            return {};
        }.bind(this));
    };

    LiveTransport.prototype.recoverGimbal = function () {
        return postJson("/api/gimbal/stop", {}).catch(function () {}).then(function () {
            return postJson("/api/gimbal/connect", {});
        }).then(function () {
            return new Promise(function (resolve) { setTimeout(resolve, 220); });
        }).then(function () {
            return this.refreshGimbalState();
        }.bind(this)).then(function (state) {
            if (!state || !state.connected || !state.feedback || state.feedback.checksumValid === false) throw new Error("云台尚未返回有效状态，请检查供电与串口");
            return state;
        });
    };

    LiveTransport.prototype.applyTelemetry = function (telemetry) {
        this.emit("telemetry", {
            latency: this.lastLatency,
            speed: speedFromTelemetry(telemetry),
            temperature: temperatureFromTelemetry(telemetry),
            attitude: telemetry.attitude || { roll: 0, pitch: 0, yaw: 0 },
            battery: telemetry.battery || {},
            imuCalibration: telemetry.imuCalibration || {},
            gps: telemetry.gps || {}
        });
    };

    LiveTransport.prototype.drive = function (vector) {
        var payload = { throttle: Math.round(clamp(-vector.y, -1, 1) * 100), steering: Math.round(clamp(vector.x, -1, 1) * 45) };
        if (this.motionLocked && (payload.throttle !== 0 || payload.steering !== 0)) return;
        var send = function () {
            this.lastDriveAt = Date.now();
            this.pendingDrive = null;
            if (this.socket && this.socket.connected) {
                this.socket.emit("rover_drive", payload);
                return;
            }
            postJson("/api/control/rover", payload).then(function (ack) { this.emit("driveAck", ack && ack.data ? ack.data : ack); }.bind(this)).catch(function (error) {
                this.log("ERROR", "CONTROL", "移动指令失败：" + error.message, "command");
            }.bind(this));
        }.bind(this);
        var remaining = 50 - (Date.now() - this.lastDriveAt);
        clearTimeout(this.driveTimer);
        if (remaining <= 0 || (payload.throttle === 0 && payload.steering === 0)) send();
        else this.driveTimer = setTimeout(send, remaining);
    };

    LiveTransport.prototype.emergencyStop = function () {
        this.drive({ x: 0, y: 0 });
        return postJson("/api/emergency/stop", {}).then(function (result) {
            this.log("CRITICAL", "SAFETY", "紧急停止已执行", "command");
            this.emit("emergency", { active: true });
            return result;
        }.bind(this));
    };

    LiveTransport.prototype.arm = function () {
        if (!this.socket || !this.socket.connected) return Promise.reject(new Error("实时控制链路未连接，拒绝武装"));
        return new Promise(function (resolve, reject) {
            var settled = false;
            var onArmed = function () { finish(null, { success: true, armed: true }); };
            var finish = function (error, result) {
                if (settled) return;
                settled = true;
                clearTimeout(timer);
                this.socket.off("aircraft_armed", onArmed);
                if (error) reject(error); else resolve(result);
            }.bind(this);
            var timer = setTimeout(function () { finish(new Error("飞控未在时限内确认武装")); }, 3000);
            this.socket.once("aircraft_armed", onArmed);
            this.socket.emit("arm");
        }.bind(this));
    };

    LiveTransport.prototype.gimbal = function (action, payload) {
        if (action === "face" || action === "swimmer" || action === "click") {
            var requestedMode = action;
            var switchMode = function () {
                if (requestedMode === "click") {
                    return postJson("/api/gimbal/track/stop", {}).then(function (result) {
                        this.applyGimbalTrackingState(result && result.state);
                        if (result && result.state) this.emit("gimbalState", result.state);
                        return result;
                    }.bind(this));
                }
                if (this.gimbalTrackingActive && this.gimbalTrackMode === requestedMode) {
                    return Promise.resolve({ success: true, active: true, alreadyRunning: true, mode: requestedMode });
                }
                var startRequestedMode = function () {
                    return postJson("/api/gimbal/track/start", { mode: requestedMode });
                };
                var request = this.gimbalTrackingActive ? postJson("/api/gimbal/track/stop", {}).then(startRequestedMode) : startRequestedMode();
                return request.then(function (result) {
                    this.applyGimbalTrackingState(result && result.state);
                    if (result && result.state) this.emit("gimbalState", result.state);
                    return result;
                }.bind(this));
            }.bind(this);
            this.gimbalModePromise = this.gimbalModePromise.catch(function () {}).then(switchMode);
            return this.gimbalModePromise;
        }
        var endpoint = null;
        var body = payload || {};
        if (action === "home") endpoint = "/api/gimbal/home";
        if (action === "stop") endpoint = "/api/gimbal/stop";
        if (action === "recordStart") endpoint = "/api/gimbal/recording/start";
        if (action === "recordStop") endpoint = "/api/gimbal/recording/stop";
        if (!endpoint) return Promise.reject(new Error("不支持的云台指令"));
        return postJson(endpoint, body).then(function (result) {
            this.log("COMMAND", "GIMBAL", "云台指令：" + action, "command");
            if (result && result.state) this.emit("gimbalState", result.state);
            return result;
        }.bind(this));
    };

    LiveTransport.prototype.gimbalClick = function (dx, dy) {
        return postJson("/api/gimbal/click", { dx: Math.round(Number(dx) || 0), dy: Math.round(Number(dy) || 0) }).then(function (result) {
            this.log("COMMAND", "GIMBAL", "点击目标 X " + dx + " / Y " + dy, "command", "Click target X " + dx + " / Y " + dy);
            if (result && result.state) this.emit("gimbalState", result.state);
            return result;
        }.bind(this));
    };

    LiveTransport.prototype.setGimbalOsd = function (enabled) {
        return postJson("/api/gimbal/osd", { mode: enabled ? 2 : 0 }).then(function (result) {
            this.log("COMMAND", "GIMBAL", enabled ? "显示云台 OSD" : "隐藏云台 OSD", "command", enabled ? "Gimbal OSD enabled" : "Gimbal OSD hidden");
            return result;
        }.bind(this));
    };

    LiveTransport.prototype.startImuCalibration = function (type) {
        return postJson("/api/calibration/imu/start", { type: String(type || "ACCEL").toUpperCase() });
    };

    LiveTransport.prototype.confirmImuCalibration = function (positionCode) {
        return postJson("/api/calibration/imu/confirm", { positionCode: Number(positionCode) });
    };

    LiveTransport.prototype.getRecordings = function () {
        return fetch("/api/gimbal/recordings", { cache: "no-store" }).then(function (response) { return response.json(); }).then(function (payload) {
            if (!payload.success) throw new Error(payload.message || "录像列表不可用");
            return payload.recordings || [];
        });
    };

    LiveTransport.prototype.disconnect = function () {
        this.drive({ x: 0, y: 0 });
        this.connected = false;
        clearInterval(this.pollTimer);
        clearTimeout(this.driveTimer);
        if (this.socket) this.socket.disconnect();
        this.socket = null;
        return Promise.resolve();
    };

    function GimbalHealthMonitor(options) {
        options = options || {};
        this.onFault = options.onFault || function () {};
        this.angleThreshold = Number(options.angleThreshold || 0.8);
        this.windowMs = Number(options.windowMs || 1800);
        this.requiredWindows = Number(options.requiredWindows || 3);
        this.minCommandRate = Number(options.minCommandRate || 8);
        this.maxStillGyro = Number(options.maxStillGyro || 1);
        this.feedbackTimeoutMs = Number(options.feedbackTimeoutMs || 5000);
        this.history = [];
        this.consecutiveStalls = 0;
        this.latched = false;
        this.armed = false;
        this.lastSampleAt = 0;
        this.watchdogTimer = setInterval(function () {
            if (!this.armed || this.latched || Date.now() - this.lastSampleAt <= this.feedbackTimeoutMs) return;
            this.armed = false;
            this.history = [];
            this.consecutiveStalls = 0;
        }.bind(this), 500);
    }

    GimbalHealthMonitor.prototype.reset = function () {
        this.history = [];
        this.consecutiveStalls = 0;
        this.latched = false;
        this.armed = false;
        this.lastSampleAt = 0;
    };

    GimbalHealthMonitor.prototype.sample = function (state) {
        if (!state || this.latched) return null;
        var feedback = state.feedback || {};
        var now = Date.now();
        var sample = {
            at: now,
            yaw: Number(feedback.yawDeg),
            pitch: Number(feedback.pitchDeg),
            gyro: Math.hypot(Number(feedback.gyroYawDps) || 0, Number(feedback.gyroPitchDps) || 0),
            command: String(state.lastCommand || ""),
            error: String(state.lastError || ""),
            connected: state.connected !== false
        };
        if (sample.error) return this.fault("GMB-DEVICE-ERROR", sample.error, state);
        if (!sample.connected) return this.fault("GMB-LINK-LOST", "云台通信已断开", state);
        if (!Number.isFinite(sample.yaw) || !Number.isFinite(sample.pitch)) return null;
        this.lastSampleAt = now;
        this.history.push(sample);
        this.history = this.history.filter(function (item) { return now - item.at <= this.windowMs * 1.4; }.bind(this));
        var first = this.history[0];
        if (!first || now - first.at < this.windowMs) return null;
        var delta = Math.hypot(sample.yaw - first.yaw, sample.pitch - first.pitch);
        var target = state.lastTarget || {};
        var targetAt = Number(target.timestamp || target.updatedAt || 0);
        var commandRate = Math.hypot(
            Number(target.desiredRateX || target.rateX || 0),
            Number(target.desiredRateY || target.rateY || 0)
        );
        var targetFresh = targetAt > 0 && now - targetAt <= this.windowMs * 1.5;
        var expectsMotion = Boolean(state.trackingActive) && targetFresh && commandRate >= this.minCommandRate && !target.gated && !target.limited;
        this.armed = expectsMotion;
        if (expectsMotion && delta < this.angleThreshold && sample.gyro < this.maxStillGyro) this.consecutiveStalls += 1;
        else this.consecutiveStalls = 0;
        this.history = [sample];
        if (this.consecutiveStalls >= this.requiredWindows) {
            return this.fault("GMB-MOTION-STALL", "云台收到非零运动目标，但角度与陀螺仪均无响应", { state: state, delta: delta, gyro: sample.gyro, commandRate: commandRate, windows: this.consecutiveStalls, inferredNotHardwareConfirmed: true });
        }
        return { healthy: true, delta: delta, gyro: sample.gyro, commandRate: commandRate, consecutiveStalls: this.consecutiveStalls };
    };

    GimbalHealthMonitor.prototype.fault = function (code, message, evidence) {
        this.latched = true;
        var report = { healthy: false, code: code, message: message, evidence: evidence, detectedAt: new Date().toISOString(), action: "STOP_GIMBAL" };
        this.onFault(report);
        return report;
    };

    window.MantaAppTransport = {
        create: function (mode) { return mode === "live" ? new LiveTransport() : new MockTransport(); },
        MockTransport: MockTransport,
        LiveTransport: LiveTransport,
        GimbalHealthMonitor: GimbalHealthMonitor,
        defaults: { device: DEFAULT_DEVICE }
    };
})(window);
