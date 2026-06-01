(function () {
    function asNumber(value, fallback) {
        var parsed = Number(value);
        return Number.isFinite(parsed) ? parsed : fallback;
    }

    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    function mergeOptionalNumber(value, fallback) {
        if (value === null || typeof value === "undefined" || value === "") {
            return fallback;
        }

        var parsed = Number(value);
        return Number.isFinite(parsed) ? parsed : fallback;
    }

    function deepMergeTelemetry(previous, incoming) {
        var next = {
            position: Object.assign({}, previous.position),
            attitude: Object.assign({}, previous.attitude),
            velocity: Object.assign({}, previous.velocity),
            battery: Object.assign({}, previous.battery),
            servoOutputs: Object.assign({}, previous.servoOutputs),
            temperature: Object.assign({}, previous.temperature),
            gps: Object.assign({}, previous.gps),
            imuCalibration: Object.assign({}, previous.imuCalibration),
            flightMode: previous.flightMode,
            systemStatus: previous.systemStatus,
            armed: previous.armed
        };

        if (incoming && typeof incoming === "object") {
            if (incoming.position) {
                next.position.lat = asNumber(incoming.position.lat, next.position.lat);
                next.position.lon = asNumber(incoming.position.lon, next.position.lon);
                next.position.alt = asNumber(incoming.position.alt, next.position.alt);
            }
            if (incoming.attitude) {
                next.attitude.roll = asNumber(incoming.attitude.roll, next.attitude.roll);
                next.attitude.pitch = asNumber(incoming.attitude.pitch, next.attitude.pitch);
                next.attitude.yaw = asNumber(incoming.attitude.yaw, next.attitude.yaw);
            }
            if (incoming.velocity) {
                next.velocity.vx = asNumber(incoming.velocity.vx, next.velocity.vx);
                next.velocity.vy = asNumber(incoming.velocity.vy, next.velocity.vy);
                next.velocity.vz = asNumber(incoming.velocity.vz, next.velocity.vz);
            }
            if (incoming.battery) {
                next.battery.voltage = asNumber(incoming.battery.voltage, next.battery.voltage);
                next.battery.current = asNumber(incoming.battery.current, next.battery.current);
                next.battery.percentage = clamp(asNumber(incoming.battery.percentage, next.battery.percentage), 0, 100);
            }
            if (incoming.servoOutputs && typeof incoming.servoOutputs === "object") {
                ["ch1", "ch2", "ch3", "ch4"].forEach(function (channel) {
                    next.servoOutputs[channel] = asNumber(incoming.servoOutputs[channel], next.servoOutputs[channel]);
                });
            }
            if (incoming.temperature) {
                next.temperature.hostBoard = mergeOptionalNumber(incoming.temperature.hostBoard, next.temperature.hostBoard);
                next.temperature.flightController = mergeOptionalNumber(incoming.temperature.flightController, next.temperature.flightController);
                next.temperature.motorLeft = mergeOptionalNumber(incoming.temperature.motorLeft, next.temperature.motorLeft);
                next.temperature.motorRight = mergeOptionalNumber(incoming.temperature.motorRight, next.temperature.motorRight);
            }
            if (incoming.gps) {
                next.gps.satellites = Math.max(0, asNumber(incoming.gps.satellites, next.gps.satellites));
                next.gps.hdop = asNumber(incoming.gps.hdop, next.gps.hdop);
            }
            if (incoming.imuCalibration && typeof incoming.imuCalibration === "object") {
                if (typeof incoming.imuCalibration.active === "boolean") {
                    next.imuCalibration.active = incoming.imuCalibration.active;
                }
                if (typeof incoming.imuCalibration.mode === "string" && incoming.imuCalibration.mode.trim()) {
                    next.imuCalibration.mode = incoming.imuCalibration.mode.trim().toUpperCase();
                }
                if (typeof incoming.imuCalibration.status === "string" && incoming.imuCalibration.status.trim()) {
                    next.imuCalibration.status = incoming.imuCalibration.status.trim().toUpperCase();
                }
                if (typeof incoming.imuCalibration.step === "string") {
                    next.imuCalibration.step = incoming.imuCalibration.step.trim().toUpperCase();
                }
                if (Object.prototype.hasOwnProperty.call(incoming.imuCalibration, "stepCode")) {
                    if (incoming.imuCalibration.stepCode === null || incoming.imuCalibration.stepCode === "") {
                        next.imuCalibration.stepCode = null;
                    } else {
                        next.imuCalibration.stepCode = asNumber(incoming.imuCalibration.stepCode, next.imuCalibration.stepCode || 0);
                    }
                }
                if (typeof incoming.imuCalibration.instructions === "string") {
                    next.imuCalibration.instructions = incoming.imuCalibration.instructions;
                }
                if (Object.prototype.hasOwnProperty.call(incoming.imuCalibration, "progress")) {
                    if (incoming.imuCalibration.progress === null || incoming.imuCalibration.progress === "") {
                        next.imuCalibration.progress = null;
                    } else {
                        next.imuCalibration.progress = asNumber(incoming.imuCalibration.progress, next.imuCalibration.progress || 0);
                    }
                }
                if (Object.prototype.hasOwnProperty.call(incoming.imuCalibration, "lastAckCommand")) {
                    if (incoming.imuCalibration.lastAckCommand === null || incoming.imuCalibration.lastAckCommand === "") {
                        next.imuCalibration.lastAckCommand = null;
                    } else {
                        next.imuCalibration.lastAckCommand = asNumber(incoming.imuCalibration.lastAckCommand, next.imuCalibration.lastAckCommand || 0);
                    }
                }
                if (typeof incoming.imuCalibration.lastAckResult === "string") {
                    next.imuCalibration.lastAckResult = incoming.imuCalibration.lastAckResult.trim().toUpperCase();
                }
                if (Object.prototype.hasOwnProperty.call(incoming.imuCalibration, "updatedAt")) {
                    next.imuCalibration.updatedAt = asNumber(incoming.imuCalibration.updatedAt, next.imuCalibration.updatedAt || 0);
                }
            }
            if (typeof incoming.flightMode === "string" && incoming.flightMode.trim()) {
                next.flightMode = incoming.flightMode.trim().toUpperCase();
            }
            if (typeof incoming.systemStatus === "string" && incoming.systemStatus.trim()) {
                next.systemStatus = incoming.systemStatus.trim().toUpperCase();
            }
            if (typeof incoming.armed === "boolean") {
                next.armed = incoming.armed;
            }
        }

        return next;
    }

    var state = {
        socketConnected: false,
        pixhawkStatus: "disconnected",
        isConnected: false,
        connectivity: {
            wireless: { interface: "wlan0", present: false, state: "missing", online: false, type: "wireless" },
            ethernet: { interface: "eth0", present: false, state: "missing", online: false, type: "ethernet" },
            can: { interface: "can0", present: false, state: "missing", online: false, type: "can" }
        },
        camera: {
            enabled: true,
            label: "OV8858 Camera",
            transport: "local",
            sensor: "ov8858",
            device: "auto",
            canInterface: "can0",
            sourceType: "mjpeg",
            sourceUrl: "/api/camera/stream",
            openUrl: "/api/camera/stream",
            refreshMs: 200,
            online: true,
            host: "10.42.0.1"
        },
        telemetry: {
            position: { lat: 0, lon: 0, alt: 0 },
            attitude: { roll: 0, pitch: 0, yaw: 0 },
            velocity: { vx: 0, vy: 0, vz: 0 },
            battery: { voltage: 0, current: 0, percentage: 100 },
            servoOutputs: { ch1: 0, ch2: 0, ch3: 0, ch4: 0 },
            temperature: { hostBoard: null, flightController: null, motorLeft: null, motorRight: null },
            gps: { satellites: 0, hdop: 999 },
            imuCalibration: {
                active: false,
                mode: "IDLE",
                status: "IDLE",
                step: "",
                stepCode: null,
                instructions: "Idle",
                progress: null,
                lastAckCommand: null,
                lastAckResult: "",
                updatedAt: null
            },
            flightMode: "MANUAL",
            systemStatus: "STANDBY",
            armed: false
        },
        roverControl: {
            throttle: 0,
            steering: 0,
            leftPwm: 1500,
            rightPwm: 1500
        },
        limits: {
            throttle: { min: -100, max: 100 },
            steering: { min: -45, max: 45 },
            pwm: { min: 1000, max: 2000, center: 1500 }
        },
        roverChannels: {
            left: 1,
            right: 3
        },
        logs: []
    };

    var lastTelemetryAt = 0;

    var listeners = {};

    function emit(eventName, payload) {
        var handlers = listeners[eventName] || [];
        handlers.forEach(function (handler) {
            try {
                handler(payload, state);
            } catch (error) {
                console.error("[RoverClient] Event handler error:", error);
            }
        });
    }

    function on(eventName, handler) {
        if (!listeners[eventName]) {
            listeners[eventName] = [];
        }
        listeners[eventName].push(handler);
        return function () {
            listeners[eventName] = (listeners[eventName] || []).filter(function (candidate) {
                return candidate !== handler;
            });
        };
    }

    function pushLog(level, message, timestamp) {
        var entry = {
            timestamp: timestamp || new Date().toISOString(),
            level: level || "INFO",
            message: message || ""
        };

        state.logs.push(entry);
        if (state.logs.length > 300) {
            state.logs = state.logs.slice(-300);
        }

        emit("log", entry);
    }

    function mergeSystemState(payload) {
        if (!payload || typeof payload !== "object") {
            return;
        }

        if (typeof payload.isConnected === "boolean") {
            state.isConnected = payload.isConnected;
            state.pixhawkStatus = payload.isConnected ? "connected" : "disconnected";
            emit("pixhawkConnection", {
                isConnected: state.isConnected,
                status: state.pixhawkStatus
            });
        }

        if (payload.telemetry) {
            state.telemetry = deepMergeTelemetry(state.telemetry, payload.telemetry);
            lastTelemetryAt = Date.now();
            emit("telemetry", state.telemetry);
        }

        if (payload.roverControl) {
            state.roverControl = {
                throttle: asNumber(payload.roverControl.throttle, state.roverControl.throttle),
                steering: asNumber(payload.roverControl.steering, state.roverControl.steering),
                leftPwm: asNumber(payload.roverControl.leftPwm, state.roverControl.leftPwm),
                rightPwm: asNumber(payload.roverControl.rightPwm, state.roverControl.rightPwm)
            };
            emit("roverControl", state.roverControl);
        }

        if (payload.connectivity && typeof payload.connectivity === "object") {
            state.connectivity = Object.assign({}, state.connectivity, payload.connectivity);
        }

        if (payload.camera && typeof payload.camera === "object") {
            state.camera = Object.assign({}, state.camera, payload.camera);
        }

        emit("status", payload);
    }

    function requestStatus() {
        return fetch("/api/status")
            .then(function (response) {
                if (!response.ok) {
                    throw new Error("HTTP " + response.status);
                }
                return response.json();
            })
            .then(function (result) {
                if (result && result.success) {
                    mergeSystemState(result.data || {});
                    if (result.limits) {
                        state.limits = result.limits;
                        emit("limits", state.limits);
                    }
                    if (result.roverChannels) {
                        state.roverChannels = {
                            left: asNumber(result.roverChannels.left, state.roverChannels.left),
                            right: asNumber(result.roverChannels.right, state.roverChannels.right)
                        };
                        emit("roverChannels", state.roverChannels);
                    }
                }
            })
            .catch(function (error) {
                pushLog("ERROR", "Status request failed: " + error.message);
            });
    }

    var socket = io({
        transports: ["polling", "websocket"],
        timeout: 5000,
        reconnection: true,
        reconnectionAttempts: Infinity,
        reconnectionDelay: 800,
        reconnectionDelayMax: 4000
    });

    socket.on("connect", function () {
        state.socketConnected = true;
        lastTelemetryAt = Date.now();
        emit("socketConnection", true);
        requestStatus();
        socket.emit("request_telemetry");
        pushLog("INFO", "Connected to control service");
    });

    socket.on("disconnect", function () {
        state.socketConnected = false;
        emit("socketConnection", false);
        pushLog("WARNING", "Disconnected from control service");
    });

    socket.on("system_state", function (payload) {
        mergeSystemState(payload);
    });

    socket.on("connection_status", function (payload) {
        state.isConnected = Boolean(payload && payload.isConnected);
        state.pixhawkStatus = payload && payload.status ? payload.status : (state.isConnected ? "connected" : "disconnected");
        emit("pixhawkConnection", {
            isConnected: state.isConnected,
            status: state.pixhawkStatus
        });
    });

    socket.on("telemetry_update", function (payload) {
        state.telemetry = deepMergeTelemetry(state.telemetry, payload);
        lastTelemetryAt = Date.now();
        emit("telemetry", state.telemetry);
    });

    socket.on("rover_control_update", function (payload) {
        if (!payload) {
            return;
        }
        state.roverControl = {
            throttle: asNumber(payload.throttle, state.roverControl.throttle),
            steering: asNumber(payload.steering, state.roverControl.steering),
            leftPwm: asNumber(payload.leftPwm, state.roverControl.leftPwm),
            rightPwm: asNumber(payload.rightPwm, state.roverControl.rightPwm)
        };
        emit("roverControl", state.roverControl);
    });

    socket.on("rover_drive_ack", function (payload) {
        emit("driveAck", payload || {});
    });

    socket.on("motor_update", function (payload) {
        emit("motorUpdate", payload || {});
    });

    socket.on("aircraft_armed", function () {
        state.telemetry.armed = true;
        emit("telemetry", state.telemetry);
    });

    socket.on("aircraft_disarmed", function () {
        state.telemetry.armed = false;
        emit("telemetry", state.telemetry);
    });

    socket.on("log_entry", function (entry) {
        pushLog(entry && entry.level, entry && entry.message, entry && entry.timestamp);
    });

    socket.on("error_message", function (payload) {
        pushLog("ERROR", payload && payload.message ? payload.message : "Unknown error");
    });

    socket.on("info_message", function (payload) {
        pushLog("INFO", payload && payload.message ? payload.message : "Informational message");
    });

    socket.on("connect_error", function (error) {
        pushLog("ERROR", "Connection failed: " + error.message);
    });

    function arm() {
        socket.emit("arm");
    }

    function disarm() {
        socket.emit("disarm");
    }

    function drive(throttle, steering) {
        socket.emit("rover_drive", {
            throttle: asNumber(throttle, 0),
            steering: asNumber(steering, 0)
        });
    }

    function setMotorPwm(channel, pwm) {
        socket.emit("motor_control", {
            channel: asNumber(channel, 0),
            pwm: asNumber(pwm, state.limits.pwm.center)
        });
    }

    function requestTelemetry() {
        socket.emit("request_telemetry");
    }

    function postJson(url, payload, fallbackError) {
        return fetch(url, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload || {})
        })
            .then(function (response) {
                return response.json().then(function (result) {
                    return { ok: response.ok, result: result };
                });
            })
            .then(function (payloadResult) {
                if (!payloadResult.ok || !payloadResult.result || !payloadResult.result.success) {
                    throw new Error(
                        (payloadResult.result && payloadResult.result.message) || fallbackError || "Request failed"
                    );
                }

                if (payloadResult.result.telemetry) {
                    state.telemetry = deepMergeTelemetry(state.telemetry, payloadResult.result.telemetry);
                    emit("telemetry", state.telemetry);
                }

                if (payloadResult.result.message) {
                    pushLog("COMMAND", payloadResult.result.message);
                }

                return payloadResult.result;
            })
            .catch(function (error) {
                pushLog("ERROR", error.message || fallbackError || "Request failed");
                throw error;
            });
    }

    function startImuCalibration(type) {
        var calibrationType = String(type || "ACCEL").trim().toUpperCase();
        return postJson(
            "/api/calibration/imu/start",
            { type: calibrationType },
            "Failed to start IMU calibration"
        );
    }

    function confirmImuCalibration(positionCode) {
        var payload = {};
        if (typeof positionCode !== "undefined" && positionCode !== null) {
            payload.positionCode = asNumber(positionCode, 0);
        }
        return postJson(
            "/api/calibration/imu/confirm",
            payload,
            "Failed to confirm IMU pose"
        );
    }

    function emergencyStop() {
        return fetch("/api/emergency/stop", {
            method: "POST",
            headers: { "Content-Type": "application/json" }
        })
            .then(function (response) {
                return response.json();
            })
            .then(function (result) {
                if (!result.success) {
                    throw new Error(result.message || "Emergency stop failed");
                }
                pushLog("CRITICAL", "Emergency stop triggered");
                return result;
            })
            .catch(function (error) {
                pushLog("ERROR", "Emergency stop failed: " + error.message);
                throw error;
            });
    }

    function fetchLogs(limit) {
        var queryLimit = Math.max(1, Math.min(1000, asNumber(limit, 200)));
        return fetch("/api/logs?limit=" + queryLimit)
            .then(function (response) { return response.json(); })
            .then(function (result) {
                if (result && result.success && Array.isArray(result.logs)) {
                    state.logs = result.logs.slice(-300);
                    emit("logsReplace", state.logs.slice());
                }
                return state.logs;
            });
    }

    function downloadLogs() {
        window.location.href = "/api/logs/download";
    }

    var telemetryTimer = window.setInterval(function () {
        if (state.socketConnected && Date.now() - lastTelemetryAt >= 1500) {
            requestTelemetry();
        }
    }, 2000);

    var statusTimer = window.setInterval(function () {
        requestStatus();
    }, 8000);

    requestStatus();

    window.addEventListener("beforeunload", function () {
        window.clearInterval(telemetryTimer);
        window.clearInterval(statusTimer);
    });

    window.RoverClient = {
        state: state,
        on: on,
        requestStatus: requestStatus,
        requestTelemetry: requestTelemetry,
        startImuCalibration: startImuCalibration,
        confirmImuCalibration: confirmImuCalibration,
        arm: arm,
        disarm: disarm,
        drive: drive,
        setMotorPwm: setMotorPwm,
        emergencyStop: emergencyStop,
        fetchLogs: fetchLogs,
        downloadLogs: downloadLogs
    };
})();
