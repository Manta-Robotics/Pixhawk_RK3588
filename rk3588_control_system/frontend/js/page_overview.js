(function () {
    var CALIBRATION_STATUS_TEXT = {
        IDLE: "Idle",
        STARTING: "Starting",
        IN_PROGRESS: "In progress",
        AWAITING_POSITION: "Waiting for pose",
        CONFIRMING_POSITION: "Pose confirmed",
        SUCCESS: "Completed",
        FAILED: "Failed",
        CANCELLED: "Cancelled"
    };
    var CALIBRATION_STEP_TEXT = {
        LEVEL: "Level",
        LEFT: "Left side down",
        RIGHT: "Right side down",
        NOSEDOWN: "Nose down",
        NOSEUP: "Nose up",
        BACK: "Tail down"
    };
    var CALIBRATION_STEP_CODE_TEXT = {
        1: "Level",
        2: "Left side down",
        3: "Right side down",
        4: "Nose down",
        5: "Nose up",
        6: "Tail down"
    };
    var CALIBRATION_PROGRESS_HINT = {
        IDLE: 0,
        STARTING: 10,
        IN_PROGRESS: 35,
        AWAITING_POSITION: 55,
        CONFIRMING_POSITION: 72,
        SUCCESS: 100,
        FAILED: 100,
        CANCELLED: 0
    };
    var STICK_IDS = {
        left: {
            axis: "leftMotorStick",
            value: "leftMotorValue",
            target: "dashLeftTarget",
            channel: "leftMotorChannel",
            actualLabel: "dashLeftActualLabel",
            actual: "dashLeftActual"
        },
        right: {
            axis: "rightMotorStick",
            value: "rightMotorValue",
            target: "dashRightTarget",
            channel: "rightMotorChannel",
            actualLabel: "dashRightActualLabel",
            actual: "dashRightActual"
        }
    };
    var DEFAULT_PWM_LIMITS = {
        min: 1000,
        max: 2000,
        center: 1500
    };
    var desiredPwm = {
        left: DEFAULT_PWM_LIMITS.center,
        right: DEFAULT_PWM_LIMITS.center
    };
    var pendingMotorPwm = {
        left: null,
        right: null
    };
    var activePointerByStick = {
        left: null,
        right: null
    };
    var sendFrameId = 0;
    var renderFrameId = 0;
    var pendingTelemetry = null;
    var currentCameraSource = "";
    var currentCameraRequestUrl = "";
    var cameraFeedReady = false;
    var cameraRefreshTimer = 0;
    var cameraReconnectTimer = 0;
    var cameraReadyFallbackTimer = 0;
    var cameraStreamController = null;
    var cameraStreamToken = 0;
    var cameraFrameUrl = "";
    var pendingCameraFrame = null;
    var cameraFrameRenderBusy = false;
    var cameraNativeStreamMode = false;
    var visionActive = false;

    function setVisionButtonState(active) {
        var btn = document.getElementById("btnVisionToggle");
        if (!btn) return;
        if (active) {
            btn.classList.add("active");
            btn.textContent = "Vision ON";
        } else {
            btn.classList.remove("active");
            btn.textContent = "Vision";
        }
    }

    function applyVisionState(active) {
        visionActive = Boolean(active);
        setVisionButtonState(visionActive);
        if (visionActive) {
            startVisionPolling();
        } else {
            stopVisionPolling();
            drawVisionDetections(null);
        }
    }

    var visionPollTimer = 0;
    function startVisionPolling() {
        if (visionPollTimer) return;
        visionPollTimer = window.setInterval(function () {
            fetch("/api/vision/state", { cache: "no-store" })
                .then(function (r) { return r.json(); })
                .then(function (b) {
                    if (!b) return;
                    if (typeof b.active === "boolean" && b.active !== visionActive) {
                        applyVisionState(b.active);
                    }
                    if (b.detections) drawVisionDetections(b.detections);
                })
                .catch(function () {});
        }, 250);
    }
    function stopVisionPolling() {
        if (visionPollTimer) { window.clearInterval(visionPollTimer); visionPollTimer = 0; }
    }

    function toggleVision() {
        var target = !visionActive;
        var endpoint = target ? "/api/vision/start" : "/api/vision/stop";
        applyVisionState(target);
        if (!target) drawVisionDetections(null);
        fetch(endpoint, { method: "POST", headers: { "Content-Type": "application/json" } })
            .then(function (resp) { return resp.json().catch(function () { return {}; }); })
            .then(function (body) {
                if (body && typeof body.active === "boolean") applyVisionState(body.active);
            })
            .catch(function () { applyVisionState(!target); });
    }

    function drawVisionDetections(det) {
        var canvas = document.getElementById("visionOverlay");
        if (!canvas) return;
        var ctx = canvas.getContext("2d");
        var feed = document.getElementById("dashCameraFeed");
        var cw = feed ? feed.clientWidth : canvas.clientWidth;
        var ch = feed ? feed.clientHeight : canvas.clientHeight;
        if (canvas.width !== cw || canvas.height !== ch) {
            canvas.width = cw; canvas.height = ch;
        }
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        if (!det || !det.rects || !det.w || !det.h) return;
        var sx = canvas.width / det.w;
        var sy = canvas.height / det.h;
        ctx.lineWidth = Math.max(2, Math.round(canvas.width / 200));
        ctx.strokeStyle = "#00ff66";
        ctx.fillStyle = "rgba(0,255,102,0.85)";
        ctx.font = "bold " + Math.max(12, Math.round(canvas.width / 40)) + "px sans-serif";
        det.rects.forEach(function (r) {
            var x = r[0] * sx, y = r[1] * sy, w = r[2] * sx, h = r[3] * sy;
            ctx.strokeRect(x, y, w, h);
            ctx.save();
            ctx.scale(-1, 1);
            ctx.fillText("PERSON", -(x + w) + 4, y - 4);
            ctx.restore();
        });
    }

    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    function fmt(value, digits) {
        return Number(value || 0).toFixed(digits);
    }

    function formatTemp(value) {
        var numeric = Number(value);
        return Number.isFinite(numeric) ? fmt(numeric, 1) + "°C" : "--";
    }

    function setText(id, value) {
        var element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }

    function poseText(step) {
        var normalized = String(step || "").trim().toUpperCase();
        if (CALIBRATION_STEP_TEXT[normalized]) {
            return CALIBRATION_STEP_TEXT[normalized];
        }

        var numeric = Number(step);
        return CALIBRATION_STEP_CODE_TEXT[numeric] || "Waiting for FCU";
    }

    function calibrationStatusText(calibration) {
        var current = calibration || {};
        var status = String(current.status || "IDLE").trim().toUpperCase();
        if (status === "AWAITING_POSITION" && (current.step || current.stepCode)) {
            return "Waiting for " + poseText(current.step || current.stepCode);
        }
        return CALIBRATION_STATUS_TEXT[status] || status;
    }

    function calibrationModeText(mode) {
        var normalized = String(mode || "IDLE").trim().toUpperCase();
        if (normalized === "ACCEL") {
            return "6-point calibration";
        }
        if (normalized === "LEVEL") {
            return "Level calibration";
        }
        return "Idle";
    }

    function ackText(value) {
        var normalized = String(value || "").trim().toUpperCase();
        return normalized ? normalized.replace(/^MAV_RESULT_/, "") : "--";
    }

    function deriveCalibrationProgress(calibration) {
        var progress = Number(calibration && calibration.progress);
        if (Number.isFinite(progress)) {
            return clamp(progress, 0, 100);
        }

        var status = String((calibration && calibration.status) || "IDLE").trim().toUpperCase();
        return CALIBRATION_PROGRESS_HINT[status] || 0;
    }

    function renderHorizon(attitude) {
        var roll = Number(attitude.roll || 0);
        var pitch = Number(attitude.pitch || 0);
        var world = document.getElementById("horizonWorld");

        if (world) {
            var visualPitch = clamp(pitch, -30, 30) * 2.2;
            var visualRoll = clamp(roll, -65, 65);
            world.style.transform = "translateY(" + visualPitch + "px) rotate(" + (-visualRoll) + "deg)";
        }

        setText("attRoll", fmt(roll, 1) + "°");
        setText("attPitch", fmt(pitch, 1) + "°");
        setText("attYaw", fmt(attitude.yaw, 1) + "°");
    }

    function addLogLine(container, entry) {
        if (!container || !entry) {
            return;
        }

        var line = document.createElement("div");
        var level = String(entry.level || "INFO").toLowerCase();
        var t = new Date(entry.timestamp || Date.now());
        var hh = String(t.getHours()).padStart(2, "0");
        var mm = String(t.getMinutes()).padStart(2, "0");
        var ss = String(t.getSeconds()).padStart(2, "0");

        line.className = "log-line " + level;
        line.textContent = "[" + hh + ":" + mm + ":" + ss + "] [" + (entry.level || "INFO") + "] " + (entry.message || "");
        container.appendChild(line);
    }

    function isControlLog(entry) {
        var level = String(entry && entry.level || "").trim().toUpperCase();
        return level === "COMMAND" || level === "MOTOR" || level === "CRITICAL" || level === "SAFETY";
    }

    function renderLogs() {
        var serverLogsBox = document.getElementById("overviewLogsServer");
        var controlLogsBox = document.getElementById("overviewLogsControl");
        if ((!serverLogsBox && !controlLogsBox) || !window.RoverClient) {
            return;
        }

        var source = window.RoverClient.state.logs || [];
        if (serverLogsBox) {
            serverLogsBox.innerHTML = "";
            source.filter(function (entry) {
                return !isControlLog(entry);
            }).slice(-180).forEach(function (entry) {
                addLogLine(serverLogsBox, entry);
            });
            serverLogsBox.scrollTop = serverLogsBox.scrollHeight;
        }

        if (controlLogsBox) {
            controlLogsBox.innerHTML = "";
            source.filter(function (entry) {
                return isControlLog(entry);
            }).slice(-180).forEach(function (entry) {
                addLogLine(controlLogsBox, entry);
            });
            controlLogsBox.scrollTop = controlLogsBox.scrollHeight;
        }
    }

    function cameraStateLabel(camera) {
        var cameraHostState = (camera && camera.hostState) || {};
        var cameraReachable = Boolean(camera && camera.online) && !(cameraHostState.isLocalName && cameraHostState.resolvable === false);

        if (cameraHostState.matchesLocalHost) {
            return "WRONG HOST";
        }
        if (cameraReachable) {
            return "LIVE";
        }
        if (cameraHostState.isLocalName) {
            return "CHECK HOST";
        }
        return "OFF";
    }

    function cameraMessage(camera) {
        if (camera && camera.reason) {
            return camera.reason;
        }

        if (camera && camera.sourceUrl) {
            return "Feed ready";
        }

        return "No source";
    }

    function cameraBadge(camera) {
        var transport = String((camera && camera.transport) || "camera").toUpperCase();
        var state = cameraStateLabel(camera);
        return transport + " " + state;
    }

    function cameraSourceText(camera) {
        var sourceUrl = String((camera && camera.sourceUrl) || "");
        if (sourceUrl) {
            return sourceUrl;
        }
        return String((camera && camera.host) || "Source pending");
    }

    function cameraTitle(camera) {
        var label = String((camera && camera.label) || "").trim();
        return label || "Camera";
    }

    function clearRenderedLogs() {
        ["overviewLogsServer", "overviewLogsControl"].forEach(function (id) {
            var logsBox = document.getElementById(id);
            if (logsBox) {
                logsBox.innerHTML = "";
            }
        });
    }

    function getPwmLimits() {
        var clientState = window.RoverClient && window.RoverClient.state;
        var pwm = clientState && clientState.limits && clientState.limits.pwm;
        if (!pwm) {
            return DEFAULT_PWM_LIMITS;
        }
        return {
            min: Number.isFinite(Number(pwm.min)) ? Number(pwm.min) : DEFAULT_PWM_LIMITS.min,
            max: Number.isFinite(Number(pwm.max)) ? Number(pwm.max) : DEFAULT_PWM_LIMITS.max,
            center: Number.isFinite(Number(pwm.center)) ? Number(pwm.center) : DEFAULT_PWM_LIMITS.center
        };
    }

    function getRoverChannels() {
        var clientState = window.RoverClient && window.RoverClient.state;
        var channels = clientState && clientState.roverChannels;
        return {
            left: Number(channels && channels.left) || 1,
            right: Number(channels && channels.right) || 3
        };
    }

    function renderStick(side, pwm) {
        var ids = STICK_IDS[side];
        var axis = document.getElementById(ids.axis);
        var limits = getPwmLimits();
        var ratio = clamp((pwm - limits.min) / Math.max(1, limits.max - limits.min), 0, 1);

        if (axis) {
            axis.style.setProperty("--stick-percent", (ratio * 100).toFixed(2) + "%");
            axis.setAttribute("aria-valuenow", String(Math.round(pwm)));
        }

        setText(ids.value, fmt(pwm, 0) + " us");
        setText(ids.target, fmt(pwm, 0));
    }

    function renderStickChannels() {
        var channels = getRoverChannels();
        setText(STICK_IDS.left.channel, "Main" + channels.left + " target");
        setText(STICK_IDS.right.channel, "Main" + channels.right + " target");
        setText(STICK_IDS.left.actualLabel, "Main" + channels.left + " actual");
        setText(STICK_IDS.right.actualLabel, "Main" + channels.right + " actual");
    }

    function renderDesiredPwm() {
        renderStick("left", desiredPwm.left);
        renderStick("right", desiredPwm.right);
    }

    function renderLimits() {
        var box = document.getElementById("limitsBox");
        var channels = getRoverChannels();
        var pwm = getPwmLimits();
        if (!box) {
            return;
        }

        box.textContent = "PWM " + pwm.min + "-" + pwm.max + " (center " + pwm.center + ") | Left motor Main" + channels.left + " | Right motor Main" + channels.right;
        renderStickChannels();
    }

    function queueMotorSend(side, pwm) {
        pendingMotorPwm[side] = pwm;
        if (sendFrameId) {
            return;
        }

        sendFrameId = window.requestAnimationFrame(function () {
            var channels = getRoverChannels();
            var segments = [];
            sendFrameId = 0;

            ["left", "right"].forEach(function (name) {
                if (pendingMotorPwm[name] === null) {
                    return;
                }

                var channel = name === "left" ? channels.left : channels.right;
                var pwmValue = pendingMotorPwm[name];
                pendingMotorPwm[name] = null;
                window.RoverClient.setMotorPwm(channel, pwmValue);
                segments.push("Main" + channel + "=" + fmt(pwmValue, 0) + "us");
            });

            if (segments.length) {
                setText("dashLastCmd", segments.join(" | "));
            }
        });
    }

    function setMotorTarget(side, pwm, sendNow) {
        if (visionActive) return;
        var limits = getPwmLimits();
        var normalized = clamp(Number(pwm || limits.center), limits.min, limits.max);
        desiredPwm[side] = normalized;
        renderStick(side, normalized);
        if (sendNow) {
            queueMotorSend(side, normalized);
        }
    }

    function pwmFromPointerEvent(event, axis) {
        var limits = getPwmLimits();
        var rect = axis.getBoundingClientRect();
        var ratio = clamp((rect.bottom - event.clientY) / Math.max(rect.height, 1), 0, 1);
        return limits.min + ratio * (limits.max - limits.min);
    }

    function pwmFromClientY(clientY, axis) {
        var limits = getPwmLimits();
        var rect = axis.getBoundingClientRect();
        var ratio = clamp((rect.bottom - clientY) / Math.max(rect.height, 1), 0, 1);
        return limits.min + ratio * (limits.max - limits.min);
    }

    function bindMotorStick(side) {
        var axis = document.getElementById(STICK_IDS[side].axis);
        var activeTouchId = null;
        var mouseActive = false;
        if (!axis) {
            return;
        }

        function updateFromPointer(event) {
            setMotorTarget(side, pwmFromPointerEvent(event, axis), true);
        }

        function updateFromClientY(clientY) {
            setMotorTarget(side, pwmFromClientY(clientY, axis), true);
        }

        function getActiveTouch(event) {
            var touches = Array.prototype.slice.call(event.touches || []).concat(Array.prototype.slice.call(event.changedTouches || []));
            return touches.find(function (touch) {
                return touch.identifier === activeTouchId;
            }) || null;
        }

        function releaseStick(pointerId) {
            if (activePointerByStick[side] === null) {
                return;
            }

            activePointerByStick[side] = null;
            axis.classList.remove("active");
            if (typeof pointerId !== "undefined" && axis.hasPointerCapture(pointerId)) {
                axis.releasePointerCapture(pointerId);
            }
        }

        axis.addEventListener("pointerdown", function (event) {
            activePointerByStick[side] = event.pointerId;
            axis.setPointerCapture(event.pointerId);
            axis.classList.add("active");
            updateFromPointer(event);
        });

        axis.addEventListener("pointermove", function (event) {
            if (activePointerByStick[side] !== event.pointerId) {
                return;
            }
            updateFromPointer(event);
        });

        axis.addEventListener("pointerup", function (event) {
            releaseStick(event.pointerId);
        });

        axis.addEventListener("pointercancel", function (event) {
            releaseStick(event.pointerId);
        });

        axis.addEventListener("lostpointercapture", function () {
            if (activePointerByStick[side] !== null) {
                releaseStick();
            }
        });

        axis.addEventListener("touchstart", function (event) {
            var touch = event.changedTouches && event.changedTouches[0];
            if (!touch) {
                return;
            }

            activeTouchId = touch.identifier;
            axis.classList.add("active");
            updateFromClientY(touch.clientY);
            event.preventDefault();
        }, { passive: false });

        axis.addEventListener("touchmove", function (event) {
            var touch = getActiveTouch(event);
            if (!touch) {
                return;
            }

            updateFromClientY(touch.clientY);
            event.preventDefault();
        }, { passive: false });

        axis.addEventListener("touchend", function (event) {
            var touch = getActiveTouch(event);
            if (!touch) {
                return;
            }

            activeTouchId = null;
            axis.classList.remove("active");
            event.preventDefault();
        }, { passive: false });

        axis.addEventListener("touchcancel", function (event) {
            var touch = getActiveTouch(event);
            if (!touch) {
                return;
            }

            activeTouchId = null;
            axis.classList.remove("active");
            event.preventDefault();
        }, { passive: false });

        axis.addEventListener("mousedown", function (event) {
            if (typeof window.PointerEvent !== "undefined") {
                return;
            }

            mouseActive = true;
            axis.classList.add("active");
            updateFromClientY(event.clientY);
            event.preventDefault();
        });

        window.addEventListener("mousemove", function (event) {
            if (!mouseActive || typeof window.PointerEvent !== "undefined") {
                return;
            }

            updateFromClientY(event.clientY);
        });

        window.addEventListener("mouseup", function () {
            if (!mouseActive || typeof window.PointerEvent !== "undefined") {
                return;
            }

            mouseActive = false;
            axis.classList.remove("active");
        });
    }

    function renderCalibration(calibration, armed) {
        var current = calibration || {};
        var status = String(current.status || "IDLE").trim().toUpperCase();
        var canConfirm = !armed && Boolean(current.active) && Number(current.stepCode) >= 1 && Number(current.stepCode) <= 6 && status !== "CONFIRMING_POSITION";
        var progress = deriveCalibrationProgress(current);
        var activeStep = String(current.step || "").trim().toUpperCase();
        var compactHint = "IMU ready";

        if (armed) {
            compactHint = "Disarm for IMU";
        } else if (current.active) {
            compactHint = current.step || current.stepCode ? "Pose: " + poseText(current.step || current.stepCode) : calibrationStatusText(current);
        }

        setText("imuCalMode", calibrationModeText(current.mode));
        setText("imuCalStatus", calibrationStatusText(current));
        setText("imuCalStep", current.step || current.stepCode ? poseText(current.step || current.stepCode) : "Waiting for FCU");
        setText("imuCalAck", ackText(current.lastAckResult));
        setText("imuCalHint", current.instructions || "Waiting for the next FCU pose prompt.");
        setText("imuCalProgressText", fmt(progress, 0) + "%");

        var progressBar = document.getElementById("imuCalProgressBar");
        if (progressBar) {
            progressBar.style.width = progress + "%";
            progressBar.className = "progress-fill status-" + status.toLowerCase();
        }

        document.querySelectorAll(".pose-card[data-step]").forEach(function (card) {
            var step = String(card.getAttribute("data-step") || "").toUpperCase();
            card.classList.toggle("active", step === activeStep && status !== "SUCCESS");
        });

        var confirmButton = document.getElementById("imuConfirmPose");
        if (confirmButton) {
            confirmButton.disabled = !canConfirm;
            confirmButton.textContent = canConfirm ? "Confirm " + poseText(current.step || current.stepCode) : "Waiting for FCU";
        }

        var startAccelButton = document.getElementById("imuStartAccel");
        if (startAccelButton) {
            startAccelButton.disabled = armed;
        }

        var startLevelButton = document.getElementById("imuStartLevel");
        if (startLevelButton) {
            startLevelButton.disabled = armed;
        }

        setText("dashImuHint", compactHint);
    }

    function renderTelemetry(telemetry) {
        var calibration = telemetry.imuCalibration || {};
        var channels = getRoverChannels();
        var servoOutputs = telemetry.servoOutputs || {};
        var temperature = telemetry.temperature || {};
        var leftChannelKey = "ch" + channels.left;
        var rightChannelKey = "ch" + channels.right;
        var speed = Math.sqrt(
            Math.pow(Number(telemetry.velocity.vx || 0), 2) +
            Math.pow(Number(telemetry.velocity.vy || 0), 2)
        );

        renderHorizon(telemetry.attitude || {});
        setText("dashHeroArmed", telemetry.armed ? "ARMED" : "SAFE");
        setText("dashHeroMode", telemetry.flightMode || "MANUAL");
        setText("dashHeroModeMirror", telemetry.flightMode || "MANUAL");
        setText("attModeBadge", telemetry.flightMode || "MANUAL");
        setText("dashGps", fmt((telemetry.gps || {}).satellites, 0) + " sat");
        setText("dashSpeed", fmt(speed, 2) + " m/s");
        setText("dashHeading", fmt((telemetry.attitude || {}).yaw, 1) + "°");
        setText("dashBattery", fmt((telemetry.battery || {}).voltage, 1) + "V / " + fmt((telemetry.battery || {}).percentage, 0) + "%");
        setText("dashHostTemp", formatTemp(temperature.hostBoard));
        setText("dashFcuTemp", formatTemp(temperature.flightController));
        setText("attPitchMirror", fmt((telemetry.attitude || {}).pitch, 1) + "°");
        setText("attRollMirror", fmt((telemetry.attitude || {}).roll, 1) + "°");
        setText("dashLeftActual", fmt(servoOutputs[leftChannelKey], 0));
        setText("dashRightActual", fmt(servoOutputs[rightChannelKey], 0));

        renderCalibration(calibration, telemetry.armed);
    }

    function scheduleTelemetryRender(telemetry) {
        pendingTelemetry = telemetry;
        if (renderFrameId) {
            return;
        }

        renderFrameId = window.requestAnimationFrame(function () {
            renderFrameId = 0;
            if (pendingTelemetry) {
                renderTelemetry(pendingTelemetry);
            }
        });
    }

    function renderConnectionSummary() {
        if (!window.RoverClient) {
            return;
        }

        var clientState = window.RoverClient.state;
        var pixhawkOnline = clientState.pixhawkStatus === "connected" || Boolean(clientState.isConnected);
        setText("dashHeroPixhawk", pixhawkOnline ? "LIVE" : "WAIT");
    }

    function stopCameraRefreshLoop() {
        if (cameraRefreshTimer) {
            window.clearInterval(cameraRefreshTimer);
            cameraRefreshTimer = 0;
        }
    }

    function stopCameraReconnectLoop() {
        if (cameraReconnectTimer) {
            window.clearTimeout(cameraReconnectTimer);
            cameraReconnectTimer = 0;
        }
    }

    function stopCameraReadyFallback() {
        if (cameraReadyFallbackTimer) {
            window.clearTimeout(cameraReadyFallbackTimer);
            cameraReadyFallbackTimer = 0;
        }
    }

    function stopCameraStreamLoop() {
        cameraStreamToken += 1;
        pendingCameraFrame = null;
        cameraFrameRenderBusy = false;
        cameraNativeStreamMode = false;

        if (cameraStreamController) {
            cameraStreamController.abort();
            cameraStreamController = null;
        }

        if (cameraFrameUrl) {
            URL.revokeObjectURL(cameraFrameUrl);
            cameraFrameUrl = "";
        }
    }

    function concatBytes(left, right) {
        if (!left || !left.length) {
            return right;
        }
        if (!right || !right.length) {
            return left;
        }

        var merged = new Uint8Array(left.length + right.length);
        merged.set(left, 0);
        merged.set(right, left.length);
        return merged;
    }

    function findMarker(buffer, first, second, startIndex) {
        for (var index = Math.max(0, startIndex || 0); index < buffer.length - 1; index += 1) {
            if (buffer[index] === first && buffer[index + 1] === second) {
                return index;
            }
        }
        return -1;
    }

    function renderPendingCameraFrame(feed) {
        if (!feed || !pendingCameraFrame) {
            cameraFrameRenderBusy = false;
            return;
        }

        cameraFrameRenderBusy = true;
        var frameBytes = pendingCameraFrame;
        pendingCameraFrame = null;
        var nextFrameUrl = URL.createObjectURL(new Blob([frameBytes], { type: "image/jpeg" }));
        var previousFrameUrl = cameraFrameUrl;

        cameraFrameUrl = nextFrameUrl;
        feed.src = nextFrameUrl;
        cameraFeedReady = true;
        renderPeripheralStatus();

        if (previousFrameUrl) {
            URL.revokeObjectURL(previousFrameUrl);
        }

        window.requestAnimationFrame(function () {
            if (pendingCameraFrame) {
                renderPendingCameraFrame(feed);
                return;
            }
            cameraFrameRenderBusy = false;
        });
    }

    function queueCameraFrame(feed, frameBytes) {
        pendingCameraFrame = frameBytes;
        if (!cameraFrameRenderBusy) {
            renderPendingCameraFrame(feed);
        }
    }

    async function startLowLatencyCameraStream(feed, sourceUrl) {
        stopCameraStreamLoop();

        if (!feed || !sourceUrl || !window.fetch) {
            return;
        }

        cameraNativeStreamMode = false;
        var controller = new AbortController();
        var token = cameraStreamToken + 1;
        cameraStreamToken = token;
        cameraStreamController = controller;
        currentCameraRequestUrl = buildCameraStreamUrl(sourceUrl);

        try {
            var response = await fetch(currentCameraRequestUrl, {
                method: "GET",
                cache: "no-store",
                mode: "cors",
                signal: controller.signal
            });

            if (!response.ok || !response.body) {
                throw new Error("Camera stream unavailable.");
            }

            var reader = response.body.getReader();
            var buffer = new Uint8Array(0);

            while (cameraStreamToken === token) {
                var result = await reader.read();
                if (result.done) {
                    throw new Error("Camera stream ended.");
                }

                buffer = concatBytes(buffer, result.value);

                while (true) {
                    var start = findMarker(buffer, 0xff, 0xd8, 0);
                    if (start === -1) {
                        if (buffer.length > 262144) {
                            buffer = buffer.slice(buffer.length - 2);
                        }
                        break;
                    }

                    var end = findMarker(buffer, 0xff, 0xd9, start + 2);
                    if (end === -1) {
                        if (start > 0) {
                            buffer = buffer.slice(start);
                        }
                        break;
                    }

                    queueCameraFrame(feed, buffer.slice(start, end + 2));
                    buffer = buffer.slice(end + 2);
                }
            }
        } catch (error) {
            if (controller.signal.aborted || cameraStreamToken !== token) {
                return;
            }

            if (!cameraNativeStreamMode) {
                startNativeCameraStream(feed, sourceUrl);
                scheduleCameraReadyFallback(window.RoverClient ? window.RoverClient.state.camera : {});
                return;
            }

            cameraFeedReady = false;
            stopCameraReadyFallback();
            scheduleCameraReconnect(window.RoverClient ? window.RoverClient.state.camera : {});
            renderPeripheralStatus();
        }
    }

    function buildCameraFrameUrl(sourceUrl) {
        var separator = sourceUrl.indexOf("?") === -1 ? "?" : "&";
        return sourceUrl + separator + "t=" + Date.now();
    }

    function buildCameraStreamUrl(sourceUrl) {
        var separator = sourceUrl.indexOf("?") === -1 ? "?" : "&";
        return sourceUrl + separator + "stream=" + Date.now();
    }

    function browserNeedsNativeCameraStream() {
        var userAgent = String(window.navigator && window.navigator.userAgent || "");
        var coarsePointer = Boolean(window.matchMedia && window.matchMedia("(pointer: coarse)").matches);
        var hasStreamReader = typeof window.fetch === "function"
            && typeof window.AbortController !== "undefined"
            && typeof window.ReadableStream !== "undefined"
            && typeof Uint8Array !== "undefined";

        if (!hasStreamReader) {
            return true;
        }

        return coarsePointer || /Android|iPhone|iPad|iPod|Mobile|HarmonyOS/i.test(userAgent);
    }

    function startNativeCameraStream(feed, sourceUrl) {
        stopCameraStreamLoop();

        if (!feed || !sourceUrl) {
            return;
        }

        cameraNativeStreamMode = true;
        currentCameraRequestUrl = buildCameraStreamUrl(sourceUrl);
        feed.src = currentCameraRequestUrl;
    }

    function applyCameraSource(feed, sourceType, sourceUrl) {
        if (!feed) {
            return;
        }

        if (sourceType === "image") {
            stopCameraStreamLoop();
            cameraNativeStreamMode = false;
            currentCameraRequestUrl = buildCameraFrameUrl(sourceUrl);
            feed.src = currentCameraRequestUrl;
            return;
        }

        if (browserNeedsNativeCameraStream()) {
            startNativeCameraStream(feed, sourceUrl);
            return;
        }

        startLowLatencyCameraStream(feed, sourceUrl);
    }

    function scheduleCameraReadyFallback(camera) {
        stopCameraReadyFallback();

        if (!camera || String(camera.sourceType || "").toLowerCase() === "image" || !currentCameraSource) {
            return;
        }

        cameraReadyFallbackTimer = window.setTimeout(function () {
            if (!currentCameraSource) {
                return;
            }

            cameraFeedReady = true;
            renderPeripheralStatus();
        }, 900);
    }

    function scheduleCameraReconnect(camera) {
        stopCameraReconnectLoop();

        if (!camera || String(camera.sourceType || "").toLowerCase() === "image" || !currentCameraSource) {
            return;
        }

        cameraReconnectTimer = window.setTimeout(function () {
            var feed = document.getElementById("dashCameraFeed");
            if (!feed || !currentCameraSource) {
                return;
            }

            cameraFeedReady = false;
            applyCameraSource(feed, "mjpeg", currentCameraSource);
            renderPeripheralStatus();
        }, 1000);
    }

    function scheduleCameraRefresh(camera) {
        stopCameraRefreshLoop();

        if (!camera || String(camera.sourceType || "").toLowerCase() !== "image" || !currentCameraSource) {
            return;
        }

        var refreshMs = Math.max(100, Number(camera.refreshMs || 1500));
        cameraRefreshTimer = window.setInterval(function () {
            var feed = document.getElementById("dashCameraFeed");
            if (!feed || !currentCameraSource) {
                return;
            }

            feed.src = buildCameraFrameUrl(currentCameraSource);
        }, refreshMs);
    }

    function renderPeripheralStatus() {
        if (!window.RoverClient) {
            return;
        }

        var camera = window.RoverClient.state.camera || {};
        var sourceType = String(camera.sourceType || "").toLowerCase();

        setText("dashCameraTitle", cameraTitle(camera));
        setText("dashCameraBadge", cameraBadge(camera));
        setText("dashCameraSource", cameraSourceText(camera));
        setText("dashCameraMessage", cameraMessage(camera));

        var feed = document.getElementById("dashCameraFeed");
        var placeholder = document.getElementById("dashCameraPlaceholder");
        var sourceUrl = String(camera.sourceUrl || "");
        if (feed && currentCameraSource !== sourceUrl) {
            currentCameraSource = sourceUrl;
            cameraFeedReady = false;
            stopCameraReconnectLoop();
            stopCameraReadyFallback();
            applyCameraSource(feed, sourceType, sourceUrl);
            scheduleCameraRefresh(camera);
            scheduleCameraReadyFallback(camera);
        }

        if (!sourceUrl) {
            stopCameraRefreshLoop();
            stopCameraReconnectLoop();
            stopCameraReadyFallback();
            stopCameraStreamLoop();
            currentCameraRequestUrl = "";
            if (feed) {
                feed.removeAttribute("src");
            }
        }

        if (feed) {
            feed.style.display = sourceUrl && cameraFeedReady ? "block" : "none";
        }

        if (placeholder) {
            placeholder.style.display = sourceUrl && cameraFeedReady ? "none" : "flex";
        }
    }

    function openImuModal() {
        var modal = document.getElementById("imuModal");
        if (!modal) {
            return;
        }

        modal.classList.add("open");
        modal.setAttribute("aria-hidden", "false");
    }

    function closeImuModal() {
        var modal = document.getElementById("imuModal");
        if (!modal) {
            return;
        }

        modal.classList.remove("open");
        modal.setAttribute("aria-hidden", "true");
    }

    document.addEventListener("DOMContentLoaded", function () {
        if (!window.RoverClient) {
            return;
        }

        bindMotorStick("left");
        bindMotorStick("right");

        var btnArm = document.getElementById("btnArm");
        if (btnArm) {
            btnArm.addEventListener("click", function () {
                window.RoverClient.arm();
            });
        }

        var btnDisarm = document.getElementById("btnDisarm");
        if (btnDisarm) {
            btnDisarm.addEventListener("click", function () {
                window.RoverClient.disarm();
            });
        }

        var btnEmergency = document.getElementById("btnEmergency");
        if (btnEmergency) {
            btnEmergency.addEventListener("click", function () {
                if (window.confirm("Trigger emergency stop? This will drive outputs to their minimum state.")) {
                    window.RoverClient.emergencyStop().catch(function () {});
                }
            });
        }

        var btnReset = document.getElementById("ctrlReset");
        if (btnReset) {
            btnReset.addEventListener("click", function () {
                var center = getPwmLimits().center;
                setMotorTarget("left", center, true);
                setMotorTarget("right", center, true);
            });
        }

        var btnImuModal = document.getElementById("btnImuModal");
        if (btnImuModal) {
            btnImuModal.addEventListener("click", openImuModal);
        }

        var btnVision = document.getElementById("btnVisionToggle");
        if (btnVision) {
            btnVision.addEventListener("click", toggleVision);
        }
        fetch("/api/vision/state").then(function (r) { return r.json(); }).then(function (b) {
            if (b && typeof b.active === "boolean") applyVisionState(b.active);
        }).catch(function () {});

        var cameraFeed = document.getElementById("dashCameraFeed");
        if (cameraFeed) {
            cameraFeed.addEventListener("load", function () {
                stopCameraReconnectLoop();
                stopCameraReadyFallback();
                cameraFeedReady = true;
                renderPeripheralStatus();
            });
            cameraFeed.addEventListener("error", function () {
                cameraFeedReady = false;
                stopCameraReadyFallback();
                scheduleCameraReconnect(window.RoverClient ? window.RoverClient.state.camera : {});
                renderPeripheralStatus();
            });
        }

        var btnImuClose = document.getElementById("imuModalClose");
        if (btnImuClose) {
            btnImuClose.addEventListener("click", closeImuModal);
        }

        var imuModal = document.getElementById("imuModal");
        if (imuModal) {
            imuModal.addEventListener("click", function (event) {
                if (event.target === imuModal) {
                    closeImuModal();
                }
            });
        }

        document.addEventListener("keydown", function (event) {
            if (event.key === "Escape") {
                closeImuModal();
            }
        });

        var startAccelButton = document.getElementById("imuStartAccel");
        if (startAccelButton) {
            startAccelButton.addEventListener("click", function () {
                window.RoverClient.startImuCalibration("ACCEL").catch(function () {});
            });
        }

        var startLevelButton = document.getElementById("imuStartLevel");
        if (startLevelButton) {
            startLevelButton.addEventListener("click", function () {
                window.RoverClient.startImuCalibration("LEVEL").catch(function () {});
            });
        }

        var confirmButton = document.getElementById("imuConfirmPose");
        if (confirmButton) {
            confirmButton.addEventListener("click", function () {
                var calibration = window.RoverClient.state.telemetry.imuCalibration || {};
                window.RoverClient.confirmImuCalibration(calibration.stepCode).catch(function () {});
            });
        }

        var logClearButton = document.getElementById("logClear");
        if (logClearButton) {
            logClearButton.addEventListener("click", function () {
                clearRenderedLogs();
            });
        }

        window.RoverClient.on("limits", function () {
            renderLimits();
            renderDesiredPwm();
        });

        window.RoverClient.on("roverChannels", function () {
            renderLimits();
            renderTelemetry(window.RoverClient.state.telemetry);
        });

        window.RoverClient.on("telemetry", function (telemetry) {
            scheduleTelemetryRender(telemetry);
        });

        window.RoverClient.on("pixhawkConnection", renderConnectionSummary);
        window.RoverClient.on("status", function () {
            renderConnectionSummary();
            renderPeripheralStatus();
        });

        window.RoverClient.on("motorUpdate", function (payload) {
            if (!payload) {
                return;
            }
            setText("dashLastCmd", "Main" + payload.channel + "=" + fmt(payload.pwm, 0) + "us");
        });

        window.RoverClient.on("log", renderLogs);
        window.RoverClient.on("logsReplace", renderLogs);

        renderDesiredPwm();
        renderLimits();
        renderConnectionSummary();
        renderPeripheralStatus();
        renderTelemetry(window.RoverClient.state.telemetry);
        window.RoverClient.fetchLogs(180);

        window.addEventListener("beforeunload", function () {
            if (renderFrameId) {
                window.cancelAnimationFrame(renderFrameId);
                renderFrameId = 0;
            }
            stopCameraRefreshLoop();
        });
    });
})();
