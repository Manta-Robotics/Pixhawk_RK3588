(function () {
    function requestedTrackModeFromLocation() {
        try {
            var requestedMode = new URLSearchParams(window.location.search).get("mode");
            if (/^(beacon|uwb)$/i.test(String(requestedMode || ""))) return "beacon";
        } catch (error) {
            // Ignore malformed or unsupported query strings and keep the normal click mode.
        }
        return "";
    }
    var mode = "click";
    var trackMode = "face";
    var runningTrackMode = "face";
    var trackingActive = false;
    var trackBusy = false;
    var recordBusy = false;
    var osdBusy = false;
    var connectBusy = false;
    var stopBusy = false;
    var homeBusy = false;
    var recordingActive = false;
    var recordingName = "";
    var connected = false;
    var videoTransport = "rtsp";
    var lastTarget = { x: 0, y: 0 };
    var trackTarget = null;
    var trackStatus = { locked: false, status: "idle", message: "idle" };
    var hasClickMarker = false;
    var clickMarker = { u: 0.5, v: 0.5 };
    var guideCenter = { u: 0.5, v: 0.5 };
    var guideAnimating = false;
    var guideFrame = 0;
    var guideStart = 0;
    var guideDurationMs = 1600;
    var mirrorX = true;
    var cameraRetryTimer = null;
    var cameraRetryDelayMs = 500;
    var cameraSource = "/api/gimbal/stream";
    var cameraState = "loading";

    function $(id) { return document.getElementById(id); }
    function setText(id, value) { var el = $(id); if (el) el.textContent = value; }
    function setStateClass(element, value) {
        if (!element) return;
        ["is-live", "is-loading", "is-warning", "is-offline"].forEach(function (name) { element.classList.remove(name); });
        element.classList.add(value);
    }
    function setCameraState(nextState, detail) {
        cameraState = nextState;
        var card = $("gimbalVideoCard");
        var badge = $("gimbalVideoBadge");
        var button = $("gimbalRefreshVideo");
        var title = $("gimbalCameraPlaceholderTitle");
        var labels = {
            live: { text: "画面正常", badge: "LIVE", className: "is-live", title: "云台画面" },
            loading: { text: "加载中", badge: "LOADING", className: "is-loading", title: "画面加载中" },
            refreshing: { text: "正在刷新", badge: "REFRESH", className: "is-loading", title: "正在刷新画面" },
            reconnecting: { text: "画面中断 · 重连中", badge: "RECONNECT", className: "is-warning", title: "画面连接中断" },
            offline: { text: "画面不可用", badge: "OFFLINE", className: "is-offline", title: "暂无云台画面" }
        };
        var item = labels[nextState] || labels.offline;
        setText("gimbalVideoStatus", item.text);
        if (badge) {
            setStateClass(badge, item.className);
            var badgeText = badge.querySelector("b");
            if (badgeText) badgeText.textContent = item.badge;
        }
        setStateClass(card, item.className);
        if (title) title.textContent = item.title;
        if (detail) setText("gimbalCameraMessage", detail);
        if (button) {
            button.disabled = nextState === "refreshing";
            button.classList.toggle("is-refreshing", nextState === "refreshing");
            button.textContent = nextState === "refreshing" ? "刷新中..." : "刷新画面";
        }
    }
    function updateControlState(state, backendOnline) {
        var card = $("gimbalControlCard");
        if (!backendOnline) {
            setText("gimbalStatus", "后端未连接");
            setText("gimbalStatusDetail", "无法读取云台状态");
            setStateClass(card, "is-offline");
            connected = false;
            renderControls();
            return;
        }
        var linkStatus = String(state.linkStatus || "offline");
        if (state.connected || linkStatus === "feedback") {
            setText("gimbalStatus", "已连接");
            setText("gimbalStatusDetail", "UART 正常 · 云台反馈在线");
            setStateClass(card, "is-live");
        } else if (state.portOpen || linkStatus === "port_only") {
            setText("gimbalStatus", "等待云台反馈");
            setText("gimbalStatusDetail", state.lastError || "串口已打开，尚未收到有效回传");
            setStateClass(card, "is-warning");
        } else {
            setText("gimbalStatus", "未连接");
            setText("gimbalStatusDetail", state.lastError || "点击 Connect 建立控制链路");
            setStateClass(card, "is-offline");
        }
    }
    function normalizeTrackMode(value) {
        var normalized = String(value || "").toLowerCase();
        if (normalized === "beacon" || normalized === "uwb") return "beacon";
        return normalized === "swimmer" ? "swimmer" : "face";
    }
    function trackLabel(value) {
        var normalized = normalizeTrackMode(value);
        if (normalized === "beacon") return "Beacon";
        return normalized === "swimmer" ? "Swimmer" : "Face";
    }
    function lostMessage(value) {
        if (normalizeTrackMode(value) === "beacon") return "UWB signal stale";
        return normalizeTrackMode(value) === "swimmer" ? "can not find swimmer" : "CAN NOT FIND FACE";
    }
    function activeTrackMode() {
        return trackingActive ? runningTrackMode : trackMode;
    }
    function renderControls() {
        var clickBtn = $("gimbalModeClick");
        var faceBtn = $("gimbalModeFace");
        var swimmerBtn = $("gimbalModeSwimmer");
        var beaconBtn = $("gimbalModeBeacon");
        var currentMode = activeTrackMode();
        if (clickBtn) clickBtn.classList.toggle("active", mode === "click" && !trackingActive);
        if (faceBtn) faceBtn.classList.toggle("active", mode === "track" && trackMode === "face");
        if (swimmerBtn) swimmerBtn.classList.toggle("active", mode === "track" && trackMode === "swimmer");
        if (beaconBtn) beaconBtn.classList.toggle("active", mode === "track" && trackMode === "beacon");
        setText("gimbalModeText", mode === "track" ? trackLabel(currentMode) + (trackingActive ? " Tracking" : " Ready") : "Click");
        setText("gimbalModeBadge", mode === "track" ? trackLabel(currentMode).toUpperCase() : "CLICK");
        var connectBtn = $("gimbalConnect");
        if (connectBtn) {
            connectBtn.textContent = connectBusy ? "Working..." : (connected ? "Disconnect" : "Connect");
            connectBtn.classList.toggle("active", connected);
            connectBtn.disabled = connectBusy;
        }
        var stopBtn = $("gimbalStop");
        if (stopBtn) {
            stopBtn.textContent = stopBusy ? "Stopping..." : "Stop";
            stopBtn.disabled = stopBusy;
        }
        var homeBtn = $("gimbalHome");
        if (homeBtn) {
            homeBtn.textContent = homeBusy ? "Centering..." : "Home";
            homeBtn.disabled = homeBusy;
        }
        var recordBtn = $("gimbalRecordToggle");
        if (recordBtn) {
            recordBtn.textContent = recordBusy ? "Working..." : (recordingActive ? "Stop Recording" : "Record");
            recordBtn.classList.toggle("active", recordingActive);
            recordBtn.disabled = recordBusy;
            recordBtn.title = recordingActive && recordingName ? recordingName : "Record clean gimbal video";
        }
        if (clickBtn) clickBtn.disabled = trackBusy;
        if (faceBtn) faceBtn.disabled = trackBusy;
        if (swimmerBtn) swimmerBtn.disabled = trackBusy;
        if (beaconBtn) beaconBtn.disabled = trackBusy;
        var osdToggle = $("gimbalOsdToggle");
        if (osdToggle) osdToggle.disabled = osdBusy;
    }
    function postJson(url, body) {
        return fetch(url, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: body ? JSON.stringify(body) : "{}"
        }).then(function (response) { return response.json().catch(function () { return {}; }); });
    }
    function setMode(nextMode, nextTrackMode) {
        mode = nextMode === "track" ? "track" : "click";
        if (nextTrackMode) trackMode = normalizeTrackMode(nextTrackMode);
        renderControls();
    }
    function updateState(state) {
        if (!state) return;
        connected = Boolean(state.connected);
        trackingActive = Boolean(state.trackingActive);
        if (state.trackMode) {
            runningTrackMode = normalizeTrackMode(state.trackMode);
            if (trackingActive || mode !== "track") trackMode = runningTrackMode;
        }
        if (trackingActive && mode !== "track") {
            hasClickMarker = false;
            guideAnimating = false;
            setMode("track", state.trackMode || trackMode);
        }
        videoTransport = state.videoTransport || "rtsp";
        setText("gimbalSerial", (state.serialPort || "--") + " @ " + (state.baudRate || "--"));
        updateControlState(state, true);
        setText("gimbalCommand", state.lastCommand || "Idle");
        setText("gimbalVideo", state.videoInput || state.videoSource || "--");
        setText("gimbalCameraSource", state.videoSource || "/api/camera/stream");
        if (state.lastError && state.lastError.indexOf("not present") !== -1) {
            setText("gimbalCameraMessage", "Serial device missing. Reboot after UART3 setup, then reconnect.");
        }
        renderControls();
        if (state.trackStatus) {
            trackStatus = state.trackStatus;
            if (!trackStatus.locked) {
                trackTarget = null;
                if (mode === "track") setText("gimbalTarget", trackStatus.message || lostMessage(activeTrackMode()));
            }
            drawOverlay();
        }
        if (state.lastTarget && mode === "track" && !hasClickMarker) {
            if (state.lastTarget.locked) trackTarget = state.lastTarget;
            var targetX = Number.isFinite(Number(state.lastTarget.x)) ? Number(state.lastTarget.x) : Number(state.lastTarget.dx || 0);
            var targetY = Number.isFinite(Number(state.lastTarget.y)) ? Number(state.lastTarget.y) : Number(state.lastTarget.dy || 0);
            var predicting = Boolean(state.lastTarget.coasting);
            var targetMode = normalizeTrackMode(state.lastTarget.mode || trackMode);
            if (state.lastTarget.mode) {
                runningTrackMode = targetMode;
                if (trackingActive) trackMode = targetMode;
            }
            lastTarget = { x: targetX, y: targetY };
            setText("gimbalModeBadge", predicting ? "PREDICT" : trackLabel(targetMode).toUpperCase());
            if (targetMode === "beacon" && state.lastTarget.locked) {
                var yawError = Number(state.lastTarget.errorYawDeg || 0).toFixed(1);
                var pitchError = Number(state.lastTarget.errorPitchDeg || 0).toFixed(1);
                setText("gimbalTarget", (state.lastTarget.dryRun ? "DRY RUN " : "BEACON ") + "Yaw " + yawError + "° / Pitch " + pitchError + "°");
                return;
            }
            setText("gimbalTarget", state.lastTarget.locked ? (predicting ? "PREDICT " : trackLabel(targetMode).toUpperCase() + " ") + Math.round(targetX) + " / " + Math.round(targetY) : (state.lastTarget.message || lostMessage(targetMode)));
            renderControls();
            drawOverlay();
        }
    }
    function refreshState() {
        fetch("/api/gimbal/state", { cache: "no-store" })
            .then(function (r) { if (!r.ok) throw new Error("state unavailable"); return r.json(); })
            .then(function (body) { if (body && body.state) updateState(body.state); })
            .catch(function () { updateControlState({}, false); });
    }
    function refreshRecordingState() {
        fetch("/api/gimbal/recording/state", { cache: "no-store" })
            .then(function (r) { return r.json(); })
            .then(function (body) {
                var state = body && body.state ? body.state : {};
                recordingActive = Boolean(state.active);
                recordingName = state.current && state.current.name ? state.current.name : "";
                renderControls();
            })
            .catch(function () {});
    }
    function startCamera(source, reason) {
        var img = $("gimbalCameraFeed");
        var placeholder = $("gimbalCameraPlaceholder");
        var url = source || "/api/gimbal/stream";
        if (!img) return;
        cameraSource = url;
        if (placeholder) placeholder.style.display = "flex";
        setCameraState(reason === "manual" ? "refreshing" : "loading", reason === "manual" ? "仅重新建立云台画面数据流，不改变控制和跟踪状态。" : "正在连接云台 RTSP 画面数据流。");
        if (cameraRetryTimer) {
            window.clearTimeout(cameraRetryTimer);
            cameraRetryTimer = null;
        }
        img.onload = function () {
            cameraRetryDelayMs = 500;
            if (cameraRetryTimer) window.clearTimeout(cameraRetryTimer);
            cameraRetryTimer = null;
            if (placeholder) placeholder.style.display = "none";
            setCameraState("live");
            drawOverlay();
        };
        img.onerror = function () {
            if (placeholder) placeholder.style.display = "flex";
            setCameraState("reconnecting", "画面数据已中断，正在自动重新连接。");
            if (cameraRetryTimer) return;
            cameraRetryTimer = window.setTimeout(function () {
                cameraRetryTimer = null;
                img.src = cameraSource + (cameraSource.indexOf("?") === -1 ? "?" : "&") + "gimbal=" + Date.now();
                cameraRetryDelayMs = Math.min(cameraRetryDelayMs * 2, 4000);
            }, cameraRetryDelayMs);
        };
        img.src = url + (url.indexOf("?") === -1 ? "?" : "&") + "gimbal=" + Date.now();
    }
    function refreshCameraStream() {
        var img = $("gimbalCameraFeed");
        if (!img || cameraState === "refreshing") return;
        if (cameraRetryTimer) {
            window.clearTimeout(cameraRetryTimer);
            cameraRetryTimer = null;
        }
        img.removeAttribute("src");
        window.setTimeout(function () { startCamera(cameraSource, "manual"); }, 60);
    }
    function drawTrackOverlay(ctx, width, height) {
        if (mode !== "track" || !trackingActive) return;
        if (trackStatus && trackStatus.locked && trackTarget) {
            var view = $("gimbalView");
            var feed = $("gimbalCameraFeed");
            var frameW = Number(trackTarget.frame_w || trackTarget.frameW || 1920);
            var frameH = Number(trackTarget.frame_h || trackTarget.frameH || 1080);
            var mapping = getVideoMapping(view, feed, frameW, frameH);
            var sourceX = Number(trackTarget.x || 0);
            var displayX = mirrorX ? (frameW - sourceX) : sourceX;
            var tx = mapping.offsetX + (displayX / Math.max(frameW, 1)) * mapping.drawnWidth;
            var ty = mapping.offsetY + (Number(trackTarget.y || 0) / Math.max(frameH, 1)) * mapping.drawnHeight;
            var bw = Math.max(48, (Number(trackTarget.w || 120) / Math.max(frameW, 1)) * mapping.drawnWidth);
            var bh = Math.max(48, (Number(trackTarget.h || 120) / Math.max(frameH, 1)) * mapping.drawnHeight);
            var x = Math.max(4, Math.min(width - bw - 4, tx - bw / 2));
            var y = Math.max(4, Math.min(height - bh - 4, ty - bh / 2));
            ctx.save();
            ctx.strokeStyle = "rgba(51, 255, 132, 0.98)";
            ctx.lineWidth = 2;
            var corner = Math.max(14, Math.min(34, bw * 0.28, bh * 0.28));
            ctx.beginPath();
            ctx.moveTo(x, y + corner); ctx.lineTo(x, y); ctx.lineTo(x + corner, y);
            ctx.moveTo(x + bw - corner, y); ctx.lineTo(x + bw, y); ctx.lineTo(x + bw, y + corner);
            ctx.moveTo(x + bw, y + bh - corner); ctx.lineTo(x + bw, y + bh); ctx.lineTo(x + bw - corner, y + bh);
            ctx.moveTo(x + corner, y + bh); ctx.lineTo(x, y + bh); ctx.lineTo(x, y + bh - corner);
            ctx.stroke();
            ctx.restore();
            return;
        }
        var label = trackStatus && trackStatus.message ? String(trackStatus.message).toUpperCase() : lostMessage(activeTrackMode()).toUpperCase();
        ctx.save();
        ctx.fillStyle = "rgba(0, 0, 0, 0.72)";
        ctx.strokeStyle = "rgba(255, 91, 91, 0.95)";
        ctx.lineWidth = 4;
        var boxW = Math.min(width - 32, 420);
        var boxH = 76;
        var boxX = (width - boxW) / 2;
        var boxY = Math.max(28, height * 0.18);
        ctx.fillRect(boxX, boxY, boxW, boxH);
        ctx.strokeRect(boxX, boxY, boxW, boxH);
        ctx.fillStyle = "#ffdfdf";
        ctx.font = "bold 24px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(label, width / 2, boxY + boxH / 2);
        ctx.restore();
    }
    function drawOverlay() {
        var canvas = $("gimbalOverlay");
        var view = $("gimbalView");
        if (!canvas || !view) return;
        var width = view.clientWidth;
        var height = view.clientHeight;
        if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
        var ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, width, height);
        var cx = width / 2;
        var cy = height / 2;
        ctx.strokeStyle = "rgba(124,224,255,0.85)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx - 24, cy); ctx.lineTo(cx + 24, cy);
        ctx.moveTo(cx, cy - 24); ctx.lineTo(cx, cy + 24);
        ctx.stroke();
        drawTrackOverlay(ctx, width, height);
        if (hasClickMarker) {
            var tx = clickMarker.u * width;
            var ty = clickMarker.v * height;
            var gx = width / 2;
            var gy = height / 2;
            ctx.strokeStyle = "rgba(126,217,165,0.95)";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(tx, ty, 12, 0, Math.PI * 2);
            ctx.moveTo(gx, gy); ctx.lineTo(tx, ty);
            ctx.stroke();
            ctx.fillStyle = "rgba(126,217,165,0.95)";
            ctx.beginPath();
            ctx.arc(gx, gy, 5, 0, Math.PI * 2);
            ctx.fill();
        }
    }
    function getVideoMapping(view, feed, sourceWidthOverride, sourceHeightOverride) {
        var rect = view.getBoundingClientRect();
        var sourceWidth = Number(sourceWidthOverride) || Number(feed && feed.naturalWidth) || 1920;
        var sourceHeight = Number(sourceHeightOverride) || Number(feed && feed.naturalHeight) || 1080;
        var containerWidth = Math.max(rect.width, 1);
        var containerHeight = Math.max(rect.height, 1);
        var scale = Math.max(containerWidth / Math.max(sourceWidth, 1), containerHeight / Math.max(sourceHeight, 1));
        var drawnWidth = sourceWidth * scale;
        var drawnHeight = sourceHeight * scale;
        var offsetX = (containerWidth - drawnWidth) / 2;
        var offsetY = (containerHeight - drawnHeight) / 2;
        return {
            rect: rect,
            sourceWidth: sourceWidth,
            sourceHeight: sourceHeight,
            containerWidth: containerWidth,
            containerHeight: containerHeight,
            drawnWidth: drawnWidth,
            drawnHeight: drawnHeight,
            offsetX: offsetX,
            offsetY: offsetY
        };
    }
    function handleClick(event) {
        if (mode !== "click") return;
        var view = $("gimbalView");
        var feed = $("gimbalCameraFeed");
        if (!view) return;
        var mapping = getVideoMapping(view, feed);
        var x = event.clientX - mapping.rect.left;
        var y = event.clientY - mapping.rect.top;
        var videoX = ((x - mapping.offsetX) / Math.max(mapping.drawnWidth, 1)) * mapping.sourceWidth;
        var videoY = ((y - mapping.offsetY) / Math.max(mapping.drawnHeight, 1)) * mapping.sourceHeight;
        videoX = Math.max(0, Math.min(mapping.sourceWidth, videoX));
        videoY = Math.max(0, Math.min(mapping.sourceHeight, videoY));
        if (mirrorX) videoX = mapping.sourceWidth - videoX;
        var dx = Math.round(((videoX / Math.max(mapping.sourceWidth, 1)) - 0.5) * 1920);
        var dy = Math.round(((videoY / Math.max(mapping.sourceHeight, 1)) - 0.5) * 1080);
        hasClickMarker = true;
        clickMarker = { u: x / Math.max(mapping.containerWidth, 1), v: y / Math.max(mapping.containerHeight, 1) };
        guideCenter = { u: 0.5, v: 0.5 };
        guideAnimating = false;
        guideStart = performance.now();
        guideDurationMs = 1600;
        lastTarget = { x: dx, y: dy };
        setText("gimbalTarget", dx + " / " + dy);
        drawOverlay();
        postJson("/api/gimbal/click", { dx: dx, dy: dy }).then(function (body) {
            if (body && body.delta && body.delta.holdMs) {
                guideDurationMs = Math.max(Number(body.delta.holdMs || 0), 900);
                guideStart = performance.now();
            }
            if (body && body.state) updateState(body.state);
        });
    }
    function startTrackMode(nextTrackMode) {
        if (trackBusy) return;
        var nextMode = normalizeTrackMode(nextTrackMode);
        if (trackingActive && nextMode === runningTrackMode) {
            trackMode = nextMode;
            setMode("track", nextMode);
            return;
        }
        hasClickMarker = false;
        guideAnimating = false;
        trackMode = nextMode;
        setMode("track", nextMode);
        trackBusy = true;
        trackTarget = null;
        trackStatus = { locked: false, status: trackingActive ? "switching" : "starting", mode: nextMode, message: (trackingActive ? "Switching to " : "Starting ") + trackLabel(nextMode) + "..." };
        setText("gimbalTarget", trackStatus.message);
        renderControls();
        drawOverlay();
        var start = function () { return postJson("/api/gimbal/track/start", { mode: nextMode }); };
        (trackingActive ? postJson("/api/gimbal/track/stop").then(start) : start())
            .then(function (body) { if (body && body.state) updateState(body.state); })
            .finally(function () {
                trackBusy = false;
                refreshState();
                renderControls();
            });
    }
    function stopTrackMode(nextMode) {
        if (trackBusy) return;
        hasClickMarker = false;
        guideAnimating = false;
        trackBusy = true;
        renderControls();
        postJson("/api/gimbal/track/stop").then(function (body) {
            if (body && body.state) updateState(body.state);
            setMode(nextMode || "click");
            drawOverlay();
        }).finally(function () {
            trackBusy = false;
            refreshState();
            renderControls();
        });
    }
    function toggleRecording() {
        if (recordBusy) return;
        recordBusy = true;
        renderControls();
        postJson(recordingActive ? "/api/gimbal/recording/stop" : "/api/gimbal/recording/start", {})
            .then(function (body) {
                var state = body && body.state ? body.state : {};
                recordingActive = Boolean(state.active);
                recordingName = state.current && state.current.name ? state.current.name : (body && body.recording && body.recording.name ? body.recording.name : "");
                renderControls();
            })
            .finally(function () {
                recordBusy = false;
                refreshRecordingState();
                renderControls();
            });
    }
    function bindSocket() {
        if (!window.io) return;
        var socket = window.io({ transports: ["polling", "websocket"], upgrade: true });
        socket.on("gimbal_state", updateState);
        socket.on("gimbal_target", function (target) {
            if (!target || mode !== "track") return;
            hasClickMarker = false;
            trackTarget = target;
            var targetMode = normalizeTrackMode(target.mode || trackMode);
            runningTrackMode = targetMode;
            trackMode = targetMode;
            trackStatus = { locked: true, status: target.status || "track", message: target.message || trackLabel(targetMode).toUpperCase() + " LOCKED" };
            var targetX = Number.isFinite(Number(target.x)) ? Number(target.x) : Number(target.dx || 0);
            var targetY = Number.isFinite(Number(target.y)) ? Number(target.y) : Number(target.dy || 0);
            lastTarget = { x: targetX, y: targetY };
            setText("gimbalModeBadge", target.coasting ? "PREDICT" : trackLabel(targetMode).toUpperCase());
            if (targetMode === "beacon") {
                var beaconYawError = Number(target.errorYawDeg || 0).toFixed(1);
                var beaconPitchError = Number(target.errorPitchDeg || 0).toFixed(1);
                setText("gimbalTarget", (target.dryRun ? "DRY RUN " : "BEACON ") + "Yaw " + beaconYawError + "° / Pitch " + beaconPitchError + "°");
                return;
            }
            setText("gimbalTarget", (target.coasting ? "PREDICT " : trackLabel(targetMode).toUpperCase() + " ") + Math.round(targetX) + " / " + Math.round(targetY));
            renderControls();
            drawOverlay();
        });
        socket.on("gimbal_track_status", function (status) {
            if (!status || mode !== "track") return;
            trackStatus = status;
            if (status.mode) {
                runningTrackMode = normalizeTrackMode(status.mode);
                if (trackingActive) trackMode = runningTrackMode;
            }
            if (!status.locked) {
                trackTarget = null;
                setText("gimbalModeBadge", trackLabel(activeTrackMode()).toUpperCase());
                setText("gimbalTarget", status.message || lostMessage(activeTrackMode()));
            }
            renderControls();
            drawOverlay();
        });
        socket.on("gimbal_recording_state", function (state) {
            recordingActive = Boolean(state && state.active);
            recordingName = state && state.current && state.current.name ? state.current.name : "";
            renderControls();
        });
    }
    document.addEventListener("DOMContentLoaded", function () {
        setMode("click");
        startCamera("/api/gimbal/stream");
        document.addEventListener("visibilitychange", function () {
            if (!document.hidden) startCamera(cameraSource);
        });
        var view = $("gimbalView");
        if (view) view.addEventListener("click", handleClick);
        var clickBtn = $("gimbalModeClick");
        var faceBtn = $("gimbalModeFace");
        var swimmerBtn = $("gimbalModeSwimmer");
        var beaconBtn = $("gimbalModeBeacon");
        var connectBtn = $("gimbalConnect");
        var stopBtn = $("gimbalStop");
        var homeBtn = $("gimbalHome");
        var refreshVideoBtn = $("gimbalRefreshVideo");
        var osdToggle = $("gimbalOsdToggle");
        var recordToggle = $("gimbalRecordToggle");
        if (clickBtn) clickBtn.addEventListener("click", function () {
            if (trackBusy) return;
            if (trackingActive) stopTrackMode("click");
            else {
                hasClickMarker = false;
                guideAnimating = false;
                setMode("click");
                drawOverlay();
            }
        });
        function selectTrackMode(nextTrackMode) {
            startTrackMode(nextTrackMode);
        }
        if (faceBtn) faceBtn.addEventListener("click", function () { selectTrackMode("face"); });
        if (swimmerBtn) swimmerBtn.addEventListener("click", function () { selectTrackMode("swimmer"); });
        if (beaconBtn) beaconBtn.addEventListener("click", function () { selectTrackMode("beacon"); });
        if (connectBtn) connectBtn.addEventListener("click", function () {
            if (connectBusy) return;
            connectBusy = true;
            renderControls();
            postJson(connected ? "/api/gimbal/disconnect" : "/api/gimbal/connect").then(function (body) {
                if (body && body.state) updateState(body.state);
            }).finally(function () {
                connectBusy = false;
                refreshState();
                renderControls();
            });
        });
        if (stopBtn) stopBtn.addEventListener("click", function () {
            if (stopBusy) return;
            stopBusy = true;
            hasClickMarker = false;
            guideAnimating = false;
            drawOverlay();
            renderControls();
            postJson("/api/gimbal/stop").then(function (body) {
                if (body && body.state) updateState(body.state);
            }).finally(function () {
                stopBusy = false;
                refreshState();
                renderControls();
            });
        });
        if (homeBtn) homeBtn.addEventListener("click", function () {
            if (homeBusy) return;
            homeBusy = true;
            hasClickMarker = false;
            guideAnimating = false;
            drawOverlay();
            renderControls();
            postJson("/api/gimbal/home", { preserveTracking: true }).then(function (body) {
                if (body && body.state) updateState(body.state);
            }).finally(function () {
                homeBusy = false;
                refreshState();
                renderControls();
            });
        });
        if (refreshVideoBtn) refreshVideoBtn.addEventListener("click", refreshCameraStream);
        if (osdToggle) osdToggle.addEventListener("change", function () {
            osdBusy = true;
            renderControls();
            postJson("/api/gimbal/osd", { mode: osdToggle.checked ? 2 : 0 })
                .then(function (body) { if (body && body.state) updateState(body.state); })
                .finally(function () {
                    osdBusy = false;
                    renderControls();
                });
        });
        if (recordToggle) recordToggle.addEventListener("click", toggleRecording);
        bindSocket();
        refreshState();
        if (requestedTrackModeFromLocation() === "beacon") {
            startTrackMode("beacon");
        }
        refreshRecordingState();
        window.setInterval(refreshState, 1000);
        window.setInterval(refreshRecordingState, 2000);
        window.addEventListener("resize", function () { drawOverlay(); });
    });
})();
