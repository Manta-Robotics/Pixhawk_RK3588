(function () {
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

    function $(id) { return document.getElementById(id); }
    function setText(id, value) { var el = $(id); if (el) el.textContent = value; }
    function normalizeTrackMode(value) {
        return String(value || "").toLowerCase() === "swimmer" ? "swimmer" : "face";
    }
    function trackLabel(value) {
        return normalizeTrackMode(value) === "swimmer" ? "Swimmer" : "Face";
    }
    function lostMessage(value) {
        return normalizeTrackMode(value) === "swimmer" ? "can not find swimmer" : "CAN NOT FIND FACE";
    }
    function activeTrackMode() {
        return trackingActive ? runningTrackMode : trackMode;
    }
    function renderControls() {
        var clickBtn = $("gimbalModeClick");
        var faceBtn = $("gimbalModeFace");
        var swimmerBtn = $("gimbalModeSwimmer");
        var currentMode = activeTrackMode();
        if (clickBtn) clickBtn.classList.toggle("active", mode === "click" && !trackingActive);
        if (faceBtn) faceBtn.classList.toggle("active", mode === "track" && trackMode === "face");
        if (swimmerBtn) swimmerBtn.classList.toggle("active", mode === "track" && trackMode === "swimmer");
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
        setText("gimbalStatus", state.connected ? "Connected" : (state.lastError || "Disconnected"));
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
            setText("gimbalTarget", state.lastTarget.locked ? (predicting ? "PREDICT " : trackLabel(targetMode).toUpperCase() + " ") + Math.round(targetX) + " / " + Math.round(targetY) : (state.lastTarget.message || lostMessage(targetMode)));
            renderControls();
            drawOverlay();
        }
    }
    function refreshState() {
        fetch("/api/gimbal/state", { cache: "no-store" })
            .then(function (r) { return r.json(); })
            .then(function (body) { if (body && body.state) updateState(body.state); })
            .catch(function () {});
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
    function startCamera(source) {
        var img = $("gimbalCameraFeed");
        var placeholder = $("gimbalCameraPlaceholder");
        var url = source || "/api/gimbal/stream";
        if (!img) return;
        img.onload = function () { if (placeholder) placeholder.style.display = "none"; drawOverlay(); };
        img.onerror = function () {
            if (placeholder) placeholder.style.display = "flex";
            setText("gimbalCameraMessage", "Proxy is running, but the RTSP stream is not producing decodable frames.");
        };
        img.src = url + (url.indexOf("?") === -1 ? "?" : "&") + "gimbal=" + Date.now();
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
        var view = $("gimbalView");
        if (view) view.addEventListener("click", handleClick);
        var clickBtn = $("gimbalModeClick");
        var faceBtn = $("gimbalModeFace");
        var swimmerBtn = $("gimbalModeSwimmer");
        var connectBtn = $("gimbalConnect");
        var stopBtn = $("gimbalStop");
        var homeBtn = $("gimbalHome");
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
        refreshRecordingState();
        window.setInterval(refreshState, 1000);
        window.setInterval(refreshRecordingState, 2000);
        window.addEventListener("resize", function () { drawOverlay(); });
    });
})();
