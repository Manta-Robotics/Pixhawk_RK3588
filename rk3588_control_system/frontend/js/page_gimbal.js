(function () {
    var mode = "click";
    var trackingActive = false;
    var connected = false;
    var videoTransport = "udp";
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

    function $(id) { return document.getElementById(id); }
    function setText(id, value) { var el = $(id); if (el) el.textContent = value; }
    function postJson(url, body) {
        return fetch(url, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: body ? JSON.stringify(body) : "{}"
        }).then(function (response) { return response.json().catch(function () { return {}; }); });
    }
    function setMode(nextMode) {
        mode = nextMode === "track" ? "track" : "click";
        var clickBtn = $("gimbalModeClick");
        var trackBtn = $("gimbalModeTrack");
        if (clickBtn) clickBtn.classList.toggle("active", mode === "click");
        if (trackBtn) trackBtn.classList.toggle("active", mode === "track");
        setText("gimbalModeText", mode === "track" ? "Swimmer" : "Click");
        setText("gimbalModeBadge", mode === "track" ? "TRACK" : "CLICK");
    }
    function updateState(state) {
        if (!state) return;
        connected = Boolean(state.connected);
        trackingActive = Boolean(state.trackingActive);
        videoTransport = state.videoTransport || "udp";
        setText("gimbalSerial", (state.serialPort || "--") + " @ " + (state.baudRate || "--"));
        setText("gimbalStatus", state.connected ? "Connected" : (state.lastError || "Disconnected"));
        setText("gimbalCommand", state.lastCommand || "Idle");
        setText("gimbalVideo", state.videoInput || state.udpVideo || state.videoSource || "--");
        setText("gimbalCameraSource", state.videoSource || "/api/camera/stream");
        if (state.lastError && state.lastError.indexOf("not present") !== -1) {
            setText("gimbalCameraMessage", "Serial device missing. Reboot after UART3 setup, then reconnect.");
        }
        var btn = $("gimbalTrackToggle");
        if (btn) {
            btn.textContent = trackingActive ? "Stop Track" : "Track";
            btn.classList.toggle("active", trackingActive);
        }
        var connectBtn = $("gimbalConnect");
        if (connectBtn) {
            connectBtn.textContent = connected ? "Disconnect" : "Connect";
            connectBtn.classList.toggle("active", connected);
        }
        if (state.trackStatus) {
            trackStatus = state.trackStatus;
            if (!trackStatus.locked) {
                trackTarget = null;
                if (mode === "track") setText("gimbalTarget", "can not find swimmer");
            }
            drawOverlay();
        }
        if (state.lastTarget && mode === "track" && !hasClickMarker) {
            if (state.lastTarget.locked) trackTarget = state.lastTarget;
            var targetX = Number.isFinite(Number(state.lastTarget.x)) ? Number(state.lastTarget.x) : Number(state.lastTarget.dx || 0);
            var targetY = Number.isFinite(Number(state.lastTarget.y)) ? Number(state.lastTarget.y) : Number(state.lastTarget.dy || 0);
            lastTarget = { x: targetX, y: targetY };
            setText("gimbalTarget", state.lastTarget.locked ? Math.round(targetX) + " / " + Math.round(targetY) : "can not find swimmer");
            drawOverlay();
        }
    }
    function refreshState() {
        fetch("/api/gimbal/state", { cache: "no-store" })
            .then(function (r) { return r.json(); })
            .then(function (body) { if (body && body.state) updateState(body.state); })
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
            if (videoTransport === "rtsp") {
                setText("gimbalCameraMessage", "Proxy is running, but the RTSP stream is not producing decodable frames.");
            } else {
                setText("gimbalCameraMessage", "Proxy is running, but no decodable UDP video is arriving on port 9554.");
            }
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
            var tx = mapping.offsetX + (Number(trackTarget.x || 0) / Math.max(frameW, 1)) * mapping.drawnWidth;
            var ty = mapping.offsetY + (Number(trackTarget.y || 0) / Math.max(frameH, 1)) * mapping.drawnHeight;
            var bw = Math.max(48, (Number(trackTarget.w || 120) / Math.max(frameW, 1)) * mapping.drawnWidth);
            var bh = Math.max(48, (Number(trackTarget.h || 120) / Math.max(frameH, 1)) * mapping.drawnHeight);
            var x = Math.max(4, Math.min(width - bw - 4, tx - bw / 2));
            var y = Math.max(4, Math.min(height - bh - 4, ty - bh / 2));
            ctx.save();
            ctx.strokeStyle = "rgba(255, 238, 76, 0.98)";
            ctx.lineWidth = 5;
            ctx.strokeRect(x, y, bw, bh);
            ctx.fillStyle = "rgba(0, 0, 0, 0.78)";
            ctx.fillRect(x, Math.max(0, y - 34), 184, 30);
            ctx.fillStyle = "#ffee4c";
            ctx.font = "bold 16px sans-serif";
            ctx.fillText("SWIMMER LOCKED", x + 10, Math.max(22, y - 12));
            ctx.beginPath();
            ctx.arc(tx, ty, 7, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
            return;
        }
        var label = "CAN NOT FIND SWIMMER";
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
            if (guideAnimating) {
                var progress = Math.min(1, (performance.now() - guideStart) / Math.max(guideDurationMs, 1));
                guideCenter.u = 0.5 + (clickMarker.u - 0.5) * progress;
                guideCenter.v = 0.5 + (clickMarker.v - 0.5) * progress;
                if (progress >= 1) {
                    hasClickMarker = false;
                    guideAnimating = false;
                    window.cancelAnimationFrame(guideFrame);
                    guideCenter = { u: clickMarker.u, v: clickMarker.v };
                    window.requestAnimationFrame(drawOverlay);
                    return;
                }
            }
            var tx = clickMarker.u * width;
            var ty = clickMarker.v * height;
            var gx = guideCenter.u * width;
            var gy = guideCenter.v * height;
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
            if (guideAnimating) {
                window.cancelAnimationFrame(guideFrame);
                guideFrame = window.requestAnimationFrame(drawOverlay);
            }
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
        var dx = Math.round(((videoX / Math.max(mapping.sourceWidth, 1)) - 0.5) * 1920);
        var dy = Math.round(((videoY / Math.max(mapping.sourceHeight, 1)) - 0.5) * 1080);
        hasClickMarker = true;
        clickMarker = { u: x / Math.max(mapping.containerWidth, 1), v: y / Math.max(mapping.containerHeight, 1) };
        guideCenter = { u: 0.5, v: 0.5 };
        guideAnimating = true;
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
    function toggleTrack() {
        setMode("track");
        postJson(trackingActive ? "/api/gimbal/track/stop" : "/api/gimbal/track/start")
            .then(function (body) { if (body && body.state) updateState(body.state); });
    }
    function bindSocket() {
        if (!window.io) return;
        var socket = window.io({ transports: ["polling", "websocket"], upgrade: true });
        socket.on("gimbal_state", updateState);
        socket.on("gimbal_target", function (target) {
            if (!target || mode !== "track") return;
            hasClickMarker = false;
            trackTarget = target;
            trackStatus = { locked: true, status: target.status || "track", message: target.message || "SWIMMER LOCKED" };
            var targetX = Number.isFinite(Number(target.x)) ? Number(target.x) : Number(target.dx || 0);
            var targetY = Number.isFinite(Number(target.y)) ? Number(target.y) : Number(target.dy || 0);
            lastTarget = { x: targetX, y: targetY };
            setText("gimbalTarget", Math.round(targetX) + " / " + Math.round(targetY));
            drawOverlay();
        });
        socket.on("gimbal_track_status", function (status) {
            if (!status || mode !== "track") return;
            trackStatus = status;
            if (!status.locked) {
                trackTarget = null;
                setText("gimbalTarget", "can not find swimmer");
            }
            drawOverlay();
        });
    }
    document.addEventListener("DOMContentLoaded", function () {
        setMode("click");
        startCamera("/api/gimbal/stream");
        var view = $("gimbalView");
        if (view) view.addEventListener("click", handleClick);
        var clickBtn = $("gimbalModeClick");
        var trackBtn = $("gimbalModeTrack");
        var connectBtn = $("gimbalConnect");
        var stopBtn = $("gimbalStop");
        var homeBtn = $("gimbalHome");
        var autofocusBtn = $("gimbalAutofocus");
        var osdHideBtn = $("gimbalOsdHide");
        var trackToggle = $("gimbalTrackToggle");
        if (clickBtn) clickBtn.addEventListener("click", function () { setMode("click"); });
        if (trackBtn) trackBtn.addEventListener("click", function () { hasClickMarker = false; guideAnimating = false; setMode("track"); });
        if (connectBtn) connectBtn.addEventListener("click", function () {
            postJson(connected ? "/api/gimbal/disconnect" : "/api/gimbal/connect").then(function (body) {
                if (body && body.state) updateState(body.state);
            });
        });
        if (stopBtn) stopBtn.addEventListener("click", function () { hasClickMarker = false; guideAnimating = false; drawOverlay(); postJson("/api/gimbal/stop").then(function (body) { if (body && body.state) updateState(body.state); }); });
        if (homeBtn) homeBtn.addEventListener("click", function () { hasClickMarker = false; guideAnimating = false; drawOverlay(); postJson("/api/gimbal/home").then(function (body) { if (body && body.state) updateState(body.state); }); });
        if (autofocusBtn) autofocusBtn.addEventListener("click", function () { postJson("/api/gimbal/focus/auto").then(function (body) { if (body && body.state) updateState(body.state); }); });
        if (osdHideBtn) osdHideBtn.addEventListener("click", function () {
            postJson("/api/gimbal/detector/stop").then(function () { return postJson("/api/gimbal/track/cancel"); }).then(function () { return postJson("/api/gimbal/osd", { mode: 0 }); }).then(function (body) { if (body && body.state) updateState(body.state); });
        });
        if (trackToggle) trackToggle.addEventListener("click", toggleTrack);
        bindSocket();
        refreshState();
        window.setInterval(refreshState, 1000);
        window.addEventListener("resize", function () { drawOverlay(); });
    });
})();
