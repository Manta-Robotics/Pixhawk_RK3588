(function (root) {
    "use strict";

    var EARTH_RADIUS_M = 6371008.8;
    var FIX_LABELS = { 0: "No GPS", 1: "No fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed", 7: "Static", 8: "PPP" };

    function finite(value, fallback) {
        var number = Number(value);
        return Number.isFinite(number) ? number : fallback;
    }
    function optionalFinite(value) {
        if (value === null || typeof value === "undefined" || value === "") return null;
        var number = Number(value);
        return Number.isFinite(number) ? number : null;
    }
    function accuracyMeters(value) {
        var accuracy = optionalFinite(value);
        return accuracy !== null && accuracy >= 0 && accuracy < 10000 ? accuracy : null;
    }
    function normalizeTimestamp(value) {
        var timestamp = optionalFinite(value);
        if (timestamp === null || timestamp <= 0) return null;
        return timestamp < 100000000000 ? timestamp * 1000 : timestamp;
    }
    function validCoordinates(latitude, longitude) {
        return Number.isFinite(latitude) && Number.isFinite(longitude) && latitude >= -90 && latitude <= 90 &&
            longitude >= -180 && longitude <= 180 && (Math.abs(latitude) > 0.000001 || Math.abs(longitude) > 0.000001);
    }
    function fixLabel(fixType) {
        var normalized = Math.max(0, Math.round(finite(fixType, 0)));
        return FIX_LABELS[normalized] || ("Fix " + normalized);
    }
    function haversineMeters(a, b) {
        if (!a || !b) return 0;
        var lat1 = finite(a.latitude, NaN) * Math.PI / 180;
        var lat2 = finite(b.latitude, NaN) * Math.PI / 180;
        var dLat = lat2 - lat1;
        var dLon = (finite(b.longitude, NaN) - finite(a.longitude, NaN)) * Math.PI / 180;
        if (![lat1, lat2, dLat, dLon].every(Number.isFinite)) return 0;
        var sinLat = Math.sin(dLat / 2);
        var sinLon = Math.sin(dLon / 2);
        var h = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
        return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1, Math.sqrt(h)));
    }
    function normalizeTelemetry(telemetry, nowMs) {
        telemetry = telemetry && typeof telemetry === "object" ? telemetry : {};
        var gps = telemetry.gps && typeof telemetry.gps === "object" ? telemetry.gps : {};
        var position = telemetry.position && typeof telemetry.position === "object" ? telemetry.position : {};
        var attitude = telemetry.attitude && typeof telemetry.attitude === "object" ? telemetry.attitude : {};
        var fixType = Math.max(0, Math.round(finite(gps.fixType, 0)));
        var gpsLatitude = finite(gps.latitude, NaN);
        var gpsLongitude = finite(gps.longitude, NaN);
        var fusedLatitude = finite(position.lat, NaN);
        var fusedLongitude = finite(position.lon, NaN);
        var useGpsRaw = fixType >= 2 && validCoordinates(gpsLatitude, gpsLongitude);
        var latitude = useGpsRaw ? gpsLatitude : fusedLatitude;
        var longitude = useGpsRaw ? gpsLongitude : fusedLongitude;
        var altitude = useGpsRaw ? finite(gps.altitude, finite(position.alt, 0)) : finite(position.alt, 0);
        var updatedAt = useGpsRaw ? normalizeTimestamp(gps.updatedAt) : normalizeTimestamp(position.updatedAt);
        var ageMs = updatedAt === null ? null : Math.max(0, finite(nowMs, Date.now()) - updatedAt);
        var coordinatesValid = validCoordinates(latitude, longitude);
        var fixValid = fixType >= 2 && coordinatesValid;
        var fresh = fixValid && ageMs !== null && ageMs <= 5000;
        var course = optionalFinite(gps.course);
        var speed = optionalFinite(gps.groundSpeed);
        var heading = course !== null && speed !== null && speed >= 0.5 ? course : finite(attitude.yaw, 0);
        return {
            status: fresh ? "available" : fixValid ? "stale" : "unavailable",
            fixValid: fixValid,
            fresh: fresh,
            fixType: fixType,
            fixLabel: fixLabel(fixType),
            satellites: Math.max(0, Math.round(finite(gps.satellites, 0))),
            hdop: optionalFinite(gps.hdop),
            latitude: latitude,
            longitude: longitude,
            altitude: altitude,
            horizontalAccuracy: accuracyMeters(gps.horizontalAccuracy),
            verticalAccuracy: accuracyMeters(gps.verticalAccuracy),
            groundSpeed: speed,
            course: course,
            heading: ((heading % 360) + 360) % 360,
            updatedAt: updatedAt,
            ageMs: ageMs,
            source: useGpsRaw ? "GPS_RAW_INT" : String(position.source || "GLOBAL_POSITION_INT")
        };
    }
    function localMeters(point, origin) {
        var latScale = Math.PI * EARTH_RADIUS_M / 180;
        var lonScale = latScale * Math.cos(origin.latitude * Math.PI / 180);
        return { x: (point.longitude - origin.longitude) * lonScale, y: (point.latitude - origin.latitude) * latScale };
    }
    function create(canvas, options) {
        options = options || {};
        if (!canvas || typeof canvas.getContext !== "function") throw new Error("A canvas element is required");
        var context = canvas.getContext("2d");
        var points = [];
        var distanceMeters = 0;
        var latest = normalizeTelemetry({}, Date.now());
        var resizeObserver = null;
        var maxPoints = Math.max(50, Math.round(finite(options.maxPoints, 2000)));
        var minimumPointDistance = Math.max(0.1, finite(options.minimumPointDistance, 0.5));
        function addPoint(location) {
            if (!location.fresh) return;
            var next = { latitude: location.latitude, longitude: location.longitude, timestamp: location.updatedAt || Date.now() };
            var previous = points[points.length - 1];
            var segment = previous ? haversineMeters(previous, next) : 0;
            if (previous && segment < minimumPointDistance) return;
            if (previous && segment < 1000) distanceMeters += segment;
            points.push(next);
            if (points.length > maxPoints) points.shift();
        }
        function sizeCanvas() {
            var ratio = Math.max(1, root.devicePixelRatio || 1);
            var rect = canvas.getBoundingClientRect();
            var width = Math.max(1, Math.round(rect.width * ratio));
            var height = Math.max(1, Math.round(rect.height * ratio));
            if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
            context.setTransform(ratio, 0, 0, ratio, 0, 0);
            return { width: Math.max(1, rect.width), height: Math.max(1, rect.height) };
        }
        function drawGrid(width, height) {
            var gradient = context.createLinearGradient(0, 0, width, height);
            gradient.addColorStop(0, "#0a1a2a"); gradient.addColorStop(0.55, "#07131d"); gradient.addColorStop(1, "#091925");
            context.fillStyle = gradient; context.fillRect(0, 0, width, height);
            context.strokeStyle = "rgba(126, 200, 227, 0.08)"; context.lineWidth = 1;
            for (var x = 0; x <= width; x += 40) { context.beginPath(); context.moveTo(x, 0); context.lineTo(x, height); context.stroke(); }
            for (var y = 0; y <= height; y += 40) { context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke(); }
            context.fillStyle = "rgba(190, 225, 241, 0.44)"; context.font = "600 10px system-ui, sans-serif"; context.fillText("N", 16, 22);
        }
        function render() {
            var size = sizeCanvas(); var width = size.width; var height = size.height;
            drawGrid(width, height);
            if (!latest.fixValid || points.length === 0) {
                context.fillStyle = "rgba(224, 239, 248, 0.58)"; context.font = "600 13px system-ui, sans-serif";
                context.textAlign = "center"; context.fillText("Waiting for flight-controller GPS fix", width / 2, height / 2); context.textAlign = "start"; return;
            }
            var origin = points[0];
            var localPoints = points.map(function (point) { return localMeters(point, origin); });
            var latestLocal = localMeters(latest, origin);
            var all = localPoints.concat([latestLocal]);
            var minX = Math.min.apply(null, all.map(function (point) { return point.x; }));
            var maxX = Math.max.apply(null, all.map(function (point) { return point.x; }));
            var minY = Math.min.apply(null, all.map(function (point) { return point.y; }));
            var maxY = Math.max.apply(null, all.map(function (point) { return point.y; }));
            var accuracy = latest.horizontalAccuracy || 0;
            var spanX = Math.max(20, maxX - minX + accuracy * 2); var spanY = Math.max(20, maxY - minY + accuracy * 2);
            var padding = 42; var scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
            var centerX = (minX + maxX) / 2; var centerY = (minY + maxY) / 2;
            function project(point) { return { x: width / 2 + (point.x - centerX) * scale, y: height / 2 - (point.y - centerY) * scale }; }
            if (latest.horizontalAccuracy !== null) {
                var accuracyPoint = project(latestLocal); context.beginPath();
                context.arc(accuracyPoint.x, accuracyPoint.y, Math.max(4, latest.horizontalAccuracy * scale), 0, Math.PI * 2);
                context.fillStyle = "rgba(74, 190, 225, 0.09)"; context.strokeStyle = "rgba(110, 215, 241, 0.35)"; context.fill(); context.stroke();
            }
            context.beginPath();
            localPoints.forEach(function (point, index) { var p = project(point); if (index === 0) context.moveTo(p.x, p.y); else context.lineTo(p.x, p.y); });
            context.strokeStyle = "#7cdcf4"; context.lineWidth = 2.5; context.lineJoin = "round"; context.lineCap = "round"; context.stroke();
            var home = project(localPoints[0]); context.beginPath(); context.arc(home.x, home.y, 4, 0, Math.PI * 2); context.fillStyle = "#f4bd64"; context.fill();
            var marker = project(latestLocal); context.save(); context.translate(marker.x, marker.y); context.rotate(latest.heading * Math.PI / 180);
            context.beginPath(); context.moveTo(0, -13); context.lineTo(9, 10); context.lineTo(0, 6); context.lineTo(-9, 10); context.closePath();
            context.fillStyle = latest.fresh ? "#7ce5ff" : "#f4bd64"; context.shadowColor = context.fillStyle; context.shadowBlur = 12; context.fill(); context.restore();
        }
        function snapshot() { return { location: latest, distanceMeters: distanceMeters, pointCount: points.length }; }
        function update(telemetry, nowMs) { latest = normalizeTelemetry(telemetry, nowMs || Date.now()); addPoint(latest); render(); return snapshot(); }
        function reset() { points = []; distanceMeters = 0; latest = normalizeTelemetry({}, Date.now()); render(); }
        if (typeof root.ResizeObserver === "function") { resizeObserver = new root.ResizeObserver(render); resizeObserver.observe(canvas); }
        else if (root.addEventListener) root.addEventListener("resize", render);
        render();
        return { update: update, render: render, reset: reset, snapshot: snapshot, destroy: function () { if (resizeObserver) resizeObserver.disconnect(); } };
    }
    root.MantaGpsMapCore = { create: create, fixLabel: fixLabel, haversineMeters: haversineMeters, normalizeTelemetry: normalizeTelemetry, validCoordinates: validCoordinates };
})(typeof window !== "undefined" ? window : globalThis);
