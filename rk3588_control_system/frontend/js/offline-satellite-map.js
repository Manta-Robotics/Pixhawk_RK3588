(function (root) {
    "use strict";

    var EARTH_RADIUS_M = 6371008.8;

    function finite(value, fallback) {
        var number = Number(value);
        return Number.isFinite(number) ? number : fallback;
    }
    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
    function longitudeToWorld(longitude) {
        return (longitude + 180) / 360;
    }
    function latitudeToWorld(latitude) {
        var radians = clamp(latitude, -85.05112878, 85.05112878) * Math.PI / 180;
        return (1 - Math.asinh(Math.tan(radians)) / Math.PI) / 2;
    }
    function worldToLongitude(value) {
        return value * 360 - 180;
    }
    function worldToLatitude(value) {
        return Math.atan(Math.sinh(Math.PI * (1 - 2 * value))) * 180 / Math.PI;
    }
    function haversineMeters(a, b) {
        if (!a || !b) return 0;
        var lat1 = a.latitude * Math.PI / 180;
        var lat2 = b.latitude * Math.PI / 180;
        var dLat = lat2 - lat1;
        var dLon = (b.longitude - a.longitude) * Math.PI / 180;
        var sinLat = Math.sin(dLat / 2);
        var sinLon = Math.sin(dLon / 2);
        var h = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
        return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1, Math.sqrt(h)));
    }
    function create(canvas, options) {
        options = options || {};
        if (!canvas || typeof canvas.getContext !== "function") throw new Error("Offline map canvas is required");
        var context = canvas.getContext("2d");
        var manifestUrl = options.manifestUrl || "/assets/offline-map/manifest.json";
        var tileRoot = manifestUrl.slice(0, manifestUrl.lastIndexOf("/"));
        var manifest = null;
        var zoom = 10.2;
        var center = { x: longitudeToWorld(114.0579), y: latitudeToWorld(22.5431) };
        var location = null;
        var track = [];
        var distanceMeters = 0;
        var imageCache = new Map();
        var resizeObserver = null;
        var pointers = new Map();
        var gesture = null;
        var followLocation = true;
        var destroyed = false;

        function emit(state, detail) {
            if (typeof options.onState === "function") options.onState(state, detail || "");
        }
        function validLocation(value) {
            return value && Number.isFinite(Number(value.latitude)) && Number.isFinite(Number(value.longitude)) &&
                Math.abs(Number(value.latitude)) <= 90 && Math.abs(Number(value.longitude)) <= 180 &&
                (Math.abs(Number(value.latitude)) > 0.000001 || Math.abs(Number(value.longitude)) > 0.000001);
        }
        function insideBounds(value) {
            if (!manifest || !validLocation(value)) return false;
            var bounds = manifest.bounds;
            return value.longitude >= bounds.west && value.longitude <= bounds.east &&
                value.latitude >= bounds.south && value.latitude <= bounds.north;
        }
        function clampCenter() {
            if (!manifest) return;
            center.x = clamp(center.x, longitudeToWorld(manifest.bounds.west), longitudeToWorld(manifest.bounds.east));
            center.y = clamp(center.y, latitudeToWorld(manifest.bounds.north), latitudeToWorld(manifest.bounds.south));
        }
        function setCenter(longitude, latitude) {
            center.x = longitudeToWorld(longitude);
            center.y = latitudeToWorld(latitude);
            clampCenter();
        }
        function setZoom(value) {
            if (!manifest) return;
            zoom = clamp(finite(value, zoom), finite(manifest.minZoom, 0), finite(manifest.maxZoom, 8));
        }
        function canvasSize() {
            var ratio = Math.max(1, root.devicePixelRatio || 1);
            var rect = canvas.getBoundingClientRect();
            var cssWidth = Math.max(1, rect.width);
            var cssHeight = Math.max(1, rect.height);
            var width = Math.round(cssWidth * ratio);
            var height = Math.round(cssHeight * ratio);
            if (canvas.width !== width || canvas.height !== height) {
                canvas.width = width;
                canvas.height = height;
            }
            context.setTransform(ratio, 0, 0, ratio, 0, 0);
            return { width: cssWidth, height: cssHeight };
        }
        function tileRange(level) {
            var count = Math.pow(2, level);
            return {
                x0: Math.floor(longitudeToWorld(manifest.bounds.west) * count),
                x1: Math.floor(longitudeToWorld(manifest.bounds.east) * count),
                y0: Math.floor(latitudeToWorld(manifest.bounds.north) * count),
                y1: Math.floor(latitudeToWorld(manifest.bounds.south) * count)
            };
        }
        function imageFor(url) {
            var cached = imageCache.get(url);
            if (cached) {
                cached.usedAt = Date.now();
                return cached;
            }
            var image = new Image();
            cached = { image: image, ready: false, failed: false, usedAt: Date.now() };
            imageCache.set(url, cached);
            image.onload = function () { cached.ready = true; if (!destroyed) render(); };
            image.onerror = function () { cached.failed = true; };
            image.src = url;
            if (imageCache.size > 96) {
                var oldestKey = null;
                var oldestTime = Infinity;
                imageCache.forEach(function (entry, key) {
                    if (entry.usedAt < oldestTime && !entry.ready) return;
                    if (entry.usedAt < oldestTime) { oldestTime = entry.usedAt; oldestKey = key; }
                });
                if (oldestKey) imageCache.delete(oldestKey);
            }
            return cached;
        }
        function screenPoint(longitude, latitude, worldSize, scale, width, height) {
            return {
                x: (longitudeToWorld(longitude) * worldSize - center.x * worldSize) * scale + width / 2,
                y: (latitudeToWorld(latitude) * worldSize - center.y * worldSize) * scale + height / 2
            };
        }
        function drawTiles(width, height) {
            var level = clamp(Math.floor(zoom), finite(manifest.minZoom, 0), finite(manifest.maxZoom, 8));
            var tileSize = finite(manifest.tileSize, 256);
            var worldSize = tileSize * Math.pow(2, level);
            var scale = Math.pow(2, zoom - level);
            var drawSize = tileSize * scale;
            var centerPixelX = center.x * worldSize;
            var centerPixelY = center.y * worldSize;
            var visible = {
                x0: Math.floor((centerPixelX - width / (2 * scale)) / tileSize),
                x1: Math.floor((centerPixelX + width / (2 * scale)) / tileSize),
                y0: Math.floor((centerPixelY - height / (2 * scale)) / tileSize),
                y1: Math.floor((centerPixelY + height / (2 * scale)) / tileSize)
            };
            var coverage = tileRange(level);
            for (var y = Math.max(visible.y0, coverage.y0); y <= Math.min(visible.y1, coverage.y1); y += 1) {
                for (var x = Math.max(visible.x0, coverage.x0); x <= Math.min(visible.x1, coverage.x1); x += 1) {
                    var screenX = (x * tileSize - centerPixelX) * scale + width / 2;
                    var screenY = (y * tileSize - centerPixelY) * scale + height / 2;
                    var entry = imageFor(tileRoot + "/" + level + "/" + x + "/" + y + ".jpg");
                    if (entry.ready) context.drawImage(entry.image, screenX, screenY, drawSize + 0.6, drawSize + 0.6);
                    else {
                        context.fillStyle = "#0b1b26";
                        context.fillRect(screenX, screenY, drawSize + 1, drawSize + 1);
                    }
                }
            }
            return { worldSize: worldSize, scale: scale };
        }
        function drawTrack(width, height, transform) {
            if (!track.length) return;
            context.save();
            context.lineJoin = "round";
            context.lineCap = "round";
            context.lineWidth = 3;
            context.strokeStyle = "rgba(124, 229, 255, 0.94)";
            context.shadowColor = "rgba(70, 190, 235, 0.75)";
            context.shadowBlur = 8;
            context.beginPath();
            track.forEach(function (point, index) {
                var screen = screenPoint(point.longitude, point.latitude, transform.worldSize, transform.scale, width, height);
                if (index === 0) context.moveTo(screen.x, screen.y); else context.lineTo(screen.x, screen.y);
            });
            context.stroke();
            context.restore();
        }
        function drawMarker(width, height, transform) {
            if (!validLocation(location) || !insideBounds(location)) return;
            var point = screenPoint(location.longitude, location.latitude, transform.worldSize, transform.scale, width, height);
            var heading = finite(location.heading, 0) * Math.PI / 180;
            context.save();
            context.translate(point.x, point.y);
            context.rotate(heading);
            context.beginPath();
            context.moveTo(0, -15);
            context.lineTo(10, 11);
            context.lineTo(0, 7);
            context.lineTo(-10, 11);
            context.closePath();
            context.fillStyle = location.fresh ? "#7ce5ff" : "#f4bd64";
            context.shadowColor = context.fillStyle;
            context.shadowBlur = 14;
            context.fill();
            context.restore();
        }
        function drawAttribution(width, height) {
            context.save();
            context.font = "600 9px system-ui, sans-serif";
            context.textAlign = "right";
            context.fillStyle = "rgba(235, 245, 250, 0.82)";
            var attribution = manifest && manifest.attribution ? manifest.attribution : "离线卫星影像";
            var resolution = manifest && manifest.shortResolution ? " · " + manifest.shortResolution : "";
            context.fillText(attribution + resolution + " · 非航海图", width - 8, height - 8);
            context.restore();
        }
        function render() {
            if (destroyed) return;
            var size = canvasSize();
            context.fillStyle = "#07131d";
            context.fillRect(0, 0, size.width, size.height);
            if (!manifest) {
                context.fillStyle = "rgba(224,239,248,.65)";
                context.font = "600 13px system-ui, sans-serif";
                context.textAlign = "center";
                context.fillText("正在加载板载离线卫星地图…", size.width / 2, size.height / 2);
                return;
            }
            var transform = drawTiles(size.width, size.height);
            drawTrack(size.width, size.height, transform);
            drawMarker(size.width, size.height, transform);
            drawAttribution(size.width, size.height);
        }
        function addTrackPoint(next) {
            if (!next.fresh || !insideBounds(next)) return;
            var previous = track[track.length - 1];
            var segment = previous ? haversineMeters(previous, next) : 0;
            if (previous && segment < 0.5) return;
            if (previous && segment < 1000) distanceMeters += segment;
            track.push({ latitude: next.latitude, longitude: next.longitude, timestamp: next.updatedAt || Date.now() });
            if (track.length > 2000) track.shift();
        }
        function update(next) {
            if (validLocation(next)) {
                location = {
                    latitude: Number(next.latitude), longitude: Number(next.longitude),
                    heading: finite(next.heading, finite(next.course, 0)), fresh: Boolean(next.fresh),
                    updatedAt: finite(next.updatedAt, Date.now())
                };
                addTrackPoint(location);
                if (followLocation && insideBounds(location)) {
                    setCenter(location.longitude, location.latitude);
                    setZoom(Math.max(zoom, finite(manifest && manifest.maxZoom, 14)));
                }
            }
            render();
            return snapshot();
        }
        function snapshot() {
            return { location: location, distanceMeters: distanceMeters, pointCount: track.length, ready: Boolean(manifest), zoom: zoom };
        }
        function locate() {
            if (insideBounds(location)) {
                followLocation = true;
                setCenter(location.longitude, location.latitude);
                setZoom(finite(manifest.maxZoom, 8));
                render();
                return true;
            }
            return false;
        }
        function zoomBy(delta) {
            followLocation = false;
            setZoom(zoom + delta);
            render();
        }
        function pointerDistance() {
            var values = Array.from(pointers.values());
            if (values.length < 2) return 0;
            return Math.hypot(values[0].x - values[1].x, values[0].y - values[1].y);
        }
        function onPointerDown(event) {
            canvas.setPointerCapture(event.pointerId);
            pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
            followLocation = false;
            gesture = pointers.size === 1 ? { x: event.clientX, y: event.clientY, centerX: center.x, centerY: center.y } :
                { distance: pointerDistance(), zoom: zoom };
        }
        function onPointerMove(event) {
            if (!pointers.has(event.pointerId) || !manifest) return;
            pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
            if (pointers.size === 1 && gesture && Number.isFinite(gesture.centerX)) {
                var worldSize = finite(manifest.tileSize, 256) * Math.pow(2, zoom);
                center.x = gesture.centerX - (event.clientX - gesture.x) / worldSize;
                center.y = gesture.centerY - (event.clientY - gesture.y) / worldSize;
                clampCenter();
            } else if (pointers.size >= 2 && gesture && gesture.distance > 0) {
                setZoom(gesture.zoom + Math.log2(pointerDistance() / gesture.distance));
            }
            render();
        }
        function onPointerUp(event) {
            pointers.delete(event.pointerId);
            if (pointers.size === 1) {
                var remaining = Array.from(pointers.values())[0];
                gesture = { x: remaining.x, y: remaining.y, centerX: center.x, centerY: center.y };
            } else if (!pointers.size) gesture = null;
        }
        canvas.style.touchAction = "none";
        canvas.addEventListener("pointerdown", onPointerDown);
        canvas.addEventListener("pointermove", onPointerMove);
        canvas.addEventListener("pointerup", onPointerUp);
        canvas.addEventListener("pointercancel", onPointerUp);
        canvas.addEventListener("wheel", function (event) {
            event.preventDefault();
            zoomBy(event.deltaY < 0 ? 0.5 : -0.5);
        }, { passive: false });
        if (typeof root.ResizeObserver === "function") {
            resizeObserver = new root.ResizeObserver(render);
            resizeObserver.observe(canvas);
        } else root.addEventListener("resize", render);
        var ready = fetch(manifestUrl, { cache: "no-store" }).then(function (response) {
            if (!response.ok) throw new Error("offline map manifest unavailable");
            return response.json();
        }).then(function (payload) {
            manifest = payload;
            zoom = finite(manifest.initialZoom, zoom);
            setCenter(finite(manifest.center && manifest.center[0], 114.0579), finite(manifest.center && manifest.center[1], 22.5431));
            emit("ready", manifest);
            render();
            return manifest;
        }).catch(function (error) {
            emit("error", String(error && error.message || error));
            throw error;
        });
        render();
        return {
            ready: ready, update: update, render: render, resize: render, locate: locate, zoomBy: zoomBy,
            snapshot: snapshot, reset: function () { track = []; distanceMeters = 0; render(); },
            destroy: function () { destroyed = true; if (resizeObserver) resizeObserver.disconnect(); }
        };
    }

    root.MantaOfflineSatelliteMap = {
        create: create,
        haversineMeters: haversineMeters,
        longitudeToWorld: longitudeToWorld,
        latitudeToWorld: latitudeToWorld,
        worldToLongitude: worldToLongitude,
        worldToLatitude: worldToLatitude
    };
})(typeof window !== "undefined" ? window : globalThis);
