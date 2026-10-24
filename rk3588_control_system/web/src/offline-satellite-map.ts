// Behavior-preserving TypeScript port; original remains a parity fixture.
export {};
(function (this: any,root?: any): any  {
    "use strict";

    var EARTH_RADIUS_M: any = 6371008.8;

    function finite(this: any,value?: any, fallback?: any): any  {
        var number: any = Number(value);
        return Number.isFinite(number) ? number : fallback;
    }
    function clamp(this: any,value?: any, minimum?: any, maximum?: any): any  {
        return Math.max(minimum, Math.min(maximum, value));
    }
    function longitudeToWorld(this: any,longitude?: any): any  {
        return (longitude + 180) / 360;
    }
    function latitudeToWorld(this: any,latitude?: any): any  {
        var radians: any = clamp(latitude, -85.05112878, 85.05112878) * Math.PI / 180;
        return (1 - Math.asinh(Math.tan(radians)) / Math.PI) / 2;
    }
    function worldToLongitude(this: any,value?: any): any  {
        return value * 360 - 180;
    }
    function worldToLatitude(this: any,value?: any): any  {
        return Math.atan(Math.sinh(Math.PI * (1 - 2 * value))) * 180 / Math.PI;
    }
    function haversineMeters(this: any,a?: any, b?: any): any  {
        if (!a || !b) return 0;
        var lat1: any = a.latitude * Math.PI / 180;
        var lat2: any = b.latitude * Math.PI / 180;
        var dLat: any = lat2 - lat1;
        var dLon: any = (b.longitude - a.longitude) * Math.PI / 180;
        var sinLat: any = Math.sin(dLat / 2);
        var sinLon: any = Math.sin(dLon / 2);
        var h: any = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
        return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1, Math.sqrt(h)));
    }
    function create(this: any,canvas?: any, options?: any): any  {
        options = options || {};
        if (!canvas || typeof canvas.getContext !== "function") throw new Error("Offline map canvas is required");
        var context: any = canvas.getContext("2d");
        var manifestUrl: any = options.manifestUrl || "/assets/offline-map/manifest.json";
        var tileRoot: any = manifestUrl.slice(0, manifestUrl.lastIndexOf("/"));
        var manifest: any = null;
        var zoom: any = 10.2;
        var center: any = { x: longitudeToWorld(114.0579), y: latitudeToWorld(22.5431) };
        var location: any = null;
        var track: any = [];
        var distanceMeters: any = 0;
        var imageCache: any = new Map();
        var resizeObserver: any = null;
        var pointers: any = new Map();
        var gesture: any = null;
        var followLocation: any = true;
        var destroyed: any = false;

        function emit(this: any,state?: any, detail?: any): any  {
            if (typeof options.onState === "function") options.onState(state, detail || "");
        }
        function validLocation(this: any,value?: any): any  {
            return value && Number.isFinite(Number(value.latitude)) && Number.isFinite(Number(value.longitude)) &&
                Math.abs(Number(value.latitude)) <= 90 && Math.abs(Number(value.longitude)) <= 180 &&
                (Math.abs(Number(value.latitude)) > 0.000001 || Math.abs(Number(value.longitude)) > 0.000001);
        }
        function insideBounds(this: any,value?: any): any  {
            if (!manifest || !validLocation(value)) return false;
            var bounds: any = manifest.bounds;
            return value.longitude >= bounds.west && value.longitude <= bounds.east &&
                value.latitude >= bounds.south && value.latitude <= bounds.north;
        }
        function clampCenter(this: any): any  {
            if (!manifest) return;
            center.x = clamp(center.x, longitudeToWorld(manifest.bounds.west), longitudeToWorld(manifest.bounds.east));
            center.y = clamp(center.y, latitudeToWorld(manifest.bounds.north), latitudeToWorld(manifest.bounds.south));
        }
        function setCenter(this: any,longitude?: any, latitude?: any): any  {
            center.x = longitudeToWorld(longitude);
            center.y = latitudeToWorld(latitude);
            clampCenter();
        }
        function setZoom(this: any,value?: any): any  {
            if (!manifest) return;
            zoom = clamp(finite(value, zoom), finite(manifest.minZoom, 0), finite(manifest.maxZoom, 8));
        }
        function canvasSize(this: any): any  {
            var ratio: any = Math.max(1, root.devicePixelRatio || 1);
            var rect: any = canvas.getBoundingClientRect();
            var cssWidth: any = Math.max(1, rect.width);
            var cssHeight: any = Math.max(1, rect.height);
            var width: any = Math.round(cssWidth * ratio);
            var height: any = Math.round(cssHeight * ratio);
            if (canvas.width !== width || canvas.height !== height) {
                canvas.width = width;
                canvas.height = height;
            }
            context.setTransform(ratio, 0, 0, ratio, 0, 0);
            return { width: cssWidth, height: cssHeight };
        }
        function tileRange(this: any,level?: any): any  {
            var count: any = Math.pow(2, level);
            return {
                x0: Math.floor(longitudeToWorld(manifest.bounds.west) * count),
                x1: Math.floor(longitudeToWorld(manifest.bounds.east) * count),
                y0: Math.floor(latitudeToWorld(manifest.bounds.north) * count),
                y1: Math.floor(latitudeToWorld(manifest.bounds.south) * count)
            };
        }
        function imageFor(this: any,url?: any): any  {
            var cached: any = imageCache.get(url);
            if (cached) {
                cached.usedAt = Date.now();
                return cached;
            }
            var image: any = new Image();
            cached = { image: image, ready: false, failed: false, usedAt: Date.now() };
            imageCache.set(url, cached);
            image.onload = function (this: any): any  { cached.ready = true; if (!destroyed) render(); };
            image.onerror = function (this: any): any  { cached.failed = true; };
            image.src = url;
            if (imageCache.size > 96) {
                var oldestKey: any = null;
                var oldestTime: any = Infinity;
                imageCache.forEach(function (this: any,entry?: any, key?: any): any  {
                    if (entry.usedAt < oldestTime && !entry.ready) return;
                    if (entry.usedAt < oldestTime) { oldestTime = entry.usedAt; oldestKey = key; }
                });
                if (oldestKey) imageCache.delete(oldestKey);
            }
            return cached;
        }
        function screenPoint(this: any,longitude?: any, latitude?: any, worldSize?: any, scale?: any, width?: any, height?: any): any  {
            return {
                x: (longitudeToWorld(longitude) * worldSize - center.x * worldSize) * scale + width / 2,
                y: (latitudeToWorld(latitude) * worldSize - center.y * worldSize) * scale + height / 2
            };
        }
        function drawTiles(this: any,width?: any, height?: any): any  {
            var level: any = clamp(Math.floor(zoom), finite(manifest.minZoom, 0), finite(manifest.maxZoom, 8));
            var tileSize: any = finite(manifest.tileSize, 256);
            var worldSize: any = tileSize * Math.pow(2, level);
            var scale: any = Math.pow(2, zoom - level);
            var drawSize: any = tileSize * scale;
            var centerPixelX: any = center.x * worldSize;
            var centerPixelY: any = center.y * worldSize;
            var visible: any = {
                x0: Math.floor((centerPixelX - width / (2 * scale)) / tileSize),
                x1: Math.floor((centerPixelX + width / (2 * scale)) / tileSize),
                y0: Math.floor((centerPixelY - height / (2 * scale)) / tileSize),
                y1: Math.floor((centerPixelY + height / (2 * scale)) / tileSize)
            };
            var coverage: any = tileRange(level);
            for (var y: any = Math.max(visible.y0, coverage.y0); y <= Math.min(visible.y1, coverage.y1); y += 1) {
                for (var x: any = Math.max(visible.x0, coverage.x0); x <= Math.min(visible.x1, coverage.x1); x += 1) {
                    var screenX: any = (x * tileSize - centerPixelX) * scale + width / 2;
                    var screenY: any = (y * tileSize - centerPixelY) * scale + height / 2;
                    var entry: any = imageFor(tileRoot + "/" + level + "/" + x + "/" + y + ".jpg");
                    if (entry.ready) context.drawImage(entry.image, screenX, screenY, drawSize + 0.6, drawSize + 0.6);
                    else {
                        context.fillStyle = "#0b1b26";
                        context.fillRect(screenX, screenY, drawSize + 1, drawSize + 1);
                    }
                }
            }
            return { worldSize: worldSize, scale: scale };
        }
        function drawTrack(this: any,width?: any, height?: any, transform?: any): any  {
            if (!track.length) return;
            context.save();
            context.lineJoin = "round";
            context.lineCap = "round";
            context.lineWidth = 3;
            context.strokeStyle = "rgba(124, 229, 255, 0.94)";
            context.shadowColor = "rgba(70, 190, 235, 0.75)";
            context.shadowBlur = 8;
            context.beginPath();
            track.forEach(function (this: any,point?: any, index?: any): any  {
                var screen: any = screenPoint(point.longitude, point.latitude, transform.worldSize, transform.scale, width, height);
                if (index === 0) context.moveTo(screen.x, screen.y); else context.lineTo(screen.x, screen.y);
            });
            context.stroke();
            context.restore();
        }
        function drawMarker(this: any,width?: any, height?: any, transform?: any): any  {
            if (!validLocation(location) || !insideBounds(location)) return;
            var point: any = screenPoint(location.longitude, location.latitude, transform.worldSize, transform.scale, width, height);
            var heading: any = finite(location.heading, 0) * Math.PI / 180;
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
        function drawAttribution(this: any,width?: any, height?: any): any  {
            context.save();
            context.font = "600 9px system-ui, sans-serif";
            context.textAlign = "right";
            context.fillStyle = "rgba(235, 245, 250, 0.82)";
            var attribution: any = manifest && manifest.attribution ? manifest.attribution : "离线卫星影像";
            var resolution: any = manifest && manifest.shortResolution ? " · " + manifest.shortResolution : "";
            context.fillText(attribution + resolution + " · 非航海图", width - 8, height - 8);
            context.restore();
        }
        function render(this: any): any  {
            if (destroyed) return;
            var size: any = canvasSize();
            context.fillStyle = "#07131d";
            context.fillRect(0, 0, size.width, size.height);
            if (!manifest) {
                context.fillStyle = "rgba(224,239,248,.65)";
                context.font = "600 13px system-ui, sans-serif";
                context.textAlign = "center";
                context.fillText("正在加载板载离线卫星地图…", size.width / 2, size.height / 2);
                return;
            }
            var transform: any = drawTiles(size.width, size.height);
            drawTrack(size.width, size.height, transform);
            drawMarker(size.width, size.height, transform);
            drawAttribution(size.width, size.height);
        }
        function addTrackPoint(this: any,next?: any): any  {
            if (!next.fresh || !insideBounds(next)) return;
            var previous: any = track[track.length - 1];
            var segment: any = previous ? haversineMeters(previous, next) : 0;
            if (previous && segment < 0.5) return;
            if (previous && segment < 1000) distanceMeters += segment;
            track.push({ latitude: next.latitude, longitude: next.longitude, timestamp: next.updatedAt || Date.now() });
            if (track.length > 2000) track.shift();
        }
        function update(this: any,next?: any): any  {
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
        function snapshot(this: any): any  {
            return { location: location, distanceMeters: distanceMeters, pointCount: track.length, ready: Boolean(manifest), zoom: zoom };
        }
        function locate(this: any): any  {
            if (insideBounds(location)) {
                followLocation = true;
                setCenter(location.longitude, location.latitude);
                setZoom(finite(manifest.maxZoom, 8));
                render();
                return true;
            }
            return false;
        }
        function zoomBy(this: any,delta?: any): any  {
            followLocation = false;
            setZoom(zoom + delta);
            render();
        }
        function pointerDistance(this: any): any  {
            var values: any = Array.from(pointers.values());
            if (values.length < 2) return 0;
            return Math.hypot(values[0].x - values[1].x, values[0].y - values[1].y);
        }
        function onPointerDown(this: any,event?: any): any  {
            canvas.setPointerCapture(event.pointerId);
            pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
            followLocation = false;
            gesture = pointers.size === 1 ? { x: event.clientX, y: event.clientY, centerX: center.x, centerY: center.y } :
                { distance: pointerDistance(), zoom: zoom };
        }
        function onPointerMove(this: any,event?: any): any  {
            if (!pointers.has(event.pointerId) || !manifest) return;
            pointers.set(event.pointerId, { x: event.clientX, y: event.clientY });
            if (pointers.size === 1 && gesture && Number.isFinite(gesture.centerX)) {
                var worldSize: any = finite(manifest.tileSize, 256) * Math.pow(2, zoom);
                center.x = gesture.centerX - (event.clientX - gesture.x) / worldSize;
                center.y = gesture.centerY - (event.clientY - gesture.y) / worldSize;
                clampCenter();
            } else if (pointers.size >= 2 && gesture && gesture.distance > 0) {
                setZoom(gesture.zoom + Math.log2(pointerDistance() / gesture.distance));
            }
            render();
        }
        function onPointerUp(this: any,event?: any): any  {
            pointers.delete(event.pointerId);
            if (pointers.size === 1) {
                var remaining: any = Array.from(pointers.values())[0];
                gesture = { x: remaining.x, y: remaining.y, centerX: center.x, centerY: center.y };
            } else if (!pointers.size) gesture = null;
        }
        canvas.style.touchAction = "none";
        canvas.addEventListener("pointerdown", onPointerDown);
        canvas.addEventListener("pointermove", onPointerMove);
        canvas.addEventListener("pointerup", onPointerUp);
        canvas.addEventListener("pointercancel", onPointerUp);
        function onWheel(event: WheelEvent): void {
            event.preventDefault();
            zoomBy(event.deltaY < 0 ? 0.5 : -0.5);
        }
        canvas.addEventListener("wheel", onWheel, { passive: false });
        if (typeof root.ResizeObserver === "function") {
            resizeObserver = new root.ResizeObserver(render);
            resizeObserver.observe(canvas);
        } else root.addEventListener("resize", render);
        var ready: any = fetch(manifestUrl, { cache: "no-store" }).then(function (this: any,response?: any): any  {
            if (!response.ok) throw new Error("offline map manifest unavailable");
            return response.json();
        }).then(function (this: any,payload?: any): any  {
            manifest = payload;
            zoom = finite(manifest.initialZoom, zoom);
            setCenter(finite(manifest.center && manifest.center[0], 114.0579), finite(manifest.center && manifest.center[1], 22.5431));
            emit("ready", manifest);
            render();
            return manifest;
        }).catch(function (this: any,error?: any): any  {
            emit("error", String(error && error.message || error));
            throw error;
        });
        render();
        return {
            ready: ready, update: update, render: render, resize: render, locate: locate, zoomBy: zoomBy,
            snapshot: snapshot, reset: function (this: any): any  { track = []; distanceMeters = 0; render(); },
            destroy: function (): void {
                destroyed = true;
                if (resizeObserver) resizeObserver.disconnect();
                else root.removeEventListener("resize", render);
                canvas.removeEventListener("pointerdown", onPointerDown);
                canvas.removeEventListener("pointermove", onPointerMove);
                canvas.removeEventListener("pointerup", onPointerUp);
                canvas.removeEventListener("pointercancel", onPointerUp);
                canvas.removeEventListener("wheel", onWheel);
                pointers.clear();
            }
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
