(function (root) {
    "use strict";

    var loaderPromise = null;

    function loadAmap(config) {
        if (root.AMap) return Promise.resolve(root.AMap);
        if (loaderPromise) return loaderPromise;
        loaderPromise = new Promise(function (resolve, reject) {
            var callbackName = "__mantaAmapReady";
            var timeout = root.setTimeout(function () { reject(new Error("Amap load timeout")); }, 10000);
            root._AMapSecurityConfig = { serviceHost: root.location.origin + config.serviceHost };
            root[callbackName] = function () {
                root.clearTimeout(timeout);
                delete root[callbackName];
                if (root.AMap) resolve(root.AMap);
                else reject(new Error("Amap API unavailable"));
            };
            var script = root.document.createElement("script");
            script.charset = "utf-8";
            script.onerror = function () { root.clearTimeout(timeout); reject(new Error("Amap network unavailable")); };
            script.src = "https://webapi.amap.com/maps?v=2.0&plugin=AMap.Scale&key=" + encodeURIComponent(config.jsKey) + "&callback=" + callbackName;
            root.document.head.appendChild(script);
        });
        return loaderPromise;
    }

    function create(container, config, onState) {
        var map = null;
        var marker = null;
        var polyline = null;
        var path = [];
        var centered = false;
        var lastInput = null;
        var converting = false;
        var pending = null;

        function state(name, detail) { if (typeof onState === "function") onState(name, detail || ""); }
        function applyConverted(AMap, lngLat) {
            var position = [lngLat.getLng(), lngLat.getLat()];
            if (!marker) {
                marker = new AMap.Marker({ position: position, anchor: "center" });
                polyline = new AMap.Polyline({ path: [], strokeColor: "#28b8ff", strokeWeight: 5, strokeOpacity: 0.9 });
                map.add([polyline, marker]);
            }
            marker.setPosition(position);
            var previous = path[path.length - 1];
            if (!previous || Math.abs(previous[0] - position[0]) > 0.000003 || Math.abs(previous[1] - position[1]) > 0.000003) {
                path.push(position);
                if (path.length > 1500) path.shift();
                polyline.setPath(path);
            }
            if (!centered) {
                map.setZoomAndCenter(18, position);
                centered = true;
            }
        }
        function convertNext(AMap) {
            if (!pending || converting) return;
            var input = pending;
            pending = null;
            converting = true;
            AMap.convertFrom([input.longitude, input.latitude], "gps", function (status, result) {
                converting = false;
                if (status === "complete" && result && String(result.info).toLowerCase() === "ok" && result.locations && result.locations[0]) {
                    applyConverted(AMap, result.locations[0]);
                } else {
                    state("fallback", "Pixhawk WGS84 to GCJ-02 conversion failed");
                }
                convertNext(AMap);
            });
        }
        function update(location) {
            if (!map || !location || !location.fresh) return;
            var key = Number(location.longitude).toFixed(7) + "," + Number(location.latitude).toFixed(7);
            if (key === lastInput) return;
            lastInput = key;
            pending = location;
            convertNext(root.AMap);
        }

        var ready = loadAmap(config).then(function (AMap) {
            map = new AMap.Map(container, { zoom: 15, viewMode: "2D", mapStyle: "amap://styles/dark" });
            map.addControl(new AMap.Scale());
            state("ready", "Amap online");
            return true;
        }).catch(function (error) {
            state("fallback", error.message || "Amap unavailable");
            return false;
        });

        return { ready: ready, update: update, destroy: function () { if (map) map.destroy(); } };
    }

    root.MantaAmapAdapter = { create: create, loadAmap: loadAmap };
})(typeof window !== "undefined" ? window : globalThis);
