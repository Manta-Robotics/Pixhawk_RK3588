(function () {
    "use strict";
    var controller = null;
    var amapAdapter = null;
    var mapProvider = "local";

    function fmt(value, digits, fallback) {
        var number = Number(value);
        return Number.isFinite(number) ? number.toFixed(digits) : fallback;
    }
    function setText(id, value) {
        var element = document.getElementById(id);
        if (element) element.textContent = value;
    }
    function ageLabel(ageMs) {
        if (ageMs === null || !Number.isFinite(Number(ageMs))) return "--";
        if (ageMs < 1000) return "< 1 s";
        return (ageMs / 1000).toFixed(1) + " s";
    }
    function render(telemetry) {
        if (!controller) return;
        var snapshot = controller.update(telemetry);
        var location = snapshot.location;
        if (amapAdapter) amapAdapter.update(location);
        var statusText = location.status === "available" ? location.fixLabel :
            location.status === "stale" ? location.fixLabel + " · stale" : location.fixLabel;
        setText("mapFixStatus", statusText);
        setText("mapHeadingQuick", fmt(location.heading, 1, "--") + "°");
        setText("mapSatQuick", location.satellites + " sat");
        setText("mapLat", location.fixValid ? fmt(location.latitude, 7, "--") : "--");
        setText("mapLon", location.fixValid ? fmt(location.longitude, 7, "--") : "--");
        setText("mapAlt", location.fixValid ? fmt(location.altitude, 1, "--") + " m" : "--");
        setText("mapSats", String(location.satellites));
        setText("mapHdop", fmt(location.hdop, 1, "--"));
        setText("mapYaw", fmt(location.heading, 1, "--") + "°");
        setText("mapAccuracy", location.horizontalAccuracy === null ? "--" : "±" + fmt(location.horizontalAccuracy, 1, "--") + " m");
        setText("mapSpeed", location.groundSpeed === null ? "--" : fmt(location.groundSpeed, 2, "--") + " m/s");
        setText("mapAge", ageLabel(location.ageMs));
        setText("mapDistance", snapshot.distanceMeters < 1000 ? snapshot.distanceMeters.toFixed(1) + " m" : (snapshot.distanceMeters / 1000).toFixed(2) + " km");
        var card = document.getElementById("mapStatusCard");
        if (card) card.dataset.state = location.status;
        var hint = document.getElementById("mapHint");
        if (!hint) return;
        if (location.status === "available" && mapProvider === "amap") hint.textContent = "Pixhawk GPS is valid. WGS84 coordinates are converted to GCJ-02 and displayed on Amap.";
        else if (location.status === "available") hint.textContent = "GPS fix is valid. The offline local track is updating.";
        else if (location.status === "stale") hint.textContent = "The last valid location is retained, but GPS data is more than five seconds old.";
        else if (location.fixType < 2) hint.textContent = "Waiting for a 2D or 3D flight-controller GPS fix.";
        else hint.textContent = "GPS reports a fix, but no valid coordinates have arrived yet.";
    }

    function initializeAmap() {
        var stack = document.getElementById("mapStack");
        var container = document.getElementById("amapMap");
        var providerText = document.getElementById("mapProviderText");
        if (!stack || !container || !window.MantaAmapAdapter) return;
        fetch("/api/map/config", { cache: "no-store" })
            .then(function (response) { if (!response.ok) throw new Error("Map config unavailable"); return response.json(); })
            .then(function (payload) {
                var config = payload && payload.data ? payload.data : {};
                if (!config.enabled || !config.jsKey) {
                    if (providerText) providerText.textContent = "高德地图 Key 未配置，当前使用板端离线轨迹图。";
                    return;
                }
                amapAdapter = window.MantaAmapAdapter.create(container, config, function (state, detail) {
                    if (state === "ready") {
                        mapProvider = "amap";
                        stack.dataset.provider = "amap";
                        if (providerText) providerText.textContent = "地图来源：高德地图；位置来源：Pixhawk GPS（WGS84 转 GCJ-02）。";
                    } else {
                        mapProvider = "local";
                        stack.dataset.provider = "local";
                        if (providerText) providerText.textContent = "高德地图不可用，已切换板端离线轨迹图。" + (detail ? " " + detail : "");
                    }
                });
            })
            .catch(function () {
                if (providerText) providerText.textContent = "地图配置读取失败，当前使用板端离线轨迹图。";
            });
    }
    document.addEventListener("DOMContentLoaded", function () {
        var canvas = document.getElementById("gpsMap");
        if (!canvas || !window.MantaGpsMapCore || !window.RoverClient) return;
        controller = window.MantaGpsMapCore.create(canvas);
        initializeAmap();
        window.RoverClient.on("telemetry", render);
        render(window.RoverClient.state.telemetry);
    });
})();
