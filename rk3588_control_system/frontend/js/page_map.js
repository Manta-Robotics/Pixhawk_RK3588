(function () {
    "use strict";
    var controller = null;

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
        var location = window.MantaGpsMapCore.normalizeTelemetry(telemetry, Date.now());
        var snapshot = controller.update(location);
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
        if (location.status === "available") hint.textContent = "GPS fix is valid. WGS84 position and track are updating on the board-hosted offline satellite map.";
        else if (location.status === "stale") hint.textContent = "The last valid location is retained, but GPS data is more than five seconds old.";
        else if (location.fixType < 2) hint.textContent = "Waiting for a 2D or 3D flight-controller GPS fix.";
        else hint.textContent = "GPS reports a fix, but no valid coordinates have arrived yet.";
    }

    function initializeOfflineMap() {
        var container = document.getElementById("offlineSatelliteMap");
        var providerText = document.getElementById("mapProviderText");
        if (!container || !window.MantaOfflineSatelliteMap) return;
        controller = window.MantaOfflineSatelliteMap.create(container, {
            onState: function (state, detail) {
                if (!providerText) return;
                providerText.textContent = state === "ready" ?
                    "地图来源：板载 NASA 离线卫星影像（500 米源）；位置来源：Pixhawk GPS。" :
                    "离线卫星地图加载失败。" + (detail ? " " + detail : "");
            }
        });
        controller.ready.catch(function () {});
    }
    document.addEventListener("DOMContentLoaded", function () {
        if (!window.MantaGpsMapCore || !window.MantaOfflineSatelliteMap || !window.RoverClient) return;
        initializeOfflineMap();
        window.RoverClient.on("telemetry", render);
        render(window.RoverClient.state.telemetry);
    });
})();
