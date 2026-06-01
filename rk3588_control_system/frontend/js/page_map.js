(function () {
    var map = null;
    var marker = null;
    var pathLine = null;
    var pathPoints = [];
    var hasCentered = false;

    function fmt(value, digits) {
        return Number(value || 0).toFixed(digits);
    }

    function setText(id, value) {
        var element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }

    function hasValidFix(position, gps) {
        var lat = Number(position.lat || 0);
        var lon = Number(position.lon || 0);
        var sats = Number(gps.satellites || 0);

        if (Math.abs(lat) < 0.000001 && Math.abs(lon) < 0.000001) {
            return false;
        }

        return sats >= 4;
    }

    function ensureMap(lat, lon) {
        if (map || typeof L === "undefined") {
            return;
        }

        map = L.map("gpsMap", { zoomControl: true }).setView([lat, lon], 17);
        L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
            maxZoom: 19,
            attribution: "&copy; OpenStreetMap contributors"
        }).addTo(map);

        marker = L.circleMarker([lat, lon], {
            radius: 7,
            color: "#3ec7ff",
            fillColor: "#3ec7ff",
            fillOpacity: 0.85,
            weight: 2
        }).addTo(map);

        pathLine = L.polyline([], {
            color: "#f2b24c",
            weight: 3,
            opacity: 0.9
        }).addTo(map);
    }

    function pushPathPoint(lat, lon) {
        var point = [lat, lon];
        var last = pathPoints[pathPoints.length - 1];

        if (!last || Math.abs(last[0] - lat) > 0.000005 || Math.abs(last[1] - lon) > 0.000005) {
            pathPoints.push(point);
            if (pathPoints.length > 1500) {
                pathPoints.shift();
            }
            if (pathLine) {
                pathLine.setLatLngs(pathPoints);
            }
        }
    }

    function render(telemetry) {
        var position = telemetry.position || {};
        var gps = telemetry.gps || {};

        var lat = Number(position.lat || 0);
        var lon = Number(position.lon || 0);
        var alt = Number(position.alt || 0);

        setText("mapLat", fmt(lat, 6));
        setText("mapLon", fmt(lon, 6));
        setText("mapAlt", fmt(alt, 1) + " m");
        setText("mapSats", fmt(gps.satellites, 0));
        setText("mapHdop", fmt(gps.hdop, 1));
        setText("mapYaw", fmt(telemetry.attitude.yaw, 1) + "°");
        setText("mapHeadingQuick", fmt(telemetry.attitude.yaw, 1) + "°");
        setText("mapSatQuick", fmt(gps.satellites, 0) + " sat");

        var hint = document.getElementById("mapHint");
        var guide = document.getElementById("mapGuideText");
        var validFix = hasValidFix(position, gps);

        if (!validFix) {
            setText("mapFixStatus", "Awaiting fix");
            if (hint) {
                hint.textContent = "Awaiting valid GPS coordinates (satellites >= 4)...";
            }
            if (guide) {
                guide.textContent = "Make sure at least 4 satellites are available and HDOP is reasonably low before expecting a stable track.";
            }
            return;
        }

        setText("mapFixStatus", "Fix valid");
        if (hint) {
            hint.textContent = "GPS locked. Map is updating live.";
        }
        if (guide) {
            guide.textContent = "The GPS fix is valid. If the map stays blank, check the map network before checking the FCU link.";
        }

        ensureMap(lat, lon);

        if (!map || !marker) {
            setText("mapFixStatus", "Map offline");
            if (hint) {
                hint.textContent = "Map initialization failed. Check network access to OpenStreetMap.";
            }
            if (guide) {
                guide.textContent = "Latitude and longitude are updating, but the basemap failed to load. This is usually an OpenStreetMap network issue.";
            }
            return;
        }

        marker.setLatLng([lat, lon]);
        pushPathPoint(lat, lon);

        if (!hasCentered) {
            map.setView([lat, lon], 18);
            hasCentered = true;
        }
    }

    document.addEventListener("DOMContentLoaded", function () {
        if (!window.RoverClient) {
            return;
        }

        window.RoverClient.on("telemetry", function (telemetry) {
            render(telemetry);
        });

        render(window.RoverClient.state.telemetry);
    });
})();
