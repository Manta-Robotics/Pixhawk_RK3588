(function () {
    function updateClock() {
        var clock = document.getElementById("headerTime");
        if (!clock) {
            return;
        }

        var now = new Date();
        var h = String(now.getHours()).padStart(2, "0");
        var m = String(now.getMinutes()).padStart(2, "0");
        var s = String(now.getSeconds()).padStart(2, "0");
        clock.textContent = h + ":" + m + ":" + s;
    }

    function setActiveNav() {
        var page = document.body.getAttribute("data-page") || "";
        document.querySelectorAll(".main-nav a[data-nav]").forEach(function (link) {
            var active = link.getAttribute("data-nav") === page;
            link.classList.toggle("active", active);
        });
    }

    function renderHeaderStatus() {
        if (!window.RoverClient) {
            return;
        }

        var connEl = document.getElementById("headerConnection");
        var pixhawkEl = document.getElementById("headerPixhawk");

        if (connEl) {
            var webConnected = Boolean(window.RoverClient.state.socketConnected);
            connEl.textContent = webConnected ? "WEB LIVE" : "WEB OFF";
            connEl.classList.toggle("connected", webConnected);
            connEl.classList.toggle("disconnected", !webConnected);
        }

        if (pixhawkEl) {
            var pixhawkConnected = window.RoverClient.state.pixhawkStatus === "connected" || Boolean(window.RoverClient.state.isConnected);
            pixhawkEl.textContent = pixhawkConnected ? "FCU LIVE" : "FCU OFF";
            pixhawkEl.classList.toggle("connected", pixhawkConnected);
            pixhawkEl.classList.toggle("disconnected", !pixhawkConnected);
        }
    }

    document.addEventListener("DOMContentLoaded", function () {
        setActiveNav();
        updateClock();
        renderHeaderStatus();

        window.setInterval(updateClock, 1000);

        if (window.RoverClient) {
            window.RoverClient.on("socketConnection", renderHeaderStatus);
            window.RoverClient.on("pixhawkConnection", renderHeaderStatus);
            window.RoverClient.on("status", renderHeaderStatus);
        }
    });
})();
