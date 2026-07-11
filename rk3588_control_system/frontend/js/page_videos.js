(function () {
    var recordings = [];
    var selectedName = "";

    function $(id) { return document.getElementById(id); }
    function formatSize(bytes) {
        var value = Number(bytes || 0);
        if (value > 1024 * 1024 * 1024) return (value / (1024 * 1024 * 1024)).toFixed(2) + " GB";
        if (value > 1024 * 1024) return (value / (1024 * 1024)).toFixed(1) + " MB";
        if (value > 1024) return (value / 1024).toFixed(1) + " KB";
        return value + " B";
    }
    function formatDate(value) {
        var date = new Date(value);
        if (Number.isNaN(date.getTime())) return "--";
        return date.toLocaleString();
    }
    function api(url, options) {
        return fetch(url, options || {}).then(function (response) { return response.json().catch(function () { return {}; }); });
    }
    function selectRecording(name) {
        selectedName = name;
        var item = recordings.find(function (entry) { return entry.name === name; });
        var video = $("videoPreview");
        var placeholder = $("videoPlaceholder");
        if (video && item) {
            video.src = item.relativeUrl || item.url;
            video.style.display = "block";
            if (placeholder) placeholder.style.display = "none";
        }
        render();
    }
    function renameRecording(name) {
        var item = recordings.find(function (entry) { return entry.name === name; });
        var nextName = window.prompt("Recording name", item ? item.title : name.replace(/\.mp4$/i, ""));
        if (!nextName) return;
        api("/api/gimbal/recordings/" + encodeURIComponent(name), {
            method: "PATCH",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ name: nextName })
        }).then(function (body) {
            recordings = body.recordings || recordings;
            if (body.recording && body.recording.name) selectRecording(body.recording.name);
            render();
        });
    }
    function deleteRecording(name) {
        var item = recordings.find(function (entry) { return entry.name === name; });
        if (!window.confirm("Delete " + (item ? item.title : name) + "?")) return;
        api("/api/gimbal/recordings/" + encodeURIComponent(name), { method: "DELETE" }).then(function (body) {
            recordings = body.recordings || [];
            if (selectedName === name) {
                selectedName = "";
                var video = $("videoPreview");
                var placeholder = $("videoPlaceholder");
                if (video) {
                    video.removeAttribute("src");
                    video.load();
                    video.style.display = "none";
                }
                if (placeholder) placeholder.style.display = "flex";
            }
            render();
        });
    }
    function render() {
        var list = $("videoList");
        if (!list) return;
        if (!recordings.length) {
            list.innerHTML = '<div class="empty-state">No recordings yet.</div>';
            return;
        }
        list.innerHTML = recordings.map(function (item) {
            var selected = item.name === selectedName ? " active" : "";
            return '<article class="video-item' + selected + '" data-name="' + encodeURIComponent(item.name) + '">' +
                '<button class="video-title" type="button" data-action="select">' + item.title + '</button>' +
                '<div class="video-meta">' + formatDate(item.modifiedAt) + ' · ' + formatSize(item.size) + '</div>' +
                '<div class="video-actions">' +
                    '<button class="btn" type="button" data-action="rename">Rename</button>' +
                    '<a class="btn" href="' + (item.relativeDownloadUrl || item.downloadUrl) + '" download="' + item.name + '">Download</a>' +
                    '<button class="btn btn-danger" type="button" data-action="delete">Delete</button>' +
                '</div>' +
            '</article>';
        }).join("");
    }
    function refresh() {
        api("/api/gimbal/recordings").then(function (body) {
            recordings = body.recordings || [];
            if (!selectedName && recordings[0]) selectRecording(recordings[0].name);
            render();
        });
    }
    document.addEventListener("DOMContentLoaded", function () {
        var refreshBtn = $("videosRefresh");
        var list = $("videoList");
        if (refreshBtn) refreshBtn.addEventListener("click", refresh);
        if (list) list.addEventListener("click", function (event) {
            var button = event.target.closest("[data-action]");
            var item = event.target.closest(".video-item");
            if (!button || !item) return;
            var name = decodeURIComponent(item.getAttribute("data-name") || "");
            var action = button.getAttribute("data-action");
            if (action === "select") selectRecording(name);
            if (action === "rename") renameRecording(name);
            if (action === "delete") deleteRecording(name);
        });
        refresh();
        window.setInterval(refresh, 5000);
    });
})();
