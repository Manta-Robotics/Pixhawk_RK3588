(function () {
    "use strict";

    var $ = function (selector, root) { return (root || document).querySelector(selector); };
    var $$ = function (selector, root) { return Array.prototype.slice.call((root || document).querySelectorAll(selector)); };
    var shell = $("#appShell");
    var toastTimer = null;
    var telemetryTimer = null;
    var recordTimerHandle = null;
    var downloadTimer = null;
    var logEntries = [];

    var state = {
        view: "device",
        connected: false,
        controlLink: false,
        connection: "idle",
        transportMode: "mock",
        layout: "standard",
        language: "zh",
        theme: "dark",
        latency: 18,
        speed: 0,
        temperature: 46,
        attitude: { roll: 1.2, pitch: -0.6, yaw: 184 },
        vector: { x: 0, y: 0 },
        recording: false,
        recordingSince: 0,
        videoLost: false,
        gimbalFault: false,
        emergency: false,
        downloading: false,
        logFilter: "all",
        osdEnabled: true,
        gps: { satellites: 0, hdop: null, estimated: true },
        calibrating: false,
        calibrationStarted: false,
        calibrationStepCode: null,
        calibrationStatus: "intro",
        calibrationConfirmed: 0,
        calibrationProgress: 0,
        calibrationResult: null,
        calibrationResultMessage: ""
    };

    var transport = window.MantaAppTransport.create(state.transportMode);
    var healthMonitor = new window.MantaAppTransport.GimbalHealthMonitor({
        angleThreshold: 0.8,
        windowMs: 800,
        requiredWindows: 3,
        onFault: function (report) { triggerGimbalFault(report, true); }
    });

    var CALIBRATION_POSES = [
        { code: 1, pose: "level", zh: "水平平放", en: "Level", instructionZh: "将 Manta 正常水平放置，保持完全静止。", instructionEn: "Place Manta level in its normal orientation and keep it completely still." },
        { code: 2, pose: "left", zh: "左侧朝下", en: "Left side down", instructionZh: "将 Manta 左侧朝下放稳，确认机身不再晃动。", instructionEn: "Rest Manta on its left side and wait until it is completely stable." },
        { code: 3, pose: "right", zh: "右侧朝下", en: "Right side down", instructionZh: "将 Manta 右侧朝下放稳，确认机身不再晃动。", instructionEn: "Rest Manta on its right side and wait until it is completely stable." },
        { code: 4, pose: "nosedown", zh: "机头朝下", en: "Nose down", instructionZh: "将机头垂直朝下并固定，保持静止。", instructionEn: "Hold the nose vertically downward, secure the robot, and keep it still." },
        { code: 5, pose: "noseup", zh: "机头朝上", en: "Nose up", instructionZh: "将机头垂直朝上并固定，保持静止。", instructionEn: "Hold the nose vertically upward, secure the robot, and keep it still." },
        { code: 6, pose: "back", zh: "背面朝下", en: "Back down", instructionZh: "将 Manta 翻转，使背面朝下放稳，保持静止。", instructionEn: "Turn Manta over, rest it on its back, and keep it completely still." }
    ];

    var STATIC_EN = {
        "请旋转设备": "Rotate your device", "Manta 仅提供横屏控制体验": "Manta is designed for landscape control",
        "连接": "LINK", "延迟": "LATENCY", "速度": "SPEED", "飞控温度": "FCU TEMP", "未连接": "Offline",
        "设备": "Device", "地图": "Map", "录像": "Media", "演示": "Demo", "附近设备": "Nearby device",
        "Manta 产品画面": "Manta product view", "发现附近的": "Nearby device", "可以连接": "Ready to connect",
        "通过蓝牙完成近距发现与安全配对；连接后以 5 GHz Wi‑Fi 传输影像，同时保留设备互联网连接。": "Use Bluetooth for nearby discovery and secure pairing. After connecting, 5 GHz Wi-Fi carries video while the device stays online.",
        "型号 ROBOTIC · 序列号 MANTA 2407": "Model ROBOTIC · Serial MANTA 2407", "发现与控制": "Discovery & control", "预览与下载": "Preview & download", "保持联网": "Stay online",
        "首次连接需要输入 Manta 显示的 6 位配对码": "First connection requires the 6-digit code shown by Manta", "近距离": "NEAR FIELD", "安全配对": "SECURE PAIRING",
        "让镜头跟上每一次破浪": "Keep the camera with every open-water move",
        "录像时间轴": "Recording timeline", "倍速": "Speed", "播放速度": "Playback speed", "暂停录像": "Pause recording",
        "让镜头": "Keep the camera", "跟上每一次破浪": "with every open-water move",
        "面向开放水域游泳与专业野泳运动员的自主水面摄影机器人。": "An autonomous surface camera robot for open-water swimmers and professionals.",
        "安全连接已就绪": "Secure connection ready", "BLE 5.0 发现": "BLE 5.0 discovery", "互联网连接保持在线": "Internet stays online", "保持联网": "Stay online",
        "连接 Manta": "Connect Manta", "蓝牙近距发现 · 5 GHz Wi‑Fi影像 · 保持互联网在线": "BLE discovery · 5 GHz video · Internet stays online",
        "手动跟随": "Manual follow", "自动跟随": "Auto follow", "即将推出": "Coming soon", "断开": "Disconnect",
        "飞控姿态": "Flight attitude", "IMU 校准": "IMU calibration", "横滚 ROLL": "ROLL", "俯仰 PITCH": "PITCH", "航向 YAW": "YAW", "校准状态": "Calibration", "良好": "Good",
        "移动控制": "Drive control", "前进": "FORWARD", "后退": "REVERSE", "左推进": "Left thrust", "右推进": "Right thrust", "紧急停止": "Emergency stop", "滑动立即停止": "Slide to stop now",
        "影像信号中断": "Video signal lost", "控制链路仍然在线，请谨慎移动": "Control link remains online. Move with caution.", "云台保护已触发": "Gimbal protection active", "检测到云台电机异常，所有云台功能已停止": "A gimbal motor fault was detected. All gimbal functions are stopped.", "查看诊断报告": "View diagnostic report",
        "点击居中": "Click center", "人脸跟踪": "Face track", "泳者跟踪": "Swimmer track",
        "输出日志": "Output log", "全部": "All", "指令": "Commands", "硬件": "Hardware", "上位机": "Host", "下位机": "Controller", "电机": "Motors", "云台": "Gimbal",
        "水域地图": "Water map", "定位 Manta": "Locate Manta", "规划路线": "Plan route", "当前航程": "Current route", "预计剩余 38 min": "Estimated 38 min remaining",
        "GPS 状态": "GPS status", "等待设备连接": "Waiting for device", "0 颗卫星": "0 satellites", "地图来源：MANTA 演示矢量 · 非导航地图": "Map source: MANTA demo vectors · Not for navigation",
        "Manta 录像": "Manta media", "机内存储": "On-device storage", "今天": "Today", "已下载": "Downloaded", "3 个项目": "3 items", "开放水域跟拍 01": "Open-water follow 01", "泳者跟踪测试": "Swimmer tracking test", "返航影像": "Return footage", "下载": "Download",
        "快捷工具": "Quick tools", "显示": "Display", "外观": "Appearance", "跟随使用环境": "Match your environment", "深色": "Dark", "浅色": "Light", "语言": "Language", "简体中文": "Simplified Chinese",
        "通信模式": "Transport mode", "Preview 模拟通信": "Preview simulation", "互联网": "Internet", "连接 Manta 时保持在线": "Stays online while connected to Manta", "完整日志": "Full logs", "导出诊断信息": "Export diagnostics", "帮助与安全": "Help & safety", "连接、控制与急停说明": "Connection, control, and emergency-stop guide",
        "状态模拟器": "State simulator", "用于在电脑端预览所有真实状态，不会向 Manta 发送指令。": "Preview real interface states on this computer without sending commands to Manta.", "正常连接": "Connected", "视频中断": "Video lost", "云台故障": "Gimbal fault", "温度告警": "Temperature alert", "急停": "Emergency", "设备离线": "Device offline",
        "输入 Manta 显示的配对码": "Enter the pairing code shown by Manta", "Preview 测试码：": "Preview test code: ", "验证并连接": "Verify and connect",
        "正在下载到 Manta App": "Downloading to Manta App", "下载完成前请保持 App 在前台并维持与 Manta 的连接。": "Keep the app in the foreground and maintain the Manta connection until the download finishes.", "下载期间无法切换页面": "Navigation is locked during download",
        "云台健康诊断报告": "Gimbal health report", "已执行云台停止保护": "Gimbal stop protection executed", "实际响应": "Actual response", "连续异常": "Consecutive faults", "通信": "Link", "处置": "Action", "当前结论是“疑似卡死”，不是硬件确定性判定。建议断电检查机械阻力、供电和串口错误，然后重新执行云台自检。": "The current result is a suspected stall, not a definitive hardware diagnosis. Power down, inspect mechanical resistance, power, and serial errors, then run gimbal self-test again.", "我知道了": "Got it",
        "IMU 六面校准": "Six-position IMU calibration", "校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。": "Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface.", "准备开始": "Ready", "准备六个放置方向": "Prepare six orientations", "开始后，请按照飞控要求逐面放稳并确认。": "After starting, follow each FCU pose request and confirm only when Manta is stable.", "取消": "Cancel", "开始校准": "Start calibration", "紧急停止推进": "Emergency stop thrust",
        "显示云台 OSD": "Show gimbal OSD", "云台回中": "Center gimbal", "停止云台": "Stop gimbal", "更多参数": "More settings",
        "下载完成前无法切换页面": "Navigation is locked until the download finishes", "云台健康监测触发保护，请查看诊断报告": "Gimbal health protection triggered. Open the diagnostic report.",
        "请先连接 Manta": "Connect to Manta first", "浏览器 Preview 不写入 App 沙盒；请在原生 App 录像页执行真实下载": "Browser Preview cannot write to the app sandbox. Use the native app media page for a real download.",
        "已切换真实通信，请连接 Manta": "Live transport selected. Connect to Manta.", "已切换 Preview 模拟通信": "Preview simulation selected",
        "控制区已移至左侧": "Controls moved to the left", "影像区已移至左侧": "Video moved to the left", "云台当前不可控制": "Gimbal control is currently unavailable",
        "云台正在回中": "Gimbal is centering", "云台已停止": "Gimbal stopped", "将由原生 App 请求相册权限后保存": "The native app will request Photos access before saving",
        "已定位 MANTA ROBOTIC": "MANTA ROBOTIC located",
        "返回设备主页": "Return to device home", "实时状态": "Live status", "交换控制台布局": "Swap console layout", "交换布局": "Swap layout", "打开工具菜单": "Open tools", "主要页面": "Main pages", "Manta 设备": "Manta device", "Manta 水面摄影机器人": "Manta surface camera robot", "设备连接能力": "Device connectivity", "自动跟随将在后续版本开放": "Auto follow will be available in a future version", "Manta 360度移动摇杆": "Manta 360-degree drive joystick", "停止": "Stopped", "向右滑动触发紧急停止": "Slide right to trigger emergency stop", "Manta 云台实时画面": "Live Manta gimbal view", "云台模式": "Gimbal mode", "播放录像": "Play recording", "关闭": "Close"
    };

    var staticTextRecords = null;
    var staticAttributeRecords = null;

    function localize(zh, en) { return state.language === "en" ? (en || STATIC_EN[zh] || zh) : zh; }

    function translatedStaticValue(value) {
        if (state.language !== "en") return value;
        if (STATIC_EN[value]) return STATIC_EN[value];
        var digit = /^第(\d)位$/.exec(value);
        return digit ? "Digit " + digit[1] : value;
    }

    function captureStaticContent() {
        if (staticTextRecords) return;
        staticTextRecords = [];
        var walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
        var node;
        while ((node = walker.nextNode())) {
            var raw = node.nodeValue;
            var trimmed = raw.trim();
            if (!trimmed || !node.parentElement || /^(SCRIPT|STYLE)$/.test(node.parentElement.tagName)) continue;
            staticTextRecords.push({ node: node, zh: trimmed, leading: (raw.match(/^\s*/) || [""])[0], trailing: (raw.match(/\s*$/) || [""])[0] });
        }
        staticAttributeRecords = [];
        $$('[title],[aria-label],[alt]').forEach(function (element) {
            ["title", "aria-label", "alt"].forEach(function (attribute) {
                if (element.hasAttribute(attribute)) staticAttributeRecords.push({ element: element, attribute: attribute, zh: element.getAttribute(attribute) });
            });
        });
    }

    function applyStaticLanguage() {
        captureStaticContent();
        staticTextRecords.forEach(function (record) {
            if (!record.node.isConnected) return;
            record.node.nodeValue = record.leading + translatedStaticValue(record.zh) + record.trailing;
        });
        staticAttributeRecords.forEach(function (record) {
            if (record.element.isConnected) record.element.setAttribute(record.attribute, translatedStaticValue(record.zh));
        });
    }

    function showToast(message, messageEn) {
        var node = $("#toast");
        clearTimeout(toastTimer);
        node.textContent = localize(message, messageEn);
        node.classList.add("visible");
        toastTimer = setTimeout(function () { node.classList.remove("visible"); }, 2400);
    }

    function formatTime(date) {
        return date.toLocaleTimeString(state.language === "zh" ? "zh-CN" : "en-US", { hour: "2-digit", minute: "2-digit", hour12: false });
    }

    function updateClock() { $("#clock").textContent = formatTime(new Date()); }

    function setModal(id, open) {
        var node = document.getElementById(id);
        if (!node) return;
        node.hidden = !open;
        if (open) {
            var focusable = $("input:not([disabled]),button:not([disabled])", node);
            if (focusable) setTimeout(function () { focusable.focus(); }, 30);
        }
    }

    function toggleDrawer(node, open) {
        node.classList.toggle("open", open);
        node.setAttribute("aria-hidden", String(!open));
    }

    function addLog(entry) {
        entry = entry || {};
        logEntries.push({
            timestamp: entry.timestamp || new Date().toISOString(),
            level: String(entry.level || "INFO").toUpperCase(),
            source: String(entry.source || "APP").toUpperCase(),
            message: String(entry.message || ""),
            messageEn: String(entry.messageEn || STATIC_EN[entry.message] || entry.message || ""),
            category: entry.category || "system"
        });
        if (logEntries.length > 120) logEntries.shift();
        renderLogs();
    }

    function renderLogs() {
        var root = $("#logStream");
        if (!root) return;
        var filtered = logEntries.filter(function (entry) { return state.logFilter === "all" || entry.category === state.logFilter; }).slice(-40);
        root.innerHTML = filtered.map(function (entry) {
            var date = new Date(entry.timestamp);
            var message = state.language === "en" ? entry.messageEn : entry.message;
            return '<div class="log-row" data-level="' + entry.level + '"><time>' + formatTime(date) + ':' + String(date.getSeconds()).padStart(2, "0") + '</time><b>' + escapeHtml(entry.source) + '</b><span>' + escapeHtml(message) + '</span></div>';
        }).join("");
        root.scrollTop = root.scrollHeight;
    }

    function escapeHtml(value) {
        return String(value).replace(/[&<>'"]/g, function (character) { return ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" })[character]; });
    }

    function bindTransport() {
        transport.on("log", addLog);
        transport.on("telemetry", function (telemetry) {
            if (!state.connected || state.emergency) return;
            if (Number.isFinite(Number(telemetry.latency)) && Number(telemetry.latency) > 0) state.latency = Number(telemetry.latency);
            if (Number.isFinite(Number(telemetry.speed))) state.speed = Number(telemetry.speed);
            if (Number.isFinite(Number(telemetry.temperature)) && Number(telemetry.temperature) > 0) state.temperature = Number(telemetry.temperature);
            if (telemetry.attitude) state.attitude = telemetry.attitude;
            if (telemetry.gps) {
                state.gps.satellites = Math.max(0, Number(telemetry.gps.satellites) || 0);
                state.gps.hdop = Number.isFinite(Number(telemetry.gps.hdop)) ? Number(telemetry.gps.hdop) : null;
                state.gps.estimated = true;
            }
            if (telemetry.imuCalibration) handleImuCalibrationTelemetry(telemetry.imuCalibration);
            renderTelemetry();
            renderGpsStatus();
        });
        transport.on("driveAck", renderDriveAck);
        transport.on("gimbalState", function (gimbalState) { healthMonitor.sample(gimbalState); });
        transport.on("connection", function (connection) {
            if (!connection) return;
            if (connection.connected) {
                state.controlLink = true;
                if (state.connected && !state.emergency && !state.videoLost) state.connection = "connected";
            } else if (connection.degraded) {
                state.controlLink = false;
                state.connection = "offline";
                state.speed = 0;
                if (window.resetMantaJoystick) window.resetMantaJoystick();
                showToast("控制链路已断开，本机控制已锁定；板端看门狗仍需实机验收", "Control link lost. Local control is locked; the board watchdog still requires hardware validation.");
            }
            renderConnection();
            renderTelemetry();
        });
        transport.on("emergency", function () { setEmergency(true); });
    }

    function renderConnection() {
        shell.dataset.connection = state.connection;
        var metric = $("#connectionMetric");
        var labels = {
            idle: state.language === "zh" ? "未连接" : "Offline",
            discovering: state.language === "zh" ? "正在发现" : "Discovering",
            pairing: state.language === "zh" ? "正在配对" : "Pairing",
            connecting: state.language === "zh" ? "正在连接" : "Connecting",
            connected: state.language === "zh" ? "已连接" : "Connected",
            degraded: state.language === "zh" ? "链路降级" : "Degraded",
            offline: state.language === "zh" ? "设备离线" : "Device Offline",
            emergency: state.language === "zh" ? "紧急停止" : "Emergency"
        };
        metric.textContent = labels[state.connection] || state.connection;
        $("#landingScreen").hidden = state.connected;
        $("#controlScreen").hidden = !state.connected;
        $("#joystick").setAttribute("aria-disabled", String(!state.connected || !state.controlLink || state.emergency || state.calibrating));
        $("#estopThumb").disabled = !state.connected || !state.controlLink || state.emergency;
        shell.dataset.calibrating = String(state.calibrating);
        if (state.connected && state.view === "device") syncVideoSource();
    }

    function renderTelemetry() {
        if (!state.connected) {
            $("#latencyMetric").textContent = "--";
            $("#speedMetric").textContent = "--";
            $("#temperatureMetric").textContent = "--";
            $(".temperature-bar b").style.width = "0";
            return;
        }
        $("#latencyMetric").textContent = Math.round(state.latency) + " ms";
        $("#speedMetric").textContent = state.speed.toFixed(1) + " m/s";
        $("#temperatureMetric").textContent = Math.round(state.temperature) + " °C";
        $(".temperature-bar b").style.width = Math.max(8, Math.min(100, (state.temperature - 20) / 75 * 100)) + "%";
        $("#rollValue").textContent = signedAngle(state.attitude.roll);
        $("#pitchValue").textContent = signedAngle(state.attitude.pitch);
        $("#yawValue").textContent = Math.round((Number(state.attitude.yaw) || 0) % 360) + "°";
        var orb = $("#attitudeOrb");
        var roll = Math.max(-25, Math.min(25, Number(state.attitude.roll) || 0));
        var pitch = Math.max(-18, Math.min(18, Number(state.attitude.pitch) || 0));
        $$(".attitude-sky,.attitude-water,.horizon,.pitch", orb).forEach(function (element) { element.style.transform = "translateY(" + pitch * 1.4 + "px) rotate(" + roll + "deg)"; });
    }

    function renderGpsStatus() {
        var card = $("#gpsSourceCard");
        if (!card) return;
        var satellites = Math.max(0, Number(state.gps.satellites) || 0);
        if (!state.connected) {
            card.dataset.state = "waiting";
            $("#gpsStatus").textContent = localize("等待设备连接", "Waiting for device");
        } else if (satellites >= 4) {
            card.dataset.state = "available";
            $("#gpsStatus").textContent = state.transportMode === "mock"
                ? localize("GPS 可用（模拟）", "GPS available (simulated)")
                : localize("GPS 信号可用（估算）", "GPS signal available (estimated)");
        } else {
            card.dataset.state = "unavailable";
            $("#gpsStatus").textContent = localize("GPS 暂不可用", "GPS currently unavailable");
        }
        var satelliteLabel = satellites + localize(" 颗卫星", " satellites");
        if (state.gps.hdop !== null && Number.isFinite(Number(state.gps.hdop))) satelliteLabel += " · HDOP " + Number(state.gps.hdop).toFixed(1);
        $("#gpsDetail").textContent = satelliteLabel;
        $("#mapSource").textContent = localize("地图来源：MANTA 演示矢量 · 非导航地图", "Map source: MANTA demo vectors · Not for navigation");
    }

    function signedAngle(value) {
        value = Number(value) || 0;
        return (value >= 0 ? "+" : "−") + Math.abs(value).toFixed(1) + "°";
    }

    function renderDriveAck(ack) {
        var left = Math.round(((Number(ack.leftPwm) || 1500) - 1500) / 5);
        var right = Math.round(((Number(ack.rightPwm) || 1500) - 1500) / 5);
        $("#leftMotor").textContent = signedPercent(left);
        $("#rightMotor").textContent = signedPercent(right);
        $("#leftMotorBar").style.width = Math.min(100, Math.abs(left)) + "%";
        $("#rightMotorBar").style.width = Math.min(100, Math.abs(right)) + "%";
    }

    function signedPercent(value) { return (value > 0 ? "+" : "") + value + "%"; }

    function showView(view) {
        if (state.downloading) { showToast("下载完成前无法切换页面"); return; }
        state.view = view;
        $$(".app-view").forEach(function (node) {
            var active = node.dataset.view === view;
            node.hidden = !active;
            requestAnimationFrame(function () { node.classList.toggle("active", active); });
        });
        $$(".nav-button[data-nav]").forEach(function (node) { node.classList.toggle("active", node.dataset.nav === view); });
        toggleDrawer($("#toolDrawer"), false);
        if (view !== "media") pauseMediaPreview();
        if (view === "map") renderGpsStatus();
        syncVideoSource();
    }

    function startConnection() {
        if (state.connected) return;
        state.connection = "discovering";
        renderConnection();
        var button = $("#connectButton");
        button.disabled = true;
        $("span", button).textContent = state.language === "zh" ? "正在发现附近设备" : "Discovering nearby device";
        transport.discover().then(function (devices) {
            if (!devices.length) throw new Error("附近没有发现 Manta");
            state.connection = "pairing";
            renderConnection();
            setModal("pairingModal", true);
            addLog({ level: "INFO", source: "BLE", message: "发现 MANTA ROBOTIC · MANTA 2407", messageEn: "Discovered MANTA ROBOTIC · MANTA 2407", category: "hardware" });
        }).catch(function (error) {
            state.connection = "idle";
            showToast(error.message);
            renderConnection();
        }).finally(function () {
            button.disabled = false;
            $("span", button).textContent = state.language === "zh" ? "连接 Manta" : "Connect Manta";
        });
    }

    function completePairing() {
        var pin = $$("#pinInputs input").map(function (input) { return input.value; }).join("");
        var button = $("#pairButton");
        button.disabled = true;
        button.textContent = state.language === "zh" ? "正在验证…" : "Verifying…";
        transport.pair(pin).then(function () {
            state.connection = "connecting";
            renderConnection();
            return transport.connect();
        }).then(function () {
            state.connected = true;
            state.controlLink = true;
            state.connection = "connected";
            state.emergency = false;
            setModal("pairingModal", false);
            $$("#pinInputs input").forEach(function (input) { input.value = ""; });
            renderConnection();
            showView("device");
            showToast(state.language === "zh" ? "Manta 已连接，互联网保持在线" : "Manta connected. Internet remains online.");
            addLog({ level: "INFO", source: "APP", message: "控制权已授予本机", messageEn: "Control granted to this device", category: "system" });
        }).catch(function (error) {
            showToast(error.message);
            state.connection = "pairing";
            renderConnection();
        }).finally(function () {
            button.textContent = state.language === "zh" ? "验证并连接" : "Verify and Connect";
            validatePin();
        });
    }

    function disconnect() {
        transport.drive({ x: 0, y: 0 });
        transport.disconnect().finally(function () {
            state.connected = false;
            state.controlLink = false;
            state.connection = "idle";
            state.speed = 0;
            state.videoLost = false;
            state.gimbalFault = false;
            state.emergency = false;
            if (state.calibrationStarted || state.calibrating) finishImuCalibration(false, localize("设备已断开，无法确认校准结果。", "The device disconnected before calibration could be confirmed."));
            stopRecording(false);
            renderConnection();
            renderTelemetry();
            showToast(state.language === "zh" ? "已断开 Manta" : "Manta disconnected");
        });
    }

    function validatePin() {
        var valid = $$("#pinInputs input").every(function (input) { return /^\d$/.test(input.value); });
        $("#pairButton").disabled = !valid;
    }

    function syncVideoSource() {
        var video = $("#gimbalVideo");
        var image = $("#liveGimbalFeed");
        var shouldRun = state.connected && state.view === "device";
        if (!shouldRun) {
            image.hidden = true;
            image.removeAttribute("src");
            video.pause();
            return;
        }
        if (state.transportMode === "live") {
            video.hidden = true;
            image.hidden = false;
            if (!image.src || !/api\/gimbal\/stream/.test(image.src)) image.src = "/api/gimbal/stream?app=" + Date.now();
        } else {
            image.hidden = true;
            image.removeAttribute("src");
            video.hidden = false;
            video.play().catch(function () {});
        }
    }

    function installJoystick() {
        var root = $("#joystick");
        var stick = $("#joyStick");
        var activePointer = null;

        function update(clientX, clientY) {
            if (!state.connected || !state.controlLink || state.emergency || state.calibrating) return;
            var rect = root.getBoundingClientRect();
            var radius = rect.width * .34;
            var dx = clientX - (rect.left + rect.width / 2);
            var dy = clientY - (rect.top + rect.height / 2);
            var distance = Math.hypot(dx, dy);
            if (distance > radius) { dx = dx / distance * radius; dy = dy / distance * radius; }
            state.vector = { x: dx / radius, y: dy / radius };
            stick.style.transform = "translate(calc(-50% + " + dx + "px), calc(-50% + " + dy + "px))";
            $("#driveOutput").textContent = "X " + state.vector.x.toFixed(2) + " · Y " + (-state.vector.y).toFixed(2);
            root.setAttribute("aria-valuetext", "X " + state.vector.x.toFixed(2) + ", Y " + (-state.vector.y).toFixed(2));
            transport.drive(state.vector);
        }

        function reset() {
            activePointer = null;
            state.vector = { x: 0, y: 0 };
            stick.style.transform = "translate(-50%, -50%)";
            $("#driveOutput").textContent = "X 0.00 · Y 0.00";
            root.setAttribute("aria-valuetext", state.language === "zh" ? "停止" : "Stopped");
            transport.drive(state.vector);
        }

        root.addEventListener("pointerdown", function (event) {
            if (!state.connected || !state.controlLink || state.emergency || state.calibrating) return;
            activePointer = event.pointerId;
            root.setPointerCapture(activePointer);
            update(event.clientX, event.clientY);
        });
        root.addEventListener("pointermove", function (event) { if (event.pointerId === activePointer) update(event.clientX, event.clientY); });
        root.addEventListener("pointerup", reset);
        root.addEventListener("pointercancel", reset);
        root.addEventListener("lostpointercapture", reset);
        root.addEventListener("keydown", function (event) {
            if (!state.connected || !state.controlLink || state.emergency || state.calibrating || !/^Arrow/.test(event.key)) return;
            event.preventDefault();
            var x = state.vector.x, y = state.vector.y;
            if (event.key === "ArrowLeft") x -= .1;
            if (event.key === "ArrowRight") x += .1;
            if (event.key === "ArrowUp") y -= .1;
            if (event.key === "ArrowDown") y += .1;
            var rect = root.getBoundingClientRect();
            update(rect.left + rect.width / 2 + clamp(x, -1, 1) * rect.width * .34, rect.top + rect.height / 2 + clamp(y, -1, 1) * rect.height * .34);
        });
        root.addEventListener("keyup", function (event) { if (/^Arrow/.test(event.key)) reset(); });
        window.addEventListener("blur", reset);
        document.addEventListener("visibilitychange", function () { if (document.hidden) reset(); });
        window.addEventListener("pagehide", reset);
        window.resetMantaJoystick = reset;
    }

    function clamp(value, min, max) { return Math.max(min, Math.min(max, Number(value) || 0)); }

    function installEmergencySlider() {
        var track = $("#estopSlider"), thumb = $("#estopThumb"), fill = $(".estop-fill", track);
        var pointer = null, position = 0;
        function setPosition(next) {
            var max = Math.max(1, track.clientWidth - thumb.offsetWidth - 6);
            position = clamp(next, 0, max);
            thumb.style.transform = "translateX(" + position + "px)";
            fill.style.width = position + thumb.offsetWidth + 3 + "px";
            return position / max;
        }
        function reset() { pointer = null; position = 0; thumb.style.transform = ""; fill.style.width = "0"; }
        thumb.addEventListener("pointerdown", function (event) { if (!state.connected || !state.controlLink || state.emergency) return; pointer = event.pointerId; thumb.setPointerCapture(pointer); });
        thumb.addEventListener("pointermove", function (event) {
            if (event.pointerId !== pointer) return;
            var rect = track.getBoundingClientRect();
            setPosition(event.clientX - rect.left - thumb.offsetWidth / 2);
        });
        thumb.addEventListener("pointerup", function () {
            var max = Math.max(1, track.clientWidth - thumb.offsetWidth - 6);
            if (position / max >= .86) transport.emergencyStop().catch(function (error) { showToast(error.message); reset(); });
            else reset();
        });
        thumb.addEventListener("pointercancel", reset);
        window.resetEmergencySlider = reset;
    }

    function setEmergency(active) {
        state.emergency = active;
        state.connection = active ? "emergency" : (state.connected ? "connected" : "idle");
        state.speed = 0;
        if (window.resetMantaJoystick) window.resetMantaJoystick();
        renderConnection();
        renderTelemetry();
        if (active) showToast("紧急停止已执行，推进输出已归零", "Emergency stop executed. Thrust output is zero.");
    }

    function renderGimbalOsd() {
        var button = $("#gimbalOsd");
        if (!button) return;
        button.classList.toggle("active", state.osdEnabled);
        button.setAttribute("aria-pressed", String(state.osdEnabled));
        button.setAttribute("title", state.osdEnabled ? localize("隐藏云台 OSD", "Hide gimbal OSD") : localize("显示云台 OSD", "Show gimbal OSD"));
    }

    function toggleGimbalOsd() {
        if (!state.controlLink || state.gimbalFault) {
            showToast("云台当前不可控制", "Gimbal control is currently unavailable");
            return;
        }
        var button = $("#gimbalOsd");
        var next = !state.osdEnabled;
        button.disabled = true;
        transport.setGimbalOsd(next).then(function () {
            state.osdEnabled = next;
            renderGimbalOsd();
            showToast(next ? "云台 OSD 已显示" : "云台 OSD 已隐藏", next ? "Gimbal OSD is visible" : "Gimbal OSD is hidden");
        }).catch(function (error) {
            showToast(error.message);
        }).finally(function () {
            button.disabled = state.gimbalFault;
        });
    }

    function setCalibrationMotionLock(locked) {
        state.calibrating = Boolean(locked);
        shell.dataset.calibrating = String(state.calibrating);
        if (transport.setMotionLocked) transport.setMotionLocked(state.calibrating);
        if (state.calibrating && window.resetMantaJoystick) window.resetMantaJoystick();
        renderConnection();
    }

    function calibrationPoseByCode(code) {
        return CALIBRATION_POSES.find(function (pose) { return pose.code === Number(code); }) || null;
    }

    function resetCalibrationWizard() {
        state.calibrationStarted = false;
        state.calibrationStepCode = null;
        state.calibrationStatus = "intro";
        state.calibrationConfirmed = 0;
        state.calibrationProgress = 0;
        state.calibrationResult = null;
        state.calibrationResultMessage = "";
    }

    function openImuCalibrationWizard() {
        if (!state.connected || !state.controlLink || state.emergency) {
            showToast("请先稳定连接 Manta，再开始 IMU 校准", "Connect to Manta with a stable control link before IMU calibration.");
            return;
        }
        resetCalibrationWizard();
        renderCalibrationWizard();
        setModal("imuCalibrationModal", true);
    }

    function closeImuCalibrationWizard() {
        if (state.calibrating) {
            showToast("校准已开始，请完成飞控要求的六个姿态", "Calibration is active. Complete all six poses requested by the FCU.");
            return;
        }
        setModal("imuCalibrationModal", false);
    }

    function renderCalibrationWizard() {
        var title = $("#imuCalibrationTitle");
        var lead = $("#imuCalibrationLead");
        var progressText = $("#imuCalibrationProgressText");
        var counter = $("#imuCalibrationCounter");
        var bar = $("#imuCalibrationBar");
        var stage = $("#imuPoseStage");
        var poseTitle = $("#imuPoseTitle");
        var instruction = $("#imuPoseInstruction");
        var next = $("#imuCalibrationNext");
        var cancel = $("#imuCalibrationCancel");
        var close = $("#imuCalibrationClose");
        var emergency = $("#imuCalibrationEmergency");
        if (!title) return;

        title.textContent = localize("IMU 六面校准", "Six-position IMU calibration");
        lead.textContent = localize("校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。", "Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface.");
        counter.textContent = Math.min(6, state.calibrationConfirmed) + " / 6";
        bar.style.width = Math.max(0, Math.min(100, Number(state.calibrationProgress) || 0)) + "%";
        cancel.hidden = state.calibrationStarted || Boolean(state.calibrationResult);
        close.hidden = state.calibrationStarted && !state.calibrationResult;
        emergency.hidden = !state.calibrationStarted || Boolean(state.calibrationResult);
        next.disabled = false;

        if (state.calibrationResult) {
            stage.dataset.pose = state.calibrationResult === "success" ? "level" : "intro";
            progressText.textContent = state.calibrationResult === "success" ? localize("飞控已确认完成", "FCU confirmed completion") : localize("校准未完成", "Calibration incomplete");
            poseTitle.textContent = state.calibrationResult === "success" ? localize("IMU 校准完成", "IMU calibration complete") : localize("IMU 校准失败", "IMU calibration failed");
            instruction.textContent = state.calibrationResultMessage || localize("请检查飞控状态后重试。", "Check the FCU status and try again.");
            next.textContent = localize("完成", "Done");
            counter.textContent = state.calibrationResult === "success" ? "6 / 6" : counter.textContent;
            bar.style.width = state.calibrationResult === "success" ? "100%" : bar.style.width;
            return;
        }

        if (!state.calibrationStarted) {
            stage.dataset.pose = "intro";
            progressText.textContent = localize("准备开始", "Ready");
            poseTitle.textContent = localize("准备六个放置方向", "Prepare six orientations");
            instruction.textContent = localize("开始后，请按照飞控要求逐面放稳并确认。", "After starting, follow each FCU pose request and confirm only when Manta is stable.");
            next.textContent = localize("开始校准", "Start calibration");
            return;
        }

        var pose = calibrationPoseByCode(state.calibrationStepCode);
        if (!pose) {
            stage.dataset.pose = "intro";
            progressText.textContent = localize("等待飞控姿态请求", "Waiting for FCU pose request");
            poseTitle.textContent = localize("不要自行切换方向", "Do not choose a pose manually");
            instruction.textContent = state.transportMode === "live"
                ? localize("仅当遥测状态为 AWAITING_POSITION 且包含有效 stepCode 时才能确认。", "Confirmation is enabled only when telemetry reports AWAITING_POSITION with a valid stepCode.")
                : localize("正在准备第一个模拟姿态。", "Preparing the first simulated pose.");
            next.textContent = localize("等待飞控确认", "Waiting for FCU");
            next.disabled = true;
            return;
        }

        stage.dataset.pose = pose.pose;
        progressText.textContent = state.calibrationStatus === "confirming"
            ? localize("确认命令已发送，等待飞控接受", "Confirmation sent; waiting for FCU acceptance")
            : localize("飞控请求当前姿态", "FCU requested this pose");
        poseTitle.textContent = localize(pose.zh, pose.en);
        instruction.textContent = localize(pose.instructionZh, pose.instructionEn);
        next.textContent = state.calibrationStatus === "confirming"
            ? localize("等待飞控确认", "Waiting for FCU")
            : localize("已放稳，确认此面", "Stable — confirm this pose");
        next.disabled = state.calibrationStatus === "confirming";
    }

    function beginImuCalibration() {
        if (state.calibrationStarted) return;
        state.calibrationStarted = true;
        state.calibrationStatus = "starting";
        setCalibrationMotionLock(true);
        renderCalibrationWizard();
        transport.startImuCalibration("ACCEL").then(function (result) {
            if (state.transportMode === "mock") {
                state.calibrationStepCode = 1;
                state.calibrationStatus = "awaiting_position";
            } else {
                state.calibrationStepCode = null;
                state.calibrationStatus = "waiting_fcu";
                var calibration = result && result.telemetry && result.telemetry.imuCalibration;
                if (calibration) handleImuCalibrationTelemetry(calibration);
            }
            addLog({ level: "COMMAND", source: "PIXHAWK", message: "IMU 六面校准已启动，推进控制已锁定", messageEn: "Six-position IMU calibration started; thrust control is locked", category: "command" });
            renderCalibrationWizard();
        }).catch(function (error) {
            finishImuCalibration(false, error.message);
        });
    }

    function confirmCurrentImuPose() {
        if (state.calibrationResult) { closeImuCalibrationWizard(); return; }
        if (!state.calibrationStarted) { beginImuCalibration(); return; }
        var pose = calibrationPoseByCode(state.calibrationStepCode);
        if (!pose || state.calibrationStatus === "confirming") return;
        if (state.transportMode === "live" && state.calibrationStatus !== "awaiting_position") {
            showToast("仍在等待飞控的 AWAITING_POSITION 状态", "Still waiting for the FCU AWAITING_POSITION state.");
            return;
        }
        state.calibrationStatus = "confirming";
        renderCalibrationWizard();
        transport.confirmImuCalibration(pose.code).then(function () {
            state.calibrationConfirmed = Math.max(state.calibrationConfirmed, CALIBRATION_POSES.indexOf(pose) + 1);
            state.calibrationProgress = state.calibrationConfirmed / 6 * 100;
            if (state.transportMode === "mock") {
                if (pose.code === 6) {
                    setTimeout(function () { finishImuCalibration(true); }, 500);
                } else {
                    state.calibrationStepCode = pose.code + 1;
                    state.calibrationStatus = "awaiting_position";
                    renderCalibrationWizard();
                }
            } else {
                state.calibrationStepCode = null;
                state.calibrationStatus = "waiting_fcu";
                showToast("确认命令已发送；HTTP 成功不代表该面已完成", "Confirmation sent; HTTP success does not mean the pose is complete.");
                renderCalibrationWizard();
            }
        }).catch(function (error) {
            state.calibrationStatus = "awaiting_position";
            showToast(error.message);
            renderCalibrationWizard();
        });
    }

    function handleImuCalibrationTelemetry(calibration) {
        if (!state.calibrationStarted || state.transportMode !== "live") return;
        var status = String(calibration.status || "").toUpperCase();
        var progress = Number(calibration.progress);
        if (Number.isFinite(progress)) state.calibrationProgress = Math.max(0, Math.min(100, progress));
        if (/COMPLETE|SUCCESS|DROBOTIC/.test(status)) {
            finishImuCalibration(true, calibration.instructions);
            return;
        }
        if (/FAIL|ERROR|CANCEL/.test(status)) {
            finishImuCalibration(false, calibration.instructions || status);
            return;
        }
        if (calibration.active === false) {
            if (state.calibrationProgress >= 100) finishImuCalibration(true, calibration.instructions);
            return;
        }
        if (status === "AWAITING_POSITION" && calibrationPoseByCode(calibration.stepCode)) {
            state.calibrationStepCode = Number(calibration.stepCode);
            state.calibrationStatus = "awaiting_position";
        } else {
            state.calibrationStepCode = null;
            state.calibrationStatus = "waiting_fcu";
        }
        renderCalibrationWizard();
    }

    function finishImuCalibration(success, message) {
        state.calibrationResult = success ? "success" : "failed";
        state.calibrationResultMessage = message || (success ? localize("飞控已确认六面校准完成。", "The FCU confirmed all six calibration poses.") : localize("请检查飞控状态后重试。", "Check the FCU status and try again."));
        state.calibrationStarted = false;
        state.calibrationStepCode = null;
        state.calibrationStatus = success ? "complete" : "failed";
        if (success) state.calibrationProgress = 100;
        setCalibrationMotionLock(false);
        $("#imuState").textContent = success ? localize("良好", "Good") : localize("需校准", "Needs calibration");
        renderCalibrationWizard();
        showToast(success ? "IMU 六面校准完成" : "IMU 校准未完成", success ? "Six-position IMU calibration complete" : "IMU calibration incomplete");
    }

    function setRecording(active) {
        if (active === state.recording) return;
        if (active) {
            state.recording = true;
            state.recordingSince = Date.now();
            $("#recordTimer").hidden = false;
            $("#recordButton").classList.add("active");
            clearInterval(recordTimerHandle);
            recordTimerHandle = setInterval(renderRecordTimer, 500);
            renderRecordTimer();
            transport.gimbal("recordStart").catch(function (error) { stopRecording(false); showToast(error.message); });
        } else stopRecording(true);
    }

    function stopRecording(sendCommand) {
        if (!state.recording && !recordTimerHandle) return;
        state.recording = false;
        clearInterval(recordTimerHandle);
        recordTimerHandle = null;
        $("#recordTimer").hidden = true;
        $("#recordButton").classList.remove("active");
        if (sendCommand) transport.gimbal("recordStop").catch(function (error) { showToast(error.message); });
    }

    function renderRecordTimer() {
        var elapsed = Math.floor((Date.now() - state.recordingSince) / 1000);
        $("#recordTimer").innerHTML = "<i></i>REC " + String(Math.floor(elapsed / 60)).padStart(2, "0") + ":" + String(elapsed % 60).padStart(2, "0");
    }

    function triggerGimbalFault(report, fromMonitor) {
        if (state.gimbalFault) return;
        state.gimbalFault = true;
        report = report || {};
        $("#gimbalFault").hidden = false;
        $("#gimbalFaultMessage").textContent = state.language === "en"
            ? (report.messageEn || "A gimbal motor fault was detected. All gimbal functions are stopped.")
            : (report.message || "检测到云台电机异常，所有云台功能已停止");
        $("#diagnosticCode").textContent = report.code || "GMB-UNKNOWN";
        $("#diagnosticCommand").textContent = report.evidence && report.evidence.commandRate
            ? localize("目标角速度 ", "Target angular rate ") + Number(report.evidence.commandRate).toFixed(1) + "°/s"
            : localize("运动目标已下发", "Motion target was issued");
        $("#diagnosticResponse").textContent = report.evidence && Number.isFinite(Number(report.evidence.delta))
            ? localize("检测窗口内角度变化 ", "Angle change in detection window ") + Number(report.evidence.delta).toFixed(2) + "°"
            : (state.language === "en" ? (report.messageEn || "No valid motion feedback") : (report.message || "无有效运动反馈"));
        $("#diagnosticWindows").textContent = (report.evidence && report.evidence.windows ? report.evidence.windows : 3) + localize(" / 3 个检测窗口", " / 3 detection windows");
        $("#diagnosticLink").textContent = report.code === "GMB-FEEDBACK-TIMEOUT"
            ? localize("反馈包超时", "Feedback packet timed out")
            : localize("状态包在线 · 疑似机械卡滞", "Status packets online · Suspected mechanical stall");
        $("#diagnosticAction").textContent = localize("停止跟踪、停止云台电机、保留主机移动控制", "Stop tracking and gimbal motors; keep vehicle drive control");
        transport.gimbal("stop").catch(function () {});
        $$("[data-gimbal-mode], [data-gimbal-action], #gimbalOsd").forEach(function (button) { button.disabled = true; });
        if (state.recording) stopRecording(true);
        addLog({ level: "CRITICAL", source: "GIMBAL", message: (report.code ? report.code + " · " : "") + "已触发云台停止保护", messageEn: (report.code ? report.code + " · " : "") + "Gimbal stop protection triggered", category: "hardware" });
        if (fromMonitor) showToast("云台健康监测触发保护，请查看诊断报告");
    }

    function setVideoLost(active) {
        state.videoLost = active;
        $("#videoAlertTitle").textContent = localize("影像信号中断", "Video signal lost");
        $("#videoAlertMessage").textContent = localize("控制链路仍然在线，请谨慎移动", "Control link remains online. Move with caution.");
        $("#videoAlert").hidden = !active;
        state.connection = active ? "degraded" : (state.emergency ? "emergency" : "connected");
        renderConnection();
        if (active) addLog({ level: "WARNING", source: "VIDEO", message: "影像信号中断，控制链路保持在线", messageEn: "Video signal lost; control link remains online", category: "hardware" });
    }

    function renderMediaPreviewButton() {
        var video = $(".featured video");
        var button = $(".featured .play-button");
        if (!video || !button) return;
        var playing = !video.paused && !video.ended;
        button.classList.toggle("playing", playing);
        button.hidden = playing;
        button.setAttribute("aria-label", localize("播放录像", "Play recording"));
        renderMediaControls();
    }

    function mediaTime(value) {
        value = Number.isFinite(Number(value)) ? Math.max(0, Number(value)) : 0;
        return String(Math.floor(value / 60)).padStart(2, "0") + ":" + String(Math.floor(value % 60)).padStart(2, "0");
    }

    function renderMediaControls() {
        var video = $(".featured video");
        var toggle = $("#mediaPlayPause");
        var scrubber = $("#mediaScrubber");
        if (!video || !toggle || !scrubber) return;
        var duration = Number.isFinite(video.duration) ? video.duration : 0;
        var current = Number.isFinite(video.currentTime) ? video.currentTime : 0;
        toggle.dataset.state = video.paused || video.ended ? "paused" : "playing";
        toggle.setAttribute("aria-label", video.paused || video.ended ? localize("播放录像", "Play recording") : localize("暂停录像", "Pause recording"));
        scrubber.max = String(Math.max(duration, 0.01));
        if (!scrubber.matches(":active")) scrubber.value = String(Math.min(current, duration || current));
        scrubber.style.setProperty("--media-progress", (duration ? current / duration * 100 : 0).toFixed(2) + "%");
        $("#mediaCurrentTime").textContent = mediaTime(current);
        $("#mediaDuration").textContent = mediaTime(duration);
        $("#mediaPlaybackRate").value = String(video.playbackRate);
    }

    function pauseMediaPreview() {
        var video = $(".featured video");
        if (video && !video.paused) video.pause();
        renderMediaPreviewButton();
    }

    function playMediaPreview() {
        var video = $(".featured video");
        if (!video || !video.paused) return;
        if (video.paused) {
            var button = $(".featured .play-button");
            if (button) button.hidden = true;
            video.currentTime = Math.max(0, video.currentTime || 0);
            video.play().then(function () {
                renderMediaPreviewButton();
                showToast("正在预览机内录像", "Previewing the on-device recording");
            }).catch(function (error) {
                if (button) button.hidden = false;
                renderMediaPreviewButton();
                showToast("录像无法播放：" + error.message, "Unable to play recording: " + error.message);
            });
        }
    }

    function toggleMediaPlayback() {
        var video = $(".featured video");
        if (!video) return;
        if (video.paused || video.ended) playMediaPreview();
        else video.pause();
    }

    function startDownload(button) {
        if (!state.connected && state.transportMode === "live") { showToast("请先连接 Manta"); return; }
        if (state.transportMode === "live") { showToast("浏览器 Preview 不写入 App 沙盒；请在原生 App 录像页执行真实下载"); return; }
        state.downloading = true;
        setModal("downloadModal", true);
        $("#downloadBar").style.width = "0%";
        $("#downloadPercent").textContent = "0%";
        $("#downloadSize").textContent = "0 MB / 19.4 MB";
        $("#downloadTitle").textContent = localize("正在下载到 Manta App", "Downloading to Manta App");
        var progress = 0;
        clearInterval(downloadTimer);
        downloadTimer = setInterval(function () {
            progress = Math.min(100, progress + 2 + Math.random() * 5);
            $("#downloadBar").style.width = progress + "%";
            $("#downloadPercent").textContent = Math.floor(progress) + "%";
            $("#downloadSize").textContent = (19.4 * progress / 100).toFixed(1) + " MB / 19.4 MB";
            if (progress >= 100) {
                clearInterval(downloadTimer);
                state.downloading = false;
                $("#downloadTitle").textContent = localize("下载完成", "Download complete");
                $("#cancelDownload").disabled = false;
                $("#cancelDownload").textContent = localize("完成", "Done");
                button.textContent = localize("保存到相册", "Save to Photos");
                button.classList.add("downloaded");
                $("#mediaBadge").hidden = false;
                addLog({ level: "INFO", source: "MEDIA", message: "录像已保存到 App 素材库", messageEn: "Recording saved to the app media library", category: "system" });
            }
        }, 180);
    }

    function resetDemoFaults() {
        state.controlLink = state.connected;
        state.videoLost = false;
        state.gimbalFault = false;
        state.emergency = false;
        state.temperature = 46;
        state.latency = 18;
        state.connection = state.connected ? "connected" : "idle";
        $("#videoAlertTitle").textContent = localize("影像信号中断", "Video signal lost");
        $("#videoAlertMessage").textContent = localize("控制链路仍然在线，请谨慎移动", "Control link remains online. Move with caution.");
        $("#videoAlert").hidden = true;
        $("#gimbalFault").hidden = true;
        $$("[data-gimbal-mode], [data-gimbal-action], #gimbalOsd").forEach(function (button) { button.disabled = false; });
        renderGimbalOsd();
        healthMonitor.reset();
        if (window.resetEmergencySlider) window.resetEmergencySlider();
    }

    function applyDemoState(name) {
        if (!state.connected) {
            state.connected = true; state.connection = "connected"; state.transportMode = "mock";
            state.controlLink = true;
            if (transport.mode !== "mock") switchTransport("mock", false);
            transport.connect();
        }

        // Demo buttons represent mutually exclusive scenarios. Starting from a
        // clean baseline prevents a previous video or gimbal fault leaking into
        // the next state while still allowing the sliders to tune that state.
        resetDemoFaults();

        if (name === "videoLost") setVideoLost(true);
        if (name === "gimbalFault") triggerGimbalFault({ code: "GMB-MOTION-STALL", message: "连续三个检测窗口内角度与陀螺仪无响应", messageEn: "No angle or gyroscope response across three consecutive detection windows" }, false);
        if (name === "hot") { state.temperature = 86; addLog({ level: "WARNING", source: "PIXHAWK", message: "飞控温度进入警戒区间", messageEn: "Flight-controller temperature entered the warning range", category: "hardware" }); }
        if (name === "emergency") setEmergency(true);
        if (name === "offline") { state.controlLink = false; state.connection = "offline"; state.speed = 0; state.videoLost = true; $("#videoAlertTitle").textContent = localize("设备连接已断开", "Device connection lost"); $("#videoAlertMessage").textContent = localize("控制与遥测已锁定，等待板端看门狗确认停车", "Control and telemetry are locked while the board watchdog confirms stop."); $("#videoAlert").hidden = false; if (window.resetMantaJoystick) window.resetMantaJoystick(); addLog({ level: "ERROR", source: "LINK", message: "控制和遥测链路已断开，本机控制已锁定", messageEn: "Control and telemetry links were lost; local control is locked", category: "hardware" }); }
        renderConnection(); renderTelemetry();
        if (state.connected) showView("device");
    }

    function switchTransport(mode, notify) {
        if (state.downloading) return;
        if (state.connected) transport.disconnect();
        state.connected = false; state.controlLink = false; state.connection = "idle"; state.transportMode = mode;
        transport = window.MantaAppTransport.create(mode);
        bindTransport();
        renderTransportLabels();
        renderConnection(); syncVideoSource();
        if (notify !== false) showToast(mode === "live" ? "已切换真实通信，请连接 Manta" : "已切换 Preview 模拟通信");
    }

    function renderTransportLabels() {
        $("#transportToggle").textContent = state.transportMode === "live" ? "LIVE" : "SIM";
        $("#transportHint").textContent = state.transportMode === "live"
            ? localize("当前 Manta 后端接口", "Current Manta backend")
            : localize("Preview 模拟通信", "Preview simulation");
        $("#pairingHint").innerHTML = state.transportMode === "live"
            ? localize("板端尚无配对令牌端点；当前仅验证六位格式", "The board has no pairing-token endpoint yet; only the six-digit format is checked.")
            : localize("Preview 测试码：", "Preview test code: ") + "<b>240724</b>";
    }

    function applyLanguage(language) {
        state.language = language;
        document.body.dataset.language = language;
        document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
        applyStaticLanguage();
        renderTransportLabels();
        renderConnection();
        renderTelemetry();
        renderGpsStatus();
        renderGimbalOsd();
        renderCalibrationWizard();
        renderMediaPreviewButton();
        if (state.calibrationResult) $("#imuState").textContent = state.calibrationResult === "success" ? localize("良好", "Good") : localize("需校准", "Needs calibration");
        renderLogs();
        updateClock();
        clearTimeout(toastTimer);
        $("#toast").textContent = "";
        $("#toast").classList.remove("visible");
    }

    function cancelPairing() {
        if (state.connected) return;
        state.connection = "idle";
        $$("#pinInputs input").forEach(function (input) { input.value = ""; });
        validatePin();
        setModal("pairingModal", false);
        renderConnection();
    }

    function seedLogs() {
        [
            { source: "APP", message: "Manta App Preview 已启动", messageEn: "Manta App Preview started", category: "system" },
            { source: "BLE", message: "近距离发现模块待机", messageEn: "Nearby discovery is standing by", category: "hardware" },
            { source: "PIXHAWK", message: "等待设备连接", messageEn: "Waiting for device connection", category: "hardware" },
            { source: "GIMBAL", message: "测试视频已载入", messageEn: "Test video loaded", category: "hardware" }
        ].forEach(addLog);
    }

    function installEvents() {
        $("#connectButton").addEventListener("click", startConnection);
        $("#pairButton").addEventListener("click", completePairing);
        $("#disconnectButton").addEventListener("click", disconnect);
        $$("#pinInputs input").forEach(function (input, index, inputs) {
            input.addEventListener("input", function () {
                input.value = input.value.replace(/\D/g, "").slice(-1);
                if (input.value && inputs[index + 1]) inputs[index + 1].focus();
                validatePin();
            });
            input.addEventListener("keydown", function (event) { if (event.key === "Backspace" && !input.value && inputs[index - 1]) inputs[index - 1].focus(); });
        });
        $$("[data-close-modal]").forEach(function (button) { button.addEventListener("click", function () { if (button.dataset.closeModal === "pairingModal") cancelPairing(); else setModal(button.dataset.closeModal, false); }); });
        $$("[data-nav]").forEach(function (button) { button.addEventListener("click", function () { showView(button.dataset.nav); }); });
        $("#toolToggle").addEventListener("click", function () { toggleDrawer($("#toolDrawer"), !$("#toolDrawer").classList.contains("open")); toggleDrawer($("#demoConsole"), false); });
        $("#toolClose").addEventListener("click", function () { toggleDrawer($("#toolDrawer"), false); });
        $("#demoToggle").addEventListener("click", function () { toggleDrawer($("#demoConsole"), !$("#demoConsole").classList.contains("open")); toggleDrawer($("#toolDrawer"), false); });
        $("#demoClose").addEventListener("click", function () { toggleDrawer($("#demoConsole"), false); });
        $("#layoutToggle").addEventListener("click", function () { state.layout = state.layout === "standard" ? "swapped" : "standard"; shell.dataset.layout = state.layout; showToast(state.layout === "standard" ? "控制区已移至左侧" : "影像区已移至左侧"); });
        $("#themeSelect").addEventListener("change", function (event) { state.theme = event.target.value; document.body.dataset.theme = state.theme; });
        $("#languageSelect").addEventListener("change", function (event) { applyLanguage(event.target.value); });
        $("#transportToggle").addEventListener("click", function () { switchTransport(state.transportMode === "mock" ? "live" : "mock"); });
        $$("[data-demo-state]").forEach(function (button) { button.addEventListener("click", function () { applyDemoState(button.dataset.demoState); }); });
        ["Latency", "Speed", "Temp"].forEach(function (name) {
            var input = $("#demo" + name), output = $("#demo" + name + "Output");
            input.addEventListener("input", function () {
                var value = Number(input.value);
                if (name === "Latency") { state.latency = value; output.textContent = value + " ms"; }
                if (name === "Speed") { state.speed = value / 10; output.textContent = state.speed.toFixed(1) + " m/s"; }
                if (name === "Temp") { state.temperature = value; output.textContent = value + " °C"; }
                renderTelemetry();
            });
        });
        $$("[data-gimbal-mode]").forEach(function (button) { button.addEventListener("click", function () { if (!state.controlLink || state.gimbalFault) { showToast("云台当前不可控制"); return; } $$("[data-gimbal-mode]").forEach(function (item) { item.classList.toggle("active", item === button); }); $("#trackingReticle").hidden = button.dataset.gimbalMode === "click"; transport.gimbal(button.dataset.gimbalMode).catch(function (error) { showToast(error.message); }); }); });
        $$("[data-gimbal-action]").forEach(function (button) { button.addEventListener("click", function () { if (!state.controlLink || state.gimbalFault) { showToast("云台当前不可控制"); return; } var action = button.dataset.gimbalAction; if (action === "record") setRecording(!state.recording); else transport.gimbal(action, action === "home" ? { preserveTracking: true } : {}).then(function () { showToast(action === "home" ? "云台正在回中" : "云台已停止"); }).catch(function (error) { showToast(error.message); }); }); });
        $("#openReport").addEventListener("click", function () { setModal("diagnosticModal", true); });
        $("#gimbalOsd").addEventListener("click", toggleGimbalOsd);
        $("#imuCalibrate").addEventListener("click", openImuCalibrationWizard);
        $("#imuCalibrationClose").addEventListener("click", closeImuCalibrationWizard);
        $("#imuCalibrationCancel").addEventListener("click", closeImuCalibrationWizard);
        $("#imuCalibrationNext").addEventListener("click", confirmCurrentImuPose);
        $("#imuCalibrationEmergency").addEventListener("click", function () {
            transport.emergencyStop().then(function () {
                finishImuCalibration(false, localize("用户在校准期间执行了紧急停止。", "Emergency stop was executed during calibration."));
                setEmergency(true);
                setModal("imuCalibrationModal", false);
            }).catch(function (error) { showToast(error.message); });
        });
        $$(".log-filter button").forEach(function (button) { button.addEventListener("click", function () { state.logFilter = button.dataset.logFilter; $$(".log-filter button").forEach(function (item) { item.classList.toggle("active", item === button); }); renderLogs(); }); });
        $$(".download-button").forEach(function (button) { button.addEventListener("click", function () { if (button.classList.contains("downloaded")) showToast("将由原生 App 请求相册权限后保存"); else startDownload(button); }); });
        $("#cancelDownload").addEventListener("click", function () { if (!state.downloading) { setModal("downloadModal", false); $("#cancelDownload").disabled = true; $("#cancelDownload").textContent = localize("下载期间无法切换页面", "Navigation is locked during download"); } });
        $("#locateButton").addEventListener("click", function () { $(".manta-marker").animate([{ transform: "scale(1)" }, { transform: "scale(1.18)" }, { transform: "scale(1)" }], { duration: 650 }); showToast("已定位 MANTA ROBOTIC"); });
        $(".featured .play-button").addEventListener("click", playMediaPreview);
        $("#mediaPlayPause").addEventListener("click", toggleMediaPlayback);
        $("#mediaScrubber").addEventListener("input", function (event) {
            var video = $(".featured video");
            if (!video || !Number.isFinite(video.duration)) return;
            video.currentTime = Math.max(0, Math.min(video.duration, Number(event.target.value) || 0));
            renderMediaControls();
        });
        $("#mediaPlaybackRate").addEventListener("change", function (event) {
            var video = $(".featured video");
            var rate = Number(event.target.value) || 1;
            video.playbackRate = rate;
            renderMediaControls();
            showToast("已切换至 " + rate + " 倍播放", "Playback speed set to " + rate + "×");
        });
        ["loadedmetadata", "durationchange", "timeupdate", "ratechange", "play", "pause", "ended"].forEach(function (eventName) {
            $(".featured video").addEventListener(eventName, renderMediaPreviewButton);
        });
        document.addEventListener("keydown", function (event) { if (event.key === "Escape") { toggleDrawer($("#toolDrawer"), false); toggleDrawer($("#demoConsole"), false); } });
    }

    captureStaticContent();
    bindTransport();
    installEvents();
    installJoystick();
    installEmergencySlider();
    seedLogs();
    updateClock();
    setInterval(updateClock, 30000);
    renderConnection();
    renderTelemetry();
    renderGpsStatus();
    renderGimbalOsd();
    renderCalibrationWizard();
    renderMediaPreviewButton();
    showView("device");

    // Exposed only for automated Preview verification.
    window.MantaPreview = {
        state: state,
        showView: showView,
        applyDemoState: applyDemoState,
        switchTransport: switchTransport,
        applyLanguage: applyLanguage,
        openImuCalibrationWizard: openImuCalibrationWizard
    };
})();
