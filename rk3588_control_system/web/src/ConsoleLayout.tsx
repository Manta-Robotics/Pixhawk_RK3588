// Original console markup, split at existing view boundaries. Do not redesign.
export function ConsoleLayout(){return <>{"\n"}<OrientationGate/>{"\n"}<div {...{"id": "appShell", "data-connection": "idle", "data-layout": "standard"}}>
{"\n"}
<AppHeader/>
{"\n"}
<main {...{"className": "viewport"}}>
{"\n  "}
{"\n  "}
<DeviceView/>
{"\n  "}
{"\n  "}
<MapView/>
{"\n  "}
{"\n  "}
<MediaView/>
{"\n"}
</main>
{"\n"}
{"\n"}
<ToolDrawer/>
{"\n"}
{"\n"}
<DemoConsole/>
{"\n"}
{"\n"}
<div {...{"className": "modal-backdrop", "id": "devicePickerModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal device-picker-modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "devicePickerTitle"}}>
{"\n    "}
<button {...{"className": "modal-close", "type": "button", "data-close-modal": "devicePickerModal", "aria-label": "关闭"}}>
{"×"}
</button>
{"\n    "}
<span {...{"className": "modal-kicker"}}>
{"NEARBY MANTA"}
</span>
{"\n    "}
<h2 {...{"id": "devicePickerTitle"}}>
{"选择要连接的设备"}
</h2>
{"\n    "}
<p {...{"id": "devicePickerSummary"}}>
{"正在扫描附近的 Manta…"}
</p>
{"\n    "}
<div {...{"className": "device-picker-list", "id": "devicePickerList"}}>

</div>
{"\n    "}
<button {...{"className": "device-picker-rescan", "id": "rescanDevices", "type": "button"}}>
{"重新扫描"}
</button>
{"\n  "}
</div>
{"\n"}
</div>
{"\n"}
{"\n"}
<PairingModal/>
{"\n"}
{"\n"}
<ImuCalibrationModal/>
{"\n"}
{"\n"}
<BeaconCalibrationModal/>
{"\n"}
{"\n"}
<DownloadModal/>
{"\n"}
{"\n"}
<DiagnosticModal/>
{"\n"}
{"\n"}
<div {...{"id": "toast", "role": "status", "aria-live": "polite"}}>

</div>
{"\n"}
</div></>;}

export function OrientationGate(){return (<div {...{"className": "orientation-gate", "aria-hidden": "true"}}>
{"\n  "}
<svg {...{"viewBox": "0 0 24 24", "fill": "none", "stroke": "currentColor", "strokeWidth": "1.5"}}>
<path {...{"d": "M4 4h16v16H4z"}}>

</path>
<path {...{"d": "M12 8v8M8 12l4-4 4 4"}}>

</path>
</svg>
{"\n  "}
<strong>
{"请旋转设备"}
</strong>
<p>
{"Manta 仅提供横屏控制体验"}
</p>
{"\n"}
</div>);}

export function AppHeader(){return (<header {...{"className": "app-header"}}>
{"\n  "}
<button {...{"className": "brand", "id": "brandBtn", "type": "button", "data-nav": "device", "aria-label": "返回设备主页"}}>
{"\n    "}
<svg {...{"viewBox": "0 0 132 50", "aria-hidden": "true"}}>
<path {...{"d": "M15 29C27 15 42 8 58 8c13 0 24 6 34 14-18-4-34-2-47 3-11 4-20 6-30 4Z"}}>

</path>
<path {...{"d": "M42 34c19 1 37-4 53-12l25-13c-8 16-20 27-35 33-17 6-32 3-43-8Z"}}>

</path>
</svg>
{"\n    "}
<div>
<span>
{"MANTA"}
</span>
<small>
{"ROBOTICS"}
</small>
</div>
{"\n  "}
</button>
{"\n  "}
<nav {...{"className": "nav-tabs", "id": "primaryNav", "aria-label": "主要页面", "hidden": true}}>
{"\n    "}
<button {...{"className": "nav-tab active", "type": "button", "data-nav": "device"}}>
{"\n      "}
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M4 15a8 8 0 0 1 16 0M7 15a5 5 0 0 1 10 0M12 15v-5M5 19h14"}}>

</path>
</svg>
<span>
{"设备"}
</span>
{"\n    "}
</button>
{"\n    "}
<button {...{"className": "nav-tab", "type": "button", "data-nav": "map"}}>
{"\n      "}
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m3 6 6-3 6 3 6-3v15l-6 3-6-3-6 3zM9 3v15m6-12v15"}}>

</path>
</svg>
<span>
{"地图"}
</span>
{"\n    "}
</button>
{"\n    "}
<button {...{"className": "nav-tab", "type": "button", "data-nav": "media"}}>
{"\n      "}
<svg {...{"viewBox": "0 0 24 24"}}>
<rect {...{"x": "3", "y": "5", "width": "18", "height": "14", "rx": "2"}}>

</rect>
<path {...{"d": "m8 15 3-3 2 2 3-4 3 5M8 9h.01"}}>

</path>
</svg>
<span>
{"录像"}
</span>
{"\n      "}
<span {...{"id": "mediaBadge", "hidden": true, "style": {"fontSize": "9px", "background": "var(--red)", "color": "#fff", "padding": "1px 5px", "borderRadius": "8px", "marginLeft": "4px"}}}>
{"1"}
</span>
{"\n    "}
</button>
{"\n  "}
</nav>
{"\n  "}
<div {...{"className": "status-bar"}}>
{"\n    "}
<div {...{"className": "status-item"}}>
<div {...{"className": "status-dot", "id": "connDot"}}>

</div>
<span>
{"连接"}
</span>
<strong {...{"id": "connectionMetric"}}>
{"未连接"}
</strong>
</div>
{"\n    "}
<div {...{"className": "status-item"}}>
<span>
{"Pixhawk"}
</span>
<strong {...{"id": "pixhawkMetric"}}>
{"OFF"}
</strong>
</div>
{"\n    "}
<div {...{"className": "status-item"}}>
<span>
{"云台"}
</span>
<strong {...{"id": "gimbalMetric"}}>
{"OFF"}
</strong>
</div>
{"\n    "}
<div {...{"className": "status-item"}}>
<span>
{"延迟"}
</span>
<strong {...{"id": "latencyMetric"}}>
{"-- ms"}
</strong>
</div>
{"\n    "}
<div {...{"className": "status-item"}}>
<span>
{"速度"}
</span>
<strong {...{"id": "speedMetric"}}>
{"0.0 m/s"}
</strong>
</div>
{"\n    "}
<div {...{"className": "status-item"}}>
<span>
{"飞控温度"}
</span>
<strong {...{"id": "temperatureMetric"}}>
{"-- °C"}
</strong>
<div {...{"className": "temp-bar", "id": "tempBar"}}>
<b>

</b>
</div>
</div>
{"\n  "}
</div>
{"\n  "}
<div {...{"className": "header-actions"}}>
{"\n    "}
<button {...{"className": "icon-btn", "id": "demoToggle", "type": "button", "title": "演示"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M5 4h14v16H5zM8 8h8M8 12h8M8 16h5"}}>

</path>
</svg>
</button>
{"\n    "}
<button {...{"className": "icon-btn", "id": "layoutToggle", "type": "button", "aria-label": "交换布局", "title": "交换布局"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M4 5h6v6H4zM14 13h6v6h-6M15 5h5v4M9 19H4v-4M18 9l2-2-2-2M6 15l-2 2 2 2"}}>

</path>
</svg>
</button>
{"\n    "}
<button {...{"className": "icon-btn", "id": "toolToggle", "type": "button", "aria-label": "工具", "title": "工具"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<circle {...{"cx": "12", "cy": "12", "r": "3"}}>

</circle>
<path {...{"d": "M12 3v3m0 12v3M3 12h3m12 0h3M5.6 5.6l2.1 2.1m8.6 8.6 2.1 2.1m0-12.8-2.1 2.1m-8.6 8.6-2.1 2.1"}}>

</path>
</svg>
</button>
{"\n  "}
</div>
{"\n"}
</header>);}

export function DeviceView(){return (<section {...{"className": "view active", "id": "deviceView", "data-view": "device"}}>
{"\n    "}
<div {...{"className": "landing-view", "id": "landingScreen"}}>
{"\n      "}
<div {...{"className": "landing-hero"}}>
{"\n        "}
<div {...{"className": "hero-product"}}>
{"\n          "}
<img {...{"src": "assets/manta-app/manta-hero.jpg", "alt": "Manta 水面摄影机器人"}}/>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "hero-tagline"}}>
<h1>
{"Your AI Camera on Water"}
</h1>
<p>
{"自主水面摄影机器人"}
</p>
</div>
{"\n      "}
</div>
{"\n      "}
<div {...{"className": "landing-panel"}}>
{"\n        "}
<div {...{"className": "device-card"}}>
{"\n          "}
<div {...{"className": "device-card-header"}}>
<span>
<i {...{"className": "status-dot", "style": {"width": "6px", "height": "6px"}}}>

</i>
{"附近设备"}
</span>
<small {...{"id": "discoveryStatus"}}>
{"等待扫描"}
</small>
</div>
{"\n          "}
<div {...{"className": "device-name"}}>
{"MANTA ROBOTIC"}
</div>
{"\n          "}
<div {...{"className": "device-desc"}}>
{"面向开放水域游泳与专业野泳运动员的自主水面摄影机器人。"}
</div>
{"\n          "}
<button {...{"className": "connect-btn", "id": "connectButton", "type": "button"}}>
{"\n            "}
<span>
{"搜索 Manta"}
</span>
{"\n            "}
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M5 12h13m-5-5 5 5-5 5"}}>

</path>
</svg>
{"\n          "}
</button>
{"\n          "}
<div {...{"className": "pairing-hint"}}>
{"选择设备后，首次连接需输入六位配对码"}
</div>
{"\n        "}
</div>
{"\n      "}
</div>
{"\n    "}
</div>
{"\n    "}
<div {...{"className": "control-view", "id": "controlScreen", "hidden": true}}>
{"\n      "}
<div {...{"className": "control-toolbar"}}>
{"\n        "}
<div {...{"className": "control-identity"}}>
<i>

</i>
<span>
<b {...{"id": "controlDeviceName"}}>
{"MANTA ROBOTIC"}
</b>
<small {...{"id": "controlDeviceSerial"}}>
{"MANTA · 2407"}
</small>
</span>
</div>
{"\n        "}
<div {...{"className": "control-mode", "role": "group", "aria-label": "船体跟随模式"}}>
{"\n          "}
<button {...{"type": "button", "id": "manualFollowButton", "className": "active"}}>
{"手动跟随"}
</button>
{"\n          "}
<button {...{"type": "button", "id": "autoFollowButton"}}>
{"自动跟随"}
</button>
{"\n        "}
</div>
{"\n        "}
<button {...{"className": "disconnect-btn", "id": "disconnectButton", "type": "button"}}>
{"断开"}
</button>
{"\n      "}
</div>
{"\n      "}
<div {...{"className": "quadrant-grid", "id": "quadrantGrid"}}>
{"\n        "}
<div {...{"className": "quad quad-imu", "data-area": "imu"}}>
{"\n          "}
<div {...{"className": "quad-header"}}>
{"\n            "}
<div {...{"className": "quad-header-left"}}>
<b>
{"飞控姿态"}
</b>
<small>
{"PIXHAWK IMU"}
</small>
</div>
{"\n            "}
<button {...{"className": "imu-calibrate-btn", "id": "imuCalibrate", "type": "button"}}>
{"IMU 校准"}
</button>
{"\n          "}
</div>
{"\n          "}
<div {...{"className": "imu-body"}}>
{"\n            "}
<div {...{"className": "attitude-orb", "id": "attitudeOrb"}}>
{"\n              "}
<div {...{"className": "attitude-sky"}}>

</div>
<div {...{"className": "attitude-water"}}>

</div>
{"\n              "}
<i {...{"className": "horizon-line"}}>

</i>
<i {...{"className": "pitch-line pitch-up"}}>

</i>
<i {...{"className": "pitch-line pitch-down"}}>

</i>
{"\n              "}
<span {...{"className": "craft-mark"}}>
<b>

</b>
<b>

</b>
<i>

</i>
</span>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "imu-readouts"}}>
{"\n              "}
<div {...{"className": "imu-readout"}}>
<span>
{"横滚 ROLL"}
</span>
<strong {...{"id": "rollValue"}}>
{"+1.2°"}
</strong>
</div>
{"\n              "}
<div {...{"className": "imu-readout"}}>
<span>
{"俯仰 PITCH"}
</span>
<strong {...{"id": "pitchValue"}}>
{"−0.6°"}
</strong>
</div>
{"\n              "}
<div {...{"className": "imu-readout"}}>
<span>
{"航向 YAW"}
</span>
<strong {...{"id": "yawValue"}}>
{"184°"}
</strong>
</div>
{"\n              "}
<div {...{"className": "imu-readout success"}}>
<span>
{"校准状态"}
</span>
<strong {...{"id": "imuState"}}>
{"良好"}
</strong>
</div>
{"\n              "}
<div {...{"className": "uwb-panel", "id": "uwbPanel", "data-state": "offline"}}>
{"\n                "}
<div {...{"className": "uwb-dial", "aria-label": "UWB 信标相对方位"}}>
<b>
{"UWB"}
</b>
<i {...{"className": "uwb-dot", "id": "uwbDot"}}>

</i>
</div>
{"\n                "}
<div {...{"className": "uwb-values"}}>
<span>
{"距离"}
</span>
<strong {...{"id": "uwbDistance"}}>
{"-- m"}
</strong>
<span>
{"方位 / 俯仰"}
</span>
<strong {...{"id": "uwbAngles"}}>
{"--° / --°"}
</strong>
</div>
{"\n              "}
</div>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "module-offline", "id": "imuOffline", "hidden": true}}>
<strong>
{"IMU 未连接"}
</strong>
<span>
{"等待 Pixhawk 遥测"}
</span>
</div>
{"\n          "}
</div>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "quad quad-video", "data-area": "video"}}>
{"\n          "}
<div {...{"className": "video-feed"}}>
{"\n            "}
<video {...{"id": "gimbalVideo", "src": "assets/manta-app/gimbal-demo.mp4", "muted": true, "loop": true, "autoPlay": true, "playsInline": true, "preload": "metadata"}}>

</video>
{"\n            "}
<video {...{"id": "liveGimbalWebrtc", "muted": true, "autoPlay": true, "playsInline": true, "disablePictureInPicture": true, "hidden": true}}>

</video>
{"\n            "}
<img {...{"id": "liveGimbalFeed", "alt": "Manta 云台实时画面", "hidden": true}}/>
{"\n            "}
<canvas {...{"id": "gimbalOverlay", "className": "gimbal-vision-overlay", "aria-hidden": "true"}}>

</canvas>
{"\n            "}
<div {...{"className": "video-overlay"}}>
{"\n              "}
<div {...{"className": "video-topline"}}>
{"\n                "}
<span {...{"className": "live-badge is-offline", "id": "videoLinkState", "aria-live": "polite"}}>
<i>

</i>
<b {...{"id": "videoTransportBadge"}}>
{"未连接"}
</b>
</span>
{"\n                "}
<span {...{"className": "video-specs"}}>
{"1080P · 30FPS"}
</span>
{"\n                "}
<span {...{"id": "recordTimer", "className": "record-timer", "hidden": true}}>
<i>

</i>
{"REC 00:00"}
</span>
{"\n              "}
</div>
{"\n              "}
<div {...{"className": "tracking-reticle", "id": "trackingReticle", "hidden": true}}>
<span {...{"id": "trackingLabel", "data-i18n-dynamic": ""}}>
{"SWIMMER"}
</span>
</div>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "video-alert", "id": "videoAlert", "role": "button", "tabIndex": 0, "aria-label": "影像信号中断，点击刷新", "hidden": true}}>
{"\n              "}
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M4 4l16 16M8 5h7v5m1 4v3H5V7"}}>

</path>
</svg>
{"\n              "}
<strong {...{"id": "videoAlertTitle"}}>
{"影像信号中断"}
</strong>
{"\n              "}
<span {...{"id": "videoAlertMessage"}}>
{"点击画面任意位置刷新，控制链路保持在线"}
</span>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "gimbal-fault-overlay", "id": "gimbalFault", "hidden": true}}>
{"\n              "}
<strong>
{"云台保护已触发"}
</strong>
{"\n              "}
<span {...{"id": "gimbalFaultMessage"}}>
{"检测到云台电机异常，所有云台功能已停止"}
</span>
{"\n              "}
<div {...{"className": "fault-actions"}}>
<button {...{"type": "button", "id": "repairGimbal", "className": "repair-btn"}}>
{"云台修复"}
</button>
<button {...{"type": "button", "id": "openReport"}}>
{"查看诊断报告"}
</button>
</div>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "video-toolbar"}}>
{"\n              "}
<div {...{"className": "gimbal-modes"}}>
{"\n                "}
<button {...{"className": "gimbal-mode-btn active", "type": "button", "data-gimbal-mode": "click"}}>
{"点击居中"}
</button>
{"\n                "}
<button {...{"className": "gimbal-mode-btn", "type": "button", "data-gimbal-mode": "face"}}>
{"人脸跟踪"}
</button>
{"\n                "}
<button {...{"className": "gimbal-mode-btn", "type": "button", "data-gimbal-mode": "swimmer"}}>
{"泳者跟踪"}
</button>
{"\n                "}
<button {...{"className": "gimbal-mode-btn", "type": "button", "data-gimbal-mode": "beacon"}}>
{"UWB 信标"}
</button>
{"\n              "}
</div>
{"\n              "}
<div {...{"className": "gimbal-actions"}}>
{"\n                "}
<button {...{"className": "gimbal-action-btn", "type": "button", "data-gimbal-action": "home", "title": "云台回中"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m4 11 8-7 8 7v9h-6v-6h-4v6H4z"}}>

</path>
</svg>
</button>
{"\n                "}
<button {...{"className": "gimbal-action-btn", "type": "button", "data-gimbal-action": "stop", "title": "停止云台"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<rect {...{"x": "6", "y": "6", "width": "12", "height": "12", "rx": "1"}}>

</rect>
</svg>
</button>
{"\n                "}
<button {...{"className": "gimbal-action-btn record-btn", "id": "recordButton", "type": "button", "data-gimbal-action": "record", "title": "录像"}}>
<i>

</i>
</button>
{"\n                "}
<button {...{"className": "gimbal-action-btn osd-btn active", "id": "gimbalOsd", "type": "button", "aria-pressed": "true", "title": "OSD"}}>
{"OSD"}
</button>
{"\n                "}
<button {...{"className": "gimbal-action-btn refresh-stream-btn", "type": "button", "id": "refreshGimbalVideo", "title": "只刷新云台画面", "aria-label": "只刷新云台画面"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "M20 6v5h-5"}}>

</path>
<path {...{"d": "M4 18v-5h5"}}>

</path>
<path {...{"d": "M18.5 9a7 7 0 0 0-11.8-2.4L4 9"}}>

</path>
<path {...{"d": "M5.5 15a7 7 0 0 0 11.8 2.4L20 15"}}>

</path>
</svg>
<span>
{"刷新"}
</span>
</button>
{"\n                "}
<button {...{"className": "gimbal-action-btn beacon-calibration-btn", "type": "button", "id": "gimbalBeaconCalibration", "title": "UWB与云台九点校准", "hidden": true}}>
{"九点校准"}
</button>
{"\n              "}
</div>
{"\n            "}
</div>
{"\n          "}
</div>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "quad quad-drive", "data-area": "drive"}}>
{"\n          "}
<div {...{"className": "quad-header"}}>
{"\n            "}
<div {...{"className": "quad-header-left"}}>
<b>
{"移动控制"}
</b>
<small {...{"id": "driveModeLabel"}}>
{"双桨推进"}
</small>
</div>
{"\n            "}
<span {...{"className": "quad-header-right", "id": "driveOutput"}}>
{"X 0.00 · Y 0.00"}
</span>
{"\n          "}
</div>
{"\n          "}
<div {...{"className": "drive-body"}}>
{"\n            "}
<div {...{"className": "joystick-wrap", "id": "joystick", "role": "slider", "tabIndex": 0, "aria-label": "Manta 360度移动摇杆", "aria-valuetext": "停止"}}>
{"\n              "}
<div {...{"className": "joystick-base"}}>

</div>
<div {...{"className": "joystick-ring"}}>

</div>
{"\n              "}
<div {...{"className": "joystick-stick", "id": "joyStick"}}>

</div>
{"\n              "}
<span {...{"className": "joystick-label fwd"}}>
{"前进"}
</span>
<span {...{"className": "joystick-label rev"}}>
{"后退"}
</span>
{"\n              "}
<span {...{"className": "joystick-label left"}}>
{"左"}
</span>
<span {...{"className": "joystick-label right"}}>
{"右"}
</span>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "drive-follow-control", "id": "driveFollowControl", "hidden": true}}>
{"\n              "}
<button {...{"className": "drive-follow-switch", "type": "button", "id": "driveFollowSwitch"}}>
{"\n                "}
<strong {...{"id": "driveFollowSwitchLabel"}}>
{"启动跟随"}
</strong>
{"\n                "}
<small {...{"id": "driveFollowSwitchStatus"}}>
{"自动选择控制模式并武装"}
</small>
{"\n              "}
</button>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "relative-position-panel", "id": "relativePositionPanel", "data-state": "offline"}}>
{"\n              "}
<div {...{"className": "relative-position-head"}}>
<span {...{"id": "relativePositionTitle"}}>
{"目标相对位置"}
</span>
<b {...{"id": "relativeFusionMode"}}>
{"等待 UWB"}
</b>
</div>
{"\n              "}
<div {...{"className": "relative-position-plot"}}>
{"\n                "}
<svg {...{"viewBox": "0 0 180 112", "role": "img", "aria-label": "UWB目标相对船体坐标"}}>
{"\n                  "}
<g {...{"className": "relative-grid"}}>
{"\n                    "}
<circle {...{"cx": "90", "cy": "56", "r": "17"}}>

</circle>
<circle {...{"cx": "90", "cy": "56", "r": "34"}}>

</circle>
<circle {...{"className": "major", "cx": "90", "cy": "56", "r": "51"}}>

</circle>
{"\n                    "}
<line {...{"className": "major", "x1": "90", "y1": "3", "x2": "90", "y2": "109"}}>

</line>
<line {...{"className": "major", "x1": "37", "y1": "56", "x2": "143", "y2": "56"}}>

</line>
{"\n                    "}
<line {...{"x1": "54", "y1": "20", "x2": "126", "y2": "92"}}>

</line>
<line {...{"x1": "126", "y1": "20", "x2": "54", "y2": "92"}}>

</line>
{"\n                    "}
<text {...{"id": "relativeRingLabel", "x": "94", "y": "9"}}>
{"5 m"}
</text>
{"\n                  "}
</g>
{"\n                  "}
<text {...{"className": "relative-axis-label", "x": "90", "y": "7", "textAnchor": "middle"}}>
{"前 +X"}
</text>
<text {...{"className": "relative-axis-label", "x": "34", "y": "59", "textAnchor": "end"}}>
{"左 +Y"}
</text>
{"\n                  "}
<line {...{"className": "relative-raw-vector", "id": "relativeRawVector", "visibility": "hidden", "x1": "90", "y1": "56", "x2": "90", "y2": "56"}}>

</line>
{"\n                  "}
<circle {...{"className": "relative-raw-target", "id": "relativeRawPoint", "visibility": "hidden", "cx": "90", "cy": "56", "r": "5.5"}}>

</circle>
{"\n                  "}
<line {...{"visibility": "hidden", "className": "relative-vector", "id": "relativeTargetVector", "x1": "90", "y1": "56", "x2": "90", "y2": "56"}}>

</line>
{"\n                  "}
<circle {...{"visibility": "hidden", "className": "relative-target", "id": "relativeTargetPoint", "cx": "90", "cy": "56", "r": "4"}}>

</circle>
{"\n                  "}
<path {...{"className": "relative-boat", "d": "M90 48 L95 61 L90 59 L85 61 Z"}}>

</path>
{"\n                "}
</svg>
{"\n              "}
</div>
{"\n              "}
<div {...{"className": "relative-sample-row", "data-kind": "raw"}}>
<strong {...{"id": "relativeRawLabel"}}>
{"○ 原始"}
</strong>
<span {...{"id": "relativeRawValue"}}>
{"-- m / --°"}
</span>
</div>
{"\n              "}
<div {...{"className": "relative-sample-row", "data-kind": "trusted"}}>
<strong {...{"id": "relativeJudgedLabel"}}>
{"● 可信"}
</strong>
<span {...{"id": "relativeRangeValue"}}>
{"-- m / --°"}
</span>
</div>
{"\n              "}
<div {...{"className": "relative-position-foot"}}>
<strong {...{"id": "relativePositionValue"}}>
{"前 -- · 左 --"}
</strong>
</div>
{"\n              "}
<div {...{"className": "relative-decision", "id": "relativeDecision"}}>
{"等待定位"}
</div>
{"\n            "}
</div>
{"\n            "}
<div {...{"className": "drive-side"}}>
{"\n              "}
<div {...{"className": "tracking-method-control", "id": "trackingMethodControl", "hidden": true}}>
{"\n                "}
<div {...{"className": "tracking-method-head"}}>
<span>
{"跟踪控制方法"}
</span>
<b {...{"id": "trackingMethodState"}}>
{"TRADITIONAL"}
</b>
</div>
{"\n                "}
<div {...{"className": "tracking-method-switch", "role": "radiogroup", "aria-label": "跟踪控制方法"}}>
{"\n                  "}
<button {...{"className": "tracking-method-btn active", "type": "button", "data-tracking-method": "traditional", "role": "radio", "aria-checked": "true"}}>
<strong>
{"传统控制"}
</strong>
<small>
{"TRADITIONAL"}
</small>
</button>
{"\n                  "}
<button {...{"className": "tracking-method-btn", "type": "button", "data-tracking-method": "neural", "role": "radio", "aria-checked": "false"}}>
<strong>
{"神经网络"}
</strong>
<small>
{"SMARTUSV PPO"}
</small>
</button>
{"\n                "}
</div>
{"\n              "}
</div>
{"\n              "}
<div {...{"className": "drive-speed-readout"}}>
<span>
{"当前速度"}
</span>
<strong {...{"id": "driveSpeedValue"}}>
{"0.0"}
<small>
{"m/s"}
</small>
</strong>
</div>
{"\n              "}
<div {...{"className": "motor-status-row"}}>
{"\n                "}
<div {...{"className": "motor-status-card", "id": "leftMotorCard"}}>
<header>
<i {...{"className": "motor-link-dot"}}>

</i>
<span {...{"id": "leftMotorLabel"}}>
{"左电机"}
</span>
</header>
<strong {...{"id": "leftMotorPwm"}}>
{"-- μs"}
</strong>
<small>
{"输出 PWM · "}
<b {...{"id": "leftMotor"}}>
{"--"}
</b>
</small>
</div>
{"\n                "}
<div {...{"className": "motor-status-card", "id": "rightMotorCard"}}>
<header>
<i {...{"className": "motor-link-dot"}}>

</i>
<span {...{"id": "rightMotorLabel"}}>
{"右电机"}
</span>
</header>
<strong {...{"id": "rightMotorPwm"}}>
{"-- μs"}
</strong>
<small>
{"输出 PWM · "}
<b {...{"id": "rightMotor"}}>
{"--"}
</b>
</small>
</div>
{"\n              "}
</div>
{"\n              "}
<div {...{"className": "arm-control"}}>
{"\n                "}
<div {...{"className": "arm-control-head"}}>
<span>
{"推进状态"}
</span>
<strong {...{"id": "armStateLabel"}}>
{"已解除武装"}
</strong>
</div>
{"\n                "}
<div {...{"className": "arm-slider", "id": "armSlider", "role": "slider", "tabIndex": 0, "aria-label": "滑动武装推进电机", "aria-valuemin": 0, "aria-valuemax": 100, "aria-valuenow": 0}}>
{"\n                  "}
<span {...{"className": "arm-slider-fill"}}>

</span>
<span {...{"className": "arm-slider-label", "id": "armSwitchText"}}>
{"滑动武装"}
</span>
<span {...{"className": "arm-slider-thumb"}}>
{"››"}
</span>
{"\n                "}
</div>
{"\n                "}
<button {...{"className": "disarm-button", "id": "disarmButton", "type": "button", "hidden": true}}>
{"解除武装"}
</button>
{"\n              "}
</div>
{"\n            "}
</div>
{"\n          "}
</div>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "quad quad-logs", "data-area": "logs"}}>
{"\n          "}
<div {...{"className": "quad-header"}}>
{"\n            "}
<div {...{"className": "quad-header-left"}}>
<b>
{"输出日志"}
</b>
<small>
{"LIVE OUTPUT"}
</small>
</div>
{"\n            "}
<div {...{"className": "log-filters"}}>
{"\n              "}
<button {...{"className": "log-filter-btn active", "type": "button", "data-log-filter": "all"}}>
{"全部"}
</button>
{"\n              "}
<button {...{"className": "log-filter-btn", "type": "button", "data-log-filter": "command"}}>
{"指令"}
</button>
{"\n              "}
<button {...{"className": "log-filter-btn", "type": "button", "data-log-filter": "hardware"}}>
{"硬件"}
</button>
{"\n            "}
</div>
{"\n          "}
</div>
{"\n          "}
<div {...{"className": "log-body", "id": "logStream", "aria-live": "polite"}}>

</div>
{"\n          "}
<div {...{"className": "log-footer"}}>
{"\n            "}
<span {...{"id": "hostLinkLight", "data-state": "offline"}}>
<i>

</i>
{"上位机"}
</span>
<span {...{"id": "pixhawkLinkLight", "data-state": "offline"}}>
<i>

</i>
{"下位机"}
</span>
<span {...{"id": "motorLinkLight", "data-state": "offline"}}>
<i>

</i>
{"电机"}
</span>
<span {...{"id": "gimbalLinkLight", "data-state": "offline"}}>
<i>

</i>
{"云台"}
</span>
{"\n          "}
</div>
{"\n        "}
</div>
{"\n      "}
</div>
{"\n    "}
</div>
{"\n  "}
</section>);}

export function MapView(){return (<section {...{"className": "view map-view", "id": "mapView", "data-view": "map", "hidden": true}}>
{"\n    "}
<div {...{"className": "map-canvas", "id": "mapCanvas", "data-provider": "offline-satellite"}}>
{"\n      "}
<canvas {...{"className": "offline-satellite-map", "id": "offlineSatelliteMap", "aria-label": "中国板载离线卫星地图"}}>

</canvas>
{"\n      "}
<div {...{"className": "offline-map-controls", "aria-label": "地图控制"}}>
{"\n        "}
<button {...{"type": "button", "id": "offlineMapZoomOut", "aria-label": "缩小地图"}}>
{"−"}
</button>
{"\n        "}
<button {...{"type": "button", "id": "offlineMapLocate", "aria-label": "定位 MANTA"}}>
{"定位"}
</button>
{"\n        "}
<button {...{"type": "button", "id": "offlineMapZoomIn", "aria-label": "放大地图"}}>
{"+"}
</button>
{"\n      "}
</div>
{"\n      "}
<div {...{"className": "map-info-card"}}>
{"\n        "}
<span>
{"飞控 GPS 坐标"}
</span>
<strong {...{"id": "mapCoordinates"}}>
{"--, --"}
</strong>
<small {...{"id": "mapHeading"}}>
{"航向 --°"}
</small>
{"\n      "}
</div>
{"\n      "}
<div {...{"className": "gps-status-card", "id": "gpsSourceCard", "data-state": "waiting"}}>
{"\n        "}
<span>
<i>

</i>
{"GPS 状态"}
</span>
{"\n        "}
<strong {...{"id": "gpsStatus"}}>
{"等待设备连接"}
</strong>
{"\n        "}
<small {...{"id": "gpsDetail"}}>
{"0 颗卫星"}
</small>
{"\n        "}
<small {...{"id": "mapSource"}}>
{"地图来源：板载深圳离线卫星影像"}
</small>
{"\n      "}
</div>
{"\n    "}
</div>
{"\n  "}
</section>);}

export function MediaView(){return (<section {...{"className": "view media-view", "id": "mediaView", "data-view": "media", "hidden": true}}>
{"\n    "}
<div {...{"className": "media-toolbar"}}>
{"\n      "}
<div {...{"className": "media-segments"}}>
{"\n        "}
<button {...{"className": "media-segment active", "type": "button", "data-media-filter": "all"}}>
{"全部"}
</button>
{"\n        "}
<button {...{"className": "media-segment", "type": "button", "data-media-filter": "today"}}>
{"今天"}
</button>
{"\n        "}
<button {...{"className": "media-segment", "type": "button", "data-media-filter": "downloaded"}}>
{"已下载"}
</button>
{"\n      "}
</div>
{"\n      "}
<span {...{"className": "media-count"}}>
{"3 个项目"}
</span>
{"\n    "}
</div>
{"\n    "}
<div {...{"className": "media-grid"}}>
{"\n      "}
<article {...{"className": "media-card featured", "data-recording": "20260709_161929_h264.mp4", "data-media-url": "assets/manta-app/gimbal-demo.mp4", "data-today": "true", "data-downloaded": "false"}}>
{"\n        "}
<div {...{"className": "media-thumb"}}>
{"\n          "}
<video {...{"src": "assets/manta-app/gimbal-demo.mp4", "muted": true, "playsInline": true, "preload": "metadata"}}>

</video>
{"\n          "}
<span {...{"className": "media-duration"}}>
{"02:47"}
</span>
<span {...{"className": "media-quality"}}>
{"1080P"}
</span>
{"\n          "}
<button {...{"className": "media-play-btn", "type": "button", "aria-label": "播放录像"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m9 7 8 5-8 5z"}}>

</path>
</svg>
</button>
{"\n          "}
<div {...{"className": "media-player-bar"}}>
{"\n            "}
<button {...{"type": "button", "id": "mediaPlayPause", "aria-label": "播放录像", "data-state": "paused"}}>
{"\n              "}
<svg {...{"viewBox": "0 0 24 24", "className": "media-icon-play"}}>
<path {...{"d": "m9 7 8 5-8 5z"}}>

</path>
</svg>
{"\n              "}
<svg {...{"viewBox": "0 0 24 24", "className": "media-icon-pause", "hidden": true}}>
<path {...{"d": "M8 6v12M16 6v12"}}>

</path>
</svg>
{"\n            "}
</button>
{"\n            "}
<time {...{"id": "mediaCurrentTime"}}>
{"00:00"}
</time>
{"\n            "}
<input {...{"className": "media-scrubber", "id": "mediaScrubber", "type": "range", "min": "0", "max": "0", "step": "0.1", "defaultValue": "0", "aria-label": "录像时间轴"}}/>
{"\n            "}
<time {...{"id": "mediaDuration"}}>
{"00:00"}
</time>
{"\n            "}
<select {...{"className": "media-rate-select", "id": "mediaPlaybackRate", "aria-label": "播放速度", "defaultValue": "1"}}>
<option {...{"value": "0.5"}}>
{"0.5×"}
</option>
<option {...{"value": "1"}}>
{"1×"}
</option>
<option {...{"value": "1.5"}}>
{"1.5×"}
</option>
<option {...{"value": "2"}}>
{"2×"}
</option>
</select>
{"\n          "}
</div>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "media-info"}}>
{"\n          "}
<div {...{"className": "media-info-left"}}>
<span>
<strong>
{"开放水域跟拍 01"}
</strong>
<small>
{"2026.07.09 · 16:19"}
</small>
</span>
</div>
{"\n          "}
<div {...{"className": "media-info-right"}}>
<b>
{"19.4 MB"}
</b>
<button {...{"className": "media-download-btn", "type": "button"}}>
{"下载"}
</button>
</div>
{"\n        "}
</div>
{"\n      "}
</article>
{"\n      "}
<article {...{"className": "media-card", "data-media-url": "assets/manta-app/gimbal-demo.mp4", "data-today": "true", "data-downloaded": "false"}}>
{"\n        "}
<div {...{"className": "media-thumb", "style": {"background": "linear-gradient(135deg,#1a2838,#0d1520)"}}}>
{"\n          "}
<span {...{"className": "media-duration"}}>
{"01:36"}
</span>
<span {...{"className": "media-quality"}}>
{"1080P"}
</span>
{"\n          "}
<button {...{"className": "media-play-btn", "type": "button", "aria-label": "播放录像"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m9 7 8 5-8 5z"}}>

</path>
</svg>
</button>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "media-info"}}>
{"\n          "}
<div {...{"className": "media-info-left"}}>
<span>
<strong>
{"泳者跟踪测试"}
</strong>
<small>
{"2026.07.09 · 15:48"}
</small>
</span>
</div>
{"\n          "}
<div {...{"className": "media-info-right"}}>
<b>
{"128 MB"}
</b>
<button {...{"className": "media-download-btn", "type": "button"}}>
{"下载"}
</button>
</div>
{"\n        "}
</div>
{"\n      "}
</article>
{"\n      "}
<article {...{"className": "media-card", "data-media-url": "assets/manta-app/gimbal-demo.mp4", "data-today": "false", "data-downloaded": "false"}}>
{"\n        "}
<div {...{"className": "media-thumb", "style": {"background": "linear-gradient(135deg,#152030,#0a1018)"}}}>
{"\n          "}
<span {...{"className": "media-duration"}}>
{"00:54"}
</span>
<span {...{"className": "media-quality"}}>
{"1080P"}
</span>
{"\n          "}
<button {...{"className": "media-play-btn", "type": "button", "aria-label": "播放录像"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m9 7 8 5-8 5z"}}>

</path>
</svg>
</button>
{"\n        "}
</div>
{"\n        "}
<div {...{"className": "media-info"}}>
{"\n          "}
<div {...{"className": "media-info-left"}}>
<span>
<strong>
{"返航影像"}
</strong>
<small>
{"2026.07.09 · 15:21"}
</small>
</span>
</div>
{"\n          "}
<div {...{"className": "media-info-right"}}>
<b>
{"76 MB"}
</b>
<button {...{"className": "media-download-btn", "type": "button"}}>
{"下载"}
</button>
</div>
{"\n        "}
</div>
{"\n      "}
</article>
{"\n    "}
</div>
{"\n  "}
</section>);}

export function ToolDrawer(){return (<aside {...{"className": "drawer", "id": "toolDrawer", "aria-hidden": "true"}}>
{"\n  "}
<div {...{"className": "drawer-head"}}>
{"\n    "}
<span>
<small>
{"QUICK TOOLS"}
</small>
<strong>
{"快捷工具"}
</strong>
</span>
{"\n    "}
<button {...{"className": "icon-btn", "id": "toolClose", "type": "button", "aria-label": "关闭"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m6 6 12 12M18 6 6 18"}}>

</path>
</svg>
</button>
{"\n  "}
</div>
{"\n  "}
<div {...{"className": "drawer-section"}}>
{"\n    "}
<h3>
{"显示"}
</h3>
{"\n    "}
<label {...{"className": "setting-row"}}>
<span>
<b>
{"外观"}
</b>
<small>
{"跟随使用环境"}
</small>
</span>
<select {...{"id": "themeSelect"}}>
<option {...{"value": "dark"}}>
{"深色"}
</option>
<option {...{"value": "light"}}>
{"浅色"}
</option>
</select>
</label>
{"\n    "}
<label {...{"className": "setting-row"}}>
<span>
<b>
{"语言"}
</b>
<small>
{"Language"}
</small>
</span>
<select {...{"id": "languageSelect"}}>
<option {...{"value": "zh"}}>
{"简体中文"}
</option>
<option {...{"value": "en"}}>
{"English"}
</option>
</select>
</label>
{"\n  "}
</div>
{"\n  "}
<div {...{"className": "drawer-section"}}>
{"\n    "}
<h3>
{"连接"}
</h3>
{"\n    "}
<div {...{"className": "setting-row"}}>
<span>
<b>
{"通信模式"}
</b>
<small {...{"id": "transportHint"}}>
{"Preview 模拟通信"}
</small>
</span>
<button {...{"className": "status-pill", "id": "transportToggle", "type": "button"}}>
{"SIM"}
</button>
</div>
{"\n    "}
<div {...{"className": "setting-row"}}>
<span>
<b>
{"互联网"}
</b>
<small>
{"连接 Manta 时保持在线"}
</small>
</span>
<span {...{"style": {"color": "var(--green)", "fontSize": "12px"}}}>
{"已开启"}
</span>
</div>
{"\n  "}
</div>
{"\n  "}
<div {...{"className": "drawer-section"}}>
{"\n    "}
<button {...{"className": "drawer-link", "type": "button", "data-drawer-page": "logs"}}>
<span>
<b>
{"完整日志"}
</b>
<small>
{"导出诊断信息"}
</small>
</span>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m9 5 7 7-7 7"}}>

</path>
</svg>
</button>
{"\n    "}
<button {...{"className": "drawer-link", "type": "button", "data-drawer-page": "help"}}>
<span>
<b>
{"帮助与安全"}
</b>
<small>
{"连接、控制与急停说明"}
</small>
</span>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m9 5 7 7-7 7"}}>

</path>
</svg>
</button>
{"\n  "}
</div>
{"\n"}
</aside>);}

export function DemoConsole(){return (<aside {...{"className": "drawer demo-console", "id": "demoConsole", "aria-hidden": "true"}}>
{"\n  "}
<div {...{"className": "drawer-head"}}>
{"\n    "}
<span>
<small>
{"PREVIEW LAB"}
</small>
<strong>
{"状态模拟器"}
</strong>
</span>
{"\n    "}
<button {...{"className": "icon-btn", "id": "demoClose", "type": "button"}}>
<svg {...{"viewBox": "0 0 24 24"}}>
<path {...{"d": "m6 6 12 12M18 6 6 18"}}>

</path>
</svg>
</button>
{"\n  "}
</div>
{"\n  "}
<p>
{"用于在电脑端预览所有真实状态，不会向 Manta 发送指令。"}
</p>
{"\n  "}
<div {...{"className": "demo-states"}}>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "connected"}}>
{"正常连接"}
</button>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "videoLost"}}>
{"视频中断"}
</button>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "gimbalFault"}}>
{"云台故障"}
</button>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "hot"}}>
{"温度告警"}
</button>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "emergency"}}>
{"急停"}
</button>
{"\n    "}
<button {...{"className": "demo-state-btn", "type": "button", "data-demo-state": "offline"}}>
{"设备离线"}
</button>
{"\n  "}
</div>
{"\n  "}
<div {...{"className": "demo-slider"}}>
<label>
{"延迟 "}
<output {...{"id": "demoLatencyOutput"}}>
{"18 ms"}
</output>
</label>
<input {...{"id": "demoLatency", "type": "range", "min": "4", "max": "350", "defaultValue": "18"}}/>
</div>
{"\n  "}
<div {...{"className": "demo-slider"}}>
<label>
{"速度 "}
<output {...{"id": "demoSpeedOutput"}}>
{"1.4 m/s"}
</output>
</label>
<input {...{"id": "demoSpeed", "type": "range", "min": "0", "max": "35", "defaultValue": "14"}}/>
</div>
{"\n  "}
<div {...{"className": "demo-slider"}}>
<label>
{"飞控温度 "}
<output {...{"id": "demoTempOutput"}}>
{"46 °C"}
</output>
</label>
<input {...{"id": "demoTemp", "type": "range", "min": "20", "max": "95", "defaultValue": "46"}}/>
</div>
{"\n"}
</aside>);}

export function PairingModal(){return (<div {...{"className": "modal-backdrop", "id": "pairingModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "pairingTitle"}}>
{"\n    "}
<button {...{"className": "modal-close", "type": "button", "data-close-modal": "pairingModal", "aria-label": "关闭"}}>
{"×"}
</button>
{"\n    "}
<span {...{"className": "modal-kicker"}}>
{"SECURE PAIRING"}
</span>
{"\n    "}
<h2 {...{"id": "pairingTitle"}}>
{"输入 Manta 显示的配对码"}
</h2>
{"\n    "}
<p {...{"id": "pairingDeviceIdentity"}}>
{"MANTA ROBOTIC · MANTA 2407"}
</p>
{"\n    "}
<div {...{"className": "pin-inputs", "id": "pinInputs"}}>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第1位"}}/>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第2位"}}/>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第3位"}}/>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第4位"}}/>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第5位"}}/>
{"\n      "}
<input {...{"className": "pin-input", "inputMode": "numeric", "maxLength": 1, "aria-label": "第6位"}}/>
{"\n    "}
</div>
{"\n    "}
<small {...{"id": "pairingHint", "style": {"display": "block", "textAlign": "center", "color": "var(--t3)", "marginBottom": "16px"}}}>
{"Preview 测试码："}
<b>
{"240724"}
</b>
</small>
{"\n    "}
<button {...{"className": "modal-action", "id": "pairButton", "type": "button", "disabled": true}}>
{"验证并连接"}
</button>
{"\n  "}
</div>
{"\n"}
</div>);}

export function ImuCalibrationModal(){return (<div {...{"className": "modal-backdrop", "id": "imuCalibrationModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "imuCalibrationTitle"}}>
{"\n    "}
<button {...{"className": "modal-close", "id": "imuCalibrationClose", "type": "button", "aria-label": "关闭"}}>
{"×"}
</button>
{"\n    "}
<span {...{"className": "modal-kicker"}}>
{"PIXHAWK · SIX-POSITION"}
</span>
{"\n    "}
<h2 {...{"id": "imuCalibrationTitle"}}>
{"IMU 六面校准"}
</h2>
{"\n    "}
<p {...{"id": "imuCalibrationLead"}}>
{"校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。"}
</p>
{"\n    "}
<div {...{"className": "cal-progress"}}>
{"\n      "}
<span {...{"id": "imuCalibrationProgressText"}}>
{"准备开始"}
</span>
{"\n      "}
<strong {...{"id": "imuCalibrationCounter"}}>
{"0 / 6"}
</strong>
{"\n    "}
</div>
{"\n    "}
<div {...{"className": "cal-progress-bar"}}>
<b {...{"id": "imuCalibrationBar"}}>

</b>
</div>
{"\n    "}
<div {...{"className": "cal-pose-stage", "id": "imuPoseStage", "data-pose": "intro"}}>
{"\n      "}
<div {...{"className": "cal-pose-device"}}>
{"MANTA"}
</div>
{"\n    "}
</div>
{"\n    "}
<strong {...{"className": "cal-pose-title", "id": "imuPoseTitle"}}>
{"准备六个放置方向"}
</strong>
{"\n    "}
<p {...{"className": "cal-pose-instruction", "id": "imuPoseInstruction"}}>
{"开始后，请按照飞控要求逐面放稳并确认。"}
</p>
{"\n    "}
<div {...{"className": "cal-actions"}}>
{"\n      "}
<button {...{"className": "secondary", "id": "imuCalibrationCancel", "type": "button"}}>
{"取消"}
</button>
{"\n      "}
<button {...{"className": "primary", "id": "imuCalibrationNext", "type": "button"}}>
{"开始校准"}
</button>
{"\n    "}
</div>
{"\n    "}
<button {...{"className": "cal-emergency", "id": "imuCalibrationEmergency", "type": "button", "hidden": true}}>
{"紧急停止推进"}
</button>
{"\n  "}
</div>
{"\n"}
</div>);}

export function BeaconCalibrationModal(){return (<div {...{"className": "modal-backdrop", "id": "beaconCalibrationModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "beaconCalibrationTitle"}}>
{"\n    "}
<button {...{"className": "modal-close", "id": "beaconCalibrationClose", "type": "button", "aria-label": "关闭"}}>
{"×"}
</button>
{"\n    "}
<span {...{"className": "modal-kicker"}}>
{"UWB BEACON · 9-POINT"}
</span>
{"\n    "}
<h2 {...{"id": "beaconCalibrationTitle"}}>
{"UWB + 云台九点校准"}
</h2>
{"\n    "}
<p {...{"id": "beaconCalibrationLead"}}>
{"云台会以随机顺序移动到左/中/右 × 上/中/下九个点位。保持信标天线朝向不变，并在正常使用距离将它放到画面正中心；系统会同时记录距离和角度，拟合三维相对位置。"}
</p>
{"\n    "}
<div {...{"className": "cal-progress"}}>
{"\n      "}
<span {...{"id": "beaconCalibrationProgressText"}}>
{"准备开始"}
</span>
{"\n      "}
<strong {...{"id": "beaconCalibrationCounter"}}>
{"0 / 9"}
</strong>
{"\n    "}
</div>
{"\n    "}
<div {...{"className": "cal-progress-bar"}}>
<b {...{"id": "beaconCalibrationBar"}}>

</b>
</div>
{"\n    "}
<div {...{"className": "beacon-calibration-preview", "id": "beaconCalibrationPreview"}}>
{"\n      "}
<canvas {...{"id": "beaconCalibrationCanvas", "aria-label": "云台实时校准画面"}}>

</canvas>
{"\n      "}
<i {...{"className": "beacon-calibration-reticle", "aria-hidden": "true"}}>

</i>
{"\n    "}
</div>
{"\n    "}
<strong {...{"className": "cal-pose-title", "id": "beaconCalibrationPoseTitle"}}>
{"准备九个 3×3 点位"}
</strong>
{"\n    "}
<p {...{"className": "cal-pose-instruction", "id": "beaconCalibrationInstruction"}}>
{"开始后，Beacon跟随会暂停；请逐点将信标放到画面中心再记录。"}
</p>
{"\n    "}
<div {...{"className": "cal-actions"}}>
{"\n      "}
<button {...{"className": "secondary", "id": "beaconCalibrationCancel", "type": "button"}}>
{"取消"}
</button>
{"\n      "}
<button {...{"className": "primary", "id": "beaconCalibrationNext", "type": "button"}}>
{"开始九点校准"}
</button>
{"\n    "}
</div>
{"\n  "}
</div>
{"\n"}
</div>);}

export function DownloadModal(){return (<div {...{"className": "modal-backdrop", "id": "downloadModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "downloadTitle"}}>
{"\n    "}
<span {...{"className": "modal-kicker"}}>
{"DOWNLOADING"}
</span>
{"\n    "}
<h2 {...{"id": "downloadTitle"}}>
{"正在下载到 Manta App"}
</h2>
{"\n    "}
<p {...{"id": "downloadFilename"}}>
{"20260709_161929_h264.mp4"}
</p>
{"\n    "}
<div {...{"className": "download-progress"}}>
{"\n      "}
<div {...{"className": "download-progress-track"}}>
<div {...{"className": "download-progress-fill", "id": "downloadBar"}}>

</div>
</div>
{"\n      "}
<div {...{"className": "download-progress-info"}}>
<strong {...{"id": "downloadPercent"}}>
{"0%"}
</strong>
<small {...{"id": "downloadSize"}}>
{"0 MB / 19.4 MB"}
</small>
</div>
{"\n    "}
</div>
{"\n    "}
<p {...{"className": "download-lock-note"}}>
{"下载完成前请保持 App 在前台并维持与 Manta 的连接。"}
</p>
{"\n    "}
<button {...{"className": "modal-action", "id": "cancelDownload", "type": "button", "disabled": true}}>
{"下载期间无法切换页面"}
</button>
{"\n  "}
</div>
{"\n"}
</div>);}

export function DiagnosticModal(){return (<div {...{"className": "modal-backdrop", "id": "diagnosticModal", "hidden": true}}>
{"\n  "}
<div {...{"className": "modal", "role": "dialog", "aria-modal": "true", "aria-labelledby": "diagnosticTitle"}}>
{"\n    "}
<button {...{"className": "modal-close", "type": "button", "data-close-modal": "diagnosticModal"}}>
{"×"}
</button>
{"\n    "}
<span {...{"className": "modal-kicker danger"}}>
{"GIMBAL PROTECTION"}
</span>
{"\n    "}
<h2 {...{"id": "diagnosticTitle"}}>
{"云台健康诊断报告"}
</h2>
{"\n    "}
<div {...{"className": "report-summary"}}>
{"\n      "}
<i>

</i>
{"\n      "}
<span>
<strong {...{"id": "diagnosticCode"}}>
{"GMB-MOTION-STALL"}
</strong>
<small>
{"已执行云台停止保护"}
</small>
</span>
{"\n    "}
</div>
{"\n    "}
<div {...{"className": "diagnostic-dl"}}>
{"\n      "}
<div>
<dt>
{"指令"}
</dt>
<dd {...{"id": "diagnosticCommand"}}>
{"SWIMMER TRACK / PAN +18°"}
</dd>
</div>
{"\n      "}
<div>
<dt>
{"实际响应"}
</dt>
<dd {...{"id": "diagnosticResponse"}}>
{"1.8 s 内角度变化 < 0.8°"}
</dd>
</div>
{"\n      "}
<div>
<dt>
{"连续异常"}
</dt>
<dd {...{"id": "diagnosticWindows"}}>
{"3 / 3 个检测窗口"}
</dd>
</div>
{"\n      "}
<div>
<dt>
{"通信"}
</dt>
<dd {...{"id": "diagnosticLink"}}>
{"串口正常 · 状态包在线"}
</dd>
</div>
{"\n      "}
<div>
<dt>
{"处置"}
</dt>
<dd {...{"id": "diagnosticAction"}}>
{"停止跟踪、停止云台电机、保留主机移动控制"}
</dd>
</div>
{"\n    "}
</div>
{"\n    "}
<p {...{"style": {"fontSize": "12px", "color": "var(--t3)", "lineHeight": "1.5", "marginBottom": "16px"}}}>
{"当前结论是\"疑似卡死\"，不是硬件确定性判定。建议断电检查机械阻力、供电和串口错误，然后重新执行云台自检。"}
</p>
{"\n    "}
<div {...{"className": "cal-actions"}}>
<button {...{"className": "secondary", "type": "button", "data-close-modal": "diagnosticModal"}}>
{"关闭"}
</button>
<button {...{"className": "primary", "id": "repairGimbalFromReport", "type": "button"}}>
{"云台修复"}
</button>
</div>
{"\n  "}
</div>
{"\n"}
</div>);}
