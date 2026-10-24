import {createConsoleScope} from './lifecycle';

// Behavior-preserving port of the original console controller. React owns
// the view tree; this scoped controller retains the established interactions.
export function mountConsole(): () => void {
const scope=createConsoleScope();
const window=scope.window, document=scope.document;
const setTimeout=scope.setTimeout, clearTimeout=scope.clearTimeout;
const setInterval=scope.setInterval, clearInterval=scope.clearInterval;
const requestAnimationFrame=scope.requestAnimationFrame, cancelAnimationFrame=scope.cancelAnimationFrame;

"use strict";
var $: any=function(this: any,s?: any,r?: any): any {return(r||document).querySelector(s)};
var $$: any=function(this: any,s?: any,r?: any): any {return Array.prototype.slice.call((r||document).querySelectorAll(s))};
var shell: any=$("#appShell");
var isLocalPreview: any=location.protocol==="file:"||/^(4173|4175)$/.test(location.port);
var state: any={view:"device",connected:false,boardOnline:false,pixhawkOnline:false,imuOnline:false,motorsOnline:false,motorLinks:{left:{online:false,outputOnline:false,status:"offline"},right:{online:false,outputOnline:false,status:"offline"}},gimbalOnline:false,gimbalOutputAvailable:false,gimbalLinkStatus:"offline",controlLink:false,connection:"idle",transportMode:isLocalPreview?"mock":"live",layout:"standard",language:"zh",theme:"dark",driveMode:"manual",armed:false,flightMode:"MANUAL",latency:18,speed:0,temperature:46,attitude:{roll:1.2,pitch:-0.6,yaw:184},uwb:{online:false,fresh:false,distanceM:null,azimuthDeg:null,elevationDeg:null},ekf:{healthy:false,updatedAt:null},uwbFollow:{active:false,status:"idle",reason:"not_started",trackingMethod:"traditional",desiredSpeedMps:0},vector:{x:0,y:0},recording:false,recordingSince:0,videoLost:false,gimbalFault:false,emergency:false,downloading:false,logFilter:"all",osdEnabled:true,gimbalMode:"click",trackingActive:false,trackMode:"face",runningTrackMode:"face",trackStatus:{locked:false,status:"idle",message:"idle"},trackTarget:null,clickMarker:null,gps:{satellites:0,hdop:null,fixType:0,estimated:true},discoveredDevices:[],selectedDevice:null,calibrating:false,calibrationStarted:false,calibrationStepCode:null,calibrationStatus:"intro",calibrationConfirmed:0,calibrationProgress:0,calibrationResult:null,calibrationResultMessage:""};
var logEntries: any=[];
var seenAlertLogKeys: any={};
var toastTimer: any,recordTimerHandle: any,downloadTimer: any,telemetryTimer: any,mediaSeeking: any=false,mediaFilter: any="all";
var transport: any=null,liveRecordings: any=[],gimbalModeBusy: any=false,beaconCalibration: any=null,beaconCalibrationBusy: any=false,gimbalWebrtcReader: any=null,gimbalWebrtcFallbackTimer: any=null,gimbalMjpegRetryTimer: any=null,gimbalWebrtcFallbackLogged: any=false,gimbalStreamGeneration: any=0,gimbalVideoStatus: any="offline",gimbalVideoTransport: any="";
var offlineMap: any=null,mapInitStarted: any=false,mapProvider: any="offline-satellite",mapSourceLabel: any="板载深圳离线卫星图 · GPS：Pixhawk",mapSourceLabelEn: any="Offline Shenzhen satellite · GPS: Pixhawk",lastGpsSourceTimestamp: any=0,lastGpsReceivedAt: any=0;

function ensureTransport(this: any): any {
  if(transport&&transport.mode===state.transportMode)return transport;
  if(transport)transport.disconnect().catch(function(this: any): any {});
  if(!window.MantaAppTransport)return null;
  transport=window.MantaAppTransport.create(state.transportMode);
  transport.on("telemetry",function(this: any,payload?: any): any {
    payload=payload||{};
    if(Number.isFinite(Number(payload.latency)))state.latency=Number(payload.latency);
    if(Number.isFinite(Number(payload.speed)))state.speed=Number(payload.speed);
    if(Number.isFinite(Number(payload.temperature)))state.temperature=Number(payload.temperature);
    if(payload.attitude)state.attitude=payload.attitude;
    if(payload.gps)state.gps=payload.gps;
    if(payload.uwb)state.uwb=payload.uwb;
    if(payload.ekf)state.ekf=payload.ekf;
    if(Object.prototype.hasOwnProperty.call(payload,"armed"))state.armed=Boolean(payload.armed);
    if(payload.flightMode)state.flightMode=String(payload.flightMode);
    renderTelemetry();renderGpsStatus();
  });
  transport.on("connection",function(this: any,payload?: any): any {
    if(payload&&payload.degraded){state.boardOnline=false;state.connection="degraded";state.controlLink=false;state.speed=0;if(window.resetMantaJoystick)window.resetMantaJoystick();}
    else if(payload&&payload.connected){state.boardOnline=true;state.connection=state.pixhawkOnline?"connected":"offline";state.controlLink=state.pixhawkOnline;}
    renderConnection();renderTelemetry();renderHardwareStatus();
  });
  transport.on("log",function(this: any,entry?: any): any {addLog(entry||{})});
  transport.on("emergency",function(this: any,payload?: any): any {
    if(!payload||payload.active!==false){state.armed=false;state.emergency=true;state.connection="emergency";state.speed=0;if(window.resetMantaJoystick)window.resetMantaJoystick();renderConnection();renderTelemetry();}
  });
  transport.on("armed",function(this: any,payload?: any): any {
    state.armed=Boolean(payload&&payload.armed);if(!state.armed)state.speed=0;renderConnection();renderTelemetry();
  });
  transport.on("hardwareStatus",function(this: any,payload?: any): any {
    payload=payload||{};["boardOnline","pixhawkOnline","imuOnline","motorsOnline","gimbalOnline","gimbalOutputAvailable"].forEach(function(this: any,key?: any): any {if(Object.prototype.hasOwnProperty.call(payload,key))state[key]=Boolean(payload[key])});
    if(payload.leftMotor)state.motorLinks.left=payload.leftMotor;
    if(payload.rightMotor)state.motorLinks.right=payload.rightMotor;
    if(state.connected){state.connection=state.boardOnline?(state.pixhawkOnline?"connected":"offline"):"degraded";state.controlLink=state.boardOnline&&state.pixhawkOnline;}
    renderConnection();renderTelemetry();renderHardwareStatus();
  });
  transport.on("gimbalState",function(this: any,payload?: any): any {applyGimbalState(payload||{});renderHardwareStatus()});
  transport.on("gimbalTarget",function(this: any,payload?: any): any {
    if(!payload||state.gimbalMode==="click")return;
    var targetMode: any=payload.mode==="beacon"?"beacon":payload.mode==="swimmer"?"swimmer":payload.mode==="face"?"face":state.trackMode;
    state.clickMarker=null;state.trackTarget=payload;state.runningTrackMode=targetMode;state.trackMode=targetMode;state.gimbalMode=targetMode;state.trackStatus={locked:true,status:payload.status||"track",message:payload.message||"LOCKED",mode:targetMode};renderTrackingMode();
  });
  transport.on("gimbalTrackStatus",function(this: any,payload?: any): any {
    if(!payload||state.gimbalMode==="click")return;
    state.trackStatus=payload;if(payload.mode){state.runningTrackMode=payload.mode==="beacon"?"beacon":payload.mode==="swimmer"?"swimmer":"face";if(state.trackingActive){state.trackMode=state.runningTrackMode;state.gimbalMode=state.runningTrackMode}}if(!payload.locked)state.trackTarget=null;renderTrackingMode();
  });
  transport.on("uwbFollow",function(this: any,payload?: any): any {state.uwbFollow=payload||state.uwbFollow;renderUwbFollow()});
  return transport;
}

var STATIC_EN: any=Object.fromEntries([["请旋转设备","Rotate your device"],["Manta 仅提供横屏控制体验","Manta is designed for landscape control"],["连接","LINK"],["延迟","LATENCY"],["速度","SPEED"],["飞控温度","FCU TEMP"],["未连接","Offline"],["工具","Tools"],["演示","Demo"],["已开启","Enabled"],["第1位","Digit 1"],["第2位","Digit 2"],["第3位","Digit 3"],["第4位","Digit 4"],["第5位","Digit 5"],["第6位","Digit 6"],["设备","Device"],["地图","Map"],["录像","Media"],["附近设备","Nearby device"],["可以连接","Ready to connect"],["已发现","Discovered"],["等待扫描","Waiting to scan"],["搜索 Manta","Find Manta"],["Manta 产品画面","Manta product view"],["选择设备后，首次连接需输入六位配对码","Select a device; first-time pairing requires a six-digit code."],["选择要连接的设备","Choose a Manta"],["正在扫描附近的 Manta…","Scanning for nearby Manta…"],["重新扫描","Scan again"],["面向开放水域游泳与专业野泳运动员的自主水面摄影机器人。","An autonomous surface camera robot for open-water swimmers and professionals."],["安全连接已就绪","Secure connection ready"],["BLE 5.0 发现","BLE 5.0 discovery"],["互联网连接保持在线","Internet stays online"],["连接 Manta","Connect Manta"],["首次连接需要输入 Manta 显示的 6 位配对码","First connection requires the 6-digit code shown by Manta"],["手动跟随","Manual follow"],["自动跟随","Auto follow"],["即将推出","Coming soon"],["断开","Disconnect"],["飞控姿态","Flight attitude"],["IMU 校准","IMU calibration"],["横滚 ROLL","ROLL"],["俯仰 PITCH","PITCH"],["航向 YAW","YAW"],["校准状态","Calibration"],["良好","Good"],["IMU 未连接","IMU offline"],["等待 Pixhawk 遥测","Waiting for Pixhawk telemetry"],["在线","Online"],["离线","Offline"],["移动控制","Drive control"],["前进","FORWARD"],["后退","REVERSE"],["左推进","Left thrust"],["右推进","Right thrust"],["紧急停止","Emergency stop"],["滑动立即停止","Slide to stop now"],["推进状态","Propulsion"],["已解除武装","Disarmed"],["已武装","Armed"],["滑动武装","Slide to arm"],["解除武装","DISARM"],["滑动武装推进电机","Slide to arm propulsion"],["推进已武装","Propulsion armed"],["推进已解除武装，电机输出已归零","Propulsion disarmed. Motor output is zero."],["请先武装推进电机","Arm propulsion first"],["影像信号中断","Video signal lost"],["控制链路仍然在线，请谨慎移动","Control link remains online. Move with caution."],["云台保护已触发","Gimbal protection active"],["检测到云台电机异常，所有云台功能已停止","A gimbal motor fault was detected. All gimbal functions are stopped."],["查看诊断报告","View diagnostic report"],["点击居中","Click center"],["人脸跟踪","Face track"],["泳者跟踪","Swimmer track"],["输出日志","Output log"],["全部","All"],["指令","Commands"],["硬件","Hardware"],["上位机","Host"],["下位机","Controller"],["电机","Motors"],["云台","Gimbal"],["GPS 状态","GPS status"],["等待设备连接","Waiting for device"],["0 颗卫星","0 satellites"],["地图来源：MANTA 演示矢量 · 非导航地图","Map source: MANTA demo vectors · Not for navigation"],["机内存储","On-device storage"],["今天","Today"],["已下载","Downloaded"],["3 个项目","3 items"],["开放水域跟拍 01","Open-water follow 01"],["泳者跟踪测试","Swimmer tracking test"],["返航影像","Return footage"],["下载","Download"],["快捷工具","Quick tools"],["显示","Display"],["外观","Appearance"],["跟随使用环境","Match your environment"],["深色","Dark"],["浅色","Light"],["语言","Language"],["简体中文","Simplified Chinese"],["通信模式","Transport mode"],["Preview 模拟通信","Preview simulation"],["互联网","Internet"],["连接 Manta 时保持在线","Stays online while connected to Manta"],["完整日志","Full logs"],["导出诊断信息","Export diagnostics"],["帮助与安全","Help & safety"],["连接、控制与急停说明","Connection, control, and emergency-stop guide"],["状态模拟器","State simulator"],["用于在电脑端预览所有真实状态，不会向 Manta 发送指令。","Preview real interface states on this computer without sending commands to Manta."],["正常连接","Connected"],["视频中断","Video lost"],["云台故障","Gimbal fault"],["温度告警","Temperature alert"],["急停","Emergency"],["设备离线","Device offline"],["输入 Manta 显示的配对码","Enter the pairing code shown by Manta"],["Preview 测试码：","Preview test code: "],["验证并连接","Verify and connect"],["正在下载到 Manta App","Downloading to Manta App"],["下载完成前请保持 App 在前台并维持与 Manta 的连接。","Keep the app in the foreground and maintain the Manta connection until the download finishes."],["下载期间无法切换页面","Navigation is locked during download"],["下载完成","Download complete"],["完成","Done"],["云台健康诊断报告","Gimbal health report"],["已执行云台停止保护","Gimbal stop protection executed"],["实际响应","Actual response"],["连续异常","Consecutive faults"],["通信","Link"],["处置","Action"],["当前结论是“疑似卡死”，不是硬件确定性判定。建议断电检查机械阻力、供电和串口错误，然后重新执行云台自检。","The current result is a suspected stall, not a definitive hardware diagnosis. Power down, inspect mechanical resistance, power, and serial errors, then run gimbal self-test again."],["我知道了","Got it"],["IMU 六面校准","Six-position IMU calibration"],["校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。","Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface."],["准备开始","Ready"],["准备六个放置方向","Prepare six orientations"],["开始后，请按照飞控要求逐面放稳并确认。","After starting, follow each FCU pose request and confirm only when Manta is stable."],["取消","Cancel"],["开始校准","Start calibration"],["紧急停止推进","Emergency stop thrust"],["显示云台 OSD","Show gimbal OSD"],["隐藏云台 OSD","Hide gimbal OSD"],["云台回中","Center gimbal"],["停止云台","Stop gimbal"],["更多参数","More settings"],["请先连接 Manta","Connect to Manta first"],["浏览器 Preview 不写入 App 沙盒；请在原生 App 录像页执行真实下载","Browser Preview cannot write to the app sandbox. Use the native app media page for a real download."],["已切换真实通信，请连接 Manta","Live transport selected. Connect to Manta."],["已切换 Preview 模拟通信","Preview simulation selected"],["控制区已移至左侧","Controls moved to the left"],["影像区已移至左侧","Video moved to the left"],["云台当前不可控制","Gimbal control is currently unavailable"],["云台正在回中","Gimbal is centering"],["云台已停止","Gimbal stopped"],["将由原生 App 请求相册权限后保存","The native app will request Photos access before saving"],["已定位 MANTA ROBOTIC","MANTA ROBOTIC located"],["Manta 已连接，互联网保持在线","Manta connected. Internet remains online."],["已连接 Manta 主机，等待 Pixhawk、IMU 与电机上线","Manta host connected; waiting for Pixhawk, IMU, and motors"],["已断开 Manta","Manta disconnected"],["紧急停止已执行，推进输出已归零","Emergency stop executed. Thrust output is zero."],["IMU 六面校准完成","Six-position IMU calibration complete"],["IMU 校准未完成","IMU calibration incomplete"],["下载完成前无法切换页面","Navigation is locked until the download finishes"],["云台健康监测触发保护，请查看诊断报告","Gimbal health protection triggered. Open the diagnostic report."],["确认命令已发送；HTTP 成功不代表该面已完成","Confirmation sent; HTTP success does not mean the pose is complete."],["仍在等待飞控的 AWAITING_POSITION 状态","Still waiting for the FCU AWAITING_POSITION state."],["请先稳定连接 Manta，再开始 IMU 校准","Connect to Manta with a stable control link before IMU calibration."],["校准已开始，请完成飞控要求的六个姿态","Calibration is active. Complete all six poses requested by the FCU."],["设备已断开，无法确认校准结果。","The device disconnected before calibration could be confirmed."],["用户在校准期间执行了紧急停止。","Emergency stop was executed during calibration."],["飞控已确认六面校准完成。","The FCU confirmed all six calibration poses."],["请检查飞控状态后重试。","Check the FCU status and try again."],["飞控已确认完成","FCU confirmed completion"],["校准未完成","Calibration incomplete"],["IMU 校准完成","IMU calibration complete"],["IMU 校准失败","IMU calibration failed"],["等待飞控姿态请求","Waiting for FCU pose request"],["不要自行切换方向","Do not choose a pose manually"],["仅当遥测状态为 AWAITING_POSITION 且包含有效 stepCode 时才能确认。","Confirmation is enabled only when telemetry reports AWAITING_POSITION with a valid stepCode."],["正在准备第一个模拟姿态。","Preparing the first simulated pose."],["等待飞控确认","Waiting for FCU"],["已放稳，确认此面","Stable — confirm this pose"],["等待飞控的 AWAITING_POSITION 状态","Still waiting for the FCU AWAITING_POSITION state."],["推进控制已锁定","Thrust control is locked"],["发现 MANTA ROBOTIC · MANTA 2407","Discovered MANTA ROBOTIC · MANTA 2407"],["控制权已授予本机","Control granted to this device"],["已建立板端会话，控制保持锁定直到 Pixhawk 上线","Board session established; control remains locked until Pixhawk is online"],["控制链路已断开，本机控制已锁定；板端看门狗仍需实机验收","Control link lost. Local control is locked; the board watchdog still requires hardware validation."],["影像信号中断，控制链路保持在线","Video signal lost; control link remains online"],["飞控温度进入警戒区间","Flight-controller temperature entered the warning range"],["控制和遥测链路已断开，本机控制已锁定","Control and telemetry links were lost; local control is locked"],["设备连接已断开","Device connection lost"],["控制与遥测已锁定，等待板端看门狗确认停车","Control and telemetry are locked while the board watchdog confirms stop."],["录像已保存到 App 素材库","Recording saved to the app media library"],["正在预览机内录像","Previewing the on-device recording"],["录像无法播放：","Unable to play recording: "],["已切换至","Playback speed set to "],["倍播放","×"],["保存到相册","Save to Photos"],["GPS 可用（模拟）","GPS available (simulated)"],["GPS 信号可用（估算）","GPS signal available (estimated)"],["GPS 暂不可用","GPS currently unavailable"],["停止","Stopped"],["向右滑动触发紧急停止","Slide right to trigger emergency stop"],["正在发现附近设备","Discovering nearby device"],["正在发现","Discovering"],["正在配对","Pairing"],["正在连接","Connecting"],["已连接","Connected"],["链路降级","Degraded"],["正在扫描","Scanning"],["未发现设备","No devices found"],["信号强","Strong signal"],["信号良好","Good signal"],["信号一般","Fair signal"],["信号较弱","Weak signal"],["左","LEFT"],["右","RIGHT"],["更多","More"],["人脸","FACE"],["泳者","SWIMMER"],["Manta 360度移动摇杆","Manta 360-degree drive joystick"],["播放录像","Play recording"],["暂停录像","Pause recording"],["返回设备主页","Return to device home"],["实时状态","Live status"],["交换控制台布局","Swap console layout"],["交换布局","Swap layout"],["打开工具菜单","Open tools"],["主要页面","Main pages"],["Manta 设备","Manta device"],["Manta 水面摄影机器人","Manta surface camera robot"],["自动跟随将在后续版本开放","Auto follow will be available in a future version"],["Manta 云台实时画面","Live Manta gimbal view"],["云台模式","Gimbal mode"],["附近没有发现 Manta","No Manta found nearby"],["配对码不正确","Incorrect pairing code"],["Manta 状态接口不可用","Manta status endpoint unavailable"],["Manta 返回无效状态","Manta returned invalid status"],["请输入六位配对码","Please enter a six-digit code"],["不支持的云台指令","Unsupported gimbal command"],["录像列表不可用","Recording list unavailable"],["移动指令失败：","Drive command failed: "],["状态轮询失败：","Status poll failed: "],["云台反馈超时，已锁定云台控制","Gimbal feedback timeout; gimbal control locked"],["云台通信已断开","Gimbal link lost"],["云台收到非零运动目标，但角度与陀螺仪均无响应","Gimbal received non-zero motion target but no angle or gyroscope response"],["目标角速度 ","Target angular rate "],["运动目标已下发","Motion target was issued"],["检测窗口内角度变化 ","Angle change in detection window "],["无有效运动反馈","No valid motion feedback"],["反馈包超时","Feedback packet timed out"],["状态包在线 · 疑似机械卡滞","Status packets online · Suspected mechanical stall"],["停止跟踪、停止云台电机、保留主机移动控制","Stop tracking and gimbal motors; keep vehicle drive control"],["附近设备","Nearby device"],["安全配对","SECURE PAIRING"],["近距离","NEAR FIELD"],["让镜头跟上每一次破浪","Keep the camera with every open-water move"],["自主水面摄影机器人","Autonomous surface camera robot"],["面向开放水域游泳与专业野泳运动员的自主水面摄影机器人。","An autonomous surface camera robot for open-water swimmers and professionals."],["安全连接已就绪","Secure connection ready"],["BLE 5.0 发现","BLE 5.0 discovery"],["互联网连接保持在线","Internet stays online"],["保持联网","Stay online"],["蓝牙近距发现 · 5 GHz Wi-Fi影像 · 保持互联网在线","BLE discovery · 5 GHz video · Internet stays online"],["控制","CONTROL"],["影像","VISION"],["CONTROL","CONTROL"],["VISION","VISION"],["INTERNET","INTERNET"],["BLE 5.0","BLE 5.0"],["5 GHz","5 GHz"],["ONLINE","ONLINE"],["发现与控制","Discovery & control"],["预览与下载","Preview & download"],["当前航程","Current route"],["预计剩余 38 min","Estimated 38 min remaining"],["定位 Manta","Locate Manta"],["规划路线","Plan route"],["机内存储","On-device storage"],["3 个项目","3 items"],["开放水域跟拍 01","Open-water follow 01"],["泳者跟踪测试","Swimmer tracking test"],["返航影像","Return footage"],["全部","All"],["今天","Today"],["已下载","Downloaded"],["19.4 MB","19.4 MB"],["128 MB","128 MB"],["76 MB","76 MB"],["02:47","02:47"],["01:36","01:36"],["00:54","00:54"],["2026.07.09 · 16:19","2026.07.09 · 16:19"],["2026.07.09 · 15:48","2026.07.09 · 15:48"],["2026.07.09 · 15:21","2026.07.09 · 15:21"],["显示","Display"],["外观","Appearance"],["跟随使用环境","Match your environment"],["深色","Dark"],["浅色","Light"],["语言","Language"],["简体中文","Simplified Chinese"],["English","English"],["通信模式","Transport mode"],["Preview 模拟通信","Preview simulation"],["当前 Manta 后端接口","Current Manta backend"],["互联网","Internet"],["连接 Manta 时保持在线","Stays online while connected to Manta"],["完整日志","Full logs"],["导出诊断信息","Export diagnostics"],["帮助与安全","Help & safety"],["连接、控制与急停说明","Connection, control, and emergency-stop guide"],["状态模拟器","State simulator"],["正常连接","Connected"],["视频中断","Video lost"],["云台故障","Gimbal fault"],["温度告警","Temperature alert"],["急停","Emergency"],["设备离线","Device offline"],["延迟","Latency"],["速度","Speed"],["飞控温度","FCU Temp"],["18 ms","18 ms"],["1.4 m/s","1.4 m/s"],["46 °C","46 °C"],["LIVE","LIVE"],["1080P · 30FPS","1080P · 30FPS"],["REC 00:00","REC 00:00"],["SWIMMER","SWIMMER"],["点击居中","Click center"],["人脸跟踪","Face track"],["泳者跟踪","Swimmer track"],["OSD","OSD"],["云台回中","Center gimbal"],["停止云台","Stop gimbal"],["录像","Record"],["更多参数","More settings"],["X 0.00 · Y 0.00","X 0.00 · Y 0.00"],["左推进","Left thrust"],["右推进","Right thrust"],["当前速度","Current speed"],["左电机","Left motor"],["右电机","Right motor"],["输出 PWM","PWM output"],["输出 PWM ·","PWM output ·"],["0%","0%"],["紧急停止","Emergency stop"],["滑动立即停止","Slide to stop now"],["飞控姿态 PIXHAWK IMU","Flight attitude PIXHAWK IMU"],["移动控制 360° VECTOR","Drive control 360° VECTOR"],["输出日志 LIVE OUTPUT","Output log LIVE OUTPUT"],["全部 指令 硬件","All Commands Hardware"],["上位机 下位机 电机 云台","Host Controller Motors Gimbal"],["水域地图","Water map"],["GPS 状态","GPS status"],["地图来源：MANTA 演示矢量 · 非导航地图","Map source: MANTA demo vectors · Not for navigation"],["50 m","50 m"],["MANTA ROBOTIC","MANTA ROBOTIC"],["SECURE PAIRING","SECURE PAIRING"],["输入 Manta 显示的配对码","Enter the pairing code shown by Manta"],["MANTA ROBOTIC · MANTA 2407","MANTA ROBOTIC · MANTA 2407"],["Preview 测试码：","Preview test code: "],["240724","240724"],["验证并连接","Verify and connect"],["正在验证…","Verifying…"],["DOWNLOADING","DOWNLOADING"],["正在下载到 Manta App","Downloading to Manta App"],["20260709_161929_h264.mp4","20260709_161929_h264.mp4"],["0%","0%"],["0 MB / 19.4 MB","0 MB / 19.4 MB"],["下载完成前请保持 App 在前台并维持与 Manta 的连接。","Keep the app in the foreground and maintain the Manta connection until the download finishes."],["下载期间无法切换页面","Navigation is locked during download"],["GIMBAL PROTECTION","GIMBAL PROTECTION"],["云台健康诊断报告","Gimbal health report"],["GMB-MOTION-STALL","GMB-MOTION-STALL"],["已执行云台停止保护","Gimbal stop protection executed"],["SWIMMER TRACK / PAN +18°","SWIMMER TRACK / PAN +18°"],["1.8 s 内角度变化 < 0.8°","Angle change < 0.8° in 1.8 s"],["3 / 3 个检测窗口","3 / 3 detection windows"],["串口正常 · 状态包在线","Serial normal · Status packets online"],["停止跟踪、停止云台电机、保留主机移动控制","Stop tracking and gimbal motors; keep vehicle drive control"],["当前结论是“疑似卡死”，不是硬件确定性判定。建议断电检查机械阻力、供电和串口错误，然后重新执行云台自检。","The current result is a suspected stall, not a definitive hardware diagnosis. Power down, inspect mechanical resistance, power, and serial errors, then run gimbal self-test again."],["当前结论是\"疑似卡死\"，不是硬件确定性判定。建议断电检查机械阻力、供电和串口错误，然后重新执行云台自检。","The current result is a suspected stall, not a definitive hardware diagnosis. Power down, inspect mechanical resistance, power, and serial errors, then run gimbal self-test again."],["我知道了","Got it"],["PIXHAWK · SIX-POSITION","PIXHAWK · SIX-POSITION"],["IMU 六面校准","Six-position IMU calibration"],["校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。","Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface."],["准备开始","Ready"],["0 / 6","0 / 6"],["准备六个放置方向","Prepare six orientations"],["开始后，请按照飞控要求逐面放稳并确认。","After starting, follow each FCU pose request and confirm only when Manta is stable."],["取消","Cancel"],["开始校准","Start calibration"],["紧急停止推进","Emergency stop thrust"],["水平平放","Level"],["左侧朝下","Left side down"],["右侧朝下","Right side down"],["机头朝下","Nose down"],["机头朝上","Nose up"],["背面朝下","Back down"],["将 Manta 正常水平放置，保持完全静止。","Place Manta level in its normal orientation and keep it completely still."],["将 Manta 左侧朝下放稳，确认机身不再晃动。","Rest Manta on its left side and wait until it is completely stable."],["将 Manta 右侧朝下放稳，确认机身不再晃动。","Rest Manta on its right side and wait until it is completely stable."],["将机头垂直朝下并固定，保持静止。","Hold the nose vertically downward, secure the robot, and keep it still."],["将机头垂直朝上并固定，保持静止。","Hold the nose vertically upward, secure the robot, and keep it still."],["将 Manta 翻转，使背面朝下放稳，保持静止。","Turn Manta over, rest it on its back, and keep it completely still."],["Level","Level"],["Left side down","Left side down"],["Right side down","Right side down"],["Nose down","Nose down"],["Nose up","Nose up"],["Back down","Back down"],["Place Manta level in its normal orientation and keep it completely still.","Place Manta level in its normal orientation and keep it completely still."],["Rest Manta on its left side and wait until it is completely stable.","Rest Manta on its left side and wait until it is completely stable."],["Rest Manta on its right side and wait until it is completely stable.","Rest Manta on its right side and wait until it is completely stable."],["Hold the nose vertically downward, secure the robot, and keep it still.","Hold the nose vertically downward, secure the robot, and keep it still."],["Hold the nose vertically upward, secure the robot, and keep it still.","Hold the nose vertically upward, secure the robot, and keep it still."],["Turn Manta over, rest it on its back, and keep it completely still.","Turn Manta over, rest it on its back, and keep it completely still."],["飞控请求当前姿态","FCU requested this pose"],["确认命令已发送，等待飞控接受","Confirmation sent; waiting for FCU acceptance"],["已放稳，确认此面","Stable — confirm this pose"],["等待飞控确认","Waiting for FCU"],["不要自行切换方向","Do not choose a pose manually"],["仅当遥测状态为 AWAITING_POSITION 且包含有效 stepCode 时才能确认。","Confirmation is enabled only when telemetry reports AWAITING_POSITION with a valid stepCode."],["正在准备第一个模拟姿态。","Preparing the first simulated pose."],["校准已开始，请完成飞控要求的六个姿态","Calibration is active. Complete all six poses requested by the FCU."],["设备已断开，无法确认校准结果。","The device disconnected before calibration could be confirmed."],["用户在校准期间执行了紧急停止。","Emergency stop was executed during calibration."],["飞控已确认六面校准完成。","The FCU confirmed all six calibration poses."],["请检查飞控状态后重试。","Check the FCU status and try again."],["飞控已确认完成","FCU confirmed completion"],["校准未完成","Calibration incomplete"],["IMU 校准完成","IMU calibration complete"],["IMU 校准失败","IMU calibration failed"],["等待飞控姿态请求","Waiting for FCU pose request"],["校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。","Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface."],["准备开始","Ready"],["准备六个放置方向","Prepare six orientations"],["开始后，请按照飞控要求逐面放稳并确认。","After starting, follow each FCU pose request and confirm only when Manta is stable."],["取消","Cancel"],["开始校准","Start calibration"],["紧急停止推进","Emergency stop thrust"],["推进控制已锁定","Thrust control is locked"],["发现 MANTA ROBOTIC · MANTA 2407","Discovered MANTA ROBOTIC · MANTA 2407"],["控制权已授予本机","Control granted to this device"],["控制链路已断开，本机控制已锁定；板端看门狗仍需实机验收","Control link lost. Local control is locked; the board watchdog still requires hardware validation."],["影像信号中断，控制链路保持在线","Video signal lost; control link remains online"],["飞控温度进入警戒区间","Flight-controller temperature entered the warning range"],["控制和遥测链路已断开，本机控制已锁定","Control and telemetry links were lost; local control is locked"],["设备连接已断开","Device connection lost"],["控制与遥测已锁定，等待板端看门狗确认停车","Control and telemetry are locked while the board watchdog confirms stop."],["录像已保存到 App 素材库","Recording saved to the app media library"],["正在预览机内录像","Previewing the on-device recording"],["录像无法播放：","Unable to play recording: "],["已切换至","Playback speed set to "],["倍播放","×"],["保存到相册","Save to Photos"],["录像时间轴","Video timeline"],["播放速度","Playback speed"],["关闭","Close"],["GPS 可用（模拟）","GPS available (simulated)"],["GPS 信号可用（估算）","GPS signal available (estimated)"],["GPS 暂不可用","GPS currently unavailable"],["停止","Stopped"],["向右滑动触发紧急停止","Slide right to trigger emergency stop"],["Manta 360度移动摇杆","Manta 360-degree drive joystick"],["播放录像","Play recording"],["暂停录像","Pause recording"],["返回设备主页","Return to device home"],["实时状态","Live status"],["交换控制台布局","Swap console layout"],["交换布局","Swap layout"],["打开工具菜单","Open tools"],["主要页面","Main pages"],["Manta 设备","Manta device"],["Manta 水面摄影机器人","Manta surface camera robot"],["自动跟随将在后续版本开放","Auto follow will be available in a future version"],["Manta 云台实时画面","Live Manta gimbal view"],["云台模式","Gimbal mode"],["附近没有发现 Manta","No Manta found nearby"],["配对码不正确","Incorrect pairing code"],["Manta 状态接口不可用","Manta status endpoint unavailable"],["Manta 返回无效状态","Manta returned invalid status"],["请输入六位配对码","Please enter a six-digit code"],["不支持的云台指令","Unsupported gimbal command"],["录像列表不可用","Recording list unavailable"],["移动指令失败：","Drive command failed: "],["状态轮询失败：","Status poll failed: "],["云台反馈超时，已锁定云台控制","Gimbal feedback timeout; gimbal control locked"],["云台通信已断开","Gimbal link lost"],["云台收到非零运动目标，但角度与陀螺仪均无响应","Gimbal received non-zero motion target but no angle or gyroscope response"],["目标角速度 ","Target angular rate "],["运动目标已下发","Motion target was issued"],["检测窗口内角度变化 ","Angle change in detection window "],["无有效运动反馈","No valid motion feedback"],["反馈包超时","Feedback packet timed out"],["状态包在线 · 疑似机械卡滞","Status packets online · Suspected mechanical stall"],["停止跟踪、停止云台电机、保留主机移动控制","Stop tracking and gimbal motors; keep vehicle drive control"]]);

function localize(this: any,zh?: any,en?: any): any {return state.language==="en"?(en||STATIC_EN[zh]||zh):zh}

function renderTransportMode(this: any): any {
  $("#transportToggle").textContent=state.transportMode==="live"?"LIVE":"SIM";
  $("#transportHint").textContent=state.transportMode==="live"?localize("当前 Manta 后端接口","Current Manta backend"):localize("Preview 模拟通信","Preview simulation");
}

var staticTextSource: any=new WeakMap(),staticAttrSource: any=new WeakMap();
function translateStaticUi(this: any,lang?: any): any {
  var walker: any=document.createTreeWalker(document.body,NodeFilter.SHOW_TEXT),node: any;
  while((node=walker.nextNode())){
    if(!node.parentElement||/^(SCRIPT|STYLE)$/.test(node.parentElement.tagName))continue;
    if(node.parentElement.closest("[data-i18n-dynamic]"))continue;
    var raw: any=node.nodeValue,trimmed: any=raw.trim();if(!trimmed)continue;
    if(!staticTextSource.has(node)&&STATIC_EN[trimmed])staticTextSource.set(node,trimmed);
    var source: any=staticTextSource.get(node);if(!source)continue;
    var translated: any=lang==="en"?STATIC_EN[source]:source;
    node.nodeValue=raw.replace(trimmed,translated);
  }
  $$('[title],[aria-label],[placeholder]').forEach(function(this: any,el?: any): any {
    if(!staticAttrSource.has(el))staticAttrSource.set(el,{});
    var sources: any=staticAttrSource.get(el);
    ["title","aria-label","placeholder"].forEach(function(this: any,attr?: any): any {
      var value: any=el.getAttribute(attr);if(!value)return;
      if(!sources[attr]&&STATIC_EN[value])sources[attr]=value;
      if(sources[attr])el.setAttribute(attr,lang==="en"?STATIC_EN[sources[attr]]:sources[attr]);
    });
  });
  var navLabels: any={device:lang==="en"?"Device":"设备",map:lang==="en"?"Map":"地图",media:lang==="en"?"Media":"录像"};
  $$("#primaryNav [data-nav]").forEach(function(this: any,button?: any): any {var label: any=$("span",button);if(label)label.textContent=navLabels[button.dataset.nav]||label.textContent});
}

function renderTrackingMode(this: any): any {
  var mode: any=state.gimbalMode;
  $$("[data-gimbal-mode]").forEach(function(this: any,button?: any): any {button.classList.toggle("active",button.dataset.gimbalMode===mode)});
  $("#trackingReticle").hidden=true;
  $(".video-feed").classList.toggle("click-target-mode",mode==="click");
  $("#gimbalBeaconCalibration").hidden=mode!=="beacon";
  drawGimbalOverlay();
  if(mode!=="click")$("#trackingLabel").textContent=mode==="beacon"?localize("UWB 信标","UWB BEACON"):mode==="face"?localize("人脸","FACE"):localize("泳者","SWIMMER");
}

function uwbFollowIsStarting(this: any): any {
  var status: any=String(state.uwbFollow&&state.uwbFollow.status||"");
  return status==="disarming"||status==="switching_mode"||status==="arming"||status==="confirming_signal";
}

function uwbFollowIsRequested(this: any): any {
  var follow: any=state.uwbFollow||{};
  return Boolean(follow.active||follow.followRequested||follow.autoResumePending)||uwbFollowIsStarting();
}

function renderDriveMode(this: any): any {
  var automatic: any=state.driveMode==="auto",joystick: any=$("#joystick"),followControl: any=$("#driveFollowControl"),methodControl: any=$("#trackingMethodControl");
  $("#manualFollowButton").classList.toggle("active",!automatic);
  $("#autoFollowButton").classList.toggle("active",automatic);
  joystick.hidden=automatic;
  followControl.hidden=!automatic;
  methodControl.hidden=!automatic;
  $("#driveModeLabel").textContent=automatic?"UWB AUTO FOLLOW":localize("双桨推进","DUAL THRUST");
  if(!automatic)$("#driveOutput").textContent="X "+Number(state.vector.x||0).toFixed(2)+" · Y "+Number(state.vector.y||0).toFixed(2);
  joystick.setAttribute("aria-disabled",String(automatic||!state.connected||!state.controlLink||!state.armed||state.calibrating));
}

function setDriveMode(this: any,mode?: any): any {
  var nextMode: any=mode==="auto"?"auto":"manual";
  if(state.driveMode===nextMode)return;
  var activeTransport: any=ensureTransport();
  if(nextMode==="auto"){
    if(window.resetMantaJoystick)window.resetMantaJoystick();
    if(activeTransport&&activeTransport.setDriveEnabled)activeTransport.setDriveEnabled(false);
  }else{
    var enableManual: any=function(this: any): any {if(activeTransport&&activeTransport.setDriveEnabled)activeTransport.setDriveEnabled(true);renderDriveMode()};
    if(activeTransport&&uwbFollowIsRequested()){
      activeTransport.stopUwbFollow().catch(function(this: any,error?: any): any {showToast(error.message||"停止自动跟随失败")}).finally(enableManual);
    }else enableManual();
  }
  state.driveMode=nextMode;
  renderDriveMode();renderUwbFollow();renderArmControl();
}

function renderUwbFollow(this: any): any {
  var follow: any=state.uwbFollow||{},button: any=$("#driveFollowSwitch"),label: any=$("#driveFollowSwitchLabel"),status: any=$("#driveFollowSwitchStatus");
  if(!button||!label||!status)return;
  var busy: any=uwbFollowIsStarting(),searching: any=Boolean(follow.autoResumePending)&&!busy,requested: any=uwbFollowIsRequested();
  var trackingMethod: any=String(follow.trackingMethod||"traditional").toLowerCase()==="neural"?"neural":"traditional";
  $$("[data-tracking-method]").forEach(function(this: any,methodButton?: any): any {
    var selected: any=methodButton.dataset.trackingMethod===trackingMethod;
    methodButton.classList.toggle("active",selected);
    methodButton.setAttribute("aria-checked",String(selected));
    methodButton.disabled=requested||!state.connected||!state.controlLink;
  });
  $("#trackingMethodState").textContent=trackingMethod==="neural"?"SMARTUSV PPO":"TRADITIONAL";
  button.classList.toggle("active",requested);
  button.classList.toggle("busy",busy);
  button.disabled=state.driveMode!=="auto"||(!requested&&(!state.connected||!state.controlLink));
  label.textContent=follow.active?localize("停止跟随","Stop follow"):searching?localize("停止搜索","Stop search"):busy?localize("取消启动","Cancel start"):localize("启动跟随","Start follow");
  var labels: any={awaiting_signal:localize("搜索 UWB","Searching for UWB"),awaiting_trusted_uwb:localize("搜索 UWB","Searching for UWB"),auto_resume_ready:localize("UWB 已找到，准备跟随","UWB acquired; preparing to follow"),confirming_recovered_uwb:localize("确认连续有效 UWB","Confirming valid UWB samples"),auto_resume_recovering:localize("定位恢复中","Recovering target"),acro_velocity_unavailable:localize("等待有效航速数据","Waiting for valid velocity"),acro_parameters_not_ready:localize("等待ACRO控制参数","Waiting for ACRO parameters"),imu_stale:localize("航向数据过期，已退出","Stale heading; stopped"),prediction_clock_reset:localize("时钟变化，已退出","Clock changed; stopped"),predicting:localize("短时预测跟随","Predicted following"),waiting_target:localize("等待目标出水","Waiting for swimmer"),recovering:localize("定位恢复中","Recovering target"),prediction_tracking:localize("短时预测跟随","Predicted following"),prediction_recovering:localize("定位恢复中","Recovering target"),prediction_reacquiring:localize("校验出水测量","Checking new measurements"),prediction_pose_unavailable:localize("运动状态不足，等待出水","Motion estimate unavailable; waiting"),prediction_warming_up:localize("等待连续有效测量","Waiting for valid samples"),prediction_limit:localize("预测到限，等待出水","Prediction limit; waiting"),prediction_clearance:localize("间距不足，等待出水","Insufficient clearance; waiting"),prediction_speed_limit:localize("船速过高，暂停预测推进","Vessel too fast; waiting"),prediction_wait_timeout:localize("目标失联超时，已退出","Target timeout; stopped"),target_identity_changed:localize("目标身份变化，已退出","Target changed; stopped"),tracking:localize("跟随中","Tracking"),holding:localize("间距保护，保持停止","Separation protection; holding"),holding_distance:localize("间距保护，保持停止","Separation protection; holding"),method_selected:localize("等待启动","Ready to start"),disarming:localize("正在解除武装","Disarming"),switching_mode:localize("正在切换控制模式","Switching control mode"),arming:localize("正在自动武装","Auto arming"),gps_fix_invalid:localize("GPS无效","GPS invalid"),ekf_unhealthy:localize("EKF无效","EKF invalid"),imu_invalid:localize("IMU航向无效","IMU yaw invalid"),uwb_stale:localize("UWB失效","UWB stale"),guided_required:localize("需要GUIDED模式","GUIDED required"),manual_required:localize("需要MANUAL模式","MANUAL required"),pilot_steering_not_ready:localize("等待电机控制参数","Waiting for motor control parameters"),manual_mode_timeout:localize("MANUAL切换超时","MANUAL timeout"),acro_required:localize("需要ACRO模式","ACRO required"),guided_mode_timeout:localize("GUIDED切换超时","GUIDED timeout"),acro_mode_timeout:localize("ACRO切换超时","ACRO timeout"),arming_timeout:localize("武装超时","Arming timeout"),manual_stop:localize("已停止","Stopped"),not_started:localize("未启动","Not started")};
  var estimate: any=follow.targetEstimate||{};
  status.textContent=searching?localize("搜索 UWB · 等待有效信号","Searching for UWB · waiting for valid signal"):labels[follow.reason]||labels[follow.status]||String(follow.reason||follow.status||localize("未启动","Not started"));
  if(follow.active&&follow.relativeImuControl&&!follow.pausePropulsion)status.textContent=(follow.planner==="nmpc"?(follow.gpsSpeedControlActive?localize("GPS 闭环预测 · ","GPS predictive control · "):localize("UWB 相对预测 · ","UWB relative predictive control · ")):(follow.gpsSpeedControlActive?localize("GPS 速度闭环 · ","GPS speed feedback · "):localize("UWB 角度跟随 · ","UWB bearing follow · ")))+status.textContent;
  if(follow.active&&follow.planner==="nmpc"&&follow.pausePropulsion)status.textContent=follow.reason==="nmpc_relative_motion_warming_up"?localize("建立相对运动估计","Estimating relative motion"):localize("预测约束未满足，推进暂停","Prediction constraints unmet; propulsion paused");
  if(searching&&follow.waitingReason&&follow.waitingReason!=="uwb_stale")status.textContent=localize("搜索 UWB · ","Searching for UWB · ")+(labels[follow.waitingReason]||follow.waitingReason);
  if(follow.active&&["innovation_outlier","occlusion_suspected","distance_out_of_range","azimuth_out_of_range","elevation_out_of_range","non_finite_measurement"].includes(state.uwb&&state.uwb.rejectionReason))status.textContent=localize("拒绝异常值 · ","Outlier rejected · ")+status.textContent;
  if(follow.active&&estimate.source==="predicted"&&Number.isFinite(estimate.predictionAgeMs))status.textContent+=" · "+(estimate.predictionAgeMs/1000).toFixed(1)+"s · "+localize("误差界 ","Error bound ")+Number(estimate.uncertaintyM||0).toFixed(2)+"m";
  renderRelativePosition();
  if(state.driveMode==="auto"){
    if(follow.active){
      var relative: any=follow.relativeBodyM||{},forward: any=Number(relative.x),right: any=Number(relative.y);
      $("#driveOutput").textContent=Number.isFinite(forward)&&Number.isFinite(right)?localize("前后 ","FWD ")+forward.toFixed(2)+"m · "+localize("左右 ","RIGHT ")+right.toFixed(2)+"m · "+String(follow.requiredFlightMode||"AUTO"):Number(follow.desiredSpeedMps||0).toFixed(2)+" m/s · AUTO";
    }else $("#driveOutput").textContent=searching?localize("搜索 UWB · 推进停止","SEARCHING UWB · PROPULSION STOPPED"):busy?String(follow.status||"STARTING").toUpperCase():"AUTO READY";
  }
}

function renderBeaconCalibration(this: any): any {
  var cal: any=beaconCalibration,next: any=$("#beaconCalibrationNext"),cancel: any=$("#beaconCalibrationCancel");
  if(!cal){
    $("#beaconCalibrationProgressText").textContent=localize("准备开始","Ready");
    $("#beaconCalibrationCounter").textContent="0 / 9";$("#beaconCalibrationBar").style.width="0%";
    $("#beaconCalibrationPoseTitle").textContent=localize("准备九个 3×3 点位","Prepare nine 3x3 points");
    $("#beaconCalibrationInstruction").textContent=localize("开始后，Beacon跟随会暂停；请逐点将信标放到画面中心再记录。","Beacon tracking pauses. Center the beacon at each point, then capture.");
    next.textContent=localize("开始九点校准","Start 9-point calibration");next.disabled=beaconCalibrationBusy;cancel.hidden=false;return;
  }
  var captured: any=Number(cal.index)||0,total: any=Number(cal.total)||9,complete: any=cal.status==="complete";
  $("#beaconCalibrationCounter").textContent=captured+" / "+total;
  $("#beaconCalibrationBar").style.width=Math.round(captured/total*100)+"%";
  $("#beaconCalibrationProgressText").textContent=complete?localize("拟合完成，Beacon跟随已恢复","Calibration complete; tracking resumed"):localize("云台已到第 "+(captured+1)+" 个点","Gimbal at point "+(captured+1));
  if(complete){
    var result: any=cal.result||{},metrics: any=result.metrics||{};
    $("#beaconCalibrationPoseTitle").textContent=localize("九点校准完成","9-point calibration complete");
    $("#beaconCalibrationInstruction").textContent=result.model==="rigid_3d"?"3D RMS "+Number(metrics.angularRmsErrorDeg||0).toFixed(2)+"° · MAX "+Number(metrics.angularMaxErrorDeg||0).toFixed(2)+"°":"Yaw R² "+Number(metrics.yawR2||0).toFixed(3)+" · Pitch R² "+Number(metrics.pitchR2||0).toFixed(3);
    next.textContent=localize("完成","Done");next.disabled=false;cancel.hidden=true;return;
  }
  if(cal.status==="failed"){
    $("#beaconCalibrationProgressText").textContent=localize("拟合未通过","Fit rejected");
    $("#beaconCalibrationPoseTitle").textContent=localize("请重新执行九点校准","Repeat the 9-point calibration");
    $("#beaconCalibrationInstruction").textContent=cal.message||localize("点位跨度或居中精度不足。","Point span or centering accuracy was insufficient.");
    next.textContent=localize("重新开始","Restart");next.disabled=false;cancel.hidden=false;return;
  }
  $("#beaconCalibrationPoseTitle").textContent=localize("将信标放到画面正中心","Place beacon at image center");
  $("#beaconCalibrationInstruction").textContent=localize("确认画面中心与信标重合后，点击记录。云台随后自动移动到下一点。","When centered, capture. The gimbal then moves automatically.");
  next.textContent=localize("已居中，记录第 "+(captured+1)+" 点","Centered — capture point "+(captured+1));
  next.disabled=beaconCalibrationBusy||cal.status!=="awaiting_center";cancel.hidden=false;
}

var beaconCalibrationPreviewFrame: any=0;
function stopBeaconCalibrationPreview(this: any): any {
  if(beaconCalibrationPreviewFrame)cancelAnimationFrame(beaconCalibrationPreviewFrame);
  beaconCalibrationPreviewFrame=0;
}

function startBeaconCalibrationPreview(this: any): any {
  stopBeaconCalibrationPreview();
  function draw(this: any): any {
    var modal: any=$("#beaconCalibrationModal"),canvas: any=$("#beaconCalibrationCanvas");
    if(!modal||modal.hidden||!canvas){beaconCalibrationPreviewFrame=0;return}
    var source: any=state.transportMode==="live"&&$("#liveGimbalWebrtc").srcObject?$("#liveGimbalWebrtc"):state.transportMode==="live"?$("#liveGimbalFeed"):$("#gimbalVideo");
    var width: any=Math.max(1,canvas.clientWidth),height: any=Math.max(1,canvas.clientHeight),ratio: any=window.devicePixelRatio||1;
    if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio)}
    var sourceWidth: any=Number(source&&source.videoWidth||source&&source.naturalWidth||0),sourceHeight: any=Number(source&&source.videoHeight||source&&source.naturalHeight||0);
    var context: any=canvas.getContext("2d");context.setTransform(ratio,0,0,ratio,0,0);context.fillStyle="#03070a";context.fillRect(0,0,width,height);
    if(source&&sourceWidth>0&&sourceHeight>0){var scale: any=Math.min(width/sourceWidth,height/sourceHeight),drawWidth: any=sourceWidth*scale,drawHeight: any=sourceHeight*scale;try{context.drawImage(source,(width-drawWidth)/2,(height-drawHeight)/2,drawWidth,drawHeight)}catch(_error: any){}}
    beaconCalibrationPreviewFrame=requestAnimationFrame(draw);
  }
  draw();
}

function openBeaconCalibration(this: any): any {
  if(state.gimbalMode!=="beacon"||!state.gimbalOutputAvailable){showToast("请先进入UWB信标模式并打开云台控制串口","Enter UWB Beacon mode and open the gimbal control port first.");return}
  beaconCalibration=null;beaconCalibrationBusy=false;renderBeaconCalibration();setModal("beaconCalibrationModal",true);startBeaconCalibrationPreview();
}

function advanceBeaconCalibration(this: any): any {
  if(beaconCalibration&&beaconCalibration.status==="complete"){stopBeaconCalibrationPreview();setModal("beaconCalibrationModal",false);return}
  var activeTransport: any=ensureTransport();
  if(!activeTransport){showToast("通信模块未加载","Transport unavailable");return}
  if(beaconCalibration&&beaconCalibration.status==="failed")beaconCalibration=null;
  beaconCalibrationBusy=true;renderBeaconCalibration();
  var request: any=beaconCalibration?activeTransport.captureBeaconCalibration():activeTransport.startBeaconCalibration();
  request.then(function(this: any,payload?: any): any {beaconCalibration=payload.calibration||null;renderBeaconCalibration()}).catch(function(this: any,error?: any): any {if(error.payload&&error.payload.calibration)beaconCalibration=error.payload.calibration;showToast(error.message||"校准操作失败","Calibration failed")}).finally(function(this: any): any {beaconCalibrationBusy=false;renderBeaconCalibration()});
}

function cancelBeaconCalibration(this: any): any {
  var activeTransport: any=ensureTransport();
  if(beaconCalibration&&beaconCalibration.active&&activeTransport&&activeTransport.cancelBeaconCalibration){activeTransport.cancelBeaconCalibration().catch(function(this: any): any {})}
  stopBeaconCalibrationPreview();
  beaconCalibration=null;beaconCalibrationBusy=false;setModal("beaconCalibrationModal",false);
}

function gimbalControlAvailable(this: any): any {
  return Boolean(state.connected&&state.boardOnline&&state.gimbalOutputAvailable&&state.gimbalLinkStatus!=="offline"&&state.gimbalLinkStatus!=="command_timeout"&&!state.gimbalFault);
}

function applyGimbalState(this: any,payload?: any): any {
  var previousLinkStatus: any=state.gimbalLinkStatus;
  state.gimbalOsdSupported=!payload.capabilities||payload.capabilities.osd!==false;
  state.gimbalOnline=Boolean(payload.connected);state.gimbalOutputAvailable=Boolean(payload.portOpen||payload.connected);state.gimbalLinkStatus=String(payload.linkStatus||(!state.gimbalOutputAvailable?"offline":"ready"));state.trackingActive=Boolean(payload.trackingActive);
  if(state.gimbalLinkStatus==="command_timeout"&&previousLinkStatus!=="command_timeout")showToast(payload.lastError||"云台命令无响应，请检查云台供电和串口接线","The gimbal did not respond to the command. Check power and serial wiring.");
  if(payload.trackMode==="face"||payload.trackMode==="swimmer"||payload.trackMode==="beacon"){state.runningTrackMode=payload.trackMode;if(state.trackingActive||state.gimbalMode==="click")state.trackMode=payload.trackMode}
  if(state.trackingActive){state.gimbalMode=state.trackMode;state.clickMarker=null}
  if(payload.trackStatus){state.trackStatus=payload.trackStatus;if(!payload.trackStatus.locked)state.trackTarget=null}
  if(payload.lastTarget&&payload.lastTarget.locked&&state.trackingActive)state.trackTarget=payload.lastTarget;
  renderGimbalOsd();renderTrackingMode();
}

function gimbalVideoMapping(this: any,sourceWidthOverride?: any,sourceHeightOverride?: any): any {
  var view: any=$(".video-feed"),webrtcVideo: any=$("#liveGimbalWebrtc"),source: any=state.transportMode==="live"&&webrtcVideo.srcObject?webrtcVideo:state.transportMode==="live"?$("#liveGimbalFeed"):$("#gimbalVideo"),width: any=view.clientWidth,height: any=view.clientHeight;
  var sourceWidth: any=Number(sourceWidthOverride)||Number(source.naturalWidth||source.videoWidth)||1920,sourceHeight: any=Number(sourceHeightOverride)||Number(source.naturalHeight||source.videoHeight)||1080,scale: any=Math.min(width/sourceWidth,height/sourceHeight);
  return {width:width,height:height,sourceWidth:sourceWidth,sourceHeight:sourceHeight,scale:scale,offsetX:(width-sourceWidth*scale)/2,offsetY:(height-sourceHeight*scale)/2};
}

function drawGimbalOverlay(this: any): any {
  var canvas: any=$("#gimbalOverlay");if(!canvas)return;var map: any=gimbalVideoMapping(),ratio: any=window.devicePixelRatio||1;
  canvas.width=Math.max(1,Math.round(map.width*ratio));canvas.height=Math.max(1,Math.round(map.height*ratio));
  var ctx: any=canvas.getContext("2d");ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,map.width,map.height);
  ctx.strokeStyle="rgba(186,225,242,.72)";ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(map.width/2-9,map.height/2);ctx.lineTo(map.width/2+9,map.height/2);ctx.moveTo(map.width/2,map.height/2-9);ctx.lineTo(map.width/2,map.height/2+9);ctx.stroke();
  if(state.gimbalMode==="click"&&state.clickMarker){var mx: any=map.offsetX+state.clickMarker.x*map.sourceWidth*map.scale,my: any=map.offsetY+state.clickMarker.y*map.sourceHeight*map.scale;ctx.strokeStyle="#88e8ff";ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(mx,my,14,0,Math.PI*2);ctx.moveTo(mx-20,my);ctx.lineTo(mx+20,my);ctx.moveTo(mx,my-20);ctx.lineTo(mx,my+20);ctx.stroke()}
  if(state.gimbalMode==="beacon")return;
  if(!state.trackingActive||state.gimbalMode==="click")return;var target: any=state.trackTarget,status: any=state.trackStatus||{};
  if(target&&status.locked!==false){var fw: any=Number(target.frame_w||target.frameW)||1920,fh: any=Number(target.frame_h||target.frameH)||1080,targetMap: any=gimbalVideoMapping(fw,fh);
    var sourceX: any=Number(target.x||0),displayX: any=fw-sourceX,tx: any=targetMap.offsetX+(displayX/Math.max(fw,1))*(fw*targetMap.scale),ty: any=targetMap.offsetY+(Number(target.y||0)/Math.max(fh,1))*(fh*targetMap.scale),bw: any=Math.max(48,(Number(target.w||120)/Math.max(fw,1))*(fw*targetMap.scale)),bh: any=Math.max(48,(Number(target.h||120)/Math.max(fh,1))*(fh*targetMap.scale)),sx: any=Math.max(4,Math.min(targetMap.width-bw-4,tx-bw/2)),sy: any=Math.max(4,Math.min(targetMap.height-bh-4,ty-bh/2));
    var c: any=Math.max(14,Math.min(34,bw*.28,bh*.28));ctx.strokeStyle="#55e6a5";ctx.lineWidth=2;ctx.beginPath();[[sx,sy,1,1],[sx+bw,sy,-1,1],[sx,sy+bh,1,-1],[sx+bw,sy+bh,-1,-1]].forEach(function(this: any,p?: any): any {ctx.moveTo(p[0]+p[2]*c,p[1]);ctx.lineTo(p[0],p[1]);ctx.lineTo(p[0],p[1]+p[3]*c)});ctx.stroke();return;
  }
  ctx.font="600 10px system-ui";var label: any=String(status.message||(state.trackMode==="swimmer"?"can not find swimmer":"CAN NOT FIND FACE")).toUpperCase(),tw: any=ctx.measureText(label).width+20;ctx.fillStyle="rgba(6,16,23,.72)";ctx.fillRect((map.width-tw)/2,14,tw,26);ctx.fillStyle="#ffbd66";ctx.fillText(label,(map.width-tw)/2+10,31);
}

function isGimbalVideoInterrupted(this: any): any {
  var alert: any=$("#videoAlert");
  return !state.gimbalFault&&(state.videoLost||gimbalVideoStatus==="warning"||Boolean(alert&&!alert.hidden&&alert.dataset.reason==="video"));
}

function handleGimbalVideoClick(this: any,event?: any): any {
  if(event.target.closest(".video-toolbar"))return;
  if(isGimbalVideoInterrupted()){
    event.preventDefault();event.stopPropagation();
    refreshGimbalVideoStream();
    return;
  }
  if(state.gimbalMode!=="click"||!gimbalControlAvailable())return;
  var map: any=gimbalVideoMapping(),rect: any=$(".video-feed").getBoundingClientRect(),px: any=event.clientX-rect.left,py: any=event.clientY-rect.top,videoX: any=(px-map.offsetX)/map.scale,videoY: any=(py-map.offsetY)/map.scale;
  if(videoX<0||videoX>map.sourceWidth||videoY<0||videoY>map.sourceHeight)return;state.clickMarker={x:videoX/map.sourceWidth,y:videoY/map.sourceHeight};drawGimbalOverlay();
  var dx: any=Math.round((videoX/map.sourceWidth-0.5)*1920),dy: any=Math.round((videoY/map.sourceHeight-0.5)*1080),activeTransport: any=ensureTransport();
  if(activeTransport&&activeTransport.gimbalClick)activeTransport.gimbalClick(dx,dy).then(function(this: any): any {showToast("云台正在转向所选目标","Gimbal is moving to the selected target")}).catch(function(this: any,error?: any): any {showToast(error.message||"点击跟随指令失败","Click-to-follow command failed")});
}

function showToast(this: any,msg?: any,msgEn?: any): any {
  var n: any=$("#toast");clearTimeout(toastTimer);
  n.textContent=localize(msg,msgEn);n.classList.add("visible");
  toastTimer=setTimeout(function(this: any): any {n.classList.remove("visible")},2400);
}

function setModal(this: any,id?: any,open?: any): any {
  var n: any=document.getElementById(id);if(!n)return;
  n.hidden=!open;if(open)n.classList.add("visible");else n.classList.remove("visible");
}

function toggleDrawer(this: any,node?: any,open?: any): any {
  node.classList.toggle("open",open);
  node.setAttribute("aria-hidden",String(!open));
}

function addLog(this: any,entry?: any): any {
  entry=entry||{};
  var normalized: any={
    timestamp:entry.timestamp||new Date().toISOString(),
    level:String(entry.level||"INFO").toUpperCase(),
    source:String(entry.source||"APP").toUpperCase(),
    message:String(entry.message||""),
    messageEn:String(entry.messageEn||STATIC_EN[entry.message]||entry.message||""),
    category:entry.category||"system"
  };
  var isAlert: any=normalized.level==="WARNING"||normalized.level==="ERROR"||normalized.level==="CRITICAL";
  var alertKey: any=/\bUWB:\s*no frame\b/i.test(normalized.message)?normalized.level+":FCU_UWB_NO_FRAME":normalized.level+":"+normalized.source+":"+normalized.message;
  if(isAlert&&seenAlertLogKeys[alertKey])return;
  if(isAlert)seenAlertLogKeys[alertKey]=true;
  logEntries.push(normalized);
  if(logEntries.length>120)logEntries.shift();
  renderLogs();
}

function renderLogs(this: any): any {
  var root: any=$("#logStream");if(!root)return;
  var filtered: any=logEntries.filter(function(this: any,e?: any): any {return state.logFilter==="all"||e.category===state.logFilter}).slice(-40);
  root.innerHTML=filtered.map(function(this: any,e?: any): any {
    var d: any=new Date(e.timestamp);
    var msg: any=state.language==="en"?e.messageEn:e.message;
    return '<div class="log-row" data-level="'+e.level+'"><time>'+String(d.getHours()).padStart(2,"0")+":"+String(d.getMinutes()).padStart(2,"0")+":"+String(d.getSeconds()).padStart(2,"0")+'</time><b>'+escapeHtml(e.source)+'</b><span>'+escapeHtml(msg)+'</span></div>';
  }).join("");
  root.scrollTop=root.scrollHeight;
}

function escapeHtml(this: any,v?: any): any {return String(v).replace(/[&<>'"]/g,function(this: any,c?: any): any {return({"&":"&amp;","<":"&lt;",">":"&gt;","'":"&#39;",'"':"&quot;"} as Record<string,string>)[c]})}

function renderConnection(this: any): any {
  shell.dataset.connection=state.connection;
  var labels: any={idle:localize("未连接","Offline"),discovering:localize("正在发现","Discovering"),selecting:localize("选择设备","Choose device"),pairing:localize("正在配对","Pairing"),connecting:localize("正在连接","Connecting"),connected:localize("已连接","Connected"),degraded:localize("链路降级","Degraded"),offline:localize("设备离线","Device Offline"),emergency:localize("已解除武装","Disarmed")};
  $("#connectionMetric").textContent=labels[state.connection]||state.connection;
  var dot: any=$("#connDot");dot.className="status-dot "+(state.connection==="connected"?"connected":state.connection==="offline"||state.connection==="emergency"?"offline":state.connection==="degraded"?"degraded":state.connection==="pairing"?"pairing":"");
  $("#landingScreen").hidden=state.connected;
  $("#controlScreen").hidden=!state.connected;
  if(!state.connected)setGimbalVideoState("offline");
  $("#primaryNav").hidden=!state.connected;
  shell.dataset.calibrating=String(state.calibrating);
  renderDriveMode();
  renderUwbFollow();
  renderArmControl();
  renderMotorStatus();
  renderHardwareStatus();
}

function renderTelemetry(this: any): any {
  renderUwbStatus();
  if(!state.connected){
    $("#latencyMetric").textContent="--";
    $("#speedMetric").textContent="--";
    $("#temperatureMetric").textContent="--";
    $("#tempBar b").style.width="0";
    if($("#driveSpeedValue"))$("#driveSpeedValue").innerHTML="0.0<small>m/s</small>";
    return;
  }
  $("#latencyMetric").textContent=Math.round(state.latency)+" ms";
  if(!state.imuOnline){
    $("#speedMetric").textContent="--";$("#temperatureMetric").textContent="--";$("#tempBar b").style.width="0";
    if($("#driveSpeedValue"))$("#driveSpeedValue").innerHTML="--<small>m/s</small>";
    $("#rollValue").textContent="--";$("#pitchValue").textContent="--";$("#yawValue").textContent="--";$("#imuState").textContent=localize("未连接","Offline");return;
  }
  $("#imuState").textContent=localize("良好","Good");
  $("#speedMetric").textContent=state.speed.toFixed(1)+" m/s";
  $("#temperatureMetric").textContent=Math.round(state.temperature)+" °C";
  if($("#driveSpeedValue"))$("#driveSpeedValue").innerHTML=state.speed.toFixed(1)+"<small>m/s</small>";
  var pct: any=Math.max(8,Math.min(100,(state.temperature-20)/75*100));
  $("#tempBar b").style.width=pct+"%";
  $("#tempBar").className="temp-bar"+(state.temperature>75?" danger":state.temperature>60?" warning":"");
  $("#rollValue").textContent=signedAngle(state.attitude.roll);
  $("#pitchValue").textContent=signedAngle(state.attitude.pitch);
  $("#yawValue").textContent=Math.round((Number(state.attitude.yaw)||0)%360)+"°";
  var roll: any=Math.max(-25,Math.min(25,Number(state.attitude.roll)||0));
  var pitch: any=Math.max(-18,Math.min(18,Number(state.attitude.pitch)||0));
  var orb: any=$("#attitudeOrb");
  if(orb){
    var sky: any=$(".attitude-sky",orb),water: any=$(".attitude-water",orb),horizons: any=$$(".horizon-line,.pitch-line",orb);
    if(sky)sky.style.transform="translateY("+pitch*1.4+"px) rotate("+roll+"deg)";
    if(water)water.style.transform="translateY("+pitch*1.4+"px) rotate("+roll+"deg)";
    horizons.forEach(function(this: any,line?: any): any {line.style.transform="translateY("+pitch*1.4+"px) rotate("+roll+"deg)"});
  }
}

function renderUwbStatus(this: any): any {
  var uwb: any=state.uwb||{},panel: any=$("#uwbPanel"),dot: any=$("#uwbDot");
  if(!panel||!dot)return;
  var display: any=uwbDisplaySmoother.view(state.connected?uwb:{},Date.now());
  var distance: any=display.distanceM,azimuth: any=display.azimuthDeg,elevation: any=Number(uwb.elevationDeg);
  var valid: any=Boolean(state.connected&&display.valid&&Number.isFinite(elevation));
  panel.dataset.state=valid?"online":"offline";
  $("#uwbDistance").textContent=valid?distance.toFixed(2)+" m":"-- m";
  $("#uwbAngles").textContent=valid?azimuth.toFixed(1)+"° / "+elevation.toFixed(1)+"°":"--° / --°";
  if(!valid){dot.style.setProperty("--uwb-x","0px");dot.style.setProperty("--uwb-y","0px");renderRelativePosition();return;}
  var displayRange: any=Math.max(2,Math.min(20,distance)),radius: any=Math.min(19,19*distance/displayRange),angle: any=azimuth*Math.PI/180;
  dot.style.setProperty("--uwb-x",(-Math.sin(angle)*radius).toFixed(2)+"px");
  dot.style.setProperty("--uwb-y",(-Math.cos(angle)*radius).toFixed(2)+"px");
  renderRelativePosition();
}

var uwbDisplaySmoother: any=new window.MantaUwbDisplaySmoother(120),relativeDisplaySmoother: any=new window.MantaUwbDisplaySmoother(120),rawUwbDisplay: any=new window.MantaUwbRawDisplay();
function renderRelativePosition(this: any): any {
  var panel: any=$("#relativePositionPanel"),point: any=$("#relativeTargetPoint"),vector: any=$("#relativeTargetVector");
  if(!panel||!point||!vector)return;
  var now: any=Date.now(),uwb: any=state.uwb||{},display: any=uwbDisplaySmoother.view(state.connected?uwb:{},now),distance: any=display.distanceM,azimuth: any=display.azimuthDeg;
  var raw: any=rawUwbDisplay.view(uwb,now,state.connected),rawPoint: any=$("#relativeRawPoint"),rawVector: any=$("#relativeRawVector");
  var follow: any=state.uwbFollow||{},estimate: any=follow.targetEstimate||{};
  var predicted: any=Boolean(follow.active&&estimate.usable&&estimate.source==="predicted");
  if(predicted){distance=Math.hypot(estimate.relativeBodyM.x,estimate.relativeBodyM.y);azimuth=Math.atan2(-estimate.relativeBodyM.y,estimate.relativeBodyM.x)*180/Math.PI}
  var valid: any=Boolean(state.connected&&(predicted||display.valid)&&Number.isFinite(distance)&&Number.isFinite(azimuth));
  point.setAttribute("visibility",valid?"visible":"hidden");vector.setAttribute("visibility",valid?"visible":"hidden");
  rawPoint.setAttribute("visibility",raw.valid?"visible":"hidden");rawVector.setAttribute("visibility",raw.valid?"visible":"hidden");
  point.style.opacity=predicted?"0.55":"1";vector.setAttribute("stroke-dasharray",predicted?"3 2":"none");
  panel.dataset.state=valid?"online":raw.valid?"raw-only":"offline";
  $("#relativePositionTitle").textContent=predicted?localize("目标预测位置","Predicted target"):localize("目标相对位置","Relative target");
  $("#relativeRawLabel").textContent=localize("○ 原始","○ Raw")+(raw.valid?localize("·基站","·RX ")+raw.receiverId:"");
  $("#relativeRawValue").textContent=raw.valid?raw.distanceM.toFixed(2)+" m / "+raw.azimuthDeg.toFixed(1)+"°":"-- m / --°";
  $("#relativeRawValue").title=raw.valid?localize("模块原始值，未筛选，不用于控制。标签 ","Unscreened module values; not used for control. Tag ")+raw.tagId:localize("未收到新鲜完整定位帧","No fresh complete measurement");
  $("#relativeJudgedLabel").textContent=predicted?localize("◌ 预测","◌ Predicted"):localize("● 可信","● Trusted");
  var relative: any=predicted?estimate.relativeBodyM:follow.active&&!(follow.pausePropulsion&&estimate.source==="unavailable")?follow.relativeBodyM:null;
  if(valid&&relative&&!predicted){
    var relativeView: any=relativeDisplaySmoother.view(Object.assign({},uwb,{distanceM:Math.hypot(relative.x,relative.y),azimuthDeg:Math.atan2(-relative.y,relative.x)*180/Math.PI}),now);
    if(relativeView.valid)relative={x:relativeView.distanceM*Math.cos(relativeView.azimuthDeg*Math.PI/180),y:-relativeView.distanceM*Math.sin(relativeView.azimuthDeg*Math.PI/180)};
  }else relativeDisplaySmoother.reset();
  var angle: any=azimuth*Math.PI/180;
  // UWB +Y is left; the controller's +Y is right.
  var forward: any=valid?(relative&&Number.isFinite(relative.x)?relative.x:distance*Math.cos(angle)):0;
  var left: any=valid?(relative&&Number.isFinite(relative.y)?-relative.y:distance*Math.sin(angle)):0;
  var magnitude: any=Math.max(valid?Math.hypot(forward,left):0,raw.valid?Math.hypot(raw.forward,raw.left):0),scales: any=[1,2,5,10,20,50],scale: any=5;
  if(valid||raw.valid){scale=scales[scales.length-1];for(var i: any=0;i<scales.length;i++){if(magnitude<=scales[i]*.9){scale=scales[i];break}}scale=Math.max(scale,Math.ceil(magnitude/.9));}
  function plot(this: any,dot?: any,line?: any,x?: any,y?: any): any {var px: any=90-clamp(y/scale,-1,1)*51,py: any=56-clamp(x/scale,-1,1)*51;dot.setAttribute("cx",px.toFixed(2));dot.setAttribute("cy",py.toFixed(2));line.setAttribute("x2",px.toFixed(2));line.setAttribute("y2",py.toFixed(2));}
  if(valid)plot(point,vector,forward,left);
  if(raw.valid)plot(rawPoint,rawVector,raw.forward,raw.left);
  $("#relativeRingLabel").textContent=scale+" m";
  $("#relativePositionValue").textContent=valid?localize("前 ","FWD ")+forward.toFixed(2)+" · "+localize("左 ","LEFT ")+left.toFixed(2):localize("可信位置：暂无","Trusted position unavailable");
  $("#relativeRangeValue").textContent=valid?distance.toFixed(2)+" m / "+azimuth.toFixed(1)+"°":state.connected&&(raw.valid||(uwb.sensors||[]).some(function(this: any,s?: any): any {return s.measurementStatus==="suspect"}))?localize("拒收 / 等待恢复","Rejected / waiting"):"-- m / --°";
  var activeIds: any=Array.isArray(uwb.activeReceiverIds)?uwb.activeReceiverIds:[],mode: any=String(uwb.fusionMode||"");
  var sensors: any=uwb.sensors||[],sensor: any=sensors.find(function(this: any,s?: any): any {return s.receiverId===raw.receiverId})||sensors[0]||{};
  var quarantine: any=sensors.some(function(this: any,s?: any): any {return s.occlusionDiagnostics&&["suspect","quarantined","recovering"].includes(s.occlusionDiagnostics.state)});
  $("#relativeFusionMode").textContent=predicted?localize("预测，非实测","PREDICTED"):valid?(mode==="overlap_weighted_fusion"?localize("双基站融合","DUAL FUSION"):activeIds.length?localize("基站 ","RX ")+activeIds.join("+"):localize("单基站","SINGLE RX")):quarantine?localize("定位异常，等待恢复","POSITION SUSPECT"):localize("等待 UWB","WAITING UWB");
  var reason: any=uwb.rejectionReason||sensor.fusionExclusionReason||sensor.filterReason||sensor.invalidReason||"";
  var reasons: any={occlusion_suspected:localize("突跳隔离","Jump quarantine"),innovation_outlier:localize("创新偏差过大","Innovation outlier"),outside_receiver_fov:localize("超出视野","Outside FOV"),distance_out_of_range:localize("距离超限","Range rejected"),radio_invalid:localize("无线定位无效","Radio invalid"),heading_unavailable:localize("航向不可用","Heading unavailable"),stale_measurement:localize("可信数据过期","Trusted data stale")};
  var decision: any=!state.connected?localize("连接已断开","Disconnected"):predicted?localize("预测，非可信实测","Prediction, not a trusted observation"):valid?localize("已接纳；原始值仅供对照","Accepted; raw values for comparison"):!raw.valid?localize("无新鲜定位帧","No fresh measurement"):localize("未接纳：","Rejected: ")+(quarantine?localize("突跳隔离","Jump quarantine"):reasons[reason]||localize("等待可信位置","Waiting for trusted position"));
  $("#relativeDecision").textContent=decision;$("#relativeDecision").title=decision+(reason?" ("+reason+")":"");
}

function currentMapLocation(this: any): any {
  var gps: any=state.gps||{},latitude: any=Number(gps.latitude),longitude: any=Number(gps.longitude),updatedAt: any=Number(gps.updatedAt)||0;
  var valid: any=Number(gps.fixType)>=2&&Number.isFinite(latitude)&&Number.isFinite(longitude)&&(Math.abs(latitude)>0.000001||Math.abs(longitude)>0.000001);
  var heading: any=Number.isFinite(Number(gps.course))?Number(gps.course):Number(state.attitude&&state.attitude.yaw)||0;
  if(updatedAt>0&&updatedAt!==lastGpsSourceTimestamp){lastGpsSourceTimestamp=updatedAt;lastGpsReceivedAt=Date.now()}
  return{latitude:latitude,longitude:longitude,heading:heading,course:heading,updatedAt:updatedAt,fixValid:valid,fresh:Boolean(valid&&lastGpsReceivedAt>0&&Date.now()-lastGpsReceivedAt<5000)};
}

function updateOfflineMapPosition(this: any): any {
  if(offlineMap)offlineMap.update(currentMapLocation());
}

function initializeOfflineMap(this: any): any {
  if(mapInitStarted||!window.MantaOfflineSatelliteMap)return;
  mapInitStarted=true;
  offlineMap=window.MantaOfflineSatelliteMap.create($("#offlineSatelliteMap"),{onState:function(this: any,status?: any,detail?: any): any {
    if(status==="ready"){
      mapProvider="offline-satellite";$("#mapCanvas").dataset.provider="offline-satellite";
      mapSourceLabel=detail&&detail.mobileLabelZh||mapSourceLabel;
      mapSourceLabelEn=detail&&detail.mobileLabelEn||mapSourceLabelEn;
      $("#mapSource").textContent=localize(mapSourceLabel,mapSourceLabelEn);
      updateOfflineMapPosition();
    }else{
      mapProvider="offline-error";$("#mapSource").textContent=localize("离线卫星地图加载失败：","Offline satellite map failed: ")+detail;
    }
  }});
  $("#offlineMapZoomOut").onclick=function(this: any): any {offlineMap.zoomBy(-1)};
  $("#offlineMapZoomIn").onclick=function(this: any): any {offlineMap.zoomBy(1)};
  $("#offlineMapLocate").onclick=function(this: any): any {if(!offlineMap.locate())showToast("等待 Pixhawk GPS 定位","Waiting for Pixhawk GPS fix")};
  offlineMap.ready.catch(function(this: any,error?: any): any {
    $("#mapSource").textContent=localize("离线卫星地图不可用：","Offline satellite map unavailable: ")+error.message;
  });
}

function renderGpsStatus(this: any): any {
  var card: any=$("#gpsSourceCard");if(!card)return;
  var sats: any=Math.max(0,Number(state.gps.satellites)||0);
  if(!state.connected){card.dataset.state="waiting";$("#gpsStatus").textContent=localize("等待设备连接","Waiting for device");}
  else if(Number(state.gps.fixType)>=2){card.dataset.state="available";$("#gpsStatus").textContent=localize("GPS 可用","GPS available");}
  else{card.dataset.state="unavailable";$("#gpsStatus").textContent=localize("GPS 暂不可用","GPS currently unavailable");}
  var label: any=sats+localize(" 颗卫星"," satellites");
  if(state.gps.hdop!==null&&Number.isFinite(Number(state.gps.hdop)))label+=" · HDOP "+Number(state.gps.hdop).toFixed(1);
  $("#gpsDetail").textContent=label;
  var location: any=currentMapLocation();
  $("#mapCoordinates").textContent=location.fresh?location.latitude.toFixed(6)+", "+location.longitude.toFixed(6):"--, --";
  $("#mapHeading").textContent=localize("航向 ","Heading ")+(Number.isFinite(Number(state.gps.course))?Number(state.gps.course).toFixed(1)+"°":"--°");
  if(mapProvider==="offline-satellite")$("#mapSource").textContent=localize(mapSourceLabel,mapSourceLabelEn);
  updateOfflineMapPosition();
}

function signedAngle(this: any,v?: any): any {v=Number(v)||0;return(v>=0?"+":"−")+Math.abs(v).toFixed(1)+"°"}
function signedPercent(this: any,v?: any): any {return(v>0?"+":"")+v+"%"}

function showView(this: any,view?: any): any {
  if(state.downloading){showToast("下载完成前无法切换页面");return;}
  if(!state.connected&&view!=="device"){showToast("请先连接 Manta","Connect to Manta first");view="device";}
  state.view=view;
  $$(".view").forEach(function(this: any,n?: any): any {
    var active: any=n.dataset.view===view;
    n.hidden=!active;
    requestAnimationFrame(function(this: any): any {n.classList.toggle("active",active)});
  });
  $$(".nav-tab[data-nav]").forEach(function(this: any,n?: any): any {n.classList.toggle("active",n.dataset.nav===view)});
  toggleDrawer($("#toolDrawer"),false);
  if(view!=="media")pauseMediaPreview();
  if(view==="media"&&state.transportMode==="live")loadLiveRecordings();
  if(view==="map"){initializeOfflineMap();renderGpsStatus();setTimeout(function(this: any): any {if(offlineMap)offlineMap.resize()},50);}
  syncVideoSource();
}

function syncVideoSource(this: any): any {
  var video: any=$("#gimbalVideo"),img: any=$("#liveGimbalFeed"),webrtcVideo: any=$("#liveGimbalWebrtc");
  var shouldRun: any=state.connected&&state.view==="device";
  if(!shouldRun){stopLiveGimbalStream();video.pause();return;}
  if(state.transportMode==="live"){video.hidden=true;startLiveGimbalStream();}
  else{stopLiveGimbalStream({keepStatus:true});img.hidden=true;webrtcVideo.hidden=true;video.hidden=false;video.play().catch(function(this: any): any {});setGimbalVideoState("live","DEMO")}
  requestAnimationFrame(drawGimbalOverlay);
}

function gimbalWebrtcUrl(this: any): any {
  var protocol: any=location.protocol==="https:"?"https:":"http:";
  return protocol+"//"+location.hostname+":8889/gimbal/whep";
}

function setVideoTransportBadge(this: any,value?: any): any {
  var badge: any=$("#videoTransportBadge");if(badge)badge.textContent=value;
}

function setGimbalVideoState(this: any,status?: any,transport?: any): any {
  var chip: any=$("#videoLinkState"),button: any=$("#refreshGimbalVideo");
  var labels: any={
    offline:localize("未连接","Disconnected"),
    loading:localize("画面加载中","Video loading"),
    refreshing:localize("正在刷新画面","Refreshing video"),
    warning:localize("画面中断 · 重连中","Video lost · Reconnecting"),
    live:(transport||"LIVE")+" · "+localize("正常","Live")
  };
  gimbalVideoStatus=status;gimbalVideoTransport=transport||"";
  if(chip)chip.className="live-badge is-"+status;
  $(".video-feed").classList.toggle("video-interrupted",status==="warning");
  setVideoTransportBadge(labels[status]||labels.offline);
  if(button){var busy: any=status==="refreshing";button.disabled=busy;button.setAttribute("aria-busy",String(busy));}
  var alert: any=$("#videoAlert");
  if(status==="live"&&state.connected&&state.videoLost&&alert&&alert.dataset.reason==="video"){
    state.videoLost=false;alert.hidden=true;state.connection=state.emergency?"emergency":"connected";renderConnection();
  }
}

function startMjpegFallback(this: any,reason?: any): any {
  if(!state.connected||state.view!=="device"||state.transportMode!=="live")return;
  var img: any=$("#liveGimbalFeed"),webrtcVideo: any=$("#liveGimbalWebrtc");
  webrtcVideo.hidden=true;img.hidden=false;
  clearTimeout(gimbalMjpegRetryTimer);gimbalMjpegRetryTimer=null;
  var currentSource: any=img.getAttribute("src")||"";
  if(!/api\/gimbal\/stream/.test(currentSource)){
    setGimbalVideoState("warning");img.src="/api/gimbal/stream?fallback="+Date.now();
  }else if(gimbalVideoStatus!=="live"||gimbalVideoTransport!=="MJPEG")setGimbalVideoState("warning");
  if(reason&&!gimbalWebrtcFallbackLogged){gimbalWebrtcFallbackLogged=true;addLog({level:"WARNING",source:"VIDEO",message:"WebRTC 暂不可用，已切换 MJPEG 备用流",messageEn:"WebRTC unavailable; MJPEG fallback active",category:"hardware"})}
}

function startLiveGimbalStream(this: any): any {
  var img: any=$("#liveGimbalFeed"),video: any=$("#liveGimbalWebrtc");
  if(gimbalWebrtcReader)return;
  var generation: any=++gimbalStreamGeneration;
  setGimbalVideoState("loading");
  if(typeof MediaMTXWebRTCReader!=="function"||!location.hostname){startMjpegFallback("reader-unavailable");return;}
  img.hidden=true;img.removeAttribute("src");video.hidden=false;
  gimbalWebrtcFallbackTimer=setTimeout(function(this: any): any {if(generation===gimbalStreamGeneration&&!video.srcObject)startMjpegFallback("handshake-timeout")},3200);
  var readerOptions: any={
    url:gimbalWebrtcUrl(),user:"",pass:"",token:"",
    onError:function(this: any,error?: any): any {if(generation===gimbalStreamGeneration&&state.connected&&state.view==="device")startMjpegFallback(String(error||"webrtc-error"))},
    onTrack:function(this: any,event?: any): any {
      if(generation!==gimbalStreamGeneration)return;
      clearTimeout(gimbalWebrtcFallbackTimer);gimbalWebrtcFallbackTimer=null;
      var stream: any=event&&event.streams&&event.streams[0];if(!stream)return;
      gimbalWebrtcFallbackLogged=false;video.srcObject=stream;video.hidden=false;img.hidden=true;img.removeAttribute("src");setGimbalVideoState("live","WEBRTC");
      video.play().catch(function(this: any): any {});requestAnimationFrame(drawGimbalOverlay);
    }
  };
  try{gimbalWebrtcReader=new MediaMTXWebRTCReader(readerOptions)}catch(error: any){gimbalWebrtcReader=null;startMjpegFallback(error.message||"reader-start-failed")}
}

function stopLiveGimbalStream(this: any,options?: any): any {
  gimbalStreamGeneration+=1;
  clearTimeout(gimbalWebrtcFallbackTimer);gimbalWebrtcFallbackTimer=null;
  clearTimeout(gimbalMjpegRetryTimer);gimbalMjpegRetryTimer=null;
  if(gimbalWebrtcReader){try{gimbalWebrtcReader.close()}catch(_error: any){}gimbalWebrtcReader=null}
  var img: any=$("#liveGimbalFeed"),video: any=$("#liveGimbalWebrtc");
  gimbalWebrtcFallbackLogged=false;img.hidden=true;img.removeAttribute("src");video.pause();video.srcObject=null;video.hidden=true;
  if(!options||!options.keepStatus)setGimbalVideoState("offline");
}

function refreshGimbalVideoStream(this: any): any {
  if(!state.connected||state.view!=="device"){
    setGimbalVideoState("offline");
    showToast(localize("Manta 未连接，暂时无法刷新画面","Manta is disconnected; video cannot be refreshed"));
    return;
  }
  if(state.transportMode!=="live"){
    var demoVideo: any=$("#gimbalVideo");setGimbalVideoState("refreshing");demoVideo.load();
    demoVideo.play().then(function(this: any): any {setGimbalVideoState("live","DEMO")}).catch(function(this: any): any {setGimbalVideoState("warning")});
  }else{
    stopLiveGimbalStream({keepStatus:true});setGimbalVideoState("refreshing");
    setTimeout(function(this: any): any {if(state.connected&&state.view==="device"&&state.transportMode==="live")startLiveGimbalStream()},120);
  }
  showToast(localize("只刷新云台画面，控制链路保持不变","Refreshing gimbal video only; control link remains unchanged"));
}

function deviceSignalLevel(this: any,rssi?: any): any {rssi=Number(rssi);if(!Number.isFinite(rssi))return 3;if(rssi>=-50)return 4;if(rssi>=-65)return 3;if(rssi>=-78)return 2;return 1}

function deviceSignalText(this: any,level?: any): any {return level>=4?localize("信号强","Strong signal"):level>=3?localize("信号良好","Good signal"):level>=2?localize("信号一般","Fair signal"):localize("信号较弱","Weak signal")}

function setDiscoveryStatus(this: any,text?: any,active?: any): any {
  $("#discoveryStatus").textContent=text;
  var dot: any=$(".device-card-header .status-dot");if(dot)dot.classList.toggle("connected",Boolean(active));
}

function renderDevicePicker(this: any): any {
  var devices: any=state.discoveredDevices||[],list: any=$("#devicePickerList");
  $("#devicePickerSummary").textContent=devices.length?localize("发现 "+devices.length+" 台 Manta，选择一台继续","Found "+devices.length+" Manta devices. Choose one to continue."):localize("附近未发现 Manta","No nearby Manta found");
  list.innerHTML=devices.map(function(this: any,device?: any,index?: any): any {
    var level: any=deviceSignalLevel(device.rssi),name: any=escapeHtml(device.name||"MANTA"),model: any=escapeHtml(device.model||"MANTA"),serial: any=escapeHtml(device.serial||"----");
    return '<button class="device-option" type="button" data-device-index="'+index+'" data-signal="'+level+'" aria-label="'+name+' '+model+' '+serial+'"><span class="device-option-main"><strong>'+name+'</strong><span>'+model+' · '+serial+'</span></span><span class="device-signal" aria-hidden="true"><i></i><i></i><i></i><i></i></span><span class="device-option-meta"><span>'+deviceSignalText(level)+'</span><b>'+Math.round(Number(device.rssi)||-60)+' dBm</b></span></button>';
  }).join("");
}

function selectDiscoveredDevice(this: any,index?: any): any {
  var device: any=state.discoveredDevices[index];if(!device)return;
  state.selectedDevice=device;state.connection="pairing";renderConnection();
  var identity: any=(device.name||"MANTA")+" · "+(device.model||"MANTA")+" "+(device.serial||"----");
  $("#pairingDeviceIdentity").textContent=identity;
  $("#controlDeviceName").textContent=device.name||"MANTA";
  $("#controlDeviceSerial").textContent=(device.model||"MANTA")+" · "+(device.serial||"----");
  setModal("devicePickerModal",false);setModal("pairingModal",true);
  $$("#pinInputs input").forEach(function(this: any,input?: any): any {input.value=""});validatePin();
  setTimeout(function(this: any): any {var first: any=$("#pinInputs input");if(first)first.focus()},80);
}

function startConnection(this: any): any {
  if(state.connected)return;
  state.connection="discovering";state.discoveredDevices=[];state.selectedDevice=null;renderConnection();setDiscoveryStatus(localize("正在扫描","Scanning"),false);
  var btn: any=$("#connectButton");btn.disabled=true;$("span",btn).textContent=localize("正在发现附近设备","Discovering nearby device");
  var activeTransport: any=ensureTransport();
  var discovery: any=activeTransport?activeTransport.discover():Promise.reject(new Error("通信模块未加载"));
  discovery.then(function(this: any,devices?: any): any {
    devices=Array.isArray(devices)?devices:[];if(!devices.length)throw new Error(localize("附近没有发现 Manta","No nearby Manta found"));
    state.discoveredDevices=devices;state.connection="selecting";renderConnection();renderDevicePicker();setModal("devicePickerModal",true);
    setDiscoveryStatus(localize("发现 "+devices.length+" 台","Found "+devices.length),true);
    devices.forEach(function(this: any,device?: any): any {addLog({level:"INFO",source:"BLE",message:"发现 "+(device.name||"MANTA")+" · "+(device.model||"MANTA")+" "+(device.serial||"----"),messageEn:"Discovered "+(device.name||"MANTA")+" · "+(device.model||"MANTA")+" "+(device.serial||"----"),category:"hardware"})});
    btn.disabled=false;$("span",btn).textContent=localize("搜索 Manta","Find Manta");
  }).catch(function(this: any,error?: any): any {state.connection="idle";renderConnection();setDiscoveryStatus(localize("未发现设备","No devices found"),false);btn.disabled=false;$("span",btn).textContent=localize("重新扫描","Scan again");showToast(error.message||"附近没有发现 Manta","No Manta found nearby")});
}

function completePairing(this: any): any {
  if(!state.selectedDevice){
    state.connection=state.discoveredDevices.length?"selecting":"idle";renderConnection();
    setModal("pairingModal",false);
    if(state.discoveredDevices.length){renderDevicePicker();setModal("devicePickerModal",true)}
    showToast(localize("请先选择一台 Manta","Choose a Manta first"));return;
  }
  var pin: any=$$("#pinInputs input").map(function(this: any,i?: any): any {return i.value}).join("");
  var btn: any=$("#pairButton");btn.disabled=true;btn.textContent=localize("正在验证…","Verifying…");
  var activeTransport: any=ensureTransport();
  var pairing: any=activeTransport?activeTransport.pair(pin).then(function(this: any): any {state.connection="connecting";renderConnection();return activeTransport.connect()}):Promise.reject(new Error("通信模块未加载"));
  pairing.then(function(this: any): any {
    state.connected=true;state.boardOnline=true;state.controlLink=state.pixhawkOnline;state.connection=state.pixhawkOnline?"connected":"offline";state.emergency=false;state.armed=false;
    setModal("pairingModal",false);$$("#pinInputs input").forEach(function(this: any,i?: any): any {i.value=""});
    renderConnection();showView("device");
    showToast(state.pixhawkOnline?localize("Manta 已连接，互联网保持在线","Manta connected. Internet remains online."):localize("已连接 Manta 主机，等待 Pixhawk、IMU 与电机上线","Manta host connected; waiting for Pixhawk, IMU, and motors"));
    addLog(state.pixhawkOnline?{level:"INFO",source:"APP",message:"控制权已授予本机",messageEn:"Control granted to this device",category:"system"}:{level:"WARNING",source:"APP",message:"已建立板端会话，控制保持锁定直到 Pixhawk 上线",messageEn:"Board session established; control remains locked until Pixhawk is online",category:"hardware"});
  }).catch(function(this: any,error?: any): any {state.connection="pairing";renderConnection();showToast(error.message||"配对码不正确","Incorrect pairing code");btn.textContent=localize("验证并连接","Verify and Connect");validatePin()});
}

function disconnect(this: any): any {
  var activeTransport: any=transport;
  if(activeTransport&&uwbFollowIsRequested()&&activeTransport.stopUwbFollow)activeTransport.stopUwbFollow().catch(function(this: any): any {});
  if(activeTransport)activeTransport.disconnect().catch(function(this: any): any {});
  state.connected=false;state.boardOnline=false;state.pixhawkOnline=false;state.imuOnline=false;state.motorsOnline=false;state.gimbalOnline=false;state.gimbalOutputAvailable=false;state.gimbalLinkStatus="offline";state.controlLink=false;state.connection="idle";state.speed=0;
  state.driveMode="manual";if(activeTransport&&activeTransport.setDriveEnabled)activeTransport.setDriveEnabled(true);
  state.videoLost=false;state.gimbalFault=false;state.emergency=false;state.armed=false;state.discoveredDevices=[];state.selectedDevice=null;
  if(state.calibrationStarted||state.calibrating)finishImuCalibration(false,localize("设备已断开，无法确认校准结果。","The device disconnected before calibration could be confirmed."));
  stopRecording(false);renderConnection();renderTelemetry();
  showToast("已断开 Manta","Manta disconnected");
  setDiscoveryStatus(localize("等待扫描","Waiting to scan"),false);$("span",$("#connectButton")).textContent=localize("搜索 Manta","Find Manta");
  clearInterval(telemetryTimer);
}

function validatePin(this: any): any {
  var valid: any=$$("#pinInputs input").every(function(this: any,i?: any): any {return/^\d$/.test(i.value)});
  $("#pairButton").disabled=!valid;
}

function startMockTelemetry(this: any): any {
  clearInterval(telemetryTimer);
  var start: any=Date.now();
  telemetryTimer=setInterval(function(this: any): any {
    if(!state.connected)return;
    var elapsed: any=(Date.now()-start)/1000;
    state.latency=14+Math.round(Math.sin(elapsed/3)*4+Math.random()*3);
    state.speed=Math.hypot(state.vector.x,state.vector.y)*2.2;
    state.temperature=45+Math.sin(elapsed/18)*2;
    state.attitude={roll:Math.sin(elapsed/4)*2.2,pitch:Math.cos(elapsed/5)*1.4,yaw:(184+elapsed*0.7)%360};
    state.gps={satellites:9,hdop:1.2,fixType:3,estimated:true};
    renderTelemetry();renderGpsStatus();
  },500);
}

function installJoystick(this: any): any {
  var root: any=$("#joystick"),stick: any=$("#joyStick");
  var activePointer: any=null;
  function update(this: any,cx?: any,cy?: any): any {
    if(!state.connected||!state.controlLink||!state.armed||state.calibrating)return;
    var rect: any=root.getBoundingClientRect();
    var radius: any=rect.width*.34;
    var dx: any=cx-(rect.left+rect.width/2);
    var dy: any=cy-(rect.top+rect.height/2);
    var dist: any=Math.hypot(dx,dy);
    if(dist>radius){dx=dx/dist*radius;dy=dy/dist*radius}
    state.vector={x:dx/radius,y:dy/radius};
    var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.drive(state.vector);
    stick.style.transform="translate(calc(-50% + "+dx+"px), calc(-50% + "+dy+"px))";
    $("#driveOutput").textContent="X "+state.vector.x.toFixed(2)+" · Y "+(-state.vector.y).toFixed(2);
    root.setAttribute("aria-valuetext","X "+state.vector.x.toFixed(2)+", Y "+(-state.vector.y).toFixed(2));
    // Motor cards display only confirmed Pixhawk output telemetry.
  }
  function reset(this: any): any {
    activePointer=null;state.vector={x:0,y:0};
    var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.drive(state.vector);
    stick.style.transform="translate(-50%, -50%)";
    $("#driveOutput").textContent="X 0.00 · Y 0.00";
    root.setAttribute("aria-valuetext",localize("停止","Stopped"));
    setMotorOutput(1500,1500);
  }
  scope.listen(root,"pointerdown",function(this: any,e?: any): any {
    if(!state.connected||!state.controlLink||!state.armed||state.calibrating)return;
    activePointer=e.pointerId;root.setPointerCapture(activePointer);update(e.clientX,e.clientY);
  });
  scope.listen(root,"pointermove",function(this: any,e?: any): any {if(e.pointerId===activePointer)update(e.clientX,e.clientY)});
  scope.listen(root,"pointerup",reset);scope.listen(root,"pointercancel",reset);
  scope.listen(root,"lostpointercapture",reset);
  window.addEventListener("blur",reset);
  document.addEventListener("visibilitychange",function(this: any): any {if(document.hidden)reset()});
  window.resetMantaJoystick=reset;
}

function renderMotorStatus(this: any): any {
  function renderSide(this: any,side?: any,label?: any): any {
    var link: any=state.motorLinks[side]||{};
    var feedback: any=Boolean(state.connected&&link.online);
    var outputOnly: any=Boolean(state.connected&&!feedback&&link.outputOnline);
    var card: any=$("#"+side+"MotorCard");
    if(card){card.classList.toggle("connected",feedback);card.classList.toggle("output-only",outputOnly);card.classList.toggle("offline",state.connected&&!feedback&&!outputOnly);}
    var status: any=feedback?localize("反馈在线","Feedback online"):outputOnly?localize("Pixhawk 输出在线 · 无 ESC 遥测","Pixhawk output online · no ESC telemetry"):localize("离线","Offline");
    $("#"+side+"MotorLabel").textContent=localize(label,side==="left"?"Left motor":"Right motor")+" · "+status;
    var pwm: any=Number(link.outputPwm)||0;
    $("#"+side+"MotorPwm").textContent=link.outputOnline&&pwm>0?Math.round(pwm)+" μs":"-- μs";
    $("#"+side+"Motor").textContent=link.outputOnline&&pwm>0?signedPercent(Math.round((pwm-1500)/5)):"--";
  }
  renderSide("left","左电机");renderSide("right","右电机");
}

function renderHardwareStatus(this: any): any {
  var identityDot: any=$(".control-identity i");if(identityDot)identityDot.classList.toggle("offline",state.connected&&!state.pixhawkOnline);
  if($("#pixhawkMetric"))$("#pixhawkMetric").textContent=state.pixhawkOnline?"LIVE":"OFF";
  if($("#gimbalMetric"))$("#gimbalMetric").textContent=state.gimbalLinkStatus==="command_timeout"?"ERR":state.gimbalLinkStatus==="feedback"?"LIVE":state.gimbalOutputAvailable?"READY":"OFF";
  var imuOffline: any=$("#imuOffline");if(imuOffline)imuOffline.hidden=!state.connected||state.imuOnline;
  if(!state.videoLost&&!state.gimbalFault)$("#videoAlert").hidden=true;
  $$('[data-gimbal-mode],[data-gimbal-action],#gimbalOsd').forEach(function(this: any,button?: any): any {button.disabled=state.connected&&!state.gimbalOutputAvailable;if(button.id==="gimbalOsd"&&state.gimbalOsdSupported===false)button.disabled=true});
  function light(this: any,id?: any,status?: any): any {var item: any=$(id);if(item)item.dataset.state=status;}
  light("#hostLinkLight",state.connected&&state.boardOnline?"online":"offline");
  light("#pixhawkLinkLight",state.connected&&state.pixhawkOnline?"online":"offline");
  var left: any=state.motorLinks.left||{},right: any=state.motorLinks.right||{};
  var motorState: any=state.connected&&(left.online||left.outputOnline)&&(right.online||right.outputOnline)?"online":"offline";
  light("#motorLinkLight",motorState);
  light("#gimbalLinkLight",state.connected&&state.gimbalOutputAvailable&&state.gimbalLinkStatus!=="command_timeout"?"online":"offline");
}

function setMotorOutput(this: any,leftPwm?: any,rightPwm?: any): any {
  leftPwm=Math.round(clamp(leftPwm,1000,2000));rightPwm=Math.round(clamp(rightPwm,1000,2000));
  var leftCard: any=$("#leftMotorCard"),rightCard: any=$("#rightMotorCard");
  if(leftCard)leftCard.dataset.targetPwm=String(leftPwm);
  if(rightCard)rightCard.dataset.targetPwm=String(rightPwm);
  // Actual motor PWM is rendered only from Pixhawk SERVO_OUTPUT_RAW telemetry.
}

function setArmSliderProgress(this: any,slider?: any,progress?: any,animate?: any): any {
  progress=clamp(progress,0,100);
  var thumb: any=$(".arm-slider-thumb",slider),max: any=Math.max(0,slider.clientWidth-(thumb?thumb.offsetWidth:26)-6);
  slider.classList.toggle("dragging",animate===false);
  slider.style.setProperty("--arm-x",Math.round(max*progress/100)+"px");
  slider.setAttribute("aria-valuenow",String(Math.round(progress)));
  slider.dataset.progress=String(progress);
}

function renderArmControl(this: any): any {
  var slider: any=$("#armSlider"),button: any=$("#disarmButton");
  if(!slider||!button)return;
  slider.hidden=state.armed||state.driveMode==="auto";
  slider.setAttribute("aria-disabled",String(!state.connected||!state.controlLink||state.calibrating));
  if(!slider.classList.contains("dragging"))setArmSliderProgress(slider,0,true);
  button.hidden=!state.armed;
  $("#armStateLabel").textContent=state.armed?localize("已武装","Armed"):localize("已解除武装","Disarmed");
  $("#armSwitchText").textContent=localize("滑动武装","Slide to arm");
}

function armPropulsion(this: any): any {
  if(!state.connected||!state.controlLink||state.calibrating){showToast("请先连接 Manta","Connect to Manta first");renderArmControl();return;}
  var activeTransport: any=ensureTransport();if(!activeTransport||typeof activeTransport.arm!=="function"){showToast("通信模块不支持武装确认","Transport cannot confirm arming");renderArmControl();return;}
  activeTransport.drive({x:0,y:0});
  activeTransport.arm().then(function(this: any): any {
    state.armed=true;state.emergency=false;state.connection="connected";state.speed=0;
    renderConnection();renderTelemetry();showToast("推进已武装","Propulsion armed");
    addLog({level:"INFO",source:"APP",message:"推进已武装",messageEn:"Propulsion armed",category:"command"});
  }).catch(function(this: any,error?: any): any {state.armed=false;renderArmControl();showToast(error.message||"飞控未确认武装","FCU did not confirm arming")});
}

function installArmControl(this: any): any {
  var slider: any=$("#armSlider"),activePointer: any=null,startX: any=0,progress: any=0;
  function enabled(this: any): any {return slider.getAttribute("aria-disabled")!=="true"&&!state.armed}
  function finish(this: any,pointerId?: any): any {
    if(activePointer===null||pointerId!==activePointer)return;
    if(slider.hasPointerCapture&&slider.hasPointerCapture(activePointer))slider.releasePointerCapture(activePointer);
    activePointer=null;slider.classList.remove("dragging");
    if(progress>=92)armPropulsion();else setArmSliderProgress(slider,0,true);
    progress=0;
  }
  scope.listen(slider,"pointerdown",function(this: any,e?: any): any {
    if(!enabled()||(e.pointerType==="mouse"&&e.button!==0))return;
    activePointer=e.pointerId;startX=e.clientX;progress=0;slider.setPointerCapture(activePointer);slider.classList.add("dragging");setArmSliderProgress(slider,0,false);e.preventDefault();
  });
  scope.listen(slider,"pointermove",function(this: any,e?: any): any {
    if(e.pointerId!==activePointer)return;
    var thumb: any=$(".arm-slider-thumb",slider),distance: any=Math.max(1,slider.clientWidth-(thumb?thumb.offsetWidth:26)-6);
    progress=clamp((e.clientX-startX)/distance*100,0,100);setArmSliderProgress(slider,progress,false);
  });
  scope.listen(slider,"pointerup",function(this: any,e?: any): any {finish(e.pointerId)});
  scope.listen(slider,"pointercancel",function(this: any,e?: any): any {finish(e.pointerId)});
  scope.listen(slider,"keydown",function(this: any,e?: any): any {if(!enabled())return;if(e.key==="End"){e.preventDefault();setArmSliderProgress(slider,100,true);armPropulsion()}else if(e.key==="Home"){e.preventDefault();setArmSliderProgress(slider,0,true)}});
  scope.listen($("#disarmButton"),"click",function(this: any): any {setEmergency(true)});
}

function setEmergency(this: any,active?: any): any {
  if(active){var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.emergencyStop().catch(function(this: any,error?: any): any {showToast(error.message||"紧急停止指令失败","Emergency-stop command failed")})}
  state.armed=false;
  state.emergency=active;
  state.connection=active?"emergency":(state.connected?"connected":"idle");
  state.speed=0;
  if(window.resetMantaJoystick)window.resetMantaJoystick();
  renderConnection();renderTelemetry();
  if(active){showToast("推进已解除武装，电机输出已归零","Propulsion disarmed. Motor output is zero.");addLog({level:"WARNING",source:"APP",message:"推进已解除武装，电机输出已归零",messageEn:"Propulsion disarmed. Motor output is zero.",category:"command"});}
}

function renderGimbalOsd(this: any): any {
  var btn: any=$("#gimbalOsd");if(!btn)return;
  btn.hidden=state.gimbalOsdSupported===false;
  btn.classList.toggle("active",state.osdEnabled);
  btn.setAttribute("aria-pressed",String(state.osdEnabled));
  btn.setAttribute("title",state.osdEnabled?localize("隐藏云台 OSD","Hide gimbal OSD"):localize("显示云台 OSD","Show gimbal OSD"));
}

function toggleGimbalOsd(this: any): any {
  if(!gimbalControlAvailable()){showToast("云台当前不可控制","Gimbal control is currently unavailable");return}
  state.osdEnabled=!state.osdEnabled;renderGimbalOsd();
  var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.setGimbalOsd(state.osdEnabled).catch(function(this: any,error?: any): any {state.osdEnabled=!state.osdEnabled;renderGimbalOsd();showToast(error.message||"OSD 指令失败","OSD command failed")});
  showToast(state.osdEnabled?"云台 OSD 已显示":"云台 OSD 已隐藏",state.osdEnabled?"Gimbal OSD is visible":"Gimbal OSD is hidden");
}

function setCalibrationMotionLock(this: any,locked?: any): any {
  state.calibrating=Boolean(locked);
  var activeTransport: any=ensureTransport();if(activeTransport&&activeTransport.setMotionLocked)activeTransport.setMotionLocked(state.calibrating);
  shell.dataset.calibrating=String(state.calibrating);
  if(state.calibrating&&window.resetMantaJoystick)window.resetMantaJoystick();
  renderConnection();
}

var CALIBRATION_POSES: any=[
  {code:1,pose:"level",zh:"水平平放",en:"Level",instructionZh:"将 Manta 正常水平放置，保持完全静止。",instructionEn:"Place Manta level in its normal orientation and keep it completely still."},
  {code:2,pose:"left",zh:"左侧朝下",en:"Left side down",instructionZh:"将 Manta 左侧朝下放稳，确认机身不再晃动。",instructionEn:"Rest Manta on its left side and wait until it is completely stable."},
  {code:3,pose:"right",zh:"右侧朝下",en:"Right side down",instructionZh:"将 Manta 右侧朝下放稳，确认机身不再晃动。",instructionEn:"Rest Manta on its right side and wait until it is completely stable."},
  {code:4,pose:"nosedown",zh:"机头朝下",en:"Nose down",instructionZh:"将机头垂直朝下并固定，保持静止。",instructionEn:"Hold the nose vertically downward, secure the robot, and keep it still."},
  {code:5,pose:"noseup",zh:"机头朝上",en:"Nose up",instructionZh:"将机头垂直朝上并固定，保持静止。",instructionEn:"Hold the nose vertically upward, secure the robot, and keep it still."},
  {code:6,pose:"back",zh:"背面朝下",en:"Back down",instructionZh:"将 Manta 翻转，使背面朝下放稳，保持静止。",instructionEn:"Turn Manta over, rest it on its back, and keep it completely still."}
];

function calibrationPoseByCode(this: any,code?: any): any {return CALIBRATION_POSES.find(function(this: any,p?: any): any {return p.code===Number(code)})||null}

function resetCalibrationWizard(this: any): any {
  state.calibrationStarted=false;state.calibrationStepCode=null;state.calibrationStatus="intro";
  state.calibrationConfirmed=0;state.calibrationProgress=0;state.calibrationResult=null;state.calibrationResultMessage="";
}

function openImuCalibrationWizard(this: any): any {
  if(!state.connected||!state.controlLink||state.emergency){
    showToast("请先稳定连接 Manta，再开始 IMU 校准","Connect to Manta with a stable control link before IMU calibration.");
    return;
  }
  resetCalibrationWizard();renderCalibrationWizard();setModal("imuCalibrationModal",true);
}
window.openMantaImuCalibration=openImuCalibrationWizard;
document.addEventListener("click",function(this: any,event?: any): any {if(event.target&&event.target.closest&&event.target.closest("#imuCalibrate"))openImuCalibrationWizard()},true);

function closeImuCalibrationWizard(this: any): any {
  if(state.calibrating){showToast("校准已开始，请完成飞控要求的六个姿态","Calibration is active. Complete all six poses requested by the FCU.");return}
  setModal("imuCalibrationModal",false);
}

function renderCalibrationWizard(this: any): any {
  var title: any=$("#imuCalibrationTitle"),lead: any=$("#imuCalibrationLead"),progressText: any=$("#imuCalibrationProgressText");
  var counter: any=$("#imuCalibrationCounter"),bar: any=$("#imuCalibrationBar"),stage: any=$("#imuPoseStage");
  var poseTitle: any=$("#imuPoseTitle"),instruction: any=$("#imuPoseInstruction"),next: any=$("#imuCalibrationNext");
  var cancel: any=$("#imuCalibrationCancel"),close: any=$("#imuCalibrationClose"),emergency: any=$("#imuCalibrationEmergency");
  if(!title)return;
  title.textContent=localize("IMU 六面校准","Six-position IMU calibration");
  lead.textContent=localize("校准期间推进输出会被锁定。请把 Manta 放在稳定、无振动的平面上。","Thrust output is locked during calibration. Place Manta on a stable, vibration-free surface.");
  counter.textContent=Math.min(6,state.calibrationConfirmed)+" / 6";
  bar.style.width=Math.max(0,Math.min(100,Number(state.calibrationProgress)||0))+"%";
  cancel.hidden=state.calibrationStarted||Boolean(state.calibrationResult);
  close.hidden=state.calibrationStarted&&!state.calibrationResult;
  emergency.hidden=!state.calibrationStarted||Boolean(state.calibrationResult);
  next.disabled=false;
  if(state.calibrationResult){
    stage.dataset.pose=state.calibrationResult==="success"?"level":"intro";
    progressText.textContent=state.calibrationResult==="success"?localize("飞控已确认完成","FCU confirmed completion"):localize("校准未完成","Calibration incomplete");
    poseTitle.textContent=state.calibrationResult==="success"?localize("IMU 校准完成","IMU calibration complete"):localize("IMU 校准失败","IMU calibration failed");
    instruction.textContent=state.calibrationResultMessage||localize("请检查飞控状态后重试。","Check the FCU status and try again.");
    next.textContent=localize("完成","Done");
    counter.textContent=state.calibrationResult==="success"?"6 / 6":counter.textContent;
    bar.style.width=state.calibrationResult==="success"?"100%":bar.style.width;
    return;
  }
  if(!state.calibrationStarted){
    stage.dataset.pose="intro";
    progressText.textContent=localize("准备开始","Ready");
    poseTitle.textContent=localize("准备六个放置方向","Prepare six orientations");
    instruction.textContent=localize("开始后，请按照飞控要求逐面放稳并确认。","After starting, follow each FCU pose request and confirm only when Manta is stable.");
    next.textContent=localize("开始校准","Start calibration");
    return;
  }
  var pose: any=calibrationPoseByCode(state.calibrationStepCode);
  if(!pose){
    stage.dataset.pose="intro";
    progressText.textContent=localize("等待飞控姿态请求","Waiting for FCU pose request");
    poseTitle.textContent=localize("不要自行切换方向","Do not choose a pose manually");
    instruction.textContent=localize("正在准备第一个模拟姿态。","Preparing the first simulated pose.");
    next.textContent=localize("等待飞控确认","Waiting for FCU");
    next.disabled=true;
    return;
  }
  stage.dataset.pose=pose.pose;
  progressText.textContent=state.calibrationStatus==="confirming"?localize("确认命令已发送，等待飞控接受","Confirmation sent; waiting for FCU acceptance"):localize("飞控请求当前姿态","FCU requested this pose");
  poseTitle.textContent=localize(pose.zh,pose.en);
  instruction.textContent=localize(pose.instructionZh,pose.instructionEn);
  next.textContent=state.calibrationStatus==="confirming"?localize("等待飞控确认","Waiting for FCU"):localize("已放稳，确认此面","Stable — confirm this pose");
  next.disabled=state.calibrationStatus==="confirming";
}

function beginImuCalibration(this: any): any {
  if(state.calibrationStarted)return;
  state.calibrationStarted=true;state.calibrationStatus="starting";
  setCalibrationMotionLock(true);renderCalibrationWizard();
  var activeTransport: any=ensureTransport();
  var request: any=activeTransport?activeTransport.startImuCalibration("ACCEL"):Promise.reject(new Error("通信模块未加载"));
  request.then(function(this: any): any {
    state.calibrationStepCode=1;state.calibrationStatus="awaiting_position";
    addLog({level:"COMMAND",source:"PIXHAWK",message:"IMU 六面校准已启动，推进控制已锁定",messageEn:"Six-position IMU calibration started; thrust control is locked",category:"command"});
    renderCalibrationWizard();
  }).catch(function(this: any,error?: any): any {finishImuCalibration(false,error.message||localize("请检查飞控状态后重试。","Check the FCU status and try again."))});
}

function confirmCurrentImuPose(this: any): any {
  if(state.calibrationResult){closeImuCalibrationWizard();return}
  if(!state.calibrationStarted){beginImuCalibration();return}
  var pose: any=calibrationPoseByCode(state.calibrationStepCode);
  if(!pose||state.calibrationStatus==="confirming")return;
  state.calibrationStatus="confirming";renderCalibrationWizard();
  var activeTransport: any=ensureTransport();
  var request: any=activeTransport?activeTransport.confirmImuCalibration(pose.code):Promise.reject(new Error("通信模块未加载"));
  request.then(function(this: any): any {
    state.calibrationConfirmed=Math.max(state.calibrationConfirmed,CALIBRATION_POSES.indexOf(pose)+1);
    state.calibrationProgress=state.calibrationConfirmed/6*100;
    if(pose.code===6){setTimeout(function(this: any): any {finishImuCalibration(true)},500)}
    else{state.calibrationStepCode=pose.code+1;state.calibrationStatus="awaiting_position";renderCalibrationWizard()}
  }).catch(function(this: any,error?: any): any {state.calibrationStatus="awaiting_position";renderCalibrationWizard();showToast(error.message||"飞控未确认当前姿态","FCU did not confirm the current pose")});
}

function finishImuCalibration(this: any,success?: any,message?: any): any {
  state.calibrationResult=success?"success":"failed";
  state.calibrationResultMessage=message||(success?localize("飞控已确认六面校准完成。","The FCU confirmed all six calibration poses."):localize("请检查飞控状态后重试。","Check the FCU status and try again."));
  state.calibrationStarted=false;state.calibrationStepCode=null;state.calibrationStatus=success?"complete":"failed";
  if(success)state.calibrationProgress=100;
  setCalibrationMotionLock(false);
  $("#imuState").textContent=success?localize("良好","Good"):localize("需校准","Needs calibration");
  renderCalibrationWizard();
  showToast(success?"IMU 六面校准完成":"IMU 校准未完成",success?"Six-position IMU calibration complete":"IMU calibration incomplete");
}

function setRecording(this: any,active?: any): any {
  if(active===state.recording)return;
  if(active){var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.gimbal("recordStart").catch(function(this: any,error?: any): any {showToast(error.message||"录像指令失败","Recording command failed")});state.recording=true;state.recordingSince=Date.now();$("#recordTimer").hidden=false;$("#recordButton").classList.add("active");clearInterval(recordTimerHandle);recordTimerHandle=setInterval(renderRecordTimer,500);renderRecordTimer()}
  else stopRecording(true);
}

function stopRecording(this: any,sendCommand?: any): any {
  if(!state.recording&&!recordTimerHandle)return;
  if(sendCommand){var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.gimbal("recordStop").catch(function(this: any,error?: any): any {showToast(error.message||"停止录像失败","Stop-recording command failed")})}
  state.recording=false;clearInterval(recordTimerHandle);recordTimerHandle=null;
  $("#recordTimer").hidden=true;$("#recordButton").classList.remove("active");
}

function renderRecordTimer(this: any): any {
  var elapsed: any=Math.floor((Date.now()-state.recordingSince)/1000);
  $("#recordTimer").innerHTML="<i></i>REC "+String(Math.floor(elapsed/60)).padStart(2,"0")+":"+String(elapsed%60).padStart(2,"0");
}

function triggerGimbalFault(this: any,report?: any,fromMonitor?: any): any {
  if(state.gimbalFault)return;
  state.gimbalFault=true;report=report||{};
  var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.gimbal("stop").catch(function(this: any): any {});
  $("#gimbalFault").hidden=false;
  $("#gimbalFaultMessage").textContent=state.language==="en"?(report.messageEn||"A gimbal motor fault was detected. All gimbal functions are stopped."):(report.message||"检测到云台电机异常，所有云台功能已停止");
  $("#diagnosticCode").textContent=report.code||"GMB-UNKNOWN";
  $("#diagnosticCommand").textContent=report.evidence&&report.evidence.commandRate?localize("目标角速度 ","Target angular rate ")+Number(report.evidence.commandRate).toFixed(1)+"°/s":localize("运动目标已下发","Motion target was issued");
  $("#diagnosticResponse").textContent=report.evidence&&Number.isFinite(Number(report.evidence.delta))?localize("检测窗口内角度变化 ","Angle change in detection window ")+Number(report.evidence.delta).toFixed(2)+"°":(state.language==="en"?(report.messageEn||"No valid motion feedback"):(report.message||"无有效运动反馈"));
  $("#diagnosticWindows").textContent=(report.evidence&&report.evidence.windows?report.evidence.windows:3)+localize(" / 3 个检测窗口"," / 3 detection windows");
  $("#diagnosticLink").textContent=report.code==="GMB-FEEDBACK-TIMEOUT"?localize("反馈包超时","Feedback packet timed out"):localize("状态包在线 · 疑似机械卡滞","Status packets online · Suspected mechanical stall");
  $("#diagnosticAction").textContent=localize("停止跟踪、停止云台电机、保留主机移动控制","Stop tracking and gimbal motors; keep vehicle drive control");
  $$("[data-gimbal-mode], [data-gimbal-action], #gimbalOsd").forEach(function(this: any,b?: any): any {b.disabled=true});
  if(state.recording)stopRecording(true);
  addLog({level:"CRITICAL",source:"GIMBAL",message:(report.code?report.code+" · ":"")+"已触发云台停止保护",messageEn:(report.code?report.code+" · ":"")+"Gimbal stop protection triggered",category:"hardware"});
  if(fromMonitor)showToast("云台健康监测触发保护，请查看诊断报告");
}

function setVideoLost(this: any,active?: any): any {
  state.videoLost=active;
  $("#videoAlert").dataset.reason="video";
  $("#videoAlertTitle").textContent=localize("影像信号中断","Video signal lost");
  $("#videoAlertMessage").textContent=localize("点击画面任意位置刷新，控制链路保持在线","Click anywhere on the video to refresh; the control link stays online.");
  $("#videoAlert").hidden=!active;
  state.connection=active?"degraded":(state.emergency?"emergency":"connected");
  if(active)setGimbalVideoState("warning");
  else if(!state.connected)setGimbalVideoState("offline");
  else if(state.transportMode!=="live")setGimbalVideoState("live","DEMO");
  else setGimbalVideoState("loading");
  renderConnection();
  if(active)addLog({level:"WARNING",source:"VIDEO",message:"影像信号中断，控制链路保持在线",messageEn:"Video signal lost; control link remains online",category:"hardware"});
}

function renderMediaPreviewButton(this: any): any {
  var video: any=$(".featured video"),btn: any=$(".featured .media-play-btn");
  if(!video||!btn)return;
  var playing: any=!video.paused&&!video.ended;
  btn.hidden=playing;
}

function renderMediaPlayer(this: any): any {renderMediaPreviewButton();renderMediaControls()}

function selectMediaCard(this: any,card?: any,autoplay?: any): any {
  if(!card)return;
  var video: any=$(".media-grid video"),controls: any=$(".media-player-bar"),thumb: any=$(".media-thumb",card),current: any=$(".media-card.featured");
  if(!video||!controls||!thumb)return;
  var changed: any=current!==card;
  if(changed){
    video.pause();
    $$(".media-play-btn").forEach(function(this: any,button?: any): any {button.hidden=false});
    $$(".media-card").forEach(function(this: any,item?: any): any {item.classList.toggle("featured",item===card)});
    thumb.insertBefore(video,thumb.firstChild);
    thumb.appendChild(controls);
  }
  var nextUrl: any=card.dataset.mediaUrl||"assets/manta-app/gimbal-demo.mp4";
  if(video.getAttribute("src")!==nextUrl){video.src=nextUrl;video.load()}
  else if(changed){try{video.currentTime=0}catch(_: any){}}
  mediaSeeking=false;setMediaScrubberValue(0,Number.isFinite(video.duration)?video.duration:0);renderMediaPlayer();
  if(autoplay)playMediaPreview();
}

function recoverGimbal(this: any): any {
  var buttons: any=[$("#repairGimbal"),$("#repairGimbalFromReport")].filter(Boolean);buttons.forEach(function(this: any,button?: any): any {button.disabled=true;button.textContent=localize("正在检查…","Checking…")});
  var activeTransport: any=ensureTransport(),request: any=activeTransport&&activeTransport.recoverGimbal?activeTransport.recoverGimbal():Promise.reject(new Error(localize("通信模块不支持恢复","Recovery is unavailable")));
  request.then(function(this: any,payload?: any): any {
    state.gimbalFault=false;state.gimbalOnline=Boolean(payload&&payload.connected);state.gimbalOutputAvailable=Boolean(payload&&(payload.portOpen||payload.connected));state.gimbalLinkStatus=String(payload&&payload.linkStatus||"offline");state.trackingActive=false;state.gimbalMode="click";state.trackTarget=null;state.trackStatus={locked:false,status:"idle",message:"idle"};
    $("#gimbalFault").hidden=true;setModal("diagnosticModal",false);renderHardwareStatus();renderTrackingMode();addLog({level:"INFO",source:"GIMBAL",message:"云台保护已解除，等待新的控制指令",messageEn:"Gimbal protection cleared; waiting for a new command",category:"hardware"});showToast("云台已恢复，请重新选择控制模式","Gimbal recovered. Select a control mode again");
  }).catch(function(this: any,error?: any): any {showToast(error.message||"云台反馈仍未恢复","Gimbal feedback has not recovered")}).finally(function(this: any): any {buttons.forEach(function(this: any,button?: any): any {button.disabled=false;button.textContent=localize("云台修复","Recover gimbal")})});
}

function mediaTime(this: any,v?: any): any {v=Number.isFinite(Number(v))?Math.max(0,Number(v)):0;return String(Math.floor(v/60)).padStart(2,"0")+":"+String(Math.floor(v%60)).padStart(2,"0")}

function setMediaScrubberValue(this: any,value?: any,duration?: any): any {
  var scrubber: any=$("#mediaScrubber");if(!scrubber)return;
  duration=Math.max(0,Number(duration)||0);value=clamp(value,0,duration||0);
  var progress: any=duration>0?value/duration*100:0;
  scrubber.dataset.value=String(value);scrubber.dataset.max=String(duration);
  scrubber.max=String(duration);scrubber.value=String(value);
  scrubber.setAttribute("aria-valuemax",String(duration));scrubber.setAttribute("aria-valuenow",String(value));
  scrubber.setAttribute("aria-valuetext",mediaTime(value)+" / "+mediaTime(duration));
  scrubber.style.setProperty("--media-progress",progress+"%");
}

function renderMediaControls(this: any): any {
  var video: any=$(".featured video"),toggle: any=$("#mediaPlayPause"),scrubber: any=$("#mediaScrubber");
  if(!video||!toggle||!scrubber)return;
  var dur: any=Number.isFinite(video.duration)?video.duration:0;
  var cur: any=Number.isFinite(video.currentTime)?video.currentTime:0;
  var paused: any=video.paused||video.ended;
  toggle.dataset.state=paused?"paused":"playing";
  toggle.setAttribute("aria-label",paused?localize("播放录像","Play recording"):localize("暂停录像","Pause recording"));
  $(".media-icon-play",toggle).toggleAttribute("hidden",!paused);
  $(".media-icon-pause",toggle).toggleAttribute("hidden",paused);
  if(!mediaSeeking)setMediaScrubberValue(cur,dur);
  $("#mediaCurrentTime").textContent=mediaTime(cur);
  $("#mediaDuration").textContent=mediaTime(dur);
  var durationLabel: any=$(".featured .media-duration");if(durationLabel&&dur>0)durationLabel.textContent=mediaTime(dur);
  $("#mediaPlaybackRate").value=String(video.playbackRate);
}

function seekMediaFromControl(this: any): any {
  var video: any=$(".featured video"),scrubber: any=$("#mediaScrubber");
  if(!video||!scrubber||!Number.isFinite(video.duration))return;
  video.currentTime=Math.max(0,Math.min(video.duration,Number(scrubber.dataset.value)||0));
  $("#mediaCurrentTime").textContent=mediaTime(video.currentTime);
  setMediaScrubberValue(video.currentTime,video.duration);
}

function seekMediaFromPointer(this: any,event?: any): any {
  var video: any=$(".featured video"),scrubber: any=$("#mediaScrubber");
  if(!video||!scrubber||!Number.isFinite(video.duration))return;
  var rect: any=scrubber.getBoundingClientRect();
  var hasClientX: any=Number.isFinite(event.clientX)&&event.clientX>=rect.left-1&&event.clientX<=rect.right+1;
  var localX: any=hasClientX?event.clientX-rect.left:Number(event.offsetX)||0;
  setMediaScrubberValue(video.duration*clamp(localX/Math.max(1,rect.width),0,1),video.duration);
  seekMediaFromControl();
}

function pauseMediaPreview(this: any): any {
  var video: any=$(".featured video");
  if(video&&!video.paused)video.pause();
  renderMediaPlayer();
}

function playMediaPreview(this: any): any {
  var video: any=$(".featured video");
  if(!video||!video.paused)return;
  video.currentTime=Math.max(0,video.currentTime||0);
  video.play().then(function(this: any): any {renderMediaPlayer();showToast("正在预览机内录像","Previewing the on-device recording")}).catch(function(this: any,e?: any): any {renderMediaPlayer();showToast("录像无法播放："+e.message,"Unable to play recording: "+e.message)});
}

function toggleMediaPlayback(this: any): any {
  var video: any=$(".featured video");
  if(!video)return;
  if(video.paused||video.ended)playMediaPreview();else video.pause();
  renderMediaPlayer();
}

function recordingSize(this: any,bytes?: any): any {
  bytes=Number(bytes)||0;if(bytes>=1073741824)return(bytes/1073741824).toFixed(1)+" GB";if(bytes>=1048576)return(bytes/1048576).toFixed(1)+" MB";return Math.max(1,Math.round(bytes/1024))+" KB";
}

function loadLiveRecordings(this: any): any {
  var activeTransport: any=ensureTransport();if(!activeTransport||typeof activeTransport.getRecordings!=="function")return;
  activeTransport.getRecordings().then(function(this: any,items?: any): any {
    liveRecordings=Array.isArray(items)?items:[];var grid: any=$(".media-grid"),cards: any=$$(".media-grid .media-card"),now: any=new Date(),selectedCard: any=$(".media-card.featured"),selectedName: any=selectedCard?selectedCard.dataset.recording||"":"";
    while(cards.length<liveRecordings.length){
      var clone: any=cards[cards.length-1].cloneNode(true);clone.classList.remove("featured");clone.hidden=false;
      var cloneVideo: any=$("video",clone),cloneControls: any=$(".media-player-bar",clone);if(cloneVideo)cloneVideo.remove();if(cloneControls)cloneControls.remove();
      grid.insertBefore(clone,$("#mediaEmptyState"));cards.push(clone);
    }
    cards.forEach(function(this: any,card?: any,index?: any): any {
      var item: any=liveRecordings[index];card.dataset.available=String(Boolean(item));if(!item){card.hidden=true;return}
      var modified: any=new Date(item.modifiedAt||item.createdAt||Date.now()),sameDay: any=modified.getFullYear()===now.getFullYear()&&modified.getMonth()===now.getMonth()&&modified.getDate()===now.getDate();
      card.dataset.recording=item.name||"";card.dataset.mediaUrl=item.relativeUrl||item.url||"";card.dataset.today=String(sameDay);card.dataset.downloaded="false";
      var title: any=$(".media-info-left strong",card),date: any=$(".media-info-left small",card),size: any=$(".media-info-right b",card),button: any=$(".media-download-btn",card);
      if(title)title.textContent=item.title||item.name||localize("未命名录像","Untitled recording");
      if(date)date.textContent=modified.toLocaleDateString(state.language==="en"?"en-CA":"zh-CN").replace(/\//g,".")+" · "+modified.toLocaleTimeString([], {hour:"2-digit",minute:"2-digit"});
      if(size)size.textContent=recordingSize(item.size);if(button){button.textContent=localize("下载","Download");button.classList.remove("downloaded")}
    });
    var selected: any=cards.find(function(this: any,card?: any): any {return card.dataset.recording===selectedName&&card.dataset.available!=="false"})||cards.find(function(this: any,card?: any): any {return card.dataset.available!=="false"});
    if(selected)selectMediaCard(selected,false);
    renderMediaFilter();
  }).catch(function(this: any,error?: any): any {showToast(error.message||"录像列表加载失败","Unable to load recordings")});
}

function renderMediaFilter(this: any): any {
  var cards: any=$$(".media-grid .media-card"),visible: any=0;
  cards.forEach(function(this: any,card?: any): any {
    var available: any=card.dataset.available!=="false";
    var show: any=available&&(mediaFilter==="all"||(mediaFilter==="today"&&card.dataset.today==="true")||(mediaFilter==="downloaded"&&card.dataset.downloaded==="true"));
    card.hidden=!show;if(show)visible++;
  });
  var active: any=$(".media-card.featured");if(active&&active.hidden){var firstVisible: any=cards.find(function(this: any,card?: any): any {return!card.hidden});if(firstVisible)selectMediaCard(firstVisible,false);else pauseMediaPreview()}
  $$("[data-media-filter]").forEach(function(this: any,button?: any): any {button.classList.toggle("active",button.dataset.mediaFilter===mediaFilter)});
  var empty: any=$("#mediaEmptyState");
  if(!empty){empty=document.createElement("div");empty.id="mediaEmptyState";empty.className="media-empty";$(".media-grid").appendChild(empty)}
  empty.textContent=mediaFilter==="downloaded"?localize("还没有已下载的录像","No downloaded recordings yet"):localize("今天没有录像","No recordings today");
  empty.hidden=visible!==0;
  $(".media-count").textContent=localize(visible+" 个项目",visible+" item"+(visible===1?"":"s"));
}

function startDownload(this: any,btn?: any): any {
  if(!state.connected&&state.transportMode==="live"){showToast("请先连接 Manta");return}
  if(state.transportMode==="live"){
    var card: any=btn.closest(".media-card"),item: any=liveRecordings.find(function(this: any,recording?: any): any {return recording.name===card.dataset.recording});
    if(!item){showToast("没有找到对应录像","Recording not found");return}
    state.downloading=true;setModal("downloadModal",true);$("#downloadBar").style.width="12%";$("#downloadPercent").textContent="12%";$("#downloadTitle").textContent=localize("正在从 Manta 下载","Downloading from Manta");$("#cancelDownload").disabled=true;
    fetch(item.relativeDownloadUrl||item.downloadUrl).then(function(this: any,response?: any): any {if(!response.ok)throw new Error("HTTP "+response.status);return response.blob()}).then(function(this: any,blob?: any): any {
      var url: any=URL.createObjectURL(blob),link: any=document.createElement("a");link.href=url;link.download=item.name||"manta-recording.mp4";document.body.appendChild(link);link.click();link.remove();setTimeout(function(this: any): any {URL.revokeObjectURL(url)},30000);
      state.downloading=false;$("#downloadBar").style.width="100%";$("#downloadPercent").textContent="100%";$("#downloadSize").textContent=recordingSize(blob.size);$("#downloadTitle").textContent=localize("下载完成","Download complete");$("#cancelDownload").disabled=false;$("#cancelDownload").textContent=localize("完成","Done");btn.textContent=localize("已下载","Downloaded");btn.classList.add("downloaded");card.dataset.downloaded="true";renderMediaFilter();
    }).catch(function(this: any,error?: any): any {state.downloading=false;$("#downloadTitle").textContent=localize("下载失败","Download failed");$("#cancelDownload").disabled=false;$("#cancelDownload").textContent=localize("关闭","Close");showToast(error.message||"录像下载失败","Recording download failed")});
    return;
  }
  state.downloading=true;setModal("downloadModal",true);
  $("#downloadBar").style.width="0%";$("#downloadPercent").textContent="0%";
  $("#downloadSize").textContent="0 MB / 19.4 MB";
  $("#downloadTitle").textContent=localize("正在下载到 Manta App","Downloading to Manta App");
  $("#cancelDownload").disabled=true;$("#cancelDownload").textContent=localize("下载期间无法切换页面","Navigation is locked during download");
  var progress: any=0;
  clearInterval(downloadTimer);
  downloadTimer=setInterval(function(this: any): any {
    progress=Math.min(100,progress+2+Math.random()*5);
    $("#downloadBar").style.width=progress+"%";
    $("#downloadPercent").textContent=Math.floor(progress)+"%";
    $("#downloadSize").textContent=(19.4*progress/100).toFixed(1)+" MB / 19.4 MB";
    if(progress>=100){
      clearInterval(downloadTimer);state.downloading=false;
      $("#downloadTitle").textContent=localize("下载完成","Download complete");
      $("#cancelDownload").disabled=false;$("#cancelDownload").textContent=localize("完成","Done");
      btn.textContent=localize("保存到相册","Save to Photos");btn.classList.add("downloaded");
      var card: any=btn.closest(".media-card");if(card)card.dataset.downloaded="true";
      renderMediaFilter();
      $("#mediaBadge").hidden=false;
      addLog({level:"INFO",source:"MEDIA",message:"录像已保存到 App 素材库",messageEn:"Recording saved to the app media library",category:"system"});
    }
  },180);
}

function resetDemoFaults(this: any): any {
  if(state.connected){state.boardOnline=true;state.pixhawkOnline=true;state.imuOnline=true;state.motorsOnline=true;state.gimbalOnline=true;state.gimbalOutputAvailable=true;state.gimbalLinkStatus="feedback"}
  state.controlLink=state.connected;state.videoLost=false;state.gimbalFault=false;state.emergency=false;state.armed=false;state.temperature=46;state.latency=18;
  state.connection=state.connected?"connected":"idle";
  $("#videoAlert").hidden=true;$("#gimbalFault").hidden=true;
  $$("[data-gimbal-mode], [data-gimbal-action], #gimbalOsd").forEach(function(this: any,b?: any): any {b.disabled=false});
  renderGimbalOsd();
  renderArmControl();
}

function applyDemoState(this: any,name?: any): any {
  if(!state.connected){state.connected=true;state.boardOnline=true;state.pixhawkOnline=true;state.imuOnline=true;state.motorsOnline=true;state.gimbalOnline=true;state.gimbalOutputAvailable=true;state.gimbalLinkStatus="feedback";state.connection="connected";state.controlLink=true;startMockTelemetry()}
  resetDemoFaults();
  if(name==="videoLost")setVideoLost(true);
  if(name==="gimbalFault")triggerGimbalFault({code:"GMB-MOTION-STALL",message:"连续三个检测窗口内角度与陀螺仪无响应",messageEn:"No angle or gyroscope response across three consecutive detection windows"},false);
  if(name==="hot"){state.temperature=86;addLog({level:"WARNING",source:"PIXHAWK",message:"飞控温度进入警戒区间",messageEn:"Flight-controller temperature entered the warning range",category:"hardware"})}
  if(name==="emergency")setEmergency(true);
  if(name==="offline"){state.boardOnline=false;state.pixhawkOnline=false;state.imuOnline=false;state.motorsOnline=false;state.gimbalOnline=false;state.gimbalOutputAvailable=false;state.gimbalLinkStatus="offline";state.controlLink=false;state.connection="offline";state.speed=0;state.videoLost=true;$("#videoAlertTitle").textContent=localize("设备连接已断开","Device connection lost");$("#videoAlertMessage").textContent=localize("控制与遥测已锁定，等待板端看门狗确认停车","Control and telemetry are locked while the board watchdog confirms stop.");$("#videoAlert").hidden=false;if(window.resetMantaJoystick)window.resetMantaJoystick();addLog({level:"ERROR",source:"LINK",message:"控制和遥测链路已断开，本机控制已锁定",messageEn:"Control and telemetry links were lost; local control is locked",category:"hardware"})}
  renderConnection();renderTelemetry();
  if(state.connected)showView("device");
}

function applyLanguage(this: any,lang?: any): any {
  state.language=lang;document.body.dataset.language=lang;document.documentElement.lang=lang==="zh"?"zh-CN":"en";
  translateStaticUi(lang);
  if(state.discoveredDevices.length){
    renderDevicePicker();
    setDiscoveryStatus(localize("发现 "+state.discoveredDevices.length+" 台","Found "+state.discoveredDevices.length),true);
  }else if(state.connection==="discovering")setDiscoveryStatus(localize("正在扫描","Scanning"),false);
  else setDiscoveryStatus(localize("等待扫描","Waiting to scan"),false);
  renderConnection();renderTelemetry();renderGpsStatus();renderGimbalOsd();renderTrackingMode();renderCalibrationWizard();renderLogs();renderMediaFilter();renderTransportMode();
}

function clamp(this: any,v?: any,min?: any,max?: any): any {return Math.max(min,Math.min(max,Number(v)||0))}

function installEvents(this: any): any {
  scope.listen($("#connectButton"),"click",startConnection);
  scope.listen($("#rescanDevices"),"click",function(this: any): any {setModal("devicePickerModal",false);startConnection()});
  scope.listen($("#devicePickerList"),"click",function(this: any,event?: any): any {var option: any=event.target.closest("[data-device-index]");if(option)selectDiscoveredDevice(Number(option.dataset.deviceIndex))});
  scope.listen($("#pairButton"),"click",completePairing);
  scope.listen($("#disconnectButton"),"click",disconnect);
  scope.listen($("#manualFollowButton"),"click",function(this: any): any {setDriveMode("manual")});
  scope.listen($("#autoFollowButton"),"click",function(this: any): any {setDriveMode("auto")});
  $$("#pinInputs input").forEach(function(this: any,input?: any,index?: any,inputs?: any): any {
    scope.listen(input,"input",function(this: any): any {
      input.value=input.value.replace(/\D/g,"").slice(-1);
      if(input.value&&inputs[index+1])inputs[index+1].focus();
      validatePin();
    });
    scope.listen(input,"keydown",function(this: any,e?: any): any {if(e.key==="Backspace"&&!input.value&&inputs[index-1])inputs[index-1].focus()});
  });
  $$("[data-close-modal]").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any {
    if(b.dataset.closeModal==="pairingModal"){$$("#pinInputs input").forEach(function(this: any,i?: any): any {i.value=""});validatePin();state.connection=state.discoveredDevices.length?"selecting":"idle";renderConnection();setModal("devicePickerModal",Boolean(state.discoveredDevices.length))}
    if(b.dataset.closeModal==="devicePickerModal"){state.connection="idle";renderConnection()}
    setModal(b.dataset.closeModal,false)
  })});
  $$("[data-nav]").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any {showView(b.dataset.nav)})});
  scope.listen($("#brandBtn"),"click",function(this: any): any {showView("device")});
  scope.listen($("#toolToggle"),"click",function(this: any): any {toggleDrawer($("#toolDrawer"),!$("#toolDrawer").classList.contains("open"));toggleDrawer($("#demoConsole"),false)});
  scope.listen($("#toolClose"),"click",function(this: any): any {toggleDrawer($("#toolDrawer"),false)});
  scope.listen($("#demoToggle"),"click",function(this: any): any {toggleDrawer($("#demoConsole"),!$("#demoConsole").classList.contains("open"));toggleDrawer($("#toolDrawer"),false)});
  scope.listen($("#demoClose"),"click",function(this: any): any {toggleDrawer($("#demoConsole"),false)});
  scope.listen($("#layoutToggle"),"click",function(this: any): any {
    state.layout=state.layout==="standard"?"swapped":"standard";
    shell.dataset.layout=state.layout;
    $("#quadrantGrid").classList.toggle("swapped",state.layout==="swapped");
    showToast(state.layout==="standard"?localize("控制区已移至左侧","Controls moved to the left"):localize("影像区已移至左侧","Video moved to the left"));
  });
  scope.listen($("#themeSelect"),"change",function(this: any,e?: any): any {
    state.theme=e.target.value;document.body.dataset.theme=state.theme;
    var meta: any=$("meta[name='theme-color']");if(meta)meta.content=state.theme==="light"?"#edf3f6":"#0a1017";
  });
  scope.listen($("#languageSelect"),"change",function(this: any,e?: any): any {applyLanguage(e.target.value)});
  scope.listen($("#transportToggle"),"click",function(this: any): any {
    if(state.connected){showToast("请先断开 Manta 再切换通信模式","Disconnect Manta before changing transport mode");return}
    state.transportMode=state.transportMode==="mock"?"live":"mock";
    ensureTransport();
    renderTransportMode();
    showToast(state.transportMode==="live"?localize("已切换真实通信，请连接 Manta","Live transport selected. Connect to Manta."):localize("已切换 Preview 模拟通信","Preview simulation selected"));
  });


  $$("[data-demo-state]").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any {
    $$("[data-demo-state]").forEach(function(this: any,x?: any): any {x.classList.remove("active")});
    b.classList.add("active");applyDemoState(b.dataset.demoState)
  })});
  ["Latency","Speed","Temp"].forEach(function(this: any,name?: any): any {
    var input: any=$("#demo"+name),output: any=$("#demo"+name+"Output");
    scope.listen(input,"input",function(this: any): any {
      var v: any=Number(input.value);
      if(name==="Latency"){state.latency=v;output.textContent=v+" ms"}
      if(name==="Speed"){state.speed=v/10;output.textContent=state.speed.toFixed(1)+" m/s"}
      if(name==="Temp"){state.temperature=v;output.textContent=v+" °C"}
      renderTelemetry()
    })
  });
  $$("[data-gimbal-mode]").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any {
if(!gimbalControlAvailable()){showToast("云台当前不可控制","Gimbal control is currently unavailable");return}
    if(gimbalModeBusy)return;
    var requestedMode: any=b.dataset.gimbalMode;
    if(state.trackingActive&&state.runningTrackMode===requestedMode){state.gimbalMode=requestedMode;state.trackMode=requestedMode;renderTrackingMode();showToast("当前跟踪模式已在运行","This tracking mode is already active");return}
    if(requestedMode==="click"&&!state.trackingActive){state.clickMarker=null;state.gimbalMode="click";state.trackTarget=null;renderTrackingMode();showToast("点击居中","Click center");return}
    var previousMode: any=state.gimbalMode,previousTracking: any=state.trackingActive,previousTrackMode: any=state.trackMode;
    state.clickMarker=null;state.gimbalMode=requestedMode;state.trackTarget=null;if(requestedMode!=="click"){state.trackMode=requestedMode;state.trackStatus={locked:false,status:state.trackingActive?"switching":"starting",mode:requestedMode,message:(state.trackingActive?"Switching to ":"Starting ")+(requestedMode==="beacon"?"UWB Beacon":requestedMode==="swimmer"?"Swimmer":"Face")+"..."}}
    gimbalModeBusy=true;$$("[data-gimbal-mode]").forEach(function(this: any,button?: any): any {button.disabled=true});
    var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.gimbal(requestedMode).catch(function(this: any,error?: any): any {state.gimbalMode=previousMode;state.trackingActive=previousTracking;state.trackMode=previousTrackMode;showToast(error.message||"云台模式指令失败","Gimbal mode command failed")}).finally(function(this: any): any {gimbalModeBusy=false;if(activeTransport&&activeTransport.refreshGimbalState)activeTransport.refreshGimbalState();$$("[data-gimbal-mode]").forEach(function(this: any,button?: any): any {button.disabled=!state.gimbalOutputAvailable});renderTrackingMode()});
    renderTrackingMode();
    showToast(requestedMode==="click"?localize("点击居中","Click center"):requestedMode==="beacon"?localize("UWB信标跟随","UWB Beacon tracking"):requestedMode==="face"?localize("人脸跟踪","Face track"):localize("泳者跟踪","Swimmer track"));
  })});
  scope.listen($(".video-feed"),"click",handleGimbalVideoClick);
  scope.listen($("#gimbalVideo"),"loadedmetadata",function(this: any): any {drawGimbalOverlay();if(state.connected&&state.transportMode!=="live")setGimbalVideoState("live","DEMO")});
  scope.listen($("#liveGimbalWebrtc"),"loadedmetadata",function(this: any): any {drawGimbalOverlay();if(state.connected&&state.transportMode==="live")setGimbalVideoState("live","WEBRTC")});
  scope.listen($("#liveGimbalWebrtc"),"timeupdate",function(this: any): any {var video: any=$("#liveGimbalWebrtc");if(state.connected&&state.transportMode==="live"&&!video.hidden&&video.readyState>=2&&(state.videoLost||gimbalVideoStatus!=="live"||gimbalVideoTransport!=="WEBRTC"))setGimbalVideoState("live","WEBRTC")});
  scope.listen($("#liveGimbalFeed"),"load",function(this: any): any {clearTimeout(gimbalMjpegRetryTimer);gimbalMjpegRetryTimer=null;setGimbalVideoState("live","MJPEG");drawGimbalOverlay()});
  scope.listen($("#liveGimbalFeed"),"error",function(this: any): any {
    if(!state.connected||state.view!=="device"||state.transportMode!=="live")return;
    setGimbalVideoState("warning");clearTimeout(gimbalMjpegRetryTimer);
    gimbalMjpegRetryTimer=setTimeout(function(this: any): any {var image: any=$("#liveGimbalFeed");image.removeAttribute("src");startMjpegFallback("mjpeg-error")},900);
  });
  scope.listen($("#refreshGimbalVideo"),"click",refreshGimbalVideoStream);
  scope.listen($("#videoAlert"),"click",function(this: any,event?: any): any {if(isGimbalVideoInterrupted()){event.preventDefault();event.stopPropagation();refreshGimbalVideoStream()}});
  scope.listen($("#videoAlert"),"keydown",function(this: any,event?: any): any {if((event.key==="Enter"||event.key===" ")&&isGimbalVideoInterrupted()){event.preventDefault();refreshGimbalVideoStream()}});
  scope.listen($("#gimbalBeaconCalibration"),"click",openBeaconCalibration);
  scope.listen($("#driveFollowSwitch"),"click",function(this: any): any {
    if(state.driveMode!=="auto")return;
    var activeTransport: any=ensureTransport();if(!activeTransport)return;
    var followStatus: any=String(state.uwbFollow&&state.uwbFollow.status||"");
    var starting: any=uwbFollowIsStarting();
    if(uwbFollowIsRequested()){
      activeTransport.stopUwbFollow().catch(function(this: any,error?: any): any {showToast(error.message||"停止自动跟随失败")});return;
    }
    if(!state.connected||!state.controlLink){showToast("Manta控制链路未连接");return}
    activeTransport.startUwbFollow().then(function(this: any,result?: any): any {if(result&&(result.searching||result.state&&result.state.autoResumePending)){showToast(localize("正在搜索 UWB；信号稳定后自动跟随","Searching for UWB; following resumes after signal confirmation"));return}showToast(Number(state.uwb.distanceM)<1.2?"自动跟随已开启；目标小于1.2米，推进保持停止":"已按定位状态选择控制模式并武装，UWB跟随已启动")}).catch(function(this: any,error?: any): any {showToast(error.message||"启动自动跟随失败")});
  });
  $$("[data-tracking-method]").forEach(function(this: any,methodButton?: any): any {scope.listen(methodButton,"click",function(this: any): any {
    var method: any=methodButton.dataset.trackingMethod;
    var follow: any=state.uwbFollow||{};
    if(uwbFollowIsRequested()){showToast(localize("请先停止跟随，再切换控制方法","Stop following before switching the controller"));return}
    var activeTransport: any=ensureTransport();
    if(!activeTransport||!activeTransport.setUwbFollowMethod){showToast(localize("当前控制链路不支持方法切换","Controller switching is unavailable"));return}
    $$("[data-tracking-method]").forEach(function(this: any,button?: any): any {button.disabled=true});
    activeTransport.setUwbFollowMethod(method).then(function(this: any,result?: any): any {
      if(result&&result.state)state.uwbFollow=result.state;
      renderUwbFollow();
      showToast(method==="neural"?localize("已切换为神经网络跟踪","Neural-network tracking selected"):localize("已切换为传统方法跟踪","Traditional tracking selected"));
    }).catch(function(this: any,error?: any): any {showToast(error.message||localize("控制方法切换失败","Controller switch failed"));renderUwbFollow()});
  })});
  scope.listen($("#beaconCalibrationNext"),"click",advanceBeaconCalibration);
  scope.listen($("#beaconCalibrationCancel"),"click",cancelBeaconCalibration);
  scope.listen($("#beaconCalibrationClose"),"click",cancelBeaconCalibration);
  window.addEventListener("resize",drawGimbalOverlay);


  $$("[data-gimbal-action]").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any {
if(!gimbalControlAvailable()){showToast("云台当前不可控制","Gimbal control is currently unavailable");return}
var action: any=b.dataset.gimbalAction;
if(action==="record"){setRecording(!state.recording)}
else{var activeTransport: any=ensureTransport();if(activeTransport)activeTransport.gimbal(action).catch(function(this: any,error?: any): any {showToast(error.message||"云台指令失败","Gimbal command failed")});showToast(action==="home"?localize("云台正在回中","Gimbal is centering"):localize("云台已停止","Gimbal stopped"))}
})});
  scope.listen($("#openReport"),"click",function(this: any): any {setModal("diagnosticModal",true)});
  scope.listen($("#repairGimbal"),"click",recoverGimbal);
  scope.listen($("#repairGimbalFromReport"),"click",recoverGimbal);


  scope.listen($("#gimbalOsd"),"click",toggleGimbalOsd);


  scope.listen($("#imuCalibrationClose"),"click",closeImuCalibrationWizard);
  scope.listen($("#imuCalibrationCancel"),"click",closeImuCalibrationWizard);


  scope.listen($("#imuCalibrationNext"),"click",confirmCurrentImuPose);
  var mediaScrubber: any=$("#mediaScrubber");
  scope.listen(mediaScrubber,"pointerdown",function(this: any): any {mediaSeeking=true});
  scope.listen(mediaScrubber,"input",function(this: any): any {
    var video: any=$(".featured video");if(!video||!Number.isFinite(video.duration))return;
    var next: any=clamp(Number(mediaScrubber.value)||0,0,video.duration);
    mediaScrubber.dataset.value=String(next);video.currentTime=next;
    $("#mediaCurrentTime").textContent=mediaTime(next);
  });
  scope.listen(mediaScrubber,"change",function(this: any): any {mediaSeeking=false;seekMediaFromControl();renderMediaPlayer()});
  scope.listen(mediaScrubber,"pointerup",function(this: any): any {mediaSeeking=false;seekMediaFromControl();renderMediaPlayer()});
  scope.listen(mediaScrubber,"pointercancel",function(this: any): any {mediaSeeking=false;renderMediaPlayer()});
  $$("[data-media-filter]").forEach(function(this: any,button?: any): any {scope.listen(button,"click",function(this: any): any {mediaFilter=button.dataset.mediaFilter;renderMediaFilter()})});
  scope.listen($(".media-grid"),"click",function(this: any,event?: any): any {
    var playButton: any=event.target.closest(".media-play-btn");if(playButton){selectMediaCard(playButton.closest(".media-card"),true);return}
    var downloadButton: any=event.target.closest(".media-download-btn");if(downloadButton){if(downloadButton.classList.contains("downloaded"))showToast("将由原生 App 请求相册权限后保存","The native app will request Photos access before saving");else startDownload(downloadButton)}
  });
scope.listen($("#imuCalibrationEmergency"),"click",function(this: any): any {
finishImuCalibration(false,localize("用户在校准期间执行了紧急停止。","Emergency stop was executed during calibration."));
setEmergency(true);setModal("imuCalibrationModal",false);
});
  $$(".log-filter-btn").forEach(function(this: any,b?: any): any {scope.listen(b,"click",function(this: any): any { state.logFilter=b.dataset.logFilter; $$(".log-filter-btn").forEach(function(this: any,x?: any): any {x.classList.toggle("active",x===b)}); renderLogs() })});
  scope.listen($("#cancelDownload"),"click",function(this: any): any { if(!state.downloading){setModal("downloadModal",false);$("#cancelDownload").disabled=true;$("#cancelDownload").textContent=localize("下载期间无法切换页面","Navigation is locked during download")} });
  scope.listen($("#mediaPlayPause"),"click",toggleMediaPlayback);
  scope.listen($("#mediaPlaybackRate"),"change",function(this: any,e?: any): any { var video: any=$(".featured video"); if(video)video.playbackRate=Number(e.target.value)||1; renderMediaPlayer(); showToast("已切换至 "+e.target.value+" 倍播放","Playback speed set to "+e.target.value+"×") });
  ["loadedmetadata","durationchange","timeupdate","ratechange","play","pause","seeking","seeked","ended"].forEach(function(this: any,evt?: any): any { var v: any=$(".media-grid video");if(v)scope.listen(v,evt,renderMediaPlayer) });
  document.addEventListener("keydown",function(this: any,e?: any): any {if(e.key==="Escape"){toggleDrawer($("#toolDrawer"),false);toggleDrawer($("#demoConsole"),false)}});
}
function seedLogs(this: any): any { [ {source:"APP",message:"Manta App Preview 已启动",messageEn:"Manta App Preview started",category:"system"}, {source:"BLE",message:"近距离发现模块待机",messageEn:"Nearby discovery is standing by",category:"hardware"}, {source:"PIXHAWK",message:"等待设备连接",messageEn:"Waiting for device connection",category:"hardware"}, {source:"GIMBAL",message:"测试视频已载入",messageEn:"Test video loaded",category:"hardware"} ].forEach(addLog) }
installEvents();installJoystick();installArmControl();seedLogs();renderTransportMode();renderConnection();renderTelemetry();renderGpsStatus();renderGimbalOsd();renderCalibrationWizard();renderMediaPlayer();renderMediaFilter();showView("device");window.MantaPreview={state:state,showView:showView,applyDemoState:applyDemoState,applyLanguage:applyLanguage,openImuCalibrationWizard:openImuCalibrationWizard};

return () => {
  if (window.resetMantaJoystick) window.resetMantaJoystick();
  stopLiveGimbalStream({keepStatus:true});
  if (offlineMap) offlineMap.destroy();
  if (transport) transport.disconnect().catch(function(){});
  scope.dispose();
};
}
