# RK3588 Manta 控制系统

本目录包含 Manta 机器人在 RK3588 上运行的完整上位机：MAVLink 桥接、云台控制、视觉跟踪、摄像头代理、录像管理、Web 控制台、Wi-Fi 热点和蓝牙 PAN。

## 1. 运行架构

```text
手机 / 平板 / VSCode
        |
        | HTTP + Socket.IO :3000
        v
backend/server.js
  |-- UDP :14551/:14552 <-> backend/mavlink_bridge.py <-> /dev/ttyS1 <-> Pixhawk TELEM2
  |-- UART /dev/ttyS3 <-> 云台控制器
  |-- HTTP 127.0.0.1:8091 <-> 云台 RTSP 代理
  |-- HTTP 127.0.0.1:8090 <-> OV8858 本地摄像头
  |-- face_track.py / infer_video.py <-> 人脸与泳者跟踪
  `-- recordings/gimbal <-> H.264 MP4 录像
```

详细的数据流与组件职责见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 2. 硬件接线

### Pixhawk TELEM2

| RK3588 物理针脚 | 功能 | Pixhawk |
| --- | --- | --- |
| 8 | UART1 TX | TELEM2 RX |
| 10 | UART1 RX | TELEM2 TX |
| 6 | GND | TELEM2 GND |

### 云台 UART

| RK3588 物理针脚 | 功能 | 云台 |
| --- | --- | --- |
| 5 | UART3 TX | RX |
| 3 | UART3 RX | TX |
| 9 | GND | GND |

TX/RX 必须交叉连接，所有设备必须共地。当前串口配置为：

- Pixhawk：`/dev/ttyS1 @ 115200`
- 云台：`/dev/ttyS3 @ 115200`

新板安装器会按主配置启用 `rk3588-lubancat-uart1-m1-overlay`（Pixhawk UART1）和 `rk3588-lubancat-uart3-m0-overlay`（云台 UART3）。接口定义与原板保持一致；overlay 写入后需手动重启才会生成对应设备节点。

## 3. 安装

要求兼容 LubanCat RK3588 的 Ubuntu 22.04/24.04 aarch64 系统和 root 权限。安装器会配置 Node.js 20+、项目 Python 虚拟环境、FFmpeg、NetworkManager 与 MediaMTX。此仓库安装应用层，不能替代厂家系统镜像烧录。

```bash
cd /root/Pixhawk_RK3588/rk3588_control_system
bash quickstart.sh
```

安装脚本会验证配置、安装依赖和九个 systemd 服务，并为每块板生成独立的蓝牙凭据到 `/etc/manta/manta.env`。当前主配置将 `Manta-Control` 设为开放热点，无需 Wi-Fi 密码；如改回 WPA2，可继续使用环境文件中的设备独立密码。UART/摄像头 overlay 首次变更后需要由操作人员确认并手动重启板子；脚本不会启动 MANTA 服务、重启服务或自动重启板子。无硬件副作用检查使用 `bash quickstart.sh --check-only`；暂不改 overlay 使用 `--skip-boot-config`。

## 4. 启停与状态

```bash
bash start.sh
bash scripts/status_report.sh
sudo systemctl status manta-backend.service
sudo journalctl -u manta-backend.service -f
```

`bash stop.sh` 默认只停止控制与媒体服务，保留 Wi-Fi/蓝牙维护通道；`bash stop.sh --all` 才会同时停止无线连接。

完整运维流程见 [docs/OPERATIONS.md](docs/OPERATIONS.md)。

## 5. 访问方式

| 连接方式 | 地址 |
| --- | --- |
| 板端本机 | `http://127.0.0.1:3000` |
| Manta Wi-Fi | `http://10.42.0.1:3000` |
| 蓝牙 PAN | `http://10.43.0.1:3000` |
| SSH/VSCode | SSH Host `manta` |

手机无需密码即可连接 `Manta-Control`。系统连通性探测会返回成功，避免 iPhone/iPad 将热点标记为受限并中断下载；请在 Safari 中打开 `http://10.42.0.1:3000`。开放热点不提供链路加密，只应在可信、近距离环境中使用。

前端提供设备状态、控制、视频、云台和 GPS 地图界面，可由手机或电脑通过 MANTA 网络访问。GPS 位置来自 Pixhawk，地图支持板载离线影像。

## 6. 云台视频与录像

- 内部识别流：原始分辨率 MJPEG，仅供跟踪算法使用。
- 手机预览流：`960x540 @ 12fps`，降低 2.4GHz 热点带宽压力。
- 录像文件：RK3588 硬件 H.264，保持 RTSP 源分辨率，不做放大伪 4K。
- 下载接口支持 `Content-Length`、Range 请求和附件响应。

真实 4K 录像要求云台 RTSP 输入本身达到至少 `3840x2160`。可使用：

```bash
GIMBAL_REQUIRE_4K=1 bash scripts/probe_gimbal_rtsp.sh
```

## 7. 跟踪模式

`face` 与 `swimmer` 是独立检测器，但共用云台运动控制层：

- `face_track.py`：人脸检测、光流补偿和短时预测。
- `infer_video.py`：泳者模型、ByteTrack 和目标平滑。
- 人脸保持区在 `config/system.config.json -> gimbal.face` 中独立配置。

## 8. 配置与检查

UWB 游泳者间歇跟随支持短时轨迹预测、等待出水和连续新测量恢复；配置、默认边界和验证步骤见 [UWB 间歇跟随](docs/UWB_INTERMITTENT_FOLLOW.md)。预测坐标始终与实测分开。

UWB 原始测量先做基础校验和预测创新检测，再做卡尔曼更新；可疑点仅预测并增大协方差。参数与诊断见 [UWB 原始测量门控](docs/UWB_MEASUREMENT_GATE.md)。

主配置：`config/system.config.json`。

```bash
npm ci
npm run check:config
npm test
npm run check
npm run maintenance
```

`npm run check:config` 只校验主配置。`npm run check` 会在对应解释器可用时检查 JavaScript、Python、Shell、JSON 和 systemd 服务模板；跳过项会明确输出。`npm run maintenance` 会先运行自动化测试，再运行全部静态检查。配置校验覆盖串口、端口、热点密码占位符、RTSP 地址、手机预览参数、录像编码器和跟踪保持区。

## 9. 目录职责

本目录同时包含板端运行程序、部署模板、维护工具和文档。维护时应先判断改动属于哪一层，再决定验证命令和需要重启的服务，避免用全量重启掩盖依赖关系。

### 9.1 核心运行目录

| 目录 | 职责与边界 | 关键入口 | 变更后的主要验证 |
| --- | --- | --- | --- |
| `backend/` | 控制面的服务端实现。`server.js` 提供 HTTP API、Socket.IO、静态页面、录像管理与云台/跟踪进程编排；`mavlink_bridge.py` 负责 Pixhawk UART 与本机 UDP 命令/遥测之间的桥接。这里不存放网页布局，也不应写入运行日志或录像。 | `backend/server.js`、`backend/mavlink_bridge.py` | `npm run maintenance`；按改动重启 `manta-backend.service` 或 `manta-bridge.service`，再检查 API、遥测和日志。 |
| `config/` | 可审查的运行参数源。`system.config.json` 管理串口、端口、热点、相机、云台、录像和跟踪；`motor_config.json` 管理电机通道与限制；`bluetooth.config.json` 管理蓝牙 PAN。代码应读取配置，不应在多个脚本中重复硬编码同一参数。 | `config/system.config.json`、`config/motor_config.json`、`config/bluetooth.config.json` | `npm run check:config` 和 `npm run maintenance`；根据参数消费者重启对应服务。 |
| `frontend/` | 浏览器控制台的静态资源。HTML 文件定义页面入口，`frontend/js/` 处理 API、Socket.IO、状态和交互，`frontend/css/` 管理样式。前端不得直接访问串口或设备文件，硬件操作必须经过后端接口。 | `frontend/index.html`、`frontend/gimbal.html`、`frontend/videos.html`、`frontend/map.html` | `npm run check`，浏览器强制刷新并验证连接、断线、加载中、错误和禁用状态；通常无需重启服务。 |
| `scripts/` | 板级集成与独立进程。包含安装、systemd 部署、热点/蓝牙、相机、云台流、模型转换、诊断、跟踪与状态报告。带硬件副作用的脚本必须显式执行，不能被静态检查或测试自动触发。 | `install.sh`、`install_boot_services.sh`、`status_report.sh`、`camera_snapshot_server.py`、`gimbal_rtsp_stream_server.py`、`face_track.py`、`infer_video.py` | `npm run maintenance`；再运行对应诊断脚本，并只重启受影响的服务。 |

### 9.2 部署、测试与文档目录

| 目录 | 职责与维护规则 |
| --- | --- |
| `systemd/` | systemd 单元的源码模板。`__PROJECT_DIR__` 和 `__RUN_USER__` 在安装时替换；`/etc/systemd/system/` 中的文件是部署产物，不是源码。修改模板后应运行 `scripts/install_boot_services.sh`、`systemctl daemon-reload`，并只重启相关单元。 |
| `test/` | 不接触硬件的快速自动化测试，当前主要覆盖配置约束。新增控制逻辑时应同时添加纯函数或协议级测试；板端串口、电机、视频和网络验证应单独记录，不能用单元测试结果替代。 |
| `docs/` | 面向开发和运维的长期文档。`ARCHITECTURE.md` 说明组件和数据流，`OPERATIONS.md` 说明部署、状态检查、日志和恢复。接口或服务拓扑变化时，代码与对应文档必须在同一提交更新。 |
| `.github/workflows/` | GitHub PR/Push 的自动检查。CI 只执行无硬件副作用的安装、测试和静态检查，不会连接 MANTA 主板或发送控制命令。该目录位于仓库根目录。 |

### 9.3 顶层文件与模型资产

| 路径 | 职责 |
| --- | --- |
| `README.md` | 当前文档：系统边界、安装、运行、目录职责和安全要求。 |
| `QUICKSTART_CN.md`、`SETUP_HELP.sh` | 首次部署的短路径说明和交互式辅助入口；不能替代完整运维文档。 |
| `PROJECT_SUMMARY_CN.md`、`algorithm_plan.txt` | 项目背景和算法规划资料，不是运行入口。 |
| `CONTRIBUTING.md`、`CHANGELOG.md` | 贡献流程、验证要求和可追溯变更记录。 |
| `package.json`、`package-lock.json` | Node.js 依赖、版本约束和维护命令。依赖变更必须同时提交 lockfile，并使用 `npm ci` 验证。 |
| `requirements.txt` | Python 运行依赖清单；增加 import 时同步更新，安装后仍需在 aarch64 板端验证。 |
| `quickstart.sh` | 首次安装入口：配置检查、依赖和服务部署。 |
| `start.sh`、`stop.sh` | 已安装系统的统一启停入口。`stop.sh` 默认保留无线维护链路，`stop.sh --all` 才停止全部连接服务。 |
| `check*.onnx`、`scripts/*.pt`、`scripts/models/` | 推理与转换资产。大模型不是普通源码；替换时必须记录来源、输入尺寸、类别、精度/性能基线和目标运行时，不能只按文件名覆盖。 |

### 9.4 运行时生成目录

以下路径由程序或工具生成，不应提交到 Git：

| 路径 | 内容 | 清理注意事项 |
| --- | --- | --- |
| `logs/`、`*.log` | systemd 服务和诊断日志 | 先保留故障时间窗口；正在写入的日志应通过日志轮转处理。 |
| `recordings/` | 云台录像及下载文件 | 不要在录像过程中删除；归档前确认文件已封装完成。 |
| `node_modules/` | `npm ci` 安装的 Node.js 依赖 | 可重新生成，不提交。 |
| `__pycache__/`、`*.pyc` | Python 字节码缓存 | 可安全重新生成，不作为部署来源。 |
| `.pids_*` | 旧式手动进程 PID 记录 | systemd 模式下不应作为服务真实状态；以 `systemctl` 为准。 |
| `*.bundle`、`*.tmp`、`config/*.bak` | 离线同步、临时文件和配置备份 | 可能含环境配置，确认用途后再清理，禁止提交凭据。 |

### 9.5 改动与服务影响矩阵

| 改动范围 | 最小验证 | 通常需要的运行操作 |
| --- | --- | --- |
| `backend/server.js` | `npm run maintenance`、HTTP/Socket.IO 冒烟检查 | 重启 `manta-backend.service` |
| `backend/mavlink_bridge.py` | `npm run maintenance`、飞控链路检查 | 重启 `manta-bridge.service` |
| `frontend/` | `npm run check`、桌面和移动浏览器交互检查 | 浏览器刷新；通常不重启服务 |
| `config/system.config.json` | `npm run check:config`、配置差异复核 | 仅重启读取了相关字段的服务 |
| 相机/云台媒体脚本 | `npm run maintenance`、对应 health/stream 端点 | 重启 `manta-camera.service` 或 `manta-gimbal-stream.service` |
| 热点、蓝牙和路由脚本 | Shell 检查、维护链路现场验证 | 重启对应连接服务，避免同时切断 SSH 和备用链路 |
| `systemd/*.template` | `npm run check`、渲染后单元复核 | 重新安装模板、`daemon-reload`、重启目标单元 |
| `docs/`、`test/` | 链接/命令复核、`npm run maintenance` | 不需要重启运行服务 |

### 9.6 依赖与修改原则

- `config/` 是运行参数的单一来源；新增参数应同时更新默认配置、校验器、测试和文档。
- 前端只通过后端公开协议控制硬件；修改事件名或 API 时，前后端和兼容处理必须一起更新。
- `systemd/` 模板是服务定义的源码；不要只在 `/etc/systemd/system/` 热修而不回写仓库。
- 自动化检查不得启动电机、连接串口、改网络或写飞控参数；这些属于明确授权的板端验证步骤。
- 软件检查通过只说明语法、配置和现有单元测试通过，不代表推进器、失联保护、急停或视频链路已完成实机验证。
- 每次提交尽量只覆盖一个可解释的维护单元，并在 PR 中写清变更范围、验证证据、受影响服务和回滚方式。

## 10. 安全要求

- 首次电机测试必须移除推进器或断开动力输出。
- 串口接线或模式切换前先解除武装。
- 不要在运行中删除当前录像文件。
- 不要把热点密码、SSH 私钥或飞控参数备份提交到 Git。
- 修改控制算法后至少运行 `npm test`、`npm run check` 和板端状态检查。

## 11. 已知限制

- 当前云台为 SIYI A8 mini：`rtsp://192.168.144.25:8554/main.264`，实测 H.264、1280×720、25 fps；RK3588 录像直接保存此码流。
- OV8858 本地摄像头依赖板级 overlay 与实际传感器状态。
- iOS 对 Linux 蓝牙 PAN 的支持不稳定，Wi-Fi 热点是推荐的手机控制路径。

## SIYI A8 mini 接入

当前云台已替换为 SIYI A8 mini，OV8858 本地相机保持原配置。控制采用 SIYI SDK UDP `192.168.144.25:37260`，路由脚本读取系统配置，在 `br-manta` 上使用 `192.168.144.101/32`。画面、识别、录像及 MediaMTX 均读取 `rtsp://192.168.144.25:8554/main.264`。旧 ZTD095 控制帧只用于内部规划，由适配器转成 SIYI 数据包后发送。

软件以向右、向下为正，SIYI 俯仰向上为正；倒装回传的 180° 偏移由适配器按设备安装状态修正。SDK 角度范围为 yaw ±135°、pitch -90° 至 +25°，对应软件 pitch -25° 至 +90°。每 100 ms 请求姿态，CRC 错误或过期反馈不用于运动控制。速度接口是比例量，`gimbal.siyi.rate_units_per_dps` 是待调参数，输出受 `max_rate_command` 限制。

录像保存到 RK3588，不自动切换相机 SD 卡录像；A8 没有旧云台的激光测距及内置检测控制，相关 API 返回 501。旧云台 RTSP OSD 命令不可用于 A8，界面隐藏该开关。A8 固定焦距，原自动对焦入口改为恢复 1x 数字变倍。

2026-10-20 用户确认 UWB 接收器中心位于 A8 云台旋转中心正下方 4 cm，朝向与回中镜头一致。已按此安装关系启用 `gimbal.beacon.extrinsic_3d`：相机坐标前/右/下，矩阵 `diag(1,-1,-1)` 保留既有 UWB 角度符号约定，平移 `[0,0,0.04]` 米；目标在 UWB 等高方向、距离 1 米时，相机应下俯约 2.29°，距离 4 米时约 0.57°。`calibration_required=false`，允许启动信标跟踪。该配置来自安装尺寸，不表示完成多点实测标定；可通过现有标定向导进一步校正安装偏差。旧角度缩放与偏置也被船体控制使用，因此原值保留，云台位置补偿通过独立三维变换计算。图像内参由 81° 水平视场近似推算，精确指向需棋盘格标定。

协议依据：[SIYI 官方 SDK V0.1.1](https://siyi.biz/siyi_file/A8%20mini/SDK%20-%20V0.1.1.pdf)。

现场发现 192.168.144.25 的 ARP 记录曾被另一 MAC 00301bba02db 抢占；配置 `gimbal.network_mac=7c:63:32:cf:c7:fe`，由路由服务为本台 A8 固定邻居记录。换一台 A8 时必须核对并更新该 MAC。

## 无 UWB 时启动自动跟随

自动跟随开关允许在 UWB 无效时发起请求。后端返回 HTTP 202，状态为 `awaiting_signal`，`followRequested=true`、`autoResumePending=true`；界面显示“搜索 UWB”，开关可再次点击“停止搜索”。搜索阶段保持中立输出及未武装状态，不发送前进或旋转搜索指令。

收到同一目标连续、不同时间戳的可信 UWB 测量后，复用现有恢复流程；仍需导航、航向、控制参数、距离和视野检查通过，然后切换模式、武装并再次确认 UWB。跟随中 UWB 超时也保留请求并进入等待。停止搜索、手动接管、切回手动、急停或明确解除武装会取消请求，之后恢复信号不会自行启动。此处“搜索”表示等待接收有效 UWB，不表示船体盲目转圈。

## 推进器方向记录

2026-10-20 根据实机反馈，将左推进器 `SERVO1_REVERSED` 从 1 改为 0。右推进器 `SERVO3_REVERSED` 实机为 0，保持原值；通道功能、RC 输入方向、中位与 PWM 上下限均未更改。`configure_skid_steer_outputs.py` 已同步左右输出反向参数为 0，避免以后应用脚本时覆盖本次修正。

## UWB 独立接收与实时推送

桥接程序独占 MAVLink 串口，持续以非阻塞方式接收。完整 UWB 距离/方位测量经过原有校验与滤波后，立即通过 UDP `type: uwb` 发布；无需等待 GPS、IMU 或整包遥测的发送周期。无效测量也立即发布失效状态，原有融合、武装、失联保护与推进器限制保持生效。

后端将独立数据保存到 `/api/telemetry` 的 `telemetry.uwb`，并发送 Socket.IO `uwb_update` 事件；原有 `telemetry_update`、相机、云台及其他 HTTP 请求均保留。相机健康请求在后台异步执行，共享最近结果，600 ms 超时不阻塞遥测；温度检查与跟踪进程清理也不再同步等待外部操作。

定位频率应按新的 `measurementUpdatedAt` 统计，不能将重复位置的遥测包数或诊断 `goodCount` 当作新位置更新率。电脑只能接收飞控实际转发的测量；仅更新上位机不会改变飞控 Lua 脚本的转发周期。

### 飞控逐帧转发

2026-10-22 检查当天已部署脚本的回读副本，发现 `SEND_PERIOD_MS = 100`：脚本每 100 ms 至多转发一个位置，因此接收端约 9 Hz，即使 UWB UART 的有效帧计数增长更快。修改 `telemetry_rate_hz` 或 MAVLink 通用流速率无法移除这个 Lua 条件。

`scripts/pixhawk/uwb_aoa.lua` 基于该双路脚本修改，保留 SERIAL1/2 Scripting、115200 波特率、距离校验及每轮最多 48 字节。每 10 ms 交替处理一个接收器，每次最多完成一个位置并立即转发，支持每路最高 50 Hz；设计目标是跟上此前约 40 Hz 的输入。达到更高输入频率或调度延迟持续增大时仍可能积压，不能据此保证任意输入速率。

每帧发送 `DIST`、`EL`、`AZ`；`AZ` 最后完成距离/方位配对，`VALID` 状态变化安排在它之前。`GOOD`、`SENT`、接收字节和 CRC 等诊断改为约每秒一轮、分散到不同调度轮次。双路各 40 Hz 时，三个未签名 MAVLink 2 浮点消息约占 7200 B/s，另加诊断与正常遥测；115200、8N1 的理论容量为 11520 B/s，实际仍需检查丢包与负载。没有关闭相机、云台或其他上位机请求，也不发送武装、推进器控制或 EKF 输入。

飞控 SD 卡上的部署位置为 `/APM/scripts/uwb_aoa.lua`。更新前应回读并备份当前文件，确认与本次基线相符；解除武装且跟随停止时替换并重启 Lua，回读校验新文件并检查脚本错误。无需刷写整套固件或更改串口参数。只提交文件到 GitHub 不会更新飞控 SD 卡；未连接硬件时只能完成离线验证。

实测工具：`python scripts/measure_uwb_forwarding.py --seconds 30`。它只读取已运行桥接的 `/api/telemetry`，不打开串口、不发送命令。分别报告每路飞控有效帧计数增长率、转发尝试计数增长率、桥接收到和滤波接受的新位置观察频率。旧 Lua 无 `SENT` 时该项为 null；计数重置会重新建立窗口，HTTP 轮询偏慢时位置观察频率可能低估。应比较每个接收器的 `sensors[].goodCount`，顶层融合计数会因接收器失效或切换而变化。

Lua 回归测试使用可选 `lupa` 的 Lua 5.3 解释器执行实际脚本并模拟 UART：`python -m unittest discover -s test -p test_pixhawk_uwb_lua.py -v`。模拟结果不能替代飞控硬件上的执行耗时、串口和 MAVLink 丢包实测。
