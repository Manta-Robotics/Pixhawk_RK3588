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

## 3. 安装

要求 Ubuntu 22.04、root 权限、Node.js 18+、Python 3、FFmpeg 和 NetworkManager。

```bash
cd /root/Pixhawk_RK3588/rk3588_control_system
bash quickstart.sh
```

安装脚本会验证配置、安装依赖和八个 systemd 服务。UART/摄像头 overlay 首次变更后需要由操作人员确认并手动重启板子，脚本不会自动重启。

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

手机连接 `Manta-Control` 后，系统连通性探测会返回成功，避免 iPhone/iPad 将热点标记为受限并中断下载。请在 Safari 中打开 `http://10.42.0.1:3000`。

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

主配置：`config/system.config.json`。

```bash
npm ci
npm run check:config
npm test
npm run check
npm run maintenance
```

`npm run check:config` 只校验主配置。`npm run check` 会统一检查 JavaScript、Python、Shell、JSON 和 systemd 服务模板；`npm run maintenance` 会先运行自动化测试，再运行全部静态检查。配置校验覆盖串口、端口、热点密码、RTSP 地址、手机预览参数、录像编码器和跟踪保持区。

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
- 每次提交尽量只覆盖一个可解释的维护单元，并在 commit 或 PR 中写清变更范围、验证证据、受影响服务和回滚方式；高风险硬件与基础设施改动优先走功能分支和 PR。

## 10. 电机控制开发指南

本系统采用“RK3588 生成运动意图、Pixhawk 执行履带混控和安全状态管理”的分层方式。新控制器应复用现有后端接口，不应从业务代码直接占用 `/dev/ttyS1`，也不应绕过 Pixhawk 直接向 Main 输出写 PWM。

### 10.1 当前控制模型与通道

当前车辆按双电机差速/履带式底盘配置，`1500us` 为中值，低于中值为一个方向，高于中值为相反方向。具体方向还受 Pixhawk 的 `SERVOx_REVERSED`、电调设置和接线影响，不能只按 PWM 数字推断实机前进方向。

| 层级 | 当前映射 | 配置来源 |
| --- | --- | --- |
| 运动意图 | 油门 `-100..100`，转向 `-45..45` | `config/system.config.json` |
| Pixhawk RC 输入 | 转向输入 CH1，油门输入 CH3 | `rover_steering_input_channel`、`rover_throttle_input_channel` |
| Pixhawk Main 输出 | Main1 左电机，Main3 右电机 | `rover_left_channel`、`rover_right_channel` |
| 后端 PWM 范围 | `1000..2000us`，中值 `1500us` | `min_motor_pwm`、`max_motor_pwm`、`default_motor_pwm` |
| 可用电机通道 | CH1、CH3 启用；CH2、CH4 预留 | `config/motor_config.json` |

`system.config.json` 中的全局 PWM 范围是后端实际限幅来源。`motor_config.json` 当前主要提供通道启用状态和电机元数据，其中每个电机的 `min_pwm`、`max_pwm`、`center_pwm` 尚未用于逐通道限幅；如果以后需要不同电机采用不同范围，应同时修改校验、混控、测试和文档。

### 10.2 从控制输入到电机输出

```text
Web / iPad / 自主控制算法
  |  REST: POST /api/control/rover
  |  Socket.IO: rover_drive
  v
backend/server.js
  |  输入转数字 -> 范围限幅 -> 油门/转向转 PWM
  |  UDP JSON 命令 ROVER_DRIVE -> 127.0.0.1:14551
  v
backend/mavlink_bridge.py
  |  MAVLink RC_CHANNELS_OVERRIDE
  |  CH3=油门输入，CH1=转向输入，其余通道保持 ignore
  v
Pixhawk 履带混控
  |  SERVO1_FUNCTION=73 -> Main1 左电机
  |  SERVO3_FUNCTION=74 -> Main3 右电机
  v
电调 -> 左/右电机

Pixhawk SERVO_OUTPUT_RAW / ESC telemetry
  -> mavlink_bridge.py -> UDP 14552 -> server.js
  -> /api/status、/api/telemetry、Socket.IO telemetry_update
```

后端负责输入校验、软件限幅、状态聚合和日志；桥接进程负责 UDP、MAVLink 与串口协议转换；实际混控、解锁状态和最终 Main 输出由 Pixhawk 决定。

### 10.3 对外控制接口

| 接口 | 输入 | 用途与注意事项 |
| --- | --- | --- |
| `POST /api/control/rover` | `{"throttle": 0, "steering": 0}` | 推荐的 REST 入口。成对提交油门和转向，超出配置范围时自动限幅，并在返回值中给出 `clamped`。 |
| Socket.IO `rover_drive` | `{ throttle, steering }` | 推荐的实时遥控入口；服务端返回 `rover_drive_ack`，并广播 `rover_control_update`。 |
| `POST /api/control/motor` | `{channel,pwm}` 或 `{motors:[...]}` | 兼容旧电机/视觉控制器。只接受 `motor_config.json` 中启用的通道，随后反算成油门/转向并仍由 Pixhawk 混控。新控制器优先使用 rover 接口。 |
| Socket.IO `motor_control` | `{ channel, pwm }` | 上述兼容入口的实时版本；错误通过 `error_message` 返回。 |
| `GET /api/motors` | 无 | 查看后端保存的电机命令状态和 rover 控制状态，不等同于 Pixhawk 实际输出。 |
| `GET /api/status` | 无 | 查看连接、解锁、限制值、输入/输出通道和综合遥测。 |
| `POST /api/emergency/stop` | 空 JSON | 将油门/转向输入置中，并发送解除武装命令。它是软件请求，仍需独立物理急停和飞控失联保护。 |
| Socket.IO `arm` / `disarm` | 无 | 当前解锁/解除武装入口；REST 暂无对应 arm/disarm 路由。 |

只读检查和中值命令示例：

```bash
curl -fsS http://127.0.0.1:3000/api/status
curl -fsS http://127.0.0.1:3000/api/motors

# 仅在推进器已移除或动力输出已物理断开的台架上发送。
curl -fsS -X POST http://127.0.0.1:3000/api/control/rover \
  -H 'Content-Type: application/json' \
  -d '{"throttle":0,"steering":0}'

curl -fsS -X POST http://127.0.0.1:3000/api/emergency/stop \
  -H 'Content-Type: application/json' \
  -d '{}'
```

前端控制器可复用 `frontend/js/realtime_client.js` 中的 `drive()`、`setMotorPwm()`、`arm()`、`disarm()` 和 `emergencyStop()`，不要在不同页面重复实现协议。

### 10.4 限幅与差速换算

`backend/server.js -> normalizeRoverControl()` 执行以下换算，所有 PWM 最终都会限制在 `PWM_MIN..PWM_MAX`：

```text
throttle_scale = (PWM_MAX - PWM_CENTER) / max(abs(throttle_min), abs(throttle_max))
steering_scale = (PWM_MAX - PWM_CENTER) / max(abs(steering_min), abs(steering_max))

throttle_input_pwm = PWM_CENTER + throttle * throttle_scale
steering_input_pwm = PWM_CENTER + steering * steering_scale

left_pwm  = PWM_CENTER + throttle * throttle_scale - steering * steering_scale
right_pwm = PWM_CENTER + throttle * throttle_scale + steering * steering_scale
```

在当前配置下，油门每单位约对应 `5us`，转向每单位约对应 `11.11us`。`left_pwm/right_pwm` 是后端用于界面显示和状态记录的期望值；确认真实输出时必须查看 Pixhawk 回传的 `telemetry.servoOutputs`。

`/api/control/motor` 会把当前左/右 PWM 反算为油门和转向输入。批量 `motors` 数组目前按元素顺序逐条处理并逐条发送，不是原子操作，可能出现极短的中间状态；需要同步更新两侧电机的新控制器应使用 `/api/control/rover`。

### 10.5 Pixhawk 参数与自动配置

桥接服务每次收到 Pixhawk 心跳后会检查并在不匹配时写入以下参数：

| 参数 | 当前目标值 | 作用 |
| --- | ---: | --- |
| `PILOT_STEER_TYPE` | `0` | 使用常规油门/转向输入方式。 |
| `RC1_REVERSED`、`RC3_REVERSED` | `0` | 输入层不反转，避免与输出反转叠加。 |
| `SERVO1_FUNCTION` | `73` | Main1 作为左侧履带/电机输出。 |
| `SERVO3_FUNCTION` | `74` | Main3 作为右侧履带/电机输出。 |
| `SERVO1_REVERSED`、`SERVO3_REVERSED` | `1` | 当前实机接线对应的输出反转设置。 |

这些参数由 `mavlink_bridge.py -> _ensure_rover_motor_outputs()` 维护，修改代码或 QGroundControl 参数前必须先明确唯一权威来源，否则桥接重连时会把参数恢复成代码中的目标值。

`scripts/configure_skid_steer_outputs.py` 可独立检查相同映射；它会直接占用 Pixhawk 串口，因此不能和 `manta-bridge.service` 同时运行：

```bash
# 前提：解除武装、移除推进器或断开动力输出，并保留现场维护连接。
sudo systemctl stop manta-backend.service
sudo systemctl stop manta-bridge.service

# 默认只检查；只有经操作人员确认后才允许使用 --apply 写参数。
python3 scripts/configure_skid_steer_outputs.py
# python3 scripts/configure_skid_steer_outputs.py --apply

sudo systemctl start manta-bridge.service
sudo systemctl start manta-backend.service
```

### 10.6 解锁、急停与当前安全边界

- 普通 rover/motor 接口当前不会检查 `isConnected` 或 `armed` 后再发送；是否产生实际动力由 Pixhawk 状态决定。
- `server.js -> sendMavlinkCommand()` 使用本机 UDP，无逐命令确认；HTTP/Socket.IO 成功表示后端已接收并发送，不表示 Pixhawk 或电调已经执行。
- 桥接对 RC Override 再做一次 `1000..2000us` 限幅，并拒绝原始 `MOTOR_CONTROL` 命令，确保输出反转仍由 Pixhawk 参数统一管理。
- 软件急停流程是“油门/转向回到 `1500us` -> 发送 DISARM -> 更新界面状态”，但当前没有等待 Pixhawk ACK 后再返回，也不是锁存式急停：后续控制命令仍会被接收，正在运行的视觉控制进程也不会由该接口停止。
- 当前代码没有通用的 motor command deadman/超时自动回中机制，Socket.IO 客户端断开时也不会自动发送中值。开发实时遥控或自主控制前，必须同时设计命令超时、客户端断开、后端退出、桥接掉线和 Pixhawk RC Override 失效后的安全行为。
- `motorStatus` 和 `roverControl` 是后端命令状态；`SERVO_OUTPUT_RAW` 才是 Pixhawk 输出观测。两者不一致时应先停止动力，再检查解锁、模式、failsafe、混控参数和串口链路。
- 软件保护不能替代物理急停、动力接触器、保险、Pixhawk failsafe 和现场安全员。任何非中值测试都应先移除推进器或架空底盘。

### 10.7 新控制逻辑应该修改哪里

| 需求 | 首选修改位置 | 必须同步验证 |
| --- | --- | --- |
| 新增 Web/iPad 操纵方式 | `frontend/`，复用 `realtime_client.js` | 断连、中值回归、重复连接、触控释放状态 |
| 修改输入范围或通道 | `config/system.config.json` | `validate_config.mjs`、配置测试、Pixhawk 参数、README |
| 修改通道启用或电机元数据 | `config/motor_config.json` | 左右通道均启用、后端启动日志、接口拒绝行为 |
| 修改限幅、混控或命令语义 | `backend/server.js` | 纯函数/协议测试、边界值、NaN、超范围、急停 |
| 修改 MAVLink 或 Pixhawk 参数策略 | `backend/mavlink_bridge.py` | 串口重连、参数读写 ACK、RC Override、真实输出遥测 |
| 新增自主控制算法 | 独立脚本，通过 `/api/control/rover` 调用后端 | 控制频率、目标丢失、进程退出、网络失败、超时回中 |
| 修改视觉跟车 | `scripts/vision_face_controller.py` | 当前兼容 motor API、目标丢失保持时间、退出回中、左右同步 |
| 修改服务启动方式 | `systemd/*.template`、安装脚本 | `daemon-reload`、ExecStart、启动顺序、异常重启和日志 |

不要在新算法中复制 PWM 限幅、串口连接或 Pixhawk 参数写入逻辑。控制算法只生成标准化的 `throttle/steering`，安全门控和协议边界集中维护，才能避免不同控制源出现相反方向、不同中值或不同急停语义。

### 10.8 推荐开发与台架调试顺序

1. 确认主板位于预期分支且工作区干净，运行 `npm run maintenance`。
2. 解除武装，移除推进器、断开动力输出或架空底盘，确认物理急停可用。
3. 用 `systemctl status manta-bridge manta-backend` 和 `/api/status` 确认桥接、心跳、通道与限制值。
4. 先发送中值 `throttle=0, steering=0`，确认命令状态和 `SERVO_OUTPUT_RAW` 都回到预期中值。
5. 按“单侧小幅 -> 另一侧小幅 -> 同向 -> 原地转向”逐步测试；每一步都记录输入、预测左右 PWM、真实 Main 输出和电机方向。
6. 测试手动 disarm、软件 emergency stop、控制端断开、后端停止、桥接停止和 Pixhawk 失联；任何场景不能保持不可控动力。
7. 只重启改动影响的服务：后端/配置修改通常重启 `manta-backend`，桥接或通道/Pixhawk 策略修改重启 `manta-bridge`，前端静态文件通常刷新浏览器即可。
8. 验证完成后提交代码、配置、测试和记录；不要提交飞控参数备份、设备凭据或现场日志。

常用观测命令：

```bash
sudo journalctl -u manta-bridge.service -f
sudo journalctl -u manta-backend.service -f
curl -fsS http://127.0.0.1:3000/api/telemetry
curl -fsS 'http://127.0.0.1:3000/api/logs?limit=100'
```

### 10.9 控制系统改动验收清单

- 输入非法、缺失、非数字或超范围时，系统会拒绝或按文档限幅，不会产生未定义 PWM。
- `throttle=0, steering=0`、急停、disarm 和控制器退出均回到确认过的安全状态。
- 左右方向、通道和反转只在一个层级定义，没有“代码反转 + Pixhawk 反转 + 接线反转”相互抵消。
- UI 命令状态、后端日志、RC 输入意图和 `SERVO_OUTPUT_RAW` 能够对应追踪。
- 客户端断开、命令超时、UDP 丢包、串口断开、Pixhawk 重启和服务崩溃都完成台架验证。
- 自动化测试覆盖混控公式、边界值、通道禁用、批量命令和急停；实机/HIL 结果单独记录。
- 修改后的服务重启范围、回滚提交和 Pixhawk 参数恢复方法已经记录。

## 11. 安全要求

- 首次电机测试必须移除推进器或断开动力输出。
- 串口接线或模式切换前先解除武装。
- 不要在运行中删除当前录像文件。
- 不要把热点密码、SSH 私钥或飞控参数备份提交到 Git。
- 修改控制算法后至少运行 `npm test`、`npm run check` 和板端状态检查。

## 12. 已知限制

- 当前可验证的云台 `/live/0` 码流为 1080p；4K 必须由云台固件提供真实高分辨率流。
- OV8858 本地摄像头依赖板级 overlay 与实际传感器状态。
- iOS 对 Linux 蓝牙 PAN 的支持不稳定，Wi-Fi 热点是推荐的手机控制路径。
