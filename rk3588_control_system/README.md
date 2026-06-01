# RK3588 + Pixhawk Rover Control System

## 1. 概览
本项目运行在 RK3588（鲁班猫5）上，提供以下能力：
- Web Dashboard（实时状态、遥测、相机、控制）
- Pixhawk MAVLink 通信桥接
- 相机 MJPEG 采集与流服务（/snapshot.jpg, /stream.mjpg）
- Vision 模式（YOLOv8 人体检测 + 双电机差速控制）

## 2. 运行架构

```
Browser UI
  └─ HTTP/WebSocket :3000
      └─ backend/server.js (Express + Socket.io)
          ├─ UDP :14551 -> backend/mavlink_bridge.py -> Pixhawk (UART)
          ├─ HTTP :8090 <- scripts/camera_snapshot_server.py <- /dev/video11
          └─ subprocess -> scripts/vision_face_controller.py
                           ├─ GET /snapshot.jpg (from :8090)
                           └─ POST /api/control/motor (to :3000)
```

### 2.1 控制链路
- 手动遥控：前端 -> `motor_control` -> `DO_SET_SERVO`
- Vision 遥控：YOLO 检测 -> 差速 PWM -> `/api/control/motor`
- 传感/状态：Pixhawk -> `mavlink_bridge.py` -> `server.js` -> 前端实时推送

### 2.2 相机链路
- 采集线程：`scripts/camera_snapshot_server.py`
- 编码方式：ffmpeg + mjpeg
- 服务端口：`8090`
- 关键端点：
  - `GET /snapshot.jpg`
  - `GET /stream.mjpg`
  - `GET /healthz`

## 3. 配置文件说明
主配置：`config/system.config.json`

### 3.1 camera 段（核心）
- `width/height/fps`：采集参数
- `input_format`：输入像素格式；默认 `auto`（推荐）
- `sensor_controls.exposure/analogue_gain`：传感器控制
- `sensor_control_reapply_seconds`：流启动后重施加控制时间点
- `stall_reconnect_seconds`：流卡死后自动重连阈值

### 3.2 vision 段
- `detect_width`：YOLO 输入宽度（建议 640）
- `conf_threshold`、`iou_threshold`：检测阈值
- `max_pwm`：Vision 模式最大 PWM（默认 1800）
- `track_min_forward_pwm`：保证双电机都转的最小前进量

## 4. 启动流程
1. `manta-camera.service` 启动相机流服务（:8090）
2. `manta-bridge.service` 启动 MAVLink 桥接
3. `manta-backend.service` 启动 Node 后端（:3000）
4. 前端连接后可切换 Vision 模式

## 5. Vision 流程
1. 前端点击 Vision 按钮
2. 后端 `POST /api/vision/start` 拉起 `vision_face_controller.py`
3. Vision 进程按周期读取 `/snapshot.jpg`
4. YOLOv8 检测人 -> 计算 offset -> 双电机差速 PWM
5. 通过 `/api/control/motor` 下发到 Pixhawk

## 6. 相机绿屏/花屏排查
优先检查：
1. `camera.input_format` 是否为 `auto`（不要硬编码 `nv12`）
2. `GET http://127.0.0.1:8090/healthz` 是否 `ok:true`
3. `logs/camera.log` 是否出现频繁 ffmpeg 退出
4. `v4l2-ctl -d /dev/v4l-subdev2 --get-ctrl=exposure,analogue_gain`

## 7. 目录结构
```
rk3588_control_system/
├── backend/
│   ├── server.js
│   └── mavlink_bridge.py
├── scripts/
│   ├── camera_snapshot_server.py
│   └── vision_face_controller.py
├── frontend/
├── config/
└── logs/
```

## 硬件连接

### Pixhawk 6X → RK3588 GPIO
```
Pixhawk Telem1 (6pin)    RK3588 40pin GPIO
  ├─ TX ────────────────→ RX (GPIO UART)
  ├─ RX ────────────────← TX (GPIO UART)
  └─ GND ───────────────→ GND

注意：请确认RK3588上TX/RX/GND的具体pin脚号
```

### 其他外设
- **蓝牙**：板载集成，无需接线
- **Wi-Fi**：板载集成，无需接线
- **摄像头**：MIPI CSI接口

## 快速开始（Ubuntu 22.04）

### 1. 系统环境准备
```bash
# 克隆项目到Ubuntu
cd /home/your_user/
git clone <your-repo-url> rk3588_control
cd rk3588_control

# 运行安装脚本
bash scripts/install.sh

# 配置串口权限
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER
newgrp dialout
```

### 2. 配置系统
```bash
# 编辑配置文件
nano config/system.config.json

# 主要配置项：
# - serial_port: Pixhawk连接的串口 (通常 /dev/ttyUSB0 或 /dev/ttyS0)
# - baud_rate: 波特率 (通常 57600)
# - web_port: Web服务器端口 (默认 3000)
# - bluetooth_device: 蓝牙设备名称
```

### 3. 启动系统
```bash
# 一键启动所有服务
bash start.sh

# 或分别启动：
cd backend
node server.js &              # 启动Node.js
python3 mavlink_bridge.py &   # 启动MAVLink桥接

# 检查状态
ps aux | grep node
ps aux | grep python
```

### 4. 访问Dashboard
```
Web浏览器打开：http://rk3588-ip:3000
或本地访问：http://localhost:3000
```

## 项目结构

```
rk3588_control_system/
├── README.md                      # 本文件
├── package.json                   # Node.js依赖
├── requirements.txt               # Python依赖
├── start.sh                       # 启动脚本
│
├── backend/
│   ├── server.js                 # Node.js Express服务器
│   ├── mavlink_bridge.py         # MAVLink通信（Python）
│   ├── motor_control.js          # 电机控制接口
│   └── logs/                     # 日志文件
│
├── frontend/
│   ├── index.html                # Dashboard主页
│   ├── js/
│   │   ├── dashboard.js          # 仪表板逻辑
│   │   └── socket_client.js      # WebSocket客户端
│   ├── css/
│   │   └── style.css             # 样式
│   └── assets/
│       └── map/                  # 地图文件
│
├── config/
│   ├── system.config.json        # 系统配置
│   ├── motor_config.json         # 电机参数
│   └── bluetooth.config.json     # 蓝牙配置
│
└── scripts/
    ├── install.sh                # Ubuntu依赖安装
    ├── setup_serial.sh           # 串口权限配置
    └── setup_bluetooth.sh        # 蓝牙配置
```

## 关键功能

### 1. 电机控制
```javascript
// 通过Web发送控制命令
socket.emit('motor_control', {
  channel: 1,           // 通道号 (1-4)
  pwm: 1500,            // PWM值 (1000-2000)
  duration: 1000        // 持续时间(ms)
});
```

### 2. 实时遥测
```
自动接收Pixhawk发送的数据：
- 位置信息 (GPS)
- 姿态数据 (Roll/Pitch/Yaw)
- 速度信息 (Velocity)
- 电池状态 (Voltage/Current)
- 飞行模式
- 系统状态
```

### 3. 蓝牙控制
```bash
# 配对蓝牙设备
bluetoothctl
> power on
> scan on
> pair <MAC_ADDRESS>
> connect <MAC_ADDRESS>

# 蓝牙数据会通过Web Socket转发到前端
```

### 4. 日志和监控
```
所有数据都会记录到：
- logs/flight_data.csv
- logs/system.log
- logs/error.log

Dashboard可实时查看和下载
```

## 常见问题

### 串口找不到
```bash
# 列出所有串口
ls -la /dev/tty*

# 查看USB设备
lsusb

# 修改权限
sudo chmod 666 /dev/ttyUSB0
```

### 蓝牙连接问题
```bash
# 查看蓝牙状态
bluetoothctl show
hciconfig

# 重启蓝牙
sudo systemctl restart bluetooth
```

### 高CPU占用
- 检查mavlink_bridge.py的循环频率
- 减少Dashboard的更新频率
- 检查是否有死循环

## 开发指南

### 添加新的电机通道
1. 在`config/motor_config.json`中添加配置
2. 在`backend/motor_control.js`中添加控制方法
3. 在前端Dashboard中添加UI控制

### 修改MAVLink消息处理
编辑`backend/mavlink_bridge.py`中的`parse_mavlink_message()`函数

### 自定义Dashboard
编辑`frontend/index.html`和`frontend/js/dashboard.js`

## 性能指标

| 指标 | 目标值 | 实现状态 |
|---|---|---|
| Web延迟 | <100ms | ✅ |
| 电机响应 | <200ms | ✅ |
| 遥测更新频率 | 50Hz | ✅ |
| 蓝牙数据率 | 115200 bps | ✅ |

## 故障排查

### 系统启动失败
```bash
# 查看日志
tail -f logs/system.log

# 检查Node.js
node --version

# 检查Python
python3 --version && python3 -c "import pymavlink"
```

### 数据无法接收
```bash
# 检查串口连接
cat /dev/ttyUSB0

# 检查波特率
stty -F /dev/ttyUSB0 57600 raw

# 使用minicom调试
minicom -D /dev/ttyUSB0 -b 57600
```

## 技术栈

- **后端**：Node.js, Express.js, Socket.io
- **前端**：HTML5, CSS3, JavaScript (ES6+)
- **飞控通信**：Python, pymavlink
- **操作系统**：Ubuntu 22.04 LTS (RK3588)
- **数据库**（可选）：SQLite
- **容器**（可选）：Docker

## 安全建议

- ✅ 所有电机命令都经过验证
- ✅ 实现了遥测超时检测
- ✅ 添加了紧急停止功能
- ✅ 日志记录所有操作
- ✅ 建议启用HTTPS（生产环境）

## License
MIT

## 作者
Manta Robotic Team

## 支持
如有问题，请提交Issue或联系开发团队
