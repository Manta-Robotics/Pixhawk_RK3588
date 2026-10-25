# 独立 C++ 控制服务

`control/bin/manta-control` 是 MAVLink 串口、UWB 滤波/双接收器融合、传统与神经网络跟随、预测恢复、NMPC、手动推进限幅和命令超时的唯一运行进程。Node.js 保留 HTTP/Socket.IO、云台、视频、地图资源、录像、标定入口和 CSV 管理；Python 保留视觉及媒体工作进程。

前端继续使用既有 React + TypeScript 原样迁移页面。本次不修改布局、样式、图标、横屏规则或交互代码。

## 构建与安装

在 RK3588 Ubuntu 或 Linux 开发机运行：

```bash
sudo apt-get install build-essential cmake nlohmann-json3-dev python3-venv
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
bash scripts/build_native_control.sh
npm ci
npm run maintenance
```

MAVLink C 头文件从安装的 pymavlink 官方 ardupilotmega XML 在构建时生成。部署后的 C++ 进程不调用 Python、Node 或 JS 引擎。模型沿用 `models/smart_usv_uwb_policy.json`，由 C++ 校验并推理。`scripts/install.sh` 包含编译依赖及构建步骤。离线构建需要预装这些软件包并准备 Python wheelhouse 与 npm 缓存。

完整安装仍使用 `sudo bash scripts/install_boot_services.sh`。控制服务沿用原有 **manta-bridge.service** 名称，以兼容 start/stop/status/doctor 和现有运维命令，其 ExecStart 已切换为 `scripts/native_control_service.sh`。不存在同时运行 Python bridge 的第二个服务。安装与编译不会自动启动、重启或解锁设备。

```bash
# 不访问串口的配置检查
control/bin/manta-control --config config/system.config.json --check-config
# 运行时操作；部署后由操作者选择执行
sudo systemctl restart manta-bridge.service
sudo systemctl restart manta-backend.service
sudo systemctl status manta-bridge.service manta-backend.service
```

更新运行程序通过原子替换完成，已运行进程继续使用旧映像，直到显式重启。项目目录是 systemd WorkingDirectory，模型的相对路径以此解析。运行用户需要串口访问权限；安装器将其加入 dialout 组。

## 进程边界与行为

- UDP 命令口默认 `127.0.0.1:14551`，遥测默认 `127.0.0.1:14552`。保留原有遥测字段及 Web API，新增 requestId 对应的命令响应。
- Web 只发送 MANUAL_TARGET、FOLLOW_START/STOP/METHOD 等操作请求。串口收发、跟随、手动加减速和超时保护均由 C++ 定时驱动。Web 重启不会取消已授权跟随；手动摇杆请求中断则按原命令租约归零。
- 启动跟随等待真实飞控遥测确认模式和武装状态；手动停止、手动接管或取消启动可立即使旧事务失效。没有强制解锁，也没有绕过 Rover 混控直接写推进器 PWM。
- UWB 长时缺测保留原等待、解除武装及可信观测窗口恢复流程。恢复期间的速度渐增、预测协方差、接近目标制动、遮挡隔离和神经网络推进规则均保留。
- 串口使用独占文件锁；重复命令端口绑定失败即退出。心跳、命令租约、进程调度和内部测量使用单调时钟，避免 NTP 回拨破坏测量顺序。发送遥测和 RPC 响应时统一换算到当前系统时间，保留每条观测的实际年龄；否则 WSL 校时后内部 epoch 偏移可能让网页误判为未来数据。过期值、未来值和空时间戳不因此变成新观测。
- 正常 SIGTERM 先清除待发送运动命令，再发送中立输出。进程被强制杀死、板端断电等情形仍依赖飞控自身的 GCS/RC 失效保护，服务软件不能替代飞控保护。
- Node 仍按每次跟随生成原 CSV。Web 离线期间控制继续运行，但该 Web CSV 记录暂停；服务日志由 systemd 写入 `logs/mavlink.log`。

## 无硬件验证

```bash
python scripts/check_native_parity.py control/build/manta-replay
python scripts/check_native_wire.py control/build/manta-wire-replay
node scripts/check_native_follow.mjs control/build/manta-follow-replay
python scripts/check_native_service.py control/build/manta-control
python scripts/check_native_stack.py control/build/manta-control
```

验证覆盖 UWB 全字段对照、真实 MAVLink 1/2 编解码、CRC/分片、四种 PILOT_STEER_TYPE、ACRO 物理量与 RC 标定编码、IMU 六面校准、推进/跟随算法对照、Linux 伪串口、UDP RPC、独立看门狗、HTTP 启停、Web 重启、自动恢复及取消启动。

旧 Python/JavaScript 算法保留为差分参考；`test/fixtures/legacy-server-control.txt` 固定迁移前的服务器行为供原 VM 测试使用，不参与生产运行。新的 native 测试验证替代实现，不能仅凭旧基线测试通过判断迁移完成。

这些检查不连接实际飞控、云台或推进器。RK3588 实机串口、视频、校准和水上运行仍需在目标设备验收。

## 本地 SITL

使用隔离配置，设 `MANTA_SIMULATION=1`、虚拟串口 `/dev/manta-sitl-virtual`、命令/遥测端口 `15551/15552`，然后：

```bash
MANTA_SIMULATION=1 MANTA_CONFIG_FILE=/absolute/simulation.json \
  control/bin/manta-control --simulation-udp 127.0.0.1:14560
MANTA_SIMULATION=1 MANTA_CONFIG_FILE=/absolute/simulation.json npm start
```

SITL/MAVLink 路由器须在指定 loopback UDP 口接收 MAVLink，并将遥测回送至服务的 UDP 源端口。旧外部仿真若导入 Python bridge，应切换到上述独立原生进程。原 Python 模块继续可供基线测试使用。
