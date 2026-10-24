# 外部本地仿真环境接入

Stonefish 环境独立维护在 <https://github.com/Manta-Robotics/Stonefish_SITL>。
船端控制算法只在本仓库维护；仿真运行时执行这里的 `backend/server.js`
和独立 C++ 服务 `control/bin/manta-control --simulation-udp 127.0.0.1:14560`。
外部适配器需要把 SITL MAVLink 接入这个 loopback UDP 端口并回送遥测。

配置入口：

- `MANTA_CONFIG_FILE`：覆盖 Node 和 C++ 服务主配置路径；优先于 `MANTA_CONFIG_PATH`。两者都不设置时使用 `config/system.config.json`。
- `MANTA_LOGS_DIR`：仅兼容保留的 Python 基线；原生服务日志由启动器/systemd 重定向。
- `MANTA_SIMULATION=1`：显式启用本地仿真外设隔离。

仿真配置要求 Web/桥接均绑定 `127.0.0.1`，桥接控制/遥测使用 `15551/15552`，
串口配置使用 `/dev/manta-sitl-virtual` 哨兵值。原生服务仅打开显式指定的 loopback UDP 仿真链路。
未满足条件时服务拒绝以仿真模式启动。硬件外设 API、外设进程清理和人脸模型常驻
在仿真模式下禁用。外部环境负责生成关闭云台、摄像头、热点和 Mission Planner 转发的配置。

这些入口不修改 UWB、恢复、推进或转向算法，不需要更改船端 systemd 服务。
正常船端运行无需设置任何上述环境变量。
