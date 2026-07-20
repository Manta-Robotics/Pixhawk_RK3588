# 系统架构

## 服务边界

| 服务 | 监听/设备 | 责任 |
| --- | --- | --- |
| `manta-backend` | TCP 3000、UDP 14551/14552、`/dev/ttyS3` | Web、控制 API、云台命令、录像 |
| `manta-bridge` | `/dev/ttyS1`、UDP 14551/14552 | Pixhawk MAVLink 桥接 |
| `manta-gimbal-stream` | TCP 8091、云台 RTSP | 原始识别流与手机低码率流 |
| `manta-camera` | TCP 8090、V4L2 | OV8858 快照和 MJPEG |
| `manta-gimbal-route` | `eth0` | 配置云台专用网段和主机路由 |
| `manta-hotspot` | `p2p0` | Wi-Fi AP 与 DHCP |
| `manta-captive-portal` | TCP 80 | 控制入口和系统连通性探测 |
| `manta-bluetooth-pan` | `manta-bt0` | 蓝牙 NAP、DHCP 和控制网段 |

## 控制链路

```text
Browser -> Socket.IO/REST -> backend/server.js
        -> UDP command -> mavlink_bridge.py -> /dev/ttyS1 -> Pixhawk
```

后端负责输入限幅、状态聚合和命令审计；MAVLink 桥接只负责协议与串口边界。

## 云台链路

```text
RTSP full resolution -> gimbal_rtsp_stream_server.py
  |-- /stream.mjpg -> face_track.py / infer_video.py
  `-- /mobile.mjpg -> backend /api/gimbal/stream -> phone

backend/server.js -> /dev/ttyS3 -> gimbal controller
backend/server.js -> FFmpeg h264_rkmpp -> recordings/gimbal/*.mp4
```

预览与识别使用不同输出，手机带宽优化不会改变检测器坐标系和云台标定。

## 配置原则

- 设备和端口位于 `config/system.config.json`。
- 电机通道位于 `config/motor_config.json`。
- 蓝牙角色位于 `config/bluetooth.config.json`。
- 服务模板不得包含固定工程路径，安装时替换 `__PROJECT_DIR__`。
- 运行产物、录像、日志和离线 wheelhouse 不进入 Git。

## 失败隔离

- 各媒体、桥接和 Web 进程由独立 systemd 服务重启。
- 云台 RTSP 断流只重启媒体代理，不重启控制后端。
- 手机网络与核心控制服务分离，默认停止控制服务时保留维护链路。
- 配置损坏时后端直接退出，避免以危险默认值继续控制硬件。
