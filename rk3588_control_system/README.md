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

手机连接 `Manta-Control` 后应手动打开 `http://10.42.0.1:3000`。热点会正确响应 Android、Apple 和 Windows 的连通性探测，防止设备因“无互联网”自动离开控制网络。

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
npm run check:config
npm test
npm run check
python3 -m compileall -q backend scripts
```

配置校验会检查串口、端口、热点密码、RTSP 地址、手机预览参数、录像编码器和跟踪保持区。

## 9. 目录职责

```text
backend/        Web API、Socket.IO、MAVLink 桥接
config/         系统、马达、蓝牙配置
frontend/       控制台页面和静态资源
scripts/        安装、诊断、媒体和跟踪进程
systemd/        可部署的服务模板
test/           自动化检查
docs/           架构和运维文档
logs/           运行日志，不进入 Git
recordings/     录像文件，不进入 Git
```

## 10. 安全要求

- 首次电机测试必须移除推进器或断开动力输出。
- 串口接线或模式切换前先解除武装。
- 不要在运行中删除当前录像文件。
- 不要把热点密码、SSH 私钥或飞控参数备份提交到 Git。
- 修改控制算法后至少运行 `npm test`、`npm run check` 和板端状态检查。

## 11. 已知限制

- 当前可验证的云台 `/live/0` 码流为 1080p；4K 必须由云台固件提供真实高分辨率流。
- OV8858 本地摄像头依赖板级 overlay 与实际传感器状态。
- iOS 对 Linux 蓝牙 PAN 的支持不稳定，Wi-Fi 热点是推荐的手机控制路径。
