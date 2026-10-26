# RK3576 云台四模式部署与验收

本工程目录名保留 `rk3588_control_system` 以兼容现有部署路径；实际芯片通过设备树识别。
四个模式分别是：点击居中、人脸跟踪、泳者跟踪、UWB 信标。板上应直接连接飞控和 SIYI 相机，不依赖电脑端的串口、视频或 WSL 转接进程。

## 板端配置

先在板上确认 `cat /proc/device-tree/model`、`uname -m`、`ls /dev/serial/by-id/`、`ip -br link`，再配置 `config/system.config.json`：

- `serial_port` 指向实际飞控串口。若使用板载 UART，`pixhawk.uart_overlay` 和 `boot_config` 必须属于具体 RK3576 板型；不能复用 RK3588 的 dtbo 名称。
- SIYI 网络相机使用 `gimbal.control_transport=udp`，相机地址、UDP 端口应与实机一致。当前工程的相机基线为 `192.168.144.25:37260`，RTSP 为 `rtsp://192.168.144.25:8554/main.264`。
- `gimbal.network_interface` 必须是接相机的真实板端网卡。旧板的 `br-manta` 不存在时路由服务会明确失败，不再静默跳过。
- 如果只接 SIYI 网络相机，设置 `camera.enabled=false`；该字段控制独立的板载 OV8858 摄像头，不控制 SIYI 视频。设置 `gimbal.auto_home_on_connect=false` 可避免连接时自动回中。
- 人脸配置可使用 `scripts/models/yolov8n-face-lindevs-rk3576-fp16.rknn`，CPU 回退权重为 `scripts/models/yolov8n-face-lindevs.pt`。RK3588 的 RKNN 模型不能直接当作 RK3576 模型使用；缺少匹配 NPU 模型时使用 CPU 回退。
- 泳者权重为 `scripts/best.pt`，通过项目虚拟环境中的 Ultralytics/PyTorch 运行。

可通过 `MANTA_CONFIG_FILE` 或 `MANTA_CONFIG_PATH` 指定配置。部署为 systemd 服务时，将同一个配置路径写入 `/etc/manta/manta.env`，后端、路由和媒体服务将读取相同配置。无需修改该文件中的已有设备凭据。

## 安装和检查

```bash
# 只有已核实设备树配置时才让安装器修改 boot 文件。
# 若内核中已启用串口，使用 --skip-boot-config 保留现有 boot 配置。
sudo bash quickstart.sh --skip-boot-config
bash scripts/python_service.sh scripts/check_gimbal_features.py --expect-soc rk3576 --load-models
```

安装器不会自动启动或重启服务。确认接线、配置和设备状态后，再启动：

```bash
sudo systemctl start manta-gimbal-route manta-gimbal-stream manta-bridge manta-backend
bash scripts/python_service.sh scripts/check_gimbal_features.py --expect-soc rk3576 --load-models
```

检查程序仅读取状态和视频，`--load-models` 使用黑色图像验证模型加载，不发送武装、推进或云台运动命令。检查通过表示四个功能的前置条件成立，不能替代下面的物理动作验收。

RKNN 模型转换环境中可以运行：

```bash
python scripts/convert_face_to_rknn.py --target rk3576
```

模型转换工具与板端 NPU 运行库是不同依赖；仅安装 `librknnrt.so` 不会自动安装 `rknnlite` Python 包。未配置 NPU 时可先使用安装好的 CPU 模型验证功能。

## 四模式实机验收

飞控保持解除武装，关闭船体自动跟随，仅测试云台：

1. 点击居中：选中模式后，再点击实际图像中的目标；确认目标向画面中心移动，云台姿态反馈同步变化。单击模式按钮本身不执行回中。
2. 人脸跟踪：相机内出现人脸并缓慢横向移动，确认检测目标与云台随动；遮挡或移出画面后应停止运动。启动接口只有收到模型就绪通知后才返回成功。
3. 泳者跟踪：使用泳者模型能识别的实际目标或测试素材，确认检测框与随动。空办公室或普通人脸不保证属于泳者模型的目标类别。
4. UWB 信标：先确认 `measurementStatus=trusted` 且测量持续更新，再启动云台 UWB 模式，缓慢移动标签检查方向；测量隔离、过期或丢失时必须停止转动。该模式与船体自动跟随开关独立。

仅视频正常不能证明云台控制可用。若显示 ERR，先检查 `/api/gimbal/state` 中的 `lastError`、UDP 地址、路由和真实姿态反馈；跟踪启动时缺 Python 包或模型会直接返回具体错误。
