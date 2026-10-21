# 快速开始

项目以 Git 仓库所在目录为运行目录，不绑定固定绝对路径；systemd 是唯一服务管理方式。仓库部署的是 MANTA 应用，不负责烧写 RK3588 的 Ubuntu 系统镜像。新板需先安装兼容的 LubanCat Ubuntu 22.04/24.04 aarch64 镜像并联网。

```bash
git clone <你的仓库地址> Pixhawk_RK3588
cd Pixhawk_RK3588/rk3588_control_system
sudo bash quickstart.sh
```

安装会为每块板单独生成热点密码和蓝牙 PIN，保存到权限为 `0600` 的 `/etc/manta/manta.env`；Git 中不保存通用密码。脚本只安装并启用服务，不会启动 MANTA 服务或自动重启板子。

默认硬件接口与原板一致：Pixhawk TELEM2 使用物理针脚 8/10/6、`/dev/ttyS1 @ 115200`，安装时启用 UART1 overlay；云台使用物理针脚 5/3/9、`/dev/ttyS3 @ 115200`，安装时启用 UART3 overlay。

网页前端提供设备状态、控制、视频、云台和 GPS 地图。地图与运行资源由 MANTA 板本机提供。

只做无副作用检查：

```bash
bash quickstart.sh --check-only
```

已有依赖和离线包时可使用 `--offline`。如果暂不确认相机/UART overlay，增加 `--skip-boot-config`。

安装完成后检查：

```bash
bash scripts/status_report.sh
sudo bash scripts/python_service.sh scripts/manta_doctor.py --installed
systemctl status manta-backend.service manta-bridge.service manta-gimbal-stream.service manta-mediamtx.service
```

手机连接 `Manta-Control`，打开 `http://10.42.0.1:3000`。

接线、视频、4K、故障排查与安全要求见 [README.md](README.md)。
