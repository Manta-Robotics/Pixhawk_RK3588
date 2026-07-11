# 快速开始

项目统一以 `/root/Pixhawk_RK3588/rk3588_control_system` 为运行目录，以 systemd 为唯一服务管理方式。

```bash
cd /root/Pixhawk_RK3588/rk3588_control_system
npm run check
npm test
bash quickstart.sh
```

安装完成后检查：

```bash
bash scripts/status_report.sh
systemctl status manta-backend.service manta-bridge.service manta-gimbal-stream.service
```

手机连接 `Manta-Control`，打开 `http://10.42.0.1:3000`。

接线、视频、4K、故障排查与安全要求见 [README.md](README.md)。
