# Manta RK3588 Control System

Manta 是运行在 LubanCat 5（RK3588）上的水面机器人控制系统，连接 Pixhawk、三轴云台、本地摄像头，并通过 Web、Wi-Fi 热点和蓝牙 PAN 提供操控、跟踪、录像与遥测能力。

当前标准部署位置为：

```text
/root/Pixhawk_RK3588/rk3588_control_system
```

## 快速入口

- [完整安装与使用说明](rk3588_control_system/README.md)
- [系统架构](rk3588_control_system/docs/ARCHITECTURE.md)
- [运行维护手册](rk3588_control_system/docs/OPERATIONS.md)
- [root 新板迁移说明](ROOT_MIGRATION_CN.md)

## 当前硬件基线

| 设备 | 接口 | 配置 |
| --- | --- | --- |
| Pixhawk TELEM2 | RK 物理针脚 8/10/6 | `/dev/ttyS1 @ 115200` |
| 云台 UART | RK 物理针脚 5/3/9 | `/dev/ttyS3 @ 115200` |
| 云台网络 | `eth0` | RK `192.168.144.101`，云台 `192.168.144.108` |
| 手机 Wi-Fi | `p2p0` | `Manta-Control`，控制地址 `10.42.0.1:3000` |
| 手机蓝牙 PAN | `manta-bt0` | 控制地址 `10.43.0.1:3000` |

## 开发检查

```bash
cd rk3588_control_system
npm ci
npm test
npm run check
```

所有运行服务由 systemd 管理。仓库中的 `start.sh`、`stop.sh` 和 `scripts/status_report.sh` 是统一运维入口，不再自行清理未知进程或抢占端口。

## 分支

当前新板迁移与持续开发分支：`codex/manta-new-board-migration`。

## License

MIT
