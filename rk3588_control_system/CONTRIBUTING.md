# Manta 贡献与维护指南

## 开发分支与提交

1. 在 `codex/manta-new-board-migration` 或独立功能分支开发，不直接在运行中的 `main` 上堆叠未验证改动。
2. 一个提交只处理一个可解释的问题；提交信息使用动词开头，并说明影响的子系统。
3. Push 后通过 Pull Request 合并到 `main`，在 PR 中记录验证结果和回滚方式。

## 提交前检查

```bash
cd rk3588_control_system
npm ci
npm run maintenance
```

`npm run maintenance` 只执行无硬件副作用的测试和静态检查。涉及硬件时，还需要按改动范围完成板端验证：

- Pixhawk/MAVLink：检查串口、心跳、遥测、武装状态和失联行为。
- 电机控制：移除推进器或断开动力输出后测试，确认中值、方向、限幅和急停。
- 相机/云台：检查 health、预览、录像、停止与异常恢复。
- 热点/蓝牙/路由：保留至少一条维护链路，确认 SSH 不会被同时切断。
- systemd：确认 `WorkingDirectory`、`ExecStart`、重启策略、日志和目标服务状态。

## 修改规则

- 新增配置字段时，同步更新配置文件、`scripts/validate_config.mjs`、测试和 README。
- 修改 HTTP API 或 Socket.IO 事件时，同步更新前端调用方和兼容逻辑。
- 修改 systemd 模板后，不要只编辑 `/etc/systemd/system`；通过安装脚本重新部署模板。
- 不提交热点密码、SSH 密钥、飞控参数备份、录像、日志、临时 bundle 或设备专属凭据。
- 模型文件变更必须记录来源、许可证、输入输出、类别定义、目标硬件和性能基线。

## PR 描述最小内容

- 变更目的和范围。
- 影响的目录、服务和接口。
- 已执行的自动化与实机检查。
- 已知风险、未覆盖项和回滚步骤。
- 如果改变安全行为，说明失联、急停、解除武装和动力输出的验证结果。
