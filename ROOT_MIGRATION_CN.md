# Manta 新板迁移说明

当前开发和运行入口已经统一到 root 用户：

- SSH/VSCode Remote：`manta`
- 板端工程目录：`/root/Pixhawk_RK3588`
- 控制台入口：`http://192.168.137.222:3000`
- 对外保留端口：`22`、`3000`

旧的 `cat` 用户不再作为开发入口使用。`/home/cat/manta_backups` 里保留的是迁移前的工程快照，仅用于追溯；当前运行脚本、服务和 VSCode 配置都应以 `/root/Pixhawk_RK3588` 为准。

如需清理旧用户，建议先确认 `/home/cat/manta_backups` 不再需要，再单独归档或删除。
