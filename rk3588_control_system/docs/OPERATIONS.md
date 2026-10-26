# 运行维护手册

## 日常检查

```bash
cd /root/Pixhawk_RK3588/rk3588_control_system
git status --short --branch
npm run check
npm test
bash scripts/status_report.sh
```

## 服务管理

```bash
bash start.sh
bash stop.sh
sudo systemctl restart manta-backend.service
sudo journalctl -u manta-backend.service -n 100 --no-pager
```

`stop.sh` 默认保留 Wi-Fi 和蓝牙。只有在现场仍有有线维护连接时才使用 `stop.sh --all`。

## 接口检查

```bash
curl -fsS http://127.0.0.1:3000/health
curl -fsS http://127.0.0.1:8091/health
curl -I http://10.42.0.1/generate_204
python3 backend/mavlink_bridge.py --help
```

## 云台网络

```bash
cat /sys/class/net/eth0/carrier
ip -br addr show eth0
ping -c 2 192.168.144.108
bash scripts/probe_gimbal_rtsp.sh
```

`carrier=0` 表示物理链路没有建立，软件无法切换或探测 4K 码流。

## 录像

```bash
ffprobe -v error -show_entries stream=codec_name,width,height -of default=noprint_wrappers=1 recordings/gimbal/example.mp4
```

录像应为 H.264，分辨率与云台 RTSP 源一致。删除录像前先停止录制并确认文件可播放。

## 发布流程

1. 运行配置校验、Node 测试和 Python 编译检查。
2. 查看 `git diff --check`，确认没有把录像、日志和模型 wheelhouse 纳入提交。
3. 在板子上重启受影响的单个服务。
4. 验证 Pixhawk、云台、手机预览和下载。
5. 提交并推送 `codex/manta-new-board-migration`。

## 故障优先级

1. 先解除武装并停止推进器输出。
2. 检查物理链路和 systemd 状态。
3. 检查 `/health` 与日志。
4. 最后才修改参数或重启服务；不自动重启整块板子。
