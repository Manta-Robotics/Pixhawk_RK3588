# Jetson USB UWB 接收与跟随

USB 接收链路为 `uwb_usb_reader.py → uwb_usb_source.js → worker → uwb_smoothed_output.js → 控制器/网页`。读取器只接收数据，不发送设备配置指令。37 字节定位帧和 18 字节心跳均校验 XOR，保留旧版 16 字节心跳兼容；分包、粘包和损坏数据会重新同步。

## 设备配置

仓库默认关闭 USB 输入，使用独立设备配置选择实际串口和信标 ID。例如，在设备私有配置中合并：

```json
{
  "uwb_usb": {
    "enabled": true,
    "port": "/dev/serial/by-id/REPLACE_WITH_RECEIVER_ID",
    "baud_rate": 115200,
    "receiver_id": 1,
    "target_tag_id": 123,
    "vessel_control_enabled": true,
    "speed_gate_enabled": false,
    "output_speed_gate_enabled": false
  },
  "uwb_follow": {
    "execution_mode": "relative_imu",
    "auto_resume_after_uwb_timeout": false,
    "uwb_loss_action": "last_point_model"
  }
}
```

`target_tag_id` 必须匹配实际信标，示例中的 123 需要替换。服务通过 `MANTA_CONFIG_FILE` 或 `MANTA_CONFIG_PATH` 读取设备配置。不要将完整私有配置、配对码、网络密码或实验时钟锚点提交到仓库。

两个速度开关控制原始/输出坐标的逐点速度硬拒收；关闭后仍保留协议校验、标签筛选、坐标范围检查和响应平滑。串口连续 3 秒没有字节会自动重开；有效心跳属于串口活动，信标遮挡不会触发无意义的反复重连。

## 定位丢失

默认 `uwb_loss_action: "stop"` 保持既有停止策略。选择 `last_point_model` 时必须使用 `relative_imu`，并关闭 `auto_resume_after_uwb_timeout`。

首次启动需要真实有效定位。已经启动且已武装的跟随会话丢失定位后，使用已施加油门和新鲜 IMU 航向，短时估算接近冻结的最后位置。不会更新真实 UWB 测量时间戳或把估算发布为真实定位。

- 当前设备的估算窗口截止最后真实测量后 1 秒；代码将可配置窗口限制在最多 2 秒。
- 估算油门不超过丢点前油门、15% 及模型计算上限；只在前方 ±45° 内接近。
- 使用跟随间距作为停止参考；当前配置为 0.5 米，模型到达阈值另加 0.05 米。
- 时间、估算路程或接近阈值到限后，持续更新中立输出，保留会话等待真实定位恢复。
- 飞控断开、解除武装、模式错误、IMU 失效、目标身份变化和急停仍会停止控制。

油门模型未经实测位移校准，不能保证准确到点；上述路程和速度属于模型估算，不能当作真实船速或位置传感器反馈。

## 定位与帧率诊断

`transport.connected` 表示 USB 串口连接。`heartbeatCount` 增长且 `goodCount` 不增长时，接收端未提供定位帧；心跳没有距离与角度，无法恢复真实坐标。此时先确认信标供电、设备绑定与无遮挡测距条件。

`positionRateHz` 是串口完整定位帧率，`processedRateHz` 是处理频率。网页有效输出频率应统计不同 `smoothing.outputMonotonicMs` 的新鲜输出；保留相同坐标值的真实新测量仍计作新输出。坐标数值变化频率受平滑和 10 cm 死区影响，应单独报告。

2026-11-01 的一次无遮挡实测：60 秒接收 2398 帧（39.97 Hz），网页新鲜独立输出 2096 帧（34.93 Hz），有效定位时间 100%；无心跳独占时段、校验错误或 USB 断开。该结果代表当次无线条件，不保证遮挡下仍达到相同帧率。

## 验证与回滚

按 `CONTRIBUTING.md` 执行 `npm ci`、安装 `requirements-test.txt` 并运行 `npm run maintenance`。USB 单元测试使用模拟输入或 PTY，不接管真实串口。丢点会话集成测试模拟执行器，不会武装或驱动硬件。

部署前备份源码和设备私有配置。回滚时恢复对应源码与配置并重启服务；只恢复代码会保留新的设备策略开关。运行中的飞控和动力测试必须由操作者在明确的硬件条件下另行完成。
