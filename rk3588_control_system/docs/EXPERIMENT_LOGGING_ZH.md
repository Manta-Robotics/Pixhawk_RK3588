# 实验日志与录像对齐

在船端 Web 首页打开「输出日志」。九个模块共用同一份实时数据。展开模块可以查看最近有效样本、数据年龄和活动告警，在「完整日志」按模块和级别筛选。配对后的只读页面可以查看与导出，启停记录、录像或跟踪沿用设备控制权。

「控制模式」显示船端选中的跟随定位来源。在历史事件中可查看该来源的切换请求和生效结果。

一次实验可包含多段录像。选择一个事件跳到录像，在播放器暂停或拖动后，可按画面反查船体状态。重叠录像和重复播放时间会要求选择具体段或显示序号。断流区间、媒体缺失、时钟未知在查询结果中分别标记。

| 文件 | 水测后可回答的问题 |
| --- | --- |
| `state_overview.csv` | 每 50 ms，当时已经到达的飞控、UWB、云台、识别与电机数据是什么？ |
| `events.csv` | 哪次模式切换已生效？同一告警何时开始、升级或恢复？ |
| `video_aligned.csv` | 当前成片的第几帧对应哪一时刻？这张画面的识别结果何时才返回？ |

[带编号短片、三张表与示例会话](../scripts/experiment_logger/example/README.md)

## 记录器与历史数据

Web 的实验记录按钮与 `bash scripts/experiment_logger/run.sh` 使用同一 Python 写者。关闭浏览器后实验继续。用 SIGINT/SIGTERM 可结束命令行启动的实验。一次实验跨诊断分片保留相同会话 ID。历史录像沿用原播放路径，旧实验仍可用 `check_logs.py` 检查。

配置位于 `scripts/experiment_logger/config.json`，本平台默认位置为 `./data/experiment_logs`，设备可用 `MANTA_LOGGER_CONFIG` 指定私有配置。将 `journal_socket` 或环境变量 `MANTA_JOURNAL_SOCKET` 指向 Node 与 Python 均可访问的本机 Unix socket 目录，可以让写入状态在数据盘故障时继续上报。改变日志路径只影响使用该配置的记录器与历史入口。部署时为运行控制服务的账户准备可写的日志与运行目录。

## 数据来源与校准

同次系统启动以 Linux 单调时钟关联数据。逐帧索引使用成片显示时间，源帧身份沿共用采集链传递。MP4 的片头空白计入播放时间。缺少实际显示时间戳的画面保留显示序号，播放时间留空。主机解码时刻标为 `host_decode`。无法确认时的逐帧身份、实验时间或传感器值留空。界面区分实测、上次有效值、预测和未测量。

板端使用运行配置 `experiment.calibration` 登记已核实的安装参数。IMU 的 `imu_frd_axes` 依次指定船体前、右、下对应的传感器轴，取带符号的 1、2、3。例如，已证实传感器 XYZ 正好分别指前、右、下时可填 `[1,2,3]`。UWB 与云台的 `uwb_calibration_id`、`gimbal_calibration_id` 记录实际采用的方位标定版本。ESC 的 `esc_rpm_units` 接受经设备协议确认的 `erpm` 或 `mechanical`，`pole_pairs.left/right` 为实测电机极对数。初始配置将这些设备资料留空。现场仍需测量更新频率、安装零偏、相机相对主机的时间误差，以及共同视频入口在目标分辨率下的持续帧率。

`config/experiment_schema.json` 定义三表与默认诊断阈值（毫秒）。设备可在 `experiment.diagnostic` 覆盖诊断阈值。受控接口为 `POST /api/experiments/start`、`POST /api/experiments/stop`。配对后的只读接口包括 `GET /api/experiments/live`、`/sessions`、`/sessions/:id`、`/sessions/:id/events`、`/sessions/:id/files/{overview,events,video}`、`/sessions/:id/locate`。列表和事件支持分页。

## 自主运镜接口

模块视图中预留「自主运镜」。配对后通过 `GET /api/cinematography/state` 读取可用状态、模式列表和可执行操作。当前集成状态为「未接入」。接入板端策略时，给 `installCinematographyApi(app, provider)` 提供 `state()`、`start({mode})`、`stop()`。状态结构为 `{schema_version,available,state,mode,reason,modes,can_start,can_stop}`，模式列表由策略提供。状态取策略端反馈。写操作沿用控制权检查，对应 `POST /api/cinematography/start` 和 `POST /api/cinematography/stop`，未接入时返回 409。现有跟随和云台仍由各自入口操作。

## 本地验证

从 `rk3588_control_system` 运行以下命令。编号短片用例需要 FFmpeg、FFprobe 及测试依赖。

```bash
npm ci
python3 -m pip install -r requirements-test.txt
npm run maintenance
```
