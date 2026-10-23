# Manta 云台健康与疑似卡死诊断规范 v1

## 1. 文档状态

- 版本：1.0
- 适用范围：当前 RK3588 上位机、云台串口协议、Web Preview 和后续 iOS/iPadOS App
- 当前实现状态：算法设计，尚未接入后端自动处置
- 安全结论：现阶段只能报告 `suspected_jam`（疑似卡死），不能报告硬件确定性卡死

本规范利用当前已经存在的云台指令、角度反馈、陀螺仪反馈、校验状态和时间戳，判断云台是否在收到明确运动指令后长期没有产生合理响应。由于系统目前没有云台电机电流、驱动器温度、绕组温度、扭矩、独立编码器或已解码的电机故障码，算法结论必须保持为“疑似”。

## 2. 目标与非目标

### 2.1 目标

1. 识别串口失联、反馈陈旧、校验连续失败和运动响应异常。
2. 区分“通信故障”“反馈质量下降”“到达软限位”和“疑似机械/电机卡死”。
3. 在高置信度疑似卡死时，立即停止云台运动、跟踪、录像和后续控制指令。
4. 生成可复现的结构化诊断报告，供 App 展示、日志归档和工程复盘。
5. 控制误报，避免把回中、刹车、软限位、低速微调或目标丢失误判为卡死。

### 2.2 非目标

1. 不修改现有云台串口协议和控制算法。
2. 不从视频画面是否变化推断云台电机状态。
3. 不依据未解码的 `selfTest`、`status` 或 `status2` 位直接宣布硬件故障。
4. 不自动触发推进电机急停；云台故障与机器人推进安全策略保持独立。
5. 不将 HTTP/Socket.IO 的“请求成功”视为云台执行确认。

## 3. 当前可用输入

### 3.1 云台状态

来源：

- `GET /api/gimbal/state`
- Socket.IO `gimbal_state`

可用字段：

```text
connected
mode
lastCommand
lastError
lastTarget
trackingActive
trackWorkerActive
trackStatus
limits
updatedAt
feedback.selfTest
feedback.status
feedback.status2
feedback.servoMode
feedback.yawDeg
feedback.pitchDeg
feedback.rollDeg
feedback.gyroYawDps
feedback.gyroPitchDps
feedback.gyroRollDps
feedback.checksumValid
feedback.updatedAt
```

### 3.2 运动命令证据

可结合以下信息判断“是否明确要求云台运动”：

- App 刚刚发送的 `/api/gimbal/click` 请求及其 `delta` 响应。
- Socket.IO `gimbal_target` 中的 `rateX`、`rateY`、`desiredRateX`、`desiredRateY`。
- `gimbal_state.lastCommand`、`mode` 和 `lastTarget`。
- `/api/gimbal/home`、`/api/gimbal/stop`、跟踪启停的本地发送时间。

仅依赖 `lastCommand` 不足以精确还原所有期望运动。正式实现时，诊断器必须在命令发送处同步记录一份标准化 `CommandSnapshot`，不能事后从文本日志猜测命令。

建议结构：

```json
{
  "commandId": "uuid",
  "kind": "click|track|home|stop|camera_only",
  "issuedAt": 0,
  "expectedYawRateDps": 0,
  "expectedPitchRateDps": 0,
  "targetYawDeg": null,
  "targetPitchDeg": null,
  "holdMs": 0,
  "source": "ios|preview|tracker|backend"
}
```

### 3.3 不能作为硬件确认的证据

- REST `success: true`
- Socket.IO `rover_drive_ack` 或其他发送层 ACK
- `lastCommand` 已更新
- 视频画面仍在刷新或已经停止刷新
- 单次角度或陀螺仪采样

这些证据最多证明上位机已经接受或尝试发送指令。

## 4. 采样与派生指标

诊断器建议以 20 Hz 运行；最低不得低于 10 Hz。保存最近 5 秒环形缓冲区，单个样本包括命令快照、云台状态和反馈。

每次评估计算：

```text
feedbackAgeMs       = now - feedback.updatedAt
yawDeltaDeg         = yaw(end) - yaw(start)
pitchDeltaDeg       = pitch(end) - pitch(start)
measuredYawRateDps  = 窗口内 gyroYawDps 的中位数绝对值
measuredPitchRateDps= 窗口内 gyroPitchDps 的中位数绝对值
expectedMagnitude   = hypot(expectedYawRateDps, expectedPitchRateDps)
measuredMagnitude   = hypot(measuredYawRateDps, measuredPitchRateDps)
directionDot        = expectedRate 与 measuredRate 的归一化点积
validRatio          = checksumValid 样本数 / 窗口样本总数
```

角度差必须处理跨界和异常跳变；单个采样跳变超过 30° 时应标记为无效样本，不参与卡死判定，并单独记录 `feedback_jump`。

## 5. 默认阈值

阈值必须集中配置并写入诊断报告，禁止散落在 UI 代码中。

| 参数 | v1 默认值 | 含义 |
| --- | ---: | --- |
| `evaluation_hz` | 20 Hz | 诊断循环频率 |
| `warmup_ms` | 2000 ms | 连接后不进行卡死判断 |
| `feedback_stale_warn_ms` | 500 ms | 反馈开始陈旧 |
| `feedback_lost_ms` | 1200 ms | 判定反馈通信丢失 |
| `bad_checksum_consecutive` | 3 | 连续校验错误警告阈值 |
| `bad_checksum_ratio` | 30% / 1 s | 反馈质量下降阈值 |
| `motion_grace_ms` | 250 ms | 指令发出后的机械响应宽限 |
| `motion_window_ms` | 750 ms | 单个运动响应判断窗口 |
| `min_expected_rate_dps` | 8°/s | 低于此速率不判断卡死 |
| `min_expected_angle_deg` | 3° | 角度目标变化小于此值不判断 |
| `max_no_motion_gyro_dps` | 1°/s | 近似无运动的陀螺仪阈值 |
| `max_no_motion_delta_deg` | 0.5° | 单窗口近似无角度变化阈值 |
| `jam_windows_required` | 3 | 连续命中后进入疑似卡死 |
| `jam_evidence_horizon_ms` | 2500 ms | 三个证据窗口允许的总时间 |
| `wrong_direction_dot` | -0.35 | 明显反向运动阈值 |
| `wrong_direction_ms` | 500 ms | 反向运动持续时间 |
| `soft_limit_guard_deg` | 5° | 临近限位时禁止卡死判断 |
| `recovery_good_windows` | 3 | 人工恢复检查需要的健康窗口 |

这些值是保守初值，必须经过台架数据标定。不得未经测试直接作为量产阈值。

## 6. 状态机

| 状态 | 说明 | 允许的主要迁移 |
| --- | --- | --- |
| `DISCONNECTED` | 串口未连接或云台不可用 | `WARMING_UP` |
| `WARMING_UP` | 连接后等待稳定反馈 | `HEALTHY`、`COMMUNICATION_FAULT` |
| `HEALTHY` | 反馈正常，无待评估运动 | `OBSERVING_MOTION`、`DEGRADED`、`COMMUNICATION_FAULT` |
| `OBSERVING_MOTION` | 存在满足阈值的运动命令 | `HEALTHY`、`DEGRADED`、`SUSPECTED_JAM` |
| `DEGRADED` | 校验错误、反馈抖动或短时陈旧 | `HEALTHY`、`COMMUNICATION_FAULT` |
| `COMMUNICATION_FAULT` | 反馈超时或串口错误 | `STOPPING` |
| `SUSPECTED_JAM` | 连续窗口显示明确命令但无运动 | `STOPPING` |
| `STOPPING` | 正在执行自动安全处置 | `LATCHED_FAULT` |
| `LATCHED_FAULT` | 故障已锁存，禁止自动恢复控制 | `RECOVERY_CHECK` |
| `RECOVERY_CHECK` | 用户明确发起恢复测试 | `HEALTHY`、`LATCHED_FAULT` |

### 6.1 关键原则

- `COMMUNICATION_FAULT` 与 `SUSPECTED_JAM` 必须使用不同故障码。
- 无反馈不能推断电机卡死，只能报告通信故障。
- 疑似卡死一旦锁存，反馈偶然恢复也不能自动恢复控制权。
- 恢复必须由用户明确发起，并在无人员、无障碍物的安全条件下完成低速自检。

## 7. 疑似卡死判定

### 7.1 判定前置条件

只有同时满足以下条件才进入运动观察：

1. `connected == true`。
2. `lastError` 为空。
3. `feedbackAgeMs <= 500`。
4. 当前窗口 `validRatio >= 0.9`。
5. 已度过连接预热期和命令宽限期。
6. 命令明确要求运动，且期望速率或目标角差达到阈值。
7. 对应轴不在软限位保护区内，也没有继续朝限位方向运动。
8. 当前不是 `stop`、刹车、相机参数、录像启停、OSD 或纯检测器命令。

### 7.2 单轴无响应证据

以 yaw 轴为例，在 750 ms 窗口中同时满足：

```text
abs(expectedYawRateDps) >= 8
median(abs(gyroYawDps)) < 1
abs(yawDeltaDeg) < 0.5
```

则记录一次 `yaw_no_motion_evidence`。pitch 轴使用相同逻辑。

对于角度目标命令，如果无法获得可靠期望速率，则要求：

```text
abs(targetYawDeg - windowStartYawDeg) >= 3
abs(yawDeltaDeg) < 0.5
median(abs(gyroYawDps)) < 1
```

### 7.3 疑似卡死

同一轴在 2.5 秒内连续三个有效窗口产生无响应证据，且中间没有 stop、反向命令、反馈中断或到达限位，则进入：

```text
classification = suspected_jam
confidence = high
axis = yaw | pitch | both
```

即使置信度为 high，报告仍必须使用 `suspected_jam`，不得改写为 `confirmed_jam`。

### 7.4 反向与异常运动

若期望速率和实测速率的归一化点积持续低于 `-0.35` 达 500 ms，报告：

```text
classification = direction_mismatch
```

该状态需要立即停止云台，但不能等同于卡死。它可能来自轴符号、安装方向、反馈映射或固件配置错误。

## 8. 必须抑制的误报场景

以下情况不得累计卡死证据：

1. 连接后的前 2 秒。
2. `/api/gimbal/home` 发出后的前 1.5 秒。
3. `stop`、取消跟踪或反向制动期间。
4. 目标进入 deadzone 或跟踪状态为 lost/holding。
5. 命令低于最小期望速率。
6. 云台距离 yaw/pitch 软限位小于 5° 且命令继续指向限位。
7. feedback 陈旧、校验失败或采样率不足。
8. 用户仅执行录像、OSD、测距、变焦复位等非运动命令。
9. 同一窗口内发生模式切换或新的相反方向命令。
10. App 进入后台、连接正在切换或控制权已经释放。

## 9. 自动处置

### 9.1 处置触发

以下状态进入自动处置：

- `SUSPECTED_JAM`
- `COMMUNICATION_FAULT`
- `direction_mismatch`
- 明确的串口 `lastError`

### 9.2 v1 处置顺序

1. 在诊断器本地立即锁住云台控制入口，拒绝新的 click、track、home 和相机动作。
2. 记录故障前至少 2 秒、故障后至少 1 秒的状态缓冲区。
3. 调用 `POST /api/gimbal/stop`。
4. 调用 `POST /api/gimbal/track/stop`，允许接口幂等失败。
5. 如果正在录像，调用 `POST /api/gimbal/recording/stop`。
6. 调用 `POST /api/gimbal/disconnect`，阻止继续写串口。
7. 将状态锁存为 `LATCHED_FAULT`，在 App 中显示不可忽略的云台故障面板。
8. 保留机器人推进控制，但持续显示“云台不可用”；是否停车由更高层整机安全策略决定。

每个接口返回成功只代表处置请求已被上位机接受。诊断报告必须继续记录反馈是否停止以及串口是否断开，不能将 HTTP 200 记作硬件处置确认。

### 9.3 App 提示

建议文案：

```text
云台疑似卡死，已停止云台控制
请检查是否有异物阻挡、线缆牵拉或机械限位。当前结论来自运动反馈推断，并非硬件确定性故障。
```

不得显示：

```text
云台电机已损坏
云台确定卡死
驱动器故障
```

除非未来取得厂商定义的确定性故障码并完成验证。

## 10. 恢复流程

1. 用户必须主动点击“开始安全检查”。
2. App 显示清空周边、检查异物、将机器人置于稳定表面的提示。
3. 重新连接串口并等待 2 秒稳定反馈。
4. 先进行不运动的反馈完整性检查。
5. 分别以不超过 5°/s 的低速短脉冲测试 yaw 和 pitch，每次不超过 250 ms。
6. 每个轴连续三个窗口响应正常，才允许清除锁存。
7. 任一轴再次无响应，立即停止并保持 `LATCHED_FAULT`。

v1 不允许自动重连后直接恢复跟踪或回中。

## 11. 诊断报告格式

建议 JSON 顶层字段：

```json
{
  "reportVersion": "gimbal-health-v1",
  "reportId": "uuid",
  "createdAt": "ISO-8601",
  "device": {
    "model": "unknown",
    "serial": "unknown",
    "appVersion": "",
    "backendVersion": "",
    "configVersion": ""
  },
  "classification": "suspected_jam",
  "certainty": "inferred_not_hardware_confirmed",
  "severity": "critical",
  "axis": "yaw",
  "stateBefore": "OBSERVING_MOTION",
  "stateAfter": "LATCHED_FAULT",
  "summary": "Explicit yaw motion command produced no measurable response in three consecutive windows.",
  "thresholds": {},
  "command": {},
  "evidence": {},
  "gimbalState": {},
  "actions": [],
  "samples": [],
  "limitations": []
}
```

### 11.1 `evidence` 字段

```text
windowStart
windowEnd
feedbackAgeMs
sampleCount
validSampleCount
validRatio
expectedYawRateDps
expectedPitchRateDps
measuredYawRateMedianDps
measuredPitchRateMedianDps
yawDeltaDeg
pitchDeltaDeg
directionDot
noMotionWindowCount
softLimitDistanceYawDeg
softLimitDistancePitchDeg
lastError
rawSelfTest
rawStatus
rawStatus2
```

### 11.2 `actions` 字段

每个自动动作记录：

```text
name
requestedAt
endpoint
httpStatus
accepted
responseMessage
observedEffect
completedAt
```

`observedEffect` 必须与接口接受状态分开记录。

### 11.3 隐私与体积

- 默认只保存故障前 2 秒、后 1 秒的必要数值样本。
- 不在报告中保存用户视频、账号信息、Wi-Fi 密码或蓝牙配对码。
- `lastRx.hex` 仅在工程诊断模式下截取有限长度，不默认上传。

## 12. 测试矩阵

| 编号 | 场景 | 操作 | 预期结果 |
| --- | --- | --- | --- |
| T01 | 正常静止 | 无运动命令保持 30 秒 | `HEALTHY`，无卡死证据 |
| T02 | 正常 click | 多方向点击移动 | 有角度/gyro 响应，返回 `HEALTHY` |
| T03 | 正常跟踪 | 人脸/游泳者持续移动 | 不误报，目标丢失时抑制判断 |
| T04 | 低速微调 | 期望速率低于 8°/s | 不进入卡死判断 |
| T05 | 回中 | 执行 home | 1.5 秒保护期内不误报 |
| T06 | 软限位 | 驱动到 yaw/pitch 限位附近 | 报告 `soft_limit` 或保持健康，不报卡死 |
| T07 | 串口拔出 | 运动前后断开串口 | `COMMUNICATION_FAULT`，不报卡死，执行停止流程 |
| T08 | 反馈冻结 | 保持连接标志但停止反馈 | feedback 超时，报告通信故障 |
| T09 | 校验损坏 | 注入连续错误校验帧 | `DEGRADED`，达到超时后通信故障，不报卡死 |
| T10 | yaw 物理阻挡 | 安全台架上限制 yaw 并发出明确命令 | 三个窗口后 `suspected_jam/yaw`，自动处置 |
| T11 | pitch 物理阻挡 | 安全台架上限制 pitch | `suspected_jam/pitch`，自动处置 |
| T12 | 双轴阻挡 | 同时限制 yaw/pitch | `suspected_jam/both` |
| T13 | 方向配置错误 | 注入与命令相反的反馈 | `direction_mismatch`，不得报告卡死 |
| T14 | 反馈高延迟 | 注入 300–900 ms 抖动延迟 | 先 `DEGRADED`；不得在无有效反馈时报卡死 |
| T15 | 快速反向 | 连续改变方向 | 切换窗口清零，不误报 |
| T16 | stop 中制动 | 高速运动后立即 stop | 制动保护期内不误报 |
| T17 | 录像中卡死 | 录像时触发 T10 | 停运动、停跟踪、停录像、断开串口并生成报告 |
| T18 | App 退后台 | 运动中触发后台/失焦 | 先发 stop；不得继续累计旧命令证据 |
| T19 | 故障后自动恢复 | 解除阻挡但不操作 App | 保持 `LATCHED_FAULT` |
| T20 | 人工恢复检查 | 按恢复流程执行低速测试 | 三个健康窗口后才允许清除锁存 |

物理阻挡测试必须在无人员、低速、短脉冲、可立即断电的台架环境中进行。禁止在开放水域首次标定卡死阈值。

## 13. 伪代码

```text
onSample(now, state, command):
  appendRingBuffer(now, state, command)

  if !state.connected:
    transition(DISCONNECTED)
    return

  if now - connectedAt < warmupMs:
    transition(WARMING_UP)
    return

  if state.lastError is not empty:
    raise(COMMUNICATION_FAULT, serial_error)
    stopAndLatch()
    return

  if feedback missing or feedbackAgeMs > feedbackLostMs:
    raise(COMMUNICATION_FAULT, feedback_timeout)
    stopAndLatch()
    return

  if checksum quality is bad:
    transition(DEGRADED)
    clearJamEvidence()
    return

  if !motionIsEligible(command, state):
    transition(HEALTHY)
    clearExpiredEvidence()
    return

  transition(OBSERVING_MOTION)
  window = validSamples(last motionWindowMs)

  if noMotionEvidence(window, command, yaw):
    addEvidence(yaw)
  else:
    clearEvidence(yaw)

  if noMotionEvidence(window, command, pitch):
    addEvidence(pitch)
  else:
    clearEvidence(pitch)

  if directionMismatch(window, command):
    raise(direction_mismatch)
    stopAndLatch()
    return

  if consecutiveEvidence(axis) >= jamWindowsRequired:
    raise(suspected_jam, axis, inferred_not_hardware_confirmed)
    stopAndLatch()
```

## 14. 验收标准

1. T01–T06、T14–T16 连续运行至少 100 次，无卡死误报。
2. T10–T12 在安全台架上至少各执行 30 次，检测率达到 95% 以上。
3. 从第三个有效证据窗口结束到本地控制锁止不超过 100 ms。
4. 自动处置的每一步都有独立时间戳、接口结果和观察结果。
5. 故障报告始终包含 `certainty: inferred_not_hardware_confirmed`。
6. 无反馈、坏校验或串口错误只能分类为通信/反馈故障，不能分类为卡死。
7. 故障锁存不会因反馈自行恢复而自动解除。

## 15. 当前传感器与协议限制

当前系统缺少以下硬件级证据：

- 云台各轴电机电流和相电流；
- 驱动器过流、堵转和过温故障位；
- 电机或驱动板温度；
- 扭矩估计；
- 独立于云台控制器的轴编码器；
- 已经由厂商文档确认的 `selfTest/status/status2` 位定义；
- 云台命令执行 ACK 和命令序列号；
- RK3588 与云台控制器之间的统一时钟；
- 机械限位开关或碰撞传感器。

角度与陀螺仪反馈可能来自同一控制器，不能被视为两个完全独立的硬件证据。视频冻结也可能由 RTSP、MJPEG、网络或解码器造成，不能作为电机卡死证据。

只有在未来加入并验证下列至少一类证据后，才可以讨论 `confirmed_hardware_fault`：

1. 厂商明确文档化的堵转/驱动故障码；
2. 持续高电流或过流故障，同时独立编码器确认无运动；
3. 驱动器硬件故障引脚或安全控制器报告；
4. 经 HIL 与实机测试验证的多传感器一致性结论。

在此之前，所有用户界面、日志、测试报告和 API 都必须使用“疑似卡死”或“运动响应异常”。
