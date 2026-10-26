# 独立 C++ 控制服务

`control/bin/manta-control` 是 MAVLink 串口、UWB 滤波/双接收器融合、传统与神经网络跟随、预测恢复、NMPC、手动推进限幅和命令超时的唯一运行进程。Node.js 保留 HTTP/Socket.IO、云台、视频、地图资源、录像、标定入口和 CSV 管理；Python 保留视觉及媒体工作进程。

前端继续使用既有 React + TypeScript 原样迁移页面。本次不修改布局、样式、图标、横屏规则或交互代码。

## 构建与安装

在 RK3588 Ubuntu 或 Linux 开发机运行：

```bash
sudo apt-get install build-essential cmake nlohmann-json3-dev python3-venv
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-test.txt
bash scripts/build_native_control.sh
npm ci
npm run maintenance
```

MAVLink C 头文件从安装的 pymavlink 官方 ardupilotmega XML 在构建时生成。部署后的 C++ 进程不调用 Python、Node 或 JS 引擎。模型沿用 `models/smart_usv_uwb_policy.json`，由 C++ 校验并推理。`scripts/install.sh` 包含编译依赖及构建步骤。离线构建需要预装这些软件包并准备 Python wheelhouse 与 npm 缓存。

完整安装仍使用 `sudo bash scripts/install_boot_services.sh`。控制服务沿用原有 **manta-bridge.service** 名称，以兼容 start/stop/status/doctor 和现有运维命令，其 ExecStart 已切换为 `scripts/native_control_service.sh`。不存在同时运行 Python bridge 的第二个服务。安装与编译不会自动启动、重启或解锁设备。

```bash
# 不访问串口的配置检查
control/bin/manta-control --config config/system.config.json --check-config
# 运行时操作；部署后由操作者选择执行
sudo systemctl restart manta-bridge.service
sudo systemctl restart manta-backend.service
sudo systemctl status manta-bridge.service manta-backend.service
```

更新运行程序通过原子替换完成，已运行进程继续使用旧映像，直到显式重启。项目目录是 systemd WorkingDirectory，模型的相对路径以此解析。运行用户需要串口访问权限；安装器将其加入 dialout 组。

## 进程边界与行为

- UDP 命令口默认 `127.0.0.1:14551`，遥测默认 `127.0.0.1:14552`。保留原有遥测字段及 Web API，新增 requestId 对应的命令响应。
- Web 只发送 MANUAL_TARGET、FOLLOW_START/STOP/METHOD 等操作请求。串口收发、跟随、手动加减速和超时保护均由 C++ 定时驱动。Web 重启不会取消已授权跟随；手动摇杆请求中断则按原命令租约归零。
- 启动跟随等待真实飞控遥测确认模式和武装状态；手动停止、手动接管或取消启动可立即使旧事务失效。没有强制解锁，也没有绕过 Rover 混控直接写推进器 PWM。
- UWB 长时缺测保留原等待、解除武装及可信观测窗口恢复流程。恢复期间的速度渐增、预测协方差、接近目标制动、遮挡隔离和神经网络推进规则均保留。
- 串口使用独占文件锁；重复命令端口绑定失败即退出。心跳、命令租约、进程调度和内部测量使用单调时钟，避免 NTP 回拨破坏测量顺序。发送遥测和 RPC 响应时统一换算到当前系统时间，保留每条观测的实际年龄；否则 WSL 校时后内部 epoch 偏移可能让网页误判为未来数据。过期值、未来值和空时间戳不因此变成新观测。
- UWB 突跳隔离保留原轨迹的 250 ms 连续恢复确认。无法返回原位置时，同一标签的新观测必须连续符合一条有界运动轨迹至少 1.5 秒、至少 16 个独立采样，相邻观测间隔不超过 150 ms。候选轨迹在源时间上做二维匀速拟合，速度不超过 `max_relative_speed_mps`，每个候选点相对拟合位置的残差不超过 `max(0.24 m, 2×距离噪声)`、方位残差不超过 `max(12°, 3×校准后角度噪声)`；同时逐包检查相邻位移/角度是否超过噪声余量与速度预算。检查包括未保留到 25ms 证据序列的中间包，不能通过抽样隐藏尖峰。超限、无效帧、视野外数据或超过 150ms 的间隔会重启确认；重复包不增加证据。确认成功后按新位置、拟合速度和非零协方差重新初始化滤波器。确认前不刷新可信时间戳，也不向控制器提供隔离数据。该连续性检查不能区分真实平滑运动与同样平滑的多径偏差漂移，协议中没有独立的 NLOS 质量证据。
- 正常 SIGTERM 先清除待发送运动命令，再发送中立输出。进程被强制杀死、板端断电等情形仍依赖飞控自身的 GCS/RC 失效保护，服务软件不能替代飞控保护。
- Node 仍按每次跟随生成原 CSV。Web 离线期间控制继续运行，但该 Web CSV 记录暂停；服务日志由 systemd 写入 `logs/mavlink.log`。

## 无硬件验证

```bash
python scripts/check_native_parity.py control/build/manta-replay
python scripts/check_native_wire.py control/build/manta-wire-replay
python scripts/check_native_uwb_contract.py control/build/manta-wire-replay control/build/manta-follow-replay
node scripts/check_native_follow.mjs control/build/manta-follow-replay
python scripts/check_native_service.py control/build/manta-control
python scripts/check_native_stack.py control/build/manta-control
```

验证覆盖 UWB 全字段对照、真实 MAVLink 1/2 编解码、CRC/分片、四种 PILOT_STEER_TYPE、ACRO 物理量与 RC 标定编码、IMU 六面校准、推进/跟随算法对照、Linux 伪串口、UDP RPC、独立看门狗、HTTP 启停、Web 重启、自动恢复及取消启动。

`native_uwb_contract` 也注册为 CTest，可运行 `ctest --test-dir control/build --output-on-failure`。
它覆盖同批“近→远”（包括近距离帧只有 DIST、丢失 EL、超过 5ms）、两种相对控制路径归零、停车消费与持续安全解除、5ms 边界、相邻旧 Lua 帧的全部残缺字段组合、NaN/Inf、来源隔离、重复帧及 uint32 源时钟回绕。重连测试包含握手前后到达的旧完整帧、旧尾字段与重启后同时间戳 DIST、错误/过期 TIMESYNC 应答及源时钟重启。`native_service` 也注册为 CTest，覆盖伪串口打开前遗留输入；`check_native_stack.py` 验证同批近远观测在真实服务进程中触发零电机输出并能恢复。

## UWB 观测帧协议

接收端兼容仓库旧 Lua 的逐字段时间戳，也支持新版共享时间戳协议，可以先升级接收端。
新版脚本的一帧 `DIST → EL → AZ` 使用相同 `NAMED_VALUE_FLOAT.time_boot_ms`，发送前先声明 `UWB1_PROTO=2` / `UWB2_PROTO=2`。
声明按来源和接收器在当前会话内保持，不能被后续降级报文取消；该来源只允许同一源时间戳的三个有效字段组成观测。
新版脚本使用 MAVLink Lua 发送接口，在收到心跳的通道上转发（最近 3 秒内活跃），不固定串口/通道映射。
每次收到对端心跳后，在该通道每个接收器的下一次观测前重发协议声明，以覆盖接收端进程重启；声明发送失败时暂不发送观测。旧接收端忽略这个诊断字段，仍可读取共享时间戳的三个位置字段。
发送字段失败时不再发送该帧的 AZ 完成字段；下一帧使用新的源时钟。

没有协议声明时，接收端仍接受三个字段源时间戳完全相同的观测。
对旧 Lua 跨毫秒的观测，只允许同一来源/接收器的连续 `DIST → EL → [VALID=1] → AZ`：整个 MAVLink 来源流的包序号必须逐包连续（允许 255→0），源时间单调且从 DIST 到末字段不超过 5ms，本地字段接收跨度也须不超过 100ms。期间正常接收到的姿态等遥测允许穿插，不会被当作丢包；实际包序号缺口才会切断兼容配对。
旧脚本每个接收器至少间隔 20ms 才处理下一帧，因此这个短源时间窗不会把相邻帧的残缺字段拼接。缺字段、序号跳变、跨度超限或来源变化时不使用跨时间戳兼容路径；没有包序号证据的离线调用也不启用该路径。
这项兼容针对 `test/fixtures/uwb_aoa_legacy.lua` 固定的旧发送器。任意字段延迟、改过发送周期的第三方旧脚本不能仅凭接收时间证明同帧，应升级到声明版本的发送器；不能通过放宽 5ms 窗口解决。

多个源观测即使同批到达、具有相同本地接收时间，也各自经过校验和滤波，不伪造未来接收时间。
启用接收器一旦收到通过会话/重放检查的危险 DIST，就独立锁存 `clearanceStop`、最小 `clearanceDistanceM` 和最近危险接收时间 `clearanceObservedAt`，无需等到角度字段完整或滤波接受。后续远帧只能更新最新观测，不能覆盖尚未处理的停车证据；小于测量下限的有限非负距离也触发保护。控制循环先归零并确认已处理，再允许该接收器后续完整、可信且原始/滤波距离均超过恢复阈值的观测累计解除证据：本地接收时间至少持续 500ms、至少 3 次不同接收时刻，相邻间隔不超过 150ms。同批远帧、单纯超时、丢失观测或被拒绝观测均不能解除；再次收到近距离必须重新确认。此锁存先于预测、NMPC 和控制周期限频检查。
NaN、Inf 或越界字段清空待配对数据并作废该源帧；新帧可以重新获取观测，同帧补包不能撤销拒绝。
生产链路须集齐同一观测的距离、方位角和仰角，不能沿用上一帧的仰角。已完成帧的晚到字段仍先做有限值和范围检查，晚到的非法仰角也会撤销该帧的可信状态。
每个接收器分别维护各 MAVLink 来源的 uint32 源时钟高水位，按回绕序比较。非连续旧帧重放不能刷新原始距离或可信时间，也不能打断正在配对的新帧；正常时钟回绕仍可接收，断连后高水位清除。
低频 `SEQ` 仍仅用于诊断，不作为生产链路的配对依据。回放 API 未传入源标识的旧离线样例保留原时间窗行为。
断连（包括心跳超时）立即发布失效 UWB 状态，并清理待配对字段、协议声明、源时钟高水位、去重、原始/滤波观测、隔离与恢复历史、融合选择/渐变状态；安装几何和滤波配置保留。新会话需要新的完整观测及新姿态，不能复用旧数据；源时钟归零或复用断连前时间戳均可重新初始化。
串口打开时清空内核输入缓存。每次连接必须先完成对选定飞控的 MAVLink TIMESYNC 往返：请求使用新随机种子递增 cookie，仅接受该飞控 500ms 内回显的应答。确认前的 UWB 报文全部隔离；确认时再次清空 UWB 与姿态有效时间，从应答携带的飞控启动毫秒建立下界，每个来源/接收器首先收到新的 DIST 才能接收尾字段。仅允许该飞控系统 ID（组件可不同）且时间在当前启动时钟附近的报文进入帧配对，旧高启动时间报文不能抬高新会话高水位。每秒刷新时钟锚点，允许正常回绕，检测到启动时钟跳变时重新清理会话。
TIMESYNC 由 ArduPilot 固件处理，不要求旧 Lua 增加字段；未取得有效应答时 UWB 保持不可用。此边界依赖受信任的有序飞控串口流，以及 Lua `time_boot_ms` 与飞控启动时钟同源，适用于仓库新旧脚本；它不是对任意重排/恶意重放的认证。旧协议没有会话 ID，完全相同来源、时间戳与字段的伪造重放无法仅靠这三个字段鉴别。

旧 Python/JavaScript 算法保留为差分参考；`test/fixtures/legacy-server-control.txt` 固定迁移前的服务器行为供原 VM 测试使用，不参与生产运行。新的 native 测试验证替代实现，不能仅凭旧基线测试通过判断迁移完成。

这些检查不连接实际飞控、云台或推进器。RK3588 实机串口、视频、校准和水上运行仍需在目标设备验收。

## UWB 时间对齐与独立测量记录

原生链路默认启用时间对齐，不要求更新旧 Lua。先通过现有 TIMESYNC 会话握手建立飞控启动时钟与本地单调时钟的映射，再用一帧 DIST 的源时间驱动滤波、速度估计与观测年龄判断。同批收到的多个观测保持各自的源时间间隔；接收时间单独保留，不把积压观测当成刚采样的数据。旧 Lua 仍使用原来的 5ms、字段顺序、包序号及来源检查；时间对齐不会放宽配对窗口。

每帧在姿态历史中寻找相同时刻的航向。相邻姿态间隔不超过 150ms 时做短弧插值（支持 ±180° 回绕）；没有后一帧时最多用角速度外推 100ms，避免额外等待一帧姿态。缺少匹配姿态时记录 `heading_unavailable`，不借用任意旧航向。历史姿态晚到可以参与后续插值，但不会覆盖较新的实时航向。断连/源时钟重启清空姿态历史和时钟映射。

逐接收器滤波结果保留测量时刻的角度；融合发布时换算到最新有效 IMU 航向对应的船体坐标，发布 `measurementHeadingDeg` 表明参考航向。控制端对这条已对齐路径取消额外的 50ms 角度限速平滑，避免重复平滑增加滞后；原有滤波参数、角度标定及执行器限制保持。`measurementSampleAt` 表示映射后的源时刻，原 `measurementUpdatedAt` / `receivedMeasurementAt` 继续表示接收/接受时刻以兼容旧消费者。未携带新字段的离线输入仍走旧时间语义。

会话内固定时钟偏移以保持滤波时间连续；周期 TIMESYNC 刷新会话检查并记录偏移不确定度，不因一次慢应答改变历史采样间隔。`sampleAt` 不超过接收时刻，内部 `filterTimeMs` 保留源时间间隔，估计时钟超前量另记 `clockLeadMs`。墙上时钟校正时，导出的本地时刻一起换算，启动毫秒、时延与不确定度不换算。长时间采集应检查 `clockUncertaintyMs`，当前没有进一步拟合两端时钟频率漂移。

**时间精度边界：** 新旧 Lua 的 `time_boot_ms` 都是脚本处理时刻，尚无无线测量硬件时间戳，不能据此消除 UWB 模块、串口到 Lua 的内部积压。当前校正的是 Lua 到主机的输送时序与航向错位；距离平移延迟由现有控制预测处理，原始记录不预测到现在。双接收器发布采用参与融合观测中较早的时间，尚不提供基于各自平移速度的异步重采样。

独立采集通过以下 API 开始/停止，无需启动跟随或解锁。请求沿用服务器的认证/同源设置，端口使用实际 Web 配置：

```text
POST /api/uwb-measurements/start
{"label":"静态 4m 正前方","referenceDistanceM":4,"referenceBearingDeg":0}
GET  /api/uwb-measurements/state
POST /api/uwb-measurements/stop
{}
```

`label` 和两个参考值可省略。参考方位使用校准后的船体角度（度），参考距离使用米；固定参考值适用于静态标定，运动试验不要填固定真值，否则误差列不代表运动精度，应在采集后按时间对齐外部真值。开始时保存标定和测量配置快照。

文件位于 `config.logs_dir`（默认 `logs`）下的 `uwb_tests/uwb_measurements_*.csv`，可通过现有 `GET /api/uwb-follow/logs` 列表和下载接口获取。同名 `.csv.json` 保存元数据，停止后 `.csv.summary.json` 保存行数、丢行数和序号缺口。CSV 每条观测独立记录源时间、接收时间、输送延迟、航向及插值方式、原始/滤波距离和角度、校准后方位、滤波修正量与拒绝原因；缺失值留空。成功、拒绝和被后帧替代的残缺帧均记录；待配对字段超过 100ms 或断连时记一次残缺记录，不用旧角度补齐。

原生进程直接逐帧发出 `uwb_measurement`，Node 不从低频状态快照采样，因此同批“近→远”也有独立记录。采集使用有界异步写队列，磁盘跟不上时计入 `droppedRows`，不阻塞控制。`sourceInstance` / `sourceSession` / `recordId` 可区分进程、重连与序号缺口；`observedSequenceGaps` 是观察到的跳号次数累计，晚到行以 `outOfOrder` 标注，不回减该计数。本地 UDP 仍可能丢包，强制结束进程也不保证缓冲数据落盘，验收记录应检查汇总并正常停止。

对应测试覆盖源时序下逐帧/每五帧批量送达的一致性、带实际角度标定的转向回放、航向回绕插值、有限外推、迟到姿态、源时钟回绕与 RTT 改变、延迟观测新鲜度、残缺/断连记录，以及未解锁时的 HTTP 采集和 CSV 内容。旧 Python 差分基线使用 `uwb_time_alignment.enabled=false` 以比较原接收时间语义；默认启用路径由原生协议回归测试单独验证。

## 本地 SITL

使用隔离配置，设 `MANTA_SIMULATION=1`、虚拟串口 `/dev/manta-sitl-virtual`、命令/遥测端口 `15551/15552`，然后：

```bash
MANTA_SIMULATION=1 MANTA_CONFIG_FILE=/absolute/simulation.json \
  control/bin/manta-control --simulation-udp 127.0.0.1:14560
MANTA_SIMULATION=1 MANTA_CONFIG_FILE=/absolute/simulation.json npm start
```

SITL/MAVLink 路由器须在指定 loopback UDP 口接收 MAVLink，并将遥测回送至服务的 UDP 源端口。旧外部仿真若导入 Python bridge，应切换到上述独立原生进程。原 Python 模块继续可供基线测试使用。
