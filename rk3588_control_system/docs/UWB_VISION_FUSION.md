# UWB + YOLO 视觉追踪

控制网页连接设备后，控制区顶部新增 **UWB+视觉追踪** 卡片。点击 **打开融合窗口** 查看相机画面、YOLO 检测框、雷达、目标距离/方位、两路数据年龄和融合状态。主入口仍为设备的 `/`；原有 UWB 跟随、左右伴随和云台页面保持各自的控制入口。

## 使用

1. 在主控制网页配对并获取控制权。
2. 打开融合窗口，展开“融合配置与相机标定”。按相机的 **1× 原始检测画面** 设置水平视场角、云台方向、图像方向和零点偏移。默认 70° 只是起始参数，并非相机标定结果。
3. 让一名佩戴当前 UWB 信标的泳者出现在画面内，在正前方与左右两侧核对视觉方位和 UWB 方位。核对后勾选标定确认，保存配置。
4. 点击“启动 YOLO 泳者追踪”，启动现有 `scripts/infer_video.py` 检测器及云台追踪。模型沿用 `config/system.config.json -> gimbal.swimmer.weights`（默认 `scripts/best.pt`），依赖和模型资产按原系统部署；本次不附带训练权重。
5. 点击“启用融合”。角度与时间校验均通过后显示“融合追踪中”。

启用融合只计算目标位置，不武装、不启动船体推进、不更改现有 UWB 控制输入。“启动 YOLO”会让云台进入泳者追踪。暂停融合不停止已经启动的 YOLO；使用原云台停止按钮结束云台追踪。急停同时暂停融合。

## 数据与算法

```text
UWB admitted telemetry → distance + calibrated body bearing ┐
YOLO TARGET output → image center + confidence + receive time ├→ time/angle gates → relative fix
Gimbal feedback → yaw + feedback receive time               ┘
```

- 共享 `frontend/js/uwb-vision-core.js` 的纯计算实现，后端和浏览器演示使用同一逻辑。
- 输入来自已经通过桥接门控的 UWB，以及现有泳者追踪进程的 `TARGET:` 输出。只接受泳者、已锁定、非预测、足够置信度且尺寸有效的检测。
- 视觉方位采用针孔近似：`atan((x/frame_w − 0.5) × 2 × tan(horizontalFov/2))`，再按方向与零点叠加实时云台 yaw。
- UWB 先沿用现有控制器的 `bearingSign / bearingScale / bearingOffsetDeg` 得到艇控的右正方位；仅在融合入口将 sign 和 offset 同时取反、保留 scale，转换为 **X 前、Y 左、方位左正右负**。原 UWB 数据及艇控输入不修改，不对融合输出再次套原始 UWB 标定。
- 默认时效：UWB 800 ms、视觉 600 ms、云台反馈 350 ms；两路最大时间差 300 ms，最大方位差 20°。这些参数在后端校验范围内可配置。
- 融合只修正方位：`UWB 方位 + wrap(视觉方位 − UWB 方位) × visionWeight × confidence`。距离始终来自有效 UWB，最大视觉权重默认 0.65。
- 视觉不可用时退回仍有效的 UWB；UWB 不可用时只返回视觉方位，距离和 XY 为 `null`；两路过期时清空位置，不把上次位置当成当前结果。
- 未标定时拒绝融合；改变视场角、方向或偏移会清除旧标定状态，除非该次保存明确重新确认。检测到云台反馈中的数字变焦大于 1× 时拒绝视觉融合。

本实现使用后端收到测量的时间戳做同步门控，尚未提供相机曝光时间同步、相机畸变修正、三维外参、多目标身份匹配或变焦标定。方位一致不等同于身份已确认，操作员需确认信标与检测目标一致。真实动态精度、视频链路延迟、模型性能和安装方向需要在板端验证。

### SIYI A8 mini 泳者输入约定

- 安装约定：云台回中 0°的光轴指向船头；反馈来自 `0x0D` 实测姿态，经现有 SIYI 解析器的 `int16 / 10` 转成数值型度。转动命令、速度和累计运动量不作为方位输入。
- A8 默认 `yawSign=-1`，将右正的实测朝向变为本接口的左正方位；画面原始右侧像素偏角默认也取负。目标居中时，实测 yaw `+30°` 对应方位 `-30°`；目标未居中时还必须加上图像偏角。零偏默认 0，已有保存配置优先，部署时需复核旧标定；不会自动置 `calibrated=true`。
- 仅当前已锁定、正在运行的泳者 worker 可以产生输入。每次收到 `TARGET:` 都复制当时的目标与真实姿态；切换人脸、点击居中、UWB 云台模式，或停止/丢失泳者时撤销该输入。SIYI 姿态需要明确的有效姿态及校验结果。
- SIYI 当前每 100 ms 请求一次姿态（10 Hz）。本分支泳者配置 `gimbal.swimmer.loop_hz=25`（未配置时服务默认 10 Hz），实际输出受推理速度限制。多个检测可复用同一真实反馈，但必须保留该反馈的 `updatedAt`；不能生成虚假的 25 Hz 姿态时间戳。
- `target.timestamp` 是服务器收到 `TARGET:` 的 epoch 毫秒；`pose.updatedAt` 是服务器收到实际反馈的 epoch 毫秒。页面约 4 Hz 查询并不刷新这两个测量时钟，也不是船体控制频率。融合结果仍仅供查询/显示，不直接启动推进控制。

## API 与配置

| 接口 | 用途 |
| --- | --- |
| `GET /api/uwb-vision/state` | 配置、模型路径、两路有效性、融合状态及结果 |
| `POST /api/uwb-vision/config` | 保存部分配置字段；错误类型/越界/未知字段返回 400 |
| `POST /api/uwb-vision/enabled` | `{ "enabled": true/false }`，仅启停融合计算 |

接口沿用现有配对与单页面控制权规则：读接口要求配对，写接口要求页面持有控制权和在线实时通道。不存在独立的免认证模型上传端口。YOLO 通过现有追踪进程输出接入，不向外部网站传输设备数据。

配置原子保存至忽略提交的 `data/uwb-vision.json`。测试可通过 `MANTA_UWB_VISION_CONFIG` 指定独立文件。重启会保留配置，但融合默认停用；文件损坏时退回未标定默认值。

结果的 `source` 为 `uwb+vision`、`uwb` 或 `vision_bearing`；无有效解时 `result` 为 `null`。`state.vision.box` 使用原始帧中心点 `x,y` 和框尺寸 `w,h`，不是左上角。

## 本地演示与验证

```bash
node scripts/serve-preview.mjs
# 打开 http://127.0.0.1:4173/mobile-preview-kimi-k26.html
# 选择演示设备并使用页面显示的测试配对码 240724

node --test test/uwb_vision_fusion.test.mjs test/uwb_vision_server.test.mjs
node --test
node scripts/check_project.mjs
```

预览端口 4173/4175 沿用主页面的演示连接。窗口明确标记“演示数据 · 不连接硬件”；示例视频上的检测框也是模拟数据。可切换两路正常、视觉丢失、UWB 丢失、方位冲突和两路过期。设备页面也可临时“查看演示”，演示配置与开关不会写入设备。

本地自动化覆盖融合、角度环绕、相机方向、过期/预测/低置信检测拒绝、失联回退、错误配置、真实后端权限与持久化；这些检查不替代实机 UWB 和 YOLO 联调。

