# React 控制台迁移

主控制台源代码位于 `web/src/`，通过 Vite 构建至 `dist/web/`。Node.js 的
`/` 和 `/index.html` 提供该构建。未构建时返回 503 并提示构建命令，避免
部署时误以为旧 HTML 已经运行了 React。安装器会自动执行构建。

## 开发和部署

需要 Node.js 20.19+ 或 22.12+。

```sh
npm ci
npm run maintenance
npm start
```

`npm run maintenance` 包括自动化测试、项目检查、TypeScript 严格检查和生产构建。
只修改前端时执行 `npm run build` 后刷新页面即可；Node 服务首次升级需要重启。
离线部署包应携带已构建的 `dist/web/` 及运行依赖，构建需要开发依赖。

独立演示（不连接硬件）：

```sh
npm run build
npm run preview
```

打开 http://127.0.0.1:4175/，使用页面显示的演示配对码。
`npm run dev:web` 启动开发服务器，API 和 Socket.IO 转发到本机 3000 端口。
开发服务器默认使用真实服务模式；硬件操作具有与原控制台相同的效果。
`npm run preview:original` 保留原 HTML 对照入口：
http://127.0.0.1:4173/mobile-preview-kimi-k26.html。

## 保持原样的边界

- `ConsoleLayout.tsx` 按原页面边界拆为设备、地图、录像、工具抽屉和标定弹窗等 React 组件。
- `console.css` 与原内联样式逐字一致（换行符除外）；沿用全部布局、颜色、字体、SVG、资源、间距和横屏规则。
- `console-controller.ts` 保留原交互和 DOM 更新逻辑，由 React 挂载/清理。没有增加布局包装节点。
- `lifecycle.ts` 管理事件、计时器、动画帧和临时窗口导出，清理时停止摇杆、视频和通信。
- UWB 原始点、可信点及完整测量帧时钟独立保留；TypeScript 显示模块有明确的输入、输出类型。
- 现有 HTTP、Socket.IO、视频、录像下载、离线地图和标定接口保持不变。其他独立工具页面仍由 `frontend/` 提供。

控制器、地图和通信适配层保留了部分动态类型，便于逐步收紧接口而不改变既有行为。
后续重构应继续运行原样对照测试，不得以组件重构为由重新设计界面。

## 验证

- `react_console_parity.test.mjs` 比较原 HTML 与 React 渲染后的完整节点结构、文字、图标、属性和样式。
- 原 UWB 显示测试同时运行于 JavaScript 和 TypeScript 实现，检查 40 Hz、重复帧、半帧、过期、失效和接收器切换。
- `react_transport.test.mjs` 比较原通信层和迁移层的驱动限幅、锁定、武装确认以及跟随、标定、录像、云台和急停请求。
- `react_lifecycle.test.mjs` 检查重复挂载时事件、计时器和窗口导出的清理，以及语言切换所需的 Document 方法绑定。
- 浏览器对照包含桌面初始页的 187 个 ID 元素位置、尺寸和计算样式，以及 844×390 横屏、配对、地图、录像和六面校准演示。

这些是软件与演示验证，不能代替板端串口、真实视频源、推进器和水面航行验收。

## C++ 迁移状态

`control/` 当前包含 UWB 量测准入、滤波、双接收器融合及离线 JSONL 回放程序。
`scripts/check_native_parity.py` 对照 Python 基线，已覆盖 14,248 帧及全部输出字段。
CI 会编译并运行这项对照。

**当前尚未切换线上控制链路。** MAVLink 串口服务、跟随控制及独立进程部署仍需完成；
现有 Node.js / Python 控制路径继续工作。不得将离线 UWB 对照通过解释为整个 C++ 控制服务已完成，
也不得在切换前删除原控制算法或降低现有看门狗与准入要求。
