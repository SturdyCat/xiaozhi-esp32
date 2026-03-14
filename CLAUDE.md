# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

小智 ESP32 是一个开源的 AI 语音聊天机器人固件，运行于 ESP32 系列芯片。通过 WebSocket 或 MQTT+UDP 协议与大语言模型进行语音交互，使用 OPUS 音频编解码、LVGL 图形显示，以及 MCP（模型上下文协议）进行设备控制。

**当前版本**：2.2.4（v2 分区表与 v1 不兼容）

## 构建命令

```bash
# 首先配置 ESP-IDF 环境
source $IDF_PATH/export.sh

# 构建指定开发板（推荐）
python scripts/release.py <开发板名称> [--name <变体名称>]

# 列出所有可用的开发板
python scripts/release.py --list-boards

# 标准 idf.py 命令（需先通过 menuconfig 选择板型）
idf.py set-target esp32s3  # 或 esp32c3, esp32p4 等
idf.py menuconfig          # 配置板型、语言等
idf.py build
idf.py flash
idf.py merge-bin           # 生成 merged-binary.bin 用于分发

# 提交前格式化代码
clang-format -i path/to/file.cpp
find main -iname "*.h" -o -iname "*.cc" | xargs clang-format -i
```

## 开发环境

- **ESP-IDF**：需要 5.5.2 或更高版本
- **IDE**：Cursor 或 VSCode + ESP-IDF 插件
- **代码风格**：Google C++ 风格，使用 `.clang-format` 配置（4空格缩进，100字符行宽）
- **平台**：推荐 Linux，编译更快、驱动问题更少

## 架构

### 核心组件

```
main/
├── application.cc/h      # 单例模式主控制器 - 主事件循环、状态机
├── device_state.h        # 设备状态：Idle, Listening, Speaking, Connecting 等
├── protocols/            # 通信协议抽象
│   ├── protocol.h        # 基类，定义音频/JSON回调
│   ├── websocket_protocol.cc
│   └── mqtt_protocol.cc
├── audio/                # 音频流水线（详见 main/audio/README.md）
│   ├── audio_service.cc  # 编排编解码器、处理器、编码器/解码器
│   ├── codecs/           # 硬件编解码驱动（ES8311, ES8388 等）
│   ├── processors/       # AEC 回声消除、VAD 语音检测（基于 ESP-SR）
│   └── wake_words/       # 唤醒词检测
├── display/              # OLED/LCD 显示（LVGL 9.x）
│   ├── lcd_display.cc
│   ├── oled_display.cc
│   └── lvgl_display/     # 表情、Emoji、字体、图片
├── mcp_server.cc/h       # 设备端 MCP 协议，用于 IoT 控制
├── ota.cc/h              # OTA 固件升级
└── boards/               # 开发板适配
    ├── common/           # 公共实现：WiFiBoard, ML307Board, NT26Board, 按键, 电池
    └── <板名>/           # 70+ 开发板配置，含 config.h, config.json, xxx_board.cc
```

### 关键设计模式

1. **Application 单例**：`Application::GetInstance()` 是主入口。调用 `Initialize()` 初始化后执行 `Run()`（阻塞式事件循环）。

2. **Protocol 抽象**：`Protocol` 基类，有 `WebSocketProtocol` 和 `MqttProtocol` 实现。注册回调处理音频、JSON 和网络事件。

3. **Board 抽象**：每个开发板定义：
   - `config.h` - 引脚映射、音频设置、显示配置
   - `config.json` - 编译目标和 sdkconfig 覆盖项
   - `*_board.cc` - 开发板初始化，实现 `WifiBoard`/`ML307Board`/`DualNetworkBoard`

4. **音频线程模型**：三个 FreeRTOS 任务 - `AudioInputTask`（录音）、`AudioOutputTask`（播放）、`OpusCodecTask`（编解码），详见 `main/audio/README.md`。

5. **状态机**：`DeviceStateMachine` 管理 Idle→Listening→Speaking 状态转换，基于事件触发。

## 开发板配置

添加新开发板步骤：
1. 创建 `main/boards/<厂商>-<板名>/` 目录
2. 创建 `config.h` 定义引脚（参考 `docs/custom-board.md`）
3. 创建 `config.json` 指定目标芯片和编译选项
4. 在 `main/CMakeLists.txt` 添加 CONFIG_BOARD_TYPE_* 条目
5. 在 `main/Kconfig.projbuild` 添加 Kconfig 选项

**重要**：切勿覆盖已有开发板配置。必须创建唯一的板型标识，否则 OTA 升级时会被原板固件覆盖导致设备无法工作。

## 网络选项

- **WiFi**：标准 WiFi 连接
- **ML307**：Cat.1 4G 模块（UART 通信）
- **NT26**：UART 以太网模块
- **双网络**：同时支持 WiFi 和 4G 的开发板

## 通信协议

- **WebSocket**：完整协议规范见 `docs/websocket.md`
- **MQTT+UDP**：混合协议见 `docs/mqtt-udp.md`

## MCP（模型上下文协议）

设备端 MCP 服务器允许大语言模型控制硬件。参考：
- `docs/mcp-protocol.md` - 协议实现
- `docs/mcp-usage.md` - 使用示例

## 依赖管理

通过 `main/idf_component.yml`（IDF Component Manager）管理。主要依赖：
- LVGL 9.4.x 图形库
- ESP-SR 唤醒词和 AEC
- ESP Codec Dev 音频编解码
- `78/` 命名空间的自定义组件（xiaozhi-fonts, esp-opus 等）

## 内存文件

自动记忆目录位于 `/Users/bigboss/.claude/projects/-Users-bigboss-Desktop-notip-com-cn-xiaozhi-esp32/memory/`。
