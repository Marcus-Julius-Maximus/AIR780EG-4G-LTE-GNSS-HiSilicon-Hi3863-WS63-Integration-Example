[English](README.md) | **简体中文**

# AIR780EG 4G LTE-GNSS × 海思 Hi3863（WS63）接入示例

将合宙 **Air780EG**（4G Cat.1 + GNSS）通过 UART + AT 指令接入海思 **Hi3863 / WS63**（OpenHarmony LiteOS-M）的完整驱动与示例代码，实现：

**4G 联网 + GNSS 定位 + MQTT 遥测上传 + 基站时间（NITZ）同步**

> 目标板卡：`isoh/nl63pro`，SoC：HiSilicon WS63（Hi3863），系统：OpenHarmony LiteOS-M。

---

## 目录

- [功能特性](#功能特性)
- [硬件连接](#硬件连接)
- [目录结构](#目录结构)
- [软件架构与数据流](#软件架构与数据流)
- [快速开始](#快速开始)
- [配置项说明](#配置项说明)
- [MQTT 主题与 JSON 数据格式](#mqtt-主题与-json-数据格式)
- [AT 指令清单](#at-指令清单)
- [关键实现与避坑记录](#关键实现与避坑记录)
- [API 速查](#api-速查)
- [注意事项](#注意事项)
- [参考资源](#参考资源)

---

## 功能特性

- **UART 中断收发**：注册接收回调 + 2048 字节环形缓冲区，任务侧非阻塞读取，彻底避开 WS63 `uapi_uart_read` 的关中断自旋问题（见[避坑记录](#关键实现与避坑记录)）。
- **驻网与模组初始化**：关回显、打开 GNSS、等待 SIM/网络附着、使能 AGNSS 辅助定位（EPO 星历需联网下载）。
- **GNSS 定位解析**：解析 `AT+CGNSINF` 的 fix / 纬度 / 经度 / 海拔，不依赖 `%f` 与 `sscanf`。
- **MQTT over AT**：`MCONFIG → MIPSTART → MCONNECT` 建链，`MSUB` 订阅云端指令，`MPUB` 发布遥测，发布失败自动标记断线待重连。
- **遥测数据中心**：任何传感器模块（GPS / IMU / NTC / 电量…）通过 setter 写入，统一生成 JSON。
- **多通道发布注册表**：5 个通道槽位（0 = 主通道，1~4 预留），可随时挂接新的 topic 与 payload 生成回调。
- **边缘计算采样**：平均模式（AVG）下自动统计动能均值 / 峰值 / 活跃帧比例（见 [JSON 格式](#mqtt-主题与-json-数据格式)）。
- **4G 基站对时（NITZ）**：`AT+CTZU=1` + `AT+CCLK?` 取基站时间，通过 `Set_Global_Time()` 注入主板，成功一次后零开销。
- **断链恢复**：UART 级自检/重建示例（`air780_test.c`，已退役，仅供排查参考）。

---

## 硬件连接

| Air780EG 引脚 | WS63 / NL63Pro 引脚 | 方向 | 说明 |
| --- | --- | --- | --- |
| TXD | GPIO7 | 模组 → MCU | UART2 接收，对应 `AIR780_RX_PIN` |
| RXD | GPIO8 | MCU → 模组 | UART2 发送，对应 `AIR780_TX_PIN` |
| GND | GND | — | 必须共地 |
| VBAT | 电源 | — | 按合宙硬件手册设计，保证电流余量与去耦电容 |
| RTS / CTS | 未接 | — | 无硬件流控 |

- 串口参数：**115200 8N1**，无硬件流控。
- Air780EG 的 UART 为 **1.8V 电平**，与 WS63 的 3.3V 之间存在电平差异，**必须经电平转换**后再连接（本项目硬件已完成处理）。
- 请接好 **4G 天线** 与 **GNSS 天线**，并安装可用的 SIM 卡（eSIM 版本请确认已开通）。
- 本工程只负责串口通信，**不包含模组上下电控制**（PWRKEY 等上电时序由硬件处理），请确认模组已正常开机并处于可响应 AT 的状态。

---

## 目录结构

```text
mk1/
├── BUILD.gn               # GN 构建脚本：编译为静态库 air780_app
├── air780_app.c           # 应用主流程：初始化、主循环、MQTT 配置、NITZ 对时、指令解析
├── air780_uart.c/.h       # UART2 中断接收 + 环形缓冲 + AT 收发 / URC 等待
├── air780_modem.c/.h      # 驻网检测、CSQ、GNSS 定位解析、AGNSS 辅助定位
├── air780_mqtt.c/.h       # MQTT over AT：连接 / 断开 / 发布 / 状态维护
├── air780_telemetry.c/.h  # 遥测数据中心、定点 JSON 拼接、5 通道发布注册表
└── air780_test.c          # 独立诊断任务（已退役，不参与构建，仅供排查参考）
```

---

## 软件架构与数据流

```text
                    UART2 @ 115200 8N1
 ┌─────────────┐  ◄────── AT 指令 ──────┐
 │  Air780EG   │                        │
 │  4G Cat.1   │  ────── 响应 / URC ───►│
 │  + GNSS     │                        │
 └─────────────┘                        ▼
                         ┌────────────────────────────────────┐
                         │       WS63 / Hi3863 (LiteOS-M)     │
                         │            Air780Task              │
                         │                                    │
                         │  air780_uart       中断 → 环形缓冲 │
                         │  air780_modem      驻网 / CSQ / GNSS│
                         │  air780_telemetry  数据中心 / JSON │
                         │  air780_mqtt       MQTT over AT    │
                         └───────────────┬────────────────────┘
                                         │ 4G（模组内置 TCP）
                                         ▼
                                   MQTT Broker
```

主流程（`Air780Task`）：

1. `Air780UartInit()`：初始化 UART2 并注册接收回调，失败自动重试。
2. `Air780ModemPrepare()`：`ATE0` → `AT+CGNSPWR=1` 打开 GNSS → 等待 `CGATT` 附着（约 60s 超时）→ `AT+CGNSAID=31,1,1,1` 使能辅助定位。
3. `Air780MqttSetup()` + `Air780SetupChannels()`：保存 broker 配置、注册主通道与预留通道。
4. 主循环：
   - `Air780RefreshSensors()` 刷新 CSQ 与 GPS 到数据中心；
   - `Air780SyncTime()` 首次成功联网后用 NITZ 基站时间给主板对时；
   - `Air780EnsureMqtt()` 确保连接（重连成功会 `MSUB` 订阅 `alpha/1/cmd` 并发布上线通知）；
   - `Air780ChannelPublishAll()` 遍历所有启用通道发布 JSON；
   - 按当前上传间隔休眠，期间短时轮询串口以处理云端的 `+MSUB` 下发指令与拨轮/按键事件。

---

## 快速开始

### 1. 放入 SDK

将本目录整体放入 OpenHarmony SDK 的应用代码目录（示例工程中为 `//app/Air780/`，请按你的 SDK 结构放置）。

### 2. 注册编译组件

在应用层 `BUILD.gn` 的 `features` 中加入：

```gn
features = [
    # ... 其他组件
    "Air780:air780_app",
]
```

本目录的 `BUILD.gn` 会把代码编译为静态库 `air780_app`，并声明 WS63 SDK 所需的头文件路径与宏（`CHIP_WS63`、`CONFIG_UART_SUPPORT_TX/RX` 等）。**不同 SDK 版本的 include 路径可能不同，请按实际情况调整。**

### 3. 修改配置

按需修改 `air780_app.c` 顶部的 `MQTT_HOST` / `MQTT_PORT` / `MQTT_CLIENTID` / 主题宏，以及 `air780_uart.h` 中的串口引脚与波特率。



### 4. 编译与烧录

在 SDK 根目录执行 OpenHarmony 标准构建流程：

```bash
hb set          # 选择目标产品（NL63Pro / WS63）
hb build        # 增量编译；全量编译可用 hb build -f
```

将生成的固件（如 `ws63-liteos-app_all.fwpkg`）烧录到开发板，打开串口查看日志：

- `[AIR780]`：UART / AT 收发
- `[MODEM]`：驻网、辅助定位
- `[MQTT]`：连接、发布
- `[APP]`：主流程与指令解析

正常启动后可依次看到 `[MODEM] network attached`、`[MQTT] connected to ...` 等日志。

---

## 配置项说明

### UART 配置（`air780_uart.h`）

| 宏 | 默认值 | 说明 |
| --- | --- | --- |
| `AIR780_UART_BUS` | `2` | UART 控制器编号 |
| `AIR780_RX_PIN` | `7` | GPIO7 ← 模组 TXD |
| `AIR780_TX_PIN` | `8` | GPIO8 → 模组 RXD |
| `AIR780_BAUDRATE` | `115200` | 波特率 |
| `AIR780_RX_BUFFER_SIZE` | `1024` | 驱动接收缓冲大小 |
| `AIR780_AT_BUFFER_SIZE` | `512` | AT 缓冲区大小 |
| `AIR780_READ_TIMEOUT` | `100` | 读超时（tick，1 tick = 10ms） |
| `AIR780_AT_TIMEOUT` | `500` | AT 响应超时（tick） |

> 注意：`Air780UartRead()` 的 `timeout_ms` 参数实际按**空闲 tick 计数**处理（每个空闲 tick 为一次 `osDelay(1)` ≈ 10ms），并非严格毫秒。

### 应用配置（`air780_app.c`）

| 宏 / 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `MQTT_HOST` | `mqtt.example.com` | 占位示例，请替换为你自己的 broker 地址 |
| `MQTT_PORT` | `1883` | broker 端口 |
| `MQTT_CLIENTID` | `MK-000100`（当前默认） | 设备客户端 ID，同一 broker 下必须唯一，修改方法见下方 |
| `MQTT_USER` / `MQTT_PASS` | `NULL` | 匿名登录（对应 mosquitto `allow_anonymous`） |
| `MQTT_KEEPALIVE` | `120` | 保活时间（秒） |
| `g_intervals[]` | `3/10/20/30/60/300` 秒 | 拨轮可切换的上传间隔档位 |
| `g_upload_ms` | `30000` | 当前上传间隔（默认 30s，`g_interval_idx = 3`） |
| `g_upload_mode` | `0` | 采样模式：`0` = 最新值，`1` = 平均采样 |

#### 修改客户端 ID（`MQTT_CLIENTID`）

客户端 ID **默认为当前代码中的 `MK-000100`**。修改方法：编辑 `air780_app.c` 文件顶部的宏定义（第 16 行附近），把字符串换成你自己的设备 ID 即可：

```c
#define MQTT_CLIENTID      "MK-000100"   /* 同一 broker 下必须唯一 */
```

- 修改后**无需改动其他代码**：该宏同时用于 MQTT 连接、上线通知、状态上报，并通过 `Air780TeleSetDeviceId(MQTT_CLIENTID)` 写入遥测 JSON 的 `device_id` 字段。
- 同一 broker 下的每台设备必须使用**不同的客户端 ID**（建议按设备编号递增，如 `MK-000101`、`MK-000102`），否则后上线的设备会把先上线的踢下线。
- 客户端 ID 长度上限为 **63 字符**（`air780_mqtt.c` 中缓冲区为 64 字节）。
- 目前该 ID 在编译期固定，批量烧录多台设备时需要逐台修改并重新编译；如需免编译切换，可自行扩展为从 Flash 读取。

主板 UI 可通过以下函数切换运行参数（`air780_app.c` 对外提供）：

- `Air780ChangeInterval(int direction)`：拨轮切换上传间隔；
- `Air780ChangeMode(void)`：按键（K2）切换采样模式。

### MQTT 主题（`air780_app.c`）

| 主题 | 方向 | 内容 | 默认状态 |
| --- | --- | --- | --- |
| `omega/1` | 发布 | 主通道整包遥测 JSON | 开启 |
| `omega/1/online` | 发布 | 上线通知 `{"device_id":"..."}` | 连接成功时发送 |
| `omega/1/status` | 发布 | 拨轮 / 模式变更状态 | 变更时发送 |
| `alpha/1/cmd` | 订阅 | 云端下发指令 `{"mode":x,"interval":x}` | `MSUB` 订阅 |
| `omega/1/aux1` ~ `aux4` | 发布 | 预留通道，可挂接新数据 | 关闭 |

---

## MQTT 主题与 JSON 数据格式

### 主通道 JSON（`omega/1`）

由 `Air780BuildMainJson()` 生成，单行、无换行，全程整数定点拼接（不使用 `%f`）：

```json
{"device_id":"MK-000100","mode":0,"gps":{"lng":114.070000,"lat":22.528000,"alt":20.5},"imu":{"ax":10.50,"ay":2.10,"az":9.800,"gx":0.05,"gy":0.12,"gz":-0.03},"base":{"battery":59,"ntctemp":39.2,"temp":39.2,"humidity":45.0}}
```

- `mode = 0`（LATEST）：`gps` / `imu` / `base` 均为最新采样值。
- `mode = 1`（AVG，平均采样）：温湿度、NTC、IMU 取周期平均值，其中 `imu` 字段被复用为边缘计算统计量：

| 字段 | 平均模式（mode=1）含义 |
| --- | --- |
| `imu.ax` | 平均净动能（剥离 9.80665 重力后的平均） |
| `imu.ay` | 净动能峰值 |
| `imu.az` | 活跃帧比例（0~1） |
| `imu.gx` | 平均陀螺仪抖动 |
| `imu.gy` | 陀螺仪抖动峰值 |
| `imu.gz` | 固定为 0 |

JSON 值中的经纬度为 6 位小数、海拔 1 位、加速度 2 位、`az` 3 位，与 `json.txt` 的结构保持一致。

### 状态 JSON（预留）

由 `Air780BuildStatusJson()` 生成，默认未挂到任何通道（需要时可配置到 `aux` 通道）：

```json
{"device_id":"MK-000100","csq":20,"fix":1}
```

### 云端下发指令（`alpha/1/cmd`）

```json
{"mode":1,"interval":30}
```

- `mode`：`0` = 最新值，`1` = 平均采样；
- `interval`：仅接受 `3 / 10 / 20 / 30 / 60 / 300`（单位：秒），其他值忽略。

收到后立即生效，并向 `omega/1/status` 回发状态：

```json
{"device_id":"MK-000100","current_interval":30000,"mode":1}
```

---

## AT 指令清单

| 指令 | 用途 |
| --- | --- |
| `AT` | 唤醒 / 探活 |
| `ATE0` | 关闭回显 |
| `AT+CGATT?` | 查询 PDP 网络附着状态 |
| `AT+CSQ` | 查询信号质量（0~31，99 = 未知） |
| `AT+CGNSPWR=1` | 打开 GNSS |
| `AT+CGNSAID=31,1,1,1` | 使能 AGNSS 辅助定位（mode/time/epo/loc 全开，需联网） |
| `AT+CGNSINF` | 读取 GNSS 定位信息 |
| `AT+CTZU=1` | 使能网络自动时区 / 时间更新（NITZ） |
| `AT+CCLK?` | 读取模组本地时间 |
| `AT+MCONFIG` | 配置 MQTT 客户端参数 |
| `AT+MIPSTART` | 建立 TCP 连接（等待 URC `CONNECT OK`） |
| `AT+MCONNECT=1,<keepalive>` | 发起 MQTT 连接（等待 URC `CONNACK OK`） |
| `AT+MSUB=<topic>,0` | 订阅主题（QoS0） |
| `AT+MPUB=<topic>,0,0,"<payload>"` | 发布消息（QoS0、retain0） |
| `AT+MDISCONNECT` | 断开 MQTT |

`+CGNSINF` 字段索引（解析位置）：`1 = fix`、`3 = lat`、`4 = lng`、`5 = alt`。

---

## 关键实现与避坑记录

1. **不要用 `uapi_uart_read` 轮询 AT 响应（hi8363最大的坑）**
   在 WS63 上该接口：① 忽略 `timeout` 参数；② 调用 `uart_porting_lock()` 关中断；③ 必须凑满请求长度才返回，FIFO 空时自旋不让出 CPU。
   用它轮询不定长 AT 响应时，模块还没回数据就会关中断 100% 自旋，喂不到看门狗，最终 NMI 复位（表现为 `Air780Task` CPU 占用 100% 后崩溃）。
   正确做法：`uapi_uart_register_rx_callback()` 注册中断回调（`UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE`，阈值 1 字节），把字节推进环形缓冲区；任务侧只从缓冲区取数据，每轮 `osDelay(1)` 让出 CPU，绝不调用 `uapi_uart_read`。

2. **两类 AT 响应读取方式**
   - `Air780UartRead()`：用于命令响应，遇到 `OK\r\n` / `ERROR\r\n` 提前返回；
   - `Air780UartWaitToken()`：用于等待 `CONNECT OK` / `CONNACK OK` 等异步 URC，缓冲区将满时保留末尾 `strlen(token)-1` 字节，保证关键字跨批次拼接后仍能匹配。

3. **Air780EG 冷启动很难定位**
   纯冷启动时可视卫星少、fix 长期为 0；说明书要求打开 GNSS 后再发 `AT+CGNSAID=<mode>,<time>,<epo>,<loc>`。代码在网络附着成功后使能三项辅助（AGNSS/EPO/位置），EPO 星历需联网下载，通常数秒即可定位。

4. **精简 libc 不支持浮点格式化**
   平台不保证 `printf("%f")` / `sscanf` / `atof` 可用，`air780_modem.c` 全部手写整数解析，`air780_telemetry.c` 用定点方式拼接 JSON，避免任何 `%f` 依赖。
   （`air780_app.c` 的 NITZ 对时用了 `sscanf` 解析 `+CCLK`，仅解析整数时间字段，同样不依赖 `%f`。）

5. **`AT+MPUB` payload 转义**
   发布前自动转义：`"` → `\22`、`\r` → `\0D`、`\n` → `\0A`、`\` → `\5C`。

6. **MQTT 连接状态维护**
   任何一次发布失败都会把内部连接状态置为断开，主循环下个周期自动重连；重连成功后会重新订阅 `alpha/1/cmd` 并发送上线通知。

7. **NITZ 基站对时**
   联网后先 `AT+CTZU=1` 打开自动时间更新，再 `AT+CCLK?` 读取；仅当年份在 `24~99` 之间才认为基站时间已校准，通过主板提供的 `Set_Global_Time()`（跨静态库链接）注入拓展板，成功一次后停止。

8. **单任务串行访问 AT**
   所有 AT 收发都在 `Air780Task` 内串行执行，同一时刻只有一个任务操作 UART，无需加锁。

---

## API 速查

### `air780_uart.h`

| 接口 | 说明 |
| --- | --- |
| `int Air780UartInit(void)` | 初始化 UART2 并注册中断接收回调，0 成功 / -1 失败 |
| `void Air780UartDeinit(void)` | 注销回调并释放 UART |
| `int Air780SendAT(cmd, resp, size, timeout)` | 发送 AT 并读取响应（发送前清空残留数据） |
| `int Air780UartRead(buf, maxlen, timeout)` | 从环形缓冲区读取响应，遇 `OK` / `ERROR` 提前返回 |
| `int Air780UartWaitToken(token, scratch, cap, timeout)` | 等待指定 URC 关键字，1 命中 / 0 超时 |
| `void Air780Wakeup(void)` | 连发 `AT` 唤醒模组并关闭回显 |

### `air780_modem.h`

| 接口 | 说明 |
| --- | --- |
| `int Air780ModemPrepare(void)` | 关回显 + 开 GNSS + 等待驻网 + 使能 AGNSS，0 成功 / -1 超时 |
| `int Air780ModemCsq(void)` | 读取信号质量（0~31，99 未知，-1 失败） |
| `int Air780ModemGps(air780_gps_t *g)` | 解析 `+CGNSINF` 到 `g`，1 解析成功 / 0 失败（是否定位看 `g->fix`） |
| `int Air780ModemAttached(void)` | 查询是否已附着网络，1 已附着 / 0 未附着 |

### `air780_mqtt.h`

| 接口 | 说明 |
| --- | --- |
| `void Air780MqttSetup(host, port, clientid, user, pass, keepalive)` | 保存连接配置（不发起连接），`user`/`pass` 传 `NULL` 表示匿名 |
| `int Air780MqttConnect(void)` | 完整建链：`MCONFIG → MIPSTART → MCONNECT`，0 成功 / -1 失败 |
| `void Air780MqttDisconnect(void)` | 断开 MQTT |
| `int Air780MqttConnected(void)` | 查询内部连接状态 |
| `int Air780MqttPublish(topic, payload)` | 发布消息（QoS0 / retain0），失败自动标记断线 |

### `air780_telemetry.h`

| 接口 | 说明 |
| --- | --- |
| `Air780TeleSetGps / SetCsq / SetImu / SetBattery / SetNtcTemp / SetTemp / SetHumidity` | 写入遥测数据中心（`SetImu` 同时执行边缘计算累加） |
| `Air780TeleSetDeviceId / SetUploadInterval / GetUploadInterval` | 设置设备 ID 与上传间隔 |
| `Air780BuildMainJson(out, cap)` | 生成主通道遥测 JSON，返回写入长度 |
| `Air780BuildStatusJson(out, cap)` | 生成 `{device_id, csq, fix}` 状态 JSON |
| `Air780ChannelConfig(idx, topic, fn)` / `Air780ChannelEnable(idx, enable)` | 配置 / 启停通道（`AIR780_CH_MAIN` ~ `AIR780_CH_AUX4`） |
| `Air780ChannelPublishAll(void)` | 发布所有已启用通道，返回成功数量；失败返回 -1 |

---

## 注意事项

- **`air780_test.c` 已退役**：它是早期独立诊断任务（定时探活 + UART 重建），同样使用 `APP_FEATURE_INIT` 注册任务，与 `air780_app.c` 同时编入会冲突。当前 `BUILD.gn` 未包含它，仅作 UART 自检 / 恢复思路参考。
- **上传公开仓库前**：请检查 `air780_app.c` 中的 `MQTT_HOST`、`MQTT_CLIENTID` 等是否为可公开的演示配置。
- **时基**：本板 1 tick = 10ms，`AIR780_DELAY_MS(ms)` 宏负责毫秒到 tick 的换算。
- **依赖**：代码依赖 WS63 SDK 的 `uapi_uart_*` 接口与特定目录结构（见 `BUILD.gn` 的 `include_dirs`），SDK 版本不同时路径可能需调整。
- **MQTT 密码**：默认匿名连接，生产环境建议在 broker 侧开启认证；当前 `MQTT_USER` / `MQTT_PASS` 传入非空即启用用户名密码。

---

## 参考资源

- 合宙 Air780EG 系列 AT 指令手册与硬件设计手册：<https://doc.openluat.com/>
- OpenHarmony 官网：<https://www.openharmony.cn/>
- OpenHarmony 源码（Gitee）：<https://gitee.com/openharmony>

---
