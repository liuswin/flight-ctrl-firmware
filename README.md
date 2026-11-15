# FlightCtrl 固件 — ESP32-C2 + GY-61 体感飞控手柄

ESP32-C2（NodeMCU-ESP32-C2 / ESP8684）读取 GY-61（ADXL335 三轴模拟加速度计），
解算横滚/俯仰倾角，建立 WiFi 热点并通过 WebSocket 推送，用于操控 three.js 飞机游戏。

## 接线

| GY-61 | ESP32-C2 | 说明 |
|---|---|---|
| VCC | 3V3 | **绝不能接 5V**（ADXL335 绝对最大 3.6V） |
| GND | GND | |
| X | GPIO0 | ADC1_CH0 |
| Y | GPIO1 | ADC1_CH1 |
| Z | GPIO3 | ADC1_CH3（避开 GPIO2 启动绑带脚） |

## 使用流程

固件为 **STA+AP 双模**：板子自动连家里路由器（常驻局域网），同时保留
`FlightCtrl-XXXX` 热点作保底（连不上路由器时用，IP 固定 192.168.4.1）。

1. **首次配置**：复制 `idf/main/wifi_secrets.h.example` 为 `idf/main/wifi_secrets.h`，
   填入家里 WiFi 的名称和**2.4GHz**网络（ESP32 不支持 5G！）与密码。
   该文件已被 .gitignore 忽略，不会入库
2. 烧录上电，**保持握姿静止 1 秒**（校准）
3. 电脑（与板子同一路由器下）打开 **http://flightctrl.local** —— 实时倾角面板
   （也可用串口日志里打印的 IP 直接访问）
4. 数据流：**ws://flightctrl.local/ws**（浏览器/Node/python 客户端连接后发一帧
   `{"cmd":"hello"}` 开始接收广播）
5. 面板"重新居中"按钮：发送 `{"cmd":"center"}`

## 数据协议

40Hz JSON 文本帧：

```json
{"roll":-12.3,"pitch":4.5,"cr":-0.41,"cp":0.15,"ms":123456}
```

- `roll` / `pitch`：相对中立位的角度（度）
- `cr` / `cp`：按 ±30° 满舵映射到 [-1, 1] 的归一化控制量（游戏端直接用这两个）
- `ms`：设备毫秒时间戳

反向指令（面板"重新居中"按钮）：`{"cmd":"center"}`

## 为什么有两套固件？

| 目录 | 框架 | 状态 |
|---|---|---|
| `idf/` | **ESP-IDF v5.5** | **主力**。ESP32-C2 只有 IDF 是官方完整支持的路线 |
| `flight_ctrl/` | Arduino | 备用。**上游 arduino-esp32 从未发布过 C2 的 SDK 库包**（esp32c2-libs 缺失，编译会失败），只能用于 ESP32/C3/S3 等其他芯片 |

两版功能、数据协议、面板完全一致。

## 构建（ESP-IDF 版）

```bash
# 首次：进入 IDF 环境
source ~/esp/esp-idf/export.sh

# 编译（在 firmware/idf/ 目录下）
idf.py set-target esp32c2   # 首次执行
idf.py build

# 烧录 + 看日志（板子 USB 连接后，/dev/cu.usbserial-* 或 /dev/cu.wchusb*）
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

> ESP32-C2 没有原生 USB，走板载 USB 转串口芯片（CH340/CH9102）烧录。
> 若烧录卡在 "Connecting..."：按住 BOOT 键，点一下 RST，松开 BOOT。

## 构建（Arduino 版，备用）

```bash
arduino-cli compile --fqbn esp32:esp32:esp32c3 \
  --libraries ./libs flight_ctrl     # 换成实际芯片的 fqbn
```

## 参数调优（`idf/main/config.h` 或 `flight_ctrl/config.h`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `MAX_TILT_DEG` | 30 | 满舵倾角，调小 = 更灵敏 |
| `EMA_ALPHA` | 0.3 | 低通滤波，调小 = 更平滑更迟钝 |
| `DEADZONE_DEG` | 2 | 死区，抑制手持抖动 |
| `SEND_INTERVAL_MS` | 25 | 推送频率（40Hz） |

## 已知限制

- ADXL335 是纯加速度计：只能测横滚/俯仰（重力方向），**测不了偏航**；
  快速挥动时读数会被动态加速度污染（低通滤波可缓解，无法根除）
- ESP32 的 ADC 存在非线性，±30° 范围内影响很小；"相对角度 + 开机校准"方案
  会把传感器的零偏误差自动抵消掉

## 早期 ESP8684 工程样片排障实录（2026-11）

实测一块 DOIT ESP-C2-12 载板（FCC 2AL3B-ESPC2-12，ROM `esp8684-api2-20220127`，
efuse v0.1）为**量产前工程样片**，固件层已修复三个 erratum，最终仍有一个硅片级
缺陷无法绕过。全记录供同款板参考：

| # | 症状 | 根因 | 修复（已含在本固件） |
|---|---|---|---|
| 1 | `assert failed: xTaskDelayUntil (xTimeIncrement > 0)`，~1.5s 重启循环 | IDF 默认 tick 100Hz，5ms 循环换算 tick=0 | `sdkconfig.defaults` 设 `CONFIG_FREERTOS_HZ=1000` + 代码兜底 `loop_tick` |
| 2 | `rst:0x7 TG0WDT_SYS_RST`，`app_main` 返回后 ~2ms 静默硬复位 | bootloader 的 TG0 flashboot 看门狗无人接管 | `kill_flashboot_wdt()`：TIMG_WDTWPROTECT 密钥 `0x50D83AA1` 后写 0 |
| 3 | `rst:0x10 RTCWDT_RTC_RST`，~7.2s 周期复位；C2 的 IDF 无 RTC WDT 驱动且头文件密钥(0xFFFFFFFF)是占位错误 | RTC 看门狗无人喂 | 同函数裸寄存器关闭，实测有效密钥同为 `0x50D83AA1` |
| 4 | 二次启动起 `Image hash failed`，GD 闪存"损坏"循环 | WDT 暴毙时闪存状态异常的次生现象 | 随 #2/#3 修复自动消失，无需降频 |
| 5 | **WiFi AP 启动 ~1.5s 后 systimer 计数器冻结**：esp_timer 停走、所有任务 delay 后永眠、串口静默、信标停发（热点空中不可见）。LED 常亮实验证实调度器冻结 | 硅片级缺陷（IDF v5.1/v5.5 均复现；`esp_wifi_set_ps(WIFI_PS_NONE)` 无效） | **无法绕过**，判定工程样片不合格 |

其他实证：该模组手册的 UART0/下载引脚与标准 ESP32-C2 不同（RXD0/TXD0=GPIO19/20、
下载引脚 IO9 而非 GPIO8、GPIO8 未引出）；载板 USB-C 座子缺 CC 下拉电阻（C-C 线
不供电，A-C 数据线可用）；串口日志波特率 74880（非 115200）。

**结论**：同款板若症状一致，直接换板（推荐带原生 USB 的 ESP32-C3 DevKitM-1，
本固件 `idf.py set-target esp32c3` 即可移植）。

## 硬件排障记录（2026-11 第一块 DOIT ESP-C2-12 载板，判定损坏）

症状与排查结论，供退换举证或遇到同类问题时参考：

| 排查项 | 方法 | 结果 |
|---|---|---|
| USB-C 座子 | C-C 线直连 Mac（LED 不亮）| CC 电阻缺陷，C 对 C 完全不供电（充电头 A 口可供电）|
| USB 数据 | hub A 口 + A-C 线 ×4 | 零 USB 枚举事件（手头 A-C 线均为纯充电线，无数据线可验证板载串口芯片）|
| USB-TTL 直刷（排针） | 两组 TX/RX × 交叉/直连，esptool 200+ 次同步 | 全部无响应 |
| USB-TTL 直刷（模组焊盘） | 胶带固定针尖于模组 RXD0/TXD0 + IO9 上电进下载模式 + 即时同步 | 仍无响应 |
| TTL 适配器自身 | TX-RX 短接回环 ×3 | 完美，适配器正常 |
| 下载模式入口 | 模组手册确认 IO9=下载引脚（非标准 C2 的 GPIO8），上电采样流程正确 | 流程正确仍无法握手 |

判定：模组本体 UART0/芯片损坏或模组-载板系统性虚焊。环境与固件均正常，换板即用。
注：该模组手册标注 UART0 为 RXD0/TXD0（映射 GPIO19/20）、下载引脚为 IO9，与标准 ESP32-C2（GPIO21/20、GPIO8）不同，属厂家定制。

## 国内网络说明

本项目工具链全部走乐鑫官方镜像（dl.espressif.com），如需重装环境：

```bash
# IDF 克隆（gitee 镜像）
git clone --depth 1 -b v5.5 --recursive --shallow-submodules \
  https://gitee.com/EspressifSystems/esp-idf.git ~/esp/esp-idf

# 工具链下载走乐鑫 CDN
export IDF_GITHUB_ASSETS="dl.espressif.com/github_assets"
cd ~/esp/esp-idf && ./install.sh esp32c2
```
