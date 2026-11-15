#pragma once

// ============ 硬件接线（ESP32-C2 / ESP8684 + GY-61 ADXL335）============
// GY-61        ESP32-C2
// VCC  ------> 3V3          （绝不能接 5V，ADXL335 极限 3.6V）
// GND  ------> GND
// X    ------> GPIO0        （ADC1_CH0）
// Y    ------> GPIO1        （ADC1_CH1）
// Z    ------> GPIO3        （ADC1_CH3，避开 GPIO2 启动绑带脚）
#define PIN_ADXL_X_CHAN   ADC_CHANNEL_0   // GPIO0
#define PIN_ADXL_Y_CHAN   ADC_CHANNEL_1   // GPIO1
#define PIN_ADXL_Z_CHAN   ADC_CHANNEL_3   // GPIO3

// ADXL335 供电 3.3V 时的典型参数
#define ADXL_ZERO_G_MV   1650   // 0g 输出电压 mV
#define ADXL_SENS_MV_G   330    // 灵敏度 mV/g
#define ADC_VREF_MV      3300   // 无校准时的粗略换算基准
#define ADC_MAX_RAW      4095   // 12-bit

// ============ WiFi ============
// STA 模式：连家里的路由器。
// 密码写在 wifi_secrets.h 里（该文件已被 .gitignore 忽略，不会进仓库）。
// 首次使用：复制 wifi_secrets.h.example 为 wifi_secrets.h 并填入你的 WiFi 信息。
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#define WIFI_SSID  "在这里填你家WiFi的名字"
#define WIFI_PASS  "在这里填你家WiFi的密码"
#endif

// AP 保底模式：STA 连不上时，热点始终可用（IP 固定 192.168.4.1）
#define AP_SSID_PREFIX  "FlightCtrl"   // 热点名前缀，上电追加 MAC 后缀
#define AP_PASSWORD     "flywithme"    // 至少 8 位
#define AP_CHANNEL      1

// STA 连接超时（超时后仅靠 AP 热点模式运行，后台继续重试 STA）
#define STA_TIMEOUT_MS  20000

// mDNS 主机名：连上路由器后可用 http://flightctrl.local 访问
#define MDNS_HOSTNAME   "flightctrl"

// ============ 采样与滤波 ============
#define SAMPLE_INTERVAL_MS   20   // ADC 采样周期（50Hz）
#define SEND_INTERVAL_MS     25   // 数据推送周期（40Hz）
#define EMA_ALPHA            0.3f // 低通滤波系数
#define CALIB_SAMPLES        800  // 开机校准采样数（约 1 秒，期间保持静止）

// ============ 控制映射 ============
#define MAX_TILT_DEG   30.0f  // 倾角满舵角度：±30° 对应游戏里 ±1
#define DEADZONE_DEG    2.0f  // 死区

// ============ 板载 LED（负值则禁用）============
#define LED_GPIO  8   // 常见 ESP32-C2 SuperMini 板载 LED
