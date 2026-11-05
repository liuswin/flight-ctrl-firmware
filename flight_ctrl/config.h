#pragma once

// ============ 硬件接线（ESP32-C2 / ESP8684 + GY-61 ADXL335）============
// GY-61        ESP32-C2
// VCC  ------> 3V3          （绝不能接 5V，ADXL335 极限 3.6V）
// GND  ------> GND
// X    ------> GPIO0        （ADC1_CH0）
// Y    ------> GPIO1        （ADC1_CH1）
// Z    ------> GPIO3        （ADC1_CH3，避开 GPIO2 启动绑带脚）
#define PIN_ADXL_X   0
#define PIN_ADXL_Y   1
#define PIN_ADXL_Z   3

// ADXL335 供电 3.3V 时的典型参数（供电电压波动会等比例影响，量程校准兜底）
#define ADXL_ZERO_G_MV   1650   // 0g 输出电压 mV
#define ADXL_SENS_MV_G   330    // 灵敏度 mV/g

// ============ WiFi 热点 ============
#define AP_SSID_PREFIX  "FlightCtrl"     // 热点名前缀，烧录后会追加 MAC 后缀
#define AP_PASSWORD     "flywithme"      // 至少 8 位
#define AP_CHANNEL      1

// ============ 服务端口 ============
#define HTTP_PORT  80    // 调试面板页面
#define WS_PORT    81    // WebSocket 倾角数据流

// ============ 采样与滤波 ============
#define SAMPLE_INTERVAL_MS   20   // ADC 采样周期（50Hz）
#define SEND_INTERVAL_MS     25   // 数据推送周期（40Hz）
#define EMA_ALPHA            0.3f // 低通滤波系数，越小越平滑越迟钝
#define CALIB_SAMPLES        800  // 开机校准采样数（约 1 秒，期间保持设备静止）

// ============ 控制映射 ============
#define MAX_TILT_DEG   30.0f  // 倾角满舵角度：±30° 对应游戏里 ±1
#define DEADZONE_DEG    2.0f  // 死区，消除手持抖动

// ============ 板载 LED（未定义则自动跳过闪烁）============
#ifdef LED_BUILTIN
  #define PIN_LED  LED_BUILTIN
#elif defined(CONFIG_IDF_TARGET_ESP8684)
  #define PIN_LED  8   // 常见 ESP32-C2 SuperMini 的板载 LED
#endif
