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

// ============ WiFi 热点 ============
#define AP_SSID_PREFIX  "FlightCtrl"   // 热点名前缀，上电追加 MAC 后两字节
#define AP_PASSWORD     "flywithme"    // 至少 8 位
#define AP_CHANNEL      1

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
