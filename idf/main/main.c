/**
 * flight-ctrl 固件 — ESP32-C3 + GY-61(ADXL335) 体感飞控手柄
 *
 * 功能：
 *   1. 上电连家里路由器（STA）+ 保底热点（AP），mDNS 注册 flightctrl.local
 *   2. 开机 1 秒静止校准（此刻握姿即"中立位"）
 *   3. 50Hz 采样三轴加速度 → 横滚/俯仰 → 低通滤波
 *   4. HTTP 轮询接口（客户端 25ms 拉一次即 40Hz）：
 *      GET /tilt   → {"roll":-12.3,"pitch":4.5,"cr":-0.41,"cp":0.15,"ms":123456}
 *      GET /center → 以当前姿势重新居中
 *      GET /       → 调试面板页面
 *      GET /health → {"status":"ok"}
 *
 * 说明：曾实现 WebSocket 服务端推送，但 IDF 的 queue_work + 异步发送组合
 * 在持续 40Hz 广播下会死锁 httpd 任务，改为客户端轮询架构（稳定可靠）。
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "mdns.h"
#include "driver/gpio.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include "soc/rtc_cntl_reg.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "sdkconfig.h"

#include "config.h"
#include "dashboard.h"

static const char *TAG = "flight-ctrl";

// ---------------- 状态 ----------------
static float g_roll = 0, g_pitch = 0;          // 滤波后的相对角度（度）
static float g_g0[3] = {0, 0, 1};              // 校准时的参考重力向量（单位 g）
static httpd_handle_t g_server;

static adc_oneshot_unit_handle_t g_adc1;
static adc_cali_handle_t g_adc_cali[3];

// ---------------- ADC ----------------
static float read_axis_mv(int idx, adc_channel_t chan) {
  int raw = 0;
  ESP_ERROR_CHECK(adc_oneshot_read(g_adc1, chan, &raw));
  int mv = raw * ADC_VREF_MV / ADC_MAX_RAW;
  if (g_adc_cali[idx]) {
    adc_cali_raw_to_voltage(g_adc_cali[idx], raw, &mv);
  }
  return (float)mv;
}

static float mv_to_g(float mv) {
  return (mv - ADXL_ZERO_G_MV) / ADXL_SENS_MV_G;
}

// 读三轴加速度（单位 g）
static void read_accel(float *gx, float *gy, float *gz) {
  *gx = mv_to_g(read_axis_mv(0, PIN_ADXL_X_CHAN));
  *gy = mv_to_g(read_axis_mv(1, PIN_ADXL_Y_CHAN));
  *gz = mv_to_g(read_axis_mv(2, PIN_ADXL_Z_CHAN));
}

// 当前重力向量相对参考向量 g0 的旋转角（度）。
// 用叉积/点积直接算"相对角"，而不是两个绝对角相减——
// 避开 atan2 在 ±180° 的断点（板子安装方向靠近断点时，
// 绝对角相减会出现 -340° 之类的跳变）。相对角在 ±180° 内连续。
static void rel_angles(float gx, float gy, float gz,
                       float *roll_out, float *pitch_out) {
  *roll_out  = atan2f(g_g0[1] * gz - g_g0[2] * gy,
                      g_g0[1] * gy + g_g0[2] * gz) * (180.0f / M_PI);
  *pitch_out = atan2f(g_g0[2] * gx - g_g0[0] * gz,
                      g_g0[0] * gx + g_g0[2] * gz) * (180.0f / M_PI);
}

static void adc_init(void) {
  adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc1));

  adc_channel_t chans[3] = {PIN_ADXL_X_CHAN, PIN_ADXL_Y_CHAN, PIN_ADXL_Z_CHAN};
  for (int i = 0; i < 3; i++) {
    adc_oneshot_chan_cfg_t chan_cfg = {
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_12,   // 量程覆盖 ~3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1, chans[i], &chan_cfg));

    g_adc_cali[i] = NULL;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_line_fitting(&cali_cfg, &g_adc_cali[i]);
#elif ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_1,
      .chan = chans[i],
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &g_adc_cali[i]);
#endif
  }
}

// ---------------- 映射 ----------------
static float apply_deadzone(float a) {
  return (a > -DEADZONE_DEG && a < DEADZONE_DEG) ? 0 : a;
}
static float map_to_control(float angle) {
  float c = angle / MAX_TILT_DEG;
  return c > 1 ? 1 : (c < -1 ? -1 : c);
}

// ---------------- HTTP 服务 ----------------
static esp_err_t tilt_get_handler(httpd_req_t *req) {
  char json[96];
  snprintf(json, sizeof(json),
           "{\"roll\":%.1f,\"pitch\":%.1f,\"cr\":%.2f,\"cp\":%.2f,\"ms\":%lu}",
           apply_deadzone(g_roll), apply_deadzone(g_pitch),
           map_to_control(g_roll), map_to_control(g_pitch),
           (unsigned long)(esp_timer_get_time() / 1000LL));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 游戏页面跨域拉取
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t center_get_handler(httpd_req_t *req) {
  // 重新居中 = 以当前重力向量为新参考（快速采 64 次取平均，抗手抖）
  float sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 64; i++) {
    float gx, gy, gz;
    read_accel(&gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  g_g0[0] = sx / 64; g_g0[1] = sy / 64; g_g0[2] = sz / 64;
  g_roll = 0; g_pitch = 0;
  ESP_LOGI(TAG, "已重新居中");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t dashboard_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t health_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_http_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 6;
  cfg.server_port = 80;

  if (httpd_start(&g_server, &cfg) != ESP_OK) return NULL;

  httpd_uri_t uris[] = {
    { .uri = "/",      .method = HTTP_GET, .handler = dashboard_get_handler },
    { .uri = "/tilt",  .method = HTTP_GET, .handler = tilt_get_handler },
    { .uri = "/center",.method = HTTP_GET, .handler = center_get_handler },
    { .uri = "/health",.method = HTTP_GET, .handler = health_get_handler },
  };
  for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
    httpd_register_uri_handler(g_server, &uris[i]);
  }
  return g_server;
}

// ---------------- WiFi：STA 连家里路由器 + AP 保底热点 ----------------
static EventGroupHandle_t s_wifi_events;
#define EVT_GOT_IP BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();   // 断线自动重连
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "已连入路由器，板子局域网 IP: " IPSTR, IP2STR(&evt->ip_info.ip));
    if (mdns_init() == ESP_OK) {
      mdns_hostname_set(MDNS_HOSTNAME);
      mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
      ESP_LOGI(TAG, "mDNS 已注册：http://%s.local", MDNS_HOSTNAME);
    }
    xEventGroupSetBits(s_wifi_events, EVT_GOT_IP);
  }
}

static void wifi_init_all(void) {
  s_wifi_events = xEventGroupCreate();
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL));

  char ssid[32];
  uint8_t mac[6] = {0};
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
  snprintf(ssid, sizeof(ssid), "%s-%02X%02X", AP_SSID_PREFIX, mac[4], mac[5]);

  wifi_config_t ap_cfg = { 0 };
  strlcpy((char *)ap_cfg.ap.ssid, ssid, sizeof(ap_cfg.ap.ssid));
  strlcpy((char *)ap_cfg.ap.password, AP_PASSWORD, sizeof(ap_cfg.ap.password));
  ap_cfg.ap.channel = AP_CHANNEL;
  ap_cfg.ap.max_connection = 4;
  ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

  wifi_config_t sta_cfg = { 0 };
  strlcpy((char *)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid));
  strlcpy((char *)sta_cfg.sta.password, WIFI_PASS, sizeof(sta_cfg.sta.password));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
  // 早期 ESP8684 硅片 erratum：modem sleep 冻结 systimer；关闭省电（对 C3 无害）
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_connect();

  ESP_LOGI(TAG, "保底热点：%s（密码 %s，IP 192.168.4.1）", ssid, AP_PASSWORD);
  ESP_LOGI(TAG, "正在连接路由器 \"%s\"（最多等 %d 秒）...",
           WIFI_SSID, STA_TIMEOUT_MS / 1000);

  EventBits_t bits = xEventGroupWaitBits(s_wifi_events, EVT_GOT_IP,
                                         pdFALSE, pdFALSE,
                                         pdMS_TO_TICKS(STA_TIMEOUT_MS));
  if (!(bits & EVT_GOT_IP)) {
    ESP_LOGW(TAG, "路由器连接超时，继续后台重试；当前可用保底热点 %s", ssid);
  }
}

// ---------------- 主任务 ----------------
static void control_task(void *arg) {
  (void)arg;
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t next_sample = 0;
  bool led_level = false;
  // tick 频率若低于 200Hz，pdMS_TO_TICKS(5) 会等于 0 导致断言崩溃，兜底防御
  const TickType_t loop_tick = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;

  while (1) {
    vTaskDelayUntil(&last_wake, loop_tick);
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (now >= next_sample) {
      next_sample = now + SAMPLE_INTERVAL_MS;
      float gx, gy, gz, r, p;
      read_accel(&gx, &gy, &gz);
      rel_angles(gx, gy, gz, &r, &p);
      g_roll  += EMA_ALPHA * (r - g_roll);
      g_pitch += EMA_ALPHA * (p - g_pitch);
    }

#if LED_GPIO >= 0
    // 1Hz 心跳灯：活着就在闪
    static uint32_t last_led = 0;
    if (now - last_led >= 500) {
      last_led = now;
      led_level = !led_level;
      gpio_set_level(LED_GPIO, led_level);
    }
#endif
  }
}

// ---------------- 入口 ----------------
// 早期 ESP8684 硅片（ROM 2022-01）erratum：ROM 启动的 TG0 flashboot 看门狗
// 不会被 IDF 5.5 的常规流程关闭，~1.7s 后静默硬复位（rst:0x7 TG0WDT_SYS_RST）。
// 这里在进 main 后第一件事用寄存器操作强杀它。
static void kill_flashboot_wdt(void) {
  WRITE_PERI_REG(TIMG_WDTWPROTECT_REG(0), 0x50D83AA1);   // 解除写保护
  WRITE_PERI_REG(TIMG_WDTCONFIG0_REG(0), 0);             // 关 WDT（含 flashboot_en）
  WRITE_PERI_REG(TIMG_WDTCONFIG1_REG(0), 0);             // 超时清零
  WRITE_PERI_REG(TIMG_WDTWPROTECT_REG(0), 0);            // 恢复写保护
  // 同一硅片上 bootloader 还会启用 RTC 看门狗（rst:0x10 RTCWDT_RTC_RST）。
  // C2 的 IDF 没有 RTC WDT 驱动且头文件密钥(0xFFFFFFFF)是占位错误，
  // 实测有效密钥与其他 ESP32 相同为 0x50D83AA1。
  const uint32_t keys[] = {0x50D83AA1, 0xFFFFFFFF};
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
    WRITE_PERI_REG(RTC_CNTL_WDTWPROTECT_REG, keys[i]);
    WRITE_PERI_REG(RTC_CNTL_WDTCONFIG0_REG, 0);   // WDT_EN=0，所有阶段关闭
    WRITE_PERI_REG(RTC_CNTL_WDTCONFIG1_REG, 0);   // 超时清零
    bool done = READ_PERI_REG(RTC_CNTL_WDTCONFIG0_REG) == 0;
    WRITE_PERI_REG(RTC_CNTL_WDTWPROTECT_REG, 0);
    if (done) break;
  }
}

void app_main(void) {
  kill_flashboot_wdt();

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

#if LED_GPIO >= 0
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
#endif

  adc_init();
  ESP_LOGI(TAG, "开始校准（保持中立握姿静止约 1 秒）...");
  float sx = 0, sy = 0, sz = 0, gn = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float gx, gy, gz;
    read_accel(&gx, &gy, &gz);
    sx += gx; sy += gy; sz += gz;
    gn += sqrtf(gx * gx + gy * gy + gz * gz);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  g_g0[0] = sx / CALIB_SAMPLES;
  g_g0[1] = sy / CALIB_SAMPLES;
  g_g0[2] = sz / CALIB_SAMPLES;
  ESP_LOGI(TAG, "校准完成：参考向量 (%.2f, %.2f, %.2f) 重力模长=%.2fg",
           g_g0[0], g_g0[1], g_g0[2], gn / CALIB_SAMPLES);

  wifi_init_all();
  start_http_server();

  xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}
