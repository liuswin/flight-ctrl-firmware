/**
 * flight-ctrl 固件（ESP-IDF 版）— ESP32-C2 + GY-61(ADXL335) 体感飞控手柄
 *
 * 功能：
 *   1. 上电建立 WiFi 热点（FlightCtrl-XXXX）
 *   2. 开机 1 秒静止校准（此刻握姿即"中立位"）
 *   3. 50Hz 采样三轴加速度 → 横滚/俯仰 → 低通滤波
 *   4. 40Hz 向所有 WebSocket 客户端推送 JSON（ws://192.168.4.1/ws）
 *   5. 内置调试面板（http://192.168.4.1/）
 *
 * 数据：{"roll":-12.3,"pitch":4.5,"cr":-0.41,"cp":0.15,"ms":123456}
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/param.h>

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
#include "driver/gpio.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "sdkconfig.h"

#include "config.h"
#include "dashboard.h"

static const char *TAG = "flight-ctrl";

// ---------------- 状态 ----------------
static float g_roll = 0, g_pitch = 0;         // 滤波后的相对角度（度）
static float g_roll_ref = 0, g_pitch_ref = 0; // 校准基准角
static float g_gravity_norm = 1.0f;
static int64_t g_boot_ms;

static adc_oneshot_unit_handle_t g_adc1;
static adc_cali_handle_t g_adc_cali[3];       // X/Y/Z 各一路校准句柄（可能为 NULL）
static httpd_handle_t g_server;

// WebSocket 客户端 fd 注册表
#define WS_MAX_CLIENTS 4
static int  g_ws_fd[WS_MAX_CLIENTS];
static bool g_ws_used[WS_MAX_CLIENTS];

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

// 由重力向量算横滚/俯仰（度）
static void read_angles(float *roll_out, float *pitch_out, float *gnorm_out) {
  float gx = mv_to_g(read_axis_mv(0, PIN_ADXL_X_CHAN));
  float gy = mv_to_g(read_axis_mv(1, PIN_ADXL_Y_CHAN));
  float gz = mv_to_g(read_axis_mv(2, PIN_ADXL_Z_CHAN));
  *pitch_out = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * (180.0f / M_PI);
  *roll_out  = atan2f(gy, gz) * (180.0f / M_PI);
  if (gnorm_out) *gnorm_out = sqrtf(gx * gx + gy * gy + gz * gz);
}

static void adc_init(void) {
  adc_oneshot_unit_init_cfg_t unit_cfg = {
    .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &g_adc1));

  adc_channel_t chans[3] = {PIN_ADXL_X_CHAN, PIN_ADXL_Y_CHAN, PIN_ADXL_Z_CHAN};
  for (int i = 0; i < 3; i++) {
    adc_oneshot_chan_cfg_t chan_cfg = {
      .bitwidth = ADC_BITWIDTH_12,
      .atten = ADC_ATTEN_DB_12,   // 量程覆盖 ~3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1, chans[i], &chan_cfg));

    // C2 支持线性拟合校准（无出厂 eFuse 数据时用默认系数；
    // 相对角度方案下残余误差会在开机校准时抵消）
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

// ---------------- WebSocket 客户端管理 ----------------
int ws_client_count(void);

static void ws_register(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (g_ws_used[i] && g_ws_fd[i] == fd) return;   // 已注册
  }
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (!g_ws_used[i]) {
      g_ws_used[i] = true;
      g_ws_fd[i] = fd;
      ESP_LOGI(TAG, "WS 客户端接入 fd=%d（共 %d 个）", fd, ws_client_count());
      return;
    }
  }
  ESP_LOGW(TAG, "WS 客户端满，拒绝 fd=%d", fd);
}

static void ws_unregister(int fd) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (g_ws_used[i] && g_ws_fd[i] == fd) {
      g_ws_used[i] = false;
      ESP_LOGI(TAG, "WS 客户端断开 fd=%d", fd);
      return;
    }
  }
}

int ws_client_count(void) {
  int n = 0;
  for (int i = 0; i < WS_MAX_CLIENTS; i++) n += g_ws_used[i];
  return n;
}

// 广播任务：通过 httpd_queue_work 切到 httpd 任务上下文再发送
typedef struct {
  int fd;
  char payload[96];
} ws_send_job_t;

static void ws_send_worker(void *arg) {
  ws_send_job_t *job = arg;
  httpd_ws_frame_t pkt = {
    .type = HTTPD_WS_TYPE_TEXT,
    .payload = (uint8_t *)job->payload,
    .len = strlen(job->payload),
    .final = true,
  };
  if (httpd_ws_send_frame_async(g_server, job->fd, &pkt) != ESP_OK) {
    ws_unregister(job->fd);   // 发送失败视为断线
  }
  free(job);
}

static void ws_broadcast(const char *json) {
  for (int i = 0; i < WS_MAX_CLIENTS; i++) {
    if (!g_ws_used[i]) continue;
    ws_send_job_t *job = malloc(sizeof(*job));
    if (!job) continue;
    job->fd = g_ws_fd[i];
    strlcpy(job->payload, json, sizeof(job->payload));
    if (httpd_queue_work(g_server, ws_send_worker, job) != ESP_OK) {
      ws_unregister(job->fd);
      free(job);
    }
  }
}

// ---------------- HTTP / WS 处理 ----------------
static esp_err_t dashboard_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t health_get_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ws_handler(httpd_req_t *req) {
  int fd = httpd_req_to_sockfd(req);
  ws_register(fd);

  // payload=NULL：由服务端内部缓冲区接收，max_len 限制长度，指针仅在本次回调内有效
  httpd_ws_frame_t pkt;
  memset(&pkt, 0, sizeof(pkt));

  esp_err_t err = httpd_ws_recv_frame(req, &pkt, 63);
  if (err != ESP_OK) {
    ws_unregister(fd);   // 读失败视为断线
    return ESP_OK;
  }
  if (pkt.type == HTTPD_WS_TYPE_CLOSE) {
    ws_unregister(fd);
    return ESP_OK;
  }
  if (pkt.type == HTTPD_WS_TYPE_TEXT && pkt.payload &&
      strstr((char *)pkt.payload, "center")) {
    // 以当前姿势重新居中：当前绝对角 = 相对角 + 基准角
    g_roll_ref = g_roll + g_roll_ref;
    g_pitch_ref = g_pitch + g_pitch_ref;
    ESP_LOGI(TAG, "已重新居中");
  }
  return ESP_OK;
}

static httpd_handle_t start_http_server(void) {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 4;
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;

  if (httpd_start(&g_server, &cfg) != ESP_OK) return NULL;

  httpd_uri_t uri_dashboard = {
    .uri = "/", .method = HTTP_GET, .handler = dashboard_get_handler,
  };
  httpd_uri_t uri_health = {
    .uri = "/health", .method = HTTP_GET, .handler = health_get_handler,
  };
  httpd_uri_t uri_ws = {
    .uri = "/ws", .method = HTTP_GET, .handler = ws_handler,
    .is_websocket = true, .handle_ws_control_frames = true,
  };
  httpd_register_uri_handler(g_server, &uri_dashboard);
  httpd_register_uri_handler(g_server, &uri_health);
  httpd_register_uri_handler(g_server, &uri_ws);
  return g_server;
}

// ---------------- WiFi 热点 ----------------
static void wifi_init_softap(void) {
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  char ssid[32];
  uint8_t mac[6] = {0};
  ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
  snprintf(ssid, sizeof(ssid), "%s-%02X%02X", AP_SSID_PREFIX, mac[4], mac[5]);

  wifi_config_t wifi_cfg = { 0 };
  strlcpy((char *)wifi_cfg.ap.ssid, ssid, sizeof(wifi_cfg.ap.ssid));
  strlcpy((char *)wifi_cfg.ap.password, AP_PASSWORD, sizeof(wifi_cfg.ap.password));
  wifi_cfg.ap.channel = AP_CHANNEL;
  wifi_cfg.ap.max_connection = 4;
  wifi_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "热点已建立：%s  密码：%s  面板：http://192.168.4.1/", ssid, AP_PASSWORD);
}

// ---------------- 主任务 ----------------
static float apply_deadzone(float a) {
  return (a > -DEADZONE_DEG && a < DEADZONE_DEG) ? 0 : a;
}

static float map_to_control(float angle) {
  float c = angle / MAX_TILT_DEG;
  return c > 1 ? 1 : (c < -1 ? -1 : c);
}

static void control_task(void *arg) {
  (void)arg;
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t next_sample = 0, next_send = 0;
  bool led_level = false;

  while (1) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    uint32_t now = (uint32_t)((esp_timer_get_time() / 1000LL) - g_boot_ms);

    if (now >= next_sample) {
      next_sample = now + SAMPLE_INTERVAL_MS;
      float r, p;
      read_angles(&r, &p, NULL);
      r -= g_roll_ref;
      p -= g_pitch_ref;
      g_roll += EMA_ALPHA * (r - g_roll);
      g_pitch += EMA_ALPHA * (p - g_pitch);
    }

    if (now >= next_send) {
      next_send = now + SEND_INTERVAL_MS;
      if (ws_client_count() > 0) {
        char json[96];
        snprintf(json, sizeof(json),
                 "{\"roll\":%.1f,\"pitch\":%.1f,\"cr\":%.2f,\"cp\":%.2f,\"ms\":%lu}",
                 apply_deadzone(g_roll), apply_deadzone(g_pitch),
                 map_to_control(g_roll), map_to_control(g_pitch),
                 (unsigned long)now);
        ws_broadcast(json);
      }
#if LED_GPIO >= 0
      led_level = !led_level;
      gpio_set_level(LED_GPIO, ws_client_count() > 0 ? led_level : 0);
#endif
    }
  }
}

// ---------------- 入口 ----------------
void app_main(void) {
  g_boot_ms = esp_timer_get_time() / 1000LL;

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

#if LED_GPIO >= 0
  gpio_reset_pin(LED_GPIO);
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
#endif

  adc_init();
  ESP_LOGI(TAG, "开始校准（保持中立握姿静止约 1 秒）...");
  float r = 0, p = 0, gn = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float ri, pi, gi;
    read_angles(&ri, &pi, &gi);
    r += ri; p += pi; gn += gi;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  g_roll_ref = r / CALIB_SAMPLES;
  g_pitch_ref = p / CALIB_SAMPLES;
  g_gravity_norm = gn / CALIB_SAMPLES;
  ESP_LOGI(TAG, "校准完成：基准 roll=%.1f pitch=%.1f 重力模长=%.2fg",
           g_roll_ref, g_pitch_ref, g_gravity_norm);

  wifi_init_softap();
  start_http_server();

  xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}
