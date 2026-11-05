/**
 * flight-ctrl 固件 — ESP32-C2 + GY-61(ADXL335) 体感飞控手柄
 *
 * 功能：
 *   1. 上电建立 WiFi 热点（SSID: FlightCtrl-XXXX）
 *   2. 开机 1 秒静止校准（此时握持的姿势即“中立位”）
 *   3. 50Hz 采样三轴加速度 → 解算横滚/俯仰角 → 低通滤波
 *   4. 40Hz 通过 WebSocket 推送 JSON 数据流（ws://192.168.4.1:81）
 *   5. 内置调试面板（浏览器打开 http://192.168.4.1 实时看倾角）
 *
 * 数据格式（JSON，一行一条）：
 *   {"roll":-12.3,"pitch":4.5,"cr":-0.41,"cp":0.15,"ms":123456}
 *   roll/pitch = 相对中立位的角度（度）；cr/cp = 按 ±30° 满舵归一化到 [-1,1]
 */
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "dashboard_html.h"

// ---------------- 状态 ----------------
static float g_roll = 0, g_pitch = 0;          // 滤波后的相对角度（度）
static float g_rollRef = 0, g_pitchRef = 0;    // 校准时的基准角度
static bool  g_calibrated = false;
static float g_gravityNorm = 1.0f;             // 校准时重力模长，用于粗略检查
static uint32_t g_lastSampleMs = 0, g_lastSendMs = 0;
static uint32_t g_msgCount = 0;
static uint8_t  g_wsClients = 0;

static WebSocketsServer g_ws(WS_PORT);
static WiFiServer g_http(HTTP_PORT);

// ---------------- ADC → 加速度 ----------------
// 读一路 ADC，返回加速度（单位 g）。analogReadMilliVolts 自带出厂校准曲线
static float readAxisG(int pin) {
  uint32_t mv = analogReadMilliVolts(pin);
  return ((float)mv - ADXL_ZERO_G_MV) / ADXL_SENS_MV_G;
}

// 由重力向量算横滚/俯仰（度）。板子水平时两角约为 0
static void readAngles(float &rollOut, float &pitchOut) {
  float ax = readAxisG(PIN_ADXL_X);
  float ay = readAxisG(PIN_ADXL_Y);
  float az = readAxisG(PIN_ADXL_Z);
  // 俯仰：绕 Y 轴；横滚：绕 X 轴。atan2 自动处理象限
  pitchOut = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  rollOut  = atan2f(ay, az) * 57.2958f;
}

// 开机静止校准：把当前握持姿势记为中立位
static void calibrate() {
  float rAvg = 0, pAvg = 0;
  float ax = 0, ay = 0, az = 0;
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float r, p;
    readAngles(r, p);
    rAvg += r; pAvg += p;
    ax += readAxisG(PIN_ADXL_X); ay += readAxisG(PIN_ADXL_Y); az += readAxisG(PIN_ADXL_Z);
    delay(1);
  }
  g_rollRef  = rAvg / CALIB_SAMPLES;
  g_pitchRef = pAvg / CALIB_SAMPLES;
  g_gravityNorm = sqrtf((ax*ax + ay*ay + az*az)) / CALIB_SAMPLES;
  g_calibrated = true;
}

// ---------------- 归一化映射 ----------------
static float applyDeadzone(float a) {
  if (a > -DEADZONE_DEG && a < DEADZONE_DEG) return 0;
  return a;
}
static float mapToControl(float angle) {
  float c = angle / MAX_TILT_DEG;
  if (c >  1) c =  1;
  if (c < -1) c = -1;
  return c;
}

// ---------------- WebSocket ----------------
static void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] 客户端 %u 已连接\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WS] 客户端 %u 断开\n", num);
      break;
    case WStype_TEXT: {
      // 支持的指令：{"cmd":"center"} 重新以当前姿势居中
      if (strstr((char *)payload, "center") != nullptr) {
        g_rollRef = g_roll + g_rollRef;   // g_roll 是相对角，叠加上基准即当前绝对角
        g_pitchRef = g_pitch + g_pitchRef;
        Serial.println("[WS] 已重新居中");
      }
      break;
    }
    default:
      break;
  }
}

// ---------------- 极简 HTTP（只服务调试面板）----------------
static void serveHttp() {
  WiFiClient client = g_http.accept();
  if (!client) return;

  // 只读第一行请求行（带 300ms 超时），够用且不阻塞主循环
  String req = "";
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {
    if (client.available()) {
      req = client.readStringUntil('\r');
      break;
    }
    delay(1);
  }
  if (req.startsWith("GET / ") || req.startsWith("GET /index.html")) {
    client.printf("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\n"
                  "Content-Length: %u\r\nConnection: close\r\n\r\n",
                  (unsigned)strlen(DASHBOARD_HTML));
    client.print(FPSTR(DASHBOARD_HTML));
  } else if (req.startsWith("GET /health")) {
    client.print(F("HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n"
                   "{\"status\":\"ok\"}"));
  } else if (req.length() > 0) {
    client.print(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n"));
  }
  client.flush();
  client.stop();
}

// ---------------- LED 状态灯 ----------------
static void blinkLed() {
#ifdef PIN_LED
  static uint32_t last = 0;
  static bool level = false;
  // 有连接快闪 150ms，无连接慢闪 800ms
  uint32_t period = g_wsClients > 0 ? 150 : 800;
  if (millis() - last >= period) {
    last = millis();
    level = !level;
    digitalWrite(PIN_LED, level);
  }
#endif
}

// ---------------- Arduino 入口 ----------------
void setup() {
  Serial.begin(115200);
#ifdef PIN_LED
  pinMode(PIN_LED, OUTPUT);
#endif
  analogReadResolution(12);

  Serial.println("\n[flight-ctrl] 启动，开始校准（保持静止/保持你想要的中立握姿）...");
  calibrate();
  Serial.printf("[flight-ctrl] 校准完成：基准 roll=%.1f° pitch=%.1f°，重力模长 %.2fg\n",
                g_rollRef, g_pitchRef, g_gravityNorm);

  // 建热点。SSID 追加 MAC 后两字节，防止多台设备重名
  uint64_t mac = ESP.getEfuseMac();
  char ssid[32];
  snprintf(ssid, sizeof(ssid), "%s-%02X%02X", AP_SSID_PREFIX,
           (uint8_t)(mac >> 8), (uint8_t)mac);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, AP_PASSWORD, AP_CHANNEL);
  Serial.printf("[flight-ctrl] 热点已建立：%s  密码：%s\n", ssid, AP_PASSWORD);
  Serial.print("[flight-ctrl] 面板地址：http://");
  Serial.println(WiFi.softAPIP());

  g_ws.begin();
  g_ws.onEvent(onWsEvent);
  g_http.begin();
  g_http.setTimeout(1000);
}

void loop() {
  uint32_t now = millis();

  // 1) 采样 + 滤波
  if (now - g_lastSampleMs >= SAMPLE_INTERVAL_MS) {
    g_lastSampleMs = now;
    float r, p;
    readAngles(r, p);
    r = r - g_rollRef;    // 换算到相对中立位
    p = p - g_pitchRef;
    g_roll  += EMA_ALPHA * (r - g_roll);
    g_pitch += EMA_ALPHA * (p - g_pitch);
  }

  // 2) 推送数据
  if (now - g_lastSendMs >= SEND_INTERVAL_MS) {
    g_lastSendMs = now;
    g_ws.loop();
    g_wsClients = g_ws.connectedClients();
    if (g_wsClients > 0) {
      char buf[96];
      snprintf(buf, sizeof(buf),
               "{\"roll\":%.1f,\"pitch\":%.1f,\"cr\":%.2f,\"cp\":%.2f,\"ms\":%lu}",
               applyDeadzone(g_roll), applyDeadzone(g_pitch),
               mapToControl(g_roll), mapToControl(g_pitch),
               (unsigned long)now);
      g_ws.broadcastTXT(buf);
      g_msgCount++;
    }
  }

  // 3) HTTP 面板与 WebSocket 心跳
  serveHttp();
  g_ws.loop();
  blinkLed();

  delay(1);  // 让出 CPU，喂看门狗
}
