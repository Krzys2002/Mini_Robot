#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ── AP credentials ────────────────────────────────────────────
const char* ap_ssid     = "ESP32_S3";
const char* ap_password = "PaSSword12345678";

// ── Camera pins ───────────────────────────────────────────────
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
    "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ── State flags ───────────────────────────────────────────────
volatile bool streamEnabled = true;
volatile bool cameraEnabled = true;

// ── Camera init / deinit ──────────────────────────────────────
bool initCamera() {
  esp_camera_deinit();  // safe to call even if not initialized — add this line
  delay(100);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  return true;
}

void deinitCamera() {
  esp_camera_deinit();
  Serial.println("Camera powered off");
}

// ── Stream handler ────────────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t* req) {
  if (!streamEnabled || !cameraEnabled) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "Stream is stopped", 17);
    return ESP_OK;
  }

  camera_fb_t* fb = NULL;
  esp_err_t res   = ESP_OK;
  char part_buf[64];

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (streamEnabled && cameraEnabled) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) break;

    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) break;

    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    fb = NULL;
    if (res != ESP_OK) break;
  }

  if (fb) esp_camera_fb_return(fb);
  // send empty final chunk to close stream cleanly
  httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

// ── Snapshot handler ──────────────────────────────────────────
static esp_err_t capture_handler(httpd_req_t* req) {
  if (!cameraEnabled) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_send(req, "Camera is off", 13);
    return ESP_OK;
  }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// ── Control handler — /control?cmd=xxx ───────────────────────
static esp_err_t control_handler(httpd_req_t* req) {
  char query[64] = {0};
  char cmd[32]   = {0};

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "cmd", cmd, sizeof(cmd));
  }

  const char* response = "OK";

  if (strcmp(cmd, "stream_stop") == 0) {
    streamEnabled = false;
    Serial.println("[CMD] Stream stopped");
  } else if (strcmp(cmd, "stream_start") == 0) {
    if (!cameraEnabled) {
      response = "ERROR: camera is off, turn it on first";
    } else {
      streamEnabled = true;
      Serial.println("[CMD] Stream started");
    }
  } else if (strcmp(cmd, "camera_off") == 0) {
    streamEnabled = false;
    cameraEnabled = false;
    deinitCamera();
    Serial.println("[CMD] Camera off");
  } else if (strcmp(cmd, "camera_on") == 0) {
    if (initCamera()) {
      cameraEnabled = true;
      streamEnabled = true;
      Serial.println("[CMD] Camera on");
    } else {
      response = "ERROR: camera init failed";
    }
  } else if (strcmp(cmd, "status") == 0) {
    char status[64];
    snprintf(status, sizeof(status),
             "{\"camera\":%s,\"stream\":%s}",
             cameraEnabled ? "true" : "false",
             streamEnabled ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, status, strlen(status));
  } else {
    response = "ERROR: unknown command";
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, strlen(response));
}

// ── Index page ────────────────────────────────────────────────
static esp_err_t index_handler(httpd_req_t* req) {
  const char* html =
    "<!DOCTYPE html><html><head><title>XIAO Camera</title>"
    "<style>"
    "body{margin:0;background:#111;display:flex;flex-direction:column;"
    "align-items:center;justify-content:center;min-height:100vh;"
    "font-family:sans-serif;color:#eee;gap:12px}"
    "img{max-width:95%;border:2px solid #333;border-radius:8px}"
    ".btn{padding:10px 24px;border:none;border-radius:6px;cursor:pointer;"
    "font-size:14px;font-weight:500;transition:opacity .2s}"
    ".btn:hover{opacity:.8}"
    ".btn-red{background:#c0392b;color:#fff}"
    ".btn-green{background:#27ae60;color:#fff}"
    ".btn-blue{background:#2980b9;color:#fff}"
    ".btn-gray{background:#555;color:#fff}"
    ".row{display:flex;gap:10px;flex-wrap:wrap;justify-content:center}"
    "#status{font-size:13px;color:#aaa;margin-top:4px}"
    "</style></head><body>"
    "<h2 style='margin:0'>XIAO ESP32S3 Sense</h2>"
    "<img id='stream' src='http://192.168.4.1:81/stream'/>"
    "<div class='row'>"
    "<button class='btn btn-red'    onclick='cmd(\"stream_stop\")'>Stop stream</button>"
    "<button class='btn btn-green'  onclick='cmd(\"stream_start\")'>Start stream</button>"
    "<button class='btn btn-gray'   onclick='cmd(\"camera_off\")'>Camera off</button>"
    "<button class='btn btn-blue'   onclick='cmd(\"camera_on\")'>Camera on</button>"
    "<button class='btn btn-blue'   onclick='snapshot()'>Snapshot</button>"
    "</div>"
    "<div id='status'>Ready</div>"
    "<script>"
    "function cmd(c){"
    "  fetch('/control?cmd='+c)"
    "  .then(r=>r.text()).then(t=>{"
    "    document.getElementById('status').textContent=t;"
    "    if(c==='stream_start'){"
    "      var img=document.getElementById('stream');"
    "      img.src='http://192.168.4.1:81/stream?t='+Date.now();"
    "    }"
    "    if(c==='stream_stop'||c==='camera_off'){"
    "      document.getElementById('stream').src='';"
    "    }"
    "  });"
    "}"
    "function snapshot(){"
    "  window.open('http://192.168.4.1/capture','_blank');"
    "}"
    "</script>"
    "</body></html>";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

// ── Start servers ─────────────────────────────────────────────
void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.server_port = 80;
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_uri_t uris[] = {
      { "/",        HTTP_GET, index_handler,   NULL },
      { "/capture", HTTP_GET, capture_handler, NULL },
      { "/control", HTTP_GET, control_handler, NULL },
    };
    for (auto& u : uris) httpd_register_uri_handler(camera_httpd, &u);
    Serial.println("HTTP server on port 80");
  }

  config.server_port = 81;
  config.ctrl_port   = 32769;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server on port 81");
  }
}

// ── Serial command handler ────────────────────────────────────
void handleSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "stream stop") {
    streamEnabled = false;
    Serial.println("Stream stopped");
  } else if (input == "stream start") {
    streamEnabled = true;
    Serial.println("Stream started");
  } else if (input == "camera off") {
    streamEnabled = false;
    cameraEnabled = false;
    deinitCamera();
  } else if (input == "camera on") {
    if (initCamera()) {
      cameraEnabled = true;
      streamEnabled = true;
      Serial.println("Camera on, stream started");
    }
  } else if (input == "status") {
    Serial.printf("Camera: %s | Stream: %s\n",
                  cameraEnabled ? "ON" : "OFF",
                  streamEnabled ? "ON" : "OFF");
  } else if (input == "help") {
    Serial.println("Commands: stream start | stream stop | camera on | camera off | status");
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nXIAO ESP32S3 Camera — type 'help' for commands");

  if (!initCamera()) {
    Serial.println("Camera failed — halting");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0)
  );

  Serial.printf("AP: %s / %s\n", ap_ssid, ap_password);
  Serial.printf("URL: http://%s\n", WiFi.softAPIP().toString().c_str());

  startServers();
}

void loop() {
  handleSerial();
  delay(10);
}