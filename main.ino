// ============================================================
//  ESP32-CAM – Fast Stream + Async Face Tracking + Tunable Speed GUI
//  Board package : ESP32 by Espressif 2.0.17
//  Board         : AI Thinker ESP32-CAM
//  Partition     : Huge APP (3MB No OTA / 1MB SPIFFS)
// ============================================================

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <Preferences.h>
#include <list>

#include "human_face_detect_msr01.hpp"  // stage 1
#include "human_face_detect_mnp01.hpp"  // stage 2

// AI Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid = "WIFISSID";
const char* password = "PASSWORDWIFI";

// Motor Pins
const int LEFT_IN1  = 12;
const int LEFT_IN2  = 13;
const int RIGHT_IN1 = 14;
const int RIGHT_IN2 = 15;

// ---------------- Tunable settings (changed from the web page) ----------------
static volatile int g_turnMs   = 120;  // motor pulse when turning
static volatile int g_fwdMs    = 120;  // motor pulse when moving forward
static volatile int g_cooldown = 150;  // pause between corrections
static volatile int g_margin   = 35;   // center dead zone (px)
static volatile int g_targetH  = 95;   // stop when face height >= this (px)
static volatile int g_jpegQ    = 50;   // video JPEG quality

Preferences prefs;

void loadSettings() {
  prefs.begin("tracker", true);
  g_turnMs   = prefs.getInt("turnMs", g_turnMs);
  g_fwdMs    = prefs.getInt("fwdMs", g_fwdMs);
  g_cooldown = prefs.getInt("cooldown", g_cooldown);
  g_margin   = prefs.getInt("margin", g_margin);
  g_targetH  = prefs.getInt("targetH", g_targetH);
  g_jpegQ    = prefs.getInt("jpegQ", g_jpegQ);
  prefs.end();
}

void saveSettings() {
  prefs.begin("tracker", false);
  prefs.putInt("turnMs", g_turnMs);
  prefs.putInt("fwdMs", g_fwdMs);
  prefs.putInt("cooldown", g_cooldown);
  prefs.putInt("margin", g_margin);
  prefs.putInt("targetH", g_targetH);
  prefs.putInt("jpegQ", g_jpegQ);
  prefs.end();
}

void resetSettings() {
  g_turnMs = 120; g_fwdMs = 120; g_cooldown = 150;
  g_margin = 35; g_targetH = 95; g_jpegQ = 50;
}

bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM;
  cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM;
  cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM;
  cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM;
  cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;

  cfg.pixel_format = PIXFORMAT_RGB565;
  cfg.frame_size = FRAMESIZE_QVGA;       // 320x240
  cfg.grab_mode = CAMERA_GRAB_LATEST;
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.fb_count = 2;

  if (!psramFound()) {
    cfg.fb_location = CAMERA_FB_IN_DRAM;
    cfg.fb_count = 1;
    cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

// ---------------- Motors (unchanged) ----------------
void leftMotor(bool forward_dir) {
  if (forward_dir) { digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, HIGH); }
  else { digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, HIGH); }
}

void rightMotor(bool forward_dir) {
  if (forward_dir) { digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, HIGH); }
  else { digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, HIGH); }
}

void stopMotors() {
  digitalWrite(LEFT_IN1, LOW); digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'W': leftMotor(true); rightMotor(true); break;
    case 'S': leftMotor(false); rightMotor(false); break;
    case 'A': stopMotors(); leftMotor(true); rightMotor(false); break;
    case 'D': stopMotors(); rightMotor(true); leftMotor(false); break;
    case 'X':
    default: stopMotors(); break;
  }
}

// ---------------- State ----------------
static volatile bool g_detectEnabled = true;
static volatile bool g_manualMode = false;
static volatile int  g_faceCount = 0;
static volatile unsigned long g_lastManualActivity = 0;
static volatile int  g_box[4] = {0, 0, 0, 0};

static volatile bool g_wantFrame = false;
static volatile bool g_frameReady = false;
static uint16_t* s_detBuf = nullptr;
static size_t s_detBufSize = 0;

static HumanFaceDetectMSR01* s_det1 = nullptr;
static HumanFaceDetectMNP01* s_det2 = nullptr;

void initDetector() {
  s_det1 = new HumanFaceDetectMSR01(0.3F, 0.3F, 10, 0.3F);
  s_det2 = new HumanFaceDetectMNP01(0.4F, 0.3F, 10);
  s_detBufSize = 320 * 240 * 2;
  s_detBuf = (uint16_t*)heap_caps_malloc(s_detBufSize, MALLOC_CAP_SPIRAM);
  if (!s_detBuf) s_detBuf = (uint16_t*)malloc(s_detBufSize);
}

static void drawBox(camera_fb_t* fb, int x1, int y1, int x2, int y2, uint16_t colour) {
  int w = fb->width, h = fb->height;
  x1 = constrain(x1, 0, w - 2); x2 = constrain(x2, 1, w - 1);
  y1 = constrain(y1, 0, h - 2); y2 = constrain(y2, 1, h - 1);
  uint16_t* buf = (uint16_t*)fb->buf;
  for (int x = x1; x <= x2; x++) {
    buf[y1 * w + x] = colour; buf[(y1 + 1) * w + x] = colour;
    buf[y2 * w + x] = colour; buf[(y2 - 1) * w + x] = colour;
  }
  for (int y = y1; y <= y2; y++) {
    buf[y * w + x1] = colour; buf[y * w + x1 + 1] = colour;
    buf[y * w + x2] = colour; buf[y * w + x2 - 1] = colour;
  }
}

// ---------------- Detection + tracking task (core 0) ----------------
void detectTask(void*) {
  unsigned long lastAction = 0;

  for (;;) {
    unsigned long now = millis();

    if (g_manualMode) {
      if (now - g_lastManualActivity > 1000) {
        g_manualMode = false;
        stopMotors();
      }
      g_faceCount = 0;
      g_box[0] = g_box[1] = g_box[2] = g_box[3] = 0;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (!g_detectEnabled || !s_detBuf) {
      g_faceCount = 0;
      g_box[0] = g_box[1] = g_box[2] = g_box[3] = 0;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (now - lastAction < (unsigned long)g_cooldown) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    g_frameReady = false;
    g_wantFrame = true;
    unsigned long t0 = millis();
    while (!g_frameReady && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(5));
    g_wantFrame = false;
    if (!g_frameReady) continue;

    std::list<dl::detect::result_t> candidates =
      s_det1->infer(s_detBuf, { 240, 320, 3 });
    std::list<dl::detect::result_t> results =
      s_det2->infer(s_detBuf, { 240, 320, 3 }, candidates);

    g_faceCount = results.size();
    char cmd = 'X';
    int pulse = 0;

    if (!results.empty()) {
      int max_area = 0, bx1 = 0, by1 = 0, bx2 = 0, by2 = 0;
      for (auto& r : results) {
        int x1 = r.box[0], y1 = r.box[1], x2 = r.box[2], y2 = r.box[3];
        int area = (x2 - x1) * (y2 - y1);
        if (area > max_area) { max_area = area; bx1 = x1; by1 = y1; bx2 = x2; by2 = y2; }
      }
      g_box[0] = bx1; g_box[1] = by1; g_box[2] = bx2; g_box[3] = by2;

      int face_cx = (bx1 + bx2) / 2;
      int face_h = by2 - by1;
      int center_x = 160;
      int margin = g_margin;

      if (face_cx < center_x - margin)      { cmd = 'D'; pulse = g_turnMs; }  // face left -> turn right
      else if (face_cx > center_x + margin) { cmd = 'A'; pulse = g_turnMs; }  // face right -> turn left
      else if (face_h < g_targetH)          { cmd = 'W'; pulse = g_fwdMs; }   // move forward
    } else {
      g_box[0] = g_box[1] = g_box[2] = g_box[3] = 0;
    }

    if (cmd != 'X' && !g_manualMode) {
      executeCommand(cmd);
      vTaskDelay(pdMS_TO_TICKS(pulse));
      if (!g_manualMode) stopMotors();
    } else if (!g_manualMode) {
      stopMotors();
    }
    lastAction = millis();
  }
}

// ---------------- Servers ----------------
httpd_handle_t g_stream = NULL;
httpd_handle_t g_cmd = NULL;

#define BOUNDARY "gc0pJq0M2Yt08j34c0p"
static const char STREAM_CT[] = "multipart/x-mixed-replace;boundary=" BOUNDARY;

esp_err_t streamHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, STREAM_CT);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partHdr[128];

  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(1); continue; }

    if (g_wantFrame && s_detBuf && fb->len <= s_detBufSize) {
      memcpy(s_detBuf, fb->buf, fb->len);
      g_wantFrame = false;
      g_frameReady = true;
    }

    if (g_faceCount > 0 && g_box[2] > g_box[0]) {
      drawBox(fb, g_box[0], g_box[1], g_box[2], g_box[3], 0x07E0);
    }

    uint8_t* jpgBuf = nullptr;
    size_t jpgLen = 0;
    bool ok = frame2jpg(fb, g_jpegQ, &jpgBuf, &jpgLen);
    esp_camera_fb_return(fb);

    if (!ok || !jpgBuf) continue;

    size_t hLen = snprintf(partHdr, sizeof(partHdr),
      "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)jpgLen);

    esp_err_t res = httpd_resp_send_chunk(req, partHdr, hLen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)jpgBuf, jpgLen);
    free(jpgBuf);
    if (res != ESP_OK) break;
  }
  return ESP_OK;
}

esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  uint8_t* jpgBuf = nullptr; size_t jpgLen = 0;
  frame2jpg(fb, 90, &jpgBuf, &jpgLen);
  esp_camera_fb_return(fb);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, (const char*)jpgBuf, jpgLen);
  free(jpgBuf);
  return ESP_OK;
}

static bool getQ(httpd_req_t* r, const char* k, char* v, size_t n) {
  char buf[128] = {};
  if (httpd_req_get_url_query_str(r, buf, sizeof(buf)) != ESP_OK) return false;
  return httpd_query_key_value(buf, k, v, n) == ESP_OK;
}

esp_err_t controlHandler(httpd_req_t* req) {
  char var[32] = {}, val[32] = {};
  getQ(req, "var", var, sizeof(var));
  getQ(req, "val", val, sizeof(val));
  int n = atoi(val);

  if (!strcmp(var, "detect")) {
    g_detectEnabled = (n != 0);
    if (!g_detectEnabled) stopMotors();
  }
  else if (!strcmp(var, "mode")) {
    g_manualMode = !strcmp(val, "manual");
    g_lastManualActivity = millis();
    if (!g_manualMode) stopMotors();
  }
  else if (!strcmp(var, "car")) {
    g_manualMode = true;
    g_lastManualActivity = millis();
    executeCommand(val[0]);
  }
  else if (!strcmp(var, "turnMs"))   g_turnMs   = constrain(n, 0, 1000);
  else if (!strcmp(var, "fwdMs"))    g_fwdMs    = constrain(n, 0, 1000);
  else if (!strcmp(var, "cooldown")) g_cooldown = constrain(n, 0, 2000);
  else if (!strcmp(var, "margin"))   g_margin   = constrain(n, 5, 150);
  else if (!strcmp(var, "targetH"))  g_targetH  = constrain(n, 30, 239);
  else if (!strcmp(var, "jpegQ"))    g_jpegQ    = constrain(n, 10, 90);
  else if (!strcmp(var, "save"))     saveSettings();
  else if (!strcmp(var, "reset"))    resetSettings();

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, "OK");
  return ESP_OK;
}

esp_err_t statusHandler(httpd_req_t* req) {
  char json[320];
  snprintf(json, sizeof(json),
    "{\"detect\":%d,\"manual\":%d,\"faces\":%d,\"turnMs\":%d,\"fwdMs\":%d,"
    "\"cooldown\":%d,\"margin\":%d,\"targetH\":%d,\"jpegQ\":%d}",
    g_detectEnabled ? 1 : 0, g_manualMode ? 1 : 0, (int)g_faceCount,
    (int)g_turnMs, (int)g_fwdMs, (int)g_cooldown, (int)g_margin, (int)g_targetH, (int)g_jpegQ);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Tracker</title>
<style>
  *{box-sizing:border-box}
  body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:10px;text-align:center}
  h2{margin:4px 0;font-size:1.1em}
  #stream{max-width:100%;border:3px solid #333;border-radius:8px;margin:5px 0}
  .panel{background:#1e1e1e;border-radius:8px;padding:8px;margin:6px auto;max-width:480px}
  button{padding:8px 14px;border:none;border-radius:5px;cursor:pointer;font-size:.9em;font-weight:bold;user-select:none;color:#fff;background:#333}
  .btn-green{background:#1a7a1a} .btn-blue{background:#1a3a7a} .btn-red{background:#7a1a1a}
  .badge{display:inline-block;padding:3px 10px;border-radius:12px;font-size:.8em;background:#333;margin:2px}
  .row{text-align:left;margin:8px 4px;font-size:.85em}
  .row input{width:100%}
  .row b{color:#6cf}
  .controller{display:grid;grid-template-columns:80px 80px 80px;grid-template-rows:70px 70px;gap:8px;justify-content:center;margin-top:10px}
  .btn-forward{grid-column:2;grid-row:1} .btn-left{grid-column:1;grid-row:2}
  .btn-stop{grid-column:2;grid-row:2;background:#d9534f} .btn-right{grid-column:3;grid-row:2}
  .ctrl-btn{background:#444;color:#fff;font-size:18px;border-radius:10px}
  .ctrl-btn:active{background:#007BFF}
</style></head><body>
<h2>ESP32-CAM Tracker</h2>
<img id="stream" src="">
<div class="panel">
  <div><span class="badge" id="faceBadge" style="background:#1a5a1a">Faces: 0</span><span class="badge" id="modeBadge" style="background:#1a3a7a">Mode: Auto-Tracking</span></div>
  <div style="margin-top:6px"><button class="btn-green" id="modeBtn" onclick="toggleMode()">Switch to Manual (WASD)</button></div>
</div>

<div class="panel">
  <div style="font-size:0.9em;color:#aaa">Speed Settings</div>
  <div style="margin:6px 0">
    <button onclick="preset(40,40,350)">Slow</button>
    <button onclick="preset(120,120,150)">Medium</button>
    <button onclick="preset(250,200,0)">Fast</button>
  </div>
  <div id="sliders"></div>
  <div style="margin-top:6px">
    <button class="btn-blue" onclick="ctrl('save',1);this.innerText='Saved!';var b=this;setTimeout(function(){b.innerText='Save to device'},1200)">Save to device</button>
    <button class="btn-red" onclick="ctrl('reset',1);setTimeout(loadSettings,200)">Reset defaults</button>
  </div>
</div>

<div class="panel">
  <div style="font-size:0.85em;color:#aaa;margin-bottom:4px">Manual Drive (WASD / Touch)</div>
  <div class="controller">
    <button class="ctrl-btn btn-forward" onmousedown="sendCmd('W')" onmouseup="sendCmd('X')" ontouchstart="sendCmd('W')" ontouchend="sendCmd('X')">W</button>
    <button class="ctrl-btn btn-left" onmousedown="sendCmd('A')" onmouseup="sendCmd('X')" ontouchstart="sendCmd('A')" ontouchend="sendCmd('X')">A</button>
    <button class="ctrl-btn btn-stop" onclick="sendCmd('X')">STOP</button>
    <button class="ctrl-btn btn-right" onmousedown="sendCmd('D')" onmouseup="sendCmd('X')" ontouchstart="sendCmd('D')" ontouchend="sendCmd('X')">D</button>
  </div>
  <div style="margin-top:8px"><button class="ctrl-btn" onmousedown="sendCmd('S')" onmouseup="sendCmd('X')" ontouchstart="sendCmd('S')" ontouchend="sendCmd('X')" style="width:248px;height:45px">BACK (S)</button></div>
</div>
<script>
var host = window.location.hostname, manualMode = false, held = 'X';
document.getElementById('stream').src = 'http://'+host+':81/stream';
function ctrl(v,val){ return fetch('http://'+host+'/control?var='+v+'&val='+val); }

var S = [
  ['turnMs','Turn pulse (ms) - rotation speed',10,600],
  ['fwdMs','Forward pulse (ms)',10,600],
  ['cooldown','Pause between corrections (ms)',0,1000],
  ['margin','Center dead zone (px)',5,100],
  ['targetH','Stop distance: face height (px)',40,200],
  ['jpegQ','Video quality (lower = faster)',10,90]
];
var html = '';
S.forEach(function(s){
  html += '<div class="row"><label>'+s[1]+': <b id="v_'+s[0]+'">-</b></label>' +
          '<input type="range" id="r_'+s[0]+'" min="'+s[2]+'" max="'+s[3]+'"></div>';
});
document.getElementById('sliders').innerHTML = html;
var timers = {};
S.forEach(function(s){
  var el = document.getElementById('r_'+s[0]);
  el.addEventListener('input', function(){
    document.getElementById('v_'+s[0]).innerText = el.value;
    clearTimeout(timers[s[0]]);
    timers[s[0]] = setTimeout(function(){ ctrl(s[0], el.value); }, 120);
  });
});
function setSlider(k,v){
  var el = document.getElementById('r_'+k);
  if(el){ el.value = v; document.getElementById('v_'+k).innerText = v; }
}
function preset(t,f,c){
  setSlider('turnMs',t); setSlider('fwdMs',f); setSlider('cooldown',c);
  ctrl('turnMs',t); ctrl('fwdMs',f); ctrl('cooldown',c);
}
function loadSettings(){
  fetch('http://'+host+'/status').then(r=>r.json()).then(function(s){
    S.forEach(function(x){ setSlider(x[0], s[x[0]]); });
  });
}
loadSettings();

function setModeUI(){
  document.getElementById('modeBadge').innerText = manualMode ? 'Mode: Manual (WASD)' : 'Mode: Auto-Tracking';
  document.getElementById('modeBadge').style.background = manualMode ? '#7a1a1a' : '#1a3a7a';
  document.getElementById('modeBtn').innerText = manualMode ? 'Switch to Auto-Tracking' : 'Switch to Manual (WASD)';
}
function sendCmd(cmd){
  held = cmd;
  if(!manualMode && cmd !== 'X'){ manualMode = true; setModeUI(); }
  ctrl('car', cmd);
}
function toggleMode(){ manualMode = !manualMode; held = 'X'; ctrl('mode', manualMode ? 'manual' : 'auto'); setModeUI(); }

// keep-alive so holding a key longer than 1s doesn't time out
setInterval(function(){ if(held !== 'X') ctrl('car', held); }, 300);

const activeKeys = {};
window.addEventListener('keydown', (e) => { let k = e.key.toUpperCase(); if(['W','A','S','D'].includes(k) && !activeKeys[k]) { activeKeys[k] = true; sendCmd(k); } });
window.addEventListener('keyup', (e) => { let k = e.key.toUpperCase(); if(['W','A','S','D'].includes(k)) { activeKeys[k] = false; sendCmd('X'); } });

setInterval(function(){
  fetch('http://'+host+'/status').then(r=>r.json()).then(function(s){
    document.getElementById('faceBadge').innerText = 'Faces: ' + s.faces;
    if(held === 'X'){ manualMode = !!s.manual; setModeUI(); }
  }).catch(function(){});
}, 1000);
</script></body></html>
)rawliteral";

esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_sendstr(req, INDEX_HTML);
  return ESP_OK;
}

void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;

  if (httpd_start(&g_cmd, &config) == ESP_OK) {
    httpd_uri_t index_uri = { "/", HTTP_GET, indexHandler, NULL };
    httpd_register_uri_handler(g_cmd, &index_uri);
    httpd_uri_t control_uri = { "/control", HTTP_GET, controlHandler, NULL };
    httpd_register_uri_handler(g_cmd, &control_uri);
    httpd_uri_t status_uri = { "/status", HTTP_GET, statusHandler, NULL };
    httpd_register_uri_handler(g_cmd, &status_uri);
    httpd_uri_t capture_uri = { "/capture", HTTP_GET, captureHandler, NULL };
    httpd_register_uri_handler(g_cmd, &capture_uri);
  }

  config.server_port = 81;
  config.ctrl_port += 1;
  config.stack_size = 8192;
  config.core_id = 1;
  if (httpd_start(&g_stream, &config) == ESP_OK) {
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, streamHandler, NULL };
    httpd_register_uri_handler(g_stream, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  loadSettings();

  if (!initCamera()) { while (1) delay(1000); }
  initDetector();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.println(WiFi.localIP());

  startServers();

  xTaskCreatePinnedToCore(detectTask, "detect", 16384, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}
