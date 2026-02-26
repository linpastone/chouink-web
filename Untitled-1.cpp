#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <ArduinoJson.h>

// ================= 設定參數 =================
const char* AP_SSID = "AirBed_System";
const char* AP_PASS = "12345678";

// 引腳定義
const int PIN_SENSOR_A = 34; // MPX5500DP 主
const int PIN_SENSOR_B = 35; // MPX5500DP 備援
const int PIN_PUMP = 26;     // 高電平觸發
const int PIN_VENT = 27;     // 高電平觸發

// 系統參數
const float MAX_PSI_DIFF = 2.0;       // 雙感測器最大允許誤差 (PSI)
const unsigned long ERROR_CHECK_MS = 2000; // 持續誤差觸發時間
const unsigned long AUTO_CHECK_INTERVAL = 15 * 60 * 1000; // 15分鐘自動檢查

// MPX5500DP 參數 (Vout = Vs * (0.009 * P + 0.04))
// P = (Vout/Vs - 0.04) / 0.009 (kPa) -> 轉 PSI
// 需根據實際電壓校準
const float VS = 3.3; // ESP32 ADC 參考電壓 (建議實測)

// ================= 全域變數 =================
Adafruit_MCP23X17 mcp;
WebServer server(80);

struct Airbag {
  int id;
  float currentPsi;
  float targetPsi;
  bool selected;
};

Airbag airbags[16];

// 系統狀態機
enum SystemState { IDLE, ADJUSTING, SCANNING, ERROR_STOP };
SystemState currentState = IDLE;
String statusMessage = "系統就緒";

unsigned long lastAutoCheckTime = 0;
unsigned long errorStartTime = 0;
bool isErrorCondition = false;

// 當前正在處理的氣囊索引 (-1 表示無)
int currentProcessingIndex = -1; 

// ================= 輔助函式 =================

// 讀取壓力 (PSI)
float readPressure(int pin) {
  int raw = analogRead(pin);
  float voltage = (raw / 4095.0) * VS; 
  // MPX5500DP Transfer Function: Vout = Vs * (0.009 * P_kpa + 0.04)
  // P_kpa = (Vout/Vs - 0.04) / 0.009
  float kpa = (voltage / VS - 0.04) / 0.009;
  if (kpa < 0) kpa = 0;
  return kpa * 0.145038; // kPa to PSI
}

// 檢查感測器健康度
bool checkSensors() {
  float p1 = readPressure(PIN_SENSOR_A);
  float p2 = readPressure(PIN_SENSOR_B);
  
  if (abs(p1 - p2) > MAX_PSI_DIFF) {
    if (errorStartTime == 0) {
      errorStartTime = millis();
    } else if (millis() - errorStartTime > ERROR_CHECK_MS) {
      currentState = ERROR_STOP;
      statusMessage = "感測器誤差過大! 系統停機";
      // 關閉所有輸出
      digitalWrite(PIN_PUMP, LOW);
      digitalWrite(PIN_VENT, LOW);
      for(int i=0; i<16; i++) mcp.digitalWrite(i, HIGH); // 關閉氣閥 (低觸發，HIGH為關)
      return false;
    }
  } else {
    errorStartTime = 0;
  }
  return true;
}

// 取得當前主線壓力 (平均值)
float getSystemPressure() {
  return (readPressure(PIN_SENSOR_A) + readPressure(PIN_SENSOR_B)) / 2.0;
}

// 控制單一氣囊閥門 (index 0-15)
void setAirbagValve(int index, bool open) {
  // 低電平觸發：LOW = 開啟閥門, HIGH = 關閉閥門
  mcp.digitalWrite(index, open ? LOW : HIGH);
}

// 關閉所有氣囊閥門
void closeAllAirbagValves() {
  for (int i = 0; i < 16; i++) {
    mcp.digitalWrite(i, HIGH);
  }
}

// ================= 網頁處理 =================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>氣壓床墊控制系統</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; background-color: #f2f2f2; }
    .grid { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; max-width: 600px; margin: 20px auto; }
    .cell { background: #fff; padding: 15px; border: 2px solid #ccc; border-radius: 8px; cursor: pointer; user-select: none; }
    .cell.selected { border-color: #007BFF; background-color: #e6f2ff; }
    .cell.active { background-color: #ffffcc; } /* 充放氣中 */
    .val { font-size: 1.2em; font-weight: bold; }
    .controls { margin: 20px; padding: 20px; background: #fff; display: inline-block; border-radius: 8px; }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; background: #28a745; color: white; border: none; border-radius: 5px; }
    button.stop { background: #dc3545; }
    #status { font-weight: bold; color: #333; margin-top: 10px; }
    .error { color: red; animation: blink 1s infinite; }
    @keyframes blink { 50% { opacity: 0; } }
  </style>
</head>
<body>
  <h2>氣壓床墊控制面板</h2>
  <div id="status">連接中...</div>
  
  <div class="grid" id="grid">
    <!-- JS 生成 16 個氣囊 -->
  </div>

  <div class="controls">
    <label>設定目標氣壓 (PSI): </label>
    <input type="number" id="targetPsi" step="0.1" value="2.0" style="width: 60px; padding: 5px;">
    <br><br>
    <button onclick="applySettings()">開始調整選取項目</button>
    <button class="stop" onclick="emergencyStop()">緊急停止</button>
  </div>

<script>
  let airbags = [];
  
  function initGrid() {
    const grid = document.getElementById('grid');
    for(let i=0; i<16; i++) {
      let div = document.createElement('div');
      div.className = 'cell';
      div.id = 'ab-' + i;
      div.onclick = () => toggleSelect(i);
      div.innerHTML = `<div>氣囊 ${i+1}</div><div class="val" id="val-${i}">--</div>`;
      grid.appendChild(div);
      airbags.push({selected: false});
    }
  }

  function toggleSelect(index) {
    airbags[index].selected = !airbags[index].selected;
    document.getElementById('ab-' + index).classList.toggle('selected', airbags[index].selected);
  }

  function updateData() {
    fetch('/status').then(r => r.json()).then(data => {
      document.getElementById('status').innerText = "狀態: " + data.msg;
      if(data.error) document.getElementById('status').className = 'error';
      else document.getElementById('status').className = '';

      for(let i=0; i<16; i++) {
        document.getElementById('val-' + i).innerText = data.psi[i].toFixed(2);
        // 如果正在調整這個氣囊，顯示黃色
        if(data.processing == i) document.getElementById('ab-' + i).classList.add('active');
        else document.getElementById('ab-' + i).classList.remove('active');
      }
    });
  }

  function applySettings() {
    let target = parseFloat(document.getElementById('targetPsi').value);
    let selectedIds = [];
    for(let i=0; i<16; i++) {
      if(airbags[i].selected) selectedIds.push(i);
    }
    
    if(selectedIds.length === 0) return alert("請先選擇氣囊");

    fetch('/set', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({target: target, ids: selectedIds})
    }).then(r => r.text()).then(msg => alert(msg));
  }

  function emergencyStop() {
    fetch('/stop').then(r => alert("已發送停止指令"));
  }

  initGrid();
  setInterval(updateData, 1000); // 每秒更新
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleStatus() {
  JsonDocument doc;
  doc["msg"] = statusMessage;
  doc["error"] = (currentState == ERROR_STOP);
  doc["processing"] = currentProcessingIndex;
  
  JsonArray psi = doc["psi"].to<JsonArray>();
  for(int i=0; i<16; i++) {
    psi.add(airbags[i].currentPsi);
  }
  
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Body not received");
    return;
  }
  
  JsonDocument doc;
  deserializeJson(doc, server.arg("plain"));
  
  float target = doc["target"];
  JsonArray ids = doc["ids"];
  
  // 重置所有選擇狀態
  for(int i=0; i<16; i++) airbags[i].selected = false;

  // 設定新的目標
  for(JsonVariant v : ids) {
    int id = v.as<int>();
    if(id >= 0 && id < 16) {
      airbags[id].selected = true;
      airbags[id].targetPsi = target;
    }
  }
  
  currentState = ADJUSTING;
  currentProcessingIndex = -1; // 重置處理指標
  statusMessage = "收到指令，開始調整...";
  server.send(200, "text/plain", "指令已接收");
}

void handleStop() {
  currentState = IDLE;
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_VENT, LOW);
  closeAllAirbagValves();
  statusMessage = "使用者手動停止";
  server.send(200, "text/plain", "Stopped");
}

// ================= 核心邏輯 =================

void setup() {
  Serial.begin(115200);

  // 初始化 GPIO
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_VENT, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_VENT, LOW);

  // 初始化 I2C & MCP23017
  Wire.begin();
  if (!mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 Error!");
    while (1);
  }
  // 設定 MCP 為輸出，並預設 HIGH (關閉繼電器)
  for (int i = 0; i < 16; i++) {
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, HIGH);
    airbags[i].id = i;
    airbags[i].currentPsi = 0.0;
    airbags[i].targetPsi = 0.0;
    airbags[i].selected = false;
  }

  // 初始化 WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  // 網頁路由
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/stop", handleStop);
  server.begin();
}

// 調整單一氣囊的邏輯 (Blocking function for simplicity in logic flow, but kept short)
// 在實際產品中，建議改為非阻塞式狀態機，這裡為了邏輯清晰使用微阻塞
void processAirbag(int index, float target) {
  statusMessage = "正在調整氣囊 " + String(index + 1);
  
  // 1. 打開該氣囊閥門
  setAirbagValve(index, true);
  delay(500); // 等待氣壓平衡到主管路

  // 2. 讀取壓力
  float currentP = getSystemPressure();
  airbags[index].currentPsi = currentP;

  // 3. 判斷充放氣
  unsigned long startProcess = millis();
  bool working = true;
  
  while(working && (millis() - startProcess < 10000)) { // 單次調整超時保護 10秒
    server.handleClient(); // 保持網頁回應
    if(!checkSensors()) { working = false; break; } // 安全檢查
    if(currentState == ERROR_STOP || currentState == IDLE) { working = false; break; }

    currentP = getSystemPressure();
    airbags[index].currentPsi = currentP; // 即時更新數據

    float diff = currentP - target;

    if (diff > 0.2) { 
      // 壓力過高 -> 洩氣
      digitalWrite(PIN_PUMP, LOW);
      digitalWrite(PIN_VENT, HIGH);
    } else if (diff < -0.2) {
      // 壓力過低 -> 充氣
      digitalWrite(PIN_VENT, LOW);
      digitalWrite(PIN_PUMP, HIGH);
    } else {
      // 達到目標 (誤差範圍內)
      working = false;
    }
    delay(100);
  }

  // 4. 關閉幫浦與洩氣閥
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_VENT, LOW);
  
  // 5. 再次讀取最終數值
  delay(500);
  airbags[index].currentPsi = getSystemPressure();
  
  // 6. 關閉氣囊閥門
  setAirbagValve(index, false);
}

// 掃描所有氣囊壓力 (不調整，只讀取)
void scanAllAirbags() {
  statusMessage = "系統自動檢測中...";
  for(int i=0; i<16; i++) {
    if(currentState == ERROR_STOP) break;
    
    setAirbagValve(i, true);
    delay(800); // 等待平衡
    airbags[i].currentPsi = getSystemPressure();
    setAirbagValve(i, false);
    delay(100);
    server.handleClient();
  }
  statusMessage = "檢測完成，系統待機";
  lastAutoCheckTime = millis();
}

void loop() {
  server.handleClient();
  
  // 安全檢查
  if (!checkSensors()) return;

  // 15分鐘自動檢查
  if (currentState == IDLE && (millis() - lastAutoCheckTime > AUTO_CHECK_INTERVAL)) {
    currentState = SCANNING;
  }

  // 狀態機處理
  switch (currentState) {
    case IDLE:
      // 什麼都不做，等待指令
      break;

    case SCANNING:
      scanAllAirbags();
      currentState = IDLE;
      break;

    case ADJUSTING:
      // 尋找下一個需要調整的氣囊
      bool jobDone = true;
      for(int i=0; i<16; i++) {
        if(airbags[i].selected) {
          currentProcessingIndex = i;
          processAirbag(i, airbags[i].targetPsi);
          airbags[i].selected = false; // 完成後取消選取
          jobDone = false;
          break; // 一次只處理一個，處理完回到 loop 讓網頁有機會更新
        }
      }
      if(jobDone) {
        currentState = IDLE;
        currentProcessingIndex = -1;
        statusMessage = "所有調整已完成";
      }
      break;

    case ERROR_STOP:
      // 已在 checkSensors 處理停機
      break;
  }
}
