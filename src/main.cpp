#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

const char* WIFI_SSID = "My_Home_2";
const char* WIFI_PASSWORD = "myhome1905";
const char* SERVER_HOST = "vps.nanasgunung.com";
const int SERVER_PORT = 3000;
const char* WS_PATH = "/ws";

// OTA Configuration
const char* OTA_HOSTNAME = "esp32-pompa-air";
const char* OTA_PASSWORD = "nasgun2026";

#define RELAY_PIN 2           // D2 (GPIO2) - Sisi KIRI ✅ WORKING
#define WATER_SENSOR_PIN 34   // D34 - Sisi KANAN (ADC only)
#define BUZZER_PIN 5          // D5 (GPIO5) - Sisi KIRI
#define WATER_THRESHOLD 3000 // Sesuaikan dengan kalibrasi sensor Anda (nilai ADC saat tandon penuh)

bool pumpState = false;
bool safetyMode = false;
unsigned long lastWaterCheck = 0;
const unsigned long WATER_CHECK_INTERVAL = 2000;
unsigned long lastSensorPrint = 0;
const unsigned long SENSOR_PRINT_INTERVAL = 500;  // Print sensor setiap 500ms
unsigned long startupTime = 0;  // Untuk tracking uptime

WebSocketsClient webSocket;

void executeCommand(String cmd);
void sendConfirmation(String cmd);
void checkWaterLevel();
void sendWaterFullAlert();
void buzzerAlert(int times);
int readSensorRealtime();
void printStatusInfo();
void setupOTA();
void setupMDNS();
void printOTAInfo();

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ WiFi connected: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi failed!");
  }
}

// Setup mDNS
void setupMDNS() {
  Serial.println("\n⚙️  Setting up mDNS...");
  
  if (MDNS.begin(OTA_HOSTNAME)) {
    Serial.printf("✅ mDNS started: %s.local\n", OTA_HOSTNAME);
    MDNS.addService("http", "tcp", 80);
    Serial.println("✅ mDNS services advertised");
  } else {
    Serial.println("❌ mDNS failed to start!");
  }
}

// Setup OTA
void setupOTA() {
  Serial.println("\n⚙️  Setting up OTA Update...");
  
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.printf("\n⬆️  OTA Update started: %s\n", type.c_str());
    digitalWrite(RELAY_PIN, HIGH);
    pumpState = false;
    Serial.println("🛡️  Pump disabled for safety during OTA");
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ OTA Update completed!");
    Serial.println("🔄 Rebooting ESP32...\n");
    buzzerAlert(2);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percent = (progress / (total / 100));
    Serial.printf("Progress: %u%%\r", percent);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n❌ OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed (wrong password)");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed (check mDNS hostname or IP)");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
    buzzerAlert(5);
  });
  
  ArduinoOTA.begin();
  Serial.printf("✅ OTA Ready!\n\n");
}

// Print OTA info
void printOTAInfo() {
  Serial.println("\n═══════════════════════════════════════════");
  Serial.println("🔄 OTA UPDATE INFO");
  Serial.println("═══════════════════════════════════════════");
  Serial.printf("  Hostname: %s.local\n", OTA_HOSTNAME);
  Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  Password: %s\n", OTA_PASSWORD);
  Serial.printf("  WiFi Signal: %d dBm\n", WiFi.RSSI());
  Serial.println("\n📋 PlatformIO Upload:");
  Serial.printf("  upload_port = %s.local\n", OTA_HOSTNAME);
  Serial.printf("  upload_flags = --auth=%s\n", OTA_PASSWORD);
  Serial.println("═══════════════════════════════════════════\n");
}

void buzzerAlert(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

int readSensorRealtime() {
  int waterLevel = analogRead(WATER_SENSOR_PIN);
  
  // Tentukan status berdasarkan threshold
  String status;
  if (waterLevel < WATER_THRESHOLD) {
    status = "🔴 PENUH";
  } else {
    status = "🟢 KOSONG";
  }
  
  // Print realtime ke Serial Monitor
  Serial.printf("📊 SENSOR: %d | Status: %s | Threshold: %d\n", 
    waterLevel, status.c_str(), WATER_THRESHOLD);
  
  return waterLevel;
}

void printStatusInfo() {
  unsigned long uptime = (millis() - startupTime) / 1000;  // Uptime dalam detik
  int waterLevel = analogRead(WATER_SENSOR_PIN);
  
  Serial.println("\n" "═══════════════════════════════════════════");
  Serial.println("📊 STATUS SYSTEM - ESP32 Water Pump Controller");
  Serial.println("═══════════════════════════════════════════");
  
  // ESP32 Info
  Serial.println("\n🔧 ESP32 INFO:");
  Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  WiFi Signal: %d dBm\n", WiFi.RSSI());
  Serial.printf("  Uptime: %lu detik (%lu menit)\n", uptime, uptime / 60);
  Serial.printf("  Heap Memory: %u bytes\n", ESP.getFreeHeap());
  
  // Pompa Info
  Serial.println("\n💧 POMPA INFO:");
  Serial.printf("  Status Pompa: %s\n", pumpState ? "⚡ HIDUP" : "🛑 MATI");
  Serial.printf("  Relay Output (D%d): %s (Level: %d)\n", 
    RELAY_PIN, digitalRead(RELAY_PIN) ? "HIGH" : "LOW", digitalRead(RELAY_PIN));
  Serial.printf("  Safety Mode: %s\n", safetyMode ? "🔒 ON (Locked)" : "🔓 OFF (Open)");
  
  // Sensor Info
  Serial.println("\n💧 SENSOR INFO:");
  Serial.printf("  Raw Value: %d / 4095\n", waterLevel);
  Serial.printf("  Threshold: %d (Trigger penuh)\n", WATER_THRESHOLD);
  if (waterLevel < WATER_THRESHOLD) {
    Serial.printf("  Status: 🔴 TANDON PENUH\n");
  } else {
    Serial.printf("  Status: 🟢 TANDON KOSONG\n");
  }
  
  Serial.println("═══════════════════════════════════════════\n");
}

void sendWaterFullAlert() {
  if (!webSocket.isConnected()) return;
  
  JsonDocument doc;
  doc["type"] = "water_full";
  doc["sensorValue"] = analogRead(WATER_SENSOR_PIN);
  
  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(payload);
  Serial.println("⚠️  Water full alert sent!");
}

void checkWaterLevel() {
  int waterLevel = analogRead(WATER_SENSOR_PIN);
  
  if (waterLevel < WATER_THRESHOLD && !safetyMode) {
    Serial.printf("\n⚠️  WATER FULL! (Level: %d)\n", waterLevel);
    
    // Auto-matikan pompa
    digitalWrite(RELAY_PIN, HIGH);
    pumpState = false;
    
    // Aktifkan safety mode
    safetyMode = true;
    
    // Buzzer alert
    buzzerAlert(3);
    
    // Kirim notifikasi ke WhatsApp
    sendWaterFullAlert();
    
    Serial.println("🔒 Safety mode: ON");
  }
}

void sendConfirmation(String cmd) {
  if (!webSocket.isConnected()) return;
  
  JsonDocument doc;
  doc["type"] = "confirm";
  doc["command"] = cmd;
  doc["status"] = "success";
  doc["ipLocal"] = WiFi.localIP().toString();
  doc["pumpState"] = pumpState;
  doc["safetyMode"] = safetyMode;
  doc["waterLevel"] = analogRead(WATER_SENSOR_PIN);
  doc["threshold"] = WATER_THRESHOLD;
  doc["relayState"] = digitalRead(RELAY_PIN);
  
  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(payload);
}

void executeCommand(String cmd) {
  Serial.printf("\n📥 Executing: %s\n", cmd.c_str());
  
  if (cmd == "pump_on") {
    // Cek water level dulu
    int waterLevel = analogRead(WATER_SENSOR_PIN);
    
    if (waterLevel < WATER_THRESHOLD) {
      Serial.printf("🚫 DITOLAK! Air masih penuh (Level: %d)\n", waterLevel);
      
      // Kirim notifikasi penolakan
      if (webSocket.isConnected()) {
        JsonDocument doc;
        doc["type"] = "command_rejected";
        doc["command"] = "pump_on";
        doc["reason"] = "Tandon masih penuh! Water level: " + String(waterLevel);
        doc["waterLevel"] = waterLevel;
        doc["threshold"] = WATER_THRESHOLD;
        
        String payload;
        serializeJson(doc, payload);
        webSocket.sendTXT(payload);
      }
      
      buzzerAlert(2); // 2x beep untuk penolakan
      return; // Batalkan perintah
    }
    
    // Reset safety mode saat user manual hidup-in
    if (safetyMode) {
      Serial.println("🔓 Safety mode: OFF (manual override)");
      safetyMode = false;
    }
    
    Serial.println("⚡ Setting RELAY_PIN = LOW");
    digitalWrite(RELAY_PIN, LOW);
    pumpState = true;
    Serial.println("⚡ POMPA HIDUP (Relay energized)\n");
  } else if (cmd == "pump_off") {
    Serial.println("⚡ Setting RELAY_PIN = HIGH");
    digitalWrite(RELAY_PIN, HIGH);
    pumpState = false;
    safetyMode = false;
    Serial.println("🛑 POMPA MATI (Relay de-energized)\n");
  } else if (cmd == "status") {
    printStatusInfo();
  } else {
    Serial.printf("⚠️  Unknown: %s\n", cmd.c_str());
    return;
  }
  
  sendConfirmation(cmd);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.println("✅ WebSocket connected");
      break;
      
    case WStype_DISCONNECTED:
      Serial.println("❌ WebSocket disconnected");
      break;
      
    case WStype_TEXT: {
      JsonDocument doc;
      if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
        String cmd = doc["command"].as<String>();
        if (cmd.length() > 0) {
          Serial.printf("📥 Command: %s\n", cmd.c_str());
          executeCommand(cmd);
        }
      }
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔═══════════════════════════════════════════╗");
  Serial.println("║   ESP32 Pompa Air Control (with OTA)    ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);
  
  // Set default relay OFF (HIGH)
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("\n🔧 Production Mode - Direct Connection");
  Serial.printf("📌 RELAY: D%d | BUZZER: D%d | WATER: D%d\n", RELAY_PIN, BUZZER_PIN, WATER_SENSOR_PIN);
  Serial.println("📌 Power: 3V3 (sisi kiri) | VIN (sisi kanan untuk water)\n");
  
  // Connect WiFi
  connectWiFi();
  
  // Setup mDNS
  setupMDNS();
  
  // Setup OTA
  setupOTA();
  
  // Print OTA info at startup
  printOTAInfo();
  
  // Connect to WebSocket
  webSocket.begin(SERVER_HOST, SERVER_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  buzzerAlert(1);
  
  startupTime = millis();  // Record waktu startup untuk tracking uptime
  
  Serial.println("✅ System Ready!\n");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  
  // Handle OTA update requests
  ArduinoOTA.handle();
  
  webSocket.loop();
  
  // Cek water level untuk safety mode
  if (millis() - lastWaterCheck > WATER_CHECK_INTERVAL) {
    checkWaterLevel();
    lastWaterCheck = millis();
  }
  
  // Print sensor realtime setiap 500ms (untuk monitoring/calibration)
  if (millis() - lastSensorPrint > SENSOR_PRINT_INTERVAL) {
    readSensorRealtime();
    lastSensorPrint = millis();
  }
  
  delay(100);
}
