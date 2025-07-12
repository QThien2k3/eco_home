
/*
 * ESP32-CAM - REAL Multi-Client Streaming (WebServer + HTTPS Proxy)
 * TRUE multi-client support up to 8 concurrent MJPEG streams
 * Author: Smart Gate System
 * Date: 2025
 * Version: 4.2 - Real Multi-Client with HTTPS Proxy
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <FirebaseESP32.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

// =============================================================================
// CONFIGURATION
// =============================================================================
#define WIFI_SSID "THIEN NHAN"
#define WIFI_PASSWORD "13022021"
#define FIREBASE_HOST "smart-home-b7a03-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyAO-6fXi3A_gLZz_uf9JKpQUOuxLfu6r1I"

// Proxy Service Configuration
#define PROXY_SERVER "esp32stream.myfreeiot.win"
#define PROXY_PORT 443
#define PROXY_PUSH_ENDPOINT "/api/push-stream"
#define PROXY_REGISTER_ENDPOINT "/api/register-device"

// Camera Model Selection
#define CAMERA_MODEL_AI_THINKER

// Camera pins for AI-THINKER ESP32-CAM
#if defined(CAMERA_MODEL_AI_THINKER)
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
#endif

// Status LEDs
#define FLASH_LED_PIN     4
#define STATUS_LED_PIN    33

// =============================================================================
// WEB SERVER & CLIENT MANAGEMENT
// =============================================================================
WebServer server(80);

// Multi-client management with individual tasks
struct StreamClient {
    WiFiClient client;
    uint32_t clientId;
    unsigned long lastFrameTime;
    bool active;
    uint32_t frameCount;
    IPAddress clientIP;
    TaskHandle_t taskHandle;
};

std::vector<StreamClient*> streamClients;
SemaphoreHandle_t clientsMutex;

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
FirebaseData firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

bool cameraInitialized = false;
bool firebaseConnected = false;
bool wifiConnected = false;
bool proxyConnected = false;
String localIP = "";
String deviceID = "";
unsigned long lastHeartbeat = 0;
unsigned long lastProxyPush = 0;
unsigned long lastProxyRegister = 0;

// Performance tracking
struct PerformanceStats {
    uint32_t totalClients = 0;
    uint32_t activeStreams = 0;
    uint32_t totalFramesSent = 0;
    uint32_t totalRequests = 0;
    uint32_t errorCount = 0;
    unsigned long lastStatsUpdate = 0;
    float avgFPS = 0.0;
} stats;

// Streaming configuration
const unsigned long FRAME_INTERVAL = 125; // 8 FPS (1000/8 = 125ms)
const unsigned long CLIENT_TIMEOUT = 30000; // 30 seconds
const size_t MAX_CONCURRENT_STREAMS = 8;

// =============================================================================
// ENHANCED CAMERA INITIALIZATION
// =============================================================================
bool initCamera() {
    Serial.println("=====================================");
    Serial.println("INITIALIZING CAMERA - REAL MULTI-CLIENT");
    Serial.println("=====================================");
    
    // Power cycle camera
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, HIGH);
    delay(100);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    delay(100);
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    
    // Optimized settings for REAL multi-client streaming
    if(psramFound()) {
        config.frame_size = FRAMESIZE_VGA;    // 640x480 - optimal for multi-client
        config.jpeg_quality = 12;             // Good balance quality/performance
        config.fb_count = 4;                  // Quad buffer for smooth multi-client
    } else {
        config.frame_size = FRAMESIZE_QVGA;   // 320x240
        config.jpeg_quality = 15;
        config.fb_count = 2;                  // Double buffer
    }
    
    Serial.printf("PSRAM found: %s\n", psramFound() ? "YES" : "NO");
    Serial.printf("Frame size: %s\n", psramFound() ? "VGA (640x480)" : "QVGA (320x240)");
    Serial.printf("JPEG quality: %d\n", config.jpeg_quality);
    Serial.printf("Frame buffers: %d\n", config.fb_count);
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera init FAILED with error 0x%x (%s)\n", err, esp_err_to_name(err));
        return false;
    }
    
    // Get sensor handle and configure
    sensor_t * s = esp_camera_sensor_get();
    if (s == nullptr) {
        Serial.println("❌ Failed to get camera sensor");
        return false;
    }
    
    // Optimized sensor settings for multi-client
    s->set_brightness(s, 0);      // -2 to 2
    s->set_contrast(s, 0);        // -2 to 2
    s->set_saturation(s, 0);      // -2 to 2
    s->set_special_effect(s, 0);  // 0 to 6 (0=No Effect)
    s->set_whitebal(s, 1);        // enable white balance
    s->set_awb_gain(s, 1);        // enable AWB gain
    s->set_wb_mode(s, 0);         // 0 to 4 (0=Auto)
    s->set_exposure_ctrl(s, 1);   // enable exposure control
    s->set_aec2(s, 0);            // disable AEC2
    s->set_gain_ctrl(s, 1);       // enable gain control
    s->set_agc_gain(s, 0);        // 0 to 30
    s->set_bpc(s, 0);             // disable black pixel correct
    s->set_wpc(s, 1);             // enable white pixel correct
    s->set_raw_gma(s, 1);         // enable gamma correct
    s->set_lenc(s, 1);            // enable lens correct
    s->set_hmirror(s, 0);         // disable horizontal mirror
    s->set_vflip(s, 0);           // disable vertical flip
    s->set_dcw(s, 1);             // enable downsize
    s->set_colorbar(s, 0);        // disable color bar
    
    // Test capture
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("❌ Test capture FAILED");
        return false;
    }
    
    Serial.printf("✅ Test capture SUCCESS - Size: %zu bytes\n", fb->len);
    esp_camera_fb_return(fb);
    
    Serial.println("✅ Camera initialized for REAL multi-client streaming");
    return true;
}

// =============================================================================
// WIFI & FIREBASE INITIALIZATION
// =============================================================================
void connectWiFi() {
    Serial.println("🌐 Connecting to WiFi...");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        localIP = WiFi.localIP().toString();
        deviceID = "ESP32CAM_" + WiFi.macAddress();
        deviceID.replace(":", "");
        
        Serial.println();
        Serial.printf("✅ WiFi connected! IP: %s\n", localIP.c_str());
        Serial.printf("📱 Device ID: %s\n", deviceID.c_str());
        Serial.printf("📶 RSSI: %d dBm\n", WiFi.RSSI());
        digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
        wifiConnected = false;
        Serial.println("\n❌ WiFi connection failed!");
    }
}

void initFirebase() {
    if (!wifiConnected) return;
    
    Serial.println("🔥 Initializing Firebase...");
    
    firebaseConfig.host = FIREBASE_HOST;
    firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
    
    Firebase.begin(&firebaseConfig, &firebaseAuth);
    Firebase.reconnectWiFi(true);
    Firebase.setReadTimeout(firebaseData, 30000);
    
    if (Firebase.setString(firebaseData, "/cam/test", "REAL_MULTICLIENT_ONLINE")) {
        firebaseConnected = true;
        Serial.println("✅ Firebase connected!");
        
        // Store device info
        Firebase.setString(firebaseData, "/cam/ip", localIP);
        Firebase.setString(firebaseData, "/cam/deviceID", deviceID);
        Firebase.setString(firebaseData, "/cam/version", "4.2_REAL_MULTICLIENT");
        Firebase.setBool(firebaseData, "/cam/status/online", true);
        Firebase.setInt(firebaseData, "/cam/maxClients", MAX_CONCURRENT_STREAMS);
        
        // Store proxy information for dashboard
        Firebase.setString(firebaseData, "/cam/proxyStreamUrl", "https://esp32stream.myfreeiot.win/stream");
        Firebase.setString(firebaseData, "/cam/proxyCaptureUrl", "https://esp32stream.myfreeiot.win/capture");
        Firebase.setString(firebaseData, "/cam/proxyStatusUrl", "https://esp32stream.myfreeiot.win/status");
        Firebase.setBool(firebaseData, "/cam/status/httpsProxyAvailable", true);
        
    } else {
        firebaseConnected = false;
        Serial.println("❌ Firebase failed: " + firebaseData.errorReason());
    }
}

// =============================================================================
// PASSIVE PROXY REGISTRATION
// =============================================================================
bool registerWithProxy() {
    if (!wifiConnected) return false;
    
    // PASSIVE PROXY MODE: Just announce our IP to Firebase
    Serial.println("📡 Enabling passive proxy mode...");
    Serial.printf("🌐 Local HTTP server: http://%s/stream\n", localIP.c_str());
    Serial.printf("🔒 HTTPS proxy access: https://esp32stream.myfreeiot.win/stream\n");
    
    if (firebaseConnected) {
        // Store our info for proxy auto-detection
        Firebase.setString(firebaseData, "/cam/localIP", localIP);
        Firebase.setString(firebaseData, "/cam/localStreamUrl", "http://" + localIP + "/stream");
        Firebase.setString(firebaseData, "/cam/localCaptureUrl", "http://" + localIP + "/capture");
        Firebase.setString(firebaseData, "/cam/localStatusUrl", "http://" + localIP + "/status");
        Firebase.setBool(firebaseData, "/cam/status/proxyReady", true);
        
        Serial.println("✅ IP registered for proxy auto-detection");
        proxyConnected = true;
        return true;
    }
    
    return false;
}

// =============================================================================
// REAL MULTI-CLIENT STREAMING FUNCTIONS
// =============================================================================
void handleStreamRequest() {
    if (!cameraInitialized) {
        server.send(503, "text/plain", "Camera not initialized");
        return;
    }
    
    stats.totalRequests++;
    
    // Check max clients
    if (streamClients.size() >= MAX_CONCURRENT_STREAMS) {
        server.send(503, "text/plain", "Max concurrent streams reached");
        Serial.printf("❌ Max clients reached (%d), rejecting new client\n", MAX_CONCURRENT_STREAMS);
        return;
    }
    
    // Get client connection
    WiFiClient client = server.client();
    
    // Create new stream client
    StreamClient* streamClient = new StreamClient();
    streamClient->client = client;
    streamClient->clientId = stats.totalClients++;
    streamClient->lastFrameTime = millis();
    streamClient->active = true;
    streamClient->frameCount = 0;
    streamClient->clientIP = client.remoteIP();
    streamClient->taskHandle = NULL;
    
    // Add to clients list
    if (xSemaphoreTake(clientsMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        streamClients.push_back(streamClient);
        stats.activeStreams = streamClients.size();
        xSemaphoreGive(clientsMutex);
    }
    
    Serial.printf("✅ New stream client #%d from %s (Total: %d)\n", 
                 streamClient->clientId, streamClient->clientIP.toString().c_str(), stats.activeStreams);
    
    // Send MJPEG headers
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Cache-Control: no-cache, no-store, must-revalidate");
    client.println("Pragma: no-cache");
    client.println("Connection: close");
    client.println();
    
    // Create dedicated task for this client
    char taskName[32];
    sprintf(taskName, "Stream_%d", streamClient->clientId);
    
    xTaskCreatePinnedToCore(
        clientStreamTask,           // Task function
        taskName,                   // Task name
        4096,                       // Stack size
        (void*)streamClient,        // Parameter (stream client)
        1,                          // Priority
        &streamClient->taskHandle,  // Task handle
        1                           // Core 1
    );
}

// Individual client streaming task - THE KEY TO REAL MULTI-CLIENT
void clientStreamTask(void* parameter) {
    StreamClient* client = (StreamClient*)parameter;
    unsigned long lastFrame = 0;
    
    Serial.printf("🎬 Started stream task for client #%d\n", client->clientId);
    
    while (client->active && client->client.connected()) {
        
        unsigned long currentTime = millis();
        
        // Control frame rate per client
        if (currentTime - lastFrame < FRAME_INTERVAL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Capture frame
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        if (fb->format != PIXFORMAT_JPEG) {
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // Send MJPEG frame
        if (client->client.connected()) {
            // Frame header
            client->client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
            
            // Frame data
            size_t written = client->client.write(fb->buf, fb->len);
            
            // Frame footer
            client->client.print("\r\n");
            
            if (written == fb->len) {
                client->frameCount++;
                client->lastFrameTime = currentTime;
                stats.totalFramesSent++;
                
                // Force flush
                client->client.flush();
            } else {
                Serial.printf("⚠️ Client #%d write failed, disconnecting\n", client->clientId);
                client->active = false;
            }
        } else {
            Serial.printf("📺 Client #%d disconnected\n", client->clientId);
            client->active = false;
        }
        
        esp_camera_fb_return(fb);
        lastFrame = currentTime;
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    // Cleanup client
    Serial.printf("🧹 Cleaning up client #%d (Frames sent: %d)\n", client->clientId, client->frameCount);
    
    client->client.stop();
    
    // Remove from clients list
    if (xSemaphoreTake(clientsMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (auto it = streamClients.begin(); it != streamClients.end(); ++it) {
            if (*it == client) {
                streamClients.erase(it);
                break;
            }
        }
        stats.activeStreams = streamClients.size();
        xSemaphoreGive(clientsMutex);
    }
    
    delete client;
    vTaskDelete(NULL); // Delete this task
}

// =============================================================================
// WEB SERVER SETUP
// =============================================================================
void setupWebServer() {
    // Main stream endpoint
    server.on("/stream", HTTP_GET, handleStreamRequest);
    
    // Single capture endpoint
    server.on("/capture", HTTP_GET, []() {
        stats.totalRequests++;
        
        if (!cameraInitialized) {
            server.send(503, "text/plain", "Camera not initialized");
            return;
        }
        
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            stats.errorCount++;
            server.send(500, "text/plain", "Capture failed");
            return;
        }
        
        server.sendHeader("Content-Disposition", "attachment; filename=capture.jpg");
        server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
        
        esp_camera_fb_return(fb);
    });
    
    // Status/Stats endpoint
    server.on("/status", HTTP_GET, []() {
        stats.totalRequests++;
        
        DynamicJsonDocument doc(1024);
        doc["deviceID"] = deviceID;
        doc["localIP"] = localIP;
        doc["version"] = "4.2-RealMultiClient";
        doc["camera"] = cameraInitialized;
        doc["wifi"] = wifiConnected;
        doc["firebase"] = firebaseConnected;
        doc["proxy"] = proxyConnected;
        doc["uptime"] = millis() / 1000;
        doc["freeHeap"] = ESP.getFreeHeap();
        doc["activeStreams"] = stats.activeStreams;
        doc["totalClients"] = stats.totalClients;
        doc["totalFrames"] = stats.totalFramesSent;
        doc["totalRequests"] = stats.totalRequests;
        doc["errorCount"] = stats.errorCount;
        doc["avgFPS"] = stats.avgFPS;
        doc["maxClients"] = MAX_CONCURRENT_STREAMS;
        
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    // Root endpoint with enhanced interface
    server.on("/", HTTP_GET, []() {
        stats.totalRequests++;
        
        String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32-CAM REAL Multi-Client v4.2</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1000px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 20px 0; }
        .stat-card { background: #f8f9fa; padding: 15px; border-radius: 8px; text-align: center; }
        .stat-value { font-size: 2em; font-weight: bold; color: #007bff; }
        .stream-container { text-align: center; margin: 30px 0; }
        .btn { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 5px; margin: 5px; cursor: pointer; }
        .btn:hover { background: #0056b3; }
        .success { color: #28a745; } .error { color: #dc3545; }
        .multiclient-notice { background: #d4edda; color: #155724; padding: 10px; border-radius: 5px; margin: 10px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>📹 ESP32-CAM REAL Multi-Client v4.2</h1>
        
        <div class="multiclient-notice">
            <strong>🚀 REAL MULTI-CLIENT:</strong> Each client gets dedicated task! Up to 8 concurrent MJPEG streams.
        </div>
        
        <div class="stats" id="stats">
            <div class="stat-card">
                <div class="stat-value" id="activeStreams">)rawliteral" + String(stats.activeStreams) + R"rawliteral(</div>
                <div>Active Streams</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="totalClients">)rawliteral" + String(stats.totalClients) + R"rawliteral(</div>
                <div>Total Clients</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="avgFPS">)rawliteral" + String(stats.avgFPS, 1) + R"rawliteral(</div>
                <div>Total FPS</div>
            </div>
            <div class="stat-card">
                <div class="stat-value" id="freeHeap">)rawliteral" + String(ESP.getFreeHeap()/1024) + R"rawliteral(</div>
                <div>Free Memory (KB)</div>
            </div>
        </div>
        
        <div class="stream-container">
            <h3>📺 TRUE Multi-Client MJPEG Test</h3>
            <p>Each stream = independent task! Max: <strong>)rawliteral" + String(MAX_CONCURRENT_STREAMS) + R"rawliteral(</strong></p>
            
            <div style="margin-top: 20px;">
                <button class="btn" onclick="window.open('/stream', '_blank')">🎬 New Stream Window</button>
                <button class="btn" onclick="testMultipleStreams()">🔀 Test 8 Streams!</button>
                <button class="btn" onclick="window.open('/capture', '_blank')">📷 Single Capture</button>
                <button class="btn" onclick="refreshStats()">📊 Refresh Stats</button>
            </div>
            
            <div style="margin-top: 20px;">
                <h4>🔒 HTTPS Proxy URLs:</h4>
                <p><a href="https://esp32stream.myfreeiot.win/stream" target="_blank">https://esp32stream.myfreeiot.win/stream</a></p>
                <p><a href="https://esp32stream.myfreeiot.win/capture" target="_blank">https://esp32stream.myfreeiot.win/capture</a></p>
            </div>
        </div>
        
        <div style="background: #343a40; color: white; padding: 15px; border-radius: 5px; margin: 20px 0;">
            <strong>🎯 REAL Multi-Client Features:</strong><br>
            • ✅ Each client = dedicated FreeRTOS task<br>
            • ✅ True concurrent streaming (not broadcast)<br>
            • ✅ Independent frame rates per client<br>
            • ✅ Automatic client cleanup & resource management<br>
            • ✅ HTTPS proxy integration<br>
            • ✅ Real-time performance monitoring<br>
            • 🚀 <strong>UP TO 8 CONCURRENT MJPEG STREAMS!</strong>
        </div>
    </div>
    
    <script>
        function testMultipleStreams() {
            for(let i = 0; i < 8; i++) {
                setTimeout(() => {
                    window.open('/stream', '_blank', 'width=400,height=300');
                }, i * 200);
            }
        }
        
        function refreshStats() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('activeStreams').textContent = data.activeStreams;
                    document.getElementById('totalClients').textContent = data.totalClients;
                    document.getElementById('avgFPS').textContent = data.avgFPS.toFixed(1);
                    document.getElementById('freeHeap').textContent = Math.round(data.freeHeap / 1024);
                });
        }
        
        // Auto refresh stats every 3 seconds
        setInterval(refreshStats, 3000);
    </script>
</body>
</html>
)rawliteral";
        
        server.send(200, "text/html", html);
    });
    
    // Handle 404
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not Found");
    });
    
    server.begin();
    Serial.println("✅ REAL Multi-Client WebServer started!");
}

// =============================================================================
// PERFORMANCE MONITORING TASK
// =============================================================================
void performanceMonitorTask(void* parameter) {
    while (true) {
        static unsigned long lastFPSCalc = 0;
        static uint32_t lastFrameCount = 0;
        unsigned long currentTime = millis();
        
        if (currentTime - lastFPSCalc > 5000) { // Every 5 seconds
            stats.avgFPS = (stats.totalFramesSent - lastFrameCount) * 1000.0 / (currentTime - lastFPSCalc);
            lastFrameCount = stats.totalFramesSent;
            lastFPSCalc = currentTime;
            
            // Debug log
            Serial.printf("📊 Performance: %d streams, %.1f total FPS, %d KB free\n", 
                         stats.activeStreams, stats.avgFPS, ESP.getFreeHeap()/1024);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// =============================================================================
// SETUP AND MAIN LOOP
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("==========================================");
    Serial.println("  ESP32-CAM REAL MULTI-CLIENT v4.2");
    Serial.println("==========================================");
    
    // Initialize LEDs
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(FLASH_LED_PIN, LOW);
    
    // Create mutex for client management
    clientsMutex = xSemaphoreCreateMutex();
    
    // Initialize camera
    if (initCamera()) {
        cameraInitialized = true;
        Serial.println("✅ Camera initialization SUCCESS");
        
        // Success blink
        for(int i = 0; i < 3; i++) {
            digitalWrite(FLASH_LED_PIN, HIGH);
            delay(200);
            digitalWrite(FLASH_LED_PIN, LOW);
            delay(200);
        }
    } else {
        cameraInitialized = false;
        Serial.println("❌ Camera initialization FAILED");
    }
    
    // Connect WiFi and Firebase
    connectWiFi();
    if (wifiConnected) {
        // --- OTA Setup ---
        ArduinoOTA.setHostname("esp32camsecurity");
        ArduinoOTA.setPassword("ota_ecohome2025");
        ArduinoOTA.onStart([](){ Serial.println("Start OTA"); });
        ArduinoOTA.onEnd([](){ Serial.println("OTA End\n"); });
        ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
            Serial.printf("OTA Progress: %u%%\r", (p/(t/100)));
        });
        ArduinoOTA.onError([](ota_error_t e){
            Serial.printf("OTA Error[%u]: ", e);
            if (e==OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (e==OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (e==OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (e==OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (e==OTA_END_ERROR) Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        Serial.println("OTA Ready");
        initFirebase();
        registerWithProxy();
    }
    
    // Setup web server
    setupWebServer();
    
    // Start performance monitoring task
    xTaskCreatePinnedToCore(
        performanceMonitorTask,    // Task function
        "PerfMonitor",             // Task name
        2048,                      // Stack size
        NULL,                      // Parameters
        1,                         // Priority
        NULL,                      // Task handle
        0                          // Core 0
    );
    
    Serial.println("==========================================");
    Serial.println("✅ REAL MULTI-CLIENT SETUP COMPLETE");
    Serial.println("==========================================");
    Serial.printf("🌐 Local Access: http://%s/\n", localIP.c_str());
    Serial.printf("🔒 HTTPS Proxy: https://esp32stream.myfreeiot.win/\n");
    Serial.printf("📺 Test 8 streams: http://%s/ (click Test 8 Streams)\n", localIP.c_str());
    Serial.printf("🎯 Each client = dedicated task!\n");
    Serial.println("==========================================");
    
    digitalWrite(STATUS_LED_PIN, cameraInitialized && wifiConnected ? HIGH : LOW);
}

void loop() {
    // Handle HTTP + OTA requests
    ArduinoOTA.handle();
    server.handleClient();
    
    // Handle WiFi reconnection
    if (WiFi.status() != WL_CONNECTED && wifiConnected) {
        Serial.println("⚠️ WiFi disconnected, reconnecting...");
        wifiConnected = false;
        firebaseConnected = false;
        proxyConnected = false;
        digitalWrite(STATUS_LED_PIN, LOW);
        
        connectWiFi();
        if (wifiConnected) {
            initFirebase();
            registerWithProxy();
            digitalWrite(STATUS_LED_PIN, HIGH);
        }
    }
    
    // Update Firebase status
    if (millis() - lastHeartbeat > 30000 && firebaseConnected) {
        Firebase.setInt(firebaseData, "/cam/lastHeartbeat", millis() / 1000);
        Firebase.setBool(firebaseData, "/cam/status/online", true);
        Firebase.setInt(firebaseData, "/cam/status/uptime", millis() / 1000);
        Firebase.setInt(firebaseData, "/cam/status/freeHeap", ESP.getFreeHeap());
        Firebase.setInt(firebaseData, "/cam/status/activeStreams", stats.activeStreams);
        Firebase.setInt(firebaseData, "/cam/status/totalClients", stats.totalClients);
        Firebase.setFloat(firebaseData, "/cam/status/avgFPS", stats.avgFPS);
        
        // Keep proxy info updated
        Firebase.setString(firebaseData, "/cam/localIP", localIP);
        Firebase.setBool(firebaseData, "/cam/status/proxyReady", true);
        
        lastHeartbeat = millis();
    }
    
    delay(10);
}
