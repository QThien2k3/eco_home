#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <FirebaseESP32.h>

// ===================== CONFIG CLASS =====================
class Config {
public:
    // WiFi credentials
    static const char* WIFI_SSID;
    static const char* WIFI_PASSWORD;
    
    // Server endpoints
    static const char* AI_SERVER_URL;
    
    // Firebase credentials
    static const char* FIREBASE_HOST;
    static const char* FIREBASE_AUTH;
    
    // Camera pins for AI Thinker Model
    static const int PWDN_GPIO_NUM = 32;
    static const int RESET_GPIO_NUM = -1;
    static const int XCLK_GPIO_NUM = 0;
    static const int SIOD_GPIO_NUM = 26;
    static const int SIOC_GPIO_NUM = 27;
    static const int Y9_GPIO_NUM = 35;
    static const int Y8_GPIO_NUM = 34;
    static const int Y7_GPIO_NUM = 39;
    static const int Y6_GPIO_NUM = 36;
    static const int Y5_GPIO_NUM = 21;
    static const int Y4_GPIO_NUM = 19;
    static const int Y3_GPIO_NUM = 18;
    static const int Y2_GPIO_NUM = 5;
    static const int VSYNC_GPIO_NUM = 25;
    static const int HREF_GPIO_NUM = 23;
    static const int PCLK_GPIO_NUM = 22;
    static const int FLASH_LED_PIN = 4;
    
    // 2FA Constants
    static const int MAX_FACE_ATTEMPTS = 10;
    static const int FACE_CAPTURE_INTERVAL = 2000; // 2 seconds
    static const int FACE_TIMEOUT = 30000; // 30 seconds
    static const float FACE_CONFIDENCE_THRESHOLD; // 0.60
};

// Define static members
const char* Config::WIFI_SSID = "    ";
const char* Config::WIFI_PASSWORD = "     ";
const char* Config::AI_SERVER_URL = "https://pyfaceid.myfreeiot.win";
const char* Config::FIREBASE_HOST = "smart-home-b7a03-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* Config::FIREBASE_AUTH = "AIzaSyAO-6fXi3A_gLZz_uf9JKpQUOuxLfu6r1I";
const float Config::FACE_CONFIDENCE_THRESHOLD = 0.60;

// ===================== FIREBASE MANAGER CLASS =====================
class FirebaseManager {
private:
    FirebaseData firebaseData;
    FirebaseConfig firebaseConfig;
    FirebaseAuth firebaseAuth;
    
public:
    bool initialize() {
        firebaseConfig.host = Config::FIREBASE_HOST;
        firebaseConfig.signer.tokens.legacy_token = Config::FIREBASE_AUTH;
        
        Firebase.begin(&firebaseConfig, &firebaseAuth);
        Firebase.reconnectWiFi(true);
        
        if (Firebase.setString(firebaseData, "/esp32cam/status", "online")) {
            Serial.println("Firebase connected successfully!");
            return true;
        } else {
            Serial.printf("Firebase connection failed: %s\n", firebaseData.errorReason().c_str());
            return false;
        }
    }
    
    bool checkFaceRequest(String& expectedUser, String& sessionId) {
        if (Firebase.getString(firebaseData, "/face_request/user")) {
            if (firebaseData.dataType() == "string" && firebaseData.stringData().length() > 0) {
                expectedUser = firebaseData.stringData();
                
                if (Firebase.getString(firebaseData, "/face_request/session_id")) {
                    sessionId = firebaseData.stringData();
                    return true;
                }
            }
        }
        return false;
    }
    
    void clearFaceRequest() {
        Firebase.setString(firebaseData, "/face_request/user", "");
        Firebase.setString(firebaseData, "/face_request/session_id", "");
    }
    
    void updateFaceProgress(String sessionId, int attempts, float confidence, String status) {
        String path = "/face_progress/" + sessionId;
        FirebaseJson json;
        json.set("attempts", attempts);
        json.set("confidence", confidence);
        json.set("status", status);
        json.set("timestamp", millis() / 1000);
        
        Firebase.setJSON(firebaseData, path, json);
    }
    
    void reportFaceResult(String sessionId, bool success, float confidence, int totalAttempts) {
        String path = "/face_result/" + sessionId;
        FirebaseJson json;
        json.set("success", success);
        json.set("confidence", confidence);
        json.set("total_attempts", totalAttempts);
        json.set("timestamp", millis() / 1000);
        
        Firebase.setJSON(firebaseData, path, json);
    }
};

// ===================== WIFI MANAGER CLASS =====================
class WiFiManager {
private:
    const char* ssid;
    const char* password;
    
public:
    WiFiManager(const char* ssid, const char* password) 
        : ssid(ssid), password(password) {}
    
    bool connect() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        Serial.print("Connecting to WiFi");
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 60) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
            return true;
        } else {
            Serial.println("\nWiFi connection failed!");
            return false;
        }
    }
    
    String getLocalIP() {
        return WiFi.localIP().toString();
    }
    
    int getRSSI() {
        return WiFi.RSSI();
    }
    
    bool isConnected() {
        return WiFi.status() == WL_CONNECTED;
    }
};

// ===================== CAMERA MANAGER CLASS =====================
class CameraManager {
private:
    bool initialized;
    
public:
    CameraManager() : initialized(false) {}
    
    bool initialize() {
        camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = Config::Y2_GPIO_NUM;
        config.pin_d1 = Config::Y3_GPIO_NUM;
        config.pin_d2 = Config::Y4_GPIO_NUM;
        config.pin_d3 = Config::Y5_GPIO_NUM;
        config.pin_d4 = Config::Y6_GPIO_NUM;
        config.pin_d5 = Config::Y7_GPIO_NUM;
        config.pin_d6 = Config::Y8_GPIO_NUM;
        config.pin_d7 = Config::Y9_GPIO_NUM;
        config.pin_xclk = Config::XCLK_GPIO_NUM;
        config.pin_pclk = Config::PCLK_GPIO_NUM;
        config.pin_vsync = Config::VSYNC_GPIO_NUM;
        config.pin_href = Config::HREF_GPIO_NUM;
        config.pin_sscb_sda = Config::SIOD_GPIO_NUM;
        config.pin_sscb_scl = Config::SIOC_GPIO_NUM;
        config.pin_pwdn = Config::PWDN_GPIO_NUM;
        config.pin_reset = Config::RESET_GPIO_NUM;
        config.xclk_freq_hz = 20000000;
        config.pixel_format = PIXFORMAT_JPEG;
        
        if(psramFound()){
            config.frame_size = FRAMESIZE_SVGA; // 800x600
            config.jpeg_quality = 8;
            config.fb_count = 2;
        } else {
            config.frame_size = FRAMESIZE_VGA;  // 640x480
            config.jpeg_quality = 10;
            config.fb_count = 1;
            config.fb_location = CAMERA_FB_IN_DRAM;
        }
        
        esp_err_t err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x", err);
            return false;
        }
        
        optimizeForFaceRecognition();
        initialized = true;
        return true;
    }
    
    void optimizeForFaceRecognition() {
        sensor_t * s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_brightness(s, 0);
            s->set_contrast(s, 0);
            s->set_saturation(s, 0);
            s->set_special_effect(s, 0);
            s->set_whitebal(s, 1);
            s->set_awb_gain(s, 1);
            s->set_wb_mode(s, 0);
            s->set_exposure_ctrl(s, 1);
            s->set_aec2(s, 0);
            s->set_ae_level(s, 0);
            s->set_aec_value(s, 300);
            s->set_gain_ctrl(s, 1);
            s->set_agc_gain(s, 0);
            s->set_gainceiling(s, (gainceiling_t)0);
            s->set_bpc(s, 0);
            s->set_wpc(s, 1);
            s->set_raw_gma(s, 1);
            s->set_lenc(s, 1);
            s->set_hmirror(s, 0);
            s->set_vflip(s, 0);
            s->set_dcw(s, 1);
            s->set_colorbar(s, 0);
            
            Serial.println("Camera settings optimized for face recognition");
        }
    }
    
    camera_fb_t* captureFrame() {
        if (!initialized) return nullptr;
        return esp_camera_fb_get();
    }
    
    void returnFrame(camera_fb_t* fb) {
        if (fb) esp_camera_fb_return(fb);
    }
    
    bool isInitialized() {
        return initialized;
    }
};

// ===================== LED CONTROLLER CLASS =====================
class LEDController {
private:
    int ledPin;
    
public:
    LEDController(int pin) : ledPin(pin) {
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, LOW);
    }
    
    void flashOn() {
        digitalWrite(ledPin, HIGH);
    }
    
    void flashOff() {
        digitalWrite(ledPin, LOW);
    }
    
    void flash(int duration = 150) {
        flashOn();
        delay(duration);
        flashOff();
    }
    
    void blink(int times, int onTime = 200, int offTime = 200) {
        for(int i = 0; i < times; i++) {
            flashOn();
            delay(onTime);
            flashOff();
            delay(offTime);
        }
    }
    
    void pulseForFaceRecognition() {
        for(int i = 0; i < 3; i++) {
            flashOn();
            delay(100);
            flashOff();
            delay(100);
        }
    }
};

// ===================== AI SERVER CLIENT CLASS =====================
class AIServerClient {
private:
    String serverUrl;
    
public:
    AIServerClient(const char* url) : serverUrl(url) {}
    
    String sendImage(camera_fb_t* fb, const String& name, const String& action) {
        if (!fb) return "{\"error\":\"No image data\"}";
        
        HTTPClient http;
        String endpoint = serverUrl + "/" + action;
        http.begin(endpoint);
        http.setTimeout(15000); // Increased timeout for face recognition
        http.addHeader("Content-Type", "multipart/form-data; boundary=----WebKitFormBoundary7MA4YWxkTrZu0gW");
        
        String body = "------WebKitFormBoundary7MA4YWxkTrZu0gW\r\n";
        body += "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n";
        body += "Content-Type: image/jpeg\r\n\r\n";
        
        String footer = "\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW\r\n";
        footer += "Content-Disposition: form-data; name=\"name\"\r\n\r\n";
        footer += name;
        footer += "\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW--\r\n";
        
        uint8_t* full_body = (uint8_t*)malloc(body.length() + fb->len + footer.length());
        memcpy(full_body, body.c_str(), body.length());
        memcpy(full_body + body.length(), fb->buf, fb->len);
        memcpy(full_body + body.length() + fb->len, footer.c_str(), footer.length());
        
        int httpResponseCode = http.POST(full_body, body.length() + fb->len + footer.length());
        String response = http.getString();
        
        http.end();
        free(full_body);
        
        return response;
    }
    
    bool recognizeFace(camera_fb_t* fb, const String& expectedUser, float& confidence) {
        String response = sendImage(fb, expectedUser, "recognize");
        
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        
        if (error) {
            Serial.printf("Failed to parse recognition response: %s\n", error.c_str());
            return false;
        }
        
        bool success = doc["success"] | false;
        String recognizedName = doc["name"] | "";
        confidence = doc["confidence"] | 0.0;
        
        Serial.printf("Recognition result: %s, Name: %s, Confidence: %.2f\n", 
                      success ? "SUCCESS" : "FAILED", recognizedName.c_str(), confidence);
        
        if (!success) {
            return false;
        }
        
        // Check confidence threshold
        if (confidence < Config::FACE_CONFIDENCE_THRESHOLD) {
            Serial.printf("Confidence too low: %.2f < %.2f\n", confidence, Config::FACE_CONFIDENCE_THRESHOLD);
            return false;
        }
        
        // Check user match (case insensitive)
        String expectedLower = expectedUser;
        expectedLower.toLowerCase();
        recognizedName.toLowerCase();
        
        if (recognizedName == expectedLower || recognizedName.indexOf(expectedLower) >= 0) {
            Serial.println("Face recognition successful - user match!");
            return true;
        } else {
            Serial.printf("User mismatch: expected %s, got %s\n", 
                          expectedUser.c_str(), recognizedName.c_str());
            return false;
        }
    }
};

// ===================== 2FA FACE RECOGNITION MANAGER CLASS =====================
class FaceRecognitionManager {
private:
    CameraManager* cameraManager;
    LEDController* ledController;
    AIServerClient* aiClient;
    FirebaseManager* firebaseManager;
    
    bool isActive;
    String currentUser;
    String currentSessionId;
    int attempts;
    unsigned long startTime;
    unsigned long lastAttemptTime;
    float lastConfidence;
    
public:
    FaceRecognitionManager(CameraManager* cam, LEDController* led, AIServerClient* ai, FirebaseManager* fb)
        : cameraManager(cam), ledController(led), aiClient(ai), firebaseManager(fb) {
        reset();
    }
    
    void reset() {
        isActive = false;
        currentUser = "";
        currentSessionId = "";
        attempts = 0;
        startTime = 0;
        lastAttemptTime = 0;
        lastConfidence = 0.0;
    }
    
    void startRecognition(const String& expectedUser, const String& sessionId) {
        Serial.printf("🔍 Starting 2FA Face Recognition for user: %s (Session: %s)\n", 
                      expectedUser.c_str(), sessionId.c_str());
        
        isActive = true;
        currentUser = expectedUser;
        currentSessionId = sessionId;
        attempts = 0;
        startTime = millis();
        lastAttemptTime = 0;
        lastConfidence = 0.0;
        
        // Signal start with LED
        ledController->pulseForFaceRecognition();
        
        // Update Firebase with start status
        firebaseManager->updateFaceProgress(sessionId, 0, 0.0, "started");
    }
    
    void stopRecognition() {
        if (isActive) {
            Serial.println("🔍 Stopping face recognition");
            firebaseManager->updateFaceProgress(currentSessionId, attempts, lastConfidence, "stopped");
            reset();
        }
    }
    
    void loop() {
        if (!isActive) return;
        
        // Check timeout
        if (millis() - startTime > Config::FACE_TIMEOUT) {
            Serial.println("🔍 Face recognition timeout!");
            firebaseManager->reportFaceResult(currentSessionId, false, lastConfidence, attempts);
            firebaseManager->updateFaceProgress(currentSessionId, attempts, lastConfidence, "timeout");
            ledController->blink(3, 500, 200); // Error blink
            reset();
            return;
        }
        
        // Check max attempts
        if (attempts >= Config::MAX_FACE_ATTEMPTS) {
            Serial.println("🔍 Max face attempts reached!");
            firebaseManager->reportFaceResult(currentSessionId, false, lastConfidence, attempts);
            firebaseManager->updateFaceProgress(currentSessionId, attempts, lastConfidence, "max_attempts");
            ledController->blink(3, 500, 200); // Error blink
            reset();
            return;
        }
        
        // Perform face recognition attempt
        if (millis() - lastAttemptTime > Config::FACE_CAPTURE_INTERVAL) {
            performFaceRecognition();
        }
    }
    
    bool getStatus() {
        return isActive;
    }
    
    String getStatusInfo() {
        if (!isActive) return "Inactive";
        
        unsigned long elapsed = millis() - startTime;
        unsigned long remaining = Config::FACE_TIMEOUT - elapsed;
        
        return String("User: ") + currentUser + 
               ", Attempts: " + String(attempts) + "/" + String(Config::MAX_FACE_ATTEMPTS) +
               ", Remaining: " + String(remaining / 1000) + "s" +
               ", Last Confidence: " + String(lastConfidence * 100, 1) + "%";
    }

private:
    void performFaceRecognition() {
        attempts++;
        lastAttemptTime = millis();
        
        Serial.printf("🔍 Face attempt %d/%d for user: %s\n", attempts, Config::MAX_FACE_ATTEMPTS, currentUser.c_str());
        
        // Flash LED during capture
        ledController->flashOn();
        
        // Capture frame
        camera_fb_t* fb = cameraManager->captureFrame();
        if (!fb) {
            Serial.println("🔍 Failed to capture frame");
            ledController->flashOff();
            firebaseManager->updateFaceProgress(currentSessionId, attempts, 0.0, "capture_failed");
            return;
        }
        
        // Recognize face
        float confidence = 0.0;
        bool success = aiClient->recognizeFace(fb, currentUser, confidence);
        lastConfidence = confidence;
        
        // Return frame
        cameraManager->returnFrame(fb);
        ledController->flashOff();
        
        // Update progress
        String status = success ? "success" : "failed";
        firebaseManager->updateFaceProgress(currentSessionId, attempts, confidence, status);
        
        Serial.printf("🔍 Recognition result: %s, Confidence: %.1f%% (need >= %.0f%%)\n", 
                      success ? "SUCCESS" : "FAILED", confidence * 100, Config::FACE_CONFIDENCE_THRESHOLD * 100);
        
        if (success) {
            Serial.println("🔍 Face recognition SUCCESSFUL! Reporting to Firebase...");
            firebaseManager->reportFaceResult(currentSessionId, true, confidence, attempts);
            ledController->blink(2, 200, 100); // Success blink
            reset();
        } else {
            Serial.printf("🔍 Face recognition failed (%.1f%% < %.0f%%). Continuing...\n", 
                          confidence * 100, Config::FACE_CONFIDENCE_THRESHOLD * 100);
            // Continue with next attempt
        }
    }
};

// ===================== WEB SERVER MANAGER CLASS =====================
class WebServerManager {
private:
    httpd_handle_t server;
    CameraManager* cameraManager;
    LEDController* ledController;
    AIServerClient* aiClient;
    WiFiManager* wifiManager;
    FaceRecognitionManager* faceManager;
    
    static esp_err_t setCORSHeaders(httpd_req_t *req) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept, Authorization, Cache-Control, Pragma");
        httpd_resp_set_hdr(req, "Access-Control-Expose-Headers", "Content-Length, Content-Range");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Credentials", "false");
        httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
        httpd_resp_set_hdr(req, "Pragma", "no-cache");
        httpd_resp_set_hdr(req, "Expires", "0");
        return ESP_OK;
    }
    
public:
    WebServerManager(CameraManager* cam, LEDController* led, AIServerClient* ai, WiFiManager* wifi, FaceRecognitionManager* face) 
        : server(nullptr), cameraManager(cam), ledController(led), aiClient(ai), wifiManager(wifi), faceManager(face) {}
    
    bool start(int port = 80) {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.server_port = port;
        
        if (httpd_start(&server, &config) == ESP_OK) {
            registerHandlers();
            return true;
        }
        return false;
    }
    
    void stop() {
        if (server) {
            httpd_stop(server);
            server = nullptr;
        }
    }
    
private:
    void registerHandlers() {
        // Stream endpoint
        httpd_uri_t stream_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = streamHandler,
            .user_ctx = this
        };
        
        // Capture endpoint
        httpd_uri_t capture_uri = {
            .uri = "/capture",
            .method = HTTP_GET,
            .handler = captureHandler,
            .user_ctx = this
        };
        
        // Send to AI endpoint
        httpd_uri_t send_to_ai_uri = {
            .uri = "/send_to_ai",
            .method = HTTP_GET,
            .handler = sendToAIHandler,
            .user_ctx = this
        };
        
        // Status endpoint
        httpd_uri_t status_uri = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = statusHandler,
            .user_ctx = this
        };
        
        // 2FA Face Recognition status endpoint
        httpd_uri_t face_status_uri = {
            .uri = "/face_status",
            .method = HTTP_GET,
            .handler = faceStatusHandler,
            .user_ctx = this
        };
        
        // OPTIONS endpoint for CORS
        httpd_uri_t options_uri = {
            .uri = "/*",
            .method = HTTP_OPTIONS,
            .handler = optionsHandler,
            .user_ctx = this
        };
        
        httpd_register_uri_handler(server, &stream_uri);
        httpd_register_uri_handler(server, &capture_uri);
        httpd_register_uri_handler(server, &send_to_ai_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &face_status_uri);
        httpd_register_uri_handler(server, &options_uri);
    }
    
    static esp_err_t streamHandler(httpd_req_t *req) {
        WebServerManager* self = (WebServerManager*)req->user_ctx;
        return self->handleStream(req);
    }
    
    static esp_err_t captureHandler(httpd_req_t *req) {
        WebServerManager* self = (WebServerManager*)req->user_ctx;
        return self->handleCapture(req);
    }
    
    static esp_err_t sendToAIHandler(httpd_req_t *req) {
        WebServerManager* self = (WebServerManager*)req->user_ctx;
        return self->handleSendToAI(req);
    }
    
    static esp_err_t statusHandler(httpd_req_t *req) {
        WebServerManager* self = (WebServerManager*)req->user_ctx;
        return self->handleStatus(req);
    }
    
    static esp_err_t faceStatusHandler(httpd_req_t *req) {
        WebServerManager* self = (WebServerManager*)req->user_ctx;
        return self->handleFaceStatus(req);
    }
    
    static esp_err_t optionsHandler(httpd_req_t *req) {
        setCORSHeaders(req);
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }
    
    esp_err_t handleStream(httpd_req_t *req) {
        camera_fb_t * fb = nullptr;
        esp_err_t res = ESP_OK;
        size_t _jpg_buf_len = 0;
        uint8_t * _jpg_buf = nullptr;
        char * part_buf[64];

        setCORSHeaders(req);
        
        res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=123456789000000000000987654321");
        if(res != ESP_OK) return res;

        while(true) {
            fb = cameraManager->captureFrame();
            if (!fb) {
                Serial.println("Camera capture failed");
                res = ESP_FAIL;
            } else {
                if(fb->width > 400) {
                    if(fb->format != PIXFORMAT_JPEG) {
                        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                        cameraManager->returnFrame(fb);
                        fb = nullptr;
                        if(!jpeg_converted) {
                            Serial.println("JPEG compression failed");
                            res = ESP_FAIL;
                        }
                    } else {
                        _jpg_buf_len = fb->len;
                        _jpg_buf = fb->buf;
                    }
                }
            }
            
            if(res == ESP_OK) {
                size_t hlen = snprintf((char *)part_buf, 64, "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
                res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
            }
            
            if(res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            }
            
            if(res == ESP_OK) {
                res = httpd_resp_send_chunk(req, "\r\n--123456789000000000000987654321\r\n", 37);
            }
            
            if(fb) {
                cameraManager->returnFrame(fb);
                fb = nullptr;
                _jpg_buf = nullptr;
            } else if(_jpg_buf) {
                free(_jpg_buf);
                _jpg_buf = nullptr;
            }
            
            if(res != ESP_OK) break;
        }
        return res;
    }
    
    esp_err_t handleCapture(httpd_req_t *req) {
        setCORSHeaders(req);
        
        ledController->flash();
        
        camera_fb_t* fb = cameraManager->captureFrame();
        if (!fb) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
        
        esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        cameraManager->returnFrame(fb);
        
        return res;
    }
    
    esp_err_t handleSendToAI(httpd_req_t *req) {
        setCORSHeaders(req);
        
        // Get query parameters
        char name[100] = {0};
        char action[20] = {0};
        
        size_t buf_len = httpd_req_get_url_query_len(req) + 1;
        if (buf_len > 1) {
            char* buf = (char*)malloc(buf_len);
            if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
                httpd_query_key_value(buf, "name", name, sizeof(name));
                httpd_query_key_value(buf, "action", action, sizeof(action));
            }
            free(buf);
        }
        
        ledController->flash();
        
        camera_fb_t* fb = cameraManager->captureFrame();
        if (!fb) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        
        String response = aiClient->sendImage(fb, String(name), String(action));
        cameraManager->returnFrame(fb);
        
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, response.c_str());
    }
    
    esp_err_t handleStatus(httpd_req_t *req) {
        setCORSHeaders(req);
        
        static char json_response[1024];
        snprintf(json_response, sizeof(json_response),
            "{\"status\":\"online\",\"heap\":%d,\"rssi\":%d,\"localip\":\"%s\",\"face_recognition_active\":%s}",
            ESP.getFreeHeap(), wifiManager->getRSSI(), wifiManager->getLocalIP().c_str(),
            faceManager->getStatus() ? "true" : "false");
        
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, json_response);
    }
    
    esp_err_t handleFaceStatus(httpd_req_t *req) {
        setCORSHeaders(req);
        
        String statusInfo = faceManager->getStatusInfo();
        String json_response = "{\"active\":" + String(faceManager->getStatus() ? "true" : "false") + 
                              ",\"info\":\"" + statusInfo + "\"}";
        
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, json_response.c_str());
    }
};

// ===================== OTA MANAGER CLASS =====================
class OTAManager {
public:
    bool initialize(const char* hostname, const char* password) {
        ArduinoOTA.setHostname(hostname);
        ArduinoOTA.setPassword(password);
        
        ArduinoOTA.onStart([]() { 
            Serial.println("OTA Start"); 
        });
        
        ArduinoOTA.onEnd([]() { 
            Serial.println("OTA End\n"); 
        });
        
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA Progress: %u%%\r", (progress/(total/100)));
        });
        
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA Error[%u]: ", error);
            if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)     Serial.println("End Failed");
        });
        
        ArduinoOTA.begin();
        Serial.println("OTA Ready");
        return true;
    }
    
    void handle() {
        ArduinoOTA.handle();
    }
};

// ===================== MAIN SYSTEM CLASS =====================
class ESP32CameraSystem {
private:
    WiFiManager* wifiManager;
    CameraManager* cameraManager;
    LEDController* ledController;
    AIServerClient* aiClient;
    FirebaseManager* firebaseManager;
    FaceRecognitionManager* faceRecognitionManager;
    WebServerManager* webServerManager;
    OTAManager* otaManager;
    
    unsigned long lastFirebaseCheck;
    const unsigned long FIREBASE_CHECK_INTERVAL = 1000; // Check every 1 second
    
public:
    ESP32CameraSystem() {
        wifiManager = new WiFiManager(Config::WIFI_SSID, Config::WIFI_PASSWORD);
        cameraManager = new CameraManager();
        ledController = new LEDController(Config::FLASH_LED_PIN);
        aiClient = new AIServerClient(Config::AI_SERVER_URL);
        firebaseManager = new FirebaseManager();
        faceRecognitionManager = new FaceRecognitionManager(cameraManager, ledController, aiClient, firebaseManager);
        webServerManager = new WebServerManager(cameraManager, ledController, aiClient, wifiManager, faceRecognitionManager);
        otaManager = new OTAManager();
        
        lastFirebaseCheck = 0;
    }
    
    ~ESP32CameraSystem() {
        delete wifiManager;
        delete cameraManager;
        delete ledController;
        delete aiClient;
        delete firebaseManager;
        delete faceRecognitionManager;
        delete webServerManager;
        delete otaManager;
    }
    
    bool initialize() {
        Serial.begin(115200);
        Serial.setDebugOutput(false);
        
        Serial.println("=== ESP32-CAM 2FA FACE RECOGNITION SYSTEM ===");
        Serial.println("Features: Continuous Face Capture for 2FA Authentication");
        Serial.println("Max attempts: " + String(Config::MAX_FACE_ATTEMPTS));
        Serial.println("Confidence threshold: " + String(Config::FACE_CONFIDENCE_THRESHOLD * 100) + "%");
        Serial.println("Capture interval: " + String(Config::FACE_CAPTURE_INTERVAL) + "ms");
        Serial.println("Timeout: " + String(Config::FACE_TIMEOUT) + "ms");
        
        // Initialize camera
        if (!cameraManager->initialize()) {
            Serial.println("Camera initialization failed!");
            return false;
        }
        
        // Connect WiFi
        if (!wifiManager->connect()) {
            Serial.println("WiFi connection failed!");
            return false;
        }
        
        // Initialize Firebase
        if (!firebaseManager->initialize()) {
            Serial.println("Firebase initialization failed!");
            // Continue without Firebase (optional)
        }
        
        // Initialize OTA
        otaManager->initialize("esp32cam_2fa", "ota_ecohome2025");
        
        // Start web server
        if (!webServerManager->start()) {
            Serial.println("Web server start failed!");
            return false;
        }
        
        Serial.print("Camera Ready! Use 'http://");
        Serial.print(wifiManager->getLocalIP());
        Serial.println("' to connect");
        
        // LED blink to show ready
        ledController->blink(3);
        
        return true;
    }
    
    void loop() {
        // Handle OTA updates
        otaManager->handle();
        
        // Check Firebase for face recognition requests
        if (millis() - lastFirebaseCheck > FIREBASE_CHECK_INTERVAL) {
            checkForFaceRequests();
            lastFirebaseCheck = millis();
        }
        
        // Handle face recognition process
        faceRecognitionManager->loop();
        
        // Small delay to prevent watchdog reset
        delay(10);
    }

private:
    void checkForFaceRequests() {
        if (!wifiManager->isConnected()) return;
        
        String expectedUser = "";
        String sessionId = "";
        
        if (firebaseManager->checkFaceRequest(expectedUser, sessionId)) {
            if (!faceRecognitionManager->getStatus()) {
                Serial.printf("📨 New face recognition request: User=%s, Session=%s\n", 
                              expectedUser.c_str(), sessionId.c_str());
                
                // Clear the request to prevent multiple triggers
                firebaseManager->clearFaceRequest();
                
                // Start face recognition
                faceRecognitionManager->startRecognition(expectedUser, sessionId);
            }
        }
    }
};

// ===================== GLOBAL INSTANCES =====================
ESP32CameraSystem* cameraSystem;

void setup() {
    cameraSystem = new ESP32CameraSystem();
    if (!cameraSystem->initialize()) {
        Serial.println("System initialization failed!");
        while(1) delay(1000);
    }
}

void loop() {
    cameraSystem->loop();
}
