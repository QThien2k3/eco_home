/*
 * ESP32 SMART GATE CONTROLLER - ENHANCED v4.0
 * Features: RFID + Face Recognition (2FA) + Auto Light Control + Motor Speed Control
 * NEW: Automatic Light Control after successful authentication
 */

#include <WiFi.h>
#include <FirebaseESP32.h>
#include <SPI.h>
#include <MFRC522.h>
#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>

// =============================================================================
// THÔNG TIN WIFI VÀ FIREBASE
// =============================================================================
#define WIFI_SSID "    "
#define WIFI_PASSWORD "    "
#define FIREBASE_HOST "smart-home-b7a03-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyAO-6fXi3A_gLZz_uf9JKpQUOuxLfu6r1I"

// Face Recognition Server URLs
#define FACE_AI_SERVER "https://pyfaceid.myfreeiot.win"
#define ESP32_CAM_API "https://apiesp32facecam.myfreeiot.win"

// =============================================================================
// ĐỊNH NGHĨA PIN
// =============================================================================
// RFID RC522
#define RST_PIN         4
#define SS_PIN          5
#define MISO_PIN        19
#define MOSI_PIN        23
#define SCK_PIN         18

// RTC DS3231 và LCD I2C (I2C shared bus)
#define SDA_PIN         21
#define SCL_PIN         22

// Motor L298N
#define MOTOR_IN1       26
#define MOTOR_IN2       27
#define MOTOR_ENA       25

// Limit Switches
#define LIMIT_OPEN      32
#define LIMIT_CLOSE     33

// PIR Sensor với debouncing
#define PIR_PIN         34

// Status LED
#define STATUS_LED      2

// Buzzer cho feedback âm thanh
#define BUZZER_PIN      15

// =============================================================================
// CẤU HÌNH MOTOR SPEED
// =============================================================================
#define MOTOR_PWM_FREQ      1000  // Tần số PWM 1kHz
#define MOTOR_PWM_RESOLUTION 8    // Độ phân giải 8-bit (0-255)
#define MOTOR_SPEED         115  // Tốc độ motor (0-255), 100 = ~39% tốc độ

// =============================================================================
// KHỞI TẠO OBJECTS
// =============================================================================
MFRC522 rfid(SS_PIN, RST_PIN);
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4); // Địa chỉ I2C 0x27, màn hình 20x4
FirebaseData firebaseData;
FirebaseConfig firebaseConfig;
FirebaseAuth firebaseAuth;

// =============================================================================
// BIẾN TOÀN CỤC VÀ ENUMS
// =============================================================================

// Authentication States
enum AuthState {
    AUTH_IDLE,
    AUTH_RFID_PENDING,
    AUTH_RFID_SUCCESS,
    AUTH_FACE_PENDING,
    AUTH_FACE_SUCCESS,
    AUTH_COMPLETED,
    AUTH_FAILED
};

// Face Recognition States
enum FaceState {
    FACE_IDLE,
    FACE_CAPTURING,
    FACE_PROCESSING,
    FACE_SUCCESS,
    FACE_FAILED,
    FACE_TIMED_OUT
};

// Global variables
unsigned long lastPIRTime = 0;
unsigned long lastMotionTime = 0;
unsigned long lastPIRCheck = 0;
unsigned long lastLCDUpdate = 0;
bool gateIsOpen = false;
bool scanRequested = false;
bool motorRunning = false;
bool pirMotionDetected = false;
bool pirCurrentState = false;
bool pirPreviousState = false;
int pirStableCount = 0;
int failCount = 0;
int currentMotorSpeed = MOTOR_SPEED; // Tốc độ motor hiện tại
String lastUID = "";
String gateCurrentStatus = "unknown";
String lastUserName = "";
String targetPosition = "";
String cameraIP = "";
unsigned long lastScanTime = 0;
unsigned long motorStartTime = 0;
unsigned long lastFirebaseCheck = 0;
unsigned long lastStatusUpdate = 0;

// 2-Factor Authentication variables
AuthState currentAuthState = AUTH_IDLE;
FaceState currentFaceState = FACE_IDLE;
String pendingUID = "";
String pendingUserName = "";
unsigned long authStartTime = 0;
unsigned long faceStartTime = 0;
int faceAttempts = 0;
float lastFaceConfidence = 0.0;
bool faceRecognitionEnabled = true;

// 🆕 NEW: Auto Light Control Variables
bool autoLightControlEnabled = true;
unsigned long lastLightCommandTime = 0;
const unsigned long LIGHT_COMMAND_COOLDOWN = 2000; // 2 giây cooldown

// Timing constants
const unsigned long FIREBASE_CHECK_INTERVAL = 1000; // 1 giây
const unsigned long PIR_TIMEOUT = 10000; // 10 giây
const unsigned long PIR_DEBOUNCE_TIME = 500; // 0.5 giây debounce
const unsigned long PIR_STABLE_COUNT = 3; // Cần 3 lần đọc ổn định
const unsigned long SCAN_TIMEOUT = 30000; // 30 giây
const unsigned long MOTOR_TIMEOUT = 15000; // 15 giây timeout cho motor
const unsigned long STATUS_UPDATE_INTERVAL = 2000; // 2 giây
const unsigned long LCD_UPDATE_INTERVAL = 500; // 0.5 giây cập nhật LCD

// Face Recognition constants
const unsigned long AUTH_TIMEOUT = 60000; // 60 giây cho toàn bộ quá trình auth
const unsigned long FACE_TIMEOUT = 30000; // 30 giây cho face recognition
const unsigned long FACE_CAPTURE_INTERVAL = 2000; // 2 giây giữa các lần capture
const int MAX_FACE_ATTEMPTS = 10; // Tăng số lần thử để quét liên tục
const float FACE_CONFIDENCE_THRESHOLD = 0.60; // 60% confidence threshold như yêu cầu

// =============================================================================
// CLASS MOTOR CONTROLLER - QUẢN LÝ TỐC ĐỘ MOTOR
// =============================================================================
class MotorController {
public:
    void begin() {
        // Cấu hình PWM cho motor ENA (ESP32 Arduino Core v3.x)
        ledcAttach(MOTOR_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
        
        // Khởi tạo các chân điều khiển hướng
        pinMode(MOTOR_IN1, OUTPUT);
        pinMode(MOTOR_IN2, OUTPUT);
        
        // Dừng motor ban đầu
        stopMotor();
        
        Serial.printf("Motor Controller initialized - Speed: %d/255 (%.1f%%)\n", 
                      currentMotorSpeed, (currentMotorSpeed / 255.0) * 100);
    }
    
    void setSpeed(int speed) {
        // Giới hạn tốc độ trong khoảng 0-255
        if (speed < 0) speed = 0;
        if (speed > 255) speed = 255;
        
        currentMotorSpeed = speed;
        Serial.printf("Motor speed set to: %d/255 (%.1f%%)\n", 
                      currentMotorSpeed, (currentMotorSpeed / 255.0) * 100);
        
        // Nếu motor đang chạy, cập nhật tốc độ ngay lập tức
        if (motorRunning) {
            ledcWrite(MOTOR_ENA, currentMotorSpeed);
        }
    }
    
    void startOpening() {
        Serial.printf("Starting motor - Opening direction at speed %d\n", currentMotorSpeed);
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_ENA, currentMotorSpeed);
    }
    
    void startClosing() {
        Serial.printf("Starting motor - Closing direction at speed %d\n", currentMotorSpeed);
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        ledcWrite(MOTOR_ENA, currentMotorSpeed);
    }
    
    void stopMotor() {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        ledcWrite(MOTOR_ENA, 0);
        Serial.println("Motor stopped");
    }
    
    int getCurrentSpeed() {
        return currentMotorSpeed;
    }
    
    float getSpeedPercentage() {
        return (currentMotorSpeed / 255.0) * 100;
    }
};

// =============================================================================
// CLASS SOUND MANAGER - QUẢN LÝ ÂM THANH
// =============================================================================
class SoundManager {
public:
    void begin() {
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
    }
    
    void beepSuccess() {
        // 2 beep ngắn cho thành công
        for (int i = 0; i < 2; i++) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(200);
            digitalWrite(BUZZER_PIN, LOW);
            delay(100);
        }
    }
    
    void beepFail() {
        // 3 beep dài cho thất bại
        for (int i = 0; i < 3; i++) {
            digitalWrite(BUZZER_PIN, HIGH);
            delay(500);
            digitalWrite(BUZZER_PIN, LOW);
            delay(200);
        }
    }
    
    void beepInfo() {
        // 1 beep ngắn cho thông tin
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
    }
    
    void beepFaceStart() {
        // Melody cho bắt đầu face scan
        int notes[] = {1000, 1200, 1500};
        for (int i = 0; i < 3; i++) {
            tone(BUZZER_PIN, notes[i], 150);
            delay(200);
        }
        noTone(BUZZER_PIN);
    }
    
    // 🆕 NEW: Special beep for auto light activation
    void beepAutoLight() {
        // Triple ascending beep for auto light activation
        int notes[] = {800, 1000, 1200, 1000};
        for (int i = 0; i < 4; i++) {
            tone(BUZZER_PIN, notes[i], 100);
            delay(120);
        }
        noTone(BUZZER_PIN);
    }
};

// =============================================================================
// CLASS ENHANCED LCD MANAGER - QUẢN LÝ HIỂN THỊ LCD VỚI FACE RECOGNITION
// =============================================================================
class EnhancedLCDManager {
private:
    unsigned long lastScrollTime = 0;
    unsigned long lastBlinkTime = 0;
    bool blinkState = false;
    
public:
    void begin() {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        
        // Màn hình khởi động
        displayBootScreen();
        delay(2000);
        lcd.clear();
    }
    
    void displayBootScreen() {
        lcd.setCursor(0, 0);
        lcd.print("  SMART GATE v4.0   ");
        lcd.setCursor(0, 1);
        lcd.print("====================");
        lcd.setCursor(0, 2);
        lcd.print("2FA + Auto Lights   ");
        lcd.setCursor(0, 3);
        lcd.printf("Motor Speed: %d/255  ", currentMotorSpeed);
    }
    
    void displayMainScreen() {
        // Dòng 1: Trạng thái cửa và auth state
        lcd.setCursor(0, 0);
        lcd.print("Gate: ");
        String gateDisplay = gateCurrentStatus;
        if (gateDisplay == "open") gateDisplay = "OPEN";
        else if (gateDisplay == "closed") gateDisplay = "CLOSED";
        else if (gateDisplay == "opening") gateDisplay = "OPENING";
        else if (gateDisplay == "closing") gateDisplay = "CLOSING ";
        else gateDisplay = "UNKNOWN ";
        lcd.print(gateDisplay);
        
        // Hiển thị auth status và auto light status
        lcd.setCursor(13, 0);
        switch (currentAuthState) {
            case AUTH_IDLE: 
                lcd.print(autoLightControlEnabled ? "[A-ON]" : "[A-OF]"); 
                break;
            case AUTH_RFID_SUCCESS: lcd.print("[RFID+]"); break;
            case AUTH_FACE_PENDING: lcd.print("[FACE?]"); break;
            case AUTH_COMPLETED: 
                lcd.print(autoLightControlEnabled ? "[LITE!]" : "[ OK! ]"); 
                break;
            case AUTH_FAILED: lcd.print("[ERROR]"); break;
            default: lcd.print("[?????]"); break;
        }
        
        // Dòng 2: Thời gian và system status
        DateTime now = rtc.now();
        lcd.setCursor(0, 1);
        lcd.printf("%02d/%02d %02d:%02d:%02d", 
                  now.day(), now.month(), 
                  now.hour(), now.minute(), now.second());
        
        // Hiển thị system status với auto light indicator
        lcd.setCursor(14, 1);
        lcd.print(WiFi.status() == WL_CONNECTED ? "W" : "-");
        lcd.print(pirMotionDetected ? "M" : "-");
        lcd.print(faceRecognitionEnabled ? "F" : "X");
        lcd.print(autoLightControlEnabled ? "L" : "-"); // L = Auto Light enabled
        
        // Dòng 3: Thông tin người dùng hoặc pending user
        lcd.setCursor(0, 2);
        if (currentAuthState == AUTH_FACE_PENDING && pendingUserName.length() > 0) {
            String userDisplay = "Auth: " + pendingUserName;
            if (userDisplay.length() > 20) {
                userDisplay = userDisplay.substring(0, 17) + "...";
            }
            lcd.print(userDisplay);
            // Pad với spaces
            while (userDisplay.length() < 20) {
                lcd.print(" ");
                userDisplay += " ";
            }
        } else if (lastUserName.length() > 0) {
            String userDisplay = "User: " + lastUserName;
            if (userDisplay.length() > 20) {
                userDisplay = userDisplay.substring(0, 17) + "...";
            }
            lcd.print(userDisplay);
            // Pad với spaces
            while (userDisplay.length() < 20) {
                lcd.print(" ");
                userDisplay += " ";
            }
        } else {
            lcd.print("No user logged      ");
        }
        
        // Dòng 4: Trạng thái hệ thống và cảnh báo
        lcd.setCursor(0, 3);
        if (!faceRecognitionEnabled) {
            // Cảnh báo Face Recognition không có
            lcd.print("WARNING: No Face AI!");
        } else if (currentAuthState == AUTH_FACE_PENDING) {
            displayFaceProgress();
        } else if (scanRequested) {
            lcd.print("SCAN CARD PLEASE... ");
        } else if (motorRunning) {
            lcd.printf("MOTOR: %d/255 (%.0f%%) ", currentMotorSpeed, (currentMotorSpeed / 255.0) * 100);
        } else if (failCount >= 5) {
            lcd.print("SECURITY ALERT!!!   ");
        } else if (failCount > 0) {
            lcd.printf("Fails: %d AutoL:%s   ", failCount, autoLightControlEnabled ? "ON" : "OF");
        } else {
            lcd.printf("2FA+AutoL Ready:%d  ", currentMotorSpeed);
        }
    }
    
    void displayFaceProgress() {
        static int progressDots = 0;
        
        lcd.setCursor(0, 3);
        
        switch (currentFaceState) {
            case FACE_CAPTURING:
                lcd.printf("Face: Scan %d/%d", faceAttempts, MAX_FACE_ATTEMPTS);
                for (int i = 0; i < (progressDots % 4); i++) {
                    lcd.print(".");
                }
                for (int i = progressDots % 4; i < 3; i++) {
                    lcd.print(" ");
                }
                progressDots++;
                break;
                
            case FACE_PROCESSING:
                lcd.printf("Face: Check %d/%d", faceAttempts, MAX_FACE_ATTEMPTS);
                for (int i = 0; i < (progressDots % 4); i++) {
                    lcd.print(".");
                }
                for (int i = progressDots % 4; i < 3; i++) {
                    lcd.print(" ");
                }
                progressDots++;
                break;
                
            case FACE_SUCCESS:
                lcd.printf("Face: OK! %.0f%%>=60%%", lastFaceConfidence * 100);
                break;
                
            case FACE_FAILED:
                lcd.printf("Face: %.0f%% (%d/%d)  ", lastFaceConfidence * 100, faceAttempts, MAX_FACE_ATTEMPTS);
                break;
                
            case FACE_TIMED_OUT:
                lcd.print("Face: TIMEOUT!      ");
                break;
                
            default:
                lcd.printf("Face: Ready (%d/%d)  ", faceAttempts, MAX_FACE_ATTEMPTS);
                break;
        }
    }
    
    void displayAuthProgress(String stage, String message) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("=== 2FA PROGRESS ===");
        
        lcd.setCursor(0, 1);
        lcd.print("Stage: " + stage);
        
        lcd.setCursor(0, 2);
        if (message.length() <= 20) {
            lcd.print(message);
        } else {
            lcd.print(message.substring(0, 20));
        }
        
        // Progress bar
        lcd.setCursor(0, 3);
        int progress = 0;
        if (currentAuthState == AUTH_RFID_SUCCESS) progress = 50;
        else if (currentAuthState == AUTH_FACE_SUCCESS) progress = 90;
        else if (currentAuthState == AUTH_COMPLETED) progress = 100;
        
        lcd.print("Progress: ");
        lcd.print(progress);
        lcd.print("%");
        
        // Visual progress bar
        lcd.setCursor(14, 3);
        int bars = progress / 20;
        for (int i = 0; i < bars && i < 6; i++) {
            lcd.print("#");
        }
    }
    
    // 🆕 NEW: Display auto light activation
    void displayAutoLightActivation() {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("=== AUTO LIGHTS ===");
        
        lcd.setCursor(0, 1);
        lcd.print("Activating lights...");
        
        lcd.setCursor(0, 2);
        lcd.print("Living Room + Garage");
        
        lcd.setCursor(0, 3);
        lcd.print("Welcome home!");
    }
    
    void displayFaceInstructions() {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("=== FACE SCAN ===");
        
        lcd.setCursor(0, 1);
        lcd.print("Look at camera");
        
        lcd.setCursor(0, 2);
        lcd.print("Stay still...");
        
        lcd.setCursor(0, 3);
        if (lastFaceConfidence > 0) {
            lcd.printf("Conf: %.0f%% (need >=60%%) ", lastFaceConfidence * 100);
        } else {
            lcd.print("Confidence: --- (>=60%)");
        }
    }
    
    void displayCardInfo(String uid, String name, bool isValid) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RFID DETECTED:");
        
        lcd.setCursor(0, 1);
        lcd.print("UID: ");
        // Hiển thị dấu * để bảo mật thay vì UID thật
        String maskedUID = "";
        for (int i = 0; i < uid.length() && i < 8; i++) {
            maskedUID += "*";
        }
        lcd.print(maskedUID);
        
        lcd.setCursor(0, 2);
        if (isValid) {
            lcd.print("User: ");
            String nameDisplay = name;
            if (nameDisplay.length() > 14) {
                nameDisplay = nameDisplay.substring(0, 11) + "...";
            }
            lcd.print(nameDisplay);
        } else {
            lcd.print("ACCESS DENIED!");
        }
        
        lcd.setCursor(0, 3);
        if (isValid) {
            lcd.print("Next: Face scan...");
        } else {
            lcd.print("Invalid card!");
        }
    }
    
    void displayError(String error) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ERROR:");
        
        // Word wrap cho error message
        if (error.length() <= 20) {
            lcd.setCursor(0, 1);
            lcd.print(error);
        } else {
            lcd.setCursor(0, 1);
            lcd.print(error.substring(0, 20));
            if (error.length() > 20) {
                lcd.setCursor(0, 2);
                lcd.print(error.substring(20, min(40, (int)error.length())));
            }
        }
        
        delay(2000);
    }
    
    void displayAlert(String message) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("*** ALERT ***");
        
        lcd.setCursor(0, 2);
        if (message.length() <= 20) {
            lcd.print(message);
        } else {
            lcd.print(message.substring(0, 20));
            if (message.length() > 20) {
                lcd.setCursor(0, 3);
                lcd.print(message.substring(20, min(40, (int)message.length())));
            }
        }
    }
    
    void setBrightness(bool bright) {
        if (bright) {
            lcd.backlight();
        } else {
            lcd.noBacklight();
        }
    }
};

// =============================================================================
// CLASS FACE RECOGNITION MANAGER
// =============================================================================
class FaceRecognitionManager {
private:
    HTTPClient http;
    
public:
    bool isServerAvailable() {
        http.begin(FACE_AI_SERVER "/health");
        http.setTimeout(5000);
        
        int httpResponseCode = http.GET();
        http.end();
        
        return (httpResponseCode == 200);
    }
    
    bool captureAndRecognize(String expectedUser, float& confidence) {
        Serial.println("Starting face capture and recognition...");
        
        // Step 1: Capture image from ESP32-CAM
        if (!captureImage()) {
            Serial.println("Failed to capture image");
            return false;
        }
        
        // Step 2: Recognize face
        return recognizeFace(expectedUser, confidence);
    }
    
private:
    bool captureImage() {
        // Sử dụng ESP32-CAM API domain để capture
        http.begin(ESP32_CAM_API "/capture");
        http.setTimeout(10000);
        
        int httpResponseCode = http.GET();
        
        if (httpResponseCode == 200) {
            Serial.println("Image captured successfully");
            http.end();
            return true;
        } else {
            Serial.printf("Capture failed with code: %d\n", httpResponseCode);
            http.end();
            return false;
        }
    }
    
    bool recognizeFace(String expectedUser, float& confidence) {
        // Gọi AI server để recognize
        http.begin(FACE_AI_SERVER "/recognize");
        http.setTimeout(15000);
        http.addHeader("Content-Type", "application/json");
        
        // Tạo request với expected user (nếu cần)
        DynamicJsonDocument doc(1024);
        doc["expected_user"] = expectedUser;
        doc["threshold"] = FACE_CONFIDENCE_THRESHOLD;
        
        String requestBody;
        serializeJson(doc, requestBody);
        
        int httpResponseCode = http.POST(requestBody);
        
        if (httpResponseCode == 200) {
            String response = http.getString();
            http.end();
            
            return parseRecognitionResponse(response, expectedUser, confidence);
        } else {
            Serial.printf("Recognition failed with code: %d\n", httpResponseCode);
            http.end();
            return false;
        }
    }
    
    bool parseRecognitionResponse(String response, String expectedUser, float& confidence) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        
        if (error) {
            Serial.println("Failed to parse recognition response");
            return false;
        }
        
        bool success = doc["success"];
        String recognizedName = doc["name"];
        confidence = doc["confidence"];
        
        Serial.printf("Recognition result: %s, Name: %s, Confidence: %.2f\n", 
                      success ? "SUCCESS" : "FAILED", recognizedName.c_str(), confidence);
        
        if (!success) {
            return false;
        }
        
        // Kiểm tra confidence threshold
        if (confidence < FACE_CONFIDENCE_THRESHOLD) {
            Serial.printf("Confidence too low: %.2f < %.2f\n", confidence, FACE_CONFIDENCE_THRESHOLD);
            return false;
        }
        
        // Kiểm tra tên người dùng (case insensitive)
        expectedUser.toLowerCase();
        recognizedName.toLowerCase();
        
        if (recognizedName == expectedUser || recognizedName.indexOf(expectedUser) >= 0) {
            Serial.println("Face recognition successful - user match!");
            return true;
        } else {
            Serial.printf("User mismatch: expected %s, got %s\n", 
                          expectedUser.c_str(), recognizedName.c_str());
            return false;
        }
    }
};

// =============================================================================
// 🆕 NEW: AUTO LIGHT CONTROLLER CLASS
// =============================================================================
class AutoLightController {
private:
    unsigned long lastCommandTime = 0;
    
public:
    void begin() {
        Serial.println("🔆 Auto Light Controller initialized");
    }
    
    bool sendLightCommand(String lightId, bool state) {
        if (millis() - lastCommandTime < LIGHT_COMMAND_COOLDOWN) {
            Serial.println("🔆 Light command cooldown active, skipping...");
            return false;
        }
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("🔆 WiFi not connected, cannot send light command");
            return false;
        }
        
        String path = "/auto_commands/" + lightId;
        
        if (Firebase.setBool(firebaseData, path.c_str(), state)) {
            Serial.printf("🔆 Auto light command sent: %s = %s\n", lightId.c_str(), state ? "ON" : "OFF");
            lastCommandTime = millis();
            return true;
        } else {
            Serial.printf("🔆 Failed to send light command: %s\n", firebaseData.errorReason().c_str());
            return false;
        }
    }
    
    void activateWelcomeLights() {
        if (!autoLightControlEnabled) {
            Serial.println("🔆 Auto light control disabled, skipping welcome lights");
            return;
        }
        
        Serial.println("🔆 Activating welcome lights (Living Room + Garage)...");
        
        // Send commands to turn on light1 (Living Room) and light4 (Garage)
        bool light1Success = sendLightCommand("light1", true);
        delay(100); // Small delay between commands
        bool light4Success = sendLightCommand("light4", true);
        
        if (light1Success || light4Success) {
            // Log the auto light activation
            logAutoLightEvent();
        }
        
        Serial.printf("🔆 Welcome lights result - Living Room: %s, Garage: %s\n", 
                     light1Success ? "Success" : "Failed",
                     light4Success ? "Success" : "Failed");
    }
    
private:
    void logAutoLightEvent() {
        if (WiFi.status() != WL_CONNECTED) return;
        
        DateTime now = rtc.now();
        long timestamp = now.unixtime();
        
        String logId = String(timestamp) + "_autolight_" + String(random(1000, 9999));
        String logPath = "/history/" + logId;
        
        FirebaseJson json;
        json.set("type", "auto_light_activation");
        json.set("timestamp", timestamp);
        json.set("device", "ESP32_GATE_2FA");
        json.set("trigger", "successful_authentication");
        json.set("lights_activated", "light1,light4");
        json.set("user", lastUserName);
        json.set("uid", lastUID);
        
        Firebase.setJSON(firebaseData, logPath, json);
        Serial.println("🔆 Auto light event logged to history");
    }
};

// =============================================================================
// CLASS ENHANCED SMART GATE - HỆ THỐNG CHÍNH VỚI 2FA VÀ AUTO LIGHT CONTROL
// =============================================================================
class EnhancedSmartGate {
private:
    EnhancedLCDManager lcdManager;
    SoundManager soundManager;
    FaceRecognitionManager faceManager;
    MotorController motorController;
    AutoLightController autoLightController; // 🆕 NEW
    unsigned long lastCardScanTime = 0;
    unsigned long lastFaceAttemptTime = 0;
    const unsigned long CARD_SCAN_COOLDOWN = 2000;
    
public:
    void begin() {
        Serial.begin(115200);
        Serial.println("=== ENHANCED SMART GATE SYSTEM STARTING v4.0 ===");
        Serial.println("=== 2-Factor Authentication: RFID + Face Recognition ===");
        Serial.println("=== Auto Light Control: Living Room + Garage ===");
        Serial.println("=== Motor Speed Control: PWM Enabled ===");
        
        initializePins();
        motorController.begin();
        soundManager.begin();
        autoLightController.begin(); // 🆕 NEW
        soundManager.beepInfo();
        
        Wire.begin(SDA_PIN, SCL_PIN);
        lcdManager.begin();
        Serial.println("Enhanced LCD I2C initialized");
        
        if (!rtc.begin()) {
            Serial.println("Không tìm thấy RTC DS3231!");
            lcdManager.displayError("RTC NOT FOUND!");
            soundManager.beepFail();
            while (1);
        }
        
        if (rtc.lostPower()) {
            Serial.println("RTC mất nguồn, đặt lại thời gian...");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        
        SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
        rfid.PCD_Init();
        Serial.println("RFID RC522 đã khởi tạo");
        
        connectWiFi();
        initializeFirebase();
        checkFaceRecognitionServer();
        syncInitialState();
        resetAuthState();
        
        blinkStatusLED(3);
        soundManager.beepSuccess();
        
        Serial.println("=== HỆ THỐNG 2FA VỚI AUTO LIGHT CONTROL ĐÃ SẴN SÀNG ===");
    }
    
    void loop() {
        // KIỂM TRA LIMIT SWITCHES NGAY LẬP TỨC - KHÔNG DEBOUNCE
        checkLimitSwitchesImmediate();
        
        if (millis() - lastFirebaseCheck > FIREBASE_CHECK_INTERVAL) {
            checkFirebaseCommands();
            lastFirebaseCheck = millis();
        }
        
        handleRFIDScan();
        handle2FactorAuth();
        
        if (millis() - lastPIRCheck > PIR_DEBOUNCE_TIME) {
            handlePIRSensorImproved();
            lastPIRCheck = millis();
        }
        
        handleScanTimeout();
        
        if (millis() - lastLCDUpdate > LCD_UPDATE_INTERVAL) {
            lcdManager.displayMainScreen();
            lastLCDUpdate = millis();
        }
        
        if (millis() - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
            updateSystemStatus();
            lastStatusUpdate = millis();
        }
        
        checkMotorTimeout();
        delay(10);
    }

private:
    void initializePins() {
        pinMode(LIMIT_OPEN, INPUT_PULLUP);
        pinMode(LIMIT_CLOSE, INPUT_PULLUP);
        pinMode(PIR_PIN, INPUT);
        pinMode(STATUS_LED, OUTPUT);
        digitalWrite(STATUS_LED, LOW);
        Serial.println("Các chân GPIO đã được khởi tạo");
    }
    
    void checkLimitSwitchesImmediate() {
        if (!motorRunning) return;
        
        bool openHit = digitalRead(LIMIT_OPEN) == LOW;
        bool closeHit = digitalRead(LIMIT_CLOSE) == LOW;
        
        if (openHit && targetPosition == "open") {
            Serial.println("*** LIMIT_OPEN HIT - STOPPING MOTOR IMMEDIATELY ***");
            motorController.stopMotor();
            motorRunning = false;
            gateIsOpen = true;
            gateCurrentStatus = "open";
            updateGateStatus("open");
            logEvent("gate_open_limit", "");
            soundManager.beepSuccess();
            lastPIRTime = millis();
            pirMotionDetected = true;
            Serial.println("Gate successfully opened - reached OPEN limit");
            return;
        }
        
        if (closeHit && targetPosition == "close") {
            Serial.println("*** LIMIT_CLOSE HIT - STOPPING MOTOR IMMEDIATELY ***");
            motorController.stopMotor();
            motorRunning = false;
            gateIsOpen = false;
            gateCurrentStatus = "closed";
            updateGateStatus("closed");
            logEvent("gate_close_limit", "");
            soundManager.beepInfo();
            lastUserName = "";
            pirMotionDetected = false;
            Serial.println("Gate successfully closed - reached CLOSE limit");
            return;
        }
        
        if (openHit && targetPosition == "close") {
            Serial.println("*** WARNING: Hit OPEN limit while closing - EMERGENCY STOP ***");
            motorController.stopMotor();
            motorRunning = false;
            gateCurrentStatus = "error";
            updateGateStatus("error");
            lcdManager.displayError("Limit Error: Open!");
            soundManager.beepFail();
            logEvent("limit_error_open", "");
        }
        
        if (closeHit && targetPosition == "open") {
            Serial.println("*** WARNING: Hit CLOSE limit while opening - EMERGENCY STOP ***");
            motorController.stopMotor();
            motorRunning = false;
            gateCurrentStatus = "error";
            updateGateStatus("error");
            lcdManager.displayError("Limit Error: Close!");
            soundManager.beepFail();
            logEvent("limit_error_close", "");
        }
    }
    
    void checkFaceRecognitionServer() {
        Serial.println("Checking Face Recognition server...");
        lcdManager.displayAuthProgress("Init", "Checking AI server...");
        
        if (faceManager.isServerAvailable()) {
            Serial.println("Face Recognition server is available");
            faceRecognitionEnabled = true;
            soundManager.beepSuccess();
        } else {
            Serial.println("Warning: Face Recognition server not available");
            faceRecognitionEnabled = false;
            lcdManager.displayError("Face AI offline!");
            soundManager.beepFail();
            delay(2000);
        }
    }
    
    void resetAuthState() {
        currentAuthState = AUTH_IDLE;
        currentFaceState = FACE_IDLE;
        pendingUID = "";
        pendingUserName = "";
        authStartTime = 0;
        faceStartTime = 0;
        faceAttempts = 0;
        lastFaceConfidence = 0.0;
        Serial.println("Authentication state reset");
    }
    
    void handle2FactorAuth() {
        if (currentAuthState != AUTH_IDLE && 
            authStartTime > 0 && 
            millis() - authStartTime > AUTH_TIMEOUT) {
            
            Serial.println("Authentication timeout!");
            lcdManager.displayError("Auth Timeout!");
            soundManager.beepFail();
            logEvent("auth_timeout", pendingUID);
            resetAuthState();
            return;
        }
        
        switch (currentAuthState) {
            case AUTH_IDLE:
                // Đang chờ quét RFID
                break;
                
            case AUTH_RFID_SUCCESS:
                // RFID thành công, BẮT BUỘC phải có face recognition
                if (faceRecognitionEnabled) {
                    Serial.println("RFID OK - Starting MANDATORY Face Recognition step...");
                    startFaceRecognition();
                } else {
                    // KHÔNG cho phép mở cửa chỉ với RFID
                    Serial.println("ERROR: Face Recognition not available - 2FA REQUIRED!");
                    currentAuthState = AUTH_FAILED;
                    lcdManager.displayError("Need Face Recognition!");
                    soundManager.beepFail();
                    logEvent("face_not_available", pendingUID);
                    failCount++;
                    updateFailCount();
                }
                break;
                
            case AUTH_FACE_PENDING:
                // Đang thực hiện face recognition
                handleFaceRecognition();
                break;
                
            case AUTH_FACE_SUCCESS:
                // Face recognition thành công - hoàn tất 2FA
                Serial.println("Face Recognition OK - Completing 2FA...");
                completeAuthentication();
                break;
                
            case AUTH_COMPLETED:
                // Authentication hoàn tất, reset sau khi mở cửa
                if (!motorRunning) {
                    Serial.println("Gate operation completed - Resetting auth state");
                    resetAuthState();
                }
                break;
                
            case AUTH_FAILED:
                // Authentication thất bại, reset sau delay
                Serial.println("Authentication failed - Resetting...");
                delay(2000);
                resetAuthState();
                break;
        }
    }
    
    void startFaceRecognition() {
        Serial.printf("Starting face recognition for user: %s\n", pendingUserName.c_str());
        
        currentAuthState = AUTH_FACE_PENDING;
        currentFaceState = FACE_IDLE;
        faceStartTime = millis();
        faceAttempts = 0;
        
        lcdManager.displayAuthProgress("Face", "Prepare for scan...");
        soundManager.beepFaceStart();
        
        logEvent("face_start", pendingUID);
    }
    
    void handleFaceRecognition() {
        if (millis() - faceStartTime > FACE_TIMEOUT) {
            Serial.println("Face recognition timeout!");
            currentFaceState = FACE_TIMED_OUT;
            currentAuthState = AUTH_FAILED;
            lcdManager.displayError("Face Timeout!");
            soundManager.beepFail();
            logEvent("face_timeout", pendingUID);
            failCount++;
            updateFailCount();
            return;
        }
        
        // Tăng số lần thử để quét liên tục đến khi đạt 60%
        if (faceAttempts >= MAX_FACE_ATTEMPTS) {
            Serial.println("Max face attempts reached!");
            currentFaceState = FACE_FAILED;
            currentAuthState = AUTH_FAILED;
            lcdManager.displayError("Face: Max attempts!");
            soundManager.beepFail();
            logEvent("face_max_attempts", pendingUID);
            failCount++;
            updateFailCount();
            return;
        }
        
        switch (currentFaceState) {
            case FACE_IDLE:
                if (millis() - lastFaceAttemptTime > FACE_CAPTURE_INTERVAL) {
                    currentFaceState = FACE_CAPTURING;
                    faceAttempts++;
                    lastFaceAttemptTime = millis();
                    
                    lcdManager.displayFaceInstructions();
                    soundManager.beepInfo();
                    
                    Serial.printf("Face attempt %d/%d - Target: %.0f%% confidence\n", 
                                  faceAttempts, MAX_FACE_ATTEMPTS, FACE_CONFIDENCE_THRESHOLD * 100);
                }
                break;
                
            case FACE_CAPTURING:
                currentFaceState = FACE_PROCESSING;
                lcdManager.displayAuthProgress("Face", "Processing...");
                
                float confidence = 0.0;
                bool faceSuccess = faceManager.captureAndRecognize(pendingUserName, confidence);
                lastFaceConfidence = confidence;
                
                if (faceSuccess && confidence >= FACE_CONFIDENCE_THRESHOLD) {
                    Serial.printf("Face recognition SUCCESS! Confidence: %.1f%% (>= %.0f%%)\n", 
                                  confidence * 100, FACE_CONFIDENCE_THRESHOLD * 100);
                    currentFaceState = FACE_SUCCESS;
                    currentAuthState = AUTH_FACE_SUCCESS;
                    
                    lcdManager.displayAuthProgress("Face", "SUCCESS!");
                    soundManager.beepSuccess();
                    
                    logEvent("face_success", pendingUID);
                } else {
                    Serial.printf("Face recognition FAILED. Confidence: %.1f%% (need >= %.0f%%) - Attempt %d/%d\n", 
                                  confidence * 100, FACE_CONFIDENCE_THRESHOLD * 100, faceAttempts, MAX_FACE_ATTEMPTS);
                    
                    // Hiển thị confidence hiện tại và tiếp tục thử
                    lcdManager.displayAuthProgress("Face", 
                        String("Try: ") + String(faceAttempts) + "/" + String(MAX_FACE_ATTEMPTS) + 
                        " (" + String((int)(confidence * 100)) + "%)");
                    
                    logEvent("face_fail", pendingUID);
                    
                    // Tiếp tục thử nếu chưa hết lần
                    if (faceAttempts < MAX_FACE_ATTEMPTS) {
                        delay(500); // Delay ngắn rồi thử lại
                        currentFaceState = FACE_IDLE;
                    } else {
                        currentFaceState = FACE_FAILED;
                        currentAuthState = AUTH_FAILED;
                        failCount++;
                        updateFailCount();
                    }
                }
                break;
        }
    }
    
    void completeAuthentication() {
        Serial.println("=== 2-Factor Authentication COMPLETED SUCCESSFULLY ===");
        Serial.printf("✓ RFID: %s (User: %s)\n", pendingUID.c_str(), pendingUserName.c_str());
        Serial.printf("✓ FACE: %.1f%% confidence (>= 60%%)\n", lastFaceConfidence * 100);
        Serial.println("✓ Both factors verified - Opening gate + Auto lights...");
        
        currentAuthState = AUTH_COMPLETED;
        lastUserName = pendingUserName;
        
        // 🆕 NEW: Show auto light activation on LCD
        lcdManager.displayAutoLightActivation();
        soundManager.beepAutoLight(); // Special beep for auto lights
        
        // 🆕 NEW: Activate welcome lights BEFORE opening gate
        autoLightController.activateWelcomeLights();
        
        delay(1000); // Show the auto light message for 1 second
        
        lcdManager.displayAuthProgress("Complete", "Both verified! Opening...");
        soundManager.beepSuccess();
        
        logEvent("2fa_success", pendingUID);
        failCount = 0;
        updateFailCount();
        
        if (!gateIsOpen && !motorRunning) {
            openGate();
        }
        
        lastPIRTime = millis();
        pirMotionDetected = true;
    }
    
    void handlePIRSensorImproved() {
        bool currentReading = digitalRead(PIR_PIN) == HIGH;
        
        if (currentReading == pirCurrentState) {
            pirStableCount++;
        } else {
            pirStableCount = 0;
            pirCurrentState = currentReading;
        }
        
        if (pirStableCount >= PIR_STABLE_COUNT) {
            if (pirCurrentState != pirPreviousState) {
                pirPreviousState = pirCurrentState;
                
                if (pirCurrentState) {
                    // Motion detected - cập nhật thời gian và trạng thái
                    pirMotionDetected = true;
                    lastPIRTime = millis();
                    Serial.println("PIR: Phát hiện chuyển động - Reset timer đóng cửa");
                    
                    if (WiFi.status() == WL_CONNECTED) {
                        Firebase.setBool(firebaseData, "/sensors/motion", true);
                        Firebase.setInt(firebaseData, "/sensors/lastMotionTime", millis() / 1000);
                    }
                } else {
                    // Motion stopped - BẮT ĐẦU đếm ngược 10 giây
                    Serial.println("PIR: Ngừng phát hiện chuyển động - Bắt đầu đếm ngược 10s");
                    lastPIRTime = millis(); // Reset timer để bắt đầu đếm 10s
                    // pirMotionDetected vẫn = true, sẽ chuyển false sau 10s
                }
            }
        }
        
        // Logic đóng cửa: CHỈ đóng khi không còn chuyển động VÀ đã đợi đủ 10s
        if (pirMotionDetected && !pirCurrentState && millis() - lastPIRTime > PIR_TIMEOUT) {
            if (gateIsOpen && !motorRunning && currentAuthState == AUTH_IDLE) {
                Serial.println("PIR: Đã hết chuyển động và đợi đủ 10s - Tự động đóng cửa");
                closeGate();
            }
            pirMotionDetected = false;
            
            if (WiFi.status() == WL_CONNECTED) {
                Firebase.setBool(firebaseData, "/sensors/motion", false);
            }
        }
        
        // Debug info mỗi 3 giây
        static unsigned long lastPIRDebug = 0;
        if (millis() - lastPIRDebug > 3000) {
            if (pirMotionDetected) {
                unsigned long timeRemaining = PIR_TIMEOUT - (millis() - lastPIRTime);
                Serial.printf("PIR Status: Motion=%s, Current=%s, TimeToClose=%lus\n",
                             pirMotionDetected ? "YES" : "NO",
                             pirCurrentState ? "DETECTING" : "CLEAR",
                             pirCurrentState ? 0 : (timeRemaining > 0 ? timeRemaining/1000 : 0));
            }
            lastPIRDebug = millis();
        }
    }
    
    void blinkStatusLED(int times) {
        for (int i = 0; i < times; i++) {
            digitalWrite(STATUS_LED, HIGH);
            delay(200);
            digitalWrite(STATUS_LED, LOW);
            delay(200);
        }
    }
    
    void connectWiFi() {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("Đang kết nối WiFi");
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connecting WiFi...");
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            Serial.print(".");
            lcd.setCursor(attempts % 20, 1);
            lcd.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println();
            Serial.print("WiFi đã kết nối! IP: ");
            Serial.println(WiFi.localIP());
            digitalWrite(STATUS_LED, HIGH);
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("WiFi Connected!");
            lcd.setCursor(0, 1);
            lcd.print(WiFi.localIP().toString());
            delay(2000);
        } else {
            Serial.println("\nLỗi kết nối WiFi!");
            lcdManager.displayError("WiFi Failed!");
        }
    }
    
    void initializeFirebase() {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Không có WiFi, bỏ qua Firebase");
            return;
        }
        
        firebaseConfig.host = FIREBASE_HOST;
        firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
        
        Firebase.begin(&firebaseConfig, &firebaseAuth);
        Firebase.reconnectWiFi(true);
        
        Firebase.setReadTimeout(firebaseData, 1000 * 60);
        Firebase.setwriteSizeLimit(firebaseData, "tiny");
        
        Serial.println("Firebase đã được khởi tạo");
        
        if (Firebase.setString(firebaseData, "/test/connection", "ESP32_GATE_2FA_AUTO_LIGHTS_V4.0")) {
            Serial.println("Firebase kết nối thành công!");
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Firebase Connected!");
            delay(2000);
        } else {
            Serial.println("Lỗi kết nối Firebase: " + firebaseData.errorReason());
            lcdManager.displayError("Firebase Failed!");
        }
    }
    
    void syncInitialState() {
        bool openSwitch = digitalRead(LIMIT_OPEN) == LOW;
        bool closeSwitch = digitalRead(LIMIT_CLOSE) == LOW;
        
        Serial.printf("Initial Limit Switch Status - OPEN: %s, CLOSE: %s\n",
                     openSwitch ? "ACTIVE (LOW)" : "INACTIVE (HIGH)",
                     closeSwitch ? "ACTIVE (LOW)" : "INACTIVE (HIGH)");
        
        if (openSwitch && !closeSwitch) {
            gateIsOpen = true;
            gateCurrentStatus = "open";
            updateGateStatus("open");
            Serial.println("Initial state: Gate is OPEN");
        } else if (closeSwitch && !openSwitch) {
            gateIsOpen = false;
            gateCurrentStatus = "closed";
            updateGateStatus("closed");
            Serial.println("Initial state: Gate is CLOSED");
        } else if (openSwitch && closeSwitch) {
            gateCurrentStatus = "error";
            updateGateStatus("error");
            Serial.println("ERROR: Both limit switches are active!");
            lcdManager.displayError("Limit Switch Error!");
            soundManager.beepFail();
        } else {
            gateCurrentStatus = "unknown";
            updateGateStatus("unknown");
            Serial.println("Initial state: Gate position UNKNOWN (between limits)");
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            if (Firebase.getInt(firebaseData, "/alerts/count")) {
                if (firebaseData.dataType() == "int") {
                    failCount = firebaseData.intData();
                    Serial.printf("Đồng bộ failCount: %d\n", failCount);
                }
            }
            
            if (Firebase.getBool(firebaseData, "/config/faceRecognitionEnabled")) {
                if (firebaseData.dataType() == "boolean") {
                    faceRecognitionEnabled = firebaseData.boolData();
                    Serial.printf("Face Recognition enabled: %s\n", faceRecognitionEnabled ? "true" : "false");
                }
            }
            
            // 🆕 NEW: Sync auto light control setting
            if (Firebase.getBool(firebaseData, "/config/autoLightControlEnabled")) {
                if (firebaseData.dataType() == "boolean") {
                    autoLightControlEnabled = firebaseData.boolData();
                    Serial.printf("Auto Light Control enabled: %s\n", autoLightControlEnabled ? "true" : "false");
                }
            } else {
                // Set default value if not exists
                Firebase.setBool(firebaseData, "/config/autoLightControlEnabled", true);
                autoLightControlEnabled = true;
            }
            
            if (Firebase.getInt(firebaseData, "/config/motorSpeed")) {
                if (firebaseData.dataType() == "int") {
                    int newSpeed = firebaseData.intData();
                    if (newSpeed >= 0 && newSpeed <= 255) {
                        motorController.setSpeed(newSpeed);
                        Serial.printf("Motor speed synced from Firebase: %d\n", newSpeed);
                    }
                }
            }
        }
        
        Serial.printf("Trạng thái ban đầu - Cửa: %s, 2FA: %s, AutoLight: %s, Motor Speed: %d\n", 
                      gateCurrentStatus.c_str(), 
                      faceRecognitionEnabled ? "enabled" : "disabled",
                      autoLightControlEnabled ? "enabled" : "disabled",
                      motorController.getCurrentSpeed());
    }
    
    void checkFirebaseCommands() {
        if (WiFi.status() != WL_CONNECTED) return;
        
        if (Firebase.getString(firebaseData, "/gate/command")) {
            if (firebaseData.dataType() == "string") {
                String command = firebaseData.stringData();
                if (command == "open" && !gateIsOpen && !motorRunning) {
                    Serial.println("Nhận lệnh mở cửa từ Firebase");
                    openGate();
                    Firebase.setString(firebaseData, "/gate/command", "");
                } else if (command == "close" && gateIsOpen && !motorRunning) {
                    Serial.println("Nhận lệnh đóng cửa từ Firebase");
                    closeGate();
                    Firebase.setString(firebaseData, "/gate/command", "");
                }
            }
        }
        
        if (Firebase.getBool(firebaseData, "/scanRequest/request")) {
            if (firebaseData.dataType() == "boolean") {
                bool requestStatus = firebaseData.boolData();
                if (requestStatus && !scanRequested) {
                    scanRequested = true;
                    lastScanTime = millis();
                    Serial.println("Bắt đầu chế độ quét thẻ từ Firebase...");
                }
            }
        }
        
        if (Firebase.getBool(firebaseData, "/config/faceRecognitionEnabled")) {
            if (firebaseData.dataType() == "boolean") {
                bool newSetting = firebaseData.boolData();
                if (newSetting != faceRecognitionEnabled) {
                    faceRecognitionEnabled = newSetting;
                    Serial.printf("Face Recognition setting changed: %s\n", 
                                  faceRecognitionEnabled ? "enabled" : "disabled");
                }
            }
        }
        
        // 🆕 NEW: Check auto light control setting
        if (Firebase.getBool(firebaseData, "/config/autoLightControlEnabled")) {
            if (firebaseData.dataType() == "boolean") {
                bool newSetting = firebaseData.boolData();
                if (newSetting != autoLightControlEnabled) {
                    autoLightControlEnabled = newSetting;
                    Serial.printf("Auto Light Control setting changed: %s\n", 
                                  autoLightControlEnabled ? "enabled" : "disabled");
                }
            }
        }
        
        if (Firebase.getInt(firebaseData, "/config/motorSpeed")) {
            if (firebaseData.dataType() == "int") {
                int newSpeed = firebaseData.intData();
                if (newSpeed >= 0 && newSpeed <= 255 && newSpeed != motorController.getCurrentSpeed()) {
                    motorController.setSpeed(newSpeed);
                    Serial.printf("Motor speed changed from Firebase: %d\n", newSpeed);
                }
            }
        }
    }
    
    void handleRFIDScan() {
        if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
            return;
        }
        
        if (millis() - lastCardScanTime < CARD_SCAN_COOLDOWN) {
            rfid.PICC_HaltA();
            rfid.PCD_StopCrypto1();
            return;
        }
        
        String uid = "";
        for (byte i = 0; i < rfid.uid.size; i++) {
            uid += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
            uid += String(rfid.uid.uidByte[i], HEX);
        }
        uid.toUpperCase();
        
        Serial.printf("Thẻ được quét: %s\n", uid.c_str());
        lastCardScanTime = millis();
        soundManager.beepInfo();
        
        if (scanRequested) {
            if (WiFi.status() == WL_CONNECTED) {
                Firebase.setString(firebaseData, "/lastScanned/uid", uid);
                Firebase.setBool(firebaseData, "/scanRequest/request", false);
            }
            scanRequested = false;
            Serial.println("UID đã được ghi vào Firebase");
        } else {
            if (currentAuthState != AUTH_IDLE) {
                Serial.println("Authentication already in progress, ignoring card");
                lcdManager.displayError("Auth in progress!");
                soundManager.beepFail();
            } else {
                checkCardPermission(uid);
            }
        }
        
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
        lastUID = uid;
    }
    
    void checkCardPermission(String uid) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Không có kết nối WiFi để kiểm tra thẻ");
            handleInvalidCard(uid);
            return;
        }
        
        String cardPath = "/cards/" + uid + "/name";
        
        if (Firebase.getString(firebaseData, cardPath)) {
            if (firebaseData.dataType() == "string" && firebaseData.stringData().length() > 0) {
                String userName = firebaseData.stringData();
                Serial.printf("Thẻ hợp lệ: %s (User: %s)\n", uid.c_str(), userName.c_str());
                
                lcdManager.displayCardInfo(uid, userName, true);
                soundManager.beepSuccess();
                delay(2000);
                
                pendingUID = uid;
                pendingUserName = userName;
                currentAuthState = AUTH_RFID_SUCCESS;
                authStartTime = millis();
                
                logEvent("rfid_success", uid);
                failCount = 0;
                updateFailCount();
                
                Serial.println("RFID authentication success - proceeding to face recognition...");
            } else {
                handleInvalidCard(uid);
            }
        } else {
            Serial.printf("Lỗi kiểm tra thẻ hoặc thẻ không tồn tại: %s\n", uid.c_str());
            handleInvalidCard(uid);
        }
    }
    
    void handleInvalidCard(String uid) {
        Serial.printf("Thẻ không hợp lệ: %s\n", uid.c_str());
        
        lcdManager.displayCardInfo(uid, "", false);
        soundManager.beepFail();
        delay(2000);
        
        failCount++;
        updateFailCount();
        
        logEvent("rfid_fail", uid);
        
        if (failCount >= 5) {
            Serial.println("CẢNH BÁO: Quá nhiều lần quét thẻ thất bại!");
            lcdManager.displayAlert("SECURITY ALERT!");
            logEvent("alert_max_fail", "");
            blinkStatusLED(5);
            soundManager.beepFail();
        }
        
        resetAuthState();
    }
    
    void updateFailCount() {
        if (WiFi.status() == WL_CONNECTED) {
            Firebase.setInt(firebaseData, "/alerts/count", failCount);
        }
    }
    
    void handleScanTimeout() {
        if (scanRequested && (millis() - lastScanTime > SCAN_TIMEOUT)) {
            Serial.println("Timeout quét thẻ, hủy yêu cầu...");
            if (WiFi.status() == WL_CONNECTED) {
                Firebase.setBool(firebaseData, "/scanRequest/request", false);
            }
            scanRequested = false;
        }
    }
    
    void openGate() {
        if (motorRunning) {
            Serial.println("Motor đang chạy, không thể mở cửa");
            return;
        }
        
        Serial.printf("Bắt đầu mở cửa với tốc độ %d...\n", motorController.getCurrentSpeed());
        motorRunning = true;
        targetPosition = "open";
        motorStartTime = millis();
        
        gateCurrentStatus = "opening";
        updateGateStatus("opening");
        
        motorController.startOpening();
        
        logEvent("gate_opening", "");
        Serial.println("Motor đã khởi động - hướng mở cửa");
    }
    
    void closeGate() {
        if (motorRunning) {
            Serial.println("Motor đang chạy, không thể đóng cửa");
            return;
        }
        
        Serial.printf("Bắt đầu đóng cửa với tốc độ %d...\n", motorController.getCurrentSpeed());
        motorRunning = true;
        targetPosition = "close";
        motorStartTime = millis();
        
        gateCurrentStatus = "closing";
        updateGateStatus("closing");
        
        motorController.startClosing();
        
        logEvent("gate_closing", "");
        Serial.println("Motor đã khởi động - hướng đóng cửa");
    }
    
    void checkMotorTimeout() {
        if (motorRunning && (millis() - motorStartTime > MOTOR_TIMEOUT)) {
            Serial.println("CẢNH BÁO: Motor timeout, dừng khẩn cấp!");
            motorController.stopMotor();
            motorRunning = false;
            gateCurrentStatus = "error";
            updateGateStatus("error");
            lcdManager.displayError("Motor Timeout!");
            soundManager.beepFail();
            logEvent("motor_timeout", "");
        }
    }
    
    void updateGateStatus(String status) {
        gateCurrentStatus = status;
        if (WiFi.status() == WL_CONNECTED) {
            Firebase.setString(firebaseData, "/gate/status", status);
        }
        Serial.printf("Trạng thái cửa: %s\n", status.c_str());
    }
    
    void updateSystemStatus() {
        if (WiFi.status() != WL_CONNECTED) return;
        
        FirebaseJson systemStatus;
        systemStatus.set("online", true);
        systemStatus.set("lastSeen", millis() / 1000);
        systemStatus.set("freeHeap", ESP.getFreeHeap());
        systemStatus.set("uptime", millis() / 1000);
        systemStatus.set("wifiRSSI", WiFi.RSSI());
        systemStatus.set("gateStatus", gateCurrentStatus);
        systemStatus.set("motorRunning", motorRunning);
        systemStatus.set("motorSpeed", motorController.getCurrentSpeed());
        systemStatus.set("motorSpeedPercent", motorController.getSpeedPercentage());
        systemStatus.set("limitOpenActive", digitalRead(LIMIT_OPEN) == LOW);
        systemStatus.set("limitCloseActive", digitalRead(LIMIT_CLOSE) == LOW);
        systemStatus.set("pirMotion", pirMotionDetected);
        systemStatus.set("failCount", failCount);
        systemStatus.set("lastUser", lastUserName);
        systemStatus.set("authState", getAuthStateString());
        systemStatus.set("faceEnabled", faceRecognitionEnabled);
        systemStatus.set("faceAttempts", faceAttempts);
        systemStatus.set("faceConfidence", lastFaceConfidence);
        systemStatus.set("autoLightEnabled", autoLightControlEnabled); // 🆕 NEW
        
        Firebase.setJSON(firebaseData, "/system/gate", systemStatus);
    }
    
    String getAuthStateString() {
        switch (currentAuthState) {
            case AUTH_IDLE: return "idle";
            case AUTH_RFID_PENDING: return "rfid_pending";
            case AUTH_RFID_SUCCESS: return "rfid_success";
            case AUTH_FACE_PENDING: return "face_pending";
            case AUTH_FACE_SUCCESS: return "face_success";
            case AUTH_COMPLETED: return "completed";
            case AUTH_FAILED: return "failed";
            default: return "unknown";
        }
    }
    
    void logEvent(String type, String uid) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.printf("Offline log: %s - %s\n", type.c_str(), uid.c_str());
            return;
        }
        
        DateTime now = rtc.now();
        long timestamp = now.unixtime();
        
        String logId = String(timestamp) + "_gate_" + String(random(1000, 9999));
        String logPath = "/history/" + logId;
        
        FirebaseJson json;
        json.set("type", type);
        json.set("timestamp", timestamp);
        json.set("device", "ESP32_GATE_2FA_AUTO_LIGHTS");
        json.set("authState", getAuthStateString());
        json.set("faceEnabled", faceRecognitionEnabled);
        json.set("autoLightEnabled", autoLightControlEnabled); // 🆕 NEW
        json.set("motorSpeed", motorController.getCurrentSpeed());
        
        if (uid.length() > 0) {
            json.set("uid", uid);
        }
        
        if (type.indexOf("face") >= 0) {
            json.set("faceAttempts", faceAttempts);
            json.set("faceConfidence", lastFaceConfidence);
        }
        
        if (Firebase.setJSON(firebaseData, logPath, json)) {
            Serial.printf("Log ghi thành công: %s\n", type.c_str());
        } else {
            Serial.printf("Lỗi ghi log: %s\n", firebaseData.errorReason().c_str());
        }
    }

public:
    void setMotorSpeed(int speed) {
        motorController.setSpeed(speed);
        
        if (WiFi.status() == WL_CONNECTED) {
            Firebase.setInt(firebaseData, "/config/motorSpeed", speed);
        }
    }
    
    int getMotorSpeed() {
        return motorController.getCurrentSpeed();
    }
    
    // 🆕 NEW: Auto light control methods
    void setAutoLightControl(bool enabled) {
        autoLightControlEnabled = enabled;
        
        if (WiFi.status() == WL_CONNECTED) {
            Firebase.setBool(firebaseData, "/config/autoLightControlEnabled", enabled);
        }
        
        Serial.printf("Auto Light Control %s\n", enabled ? "enabled" : "disabled");
    }
    
    bool getAutoLightControlEnabled() {
        return autoLightControlEnabled;
    }
};

// =============================================================================
// KHỞI TẠO OBJECT VÀ MAIN FUNCTIONS
// =============================================================================
EnhancedSmartGate smartGate;

void setup() {
    smartGate.begin();
    // ----- OTA Setup -----
    ArduinoOTA.setHostname("doorsystems");
    ArduinoOTA.setPassword("ota_ecohome2025");
    ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
    ArduinoOTA.onEnd([]()   { Serial.println("OTA End\n"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("OTA Progress: %u%%\r", (p/(t/100)));
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("OTA Error[%u]: ", e);
        if      (e==OTA_AUTH_ERROR)   Serial.println("Auth Failed");
        else if (e==OTA_BEGIN_ERROR)  Serial.println("Begin Failed");
        else if (e==OTA_CONNECT_ERROR)Serial.println("Connect Failed");
        else if (e==OTA_RECEIVE_ERROR)Serial.println("Receive Failed");
        else if (e==OTA_END_ERROR)    Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
}

void loop() {
    ArduinoOTA.handle();
    smartGate.loop();
}
