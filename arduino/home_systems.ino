/*************************************************************
  Smart Home System with Solar Power Management + Firebase
  INSTANT RESPONSE VERSION - ENHANCED BATTERY SOC SYSTEM
  🔧 FIXED: ESP32 SD library conflict resolution
  🔧 FIXED: Firebase ESP32 Client v4.4.17 compatibility
  🔧 NEW: Advanced 4S Li-ion SoC calculation with dual methods
  🔧 NEW: Voltage curve + Coulomb counting + Time estimation
  
  Components:
  - Solar Panel 18V-15W with MPPT CN3722
  - LiPo 4x18650 4800mAh with BMS (9V-16.8V, 12C discharge)
  - 12V Adapter backup with IRF4905 switching
  - 3x INA219 for power monitoring (Solar->LiPo, LiPo->System, Adapter->System)
  - DHT22 for temperature/humidity
  - MQ-2 for gas detection
  - Light sensor (digital out)
  - 8 Relays (4 lights, 3 fans, 1 auto light)
  - Power MUX control (3 gate pins)
 *************************************************************/

// 🔧 CRITICAL: Disable SD filesystem support to prevent conflict
#define FIREBASE_DISABLE_SD_FILESYSTEM

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// 🔧 FIXED: Use DHT22 library v1.0.7 by dvarrel  
#include <DHT22.h>

// 🔧 FIXED: Use Firebase ESP32 Client v4.4.17 (Mobizt)
#include <FirebaseESP32.h>
#include <ArduinoOTA.h>

// 🔧 FIXED: Firebase configuration
const char* ssid = "THIEN NHAN";
const char* password = "13022021";

#define FIREBASE_HOST "smart-home-b7a03-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyAO-6fXi3A_gLZz_uf9JKpQUOuxLfu6r1I"

// 🔧 NEW: Power MUX Gate Pins
#define GATE_SOLAR_PIN 5
#define GATE_BATTERY_PIN 18
#define GATE_ADAPTER_PIN 23

// Pin definitions cho ESP32
#define DHT_PIN             4
#define MQ2_PIN             36
#define LIGHT_SENSOR_PIN    34
#define FAN1_PIN            25
#define FAN2_PIN            26
#define FAN3_PIN            27
#define LIGHT1_PIN          12
#define LIGHT2_PIN          13
#define LIGHT3_PIN          14
#define LIGHT4_PIN          19
#define AUTO_LIGHT_PIN      33
#define SDA_PIN             21
#define SCL_PIN             22

// INA219 addresses
#define INA219_SOLAR_ADDR    0x40
#define INA219_BATTERY_ADDR  0x41
#define INA219_ADAPTER_ADDR  0x44

// 🔧 OPTIMIZED: INA219 Shunt Resistor Configuration
#define SHUNT_RESISTOR_VALUE 0.01  // 0.01 ohm shunt resistor

// 🔋 NEW: Enhanced Battery SoC Constants
#define BATTERY_CAPACITY_MAH      4800    // 4x18650 1200mAh each = 4800mAh total
#define BATTERY_NOMINAL_VOLTAGE   14.8    // 4S Li-ion nominal (3.7V * 4)
#define BATTERY_MIN_VOLTAGE       12.0    // 4S Li-ion minimum (3.0V * 4)
#define BATTERY_MAX_VOLTAGE       16.8    // 4S Li-ion maximum (4.2V * 4)
#define SOC_VOLTAGE_CURVE_POINTS  12      // Number of voltage curve points (FIXED: was 11, now 12)

// Gas sensor constants
#define GAS_WARMUP_TIME     120000  // 2 minutes warmup
#define GAS_CALIBRATION_TIME 30000  // 30 seconds calibration
#define GAS_SAMPLES         10      // Number of samples for baseline

// 🔧 OPTIMIZED: Power thresholds with better precision
#define MIN_POWER_THRESHOLD_MW    50    // 0.05W minimum (reduced for better sensitivity)
#define DISPLAY_POWER_THRESHOLD_MW 500  // 0.5W minimum to display energy flow
#define MAX_POWER_DISPLAY_MW      20000 // 20W maximum power for display (increased from 15W)

// 🔧 FIXED: Firebase objects (Mobizt v4.4.17)
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// 🔧 FIXED: DHT22 sensor object (dvarrel library)
DHT22 dht22(DHT_PIN);

// ========== BASE CLASSES ==========

class Device {
protected:
    int pin;
    bool state;
    String name;
    
public:
    Device(int devicePin, String deviceName) : pin(devicePin), state(false), name(deviceName) {}
    
    virtual void init() {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    
    virtual void setState(bool newState) {
        Serial.printf("⚡ INSTANT: Setting %s to %s (Pin %d)\n", name.c_str(), newState ? "HIGH" : "LOW", pin);
        
        state = newState;
        digitalWrite(pin, state ? HIGH : LOW);
        
        // Verify pin state immediately
        int actualPinState = digitalRead(pin);
        Serial.printf("✅ Pin %d actual state: %s (Response time: instant)\n", pin, actualPinState ? "HIGH" : "LOW");
        
        // Update Firebase with high priority (non-blocking)
        String path = "/devices/" + name + "/state";
        if (Firebase.ready()) {
            if (Firebase.setBool(fbdo, path.c_str(), state)) {
                Serial.printf("📡 Firebase updated instantly: %s = %s\n", path.c_str(), state ? "true" : "false");
            } else {
                Serial.printf("⚠️ Firebase update failed: %s\n", fbdo.errorReason().c_str());
            }
        }
    }
    
    virtual bool getState() const { return state; }
    virtual String getName() const { return name; }
};

class Sensor {
protected:
    int pin;
    String name;
    
public:
    Sensor(int sensorPin, String sensorName) : pin(sensorPin), name(sensorName) {}
    virtual void init() = 0;
    virtual void read() = 0;
    virtual String getName() const { return name; }
};

// ========== ENHANCED POWER MANAGEMENT CLASS ==========

class PowerManager {
private:
    Adafruit_INA219 ina219_solar;
    Adafruit_INA219 ina219_battery;
    Adafruit_INA219 ina219_adapter;
    
    struct PowerData {
        float voltage = 0;
        float current = 0;
        float power = 0;
        float energy = 0; // Accumulated energy in Wh
        unsigned long lastUpdate = 0;
        bool isValid = false;
        float shuntVoltage = 0; // For debugging
        float busVoltage = 0;   // For debugging
    };
    
    // 🔋 NEW: Enhanced Battery State Structure
    struct BatteryState {
        float voltage = 0;
        float current = 0;
        float power = 0;
        float soc_voltage = 0;      // SoC based on voltage curve
        float soc_coulomb = 0;      // SoC based on coulomb counting
        float soc_combined = 0;     // Combined SoC estimate
        float remaining_mah = 0;    // Estimated remaining capacity
        float time_remaining = 0;   // Estimated time remaining (hours)
        bool is_charging = false;
        bool is_discharging = false;
        bool is_valid = false;
        unsigned long last_update = 0;
        float accumulated_mah = 0;  // Coulomb counting accumulator
    };
    
    PowerData solarData, batteryData, adapterData;
    BatteryState batteryState;
    int currentPowerSource = 0;
    float totalSystemPower = 0;
    
    // 🔋 NEW: Voltage-SoC lookup table for 4S Li-ion (18650) - FIXED: Now has 12 points
    const float voltage_soc_curve[SOC_VOLTAGE_CURVE_POINTS][2] = {
        {12.0, 0},    // 0% - 3.0V per cell
        {12.4, 5},    // 5% - 3.1V per cell
        {12.8, 10},   // 10% - 3.2V per cell
        {13.2, 20},   // 20% - 3.3V per cell
        {13.6, 30},   // 30% - 3.4V per cell
        {14.0, 40},   // 40% - 3.5V per cell
        {14.4, 50},   // 50% - 3.6V per cell
        {14.8, 60},   // 60% - 3.7V per cell (nominal)
        {15.2, 75},   // 75% - 3.8V per cell
        {15.6, 85},   // 85% - 3.9V per cell
        {16.0, 95},   // 95% - 4.0V per cell
        {16.8, 100}   // 100% - 4.2V per cell
    };
    
public:
    PowerManager() : 
        ina219_solar(INA219_SOLAR_ADDR),
        ina219_battery(INA219_BATTERY_ADDR),
        ina219_adapter(INA219_ADAPTER_ADDR) {}
    
    void init() {
        Serial.println("🔧 Initializing ENHANCED INA219 sensors with Enhanced Battery SoC...");
        
        // 🔧 OPTIMIZED: Solar Panel calibration (18V-15W, max ~0.85A)
        if (!ina219_solar.begin()) {
            Serial.println("❌ Failed to find INA219 Solar chip");
        } else {
            Serial.println("✅ INA219 Solar initialized");
            ina219_solar.setCalibration_32V_1A();
        }
        
        // 🔧 OPTIMIZED: Battery calibration (16.8V max, high current 57.6A theoretical)
        if (!ina219_battery.begin()) {
            Serial.println("❌ Failed to find INA219 Battery chip");
        } else {
            Serial.println("✅ INA219 Battery initialized");
            applyCustomBatteryCalibration();
        }
        
        // 🔧 OPTIMIZED: Adapter calibration (12V-5A)
        if (!ina219_adapter.begin()) {
            Serial.println("❌ Failed to find INA219 Adapter chip");
        } else {
            Serial.println("✅ INA219 Adapter initialized");
            ina219_adapter.setCalibration_32V_2A();
        }
        
        // Initialize last update times
        unsigned long now = millis();
        solarData.lastUpdate = now;
        batteryData.lastUpdate = now;
        adapterData.lastUpdate = now;
        batteryState.last_update = now;
        
        // 🔋 Initialize battery SoC system
        Serial.println("🔋 Initializing Enhanced Battery SoC System...");
        Serial.printf("🔋 Battery Config: %.0fmAh, %.1fV-%.1fV, %d curve points\n", 
                     BATTERY_CAPACITY_MAH, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, SOC_VOLTAGE_CURVE_POINTS);
        
        Serial.println("🔧 INA219 optimization complete with Enhanced Battery SoC");
    }
    
    void readPowerData() {
        unsigned long currentTime = millis();
        
        // 🔧 OPTIMIZED: Read solar panel data with enhanced validation
        if (readOptimizedSensorData(&ina219_solar, &solarData, currentTime, "Solar")) {
            solarData.isValid = (solarData.busVoltage > 5.0 && solarData.busVoltage < 25.0 && 
                               solarData.power > MIN_POWER_THRESHOLD_MW &&
                               solarData.power <= 15000 && solarData.current >= 0);
            
            if (!solarData.isValid && solarData.busVoltage < 5.0) {
                solarData.power = 0;
                solarData.current = 0;
            }
            
            Serial.printf("🌞 Solar: %.2fW, %.2fV, %.0fmA, Valid: %s\n", 
                         solarData.power/1000.0, solarData.busVoltage, solarData.current, 
                         solarData.isValid ? "YES" : "NO");
        }
        
        // 🔧 ENHANCED: Read battery data with Enhanced SoC calculation
        if (readOptimizedSensorData(&ina219_battery, &batteryData, currentTime, "Battery")) {
            batteryData.isValid = (batteryData.busVoltage >= 9.0 && batteryData.busVoltage <= 17.0 && 
                                 abs(batteryData.current) > 5.0 &&
                                 abs(batteryData.power) <= 60000);
            
            // 🔋 Calculate Enhanced Battery SoC
            calculateEnhancedBatterySoC();
            
            Serial.printf("🔋 Battery: %.2fW, %.2fV, %.0fmA, SoC: %.1f%%, %s, Valid: %s\n", 
                         batteryData.power/1000.0, batteryData.busVoltage, batteryData.current, 
                         batteryState.soc_combined,
                         batteryState.is_charging ? "Charging" : (batteryState.is_discharging ? "Discharging" : "Idle"),
                         batteryData.isValid ? "YES" : "NO");
        }
        
        // 🔧 OPTIMIZED: Read adapter data with enhanced validation
        if (readOptimizedSensorData(&ina219_adapter, &adapterData, currentTime, "Adapter")) {
            adapterData.isValid = (adapterData.busVoltage > 8.0 && adapterData.busVoltage < 16.0 && 
                                 adapterData.current > 2.0 &&
                                 adapterData.power > MIN_POWER_THRESHOLD_MW &&
                                 adapterData.power <= 60000);
            
            if (!adapterData.isValid && adapterData.busVoltage < 8.0) {
                adapterData.power = 0;
                adapterData.current = 0;
            }
            
            Serial.printf("🔌 Adapter: %.2fW, %.2fV, %.0fmA, Valid: %s\n", 
                         adapterData.power/1000.0, adapterData.busVoltage, adapterData.current, 
                         adapterData.isValid ? "YES" : "NO");
        }
        
        // Determine current power source and calculate total system power
        currentPowerSource = determinePowerSource();
        calculateTotalSystemPower();
        
        // 🔧 OPTIMIZED: Enhanced logging with SoC info
        Serial.printf("⚡ Power Status - Source: %d, Solar: %.2fW%s, Battery: %.2fW%s, Adapter: %.2fW%s, SoC: %.1f%%\n",
                     currentPowerSource,
                     solarData.power/1000.0, solarData.isValid ? "✓" : "✗",
                     abs(batteryData.power)/1000.0, batteryData.isValid ? "✓" : "✗", 
                     adapterData.power/1000.0, adapterData.isValid ? "✓" : "✗",
                     batteryState.soc_combined);
    }
    
    void sendDataToFirebase() {
        if (!Firebase.ready()) {
            return;
        }
        
        // 🔧 OPTIMIZED: Enhanced Firebase data with calibration info
        // Solar data
        Firebase.setFloat(fbdo, "/power/solar/voltage", solarData.busVoltage);
        Firebase.setFloat(fbdo, "/power/solar/current", solarData.current);
        Firebase.setFloat(fbdo, "/power/solar/power", solarData.power);
        Firebase.setFloat(fbdo, "/power/solar/energy", solarData.energy);
        Firebase.setFloat(fbdo, "/power/solar/shuntVoltage", solarData.shuntVoltage);
        Firebase.setBool(fbdo, "/power/solar/isValid", solarData.isValid);
        
        // 🔋 ENHANCED: Battery data with detailed SoC information
        Firebase.setFloat(fbdo, "/power/battery/voltage", batteryData.busVoltage);
        Firebase.setFloat(fbdo, "/power/battery/current", batteryData.current);
        Firebase.setFloat(fbdo, "/power/battery/power", batteryData.power);
        Firebase.setFloat(fbdo, "/power/battery/energy", batteryData.energy);
        Firebase.setFloat(fbdo, "/power/battery/shuntVoltage", batteryData.shuntVoltage);
        Firebase.setBool(fbdo, "/power/battery/isValid", batteryData.isValid);
        
        // 🔋 Enhanced Battery SoC data
        Firebase.setFloat(fbdo, "/power/battery/percent", batteryState.soc_combined);
        Firebase.setFloat(fbdo, "/power/battery/soc/combined", batteryState.soc_combined);
        Firebase.setFloat(fbdo, "/power/battery/soc/voltage", batteryState.soc_voltage);
        Firebase.setFloat(fbdo, "/power/battery/soc/coulomb", batteryState.soc_coulomb);
        Firebase.setFloat(fbdo, "/power/battery/remaining_mah", batteryState.remaining_mah);
        Firebase.setFloat(fbdo, "/power/battery/time_remaining", batteryState.time_remaining);
        Firebase.setBool(fbdo, "/power/battery/is_charging", batteryState.is_charging);
        Firebase.setBool(fbdo, "/power/battery/is_discharging", batteryState.is_discharging);
        
        // Battery health and configuration
        Firebase.setFloat(fbdo, "/power/battery/capacity_mah", BATTERY_CAPACITY_MAH);
        Firebase.setFloat(fbdo, "/power/battery/min_voltage", BATTERY_MIN_VOLTAGE);
        Firebase.setFloat(fbdo, "/power/battery/max_voltage", BATTERY_MAX_VOLTAGE);
        Firebase.setFloat(fbdo, "/power/battery/nominal_voltage", BATTERY_NOMINAL_VOLTAGE);
        
        // Adapter data
        Firebase.setFloat(fbdo, "/power/adapter/voltage", adapterData.busVoltage);
        Firebase.setFloat(fbdo, "/power/adapter/current", adapterData.current);
        Firebase.setFloat(fbdo, "/power/adapter/power", adapterData.power);
        Firebase.setFloat(fbdo, "/power/adapter/energy", adapterData.energy);
        Firebase.setFloat(fbdo, "/power/adapter/shuntVoltage", adapterData.shuntVoltage);
        Firebase.setBool(fbdo, "/power/adapter/isValid", adapterData.isValid);
        
        // System info
        Firebase.setInt(fbdo, "/power/currentSource", currentPowerSource);
        Firebase.setFloat(fbdo, "/power/totalPower", totalSystemPower);
        
        // 🔧 OPTIMIZED: Power source percentages with better precision
        float totalActivePower = getTotalActivePower();
        if (totalActivePower > MIN_POWER_THRESHOLD_MW) {
            float solarPercentage = (solarData.isValid && solarData.power > MIN_POWER_THRESHOLD_MW) ? 
                                  (solarData.power / totalActivePower) * 100 : 0;
            float batteryPercentage = (batteryData.isValid && abs(batteryData.power) > MIN_POWER_THRESHOLD_MW) ? 
                                    (abs(batteryData.power) / totalActivePower) * 100 : 0;
            float adapterPercentage = (adapterData.isValid && adapterData.power > MIN_POWER_THRESHOLD_MW) ? 
                                    (adapterData.power / totalActivePower) * 100 : 0;
            
            Firebase.setFloat(fbdo, "/power/percentages/solar", solarPercentage);
            Firebase.setFloat(fbdo, "/power/percentages/battery", batteryPercentage);
            Firebase.setFloat(fbdo, "/power/percentages/adapter", adapterPercentage);
        } else {
            Firebase.setFloat(fbdo, "/power/percentages/solar", 0);
            Firebase.setFloat(fbdo, "/power/percentages/battery", 0);
            Firebase.setFloat(fbdo, "/power/percentages/adapter", 0);
        }
        
        // 🔧 OPTIMIZED: Add calibration and configuration info
        Firebase.setFloat(fbdo, "/power/config/shuntResistor", SHUNT_RESISTOR_VALUE);
        Firebase.setFloat(fbdo, "/power/thresholds/minPower", MIN_POWER_THRESHOLD_MW / 1000.0);
        Firebase.setFloat(fbdo, "/power/thresholds/displayPower", DISPLAY_POWER_THRESHOLD_MW / 1000.0);
        Firebase.setFloat(fbdo, "/power/thresholds/maxPower", MAX_POWER_DISPLAY_MW / 1000.0);
        
        Firebase.setTimestamp(fbdo, "/power/lastUpdate");
    }
    
    float getTotalPower() const { return totalSystemPower; }
    
    // 🔧 OPTIMIZED: Better total active power calculation
    float getTotalActivePower() const {
        float total = 0;
        
        if (solarData.isValid && solarData.power > MIN_POWER_THRESHOLD_MW) {
            total += solarData.power;
        }
        if (batteryData.isValid && abs(batteryData.power) > MIN_POWER_THRESHOLD_MW) {
            total += abs(batteryData.power);
        }
        if (adapterData.isValid && adapterData.power > MIN_POWER_THRESHOLD_MW) {
            total += adapterData.power;
        }
        
        return total;
    }
    
    int getCurrentPowerSource() const { return currentPowerSource; }
    
    PowerData getSolarData() const { return solarData; }
    PowerData getBatteryData() const { return batteryData; }
    PowerData getAdapterData() const { return adapterData; }
    BatteryState getBatteryState() const { return batteryState; }
    
    // 🔧 NEW: Power MUX control function moved to PowerManager class
    void updatePowerMux() {
        // Set all gates to HIGH (disabled) first
        digitalWrite(GATE_SOLAR_PIN, HIGH);
        digitalWrite(GATE_BATTERY_PIN, HIGH);
        digitalWrite(GATE_ADAPTER_PIN, HIGH);
        
        // Enable selected power source (set gate to LOW)
        switch (currentPowerSource) {
            case 1: // Solar
                digitalWrite(GATE_SOLAR_PIN, LOW);
                Serial.println("🔧 Power MUX: Solar gate enabled (LOW)");
                break;
            case 2: // Battery
                digitalWrite(GATE_BATTERY_PIN, LOW);
                Serial.println("🔧 Power MUX: Battery gate enabled (LOW)");
                break;
            case 3: // Adapter
                digitalWrite(GATE_ADAPTER_PIN, LOW);
                Serial.println("🔧 Power MUX: Adapter gate enabled (LOW)");
                break;
            default: // No source
                Serial.println("🔧 Power MUX: All gates disabled (HIGH)");
                break;
        }
        
        // Send MUX status to Firebase
        if (Firebase.ready()) {
            Firebase.setInt(fbdo, "/power/mux/activeSource", currentPowerSource);
            Firebase.setBool(fbdo, "/power/mux/solarGate", digitalRead(GATE_SOLAR_PIN) == LOW);
            Firebase.setBool(fbdo, "/power/mux/batteryGate", digitalRead(GATE_BATTERY_PIN) == LOW);
            Firebase.setBool(fbdo, "/power/mux/adapterGate", digitalRead(GATE_ADAPTER_PIN) == LOW);
            Firebase.setTimestamp(fbdo, "/power/mux/lastUpdate");
        }
    }
    
private:
    // 🔋 NEW: Enhanced Battery SoC Calculation
    void calculateEnhancedBatterySoC() {
        if (!batteryData.isValid) {
            batteryState.soc_combined = 0;
            batteryState.is_valid = false;
            return;
        }
        
        float voltage = batteryData.busVoltage;
        float current = batteryData.current; // mA
        unsigned long currentTime = millis();
        
        // Update battery state
        batteryState.voltage = voltage;
        batteryState.current = current;
        batteryState.power = batteryData.power;
        batteryState.is_charging = (current < -10); // Charging if current < -10mA
        batteryState.is_discharging = (current > 10); // Discharging if current > 10mA
        batteryState.is_valid = true;
        
        // 1. Voltage-based SoC calculation using lookup table
        batteryState.soc_voltage = calculateVoltageSoC(voltage);
        
        // 2. Coulomb counting SoC calculation
        if (batteryState.last_update > 0) {
            float deltaTime = (currentTime - batteryState.last_update) / 3600000.0; // hours
            float deltaMah = (current * deltaTime); // mAh change
            
            batteryState.accumulated_mah -= deltaMah; // Negative current = charging
            
            // Limit accumulator
            batteryState.accumulated_mah = constrain(batteryState.accumulated_mah, 0, BATTERY_CAPACITY_MAH);
            
            batteryState.soc_coulomb = (batteryState.accumulated_mah / BATTERY_CAPACITY_MAH) * 100.0;
        } else {
            // Initialize coulomb counter with voltage-based SoC
            batteryState.accumulated_mah = (batteryState.soc_voltage / 100.0) * BATTERY_CAPACITY_MAH;
            batteryState.soc_coulomb = batteryState.soc_voltage;
        }
        
        // 3. Combined SoC calculation
        // Use voltage-based SoC for extreme values, coulomb counting for middle range
        if (voltage <= 12.5 || voltage >= 16.5) {
            // At extreme voltages, trust voltage more
            batteryState.soc_combined = batteryState.soc_voltage;
        } else {
            // In middle range, combine both methods
            float voltage_weight = 0.3;
            float coulomb_weight = 0.7;
            
            // Adjust weights based on charging/discharging state
            if (batteryState.is_charging) {
                voltage_weight = 0.4; // Trust voltage more during charging
                coulomb_weight = 0.6;
            }
            
            batteryState.soc_combined = (batteryState.soc_voltage * voltage_weight) + 
                                      (batteryState.soc_coulomb * coulomb_weight);
        }
        
        // Ensure SoC is within bounds
        batteryState.soc_combined = constrain(batteryState.soc_combined, 0, 100);
        
        // 4. Calculate remaining capacity and time
        batteryState.remaining_mah = (batteryState.soc_combined / 100.0) * BATTERY_CAPACITY_MAH;
        
        if (batteryState.is_discharging && current > 10) {
            batteryState.time_remaining = batteryState.remaining_mah / current; // hours
        } else {
            batteryState.time_remaining = 0;
        }
        
        batteryState.last_update = currentTime;
    }
    
    // 🔋 NEW: Voltage-based SoC calculation using lookup table
    float calculateVoltageSoC(float voltage) {
        // Clamp voltage to valid range
        if (voltage <= BATTERY_MIN_VOLTAGE) return 0.0;
        if (voltage >= BATTERY_MAX_VOLTAGE) return 100.0;
        
        // Linear interpolation in lookup table
        for (int i = 0; i < SOC_VOLTAGE_CURVE_POINTS - 1; i++) {
            if (voltage >= voltage_soc_curve[i][0] && voltage <= voltage_soc_curve[i + 1][0]) {
                float v1 = voltage_soc_curve[i][0];
                float soc1 = voltage_soc_curve[i][1];
                float v2 = voltage_soc_curve[i + 1][0];
                float soc2 = voltage_soc_curve[i + 1][1];
                
                // Linear interpolation
                float soc = soc1 + ((voltage - v1) / (v2 - v1)) * (soc2 - soc1);
                return soc;
            }
        }
        
        return 0.0; // Fallback
    }
    
    // 🔧 NEW: Optimized sensor data reading with raw values
    bool readOptimizedSensorData(Adafruit_INA219* sensor, PowerData* data, unsigned long currentTime, const char* name) {
        try {
            // Read raw values for better debugging
            data->shuntVoltage = sensor->getShuntVoltage_mV();
            data->busVoltage = sensor->getBusVoltage_V();
            data->current = sensor->getCurrent_mA();
            data->power = sensor->getPower_mW();
            
            // Voltage is always bus voltage (load voltage)
            data->voltage = data->busVoltage;
            
            // 🔧 OPTIMIZED: Power calculation verification using Ohm's law
            if (data->power == 0.0 && data->busVoltage > 0 && abs(data->current) > 0) {
                data->power = (data->busVoltage * data->current);
            }
            
            // 🔧 OPTIMIZED: Enhanced range validation
            if (isnan(data->voltage) || isnan(data->current) || isnan(data->power) || 
                data->voltage < 0 || data->voltage > 30 || 
                abs(data->current) > 30000 || abs(data->power) > MAX_POWER_DISPLAY_MW ||
                abs(data->shuntVoltage) > 320) {
                return false;
            }
            
            updateEnergy(data, currentTime);
            return true;
        } catch (...) {
            Serial.printf("❌ %s: Exception during reading\n", name);
            return false;
        }
    }
    
    // 🔧 NEW: Custom battery calibration for high current
    void applyCustomBatteryCalibration() {
        uint16_t cal_value = 1024;  // Calculated for 10A max with 0.01Ω shunt
        
        // Write calibration register directly
        Wire.beginTransmission(INA219_BATTERY_ADDR);
        Wire.write(0x05);  // Calibration register
        Wire.write((cal_value >> 8) & 0xFF);  // MSB
        Wire.write(cal_value & 0xFF);         // LSB
        Wire.endTransmission();
        
        Serial.printf("🔧 Battery custom calibration applied: cal_value=%d\n", cal_value);
    }
    
    void updateEnergy(PowerData* data, unsigned long currentTime) {
        if (data->lastUpdate > 0) {
            float deltaTime = (currentTime - data->lastUpdate) / 3600000.0;
            data->energy += (abs(data->power) / 1000.0) * deltaTime;
        }
        data->lastUpdate = currentTime;
    }
    
    // 🔧 OPTIMIZED: Enhanced power source determination
    int determinePowerSource() {
        // Priority 1: Adapter (highest priority if available and > 0.05W, voltage > 8V)
        if (adapterData.isValid && adapterData.power > MIN_POWER_THRESHOLD_MW && adapterData.busVoltage > 8.0) {
            return 3; // Adapter
        }
        
        // Priority 2: Solar (clean renewable energy if > 0.05W)
        if (solarData.isValid && solarData.power > MIN_POWER_THRESHOLD_MW) {
            return 1; // Solar
        }
        
        // Priority 3: Battery (backup power if discharge > 0.05W)
        if (batteryData.isValid && batteryData.current > 0 && abs(batteryData.power) > MIN_POWER_THRESHOLD_MW) {
            return 2; // Battery
        }
        
        return 0; // No active source above threshold
    }
    
    void calculateTotalSystemPower() {
        // Calculate total power from primary source
        switch(currentPowerSource) {
            case 1: 
                totalSystemPower = solarData.power / 1000.0; 
                break;
            case 2: 
                totalSystemPower = abs(batteryData.power) / 1000.0; 
                break;
            case 3: 
                totalSystemPower = adapterData.power / 1000.0; 
                break;
            default: 
                totalSystemPower = 0.0; 
                break;
        }
    }
};

// ========== SENSOR CLASSES ==========

class TemperatureHumiditySensor : public Sensor {
private:
    float temperature = 0;
    float humidity = 0;
    
public:
    TemperatureHumiditySensor(int pin) : Sensor(pin, "tempHumid") {}
    
    void init() override {
        Serial.println("DHT22 sensor initialized");
    }
    
    void read() override {
        float h = dht22.getHumidity();
        float t = dht22.getTemperature();
        
        if (!isnan(h) && !isnan(t) && h > 0.0 && t > 0.0 && h <= 100.0 && t >= -40.0 && t <= 80.0) {
            humidity = h;
            temperature = t;
            
            if (Firebase.ready()) {
                Firebase.setFloat(fbdo, "/sensors/temperature", temperature);
                Firebase.setFloat(fbdo, "/sensors/humidity", humidity);
                Firebase.setTimestamp(fbdo, "/sensors/tempHumid/lastUpdate");
            }
        }
    }
    
    float getTemperature() const { return temperature; }
    float getHumidity() const { return humidity; }
};

class GasSensor : public Sensor {
private:
    int gasValue = 0;
    float gasPPM = 0;
    bool alertSent = false;
    unsigned long startTime = 0;
    unsigned long lastReading = 0;
    
    const int CLEAN_AIR_VALUE = 100;
    const float WARNING_THRESHOLD = 200.0;
    const float DANGER_THRESHOLD = 800.0;
    const float MAX_REASONABLE_PPM = 2000.0;
    
    const float RL = 10.0;
    const float RO_CLEAN_AIR = 9.83;
    
public:
    GasSensor(int pin) : Sensor(pin, "gas") {
        startTime = millis();
    }
    
    void init() override {
        pinMode(pin, INPUT);
        Serial.println("MQ-2 gas sensor initialized");
    }
    
    void read() override {
        unsigned long currentTime = millis();
        
        gasValue = analogRead(pin);
        
        if (currentTime - startTime < 20000) {
            gasPPM = 0;
        } else {
            float voltage = (gasValue / 4095.0) * 3.3;
            float Rs = ((3.3 - voltage) / voltage) * RL;
            float ratio = Rs / RO_CLEAN_AIR;
            
            if (ratio > 0.1) {
                gasPPM = 613.9 * pow(ratio, -2.074);
            } else {
                gasPPM = 0;
            }
            
            if (gasValue > CLEAN_AIR_VALUE) {
                float simplePPM = (gasValue - CLEAN_AIR_VALUE) * 2.0;
                
                if (gasPPM > MAX_REASONABLE_PPM || gasPPM < 0) {
                    gasPPM = simplePPM;
                }
            }
            
            if (gasPPM < 0) gasPPM = 0;
            if (gasPPM > MAX_REASONABLE_PPM) gasPPM = MAX_REASONABLE_PPM;
        }
        
        if (Firebase.ready()) {
            Firebase.setInt(fbdo, "/sensors/gas/value", gasValue);
            Firebase.setFloat(fbdo, "/sensors/gas/ppm", gasPPM);
            Firebase.setBool(fbdo, "/sensors/gas/isCalibrated", true);
            Firebase.setTimestamp(fbdo, "/sensors/gas/lastUpdate");
            
            if (gasPPM > DANGER_THRESHOLD && !alertSent) {
                Firebase.setBool(fbdo, "/alerts/gasHigh", true);
                Firebase.setString(fbdo, "/alerts/gasLevel", "DANGER");
                Firebase.setString(fbdo, "/alerts/gasMessage", "NGUY HIỂM! Nồng độ khí gas rất cao: " + String(gasPPM, 0) + " ppm");
                Firebase.setTimestamp(fbdo, "/alerts/gasTime");
                alertSent = true;
            } else if (gasPPM > WARNING_THRESHOLD && gasPPM <= DANGER_THRESHOLD) {
                Firebase.setBool(fbdo, "/alerts/gasHigh", true);
                Firebase.setString(fbdo, "/alerts/gasLevel", "WARNING");
                Firebase.setString(fbdo, "/alerts/gasMessage", "Cảnh báo: Nồng độ khí gas cao: " + String(gasPPM, 0) + " ppm");
                Firebase.setTimestamp(fbdo, "/alerts/gasTime");
            } else if (gasPPM <= WARNING_THRESHOLD * 0.8) {
                if (alertSent) {
                    alertSent = false;
                }
                Firebase.setBool(fbdo, "/alerts/gasHigh", false);
                Firebase.setString(fbdo, "/alerts/gasLevel", "NORMAL");
            }
        }
        
        lastReading = currentTime;
    }
    
    int getGasValue() const { return gasValue; }
    float getGasPPM() const { return gasPPM; }
    bool getIsCalibrated() const { return (millis() - startTime) >= 20000; }
};

class LightSensor : public Sensor {
private:
    int lightValue = 0;
    
public:
    LightSensor(int pin) : Sensor(pin, "light") {}
    
    void init() override {
        pinMode(pin, INPUT);
        Serial.println("Light sensor initialized");
    }
    
    void read() override {
        lightValue = digitalRead(pin);
        
        if (Firebase.ready()) {
            Firebase.setInt(fbdo, "/sensors/light/value", lightValue);
            Firebase.setBool(fbdo, "/sensors/light/isDark", lightValue == HIGH);
            Firebase.setTimestamp(fbdo, "/sensors/light/lastUpdate");
        }
    }
    
    bool isDark() const { return lightValue == HIGH; }
    int getLightValue() const { return lightValue; }
};

// ========== DEVICE CLASSES ==========

class Fan : public Device {
public:
    Fan(int pin, String name) : Device(pin, name) {}
};

class Light : public Device {
public:
    Light(int pin, String name) : Device(pin, name) {}
};

class AutoLight : public Device {
private:
    bool enabled = true;
    
public:
    AutoLight(int pin, String name) : Device(pin, name) {}
    
    void setEnabled(bool enable) {
        enabled = enable;
        if (!enabled) {
            setState(false);
        }
        if (Firebase.ready()) {
            Firebase.setBool(fbdo, ("/devices/" + name + "/enabled").c_str(), enabled);
        }
    }
    
    bool isEnabled() const { return enabled; }
    
    void updateState(bool isDark) {
        if (!enabled) {
            setState(false);
            return;
        }
        setState(isDark);
    }
};

// ========== MAIN SMART HOME CLASS ==========

class SmartHome {
private:
    PowerManager powerManager;
    TemperatureHumiditySensor tempHumidSensor;
    GasSensor gasSensor;
    LightSensor lightSensor;
    
    Fan fans[3] = {
        Fan(FAN1_PIN, "fan1"),
        Fan(FAN2_PIN, "fan2"),
        Fan(FAN3_PIN, "fan3")
    };
    
    Light lights[4] = {
        Light(LIGHT1_PIN, "light1"),
        Light(LIGHT2_PIN, "light2"),
        Light(LIGHT3_PIN, "light3"),
        Light(LIGHT4_PIN, "light4")
    };
    
    AutoLight autoLight;
    
    unsigned long lastSensorRead = 0;
    unsigned long lastPowerRead = 0;
    unsigned long lastFirebaseCheck = 0;
    unsigned long lastAutoLightUpdate = 0;
    unsigned long lastHeartbeat = 0;
    
public:
    SmartHome() : 
        tempHumidSensor(DHT_PIN),
        gasSensor(MQ2_PIN),
        lightSensor(LIGHT_SENSOR_PIN),
        autoLight(AUTO_LIGHT_PIN, "autoLight") {}
    
    void init() {
        Serial.begin(115200);
        Serial.println("\n=== ECO SMART HOME SYSTEM v3.2 - ENHANCED BATTERY SOC SYSTEM ===");
        Serial.printf("🔋 Enhanced Battery: 4S Li-ion %.0fmAh (%.1fV-%.1fV)\n", 
                     BATTERY_CAPACITY_MAH, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE);
        Serial.printf("🔧 Power thresholds: Min %.2fW, Display %.1fW, Max %.0fW\n", 
                     MIN_POWER_THRESHOLD_MW/1000.0, DISPLAY_POWER_THRESHOLD_MW/1000.0, MAX_POWER_DISPLAY_MW/1000.0);
        Serial.printf("🔧 Shunt resistor: %.3fΩ with custom calibrations\n", SHUNT_RESISTOR_VALUE);
        Serial.println("🔋 Dual SoC calculation: Voltage curve + Coulomb counting");
        Serial.println("🔧 Power MUX: 3-gate control for source switching");
        
        Wire.begin(SDA_PIN, SCL_PIN);
        Serial.println("I2C initialized");
        
        // Initialize hardware
        powerManager.init();
        tempHumidSensor.init();
        gasSensor.init();
        lightSensor.init();
        
        for (int i = 0; i < 3; i++) {
            fans[i].init();
            Serial.printf("Fan %d initialized\n", i + 1);
        }
        for (int i = 0; i < 4; i++) {
            lights[i].init();
            Serial.printf("Light %d initialized\n", i + 1);
        }
        autoLight.init();
        Serial.println("Auto light initialized");
        
        // 🔧 NEW: Initialize Power MUX gate pins
        pinMode(GATE_SOLAR_PIN, OUTPUT);
        pinMode(GATE_BATTERY_PIN, OUTPUT);
        pinMode(GATE_ADAPTER_PIN, OUTPUT);
        digitalWrite(GATE_SOLAR_PIN, HIGH);
        digitalWrite(GATE_BATTERY_PIN, HIGH);
        digitalWrite(GATE_ADAPTER_PIN, HIGH);
        Serial.println("🔧 Power MUX gates initialized (all HIGH = disabled)");
        
        // Initialize WiFi
        WiFi.begin(ssid, password);
        Serial.print("Connecting to WiFi");
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            delay(300);
        }
        Serial.println();
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
        
        // 🔧 FIXED: Initialize Firebase với thư viện v4.4.17
        config.host = FIREBASE_HOST;
        config.signer.tokens.legacy_token = FIREBASE_AUTH;
        
        Firebase.begin(&config, &auth);
        Firebase.reconnectWiFi(true);
        
        // Set timeout và size limit
        Firebase.setReadTimeout(fbdo, 1000 * 60);
        Firebase.setwriteSizeLimit(fbdo, "tiny");
        
        Serial.println("Waiting for Firebase connection...");
        unsigned long firebaseTimeout = millis();
        while (!Firebase.ready() && (millis() - firebaseTimeout < 30000)) {
            Serial.print(".");
            delay(1000);
        }
        
        if (Firebase.ready()) {
            Serial.println("\nFirebase connected!");
            Firebase.setString(fbdo, "/system/version", "3.2");
            Firebase.setString(fbdo, "/system/features", "Enhanced Battery SoC + Dual Method Calculation + Power MUX + 4S Li-ion");
            Firebase.setString(fbdo, "/system/powerConfig", "0.01Ω Shunt + Enhanced SoC + Voltage Curve + Coulomb Counting");
            sendInitialStates();
        } else {
            Serial.println("\nFirebase connection failed!");
        }
        
        Serial.println("=== SMART HOME SYSTEM READY - ENHANCED BATTERY SOC SYSTEM ===");
    }
    
    void run() {
        unsigned long currentTime = millis();
        
        // ⚡ CRITICAL: Check Firebase commands every 100ms for INSTANT response
        if (currentTime - lastFirebaseCheck >= 100) {
            checkFirebaseCommands();
            lastFirebaseCheck = currentTime;
        }
        
        // Read sensors every 3 seconds (for gas sensor stability)
        if (currentTime - lastSensorRead >= 3000) {
            readSensors();
            lastSensorRead = currentTime;
        }
        
        // Read power data every 2 seconds
        if (currentTime - lastPowerRead >= 2000) {
            readPowerData();
            // 🔧 FIXED: Call updatePowerMux method from powerManager
            powerManager.updatePowerMux();
            lastPowerRead = currentTime;
        }
        
        // Update auto light every 10 seconds
        if (currentTime - lastAutoLightUpdate >= 10000) {
            updateAutoLight();
            lastAutoLightUpdate = currentTime;
        }
        
        // Send heartbeat every 30 seconds
        if (currentTime - lastHeartbeat >= 30000) {
            sendHeartbeat();
            lastHeartbeat = currentTime;
        }
    }
    
    void readSensors() {
        tempHumidSensor.read();
        gasSensor.read();
        lightSensor.read();
    }
    
    void readPowerData() {
        powerManager.readPowerData();
        powerManager.sendDataToFirebase();
    }
    
    void updateAutoLight() {
        autoLight.updateState(lightSensor.isDark());
    }
    
    void sendHeartbeat() {
        if (Firebase.ready()) {
            Firebase.setString(fbdo, "/system/status", "online");
            Firebase.setFloat(fbdo, "/system/uptime", millis() / 1000.0);
            Firebase.setFloat(fbdo, "/system/freeHeap", ESP.getFreeHeap() / 1024.0);
            Firebase.setString(fbdo, "/system/powerSupport", "Enhanced Battery SoC + Dual Method + Power MUX");
            Firebase.setTimestamp(fbdo, "/system/lastHeartbeat");
            Serial.printf("❤️ Heartbeat sent - Uptime: %.1fs, Free Heap: %.1fKB, SoC: %.1f%%\n", 
                         millis() / 1000.0, ESP.getFreeHeap() / 1024.0, powerManager.getBatteryState().soc_combined);
        }
    }
    
    void sendInitialStates() {
        Serial.println("Sending initial states to Firebase...");
        
        // Send device states
        for (int i = 0; i < 3; i++) {
            Firebase.setBool(fbdo, ("/devices/" + fans[i].getName() + "/state").c_str(), fans[i].getState());
        }
        for (int i = 0; i < 4; i++) {
            Firebase.setBool(fbdo, ("/devices/" + lights[i].getName() + "/state").c_str(), lights[i].getState());
        }
        Firebase.setBool(fbdo, "/devices/autoLight/state", autoLight.getState());
        Firebase.setBool(fbdo, "/devices/autoLight/enabled", autoLight.isEnabled());
        
        // System info
        Firebase.setString(fbdo, "/system/status", "online");
        Firebase.setTimestamp(fbdo, "/system/lastBoot");
        
        Serial.println("Initial states sent successfully");
    }
    
    void checkFirebaseCommands() {
        if (!Firebase.ready()) return;
        
        if (Firebase.get(fbdo, "/commands")) {
            if (fbdo.dataType() == "json") {
                Serial.println("⚡ Processing commands instantly...");
                FirebaseJson json = fbdo.jsonObject();
                
                FirebaseJsonData jsonData;
                size_t len = json.iteratorBegin();
                String key, value = "";
                int type = 0;
                
                for (size_t i = 0; i < len; i++) {
                    json.iteratorGet(i, type, key, value);
                    bool boolValue = (value == "true" || value == "1");
                    
                    Serial.printf("⚡ INSTANT COMMAND: %s = %s\n", key.c_str(), boolValue ? "ON" : "OFF");
                    
                    // Execute commands immediately with no delay
                    if (key == "fan1") fans[0].setState(boolValue);
                    else if (key == "fan2") fans[1].setState(boolValue);
                    else if (key == "fan3") fans[2].setState(boolValue);
                    else if (key == "light1") lights[0].setState(boolValue);
                    else if (key == "light2") lights[1].setState(boolValue);
                    else if (key == "light3") lights[2].setState(boolValue);
                    else if (key == "light4") lights[3].setState(boolValue);
                    else if (key == "autoLight") autoLight.setEnabled(boolValue);
                }
                json.iteratorEnd();
                
                // Clear commands immediately after processing
                Firebase.deleteNode(fbdo, "/commands");
                Serial.println("✅ Commands processed instantly and cleared");
            }
        }
    }
    
    // 🔧 NEW: Expose powerManager for external access if needed
    PowerManager& getPowerManager() { return powerManager; }
};

// ========== GLOBAL INSTANCE ==========
SmartHome smartHome;

// ========== MAIN FUNCTIONS ==========

void setup() {
    smartHome.init();
    // ----- OTA Setup -----
    ArduinoOTA.setHostname("homesystems");
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
    smartHome.run();
    // ⚡ CRITICAL: Minimal delay for instant response
    delay(50);
}
