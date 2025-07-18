#define FIREBASE_DISABLE_SD_FILESYSTEM

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <DHT22.h>
#include <FirebaseESP32.h>
#include <ArduinoOTA.h>

// Firebase configuration
const char* ssid = "   ";
const char* password = "    ";

#define FIREBASE_HOST "smart-home-b7a03-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyAO-6fXi3A_gLZz_uf9JKpQUOuxLfu6r1I"

//  Power MUX Gate Pins
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

//  INA219 Shunt Resistor Configuration
#define SHUNT_RESISTOR_VALUE 0.01  // 0.01 ohm shunt resistor

// 🔋 NEW: Enhanced Battery SoC Constants
#define BATTERY_CAPACITY_MAH      4800    // 4x18650 1200mAh each = 4800mAh total
#define BATTERY_NOMINAL_VOLTAGE   14.8    // 4S Li-ion nominal (3.7V * 4)
#define BATTERY_MIN_VOLTAGE       12.0    // 4S Li-ion minimum (3.0V * 4)
#define BATTERY_MAX_VOLTAGE       16.8    // 4S Li-ion maximum (4.2V * 4)
#define SOC_VOLTAGE_CURVE_POINTS  12      // Number of voltage curve points

// 🆕 NEW: Auto Temperature Control Constants
#define DEFAULT_TEMP_THRESHOLD     28.0    // Default temperature threshold (°C)
#define TEMP_HYSTERESIS           0.5     // ±0.5°C hysteresis
#define TEMP_CHECK_INTERVAL       5000    // Check temperature every 5 seconds
#define TEMP_THRESHOLD_MIN        16.0    // Minimum settable threshold
#define TEMP_THRESHOLD_MAX        40.0    // Maximum settable threshold

// 🆕 NEW: Enhanced Gas Detection Constants
#define GAS_WARMUP_TIME           120000  // 2 minutes warmup
#define GAS_CALIBRATION_TIME      30000   // 30 seconds calibration
#define GAS_SAMPLES              10       // Number of samples for baseline
#define GAS_CHECK_INTERVAL       2000     // Check gas every 2 seconds
#define GAS_PPM_SMOOTHING        5        // Moving average for PPM readings

// 🆕 NEW: Default Gas Thresholds (PPM)
#define DEFAULT_GAS_THRESHOLD_1   200     // Level 1: Warning (turn on 1 fan)
#define DEFAULT_GAS_THRESHOLD_2   500     // Level 2: Alert (turn on 2 fans)
#define DEFAULT_GAS_THRESHOLD_3   800     // Level 3: Danger (turn on 3 fans)

//  Power thresholds with better precision
#define MIN_POWER_THRESHOLD_MW    50    // 0.05W minimum (reduced for better sensitivity)
#define DISPLAY_POWER_THRESHOLD_MW 500  // 0.5W minimum to display energy flow
#define MAX_POWER_DISPLAY_MW      20000 // 20W maximum power for display

// Firebase objects (Mobizt v4.4.17)
FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

//  DHT22 sensor object (dvarrel library)
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
    
    // 🔋 NEW: Voltage-SoC lookup table for 4S Li-ion (18650)
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
    
    // 🔧 NEW: Power MUX control function
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

// 🔧 ENHANCED: Gas Sensor with Dynamic Calibration and Conflict Resolution
class EnhancedGasSensor : public Sensor {
private:
    int gasValue = 0;
    float gasPPM = 0;
    bool alertSent = false;
    unsigned long startTime = 0;
    unsigned long lastReading = 0;
    
    // 🔧 NEW: Dynamic Calibration System
    bool isWarmupComplete = false;
    bool isCalibrationComplete = false;
    float dynamicR0 = 0;
    float calibrationSamples[GAS_SAMPLES];
    int calibrationIndex = 0;
    unsigned long calibrationStartTime = 0;
    
    // Moving average for PPM smoothing
    float ppmHistory[GAS_PPM_SMOOTHING];
    int ppmHistoryIndex = 0;
    bool historyFilled = false;
    
    // Enhanced calibration parameters
    const float R_LOAD = 10.0;        // Load resistance in kΩ
    const float DEFAULT_R0 = 9.83;    // Fallback R0 value
    const float CURVE_A = 574.25;     // Curve fitting parameter A
    const float CURVE_B = -2.222;     // Curve fitting parameter B (slope)
    
    // 3-Level thresholds
    float gasThreshold1 = DEFAULT_GAS_THRESHOLD_1;
    float gasThreshold2 = DEFAULT_GAS_THRESHOLD_2;
    float gasThreshold3 = DEFAULT_GAS_THRESHOLD_3;
    
    int currentGasLevel = 0;
    int lastSentGasLevel = -1; // Track last sent level to avoid spam
    
public:
    EnhancedGasSensor(int pin) : Sensor(pin, "enhancedGas") {
        startTime = millis();
        // Initialize arrays
        for (int i = 0; i < GAS_PPM_SMOOTHING; i++) {
            ppmHistory[i] = 0;
        }
        for (int i = 0; i < GAS_SAMPLES; i++) {
            calibrationSamples[i] = 0;
        }
    }
    
    void init() override {
        pinMode(pin, INPUT);
        loadGasThresholds();
        Serial.println("🔥 Enhanced MQ-2 with Dynamic Calibration initialized");
        Serial.printf("🔥 Calibration: Warmup=%ds, Calibration=%ds, Samples=%d\n", 
                     GAS_WARMUP_TIME/1000, GAS_CALIBRATION_TIME/1000, GAS_SAMPLES);
    }
    
    void read() override {
        unsigned long currentTime = millis();
        
        if (currentTime - lastReading < GAS_CHECK_INTERVAL) {
            return;
        }
        
        gasValue = analogRead(pin);
        
        // 🔧 FIXED: Proper calibration sequence
        if (!isWarmupComplete) {
            if (currentTime - startTime >= GAS_WARMUP_TIME) {
                isWarmupComplete = true;
                calibrationStartTime = currentTime;
                Serial.println("🔥 Gas sensor warmup complete - Starting calibration...");
                
                // Send warmup complete status
                if (Firebase.ready()) {
                    Firebase.setString(fbdo, "/sensors/gas/status", "calibrating");
                }
            } else {
                // During warmup, don't calculate PPM
                gasPPM = 0;
                updateFirebaseBasicData();
                lastReading = currentTime;
                return;
            }
        }
        
        // 🔧 NEW: Dynamic calibration process
        if (isWarmupComplete && !isCalibrationComplete) {
            if (currentTime - calibrationStartTime < GAS_CALIBRATION_TIME) {
                // Collect calibration samples
                if (calibrationIndex < GAS_SAMPLES) {
                    float voltage = (gasValue / 4095.0) * 3.3;
                    float Rs = ((3.3 - voltage) / voltage) * R_LOAD;
                    calibrationSamples[calibrationIndex] = Rs;
                    calibrationIndex++;
                    
                    Serial.printf("🔥 Calibration sample %d/%d: Rs=%.2f kΩ (ADC=%d)\n", 
                                 calibrationIndex, GAS_SAMPLES, Rs, gasValue);
                }
                
                gasPPM = 0; // Don't calculate PPM during calibration
                updateFirebaseBasicData();
                lastReading = currentTime;
                return;
            } else {
                // Calibration complete - calculate R0
                completeDynamicCalibration();
            }
        }
        
        // 🔧 FIXED: Only calculate PPM after calibration complete
        if (isCalibrationComplete) {
            gasPPM = calculateEnhancedPPM(gasValue);
            
            // Apply moving average smoothing
            ppmHistory[ppmHistoryIndex] = gasPPM;
            ppmHistoryIndex = (ppmHistoryIndex + 1) % GAS_PPM_SMOOTHING;
            if (ppmHistoryIndex == 0) historyFilled = true;
            
            if (historyFilled) {
                float sum = 0;
                for (int i = 0; i < GAS_PPM_SMOOTHING; i++) {
                    sum += ppmHistory[i];
                }
                gasPPM = sum / GAS_PPM_SMOOTHING;
            }
            
            // Check thresholds only after calibration
            checkGasThresholds();
        } else {
            gasPPM = 0;
        }
        
        updateFirebaseBasicData();
        lastReading = currentTime;
    }
    
    bool isCalibrated() const { 
        return isWarmupComplete && isCalibrationComplete; 
    }
    
    float getGasPPM() const { return gasPPM; }
    int getGasLevel() const { return currentGasLevel; }
    
    void updateGasThresholds(float threshold1, float threshold2, float threshold3) {
        gasThreshold1 = constrain(threshold1, 50, 2000);
        gasThreshold2 = constrain(threshold2, 100, 2000);
        gasThreshold3 = constrain(threshold3, 200, 2000);
        
        // Ensure ascending order
        if (gasThreshold2 <= gasThreshold1) gasThreshold2 = gasThreshold1 + 50;
        if (gasThreshold3 <= gasThreshold2) gasThreshold3 = gasThreshold2 + 100;
        
        Serial.printf("🔥 Gas thresholds updated: %.0f/%.0f/%.0f PPM\n", 
                     gasThreshold1, gasThreshold2, gasThreshold3);
    }
    
private:
    void completeDynamicCalibration() {
        // Calculate average R0 from calibration samples
        float totalR0 = 0;
        int validSamples = 0;
        
        for (int i = 0; i < GAS_SAMPLES; i++) {
            if (calibrationSamples[i] > 0 && calibrationSamples[i] < 100) {
                totalR0 += calibrationSamples[i];
                validSamples++;
            }
        }
        
        if (validSamples >= GAS_SAMPLES/2) {
            dynamicR0 = totalR0 / validSamples;
            isCalibrationComplete = true;
            
            Serial.printf("🔥 ✅ Dynamic calibration complete! R0 = %.2f kΩ (from %d samples)\n", 
                         dynamicR0, validSamples);
            
            // Send calibration complete status
            if (Firebase.ready()) {
                Firebase.setString(fbdo, "/sensors/gas/status", "ready");
                Firebase.setFloat(fbdo, "/sensors/gas/calibration/r0", dynamicR0);
                Firebase.setInt(fbdo, "/sensors/gas/calibration/samples", validSamples);
                Firebase.setTimestamp(fbdo, "/sensors/gas/calibration/completedAt");
            }
        } else {
            // Fallback to default R0
            dynamicR0 = DEFAULT_R0;
            isCalibrationComplete = true;
            
            Serial.printf("🔥 ⚠️ Calibration failed - using default R0 = %.2f kΩ\n", dynamicR0);
            
            if (Firebase.ready()) {
                Firebase.setString(fbdo, "/sensors/gas/status", "fallback");
                Firebase.setFloat(fbdo, "/sensors/gas/calibration/r0", dynamicR0);
            }
        }
    }
    
    float calculateEnhancedPPM(int rawValue) {
        // Convert ADC to voltage
        float voltage = (rawValue / 4095.0) * 3.3;
        
        // Calculate sensor resistance
        float Rs = ((3.3 - voltage) / voltage) * R_LOAD;
        
        // Calculate Rs/R0 ratio using dynamic R0
        float ratio = Rs / dynamicR0;
        
        // 🔧 FIXED: Better fallback logic
        if (ratio > 0.3 && ratio < 10.0) {
            // Use curve fitting for valid range
            float ppm = CURVE_A * pow(ratio, CURVE_B);
            return constrain(ppm, 0, 10000);
        } else if (ratio >= 10.0) {
            // Clean air condition
            return 0;
        } else {
            // Low ratio - possible high concentration or sensor issue
            return constrain((rawValue - 1000) * 1.5, 0, 5000);
        }
    }
    
    void checkGasThresholds() {
        int newGasLevel = 0;
        
        if (gasPPM >= gasThreshold3) {
            newGasLevel = 3; // DANGER
        } else if (gasPPM >= gasThreshold2) {
            newGasLevel = 2; // ALERT
        } else if (gasPPM >= gasThreshold1) {
            newGasLevel = 1; // WARNING
        }
        
        // Only send commands if level actually changed
        if (newGasLevel != currentGasLevel) {
            currentGasLevel = newGasLevel;
            
            String levelNames[] = {"NORMAL", "WARNING", "ALERT", "DANGER"};
            Serial.printf("🔥 Gas level changed: %s (%.0f PPM)\n", 
                         levelNames[newGasLevel].c_str(), gasPPM);
            
            // Update Firebase alerts
            updateFirebaseAlerts(newGasLevel);
            
            // Trigger auto fan control with conflict resolution
            triggerAutoFanControlSafe(newGasLevel);
        }
    }
    
    void updateFirebaseAlerts(int gasLevel) {
        if (!Firebase.ready()) return;
        
        String levelNames[] = {"NORMAL", "WARNING", "WARNING", "DANGER"};
        String alertLevels[] = {"NORMAL", "WARNING", "WARNING", "DANGER"};
        
        Firebase.setBool(fbdo, "/alerts/gasHigh", gasLevel > 0);
        Firebase.setString(fbdo, "/alerts/gasLevel", alertLevels[gasLevel]);
        Firebase.setString(fbdo, "/alerts/gasMessage", 
            String("Gas ") + levelNames[gasLevel] + ": " + String(gasPPM, 0) + " ppm");
        Firebase.setTimestamp(fbdo, "/alerts/gasTime");
        
        Firebase.setInt(fbdo, "/status/gasLevel", gasLevel);
        Firebase.setString(fbdo, "/status/gasLevelName", levelNames[gasLevel]);
    }
    
    void updateFirebaseBasicData() {
        if (!Firebase.ready()) return;
        
        Firebase.setInt(fbdo, "/sensors/gas/value", gasValue);
        Firebase.setFloat(fbdo, "/sensors/gas/ppm", gasPPM);
        Firebase.setBool(fbdo, "/sensors/gas/isCalibrated", isCalibrated());
        Firebase.setTimestamp(fbdo, "/sensors/gas/lastUpdate");
        
        Firebase.setInt(fbdo, "/sensors/gas/level", currentGasLevel);
        Firebase.setFloat(fbdo, "/sensors/gas/thresholds/warning", gasThreshold1);
        Firebase.setFloat(fbdo, "/sensors/gas/thresholds/alert", gasThreshold2);
        Firebase.setFloat(fbdo, "/sensors/gas/thresholds/danger", gasThreshold3);
    }
    
    void loadGasThresholds() {
        if (!Firebase.ready()) return;
        
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/warning")) {
            gasThreshold1 = fbdo.floatData();
        }
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/alert")) {
            gasThreshold2 = fbdo.floatData();
        }
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/danger")) {
            gasThreshold3 = fbdo.floatData();
        }
    }
    
    // 🔧 NEW: Safe auto fan control with conflict resolution
    void triggerAutoFanControlSafe(int gasLevel) {
        if (!Firebase.ready()) return;
        
        Serial.printf("🔥 Checking auto fan control for gas level %d\n", gasLevel);
        
        // Check current fan states before sending auto commands
        bool fan1Current = false, fan2Current = false, fan3Current = false;
        
        if (Firebase.getBool(fbdo, "/devices/fan1/state")) {
            fan1Current = fbdo.boolData();
        }
        if (Firebase.getBool(fbdo, "/devices/fan2/state")) {
            fan2Current = fbdo.boolData();
        }
        if (Firebase.getBool(fbdo, "/devices/fan3/state")) {
            fan3Current = fbdo.boolData();
        }
        
        // 🔧 FIXED: Only send auto commands if state needs to change
        switch (gasLevel) {
            case 0: // Normal - Turn off fans only if they're on
                if (fan1Current) Firebase.setBool(fbdo, "/auto_commands/fan1", false);
                if (fan2Current) Firebase.setBool(fbdo, "/auto_commands/fan2", false);
                if (fan3Current) Firebase.setBool(fbdo, "/auto_commands/fan3", false);
                break;
                
            case 1: // Warning - Turn on kitchen fan if not already on
                if (!fan3Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan3", true);
                    Serial.println("🔥 Auto: Turning ON kitchen fan (fan3)");
                }
                break;
                
            case 2: // Alert - Turn on kitchen + bedroom fans if not already on
                if (!fan3Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan3", true);
                    Serial.println("🔥 Auto: Turning ON kitchen fan (fan3)");
                }
                if (!fan2Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan2", true);
                    Serial.println("🔥 Auto: Turning ON bedroom fan (fan2)");
                }
                break;
                
            case 3: // Danger - Turn on all fans if not already on
                if (!fan1Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan1", true);
                    Serial.println("🔥 Auto: Turning ON living room fan (fan1)");
                }
                if (!fan2Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan2", true);
                    Serial.println("🔥 Auto: Turning ON bedroom fan (fan2)");
                }
                if (!fan3Current) {
                    Firebase.setBool(fbdo, "/auto_commands/fan3", true);
                    Serial.println("🔥 Auto: Turning ON kitchen fan (fan3)");
                }
                break;
        }
        
        // Log the event
        if (gasLevel > 0) {
            logGasAutoFanEvent(gasLevel);
        }
    }
    
    void logGasAutoFanEvent(int gasLevel) {
        if (!Firebase.ready()) return;
        
        unsigned long timestamp = millis() / 1000;
        String logId = String(timestamp) + "_gas_fan_" + String(random(1000, 9999));
        String logPath = "/history/" + logId;
        
        FirebaseJson json;
        json.set("type", "gas_auto_fan_control");
        json.set("timestamp", timestamp);
        json.set("device", "ESP32_HOME_GAS_SYSTEM");
        json.set("gasLevel", gasLevel);
        json.set("gasPPM", gasPPM);
        json.set("fansActivated", gasLevel);
        
        Firebase.setJSON(fbdo, logPath, json);
    }
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

// 🔧 ENHANCED: Auto Temperature Controller with Immediate Response and Conflict Resolution
class AutoTemperatureController {
private:
    float temperatureThreshold = DEFAULT_TEMP_THRESHOLD;
    bool autoTempEnabled = true;
    bool fanState = false;
    unsigned long lastTempCheck = 0;
    bool coolingMode = false;
    float lastTemperature = 0; // Store last temperature for immediate response
    
public:
    void init() {
        loadTemperatureSettings();
        Serial.printf("🌡️ Auto Temperature Controller initialized - Threshold: %.1f°C\n", temperatureThreshold);
    }
    
    void checkTemperature(float currentTemp) {
        unsigned long currentTime = millis();
        
        if (currentTime - lastTempCheck < TEMP_CHECK_INTERVAL) {
            return;
        }
        
        if (!autoTempEnabled || currentTemp <= 0) {
            lastTempCheck = currentTime;
            return;
        }
        
        lastTemperature = currentTemp; // Store for immediate response
        processTemperatureLogic(currentTemp);
        lastTempCheck = currentTime;
    }
    
    // 🔧 NEW: Immediate temperature response
    void processTemperatureLogic(float currentTemp) {
        bool shouldTurnOnFan = false;
        
        if (!coolingMode) {
            // Fan is off, check if we need to turn it on
            if (currentTemp > temperatureThreshold + TEMP_HYSTERESIS) {
                shouldTurnOnFan = true;
                coolingMode = true;
                Serial.printf("🌡️ Temp %.1f°C > %.1f°C + %.1f°C hysteresis - AUTO ON fan1\n", 
                             currentTemp, temperatureThreshold, TEMP_HYSTERESIS);
            }
        } else {
            // Fan is on, check if we can turn it off
            if (currentTemp < temperatureThreshold - TEMP_HYSTERESIS) {
                shouldTurnOnFan = false;
                coolingMode = false;
                Serial.printf("🌡️ Temp %.1f°C < %.1f°C - %.1f°C hysteresis - AUTO OFF fan1\n", 
                             currentTemp, temperatureThreshold, TEMP_HYSTERESIS);
            } else {
                shouldTurnOnFan = true; // Keep fan on
            }
        }
        
        // Update fan state if needed
        if (shouldTurnOnFan != fanState) {
            fanState = shouldTurnOnFan;
            sendFanCommandSafe(shouldTurnOnFan);
            updateFirebaseStatus(currentTemp);
            logTemperatureControlEvent(currentTemp, shouldTurnOnFan);
        }
    }
    
    // 🔧 FIXED: Immediate response to settings changes
    void updateSettings(float newThreshold, bool enabled) {
        if (newThreshold >= TEMP_THRESHOLD_MIN && newThreshold <= TEMP_THRESHOLD_MAX) {
            temperatureThreshold = newThreshold;
        }
        autoTempEnabled = enabled;
        
        Serial.printf("🌡️ Settings updated - Threshold: %.1f°C, Enabled: %s\n", 
                     temperatureThreshold, autoTempEnabled ? "YES" : "NO");
        
        // Save to Firebase
        if (Firebase.ready()) {
            Firebase.setFloat(fbdo, "/settings/temperatureThreshold", temperatureThreshold);
            Firebase.setBool(fbdo, "/settings/autoTemperatureEnabled", autoTempEnabled);
        }
        
        // 🔧 FIXED: Check temperature immediately with new settings
        if (autoTempEnabled && lastTemperature > 0) {
            Serial.printf("🌡️ Checking temperature immediately with new threshold: %.1f°C\n", lastTemperature);
            processTemperatureLogic(lastTemperature);
        }
        
        // If disabled, turn off fan
        if (!autoTempEnabled && fanState) {
            fanState = false;
            coolingMode = false;
            sendFanCommandSafe(false);
        }
    }
    
    float getThreshold() const { return temperatureThreshold; }
    bool isEnabled() const { return autoTempEnabled; }
    bool getFanState() const { return fanState; }
    
private:
    void loadTemperatureSettings() {
        if (!Firebase.ready()) return;
        
        if (Firebase.getFloat(fbdo, "/settings/temperatureThreshold")) {
            float threshold = fbdo.floatData();
            if (threshold >= TEMP_THRESHOLD_MIN && threshold <= TEMP_THRESHOLD_MAX) {
                temperatureThreshold = threshold;
            }
        } else {
            Firebase.setFloat(fbdo, "/settings/temperatureThreshold", DEFAULT_TEMP_THRESHOLD);
        }
        
        if (Firebase.getBool(fbdo, "/settings/autoTemperatureEnabled")) {
            autoTempEnabled = fbdo.boolData();
        } else {
            Firebase.setBool(fbdo, "/settings/autoTemperatureEnabled", true);
        }
    }
    
    // 🔧 NEW: Safe fan command with conflict resolution
    void sendFanCommandSafe(bool state) {
        if (!Firebase.ready()) return;
        
        // Check current fan1 state before sending command
        bool fan1Current = false;
        if (Firebase.getBool(fbdo, "/devices/fan1/state")) {
            fan1Current = fbdo.boolData();
        }
        
        // Only send command if state needs to change
        if (state != fan1Current) {
            Firebase.setBool(fbdo, "/auto_commands/fan1", state);
            Serial.printf("🌡️ Auto temp command: fan1 = %s\n", state ? "ON" : "OFF");
        } else {
            Serial.printf("🌡️ Fan1 already %s - no command needed\n", state ? "ON" : "OFF");
        }
    }
    
    void updateFirebaseStatus(float currentTemp) {
        if (!Firebase.ready()) return;
        
        Firebase.setBool(fbdo, "/status/autoTemperature/fanOn", fanState);
        Firebase.setFloat(fbdo, "/status/autoTemperature/currentTemp", currentTemp);
        Firebase.setFloat(fbdo, "/status/autoTemperature/threshold", temperatureThreshold);
        Firebase.setBool(fbdo, "/status/autoTemperature/coolingMode", coolingMode);
        Firebase.setTimestamp(fbdo, "/status/autoTemperature/lastUpdate");
    }
    
    void logTemperatureControlEvent(float temp, bool fanOn) {
        if (!Firebase.ready()) return;
        
        unsigned long timestamp = millis() / 1000;
        String logId = String(timestamp) + "_temp_" + String(random(1000, 9999));
        String logPath = "/history/" + logId;
        
        FirebaseJson json;
        json.set("type", "auto_temperature_control");
        json.set("timestamp", timestamp);
        json.set("device", "ESP32_HOME_TEMP_SYSTEM");
        json.set("temperature", temp);
        json.set("threshold", temperatureThreshold);
        json.set("fanState", fanOn);
        json.set("coolingMode", coolingMode);
        
        Firebase.setJSON(fbdo, logPath, json);
    }
};

// ========== ENHANCED SMART HOME CLASS ==========

class SmartHome {
private:
    PowerManager powerManager;
    TemperatureHumiditySensor tempHumidSensor;
    EnhancedGasSensor enhancedGasSensor;
    LightSensor lightSensor;
    AutoTemperatureController tempController;
    
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
    unsigned long lastAutoCommandCheck = 0;
    
public:
    SmartHome() : 
        tempHumidSensor(DHT_PIN),
        enhancedGasSensor(MQ2_PIN),
        lightSensor(LIGHT_SENSOR_PIN),
        autoLight(AUTO_LIGHT_PIN, "autoLight") {}
    
    void init() {
        Serial.begin(115200);
        Serial.println("\n=== ECO SMART HOME SYSTEM v4.1 - ENHANCED & OPTIMIZED ===");
        Serial.printf("🔋 Enhanced Battery: 4S Li-ion %.0fmAh (%.1fV-%.1fV)\n", 
                     BATTERY_CAPACITY_MAH, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE);
        Serial.printf("🌡️ Auto Temperature Control: Threshold %.1f°C ± %.1f°C hysteresis\n", 
                     DEFAULT_TEMP_THRESHOLD, TEMP_HYSTERESIS);
        Serial.printf("🔥 Enhanced Gas System: 3-level warning (%.0f/%.0f/%.0f PPM)\n", 
                     DEFAULT_GAS_THRESHOLD_1, DEFAULT_GAS_THRESHOLD_2, DEFAULT_GAS_THRESHOLD_3);
        Serial.println("💡 Auto Light Control: Responds to gate authentication");
        Serial.printf("🔧 Power thresholds: Min %.2fW, Display %.1fW, Max %.0fW\n", 
                     MIN_POWER_THRESHOLD_MW/1000.0, DISPLAY_POWER_THRESHOLD_MW/1000.0, MAX_POWER_DISPLAY_MW/1000.0);
        Serial.printf("🔧 Shunt resistor: %.3fΩ with custom calibrations\n", SHUNT_RESISTOR_VALUE);
        Serial.println("🔋 Dual SoC calculation: Voltage curve + Coulomb counting");
        Serial.println("🔧 Power MUX: 3-gate control for source switching");
        Serial.println("🔧 ENHANCED: Dynamic Gas Calibration + Conflict Resolution");
        
        Wire.begin(SDA_PIN, SCL_PIN);
        Serial.println("I2C initialized");
        
        // Initialize hardware
        powerManager.init();
        tempHumidSensor.init();
        enhancedGasSensor.init();
        lightSensor.init();
        tempController.init();
        
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
            Firebase.setString(fbdo, "/system/version", "4.1");
            Firebase.setString(fbdo, "/system/features", "Enhanced Auto Controls + Dynamic Gas Calibration + Conflict Resolution");
            Firebase.setString(fbdo, "/system/powerConfig", "0.01Ω Shunt + Enhanced SoC + Auto Controls");
            sendInitialStates();
        } else {
            Serial.println("\nFirebase connection failed!");
        }
        
        Serial.println("=== SMART HOME SYSTEM READY - ENHANCED & OPTIMIZED ===");
    }
    
    void run() {
        unsigned long currentTime = millis();
        
        // ⚡ CRITICAL: Check Firebase commands every 100ms for INSTANT response
        if (currentTime - lastFirebaseCheck >= 100) {
            checkFirebaseCommands();
            lastFirebaseCheck = currentTime;
        }
        
        // 🔧 ENHANCED: Check auto commands every 200ms with conflict resolution
        if (currentTime - lastAutoCommandCheck >= 200) {
            checkAutoCommands();
            lastAutoCommandCheck = currentTime;
        }
        
        // Read sensors every 3 seconds (for gas sensor stability)
        if (currentTime - lastSensorRead >= 3000) {
            readSensors();
            lastSensorRead = currentTime;
        }
        
        // Read power data every 2 seconds
        if (currentTime - lastPowerRead >= 2000) {
            readPowerData();
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
        enhancedGasSensor.read();
        lightSensor.read();
        
        // 🔧 ENHANCED: Auto temperature control with immediate response
        float currentTemp = tempHumidSensor.getTemperature();
        if (currentTemp > 0) {
            tempController.checkTemperature(currentTemp);
        }
    }
    
    void readPowerData() {
        powerManager.readPowerData();
        powerManager.sendDataToFirebase();
    }
    
    void updateAutoLight() {
        autoLight.updateState(lightSensor.isDark());
    }
    
    // 🔧 ENHANCED: Auto commands with conflict resolution
    void checkAutoCommands() {
        if (!Firebase.ready()) return;
        
        // Check auto light commands (no conflicts expected)
        checkAutoLightCommands();
        
        // Check auto fan commands with conflict resolution
        checkAutoFanCommands();
    }
    
    void checkAutoLightCommands() {
        // Auto light commands from gate authentication
        if (Firebase.getBool(fbdo, "/auto_commands/light1")) {
            if (fbdo.dataType() == "boolean" && fbdo.boolData()) {
                lights[0].setState(true);
                Firebase.setBool(fbdo, "/auto_commands/light1", false);
                Serial.println("💡 Auto: Turned ON light1 (Living Room)");
            }
        }
        
        if (Firebase.getBool(fbdo, "/auto_commands/light4")) {
            if (fbdo.dataType() == "boolean" && fbdo.boolData()) {
                lights[3].setState(true);
                Firebase.setBool(fbdo, "/auto_commands/light4", false);
                Serial.println("💡 Auto: Turned ON light4 (Garage)");
            }
        }
    }
    
    void checkAutoFanCommands() {
        // 🔧 NEW: Check auto fan commands with conflict resolution
        for (int i = 0; i < 3; i++) {
            String fanPath = "/auto_commands/fan" + String(i + 1);
            
            if (Firebase.getBool(fbdo, fanPath.c_str())) {
                if (fbdo.dataType() == "boolean") {
                    bool commandState = fbdo.boolData();
                    bool currentState = fans[i].getState();
                    
                    // Only execute if state actually needs to change
                    if (commandState != currentState) {
                        fans[i].setState(commandState);
                        Serial.printf("🔥 Auto: Changed fan%d to %s\n", 
                                     i + 1, commandState ? "ON" : "OFF");
                    }
                    
                    // Always clear the command after processing
                    Firebase.setBool(fbdo, fanPath.c_str(), false);
                }
            }
        }
    }
    
    void sendHeartbeat() {
        if (Firebase.ready()) {
            Firebase.setString(fbdo, "/system/status", "online");
            Firebase.setFloat(fbdo, "/system/uptime", millis() / 1000.0);
            Firebase.setFloat(fbdo, "/system/freeHeap", ESP.getFreeHeap() / 1024.0);
            Firebase.setString(fbdo, "/system/autoControls", "Enhanced Temperature + Gas 3-Level + Auto Lights");
            
            // 🔧 ENHANCED: Enhanced heartbeat with auto control status
            Firebase.setBool(fbdo, "/system/autoTemperatureEnabled", tempController.isEnabled());
            Firebase.setFloat(fbdo, "/system/temperatureThreshold", tempController.getThreshold());
            Firebase.setInt(fbdo, "/system/gasLevel", enhancedGasSensor.getGasLevel());
            Firebase.setFloat(fbdo, "/system/currentGasPPM", enhancedGasSensor.getGasPPM());
            Firebase.setFloat(fbdo, "/system/batterySOC", powerManager.getBatteryState().soc_combined);
            Firebase.setBool(fbdo, "/system/gasCalibrated", enhancedGasSensor.isCalibrated());
            
            Firebase.setTimestamp(fbdo, "/system/lastHeartbeat");
            Serial.printf("❤️ Enhanced Heartbeat - Temp: %.1f°C, Gas: %.0f ppm (L%d), SoC: %.1f%%, GasCal: %s\n", 
                         tempHumidSensor.getTemperature(), 
                         enhancedGasSensor.getGasPPM(),
                         enhancedGasSensor.getGasLevel(),
                         powerManager.getBatteryState().soc_combined,
                         enhancedGasSensor.isCalibrated() ? "YES" : "NO");
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
        
        // 🔧 ENHANCED: Send auto control initial states
        Firebase.setBool(fbdo, "/settings/autoTemperatureEnabled", tempController.isEnabled());
        Firebase.setFloat(fbdo, "/settings/temperatureThreshold", tempController.getThreshold());
        
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
        
        // 🔧 ENHANCED: Check for settings updates with immediate response
        checkSettingsUpdates();
    }
    
    // 🔧 ENHANCED: Settings updates with immediate response
    void checkSettingsUpdates() {
        static float lastTempThreshold = 0;
        static bool lastTempEnabled = true;
        
        // Check temperature threshold updates
        if (Firebase.getFloat(fbdo, "/settings/temperatureThreshold")) {
            float newThreshold = fbdo.floatData();
            
            bool tempEnabled = tempController.isEnabled();
            if (Firebase.getBool(fbdo, "/settings/autoTemperatureEnabled")) {
                tempEnabled = fbdo.boolData();
            }
            
            // 🔧 FIXED: Update settings if changed and trigger immediate check
            if (abs(newThreshold - lastTempThreshold) > 0.1 || tempEnabled != lastTempEnabled) {
                tempController.updateSettings(newThreshold, tempEnabled);
                
                // 🔧 FIXED: Immediate temperature check with new settings
                float currentTemp = tempHumidSensor.getTemperature();
                if (currentTemp > 0) {
                    tempController.checkTemperature(currentTemp);
                }
                
                lastTempThreshold = newThreshold;
                lastTempEnabled = tempEnabled;
            }
        }
        
        // Check gas threshold updates
        checkGasThresholdUpdates();
    }
    
    void checkGasThresholdUpdates() {
        static float lastGasThresholds[3] = {0, 0, 0};
        
        bool thresholdsChanged = false;
        float newThresholds[3];
        
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/warning")) {
            newThresholds[0] = fbdo.floatData();
            if (abs(newThresholds[0] - lastGasThresholds[0]) > 1) {
                thresholdsChanged = true;
            }
        }
        
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/alert")) {
            newThresholds[1] = fbdo.floatData();
            if (abs(newThresholds[1] - lastGasThresholds[1]) > 1) {
                thresholdsChanged = true;
            }
        }
        
        if (Firebase.getFloat(fbdo, "/settings/gasThresholds/danger")) {
            newThresholds[2] = fbdo.floatData();
            if (abs(newThresholds[2] - lastGasThresholds[2]) > 1) {
                thresholdsChanged = true;
            }
        }
        
        if (thresholdsChanged) {
            enhancedGasSensor.updateGasThresholds(newThresholds[0], newThresholds[1], newThresholds[2]);
            
            // Update tracking variables
            lastGasThresholds[0] = newThresholds[0];
            lastGasThresholds[1] = newThresholds[1];
            lastGasThresholds[2] = newThresholds[2];
        }
    }
    
    // 🔧 NEW: Expose controllers for external access if needed
    PowerManager& getPowerManager() { return powerManager; }
    AutoTemperatureController& getTemperatureController() { return tempController; }
    EnhancedGasSensor& getGasSensor() { return enhancedGasSensor; }
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
    
    Serial.println("🔧 Smart Home v4.1 - Enhanced & Optimized!");
    Serial.println("✅ Enhanced: MQ2 Dynamic Calibration");
    Serial.println("✅ Enhanced: Immediate Temperature Response");
    Serial.println("✅ Enhanced: Manual vs Auto Control Conflict Resolution");
    Serial.println("✅ Enhanced: State Management & Firebase Integration");
    Serial.println("OTA Ready - System fully operational!");
}

void loop() {
    ArduinoOTA.handle();
    smartHome.run();
    delay(50); // Minimal delay for instant response
}
