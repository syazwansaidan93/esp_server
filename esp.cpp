#include <Wire.h>
#include <Adafruit_INA219.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "esp32-hal-cpu.h"
#include <WiFi.h>

Adafruit_INA219 ina219;
bool ina219_found = false;

#define DS18B20_PIN 4
OneWire oneWireBus(DS18B20_PIN);
DallasTemperature sensors(&oneWireBus);

#define RELAY_PIN 5

DeviceAddress outdoorThermometer;

float voltage_low_cutoff_V = 12.1;
float voltage_high_on_threshold_V = 13.2;
float power_on_threshold_mW = 2000.0;
float power_off_threshold_mW = 500.0;
unsigned long debounce_delay_ms = 60000;
unsigned long debounce_timer_start = 0;
int last_stable_state = LOW;

void setINA219PowerDown() {
  uint16_t config_value = 0x399F;
  config_value &= ~0x0007;
  Wire.beginTransmission(0x40);
  Wire.write(0x00);
  Wire.write((config_value >> 8) & 0xFF);
  Wire.write(config_value & 0xFF);
  Wire.endTransmission();
}

void setINA219Active() {
  uint16_t config_value = 0x399F;
  Wire.beginTransmission(0x40);
  Wire.write(0x00);
  Wire.write((config_value >> 8) & 0xFF);
  Wire.write(config_value & 0xFF);
  Wire.endTransmission();
}

void printOutdoorTemp() {
  sensors.requestTemperatures();
  float outdoor_temp_C = sensors.getTempC(outdoorThermometer);
  Serial.print("{ \"sensor\": \"o_temp\", \"value\": ");
  if (outdoor_temp_C != DEVICE_DISCONNECTED_C) {
    Serial.print(outdoor_temp_C);
  } else {
    Serial.print("\"error\"");
  }
  Serial.println(" }");
}

void printSolarData() {
  setINA219Active();
  delay(50);
  Serial.print("{ \"sensor\": \"solar_pwr\", ");
  if (!ina219_found) {
    Serial.println("\"status\": \"error\" }");
  } else {
    float ina219_voltage_V = ina219.getBusVoltage_V();
    float ina219_current_mA = ina219.getCurrent_mA();
    float ina219_power_mW = ina219.getPower_mW();
    Serial.print("\"voltage_V\": ");
    Serial.print(ina219_voltage_V);
    Serial.print(", \"current_mA\": ");
    Serial.print(ina219_current_mA);
    Serial.print(", \"power_mW\": ");
    Serial.print(ina219_power_mW);
    Serial.println(" }");
  }
  setINA219PowerDown();
}

void printRelayStatus() {
  int relayStatus = digitalRead(RELAY_PIN);
  if (relayStatus == HIGH) {
    Serial.println("{\"sensor\": \"relay\", \"value\": \"ON\"}");
  } else {
    Serial.println("{\"sensor\": \"relay\", \"value\": \"OFF\"}");
  }
}

void printUptime() {
  unsigned long uptime_ms = millis();
  Serial.print("{ \"sensor\": \"uptime\", \"value_ms\": ");
  Serial.print(uptime_ms);
  Serial.println(" }");
}

void checkAndControlRelay() {
  if (!ina219_found) {
    return;
  }
  
  setINA219Active();
  delay(50);
  float current_power_mW = ina219.getPower_mW();
  float current_voltage_V = ina219.getBusVoltage_V();
  setINA219PowerDown();

  int desired_state = last_stable_state;

  if ((current_voltage_V >= voltage_high_on_threshold_V) || ((current_power_mW >= power_on_threshold_mW) && (current_voltage_V > voltage_low_cutoff_V))) {
    desired_state = HIGH;
  } else if ((current_power_mW <= power_off_threshold_mW) || (current_voltage_V <= voltage_low_cutoff_V)) {
    desired_state = LOW;
  }

  if (desired_state != last_stable_state) {
    if (debounce_timer_start == 0) {
      debounce_timer_start = millis();
    }
    
    if (millis() - debounce_timer_start >= debounce_delay_ms) {
      digitalWrite(RELAY_PIN, desired_state);
      last_stable_state = desired_state;
      debounce_timer_start = 0;
      
      if (desired_state == HIGH) {
        Serial.println("{\"relay_event\": \"on\"}");
      } else {
        Serial.println("{\"relay_event\": \"off\"}");
      }
    }
  } else {
    debounce_timer_start = 0;
  }
}

void setup() {
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  
  WiFi.mode(WIFI_OFF);
  
  Wire.begin(6, 7);
  
  ina219_found = ina219.begin();
  if (!ina219_found) {
    Serial.println("Error: INA219 not found!");
  }

  sensors.begin();
  if (sensors.getDeviceCount() < 1) {
    Serial.println("Error: No DS18B20 sensors found!");
  } else {
    const DeviceAddress outdoorAddress = { 0x28, 0x09, 0x8A, 0xC0, 0x00, 0x00, 0x00, 0xC7 };
    
    memcpy(outdoorThermometer, outdoorAddress, 8);
    
    sensors.setResolution(outdoorThermometer, 10);
  }
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  setINA219PowerDown();
}

void loop() {
  checkAndControlRelay();
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "o") {
      printOutdoorTemp();
    } else if (command == "s") {
      printSolarData();
    } else if (command == "r") {
      printRelayStatus();
    } else if (command == "u") {
      printUptime();
    } else {
      Serial.println("Invalid command.");
    }
  }
}
