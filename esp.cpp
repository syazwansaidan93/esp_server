#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "time.h"
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define MAINLED_PIN 1
#define NIGHTLED_PIN 7
#define MAINLEDSW_PIN 2
#define MASTERSW_PIN 3
#define DS18B20_PIN 4
#define FAN_PIN 6
#define ACTIVITY_LED_PIN 0

#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8
#define LED_CHANNEL 0

const char* ssid = "wifi_slow2";

IPAddress staticIP(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

const char* ntpServer = "192.168.1.2";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

int mainledswState = 0;
int lastMainledswReading = 0;
int masterswState = 0;
int lastMasterswReading = 0;
int nightledPWMValue = 0;

OneWire oneWireBus(DS18B20_PIN);
DallasTemperature sensors(&oneWireBus);
DeviceAddress tempSensorAddress;
float tempThresholdOn = 28.9;
float tempThresholdOff = 28.7;
float currentTemperature = 0.0;
unsigned long lastSensorReadTime = 0;
const unsigned long sensorReadInterval = 4000;

bool fanOverride = false;
unsigned long fanOverrideStartTime = 0;
const unsigned long fanOverrideDuration = 30 * 60 * 1000;
unsigned long lastFanStateChange = 0;
const unsigned long fanCooldownDelay = 5000;
bool fanIsOnAutomatic = true;
bool serverStarted = false;
bool isAutoToggleDone = false;

Preferences preferences;
WebServer server(80);

const int maxFailedReads = 5;
int consecutiveFailedReads = 0;
bool sensorIsFaulty = false;

bool isBlinking = false;
unsigned long blinkStartTime = 0;
const unsigned long blinkDuration = 50;

void triggerBlink() {
  digitalWrite(ACTIVITY_LED_PIN, HIGH);
  blinkStartTime = millis();
  isBlinking = true;
}

void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      WiFi.begin(ssid);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      break;
    default:
      break;
  }
}

bool isTimeInRange(int startHour, int startMin, int endHour, int endMin) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  
  int nowInMinutes = timeinfo.tm_hour * 60 + timeinfo.tm_min;
  int startInMinutes = startHour * 60 + startMin;
  int endInMinutes = endHour * 60 + endMin;

  if (startInMinutes > endInMinutes) {
    return (nowInMinutes >= startInMinutes || nowInMinutes < endInMinutes);
  } else {
    return (nowInMinutes >= startInMinutes && nowInMinutes < endInMinutes);
  }
}

void handleMainToggle() {
  triggerBlink();
  if (masterswState == 1) {
    mainledswState = 1 - mainledswState;
  }
  String response = "Main LED Switch state is now: " + String(mainledswState);
  server.send(200, "text/plain", response);
}

void handleMasterToggle() {
  triggerBlink();
  masterswState = 1 - masterswState;
  String response = "Master Switch state is now: " + String(masterswState);
  server.send(200, "text/plain", response);
}

void handleSetTempOn() {
  triggerBlink();
  if (!server.hasArg("tempOn")) {
    server.send(400, "text/plain", "Missing 'tempOn' parameter.");
    return;
  }
  
  float newTempOn = server.arg("tempOn").toFloat();
  if (newTempOn > tempThresholdOff) {
    tempThresholdOn = newTempOn;
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOn", tempThresholdOn);
    preferences.end();
    server.send(200, "text/plain", "Fan 'tempOn' threshold set successfully.");
  } else {
    server.send(400, "text/plain", "New 'tempOn' must be greater than current 'tempOff'.");
  }
}

void handleSetTempOff() {
  triggerBlink();
  if (!server.hasArg("tempOff")) {
    server.send(400, "text/plain", "Missing 'tempOff' parameter.");
    return;
  }
  
  float newTempOff = server.arg("tempOff").toFloat();
  if (tempThresholdOn > newTempOff) {
    tempThresholdOff = newTempOff;
    preferences.begin("bilik-config", false);
    preferences.putFloat("tempOff", tempThresholdOff);
    preferences.end();
    server.send(200, "text/plain", "Fan 'tempOff' threshold set successfully.");
  } else {
    server.send(400, "text/plain", "New 'tempOff' must be less than current 'tempOn'.");
  }
}

void handleFanOn30m() {
  triggerBlink();
  if (masterswState == 1 && fanIsOnAutomatic == true && digitalRead(FAN_PIN) == LOW) {
    fanOverride = true;
    fanOverrideStartTime = millis();
    fanIsOnAutomatic = false;
    digitalWrite(FAN_PIN, HIGH);
    server.send(200, "text/plain", "Fan turned on for 30 minutes. Automatic mode is suspended.");
  } else {
    server.send(400, "text/plain", "Cannot turn fan on. Master switch is off, fan is not in automatic mode, or fan is already on.");
  }
}

void handleFanOff() {
  triggerBlink();
  digitalWrite(FAN_PIN, LOW);
  fanOverride = false;
  fanIsOnAutomatic = true;
  server.send(200, "text/plain", "Fan turned off. Automatic mode is restored.");
}

void handleSetNightledPWM() {
  triggerBlink();
  if (!server.hasArg("pwmValue")) {
    server.send(400, "text/plain", "Missing 'pwmValue' parameter.");
    return;
  }
  
  int newPWM = server.arg("pwmValue").toInt();
  if (newPWM >= 0 && newPWM <= 255) {
    nightledPWMValue = newPWM;
    preferences.begin("bilik-config", false);
    preferences.putInt("nightledPWM", nightledPWMValue);
    preferences.end();
    server.send(200, "text/plain", "Night LED PWM value set successfully.");
  } else {
    server.send(400, "text/plain", "PWM value must be between 0 and 255.");
  }
}

bool shouldNightLedBeOn() {
  return isTimeInRange(19, 15, 7, 15) && (mainledswState == 0 || masterswState == 0);
}

void handleITemp() {
  triggerBlink();
  if (sensorIsFaulty) {
    server.send(404, "text/plain", "Sensor is faulty.");
  } else {
    server.send(200, "text/plain", String(currentTemperature));
  }
}

void handleState() {
  triggerBlink();
  StaticJsonDocument<300> doc;
  
  struct tm timeinfo;
  char timeString[64];
  getLocalTime(&timeinfo);
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  doc["currentTime"] = timeString;

  long uptimeSeconds = millis() / 1000;
  doc["uptimeSeconds"] = uptimeSeconds;

  doc["mainledswState"] = mainledswState;
  doc["masterswState"] = masterswState;
  doc["mainledState"] = digitalRead(MAINLED_PIN);
  
  doc["nightledState"] = shouldNightLedBeOn();
  doc["nightledPWMValue"] = nightledPWMValue;

  doc["temperature"] = currentTemperature;
  doc["fanState"] = digitalRead(FAN_PIN);
  doc["tempThresholdOn"] = tempThresholdOn;
  doc["tempThresholdOff"] = tempThresholdOff;
  doc["fanOverride"] = fanOverride;
  doc["sensorIsFaulty"] = sensorIsFaulty;
  doc["fanIsOnAutomatic"] = fanIsOnAutomatic;

  String jsonResponse;
  serializeJson(doc, jsonResponse);

  server.send(200, "application/json", jsonResponse);
}

void controlNightLED() {
  if (shouldNightLedBeOn()) {
    ledcWrite(NIGHTLED_PIN, nightledPWMValue);
  } else {
    ledcWrite(NIGHTLED_PIN, 0);
  }
}

void controlFan() {
  if (fanIsOnAutomatic && !sensorIsFaulty) {
    if (masterswState == 1) {
      if (millis() - lastFanStateChange < fanCooldownDelay) {
        return;
      }
      
      if (digitalRead(FAN_PIN) == LOW && currentTemperature >= tempThresholdOn) {
        digitalWrite(FAN_PIN, HIGH);
        lastFanStateChange = millis();
      }
      else if (digitalRead(FAN_PIN) == HIGH && currentTemperature <= tempThresholdOff) {
        digitalWrite(FAN_PIN, LOW);
        lastFanStateChange = millis();
      }
    } else {
      digitalWrite(FAN_PIN, LOW);
    }
  } else if (!fanIsOnAutomatic) {
  } else {
    digitalWrite(FAN_PIN, LOW);
  }
}

void setup() {
  setCpuFrequencyMhz(80);
  WiFi.onEvent(onWiFiEvent);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid);
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  preferences.begin("bilik-config", false);
  tempThresholdOn = preferences.getFloat("tempOn", 28.9);
  tempThresholdOff = preferences.getFloat("tempOff", 28.7);
  nightledPWMValue = preferences.getInt("nightledPWM", 0);
  preferences.end();

  pinMode(MAINLED_PIN, OUTPUT);
  pinMode(NIGHTLED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(ACTIVITY_LED_PIN, OUTPUT);
  pinMode(MAINLEDSW_PIN, INPUT_PULLUP);
  pinMode(MASTERSW_PIN, INPUT_PULLUP);

  sensors.begin();
  sensors.setResolution(tempSensorAddress, 12);

  tempSensorAddress[0] = 0x28;
  tempSensorAddress[1] = 0x07;
  tempSensorAddress[2] = 0xBB;
  tempSensorAddress[3] = 0x83;
  tempSensorAddress[4] = 0x00;
  tempSensorAddress[5] = 0x00;
  tempSensorAddress[6] = 0x00;
  tempSensorAddress[7] = 0xF5;
  
  ledcAttach(NIGHTLED_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

  server.on("/mainledswState", handleMainToggle);
  server.on("/masterswState", handleMasterToggle);
  server.on("/set_temp_on", handleSetTempOn);
  server.on("/set_temp_off", handleSetTempOff);
  server.on("/on-30m", handleFanOn30m);
  server.on("/off", handleFanOff);
  server.on("/set_nightled_pwm", handleSetNightledPWM);
  server.on("/state", handleState);
  server.on("/i_temp", handleITemp);
}

void loop() {
  if (isBlinking && (millis() - blinkStartTime) >= blinkDuration) {
    digitalWrite(ACTIVITY_LED_PIN, LOW);
    isBlinking = false;
  }
  if (WiFi.status() == WL_CONNECTED && !serverStarted) {
    server.begin();
    serverStarted = true;
  }
  
  server.handleClient();

  if (millis() - lastSensorReadTime >= sensorReadInterval && masterswState == 1 && fanIsOnAutomatic) {
    triggerBlink();
    sensors.requestTemperatures();
    float temp = sensors.getTempC(tempSensorAddress);

    if (temp != DEVICE_DISCONNECTED_C) {
      currentTemperature = temp;
      consecutiveFailedReads = 0;
      sensorIsFaulty = false;
    } else {
      consecutiveFailedReads++;
      if (consecutiveFailedReads >= maxFailedReads) {
        sensorIsFaulty = true;
      }
    }
    lastSensorReadTime = millis();
  }

  if (fanOverride && (millis() - fanOverrideStartTime) >= fanOverrideDuration) {
    fanOverride = false;
    fanIsOnAutomatic = true;
  }
  
  if (fanOverride) {
    digitalWrite(FAN_PIN, HIGH);
  }

  int currentMainledswReading = digitalRead(MAINLEDSW_PIN);
  if (currentMainledswReading != lastMainledswReading && currentMainledswReading == LOW) {
    if (masterswState == 1) {
      mainledswState = 1 - mainledswState;
    }
  }
  lastMainledswReading = currentMainledswReading;

  int currentMasterswReading = digitalRead(MASTERSW_PIN);
  if (currentMasterswReading != lastMasterswReading && currentMasterswReading == LOW) {
    masterswState = 1 - masterswState;
  }
  lastMasterswReading = currentMasterswReading;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_hour == 19 && timeinfo.tm_min == 0 && !isAutoToggleDone) {
      if (mainledswState == 0) {
        mainledswState = 1;
        isAutoToggleDone = true;
      }
    }
    if (timeinfo.tm_hour < 19) {
      isAutoToggleDone = false;
    }
  }

  if (mainledswState == 1 && masterswState == 1) {
    digitalWrite(MAINLED_PIN, HIGH);
  } else {
    digitalWrite(MAINLED_PIN, LOW);
  }

  controlNightLED();
  controlFan();

  yield();
}
