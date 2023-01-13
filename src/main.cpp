#include <Arduino.h>
#include <rom/rtc.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include "app.h"
#include "SwitchRelay.h"
#include "SwitchController.h"
#include "MotionSensor.h"
#include "LcdFixedPositionPrint.h"
#include "LcdFixedPositionVerticalPrint.h"
#include "LcdMarqueeString.h"
#include "LcdBigDigits.h"
#include "LcdSymbolAlert.h"
#include "LcdBigSymbolAlert.h"
#include "LcdPrintDrawer.h"
#include "time.h"
#include "reset_info.h"
#include "version.h"

struct config_t {
  bool motion = true;
  bool backlight = true;
  unsigned int backlightTimeout = LCD_BACKLIGHT_TIMEOUT_MILLIS;
  unsigned long wifi_reconnect_ms = WIFI_RECONNECT_MILLIS;
  unsigned long wifi_watchdog_ms = WIFI_WATCHDOG_MILLIS;
  unsigned int daylight_offset_sec = DAYLIGHT_OFFSET_SEC;
  unsigned short wdt_timeout_sec = WDT_TIMEOUT_SEC;
} Config;

const PROGMEM uint8_t lcdCustomChars[4][8] = {
  { 0x1F, 0x1F, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F }, // 1: o
  { 0x1F, 0x1F, 0x1F, 0x07, 0x0F, 0x0F, 0x1F, 0x1F }, // 2: n
  { 0x1F, 0x10, 0x10, 0x11, 0x11, 0x10, 0x10, 0x1F }, // 3: left half-rect
  { 0x1F, 0x01, 0x01, 0x11, 0x11, 0x01, 0x01, 0x1F }  // 4: right half-rect
};

RESET_REASON
  reset_reason[2];

unsigned long 
  now = 0,
  lastWifiOnline = 0,
  lastWifiReconnect = 0,
  lastPubSubReconnectAttempt = 0,
  lastUptimeUpdate = 0,
  lastMotionDetected = 0,
  lastWifiRSSI = 0,
  lastMeetingNameUpdate = 0,
  lastTimeConfig = 0,
  lastClockDraw = 0,
  lastAcPublish = 0,
  lastUiRedraw = 0,
  lastBlindsRead = 0,
  lastOtaHandle = 0,
  otaUpdateStart = 0,
  runCounter = 0;

bool 
  needPublishCurrentState = true,
  needPublishMotionState = true,
  justStarted = true,
  otaUpdateMode = false,
  lastBlindsZero = false,
  showCurrent = false,
  spiffsEnabled = true;

int
  lastTimeMin = -1;

char
  sw_reset_reason = 0,
  clock_separator = ' ';

struct tm timeinfo;

Preferences preferences;
WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
hd44780_I2Cexp lcd;
MotionSensor motionSensor(MOTION_SENSOR_PIN);

LcdFixedPositionPrint meetingTextDisplay(&lcd, 2, 0);
LcdMarqueeString meetingTextControl(15);

LcdBigSymbolAlert hallAlert(&lcd, 10, 17);
LcdBigSymbolAlert entranceAlert(&lcd, 11, 17);

LcdPrintDrawer 
  powerSourceDrawer,
  voltageDrawer,
  batteryLevelDrawer,
  currentDrawer,
  splitterDrawer,
  *voltageOrBatteryLevelDrawer = &voltageDrawer;

LcdFixedPositionPrint 
  powerSourceDrawerTextDisplay(&lcd, 0, 16),
  voltageDrawerTextDisplay(&lcd, 1, 16),
  batteryLevelDrawerTextDisplay(&lcd, 1, 16),
  batteryLevelDrawerTextDisplay2(&lcd, 3, 16),
  currentDrawerTextDisplay(&lcd, 2, 16),
  switchControllerTextDisplay(&lcd, 3, 0);

LcdFixedPositionVerticalPrint
  splitterDrawerTextDisplay(&lcd, 0, 15);

void restart(char code) {
  preferences.putULong("SW_RESET_UPTIME", millis());
  preferences.putUChar("SW_RESET_REASON", code);
  preferences.end();

  delay(200);
  ESP.restart();
}

bool getSpiffsEnabled() {
  return spiffsEnabled;
}

void setLcdBacklight(boolean state) {
  if (!Config.backlight) return;

  if (state) lcd.backlight();
  else lcd.noBacklight();
}

bool reconnectPubSub() {
  if (now - lastPubSubReconnectAttempt > MQTT_RECONNECT_MILLIS) {
    lastPubSubReconnectAttempt = now;

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, true)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, MQTT_STATUS_ONLINE_MSG, true);
      pubSubClient.publish(MQTT_VERSION_TOPIC, VERSION, true);
      
      pubSubClient.subscribe(MQTT_RESTART_CONTROL_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_CURRENT_MEETING_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_BACKLIGHT_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_CONFIG_TOPIC "/set", MQTTQOS0);
      pubSubClient.subscribe(MQTT_CURRENT_MEETING_TOPIC, MQTTQOS0);

      // pubSubClient.subscribe("loc/hall/motion", MQTTQOS0);
      // pubSubClient.subscribe("entrance/motion", MQTTQOS0);

      pubSubClient.subscribe("dev/power-line/source", MQTTQOS0);
      pubSubClient.subscribe("dev/power-line/voltage", MQTTQOS0);
      pubSubClient.subscribe("dev/power-line/current", MQTTQOS0);
      pubSubClient.subscribe("dev/esp32-ups-01/battery/percent/int", MQTTQOS0);

      pubSubSwitchControllerSubscribe(&pubSubClient);
    }
    
    return pubSubClient.connected();
  }

  return false;
}

bool wifiLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiOnline > Config.wifi_watchdog_ms) restart(RESET_ON_WIFI_WD_TIMEOUT);
    else if (now - lastWifiReconnect > Config.wifi_reconnect_ms) {
      lastWifiReconnect = now;

      if (WiFi.reconnect()) {
        lastWifiOnline = now;
        return true;
      }
    }

    return false;
  }
  
  lastWifiReconnect = now;
  lastWifiOnline = now;
  return true;
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;
  if (length > 255) return;
  log_d("Handle message from '%s':\n%s", topic, (char*)payload);

  String topicStr = topic;
  if (pubSubSwitchControllerHandleMessage(topicStr, payload, length)) {
    return;
  }
  
  if (topicStr.equals(MQTT_BACKLIGHT_TOPIC)) {
    setLcdBacklight(parseBooleanMessage(payload, length));
  }
  else if (topicStr.equals(MQTT_CONFIG_TOPIC "/set")) {
    auto f = SPIFFS.open("/config.json", "w");
    f.write(payload, length);
    f.close();

    restart(RESET_ON_CONFIG_UPDATE);
    return;
  }
  else if (topicStr.equals(MQTT_RESTART_CONTROL_TOPIC)) {
    restart(RESET_ON_MQTT_RESET_TOPIC);
  }
  else if (topicStr.equals(MQTT_CURRENT_MEETING_TOPIC)) {
    if (now - lastMeetingNameUpdate > 5000) {
      lastMeetingNameUpdate = now;
      
      char meeting[256];
      int l = length > 255 ? 255 : length;
      strncpy(meeting, (const char*)payload, l);
      meeting[l] = 0;

      meetingTextControl.setText(meeting);
    }
  }
  else if (topicStr.equals("dev/power-line/source")) {
    if (length > 0) {
      if (payload[0] == 'l') {
        powerSourceDrawer.print("LINE");
        voltageOrBatteryLevelDrawer = &voltageDrawer;
        showCurrent = true;
      }
      else if (payload[0] == 'g') {
        powerSourceDrawer.print("GENR");
        voltageOrBatteryLevelDrawer = &voltageDrawer;
        showCurrent = true;
      }
      else if (payload[0] == 'b') {
        powerSourceDrawer.print("BATT");
        voltageOrBatteryLevelDrawer = &batteryLevelDrawer;
        showCurrent = false;
      }
      else {
        powerSourceDrawer.print("   -");
        voltageOrBatteryLevelDrawer = &voltageDrawer;
      }
    }
  }
  else if (topicStr.equals("dev/power-line/voltage")) {
    char voltage[4] = {'-', ' ', ' ', 'V'};
    if (length == 3) {
      memcpy(voltage, payload, 3);
    }
    voltageDrawer.print(voltage);
  }
  else if (topicStr.equals("dev/power-line/current")) {
    char current[4] = {' ', ' ', ' ', ' '};
    if (length > 0 && length <= 3) {
      memcpy(&current[3-length], payload, length);
      current[3] = 'A';
      showCurrent = true;
    }
    currentDrawer.print(current);
  }
  else if (topicStr.equals("dev/esp32-ups-01/battery/percent/int")) {
    char percent[4] = {'-', ' ', ' ', '%'};
    if (length > 0 && length <= 3) {
      memcpy(percent, payload, length);
    }
    batteryLevelDrawer.print(percent);
  }
  else if (topicStr.equals("entrance/motion")) {
    if (payload[0] == '1') entranceAlert.blink(5000);
    // else entranceAlert.reset();
  }
  else if (topicStr.equals("loc/hall/motion")) {
    if (payload[0] == '1') hallAlert.blink(5000);
    // else hallAlert.reset();
  }
}

void onMotionSensor() {
  log_d("On MotionSensor event: state=%s", motionSensor.getState() == None ? "NONE" : "DETECTED");
  needPublishMotionState = true;

  if (motionSensor.getState() == Detected) {
    lastMotionDetected = now;
    setLcdBacklight(true);
  }
}

#ifdef AC_DETECTOR
void IRAM_ATTR blindsZeroISR() {
  lastBlindsZero = true;
}

unsigned long zc = 0;
void IRAM_ATTR acDetectorISR() {
  zc++;
}

void setupAcDetector() {
  pinMode(AC_DETECTOR_PIN, INPUT);
  attachInterrupt(AC_DETECTOR_PIN, acDetectorISR, RISING);
}
#endif

void setupLcd() {
  log_d("Initialize LCD display");

  // lcd.setExecTimes(hd44780::HD44780_CHEXECTIME, 37);
  lcd.begin(20, 4);
  lcd.setExecTimes(1520, 37);

  for(uint8_t i=0; i < 4; i++)
    lcd.createChar(i, lcdCustomChars[i]);

  setupBigDigit(&lcd);

  setLcdBacklight(true);
  
  lcd.home();
  lcd.print(WIFI_HOSTNAME);
}

void otaStarted() {
  now = millis();
  otaUpdateStart = now;
  otaUpdateMode = true;

  lcd.clear();
  lcd.setCursor(0, 0);
  setLcdBacklight(true);
  lcd.print(" UPDATING FIRMWARE");
}

void otaProgress(unsigned int currentBytes, unsigned int totalBytes) {
  now = millis();

  unsigned int 
    minutes = (now-otaUpdateStart)/1000./60.,
    seconds = (now-otaUpdateStart)/1000. - minutes*60,
    progress = ((float)currentBytes / totalBytes) * 20;

  lcd.setCursor(0, 1);
  for (int i=0; i<=progress && i<20; i++) {
    lcd.print((char)0xFF);
  }
  
  lcd.setCursor(0, 2);
  lcd.printf("%dB / %dB", currentBytes, totalBytes);

  lcd.setCursor(0, 3);
  lcd.printf("%02d:%02d", minutes, seconds);

  esp_task_wdt_reset();
}

void otaEnd() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("        DONE        ");

  for (int i=0;i<10;i++) {
    if (i%2) setLcdBacklight(false);
    else setLcdBacklight(true);
    delay(200);
  }

  restart(RESET_ON_OTA_SUCCESS);
}

void otaError(ota_error_t error) {
  lcd.clear();
  lcd.setCursor(0, 1);

  for (int i=0;i<2;i++) {
    if (i%2) setLcdBacklight(false);
    else setLcdBacklight(true);
    delay(200);
  }

  lcd.printf("ERROR: %d", error);

  delay(5000);
  restart(RESET_ON_OTA_FAIL);
}

bool getAdjustedTime(tm *t) {
  if (!getLocalTime(t)) {
    return false;
  }

  t->tm_min += 4;
  mktime(t);

  return true;
}

void setup() {
  reset_reason[0] = rtc_get_reset_reason(0);
  reset_reason[1] = rtc_get_reset_reason(1);

  preferences.begin("multiswitch-01");
  runCounter = preferences.getULong("__RUN_N", 0) + 1;
  preferences.putULong("__RUN_N", runCounter);

  setupSwitchController();

  if(spiffsEnabled && !SPIFFS.begin(true)) {
    log_e("SPIFFS mount failed");
    spiffsEnabled = false;
  }

  if (spiffsEnabled && SPIFFS.exists("/state.bin")) {
    char b[SWITCH_RELAY_COUNT];
    auto f = SPIFFS.open("/state.bin");
    f.readBytes(b, SWITCH_RELAY_COUNT);

    for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
      setSwitchState(i, b[i] == '1' ? On : Off, false);
    }

    f.close();
  }
  
  if (spiffsEnabled && SPIFFS.exists("/config.json")) {
    auto f = SPIFFS.open("/config.json");
    StaticJsonDocument<1024> jdoc;
    DeserializationError error = deserializeJson(jdoc, f);

    if (!error) {
      Config.backlight = jdoc["backlight"];
      Config.backlightTimeout = jdoc["backlightTimeout"];
      Config.motion = jdoc["motion"];
      Config.wifi_reconnect_ms = jdoc["wifi_reconnect_ms"] > 0 ? jdoc["wifi_reconnect_ms"] : WIFI_RECONNECT_MILLIS;
      Config.wifi_watchdog_ms = jdoc["wifi_watchdog_ms"] > 0 ? jdoc["wifi_watchdog_ms"] : WIFI_WATCHDOG_MILLIS;
      Config.daylight_offset_sec = jdoc["daylight_offset_sec"];
      Config.wdt_timeout_sec = jdoc["wdt_timeout_sec"] > 0 ? jdoc["wdt_timeout_sec"] : WDT_TIMEOUT_SEC;
    }

    f.close();
  }

  // if (spiffsEnabled && SPIFFS.exists(SW_RESET_REASON_FILENAME)) {
  //   auto f = SPIFFS.open(SW_RESET_REASON_FILENAME);
  //   sw_reset_reason = f.read();
  //   f.close();

  //   SPIFFS.remove(SW_RESET_REASON_FILENAME);
  // }

  sw_reset_reason = preferences.getUChar("SW_RESET_REASON");

  // Setup WDT
  esp_task_wdt_init(Config.wdt_timeout_sec, true);
  esp_task_wdt_add(NULL);

  setupLcd();

#ifdef AC_DETECTOR
  setupAcDetector();
#endif

  if (Config.motion) {
    motionSensor.onChanged(onMotionSensor);
  }

  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(wifi_ps_type_t::WIFI_PS_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.onStart(otaStarted);
  ArduinoOTA.onProgress(otaProgress);
  ArduinoOTA.onEnd(otaEnd);
  ArduinoOTA.onError(otaError);
  ArduinoOTA.begin();

  pubSubClient.setCallback(onMqttMessage);
  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);

  pinMode(BLINDS_ZERO_PIN, INPUT_PULLUP);
  // attachInterrupt(BLINDS_ZERO_PIN, blindsZeroISR, ONLOW);

  now = millis();
  lastWifiOnline = now;

  lcd.setCursor(0, 2);
  lcd.print("RUN #");
  lcd.print(runCounter);
  lcd.setCursor(0, 3);
  lcd.print("VERSION: ");
  lcd.print(VERSION_SHORT);

  delay(preferences.getUInt("RUN_N_MSG_MS", 1000));
}

bool onJustStarted() {
    bool result = true;
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/0", get_reset_reason_info(reset_reason[0]).c_str(), true);
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/1", get_reset_reason_info(reset_reason[1]).c_str(), true);
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/uptime", String(preferences.getULong("SW_RESET_UPTIME", 0)).c_str(), true);
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/code", String((uint8_t)sw_reset_reason).c_str(), true);
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/sw", get_sw_reset_reason_info(sw_reset_reason).c_str(), true);
    result &= pubSubClient.publish(MQTT_CLIENT_ID "/restart_reason/run_id", String(runCounter-1).c_str(), true);

    String buf;
    DynamicJsonDocument jdoc(256);
    jdoc["backlight"] = Config.backlight;
    jdoc["backlightTimeout"] = Config.backlightTimeout;
    jdoc["motion"] = Config.motion;
    jdoc["wifi_reconnect_ms"] = Config.wifi_reconnect_ms;
    jdoc["wifi_watchdog_ms"] = Config.wifi_watchdog_ms;
    jdoc["daylight_offset_sec"] = Config.daylight_offset_sec;
    jdoc["wdt_timeout_sec"] = Config.wdt_timeout_sec;
    
    serializeJson(jdoc, buf);
    result &= pubSubClient.publish(MQTT_CONFIG_TOPIC, buf.c_str(), true);

    return result;
}

void clock_loop(unsigned long now) {
  if (lastTimeConfig > 0 && now - lastClockDraw >= 1000) {
    if (lastClockDraw == 0) {
      lcd.setCursor(0, 0);
      lcd.write("                    ");
      lcd.setCursor(0, 1);
      lcd.write("                    ");
      lastTimeMin = -1;
    }

    lastClockDraw = now;

    if (lastTimeConfig != now) {
      timeinfo.tm_sec += 1;
      if (timeinfo.tm_sec == 60 || timeinfo.tm_sec == 0)
        mktime(&timeinfo);
    }

    if (timeinfo.tm_min != lastTimeMin) {
      lastTimeMin = timeinfo.tm_min;
      lcd.setCursor(0, 0);
      // lcd.write("                ");
      lcd.setCursor(0, 1);
      // lcd.write("                ");
      showBigNumberFixed(&lcd, timeinfo.tm_hour, 2, 0);
      showBigNumberFixed(&lcd, timeinfo.tm_min, 2, 8);

      splitterDrawerTextDisplay.print("|||");
    }

    clock_separator = clock_separator == ' ' ? '.' : ' ';
    lcd.setCursor(7, 0);
    lcd.write(clock_separator);
    lcd.setCursor(7, 1);
    lcd.write(clock_separator);
  }
}

void timeconfig_loop(unsigned long now) {
  if (lastTimeConfig == 0 || now - lastTimeConfig > NTP_UPDATE_MS) {
    lastTimeConfig = now;
    configTime(GMT_OFFSET_SEC, Config.daylight_offset_sec, "0." NTP_SERVER, "1." NTP_SERVER, "2." NTP_SERVER);

    if (!getAdjustedTime(&timeinfo)) {
      lastTimeConfig = 0;
    }
  }
}

void pubSubClientLoop() {
  if (!pubSubClient.connected() && !reconnectPubSub()) return;

  if (needPublishCurrentState) {
    needPublishCurrentState = pubSubSwitchControllerPublishState(&pubSubClient);
  }

  if (needPublishMotionState) {
    needPublishMotionState = !pubSubClient.publish("balcony/motion", motionSensor.getState() != None ? "1" : "0");
  }

  pubSubClient.loop();
}

#ifdef AC_DETECTOR
void ac_loop(unsigned long now) {
  if (now - lastAcPublish > 1000) {
    auto value = zc;
    zc = 0;

    pubSubClient.publish(MQTT_CLIENT_ID "/ac/count", String(value).c_str());
    pubSubClient.publish(MQTT_CLIENT_ID "/ac/millis", String(now - lastAcPublish).c_str());

    lastAcPublish = now;
  }
}
#endif

void motion_loop() {
  if (Config.motion) {
    motionSensor.loop();
  }

  if (now - lastMotionDetected > Config.backlightTimeout) {
    setLcdBacklight(false);
  }
}

void ui_loop() {
  if (now - lastUiRedraw < 150) return;
  lastUiRedraw = now;

  drawSwitchControlLcd(&switchControllerTextDisplay);
  meetingTextControl.draw(&meetingTextDisplay);
  // hallAlert.draw();
  // entranceAlert.draw();
  powerSourceDrawer.draw(&powerSourceDrawerTextDisplay);
  splitterDrawer.draw(&splitterDrawerTextDisplay);

  if (showCurrent) {
    voltageDrawer.draw(&voltageDrawerTextDisplay);
    currentDrawer.draw(&currentDrawerTextDisplay);
    batteryLevelDrawer.draw(&batteryLevelDrawerTextDisplay2);
    
  }
  else {
    batteryLevelDrawer.draw(&batteryLevelDrawerTextDisplay);
    currentDrawerTextDisplay.print("    ");
    batteryLevelDrawerTextDisplay2.print("    ");
  }

  clock_loop(now);
}

void blinds_loop() {
  if (now - lastBlindsRead > 50) {
    lastBlindsRead = now;

    auto blindsZero = digitalRead(BLINDS_ZERO_PIN) == 0;
    if (blindsZero != lastBlindsZero) {
      lastBlindsZero = blindsZero;

      pubSubClient.publish(MQTT_BLINDS_ZERO_TOPIC, blindsZero ? "1" : "0", false);
    }
  }
}

void loop() {
  esp_task_wdt_reset();

  auto before = now;
  now = millis();
  
  if (otaUpdateMode) {
    if (now - otaUpdateStart > OTA_UPDATE_TIMEOUT_MILLIS) restart(RESET_ON_OTA_TIMEOUT);
    
    ArduinoOTA.handle();
    return;
  }

  // ac_loop(now);
  motion_loop();
  blinds_loop();

  if (!wifiLoop()) {
    if (lastClockDraw != 0) {
      lcd.clear();
      lcd.print("NO WIFI CONNECTION!");
    }

    lastClockDraw = 0;
    return;
  }

  if (justStarted) {
    justStarted = !onJustStarted();
  }

  timeconfig_loop(now);
  pubSubClientLoop();
  ui_loop();

  if (now - lastOtaHandle > 2000) {
    lastOtaHandle = now;
    ArduinoOTA.handle();
  }
}

/* TOOLS */
bool parseBooleanMessage(byte* payload, unsigned int length, boolean defaultValue) {
  switch (length) {
    case 1: 
      switch (payload[0]) {
        case '1': return true;
        case '0': return false;
        default: return defaultValue;
      }
      break;
    case 2:
    case 3:
      switch (payload[1]) {
        case 'n': return true;
        case 'f': return false;
      }
      break;
    case 4: return true;
    case 5: return false;
    default: return defaultValue;
  }

  return defaultValue;
}
