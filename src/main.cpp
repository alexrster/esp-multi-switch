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
#include "SwitchRelay.h"
#include "MotionSensor.h"
#include "LcdFixedPositionPrint.h"
#include "LcdMarqueeString.h"
#include "LcdBigDigits.h"
#include "LcdSymbolAlert.h"
#include "LcdBigSymbolAlert.h"
#include "LcdPrintDrawer.h"
#include "time.h"
#include "reset_info.h"
#include "version.h"

#define LCD_BACKLIGHT_TIMEOUT_MILLIS  15000
#define LCD_PROGRESS_BAR_CHAR         0xff
#define LCD_PROGRESS_BAR              "" LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR

#define SWITCH_RELAY_COUNT            10

#define SWITCH_RELAY0_PIN             25
#define SWITCH_RELAY1_PIN             32
#define SWITCH_RELAY2_PIN             4
#define SWITCH_RELAY3_PIN             13
#define SWITCH_RELAY4_PIN             16
#define SWITCH_RELAY5_PIN             23
#define SWITCH_RELAY6_PIN             19
#define SWITCH_RELAY7_PIN             18
#define SWITCH_RELAY8_PIN             17
#define SWITCH_RELAY9_PIN             33

#define MOTION_SENSOR_PIN             39

#ifndef INT_LED_PIN
#define INT_LED_PIN                   2
#endif

#define BLINDS_ZERO_PIN               26

#define APP_INIT_DELAY_MILLIS         2500

#define WIFI_SSID                     "qx.zone"
#define WIFI_PASSPHRASE               "1234Qwer-"
#define WIFI_RECONNECT_MILLIS         10000
#define WIFI_WATCHDOG_MILLIS          60000

#ifndef WIFI_HOSTNAME
#define WIFI_HOSTNAME                 "multiswitch-01"
#endif

#define MQTT_SERVER_NAME              "ns2.in.qx.zone"
#define MQTT_SERVER_PORT              1883
#define MQTT_USERNAME                 NULL
#define MQTT_PASSWORD                 NULL
#define MQTT_RECONNECT_MILLIS         5000

#ifndef MQTT_CLIENT_ID
#define MQTT_CLIENT_ID                WIFI_HOSTNAME
#endif

#define MQTT_STATUS_TOPIC             MQTT_CLIENT_ID "/status"
#define MQTT_VERSION_TOPIC            MQTT_CLIENT_ID "/version"
#define MQTT_RESTART_CONTROL_TOPIC    MQTT_CLIENT_ID "/restart"
#define MQTT_SWITCH_STATE_TOPIC(x)    MQTT_CLIENT_ID "/" #x
#define MQTT_BACKLIGHT_TOPIC          MQTT_CLIENT_ID "/backlight"
#define MQTT_CONFIG_TOPIC             MQTT_CLIENT_ID "/config"
#define MQTT_CURRENT_MEETING_TOPIC    "ay/calendar/events/current/title"

#define MQTT_BLINDS_ZERO_TOPIC        MQTT_CLIENT_ID "/blinds-zero"

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

#define SWITCH_STATE_FILENAME         "/state.bin"
#define SW_RESET_REASON_FILENAME      "/swresr.bin"

#define GMT_OFFSET_SEC                2*60*60
#define DAYLIGHT_OFFSET_SEC           1*60*60

#define NTP_SERVER                    "ua.pool.ntp.org"
#define NTP_UPDATE_MS                 1*60000                                  // every 1 minute

#define OTA_UPDATE_TIMEOUT_MILLIS     5*60000

#define WDT_TIMEOUT_SEC               20
#define UI_FORCED_REDRAW_MS           15*1000

typedef std::function<void(void)> ActionCallback;

struct config_t {
  bool motion = true;
  bool backlight = true;
  unsigned int backlightTimeout = LCD_BACKLIGHT_TIMEOUT_MILLIS;
  unsigned long wifi_reconnect_ms = WIFI_RECONNECT_MILLIS;
  unsigned long wifi_watchdog_ms = WIFI_WATCHDOG_MILLIS;
  unsigned int daylight_offset_sec = DAYLIGHT_OFFSET_SEC;
  unsigned short wdt_timeout_sec = WDT_TIMEOUT_SEC;
} Config;

const String PubSubRestartControlTopic = String(MQTT_RESTART_CONTROL_TOPIC);
const String PubSubCurrentMeetingTopic = String(MQTT_CURRENT_MEETING_TOPIC);
const String PubSubBacklightTopic = String(MQTT_BACKLIGHT_TOPIC);

const String PubSubSwitchTopic[] = { 
  MQTT_SWITCH_STATE_TOPIC(1), 
  MQTT_SWITCH_STATE_TOPIC(2), 
  MQTT_SWITCH_STATE_TOPIC(3), 
  MQTT_SWITCH_STATE_TOPIC(4),
  MQTT_SWITCH_STATE_TOPIC(5),
  MQTT_SWITCH_STATE_TOPIC(6),
  MQTT_SWITCH_STATE_TOPIC(7),
  MQTT_SWITCH_STATE_TOPIC(8),
  MQTT_SWITCH_STATE_TOPIC(9),
  MQTT_SWITCH_STATE_TOPIC(10),
};

const char* PubSubMotionStateTopic = "balcony/motion";

const PROGMEM uint8_t lcdCustomChars[8][8] = {
  { 0x1F, 0x1F, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F }, // 1: o
  { 0x1F, 0x1F, 0x1F, 0x07, 0x0F, 0x0F, 0x1F, 0x1F }, // 2: n
  { 0x1F, 0x10, 0x10, 0x11, 0x11, 0x10, 0x10, 0x1F }, // 3: left half-rect
  { 0x1F, 0x01, 0x01, 0x11, 0x11, 0x01, 0x01, 0x1F }, // 4: right half-rect
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x14 }, // 5: wifi < 15% (l)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x15 }, // 6: wifi < 40% (l)
  { 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10 }, // 7: wifi < 55% (r)
  { 0x00, 0x00, 0x01, 0x05, 0x15, 0x15, 0x15, 0x15 }  // 8: wifi < 85% (r)
};

SwitchRelay* switchRelays[SWITCH_RELAY_COUNT];

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
  otaUpdateStart = 0,
  runCounter = 0;

bool 
  needPublishCurrentState = true,
  needPublishMotionState = true,
  spiffsEnabled = true,
  justStarted = true,
  otaUpdateMode = false,
  lastBlindsZero = false;

RESET_REASON
  reset_reason[2];

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

LcdFixedPositionPrint meetingTextDisplay(&lcd, 2);
LcdMarqueeString meetingTextControl(20);

// LcdFixedPositionPrint hallAlertPrint(&lcd, 0, 18);
LcdBigSymbolAlert hallAlert(&lcd, 10, 17);

// LcdFixedPositionPrint entranceAlertPrint(&lcd, 0, 19);
LcdBigSymbolAlert entranceAlert(&lcd, 11, 17);

LcdFixedPositionPrint powerSourceDisplay(&lcd, 0, 16);
LcdFixedPositionPrint voltageDisplay(&lcd, 1, 16);

LcdPrintDrawer powerSourceDrawer(&powerSourceDisplay);
LcdPrintDrawer voltageDrawer(&voltageDisplay);

void restart(char code) {
  // if (spiffsEnabled) {
  //   auto f = SPIFFS.open(SW_RESET_REASON_FILENAME, FILE_WRITE);
  //   f.write(code);
  //   f.flush();
  //   f.close();
  // }

  preferences.putULong("SW_RESET_UPTIME", millis());
  preferences.putUChar("SW_RESET_REASON", code);
  preferences.end();

  delay(200);
  ESP.restart();
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

      for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
        pubSubClient.subscribe(PubSubSwitchTopic[i].c_str(), MQTTQOS0);
      }
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

void saveSwitchState() {
  if (!spiffsEnabled) 
    return;

  auto f = SPIFFS.open(SWITCH_STATE_FILENAME, FILE_WRITE);
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    f.write(switchRelays[i]->getState() == On ? '1' : '0');
  }
  f.close();
}

void setSwitchState(uint8_t switchId, SwitchState_t newSwitchState, bool saveState = true) {
  log_i("Set switch %d to %s", switchId, newSwitchState == On ? "ON" : "OFF");
  switchRelays[switchId]->setState(newSwitchState);

  if (saveState)
    saveSwitchState();
}

boolean parseBooleanMessage(byte* payload, unsigned int length, boolean defaultValue = false) {
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

void updateSwitchControlLcd() {
  lcd.setCursor(0, 3);
  for (uint8_t i = 0; i < 8; i++) {
    if (switchRelays[i]->getState() == On) {
      lcd.write(0);
      lcd.write(1);
    }
    else {
      lcd.print("\2\3");
    }
    if (i > 0 && (i+1)%4 == 0) lcd.print("  ");
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;
  if (length > 255) return;
  log_d("Handle message from '%s':\n%s", topic, (char*)payload);

  String topicStr = topic;
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    if (PubSubSwitchTopic[i] == topicStr) {
      setSwitchState(i, parseBooleanMessage(payload, length, (switchRelays[i]->getState() == On)) ? On : Off);
      updateSwitchControlLcd();
      return;
    }
  }
  
  if (PubSubBacklightTopic == topicStr) {
    setLcdBacklight(parseBooleanMessage(payload, length));
  }
  else if (topicStr.equals(MQTT_CONFIG_TOPIC "/set")) {
    auto f = SPIFFS.open("/config.json", "w");
    f.write(payload, length);
    f.close();

    restart(RESET_ON_CONFIG_UPDATE);
    return;
  }
  else if (PubSubRestartControlTopic == topicStr) {
    restart(RESET_ON_MQTT_RESET_TOPIC);
  }
  else if (PubSubCurrentMeetingTopic == topicStr) {
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
      if (payload[0] == 'l')   powerSourceDrawer.print("LINE");
      else if (payload[0] == 'b') powerSourceDrawer.print("BATT");
      else if (payload[0] == 'g') powerSourceDrawer.print(" GEN");
      else powerSourceDrawer.print("   -");
    }
  }
  else if (topicStr.equals("dev/power-line/voltage")) {
    if (length == 3) {
      char voltage[3];
      strncpy(voltage, (const char*)payload, 3);
      voltageDrawer.printf("%sV", voltage);
    }
    else {
      voltageDrawer.print("   -");
    }
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

void IRAM_ATTR blindsZeroISR() {
  lastBlindsZero = true;
}

#ifdef AC_DETECTOR
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
  lcd.begin(20, 4);

	for(uint8_t i=0; i < 8; i++)
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

  switchRelays[0] = new SwitchRelayPin(SWITCH_RELAY0_PIN, 0);
  switchRelays[1] = new SwitchRelayPin(SWITCH_RELAY1_PIN, 0);
  switchRelays[2] = new SwitchRelayPin(SWITCH_RELAY2_PIN, 0);
  switchRelays[3] = new SwitchRelayPin(SWITCH_RELAY3_PIN, 0);
  switchRelays[4] = new SwitchRelayPin(SWITCH_RELAY4_PIN, 0);
  switchRelays[5] = new SwitchRelayPin(SWITCH_RELAY5_PIN, 0);
  switchRelays[6] = new SwitchRelayPin(SWITCH_RELAY6_PIN, 0);
  switchRelays[7] = new SwitchRelayPin(SWITCH_RELAY7_PIN, 0);
  switchRelays[8] = new SwitchRelayPin(SWITCH_RELAY8_PIN, 0);
  switchRelays[9] = new SwitchRelayPin(SWITCH_RELAY9_PIN, 0);

  if(spiffsEnabled && !SPIFFS.begin(true)) {
    log_e("SPIFFS mount failed");
    spiffsEnabled = false;
  }

  if (spiffsEnabled && SPIFFS.exists("/state.bin")) {
    char b[SWITCH_RELAY_COUNT];
    auto f = SPIFFS.open("/state.bin");
    f.readBytes(b, SWITCH_RELAY_COUNT);

    for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
      switchRelays[i]->setState(b[i] == '1' ? On : Off);
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

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("READY!");
  lcd.setCursor(0, 1);
  lcd.print("RUN #");
  lcd.print(runCounter);

  delay(preferences.getUInt("RUN_N_MSG_MS", 1000));
  lcd.noBacklight();
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
      lcd.print("                                        ");
      lastTimeMin = -1;
    }

    lastClockDraw = now;

    if (lastTimeConfig != now) {
      timeinfo.tm_sec += 1;
      mktime(&timeinfo);
    }

    if (timeinfo.tm_min != lastTimeMin) {
      lastTimeMin = timeinfo.tm_min;
      showBigNumberFixed(&lcd, timeinfo.tm_hour, 2, 0);
      showBigNumberFixed(&lcd, timeinfo.tm_min, 2, 8);
    }

    clock_separator = clock_separator == ' ' ? '.' : ' ';
    lcd.setCursor(7, 0);
    lcd.print(clock_separator);
    lcd.setCursor(7, 1);
    lcd.print(clock_separator);
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

void publish_state_loop() {
  if (needPublishCurrentState) {
    needPublishCurrentState = false;

    for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
      needPublishCurrentState |= !pubSubClient.publish(PubSubSwitchTopic[i].c_str(), switchRelays[i]->getState() == On ? "1" : "0");
    }
  }

  if (needPublishMotionState) {
    needPublishMotionState = !pubSubClient.publish(PubSubMotionStateTopic, motionSensor.getState() != None ? "1" : "0");
  }

}

void pubSubClientLoop() {
  if (!pubSubClient.connected() && !reconnectPubSub()) return;

  publish_state_loop();

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
  if (now - lastUiRedraw > UI_FORCED_REDRAW_MS) {
    lastUiRedraw = now;
    updateSwitchControlLcd();
  }

  meetingTextControl.draw(&meetingTextDisplay);
  hallAlert.draw();
  entranceAlert.draw();
  powerSourceDrawer.draw();
  voltageDrawer.draw();
}

void blinds_loop() {
  if (now - lastBlindsRead > 50) {
    lastBlindsRead = now;

    auto blindsZero = digitalRead(BLINDS_ZERO_PIN) == 0;
    if (blindsZero != lastBlindsZero) {
      lastBlindsZero = blindsZero;

      pubSubClient.publish(MQTT_BLINDS_ZERO_TOPIC, blindsZero ? "1" : "0", false);
    }

    // setSwitchState(8, On);
    // setSwitchState(9, On);
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

  clock_loop(now);
  pubSubClientLoop();
  ui_loop();

  ArduinoOTA.handle();
}
