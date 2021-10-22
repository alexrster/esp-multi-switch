#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SwitchRelay.h"
#include "MotionSensor.h"
#include "LcdFixedPositionPrint.h"
#include "LcdMarqueeString.h"
#include "version.h"

#define LCD_BACKLIGHT_TIMEOUT_MILLIS  15000
#define LCD_PROGRESS_BAR_CHAR         0xff
#define LCD_PROGRESS_BAR              "" LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR LCD_PROGRESS_BAR_CHAR

#define SWITCH_RELAY_COUNT            8

#define SWITCH_RELAY0_PIN             27 // GPIO27
#define SWITCH_RELAY1_PIN             32 // GPIO32
#define SWITCH_RELAY2_PIN             4  // GPIO4
#define SWITCH_RELAY3_PIN             0  // GPIO0
#define SWITCH_RELAY4_PIN             5  // GPIO5
#define SWITCH_RELAY5_PIN             23 // GPIO23
#define SWITCH_RELAY6_PIN             19 // GPIO19
#define SWITCH_RELAY7_PIN             18 // GPIO18

#define MOTION_SENSOR_PIN             39 // GPIO39

#ifndef INT_LED_PIN
#define INT_LED_PIN                   2  // GPIO2
#endif

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

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

#define SWITCH_STATE_FILENAME         "/state.bin"

#define OTA_UPDATE_TIMEOUT_MILLIS     5*60000

struct config_t {
  bool motion = true;
  bool backlight = true;
  unsigned int backlightTimeout = LCD_BACKLIGHT_TIMEOUT_MILLIS;
} Config;

const String PubSubRestartControlTopic = String(MQTT_RESTART_CONTROL_TOPIC);
const String PubSubCurrentMeetingTopic = String(MQTT_CURRENT_MEETING_TOPIC);
const String PubSubBacklightTopic = String(MQTT_BACKLIGHT_TOPIC);
const String PubSubConfigTopic = String(MQTT_CONFIG_TOPIC);

const String PubSubSwitchTopic[] = { 
  MQTT_SWITCH_STATE_TOPIC(1), 
  MQTT_SWITCH_STATE_TOPIC(2), 
  MQTT_SWITCH_STATE_TOPIC(3), 
  MQTT_SWITCH_STATE_TOPIC(4),
  MQTT_SWITCH_STATE_TOPIC(5),
  MQTT_SWITCH_STATE_TOPIC(6),
  MQTT_SWITCH_STATE_TOPIC(7),
  MQTT_SWITCH_STATE_TOPIC(8),
};

const char* PubSubMotionStateTopic = "balcony/motion";

const PROGMEM uint8_t invertedNumbers[4][8] = {
  { 0x1F, 0x1F, 0x1E, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F }, // 1: o
  { 0x1F, 0x1F, 0x1F, 0x07, 0x0F, 0x0F, 0x1F, 0x1F }, // 2: n
  { 0x1F, 0x10, 0x10, 0x11, 0x11, 0x10, 0x10, 0x1F }, // 3: left half-rect
  { 0x1F, 0x01, 0x01, 0x11, 0x11, 0x01, 0x01, 0x1F }  // 4: right half-rect
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
  otaUpdateStart = 0;

bool 
  needPublishCurrentState = true,
  needPublishMotionState = true,
  spiffsEnabled = true,
  otaUpdateMode = false;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
hd44780_I2Cexp lcd;
MotionSensor motionSensor(MOTION_SENSOR_PIN);

LcdFixedPositionPrint meetingTextDisplay(&lcd, 1);
LcdMarqueeString meetingTextControl(20);

void restart() {
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

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, false)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, MQTT_STATUS_ONLINE_MSG, true);
      pubSubClient.publish(MQTT_VERSION_TOPIC, VERSION, true);
      
      pubSubClient.subscribe(MQTT_RESTART_CONTROL_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_CURRENT_MEETING_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_BACKLIGHT_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_CONFIG_TOPIC, MQTTQOS0);
      pubSubClient.subscribe(MQTT_CURRENT_MEETING_TOPIC, MQTTQOS0);

      for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
        pubSubClient.subscribe(PubSubSwitchTopic[i].c_str(), MQTTQOS0);
      }
    }
    
    return pubSubClient.connected();
  }

  return false;
}

void pubSubClientLoop() {
  if (!pubSubClient.connected() && !reconnectPubSub()) return;

  pubSubClient.loop();
}

bool wifiLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiOnline > WIFI_WATCHDOG_MILLIS) restart();
    else if (now - lastWifiReconnect > WIFI_RECONNECT_MILLIS) {
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
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    lcd.print(switchRelays[i]->getState() == On ? "\1\2" : "\3\4");
    if (i > 0 && (i+1)%4 == 0) lcd.print("  ");
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;
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
  else if (PubSubConfigTopic == topicStr) {
    auto f = SPIFFS.open("/config.json", "w");
    f.write(payload, length);
    f.close();

    restart();
    return;
  }
  else if (PubSubRestartControlTopic == topicStr) {
    restart();
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
}

void onMotionSensor() {
  log_d("On MotionSensor event: state=%s", motionSensor.getState() == None ? "NONE" : "DETECTED");
  needPublishMotionState = true;

  if (motionSensor.getState() == Detected) {
    lastMotionDetected = now;
    setLcdBacklight(true);
  }
}

void setupLcd() {
  log_d("Initialize LCD display");
  lcd.begin(20, 4);

	for(uint8_t i=0; i < 4; i++)
		lcd.createChar(i+1, invertedNumbers[i]);

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
    progress = ((float)currentBytes / totalBytes) * 21;

  lcd.setCursor(0, 1);
  for (int i=0; i<=progress && i<20; i++) {
    lcd.print((char)0xFF);
  }
  
  lcd.setCursor(0, 2);
  lcd.printf("%dB / %dB", currentBytes, totalBytes);

  lcd.setCursor(0, 3);
  lcd.printf("%02d:%02d", minutes, seconds);
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

  restart();
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
  restart();
}

void setup() {
  switchRelays[0] = new SwitchRelayPin(SWITCH_RELAY0_PIN, 0);
  switchRelays[1] = new SwitchRelayPin(SWITCH_RELAY1_PIN, 0);
  switchRelays[2] = new SwitchRelayPin(SWITCH_RELAY2_PIN, 0);
  switchRelays[3] = new SwitchRelayPin(SWITCH_RELAY3_PIN, 0);
  switchRelays[4] = new SwitchRelayPin(SWITCH_RELAY4_PIN, 0);
  switchRelays[5] = new SwitchRelayPin(SWITCH_RELAY5_PIN, 0);
  switchRelays[6] = new SwitchRelayPin(SWITCH_RELAY6_PIN, 0);
  switchRelays[7] = new SwitchRelayPin(SWITCH_RELAY7_PIN, 0);

  if(spiffsEnabled && !SPIFFS.begin(true)){
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
    }

    f.close();
  }

  setupLcd();

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
 
  now = millis();
  lastWifiOnline = now;

  lcd.setCursor(0, 0);
  lcd.print("READY!");
  lcd.noBacklight();
}

void loop() {
  now = millis();

  if (otaUpdateMode) {
    if (now - otaUpdateStart > OTA_UPDATE_TIMEOUT_MILLIS) restart();
    
    ArduinoOTA.handle();
    return;
  }

  if (Config.motion) {
    motionSensor.loop();
  }

  if (now - lastMotionDetected > Config.backlightTimeout) {
    setLcdBacklight(false);
  }

  if (!wifiLoop()) {
    lcd.setCursor(0, 0);
    lcd.print("NO WIFI CONNECTION!");
    return;
  }
  else {
    if (now - lastWifiRSSI > 2000) {
      lastWifiRSSI = now;
      lcd.setCursor(0, 0);
      lcd.print("                    ");
      lcd.setCursor(0, 0);
      lcd.print("WIFI: ");
      lcd.print(WiFi.RSSI());
    }
  }

  pubSubClientLoop();

  if (needPublishCurrentState) {
    needPublishCurrentState = false;

    for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
      needPublishCurrentState |= !pubSubClient.publish(PubSubSwitchTopic[i].c_str(), switchRelays[i]->getState() == On ? "1" : "0");
    }
  }

  if (needPublishMotionState) {
    needPublishMotionState = !pubSubClient.publish(PubSubMotionStateTopic, motionSensor.getState() != None ? "1" : "0");
  }

  meetingTextControl.draw(&meetingTextDisplay);

  ArduinoOTA.handle();
}
