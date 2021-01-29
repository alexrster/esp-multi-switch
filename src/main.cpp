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
#include "version.h"

#define LCD_BACKLIGHT_TIMEOUT_MILLIS  15000

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
#define WIFI_RECONNECT_MILLIS         1000
#define WIFI_WATCHDOG_MILLIS          5*60000

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
#define MQTT_RESTART_CONTROL_TOPIC    MQTT_CLIENT_ID "/restart"
#define MQTT_SWITCH_STATE_TOPIC(x)    MQTT_CLIENT_ID "/" #x

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

#define SWITCH_STATE_FILENAME "/state.bin"

const String PubSubRestartControlTopic = String(MQTT_RESTART_CONTROL_TOPIC);

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

SwitchRelay* switchRelays[SWITCH_RELAY_COUNT];

unsigned long 
  now = 0,
  lastWifiOnline = 0,
  lastPubSubReconnectAttempt = 0,
  lastUptimeUpdate = 0,
  lastMotionDetected = 0;

bool 
  needPublishCurrentState = true,
  spiffsEnabled = true;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
hd44780_I2Cexp lcd;
MotionSensor motionSensor(MOTION_SENSOR_PIN);

bool reconnectPubSub() {
  if (now - lastPubSubReconnectAttempt > MQTT_RECONNECT_MILLIS) {
    lastPubSubReconnectAttempt = now;

    if (pubSubClient.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS_TOPIC, MQTTQOS0, true, MQTT_STATUS_OFFLINE_MSG, false)) {
      pubSubClient.publish(MQTT_STATUS_TOPIC, VERSION, true);
      
      pubSubClient.subscribe(MQTT_RESTART_CONTROL_TOPIC, MQTTQOS0);

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
    if(now - lastWifiOnline > WIFI_WATCHDOG_MILLIS) ESP.restart();

    return false;
  }
  
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

void handleSwitchControlMessage(uint8_t switchId, byte* payload, unsigned int length) {
  switch (length) {
    case 1: 
      switch (payload[0]) {
        case '1': setSwitchState(switchId, On); return;
        case '0': setSwitchState(switchId, Off); return;
        default: return;
      }
      break;
    case 2:
    case 3:
      switch (payload[1]) {
        case 'n': setSwitchState(switchId, On); return;
        case 'f': setSwitchState(switchId, Off); return;
      }
      break;
    case 4: setSwitchState(switchId, On); return;
    case 5: setSwitchState(switchId, Off); return;
    default: return;
  }  
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;
  log_d("Handle message from '%s':\n%s", topic, (char*)payload);

  String topicStr = topic;
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    if (PubSubSwitchTopic[i] == topicStr) {
      handleSwitchControlMessage(i, payload, length);
      return;
    }
  }
  
  if (PubSubRestartControlTopic == topicStr) {
    ESP.restart();
  }
}

void onMotionSensor() {
  log_d("On MotionSensor event: state=%s", motionSensor.getState() == None ? "NONE" : "DETECTED");
  lcd.setCursor(0, 2);

  if (motionSensor.getState() == Detected) {
    lcd.backlight();
    lcd.print("Motion DETECTED!");
    lastMotionDetected = now;
  }
  else
    lcd.print("                    ");
}

void setupLcd() {
  int status = lcd.begin(20, 4);
  log_d("LCD init: status=%d", status);

  lcd.backlight();
  lcd.home();
  lcd.print(WIFI_HOSTNAME);
}

void setup() {
  switchRelays[0] = new SwitchRelayPin(SWITCH_RELAY0_PIN, 0);
  switchRelays[1] = new SwitchRelayMock();
  switchRelays[2] = new SwitchRelayMock();
  switchRelays[3] = new SwitchRelayMock();
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

  setupLcd();

  motionSensor.onChanged(onMotionSensor);

  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.begin();

  pubSubClient.setCallback(onMqttMessage);
  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);
 
  now = millis();
  lastWifiOnline = now;

  lcd.setCursor(0, 1);
  lcd.print("READY!");
}

void loop() {
  now = millis();

  motionSensor.loop();

  if (now - lastMotionDetected > LCD_BACKLIGHT_TIMEOUT_MILLIS) {
    lcd.noBacklight();
  }

  if (!wifiLoop()) {
    return;
  }

  pubSubClientLoop();

  if (needPublishCurrentState) {
    needPublishCurrentState = false;

    for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
      needPublishCurrentState |= !pubSubClient.publish(PubSubSwitchTopic[i].c_str(), switchRelays[i]->getState() == On ? "1" : "0");
    }
  }

  ArduinoOTA.handle();

#if 0
  if (lastUptimeUpdate - now > 1000) {
    lastUptimeUpdate = now;

    char buf[32];
    unsigned int 
      hours = now % (1000 * 60 * 60),
      minutes = (now - hours * 1000 * 60 * 60) % (1000 * 60),
      seconds = (now - hours * 1000 * 60 * 60 - minutes * 1000 * 60) % 1000;
    // sprintf(buf, "UPTIME: %02d:%02d:%02d", hours, minutes, seconds);
    // lcd.print(buf);
  }
#endif
}
