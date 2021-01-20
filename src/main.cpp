#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "version.h"

#define LCD_SDA                       21
#define LCD_SCL                       22

#ifndef SWITCH_RELAY_COUNT
#define SWITCH_RELAY_COUNT            4
#endif

#ifndef SWITCH_RELAY0_PIN
#define SWITCH_RELAY0_PIN             27 // GPIO27
#endif

#ifndef SWITCH_RELAY1_PIN
#define SWITCH_RELAY1_PIN             32 // GPIO32
#endif

#ifndef SWITCH_RELAY2_PIN
#define SWITCH_RELAY2_PIN             4  // GPIO4
#endif

#ifndef SWITCH_RELAY3_PIN
#define SWITCH_RELAY3_PIN             0  // GPIO0
#endif

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
#define MQTT_SWITCH_STATE_TOPIC(x)    MQTT_CLIENT_ID "/" #x
#define MQTT_RESTART_CONTROL_TOPIC    MQTT_CLIENT_ID "/restart"

#define MQTT_STATUS_ONLINE_MSG        "online"
#define MQTT_STATUS_OFFLINE_MSG       "offline"

typedef enum SwitchState : uint8_t { 
  On = 0,  // inversed logic on solid state relays board I own
  Off = 1 
} SwitchState_t;

const String PubSubRestartControlTopic = String(MQTT_RESTART_CONTROL_TOPIC);

const String PubSubSwitchTopic[] = { 
  MQTT_SWITCH_STATE_TOPIC(1), 
  MQTT_SWITCH_STATE_TOPIC(2), 
  MQTT_SWITCH_STATE_TOPIC(3), 
  MQTT_SWITCH_STATE_TOPIC(4) 
};

const uint8_t SwitchRelayPin[] = {
  SWITCH_RELAY0_PIN,
  SWITCH_RELAY1_PIN,
  SWITCH_RELAY2_PIN,
  SWITCH_RELAY3_PIN,
};

unsigned long 
  now = 0,
  lastWifiOnline = 0,
  lastPubSubReconnectAttempt = 0,
  lastUptimeUpdate = 0;

int switchUpdateFlags = 0;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
SwitchState_t switchState[SWITCH_RELAY_COUNT];
// LiquidCrystal_I2C lcd(0x27); // 0x3F ??? 0x27 -- seems to able to control it at least
// LiquidCrystal_I2C lcd(0x3F); // 0x3F ??? 0x27 -- seems to able to control it at least

void onMqttMessage(char* topic, byte* payload, unsigned int length);

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

void setSwitch(uint8_t switchId, SwitchState_t newSwitchState) {
  log_i("Set switch %d to %s, pin%d=%d", switchId, newSwitchState == On ? "ON" : "OFF", SwitchRelayPin[switchId], (uint8_t)newSwitchState);
  digitalWrite(SwitchRelayPin[switchId], (uint8_t)newSwitchState);
  switchState[switchId] = newSwitchState;
  // switchUpdateFlags += pow(2, switchId);
}

void setup() {
  // lcd.begin(16, 2);
  // lcd.backlight();
  // lcd.home();
  // lcd.print("MULTI SOCKET 01 ");  

  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    pinMode(SwitchRelayPin[i], OUTPUT_OPEN_DRAIN);
    setSwitch(i, Off);
  }

  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);

  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.begin();

  pubSubClient.setCallback(onMqttMessage);
  pubSubClient.setServer(MQTT_SERVER_NAME, MQTT_SERVER_PORT);
 
  now = millis();
  lastWifiOnline = now;

  // lcd.setCursor(0, 1);
  // lcd.print("READY!");
  // lastUptimeUpdate = now + 5000;
}

void loop() {
  now = millis();

  if (wifiLoop()) {
    pubSubClientLoop();
    ArduinoOTA.handle();
  }

  // if (switchUpdateFlags > 0) {
  //   for (int i = SWITCH_RELAY_COUNT-1; i >= 0; i--) {
  //     if (switchUpdateFlags - pow(2, i) >= 0) {
  //       pubSubClient.publish(PubSubSwitchTopic[i].c_str(), (uint8_t)switchState[i] > 0 ? "1" : "0");
  //       switchUpdateFlags -= pow(2, i);
  //     }
  //   }
  // }

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

void handleSwitchControlMessage(uint8_t switchId, byte* payload, unsigned int length) {
  switch (length) {
    case 1: 
      switch (payload[0]) {
        case '1': setSwitch(switchId, On); return;
        case '0': setSwitch(switchId, Off); return;
        default: return;
      }
      break;
    case 2:
    case 3:
      switch (payload[1]) {
        case 'n': setSwitch(switchId, On); return;
        case 'f': setSwitch(switchId, Off); return;
      }
      break;
    case 4: setSwitch(switchId, On); return;
    case 5: setSwitch(switchId, Off); return;
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
