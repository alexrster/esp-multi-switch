#include "SwitchController.h"
#include <SPIFFS.h>

#define MQTT_SWITCH_STATE_TOPIC(x)              MQTT_CLIENT_ID "/" #x

SwitchRelay* switchRelays[SWITCH_RELAY_COUNT];

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

char switchControlLcdBuf[18] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
bool switchControlLcdBufReady = false;

void setupSwitchController() {
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
}

void saveSwitchState() {
  if (!getSpiffsEnabled()) 
    return;

  auto f = SPIFFS.open(SWITCH_STATE_FILENAME, FILE_WRITE);
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    f.write(switchRelays[i]->getState() == On ? '1' : '0');
  }
  f.close();
}

SwitchState_t getSwitchState(uint8_t switchId) {
  return switchRelays[switchId]->getState();
}

void setSwitchState(uint8_t switchId, SwitchState_t newSwitchState, bool saveState) {
  log_i("Set switch %d to %s", switchId, newSwitchState == On ? "ON" : "OFF");
  switchRelays[switchId]->setState(newSwitchState);
  switchControlLcdBufReady = false;

  if (saveState)
    saveSwitchState();
}

void pubSubSwitchControllerSubscribe(PubSubClient *pubSubClient) {
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    pubSubClient->subscribe(PubSubSwitchTopic[i].c_str(), MQTTQOS0);
  }
}

bool pubSubSwitchControllerPublishState(PubSubClient *pubSubClient) {
  bool result = false;
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    result |= !pubSubClient->publish(PubSubSwitchTopic[i].c_str(), switchRelays[i]->getState() == On ? "1" : "0");
  }

  return result;
}

bool pubSubSwitchControllerHandleMessage(String topicStr, byte* payload, unsigned int length) {
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    if (PubSubSwitchTopic[i] == topicStr) {
      auto actual = switchRelays[i]->getState();
      auto target = parseBooleanMessage(payload, length, (actual == On)) ? On : Off;
      
      if (actual != target) {
        setSwitchState(i, target, true);
        switchControlLcdBufReady = false;
      }
      
      return true;
    }
  }

  return false;
}

void drawSwitchControlLcd(Print *out) {
  if (!switchControlLcdBufReady) {
    uint8_t k = 0;
    for (uint8_t i = 0; i < 10; i++) {
      if (switchRelays[i]->getState() == On) {
        switchControlLcdBuf[i+k] = 1;
      }
      else {
        switchControlLcdBuf[i+k] = 0;
      }
      if (i == 3 || i == 7) {
        k++;
        switchControlLcdBuf[i+k] = ' ';
      }
    }

    switchControlLcdBufReady = true;

    out->write(switchControlLcdBuf, 15);
  }
}