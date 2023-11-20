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
PubSub *pubSubClient;

void saveSwitchState() {
  if (!getSpiffsEnabled()) 
    return;

  auto f = SPIFFS.open(SWITCH_STATE_FILENAME, FILE_WRITE);
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    f.write(switchRelays[i]->getState() == SwitchState::On ? '1' : '0');
  }
  f.close();
}

SwitchState getSwitchState(uint8_t switchId) {
  return switchRelays[switchId]->getState();
}

void setSwitchState(uint8_t switchId, SwitchState newSwitchState, bool saveState) {
  log_i("Set switch %d to %s", switchId, newSwitchState == SwitchState::On ? "ON" : "OFF");
  switchRelays[switchId]->setState(newSwitchState);
  switchControlLcdBufReady = false;

  if (saveState)
    saveSwitchState();
}

bool publishSwitchControlState() {
  bool result = false;
  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    result |= !pubSubClient->publish(PubSubSwitchTopic[i].c_str(), switchRelays[i]->getState() == SwitchState::On ? "1" : "0");
  }

  return result;
}

void pubSubSwitchControllerHandleMessage(uint8_t i, byte* payload, unsigned int length) {
  auto actual = switchRelays[i]->getState();
  auto target = parseBooleanMessage(payload, length, (actual == SwitchState::On)) ? SwitchState::On : SwitchState::Off;
  
  if (actual != target) {
    setSwitchState(i, target, true);
    switchControlLcdBufReady = false;
  }
}

void drawSwitchControlLcd(Print *out) {
  if (!switchControlLcdBufReady) {
    uint8_t k = 0;
    for (uint8_t i = 0; i < 10; i++) {
      if (switchRelays[i]->getState() == SwitchState::On) {
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

void setupSwitchControl(PubSub *c) {
  pubSubClient = c;

  switchRelays[0] = new SwitchRelayPin(SWITCH_RELAY0_PIN, LOW, HIGH, OUTPUT);
  switchRelays[1] = new SwitchRelayPin(SWITCH_RELAY1_PIN, LOW, HIGH, OUTPUT);
  switchRelays[2] = new SwitchRelayPin(SWITCH_RELAY2_PIN, LOW, HIGH, OUTPUT);
  switchRelays[3] = new SwitchRelayPin(SWITCH_RELAY3_PIN, LOW, HIGH, OUTPUT);
  switchRelays[4] = new SwitchRelayPin(SWITCH_RELAY4_PIN, LOW, HIGH, OUTPUT);
  switchRelays[5] = new SwitchRelayPin(SWITCH_RELAY5_PIN, LOW, HIGH, OUTPUT);
  switchRelays[6] = new SwitchRelayPin(SWITCH_RELAY6_PIN, LOW, HIGH, OUTPUT);
  switchRelays[7] = new SwitchRelayPin(SWITCH_RELAY7_PIN, LOW, HIGH, OUTPUT);
  switchRelays[8] = new SwitchRelayPin(SWITCH_RELAY8_PIN, LOW, HIGH, OUTPUT);
  switchRelays[9] = new SwitchRelayPin(SWITCH_RELAY9_PIN, LOW, HIGH, OUTPUT);

  for (uint8_t i = 0; i < SWITCH_RELAY_COUNT; i++) {
    pubSubClient->subscribe(PubSubSwitchTopic[i].c_str(), MQTTQOS0, [i](uint8_t* d, unsigned int l) { pubSubSwitchControllerHandleMessage(i, d, l); });
  }
}
