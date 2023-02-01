#ifndef __SWITCHCONTROLLER_H
#define __SWITCHCONTROLLER_H

#include "app.h"
#include "SwitchRelay.h"
#include "pubsub.h"

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

#define SWITCH_STATE_FILENAME         "/state.bin"

void setupSwitchControl(PubSub *pubSubClient);
SwitchState_t getSwitchState(uint8_t switchId);
void setSwitchState(uint8_t switchId, SwitchState_t newSwitchState, bool saveState = true);
bool publishSwitchControlState();
void drawSwitchControlLcd(Print *out);

#endif