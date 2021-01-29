#ifndef MOTION_SENSOR_H
#define MOTION_SENSOR_H

#include <Arduino.h>
#include <functional>

typedef enum MotionState { 
  None = 0,
  Detected
} MotionState_t;

typedef std::function<void(void)> MotionSensorEventCallback;

class MotionSensor
{
  public:
    MotionSensor(uint8_t pin) : pin(pin)
    {
      pinMode(pin, INPUT);
      setState(digitalRead(pin));
    }

    MotionState_t getState()
    {
      return state;
    }

    void onChanged(MotionSensorEventCallback cb)
    {
      onMotionStateCallback = cb;
    }

    void loop() {
      int s = digitalRead(pin);
      if (s != (int)state) {
        setState(s);

        if (onMotionStateCallback != NULL)
          onMotionStateCallback();
      }
    }

  private:
    uint8_t pin;
    MotionState_t state = None, lastState = None;
    MotionSensorEventCallback onMotionStateCallback = NULL;

    void setState(int value) {
      log_d("Motion sensor @%d: value=%d", pin, value);

      lastState = state;
      state = value == 0 ? None : Detected;
    }
};

#endif