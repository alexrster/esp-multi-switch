#ifndef PUSH_BUTTON_H
#define PUSH_BUTTON_H

#include <Arduino.h>
#include <functional>

typedef enum ButtonState { 
  Off = 0,
  On
} ButtonState_t;


class PushButton
{
  public:
    typedef std::function<void(PushButton*)> PushButtonEventCallback;

    PushButton(uint8_t pin, unsigned int threshold_ms = 100) : pin(pin), threshold_ms(threshold_ms)
    {
      pinMode(pin, INPUT);
      setState(digitalRead(pin));
    }

    ButtonState_t getState()
    {
      return state;
    }

    void onChanged(PushButtonEventCallback cb)
    {
      onStateCallback = cb;
    }

    void loop(unsigned long now) {
      int s = digitalRead(pin);
      if (s != (int)state) {
        if (newState != s) {
          newState = s;
          newState_ms = now;
        }
        else if (now - newState_ms > threshold_ms) {
          setState(s);

          if (onStateCallback != NULL)
            onStateCallback(this);
        }
      }
    }

  private:
    uint8_t pin;
    unsigned int threshold_ms;
    unsigned long newState_ms = 0;
    ButtonState_t state = Off, lastState = Off, newState = Off;
    PushButtonEventCallback onStateCallback = NULL;

    void setState(int value) {
      log_d("PushButton @%d: value=%d", pin, value);

      lastState = state;
      state = value == 0 ? Off : On;
    }
};

#endif