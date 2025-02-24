#pragma once

#include <Arduino.h>

#define PIN_DIGITAL_OUT_1 5
#define PIN_DIGITAL_OUT_2 2
#define PIN_DIGITAL_OUT_3 4
#define PIN_DIGITAL_OUT_4 15

// typedef uint8_t OutputPin;

class PinDebug
{
  private:
    uint8_t pins[4];
    uint8_t pinCount;

  public:
    PinDebug(std::initializer_list<uint8_t> pinList)
    {
        pinCount = (pinList.size() > 4) ? 4 : pinList.size(); // Max 4 pins
        int i    = 0;
        for (uint8_t pin : pinList)
        {
            if (i < 4)
            {
                pins[i] = pin;
                pinMode(pins[i], OUTPUT);
                digitalWrite(pins[i], HIGH);
                i++;
            }
        }
    }

    void togglePin(uint8_t pin)
    {
        for (int i = 0; i < 4; i++)
        {
            if (pins[i] == pin)
            {
                digitalWrite(pin, !digitalRead(pin)); // Toggle state
                Serial.printf("pin: %d output: %d\n", pin, digitalRead(pin));
                break; // Stop looping once pin is found
            }
        }
    }
};
