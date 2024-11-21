#pragma once

#include <Arduino.h>

#define PIN_EN_READ_BATT_VOLT 23
#define PIN_READ_BATT_VOLT 34
#define PIN_EXT_WAKE_UP 35

#define BATTERY_THRESHOLD 3.5
#define LOWEST_ANALOG_THRESHOLD 2.0f
#define MONITOR_DURATION 10000 // 10 seconds
#define K_VALUE_OF_ADC_READING 0.001724

void printWakeupReason();

float calculateBatteryVoltage(int16_t& adc);
