#pragma once

#include <Arduino.h>

#define PIN_EN_READ_BATT_VOLT      23
#define PIN_READ_BATT_VOLT         34
#define PIN_EXT_WAKE_UP            35
#define DEEP_SLEEP_TIMER_CYCLE_OFF 2 * 60 // OFF state in seconds
#define uS_TO_S_FACTOR             1000000ULL
#define DEEP_SLEEP_TIMER_CYCLE_ON  1 * 60 // ON State in second

#define BATTERY_THRESHOLD       4.6
#define LOWEST_ANALOG_THRESHOLD 3.0f
#define MONITOR_DURATION        10000 // 10 seconds
#define K_VALUE_OF_ADC_READING  0.001862

void printWakeupReason();

float calculateBatteryVoltage(int16_t &adc);
