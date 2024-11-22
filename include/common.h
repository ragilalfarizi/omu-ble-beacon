#pragma once

#include <Arduino.h>

#define PIN_RX_RS485 18
#define PIN_TX_RS485 19

#define STALL_TOLERANCE_MS 5000 // 5 seconds tolerance

struct GPSData_t
{
    float longitude;
    float latitude;
    char status;
};

struct BeaconData_t
{
    GPSData_t gps;
    float voltageSupply;
    time_t hourMeter;
};

struct Setting_t
{
    String ID;
    uint8_t thresholdHM;
    float offsetAnalogInput;
};

enum ListID_t
{
    AC = 0,
    CC,
    CD,
    CE,
    CG,
    CO,
    CT,
    DP,
    FL,
    LS,
    MC,
    MS,
    PP,
    ST,
    TH,
    TL,
    WT,
    XCD,
    XCE,
    XCT,
    XDP,
    XMC,
    XST,
    XWT,
    UNKNOWN,
};