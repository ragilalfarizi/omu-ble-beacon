#pragma once

#include <Arduino.h>

#define PIN_RX_RS485     18
#define PIN_TX_RS485     19
#define FIRMWARE_VERSION "v1.6.0-rc2-210325"

#define STALL_TOLERANCE_MS 5000 // 5 seconds tolerance

struct GPSData_t
{
    float longitude;
    float latitude;
    char  status;
};

// TODO: Remove All Offset.
struct BeaconData_t
{
    GPSData_t gps;
    float     voltageSupply;
    time_t    hourMeter;
    float     hourMeterWithOffset;
};

struct Setting_t
{
    String ID;
    float  thresholdHM;
    float  offsetAnalogInput;
    float  offsetHM;
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

enum SystemState_t
{
    NORMAL = 0,
    ON_BATTERY,
};
