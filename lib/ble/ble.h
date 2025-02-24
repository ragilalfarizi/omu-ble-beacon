#pragma once

#include "NimBLEAdvertising.h"
#include "NimBLEBeacon.h"
#include "NimBLEDevice.h"
#include "NimBLEEddystoneURL.h"
#include "common.h"
#include "esp_err.h"
#include "id_management.h"
#include <Arduino.h>

class BLE
{
  private:
    BLEAdvertising *_pAdvertising;

    esp_err_t _updateFirmware();
    time_t    _calculateHMOffsetSeconds(time_t seconds, float offset);

  public:
    BLE();

    void begin();
    void setCustomBeacon(BeaconData_t &data, Setting_t &setting);
    void advertise();
};
