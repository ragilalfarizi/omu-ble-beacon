#pragma once

#include "NimBLEAdvertising.h"
#include "NimBLEBeacon.h"
#include "NimBLEDescriptor.h"
#include "NimBLEDevice.h"
#include "NimBLEEddystoneURL.h"
#include "NimBLEServer.h"
#include "common.h"
#include "esp_err.h"
#include "id_management.h"
#include <Arduino.h>

#define SERVICE_UUID        "6f7e9280-e3d8-47b7-9733-ef0ffffc65fd"
#define CHARACTERISTIC_UUID "2233b614-a358-4551-8ad1-b113764c68f3"

class BLE
{
  private:
    static const char *_SSID;
    static const char *_password;

    // For Beacon
    BLEAdvertising *_pAdvertising;

    // For Server
    NimBLEServer         *_pServer;
    NimBLEService        *_pService;
    NimBLECharacteristic *_pCharacteristic;
    NimBLEAdvertising    *_pAdvertisingOTA;

    esp_err_t _updateFirmware();
    time_t    _calculateHMOffsetSeconds(time_t seconds, float offset);

  public:
    BLE();

    void begin();
    void setCustomBeacon(BeaconData_t &data, Setting_t &setting);
    void advertiseBeacon();
    void startServerOTA();
    bool connectToWiFi();

    bool isConnected = false;
};

// Custom BLE Server Callback Class
class OTAServerCallback : public NimBLEServerCallbacks
{
  private:
    BLE *_bleInstance;

  public:
    OTAServerCallback(BLE *bleInstance) : _bleInstance(bleInstance)
    {
    }

    void onConnect(NimBLEServer *pServer) override
    {
        Serial.println("Device Connected!");
        _bleInstance->isConnected = true;
    }

    void onDisconnect(NimBLEServer *pServer) override
    {
        Serial.println("Device Disconnected!");
        _bleInstance->isConnected = false;
    }
};

// Custom BLE Callback Class
class OTACharacteristicCallback : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *pCharacteristic) override
    {
        std::string value = pCharacteristic->getValue();
        Serial.print("Received: ");
        Serial.println(value.c_str());
    }
};
