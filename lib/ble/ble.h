#pragma once

#include "NimBLEAdvertising.h"
#include "NimBLEBeacon.h"
#include "NimBLEDescriptor.h"
#include "NimBLEDevice.h"
#include "NimBLEEddystoneURL.h"
#include "NimBLEServer.h"
#include "WiFiGeneric.h"
#include "common.h"
#include "esp_err.h"
#include "id_management.h"
#include <Arduino.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>

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

    esp_err_t   _updateFirmware();
    time_t      _calculateHMOffsetSeconds(time_t seconds, float offset);
    void        _setNetworkConfig();
    static void _WiFiAPConnectedCB(arduino_event_id_t e);
    static void _WiFiAPDisconnectedCB(arduino_event_id_t e);

  public:
    BLE();

    void begin();
    void setCustomBeacon(BeaconData_t &data, Setting_t &setting);
    void advertiseBeacon();
    void startServerOTA();
    void startHTTPServer();
    void startWiFi();
    bool connectToWiFi();

    static bool isConnectedToWiFi;
    WiFiClass  *_wifi; // For WebServer
    WebServer  *_server;

    // Set Static IP
    IPAddress localIP;
    IPAddress gateway;
    IPAddress subnet;
    IPAddress primaryDNS;
    IPAddress secondaryDNS;
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
        Serial.println("Device is Connected to ESP32 BLE!");
        // _bleInstance->isConnected = true;

        // start WiFi AP
        _bleInstance->startWiFi();

        // Start HTTP Server
        // _bleInstance->startHTTPServer();
    }

    void onDisconnect(NimBLEServer *pServer) override
    {
        Serial.println("Device is disconnected from ESP32 BLE!");

        delete _bleInstance->_wifi;
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
