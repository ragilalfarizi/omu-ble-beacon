#pragma once

#include "common.h"
#include "hour_meter_manager.h"
#include "id_management.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <NimBLEDevice.h>
#include <Update.h>
#include <WiFi.h>
#include <esp_err.h>

#define SERVICE_OTA_UUID     "6f7e9280-e3d8-47b7-9733-ef0ffffc65fd"
#define CHAR_OTA_UUID        "2233b614-a358-4551-8ad1-b113764c68f3"
#define SERVICE_DIS_UUID     "180A"
#define CHAR_DIS_FW_VER_UUID "2A26"
#define OTA_WIFI_DURATION    (60 * 60 * 1000) // 2 minutes

extern BeaconData_t data;
extern Setting_t    setting;
extern HourMeter   *hm;

class BLE
{
  private:
    String      _WIFI_SSID;
    const char *_WIFIpwd;

    String _BT_SSID;

    // For Beacon
    BLEAdvertising *_pAdvertising;

    // For Server
    NimBLEServer         *_pServer;
    NimBLEService        *_pOTAService;
    NimBLEService        *_pDISService;
    NimBLECharacteristic *_pFirmwareChar;
    NimBLECharacteristic *_pCharacteristic;
    NimBLEAdvertising    *_pAdvertisingOTA;
    bool                  _isOTASuccess = false;
    bool                  _isFileBin    = false;

    esp_err_t   _updateFirmware();
    time_t      _calculateHMOffsetSeconds(time_t seconds, float offset);
    void        _setNetworkConfig();
    static void _WiFiAPConnectedCB(arduino_event_id_t e);
    static void _WiFiAPDisconnectedCB(arduino_event_id_t e);
    static void _handleSerializingDataJSON(AsyncWebServerRequest *request);

  public:
    BLE();

    enum OTAState
    {
        IDLE = 0,
        OTA_ON
    };

    void        begin();
    void        setCustomBeacon(BeaconData_t &data, Setting_t &setting);
    void        advertiseBeacon();
    void        startServerOTA();
    void        startHTTPServer();
    void        stopHTTPServer();
    void        startWiFi();
    bool        connectToWiFi();
    String      getMonitoringData(BeaconData_t &data, Setting_t &setting);
    String      getSettingData(Setting_t &setting);
    void        stopAdvertiseOTA();
    void        setWiFiSSID(String newSSID);
    static void restartTask(void *param);

    static bool     isConnectedToWiFi;
    WiFiClass      *_wifi   = nullptr; // For WebServer
    AsyncWebServer *_server = nullptr;

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

        if (NimBLEDevice::getAdvertising()->isAdvertising())
        {
            NimBLEDevice::getAdvertising()->stop();
            Serial.println("BLE advertising stopped.");
        }

        // start WiFi AP
        _bleInstance->startWiFi();
    }

    void onDisconnect(NimBLEServer *pServer) override
    {
        Serial.println("Device is disconnected from ESP32 BLE!");

        NimBLEDevice::getAdvertising()->start();
        Serial.println("BLE advertising restarted.");

        if (_bleInstance->_wifi)
        {
            _bleInstance->_wifi->disconnect(true); // disconnect WiFi
            vTaskDelay(pdMS_TO_TICKS(500));

            // save the new name before disconnect
            _bleInstance->setWiFiSSID(setting.ID);

            _bleInstance->_wifi->mode(WIFI_OFF); // Turn off WiFi
            vTaskDelay(pdMS_TO_TICKS(500));

            delete _bleInstance->_wifi;
            vTaskDelay(pdMS_TO_TICKS(100));
            _bleInstance->_wifi = nullptr;
        }
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
