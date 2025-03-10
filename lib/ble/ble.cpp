#include "ble.h"

bool                BLE::isConnectedToWiFi = false;
extern BeaconData_t data;
extern Setting_t    setting;

/* PUBLIC METHOD */
BLE::BLE()
{
    _WIFI_SSID = "WIFI_" + setting.ID; // name + wifi
    _WIFIpwd   = "1234567890";

    _BT_SSID = "BT_" + setting.ID;

    Serial.println(_WIFI_SSID);
    Serial.println(_BT_SSID);
}

void BLE::begin()
{
    BLEDevice::init(_BT_SSID.c_str());
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    _pAdvertising = BLEDevice::getAdvertising();
}

void BLE::startServerOTA()
{
    // create server
    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(new OTAServerCallback(this));

    // create service
    _pService = _pServer->createService(SERVICE_UUID);

    // create characteristic
    _pCharacteristic =
        _pService->createCharacteristic(CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    _pCharacteristic->setCallbacks(new OTACharacteristicCallback());

    // start the service
    _pService->start();

    // Start Advertising
    _pAdvertisingOTA = NimBLEDevice::getAdvertising();
    _pAdvertisingOTA->addServiceUUID(SERVICE_UUID);
    _pAdvertisingOTA->start();
}

void BLE::setCustomBeacon(BeaconData_t &data, Setting_t &setting)
{
    // atur data advertising
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    BLEAdvertisementData oScanResponseData  = BLEAdvertisementData();

    const uint16_t beaconUUID = 0xFEAA;
    oScanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
    oScanResponseData.setCompleteServices(BLEUUID(beaconUUID));

    // oAdvertisementData.setFlags(0x06);

    // uint16_t voltage = random(2800, 3700); // dalam millivolts
    // analogInputVal = ain->readAnalogInput(AnalogPin::PIN_A1);
    // float current = 1.5;             // dalam ampere
    // uint32_t timestamp = 1678801234; // contoh Unix TimeStamp

    // Convert current to a 16-bit fixed-point format (e.g., 1.5 A -> 384 in 8.8
    // format) Konversi arus ke format (contoh: 1.5 A -> 384 di format 8.8)
    // int16_t currentFixedPoint = (int16_t)(current * 256);
    // int16_t analogInputFixedPoint = (int16_t)(analogInputVal * 256);

    /* PROCESSING DATA SEBELUM DIKIRIM MELALUI BLE */
    char beacon_data[19];

    // Processing ID
    ListID_t id;
    uint16_t number;

    extractBeaconID(std::string(setting.ID.c_str()), id, number);

    uint16_t volt                = data.voltageSupply * 1000; // 3300mV = 3.3V
    int32_t  latitudeFixedPoint  = (int32_t)(data.gps.latitude * 256);
    int32_t  longitudeFixedPoint = (int32_t)(data.gps.longitude * 256);

    // TODO: add offset to the hourmeter first
    time_t offsettedSecondsHM = _calculateHMOffsetSeconds(data.hourMeter, setting.offsetHM);

    // NOTE: DEBUG
    // Serial.printf("[DEBUG] original HM -> %ld, offsettedHM -> %ld\n", data.hourMeter, offsettedSecondsHM);

    beacon_data[0]  = static_cast<uint8_t>(id); // List ID
    beacon_data[1]  = ((number & 0xFF00) >> 8); // Upper ID Digit
    beacon_data[2]  = (number & 0xFF);          // Lower ID Digit
    beacon_data[3]  = ((volt & 0xFF00) >> 8);   // Battery voltage, 1 mV/bit i.e. 0xCE4 = 3300mV = 3.3V
    beacon_data[4]  = (volt & 0xFF);            //
    beacon_data[5]  = 0;                        // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[6]  = data.gps.status;          //
    beacon_data[7]  = ((longitudeFixedPoint & 0xFF000000) >> 24); //
    beacon_data[8]  = ((longitudeFixedPoint & 0xFF0000) >> 16);   //
    beacon_data[9]  = ((longitudeFixedPoint & 0xFF00) >> 8);      //
    beacon_data[10] = (longitudeFixedPoint & 0xFF);               //
    beacon_data[11] = ((latitudeFixedPoint & 0xFF000000) >> 24);  //
    beacon_data[12] = ((latitudeFixedPoint & 0xFF0000) >> 16);    //
    beacon_data[13] = ((latitudeFixedPoint & 0xFF00) >> 8);       //
    beacon_data[14] = (latitudeFixedPoint & 0xFF);                //
    beacon_data[15] = ((offsettedSecondsHM & 0xFF000000) >> 24);  //
    beacon_data[16] = ((offsettedSecondsHM & 0xFF0000) >> 16);    //
    beacon_data[17] = ((offsettedSecondsHM & 0xFF00) >> 8);       //
    beacon_data[18] = (offsettedSecondsHM & 0xFF);                //

    oScanResponseData.setServiceData(BLEUUID(beaconUUID), std::string(beacon_data, sizeof(beacon_data)));
    oAdvertisementData.setName(_BT_SSID.c_str());
    _pAdvertising->setAdvertisementData(oAdvertisementData);
    _pAdvertising->setScanResponseData(oScanResponseData);
}

void BLE::advertiseBeacon()
{
    _pAdvertising->start();
}

void BLE::startWiFi()
{
    if (!_wifi)
        _wifi = new WiFiClass();
    _wifi->mode(WIFI_AP);
    _setNetworkConfig();
    _wifi->softAPConfig(localIP, gateway, subnet);
    _wifi->softAP(_WIFI_SSID.c_str(), _WIFIpwd);
    _wifi->onEvent(_WiFiAPConnectedCB, arduino_event_id_t::ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);
    _wifi->onEvent(_WiFiAPDisconnectedCB, arduino_event_id_t::ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    Serial.printf("WiFi AP is Started! SSID: %s, IP: ", _WIFI_SSID.c_str());
    Serial.println(_wifi->softAPIP());
}

void BLE::startHTTPServer()
{
    if (!_server)
    {
        _server = new AsyncWebServer(80);
        Serial.println("[HTTP] HTTP Server started!");
    }

    /* "/" -> simple  Hello World */
    _server->on("/", HTTP_GET,
                [](AsyncWebServerRequest *request) { request->send(200, "text/plain", "Hello from ESP32!"); });

    /* "/data" -> for retreive data */
    _server->on("/data", HTTP_GET, _handleSerializingDataJSON);

    // Add JSON handler for /setting endpoint
    AsyncCallbackJsonWebHandler *handler =
        new AsyncCallbackJsonWebHandler("/setting", [](AsyncWebServerRequest *request, JsonVariant &json) {
            JsonObject jsonObj = json.as<JsonObject>();

            if (jsonObj.isNull())
            {
                JsonDocument errorResponse;
                errorResponse["success"] = false;
                errorResponse["message"] = "Invalid JSON";

                String response;
                serializeJson(errorResponse, response);
                request->send(400, "application/json", response);
                return;
            }

            // prints the json
            serializeJson(jsonObj, Serial);
            Serial.println();

            if (jsonObj["ID"].is<String>())
            {
                Serial.print("ok ");
                setting.ID = jsonObj["ID"].as<String>();
            }

            if (jsonObj["voltageThreshold"].is<float>())
            {
                Serial.print("ok ");
                setting.thresholdHM = jsonObj["voltageThreshold"].as<float>();
            }

            if (jsonObj["offsetAnalogInput"].is<float>())
            {
                Serial.print("ok ");
                setting.offsetAnalogInput = jsonObj["offsetAnalogInput"].as<float>();
            }

            if (jsonObj["hourMeter"].is<float>() || jsonObj["hourMeter"].is<int>())
            {
                Serial.print("ok ");
                data.hourMeter = jsonObj["hourMeter"].as<float>();
            }

            if (jsonObj["offsetHourMeter"].is<float>())
            {
                Serial.print("ok ");
                setting.offsetHM = jsonObj["offsetHourMeter"].as<float>();
            }

            hm->saveSettings(setting);
            hm->saveToStorage(data.hourMeter);

            // Create the success response
            JsonDocument responseDoc;
            responseDoc["success"] = true;
            responseDoc["message"] = "Setting updated successfully";
            responseDoc["data"]    = JsonObject(); // Empty data object if nothing needs to be returned

            String response;
            serializeJson(responseDoc, response);
            request->send(200, "application/json", response);
        });

    _server->addHandler(handler); // Attach JSON handler to server

    // Serve the update page
    _server->on(
        "/update", HTTP_POST,
        [this](AsyncWebServerRequest *request) {
            // Parameter 3 (Triggered After Upload Completes)
            if (_isFileBin)
            {
                if (Update.end(true))
                {
                    _isOTASuccess = true;
                    request->send(
                        200, "application/json",
                        R"({"success": true, "message": "OTA update successful. Restarting...", "data": {}})");

                    Serial.println("OTA update successful, restarting in 3 seconds...");

                    // Instead of blocking, create a new FreeRTOS task for restart
                    xTaskCreate(restartTask, "RestartTask", 2048, NULL, 1, NULL);
                }
                else
                {
                    request->send(500, "application/json",
                                  R"({"success": false, "message": "OTA update failed.", "data": {}})");
                }
            }
            else
            {
                request->send(400, "application/json",
                              R"({"success": false, "message": "No binary file received.", "data": {}})");
            }
        },
        [this](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len,
               bool final) {
            // Parameter 4 (File Upload Handling)
            if (!index)
            { // First chunk of file
                Serial.printf("OTA Update Start: %s\n", filename.c_str());

                // check if filename has .bin extension
                if (!filename.endsWith(".bin"))
                {
                    Serial.println("Rejected: Invalid file type!");
                    _isFileBin = false;
                    return;
                }
                if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                {
                    request->send(500, "application/json",
                                  R"({"success": false, "message": "OTA begin failed.", "data": {}})");
                    return;
                }
            }

            // Write the file chunk
            if (Update.write(data, len) != len)
            {
                request->send(500, "application/json",
                              R"({"success": false, "message": "OTA write failed.", "data": {}})");
                return;
            }

            if (final)
            { // Last chunk received
                Serial.println("OTA Update Completed.");
                _isFileBin = true; // Mark that the file was received
            }
        });

    _server->begin();
}

void BLE::stopHTTPServer()
{
    if (_server)
    {
        _server->end();
        delete _server;
        _server = nullptr;
    }
}

void BLE::stopAdvertiseOTA()
{
    if (_pAdvertisingOTA->isAdvertising())
    {
        _pAdvertisingOTA->stop();
    }

    // if (_pServer->)
    // {
    //     _pServer->disconnect(0);
    // }
}

void BLE::setWiFiSSID(String newSSID)
{
    _WIFI_SSID = newSSID;
}

void BLE::restartTask(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(3000)); // Give enough time for the response to send
    ESP.restart();
}

/* PRIVATE METHOD */
esp_err_t BLE::_updateFirmware()
{
    return ESP_OK;
}

time_t BLE::_calculateHMOffsetSeconds(time_t seconds, float offset)
{
    time_t calculatedHM = static_cast<time_t>(round(seconds + (seconds * (offset / 100.0f))));
    return calculatedHM;
}

void BLE::_setNetworkConfig()
{
    localIP      = IPAddress(192, 168, 1, 100);
    gateway      = IPAddress(192, 168, 1, 1);
    subnet       = IPAddress(255, 255, 255, 0);
    primaryDNS   = IPAddress(8, 8, 8, 8);
    secondaryDNS = IPAddress(8, 8, 4, 4);
}

void BLE::_WiFiAPConnectedCB(arduino_event_id_t e)
{
    isConnectedToWiFi = true;

    Serial.printf("A Device is Connected to ESP32 WiFi\n");
}

void BLE::_WiFiAPDisconnectedCB(arduino_event_id_t e)
{
    isConnectedToWiFi = false;
    Serial.printf("A Device is disconnected from ESP32 WiFi\n");
}

void BLE::_handleSerializingDataJSON(AsyncWebServerRequest *request)
{
    JsonDocument _ResponseDoc; // Response wrapper
    JsonDocument _DataDoc;     // Data JSON structure

    // Try populating the JSON object
    try
    {
        _DataDoc["data"]["gps"]["longitude"] = data.gps.longitude;
        _DataDoc["data"]["gps"]["latitude"]  = data.gps.latitude;
        _DataDoc["data"]["gps"]["status"]    = static_cast<String>(data.gps.status);

        _DataDoc["data"]["voltageSupply"] = data.voltageSupply;
        _DataDoc["data"]["hourMeter"]     = data.hourMeter;

        _DataDoc["setting"]["ID"]                = setting.ID;
        _DataDoc["setting"]["thresholdHM"]       = setting.thresholdHM;
        _DataDoc["setting"]["offsetAnalogInput"] = setting.offsetAnalogInput;
        _DataDoc["setting"]["offsetHM"]          = setting.offsetHM;

        // Wrap inside success response
        _ResponseDoc["success"] = true;
        _ResponseDoc["message"] = "Data retrieved successfully";
        _ResponseDoc["data"]    = _DataDoc;
    }
    catch (...)
    { // Catch unexpected errors
        Serial.println("Error: Failed to generate JSON response");

        JsonDocument _ErrorDoc;
        _ErrorDoc["success"] = false;
        _ErrorDoc["message"] = "Failed to process data";

        String errorResponse;
        serializeJson(_ErrorDoc, errorResponse);
        request->send(400, "application/json", errorResponse);
        return;
    }

    // Serialize the final response and send it
    String response;
    serializeJson(_ResponseDoc, response);
    request->send(200, "application/json", response);
}
