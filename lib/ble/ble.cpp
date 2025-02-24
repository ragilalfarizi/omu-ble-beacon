#include "ble.h"

const char *BLE::_SSID     = "ESP32-WIFI-RAGIL";
const char *BLE::_password = "1234567890";

/* PUBLIC METHOD */
BLE::BLE()
{
}

void BLE::begin()
{
    BLEDevice::init("");
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
    oAdvertisementData.setName(setting.ID.c_str());
    _pAdvertising->setAdvertisementData(oAdvertisementData);
    _pAdvertising->setScanResponseData(oScanResponseData);
}

void BLE::advertiseBeacon()
{
    _pAdvertising->start();
}

void BLE::startWiFi()
{
    _wifi = new WiFiClass();

    // WiFi.mode(WIFI_AP);
    // WiFi.xPower(WIFI_POWER_MINUS_1dBm);
    // WiFi.softAP(_SSID, _password);

    // WARNING: - The BLE off automatically when WIFI is on.
    // - WIFI can't be turned on while charging. (brownout detected)

    Serial.printf("Free heap before WiFi: %u\n", ESP.getFreeHeap());
    _wifi->mode(WIFI_AP);
    // _wifi->setTxPower(WIFI_POWER_MINUS_1dBm);
    _wifi->softAP(_SSID, _password);
    // _wifi->setSleep(true);
    Serial.printf("Free heap after WiFi: %u\n", ESP.getFreeHeap());

    // WiFi.mode(WIFI_STA);
    // WiFi.begin("POCO X5 5G", "gantengsekali");
    // Serial.print("Connecting to WiFi...");
    // while (WiFi.status() != WL_CONNECTED)
    // {
    //     Serial.print(".");
    //     delay(100);
    // }
    //
    // Serial.println("\n✅ Connected!");
    // Serial.print("STA IP: ");
    // Serial.println(WiFi.localIP());

    Serial.printf("WiFi AP is Started! SSID: %s\n", _SSID);
}

void BLE::startHTTPServer()
{
    _server = new WebServer(80);
}

/* PRIVATE METHOD */
esp_err_t BLE::_updateFirmware()
{
}

time_t BLE::_calculateHMOffsetSeconds(time_t seconds, float offset)
{
    time_t calculatedHM = static_cast<time_t>(round(seconds + (seconds * (offset / 100.0f))));
    return calculatedHM;
}
