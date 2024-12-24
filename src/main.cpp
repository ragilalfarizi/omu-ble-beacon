#include "analog_input.h"
#include "rtc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "NimBLEDevice.h"
#include "NimBLEBeacon.h"
#include "NimBLEAdvertising.h"
#include "NimBLEEddystoneURL.h"
#include "common.h"
#include "gps.h"
#include "hour_meter_manager.h"
#include "id_management.h"
#include "check_sleep.h"
#include <numeric>

/* DEKLARASI OBJEK YANG DIGUNAKAN TERSIMPAN DI HEAP */
RTC *rtc;
AnalogInput *ain;
GPS *gps;
HourMeter *hm;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void RTCDemo(void *pvParam);
static void dataAcquisition(void *pvParam);
static void sendBLEData(void *pvParam);
static void retrieveGPSData(void *pvParam);
static void sendToRS485(void *pvParam);
static void countingHourMeter(void *pvParam);
static void serialConfig(void *pvParam);
static void checkDeepSleepTask(void *param);
static void setCustomBeacon();
static bool updateConfigFromUART(Setting_t &setting, const String &input);

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t dataAcquisitionHandler = NULL;
TaskHandle_t sendBLEDataHandler = NULL;
TaskHandle_t retrieveGPSHandler = NULL;
TaskHandle_t sendToRS485Handler = NULL;
TaskHandle_t countingHMHandler = NULL;
TaskHandle_t settingUARTHandler = NULL;
TaskHandle_t checkSleepHandler = NULL;
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t dataReadySemaphore = NULL;
// TaskHandle_t RTCDemoHandler = NULL;

/* GLOBAL VARIABLES */
BLEAdvertising *pAdvertising;
BeaconData_t data;
HardwareSerial modbus(1);
Setting_t setting;
float scaleAdjusted;
uint16_t glitchCounter = 0;

void setup()
{
    /* SERIAL INIT */
    Serial.begin(9600);
    Serial.println("[Serial] Mesin dinyalakan");

    /* DEEP SLEEP INIT */
    printWakeupReason();
    pinMode(PIN_EN_READ_BATT_VOLT, OUTPUT);
    pinMode(PIN_READ_BATT_VOLT, INPUT);

    /* RTC INIT */
    Serial.println("[RTC] Inisialisasi RTC");
    rtc = new RTC();
    if (rtc == nullptr)
    {
        Serial.println("[ERROR] Failed to allocate memory for RTC object, retrying in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
    }

    /* ANALOG INPUT INIT */
    Serial.println("[AIN] Inisialisasi Analog Input");
    ain = new AnalogInput();
    data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0); // untuk membaca di pin_a0 (PIN 1 pada silkscreen)
    Serial.printf("DATA VOLTAGE SUPPLY DARI SETUP : %f\n", data.voltageSupply);

    /* GPS INIT */
    Serial.println("[GPS] Inisialisasi GPS");
    gps = new GPS();
    data.gps.latitude = 0;
    data.gps.longitude = 0;
    data.gps.status = 'V';

    /* RS485 INIT */
    Serial.println("[485] Inisialisasi RS485");
    modbus.begin(9600, SERIAL_8N1, PIN_RX_RS485, PIN_TX_RS485);

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);
    dataReadySemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(dataReadySemaphore);

    /* BLE INIT */
    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    pAdvertising = BLEDevice::getAdvertising();

    /* HOUR METER INIT */
    hm = new HourMeter();
    data.hourMeter = hm->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter yang tersimpan adalah %ld\n", data.hourMeter);
    // TODO: Print juga hour meter dalam jam

    /* LOAD SETTING */
    setting = hm->loadSetting();
    Serial.printf("[setting] ID\t\t\t: %s\n", setting.ID);
    Serial.printf("[setting] threshold HM\t\t: %d\n", setting.thresholdHM);
    Serial.printf("[setting] offsetAnalogInput\t: %f\n", setting.offsetAnalogInput);
    scaleAdjusted = setting.offsetAnalogInput;

    /* ENABLING ESP32 DEEP SLEEP BY TIMER */
    esp_sleep_enable_timer_wakeup(BLE_SLEEP_DURATION);

    // xTaskCreatePinnedToCore(RTCDemo, "RTC Demo", 2048, NULL, 3, &RTCDemoHandler, 1); // TODO: Depreciating
    // xTaskCreatePinnedToCore(sendToRS485, "send data to RS485", 2048, NULL, 3, &sendToRS485Handler, 0);
    xTaskCreatePinnedToCore(dataAcquisition, "Data Acquisition", 4096, NULL, 3, &dataAcquisitionHandler, 1);
    xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
    xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 4096, NULL, 4, &retrieveGPSHandler, 1);
    xTaskCreatePinnedToCore(countingHourMeter, "Updating Hour Meter", 8192, NULL, 3, &countingHMHandler, 0);
    xTaskCreatePinnedToCore(serialConfig, "Updating setting", 4096, NULL, 3, &settingUARTHandler, 1);
    xTaskCreatePinnedToCore(checkDeepSleepTask, "Check Deep Sleep", 4096, NULL, 1, &settingUARTHandler, 1);
}

void loop()
{
}

static void RTCDemo(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            Serial.println();
            Serial.printf("============================================\n");
            rtc->printRTCData();
            Serial.printf("============================================\n");

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void dataAcquisition(void *pvParam)
{
    float previousLatitude = 0.0, previousLongitude = 0.0;
    int stallCounter = 0; // Time spent stalled in milliseconds
    bool latitudeStalled = false, longitudeStalled = false;

    while (1)
    {
        if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE)
        {
            data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0);

            float currentLatitude = gps->getlatitude();
            float currentLongitude = gps->getLongitude();

            // Check for changes
            latitudeStalled = fabs(currentLatitude - previousLatitude) < 0.00000001; // Adjust threshold if needed
            longitudeStalled = fabs(currentLongitude - previousLongitude) < 0.00000001;

            if (latitudeStalled && longitudeStalled)
            {
                ; // Increment by task delay
                if (stallCounter >= STALL_TOLERANCE_MS)
                {
                    data.gps.status = 'V'; // Mark as invalid after tolerance
                }
            }
            else
            {
                stallCounter = 0;      // Reset counter if data changes
                data.gps.status = 'A'; // Mark as active
            }

            // Store current values for next comparison
            previousLatitude = currentLatitude;
            previousLongitude = currentLongitude;

            // Save data
            data.gps.latitude = currentLatitude;
            data.gps.longitude = currentLongitude;

            // Print output
            Serial.printf("============================================\n");
            Serial.printf("GPS STATUS\t\t= %c\n", data.gps.status);
            Serial.printf("GPS LATITUDE\t\t= %f\n", data.gps.latitude);
            Serial.printf("GPS LONGITUDE\t\t= %f\n", data.gps.longitude);
            Serial.printf("Analog Input\t\t= %.2f V\n", data.voltageSupply);
            Serial.printf("Hour Meter\t\t= %ld s\n", data.hourMeter);
            Serial.printf("============================================\n");
            Serial.printf("[setting] ID\t\t\t: %s\n", setting.ID);
            Serial.printf("[setting] threshold HM\t\t: %d V\n", setting.thresholdHM);
            Serial.printf("[setting] offsetAnalogInput\t: %f\n", setting.offsetAnalogInput);
            Serial.printf("[Err. Counter] Glitch Counter\t: %d\n", glitchCounter);
            Serial.printf("============================================\n");

            xSemaphoreGive(dataReadySemaphore);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        }
    }
}

static void setCustomBeacon()
{
    // atur data advertising
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

    const uint16_t beaconUUID = 0xFEAA;
    oScanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
    oScanResponseData.setCompleteServices(BLEUUID(beaconUUID));

    // uint16_t voltage = random(2800, 3700); // dalam millivolts
    // analogInputVal = ain->readAnalogInput(AnalogPin::PIN_A1);
    // float current = 1.5;             // dalam ampere
    // uint32_t timestamp = 1678801234; // contoh Unix TimeStamp

    // Convert current to a 16-bit fixed-point format (e.g., 1.5 A -> 384 in 8.8 format)
    // Konversi arus ke format (contoh: 1.5 A -> 384 di format 8.8)
    // int16_t currentFixedPoint = (int16_t)(current * 256);
    // int16_t analogInputFixedPoint = (int16_t)(analogInputVal * 256);

    /* PROCESSING DATA SEBELUM DIKIRIM MELALUI BLE */
    char beacon_data[19];

    // Processing ID
    ListID_t id;
    uint16_t number;

    extractBeaconID(std::string(setting.ID.c_str()), id, number);

    uint16_t volt = data.voltageSupply * 1000; // 3300mV = 3.3V
    int32_t latitudeFixedPoint = (int32_t)(data.gps.latitude * 256);
    int32_t longitudeFixedPoint = (int32_t)(data.gps.longitude * 256);

    beacon_data[0] = static_cast<uint8_t>(id);                   // List ID
    beacon_data[1] = ((number & 0xFF00) >> 8);                   // Upper ID Digit
    beacon_data[2] = (number & 0xFF);                            // Lower ID Digit
    beacon_data[3] = ((volt & 0xFF00) >> 8);                     // Battery voltage, 1 mV/bit i.e. 0xCE4 = 3300mV = 3.3V
    beacon_data[4] = (volt & 0xFF);                              //
    beacon_data[5] = 0;                                          // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[6] = data.gps.status;                            //
    beacon_data[7] = ((longitudeFixedPoint & 0xFF000000) >> 24); //
    beacon_data[8] = ((longitudeFixedPoint & 0xFF0000) >> 16);   //
    beacon_data[9] = ((longitudeFixedPoint & 0xFF00) >> 8);      //
    beacon_data[10] = (longitudeFixedPoint & 0xFF);              //
    beacon_data[11] = ((latitudeFixedPoint & 0xFF000000) >> 24); //
    beacon_data[12] = ((latitudeFixedPoint & 0xFF0000) >> 16);   //
    beacon_data[13] = ((latitudeFixedPoint & 0xFF00) >> 8);      //
    beacon_data[14] = (latitudeFixedPoint & 0xFF);               //
    beacon_data[15] = ((data.hourMeter & 0xFF000000) >> 24);     //
    beacon_data[16] = ((data.hourMeter & 0xFF0000) >> 16);       //
    beacon_data[17] = ((data.hourMeter & 0xFF00) >> 8);          //
    beacon_data[18] = (data.hourMeter & 0xFF);                   //

    oScanResponseData.setServiceData(BLEUUID(beaconUUID), std::string(beacon_data, sizeof(beacon_data)));
    oAdvertisementData.setName("UMO BEACON");
    pAdvertising->setAdvertisementData(oAdvertisementData);
    pAdvertising->setScanResponseData(oScanResponseData);
}

static void sendBLEData(void *pvParam)
{
    std::vector<float> analogVoltageContainer;

    uint32_t startTime;
    int16_t voltageSum = 0;
    uint8_t readingCount = 0;
    static const time_t DURATION = 5000;
    static const float THRESHOLD = 3.00;

    while (1)
    {
        setCustomBeacon();

        if (data.voltageSupply > THRESHOLD)
        {
            Serial.printf("[DEBUG] Beacon is running normally (1s)\n");
            pAdvertising->start();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            // Check if the voltage is stalled for 5 seconds
            startTime = millis();

            while (millis() - startTime < DURATION)
            {
                float reading = ain->readAnalogInput(AnalogPin::PIN_A0);

                analogVoltageContainer.push_back(reading);
                delay(500);
            }

            // Calculate the average
            float sum = std::accumulate(analogVoltageContainer.begin(), analogVoltageContainer.end(), 0.0f);
            size_t size = analogVoltageContainer.size();
            float averageVoltage = sum / size;

            Serial.printf("[DEBUG] average voltage monitored : %d\n", averageVoltage);

            if (averageVoltage < THRESHOLD)
            {
                Serial.printf("[DEBUG] Beacon is running on battery saving mode (10s)\n");
                pAdvertising->start(10); // beacon for 10 second
                esp_deep_sleep_start();
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // if (no tegangan input)
        // send beacon 10 seconds then sleep
        // (wake up by counter)
        // else
        //
        // Serial.println("[BLE] Advertising...");
    }
}

/**
 * WARNING: Haven't fully tested. It should be working.
 *          But when I tested it, it took too long to retreive
 *          GPS Data.
 * */
static void retrieveGPSData(void *pvParam)
{
    bool isValid = false;

    while (1)
    {
        // Serial.println("[GPS] encoding...");

        while (Serial.available() > 0)
        {
            char gpsChar = Serial.read();
            gps->encode(gpsChar);
        }

        isValid = gps->getValidation();

        if ((gps->getCharProcessed()) < 10)
        {
            // Serial.println("[GPS] GPS module not sending data, check wiring or module power");
            data.gps.status = 'V';
        }
        else
        {
            if (isValid)
            {
                // Serial.println("GPS is valid");
            }
            else
            {
                // Serial.println("[GPS] GPS is searching for a signal...");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void sendToRS485(void *pvParam)
{

    while (1)
    {
        modbus.printf("%c,%f,%f,%.2f", data.gps.status, data.gps.latitude, data.gps.longitude, data.voltageSupply);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief   : Task to receive new config and update them.
 *            Default state is not running. Only the interrupt
 *            from RS485 input will make it run.
 * @param   : none
 * @retval  : none
 */
static void serialConfig(void *pvParam)
{
    // NOTE: TURN OFF GPS SWITCH
    // NOTE: TURN OFF GPS SWITCH
    TickType_t startTime = xTaskGetTickCount();     // Record the start time
    TickType_t duration = pdMS_TO_TICKS(60000 / 4); // 15 seconds

    while (1)
    {
        if (Serial.available())
        {
            String uartInput = Serial.readStringUntil('\n');
            uartInput.trim();

            if (!uartInput.isEmpty())
            {
                if (updateConfigFromUART(setting, uartInput))
                {

                    // save the config to littlefs
                    hm->saveSettings(setting);
                    hm->saveToStorage(data.hourMeter);

                    Serial.println("Config updated from UART input.");
                }
                else
                {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        }

        // Check if 1 minute has passed
        if (xTaskGetTickCount() - startTime >= duration)
        {
            Serial.println("Serial config task timeout reached. Deleting task.");
            vTaskDelete(NULL); // Delete the task after 1 minute
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void countingHourMeter(void *pvParam)
{
    DateTime startTime, currentTime, previousTime;
    int8_t intervalRaw = 0;
    time_t intervalTime = 0;
    // time_t runTimeAccrued = 0;
    bool isCounting = false; // Tracks if counting has started

    while (1)
    {
        if (data.voltageSupply >= setting.thresholdHM)
        {
            if (!isCounting)
            {
                // Initialize startTime when counting begins
                startTime = rtc->now();
                Serial.printf("[HM] Voltage above threshold. Counting started at %02d:%02d:%02d\n",
                              startTime.hour(), startTime.minute(), startTime.second());
                isCounting = true;

                previousTime = startTime;
            }

            currentTime = rtc->now();
            // runTimeAccrued = static_cast<time_t>(currentTime.secondstime()) - static_cast<time_t>(startTime.secondstime());

            intervalRaw = (int8_t)(currentTime.secondstime() - previousTime.secondstime());
            if (intervalRaw > 11 || intervalRaw < 0)
            {
                Serial.printf("[ERROR] GLITCH IS FOUND! Counter : %d\n", glitchCounter);
                glitchCounter += 1;
                intervalTime = 10;
            }
            else
            {
                intervalTime = (time_t)abs(intervalRaw);
            }

            // NOTE: UNCOMMENT TO DEBUG
            /**
            Serial.printf("============================================\n");
            Serial.printf("[DEBUG] current - start = %d - %d = %d \n", currentTime.secondstime(), startTime.secondstime(), runTimeAccrued);
            Serial.printf("============================================\n");
            */

            data.hourMeter += intervalTime;

            previousTime = currentTime;

            // Serial.printf("[HM] Hour Meter is updated\n");

            if (hm->saveToStorage(data.hourMeter))
            {
                // Serial.println("[HM] total run hour is saved to storage");
            }
            else
            {
                // Serial.println("[HM] total run hour is failed to be saved");
            }

            startTime = currentTime; // Update start time for the next interval
        }
        else
        {
            // Serial.println("[HM] Voltage below threshold, counting paused.");
            isCounting = false; // Reset counting state
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static bool updateConfigFromUART(Setting_t &setting, const String &input)
{
    // Check if input starts with "CONFIG"
    if (input.startsWith("CONFIG"))
    {
        // Remove "CONFIG" from the string
        String tail = input.substring(input.indexOf(',') + 1);

        // Now, tail holds everything after "CONFIG", such as "CD0002,24,0.1,987,DONE"

        // Find the positions of the commas
        int firstComma = tail.indexOf(',');
        int secondComma = tail.indexOf(',', firstComma + 1);
        int thirdComma = tail.indexOf(',', secondComma + 1);
        int fourthComma = tail.indexOf(',', thirdComma + 1);

        // Parse each part of the string after "CONFIG"
        if (firstComma > 0)
        {
            setting.ID = tail.substring(0, firstComma); // Extract ID
        }

        if (secondComma > firstComma + 1)
        {
            String thresholdPart = tail.substring(firstComma + 1, secondComma);
            setting.thresholdHM = thresholdPart.toInt(); // Extract threshold value
        }

        if (thirdComma > secondComma + 1)
        {
            String offsetPart = tail.substring(secondComma + 1, thirdComma);
            setting.offsetAnalogInput = offsetPart.toFloat(); // Extract offset value
            scaleAdjusted = setting.offsetAnalogInput;
        }

        if (fourthComma > thirdComma + 1)
        {
            String hourMeterPart = tail.substring(thirdComma + 1, fourthComma);
            data.hourMeter = hourMeterPart.toInt(); // Extract hour meter value
        }

        // Extract the last part (Done)
        String donePart = tail.substring(fourthComma + 1);
        if (donePart == "DONE")
        {
            Serial.println("[UART] Config update complete.");
        }

        return true;
    }
    else
    {
        Serial.println("[UART] Invalid input, expected 'CONFIG' at the start.");
        return false;
    }
}

static void checkDeepSleepTask(void *param)
{
    std::vector<float> batteryVoltageReadings;

    while (1)
    {
        if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE)
        {
            if (data.voltageSupply > LOWEST_ANALOG_THRESHOLD)
            {
                // Serial.println("Adapter is plugged in. Keeping system running.");
            }
            else
            {
                // Serial.printf("Analog Input is less than %.2f V. Reading the battery voltage..\n", LOWEST_ANALOG_THRESHOLD);

                // enable pin to read battery voltage
                digitalWrite(PIN_EN_READ_BATT_VOLT, HIGH);
                vTaskDelay(pdMS_TO_TICKS(100));

                // Monitor battery voltage for 15 seconds
                unsigned long startTime = millis();
                int voltageSum = 0;
                int readingsCount = 0;

                while (millis() - startTime < MONITOR_DURATION)
                {
                    int16_t batteryADC = analogRead(PIN_READ_BATT_VOLT);
                    float batteryVoltage = calculateBatteryVoltage(batteryADC);
                    // Serial.printf("batteryVoltage : %.2f\n", batteryVoltage);

                    batteryVoltageReadings.push_back(batteryVoltage);

                    // voltageSum += batteryVoltage;
                    // readingsCount++;
                    delay(500); // Read battery voltage every 500ms
                }

                // Calculate the average battery voltage
                float sum = std::accumulate(batteryVoltageReadings.begin(), batteryVoltageReadings.end(), 0.0f);
                // Serial.printf("Sum of Batt Voltage : %f\n", sum);

                size_t vectorSize = batteryVoltageReadings.size();
                // Serial.printf("Size of Batt Voltage Vector : %d\n", vectorSize);

                float averageVoltage = sum / vectorSize;
                // Serial.print("Average Voltage: ");
                Serial.println(averageVoltage);

                // Check if average voltage is below the threshold
                if (averageVoltage < BATTERY_THRESHOLD)
                {
                    Serial.println("Battery voltage is critically low. Entering deep sleep...");

                    // PIN CHG_STS will be low when the adaptor is plugged in again.
                    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 1 = High, 0 = Low
                    digitalWrite(PIN_EN_READ_BATT_VOLT, LOW);     // Recontrol pin before sleeping
                    batteryVoltageReadings.clear();

                    vTaskDelay(pdMS_TO_TICKS(100));

                    esp_deep_sleep_start();
                }
                else
                {
                    batteryVoltageReadings.clear();
                    // Serial.println("Battery voltage is sufficient. Continuing operation...");
                }
            }

            xSemaphoreGive(dataReadySemaphore);
            // Small delay to prevent task starvation
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}