#include <ctime>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <numeric>

#include "analog_input.h"
#include "ble.h"
#include "check_sleep.h"
#include "common.h"
#include "gps.h"
#include "hour_meter_manager.h"
#include "id_management.h"
#include "rtc.h"

/* DEKLARASI OBJEK YANG DIGUNAKAN TERSIMPAN DI HEAP */
RTC         *rtc;
AnalogInput *ain;
GPS         *gps;
HourMeter   *hm;
BLE         *ble;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void   dataAcquisition(void *pvParam);
static void   sendBLEData(void *pvParam);
static void   retrieveGPSData(void *pvParam);
static void   sendToRS485(void *pvParam);
static void   countingHourMeter(void *pvParam);
static void   serialConfig(void *pvParam);
static void   checkDeepSleepTask(void *param);
static void   OTABLEUpdate(void *pvParam);
static bool   updateConfigFromUART(Setting_t &setting, const String &input);
static void   printFirmwareVersion();
static float  calculateHMOffset(time_t seconds, float offset);
static time_t calculateHMOffsetSeconds(time_t seconds, float offset);
static void   wifiTimeoutCallback(TimerHandle_t xTimer);
void          goingDeepSleepTimerCB(TimerHandle_t xTimer);

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t      dataAcquisitionHandler = NULL;
TaskHandle_t      sendBLEDataHandler     = NULL;
TaskHandle_t      retrieveGPSHandler     = NULL;
TaskHandle_t      sendToRS485Handler     = NULL;
TaskHandle_t      countingHMHandler      = NULL;
TaskHandle_t      settingUARTHandler     = NULL;
TaskHandle_t      checkSleepHandler      = NULL;
TaskHandle_t      OTABLEUpdateHandler    = NULL;
SemaphoreHandle_t xSemaphore             = NULL;
SemaphoreHandle_t dataReadySemaphore     = NULL;
// TaskHandle_t RTCDemoHandler = NULL;

/* GLOBAL VARIABLES */
BeaconData_t   data;
HardwareSerial modbus(1);
Setting_t      setting;
float          scaleAdjusted;
uint16_t       glitchCounter = 0;
SystemState_t  currentState;
float          hourMeterInHours;

void setup()
{
    /* SERIAL INIT */
    Serial.begin(9600);
    Serial.println("[Serial] Mesin dinyalakan");

    /* PRINT FIRMWARE VERSION */
    printFirmwareVersion();

    /* DEEP SLEEP INIT */
    printWakeupReason();
    pinMode(PIN_EN_READ_BATT_VOLT, OUTPUT);
    pinMode(PIN_READ_BATT_VOLT, INPUT);

    /* RTC INIT */
    Serial.println("[RTC] Inisialisasi RTC");
    rtc = new RTC();
    if (rtc == nullptr)
    {
        Serial.println("[ERROR] Failed to allocate memory for RTC object, retrying in 5 "
                       "seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
    }

    /* ANALOG INPUT INIT */
    Serial.println("[AIN] Inisialisasi Analog Input");
    ain                = new AnalogInput();
    data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0); // untuk membaca di pin_a0 (PIN 1 pada silkscreen)
    Serial.printf("DATA VOLTAGE SUPPLY DARI SETUP : %f\n", data.voltageSupply);

    /* GPS INIT */
    Serial.println("[GPS] Inisialisasi GPS");
    gps                = new GPS();
    data.gps.latitude  = 0;
    data.gps.longitude = 0;
    data.gps.status    = 'V';

    /* RS485 INIT */
    Serial.println("[485] Inisialisasi RS485");
    modbus.begin(9600, SERIAL_8N1, PIN_RX_RS485, PIN_TX_RS485);

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);
    dataReadySemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(dataReadySemaphore);

    /* HOUR METER INIT */
    hm             = new HourMeter();
    data.hourMeter = hm->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter yang tersimpan : %ld s\n", data.hourMeter);
    hourMeterInHours = data.hourMeter / 3600.f;
    Serial.printf("[HM] Hour Meter yang tersimpan : %.2f Hrs\n", hourMeterInHours);

    /* LOAD SETTING */
    setting = hm->loadSetting();
    Serial.printf("[setting] ID\t\t\t: %s\n", setting.ID.c_str());
    Serial.printf("[setting] threshold HM\t: %.2f\n", setting.thresholdHM);
    Serial.printf("[setting] offsetAnalogInput\t: %f\n", setting.offsetAnalogInput);
    Serial.printf("[setting] offsetHM \t\t: %.2f %%\n", setting.offsetHM);
    scaleAdjusted = setting.offsetAnalogInput;

    /* BLE INIT */
    ble = new BLE();
    ble->begin();
    Serial.println("[BLE] Inisialisasi BLE");

    /* ENABLING ESP32 DEEP SLEEP BY TIMER */
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 1 = High, 0 = Low
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIMER_CYCLE_OFF * uS_TO_S_FACTOR);

    xTaskCreatePinnedToCore(dataAcquisition, "Data Acquisition", 4096, NULL, 3, &dataAcquisitionHandler, 1);
    xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
    xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 4096, NULL, 4, &retrieveGPSHandler, 1);
    xTaskCreatePinnedToCore(countingHourMeter, "Updating Hour Meter", 8192, NULL, 3, &countingHMHandler, 0);
    xTaskCreatePinnedToCore(serialConfig, "Updating setting", 4096, NULL, 3, &settingUARTHandler, 1);
    xTaskCreatePinnedToCore(checkDeepSleepTask, "Check Deep Sleep", 4096, NULL, 1, &checkSleepHandler, 1);
    xTaskCreatePinnedToCore(OTABLEUpdate, "OTA BLE Update", 4096, NULL, 2, &OTABLEUpdateHandler, 1);
}

void loop()
{
}

static void dataAcquisition(void *pvParam)
{
    float previousLatitude = 0.0, previousLongitude = 0.0;
    int   stallCounter    = 0; // Time spent stalled in milliseconds
    bool  latitudeStalled = false, longitudeStalled = false;

    while (1)
    {
        // if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE)
        // {
        data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0);

        float currentLatitude  = gps->getlatitude();
        float currentLongitude = gps->getLongitude();

        // Check for changes
        latitudeStalled  = fabs(currentLatitude - previousLatitude) < 0.00000001; // Adjust threshold if needed
        longitudeStalled = fabs(currentLongitude - previousLongitude) < 0.00000001;

        if (latitudeStalled && longitudeStalled)
        {
            // Increment by task delay
            if (stallCounter >= STALL_TOLERANCE_MS)
            {
                data.gps.status = 'V'; // Mark as invalid after tolerance
            }
        }
        else
        {
            stallCounter    = 0;   // Reset counter if data changes
            data.gps.status = 'A'; // Mark as active
        }

        // Store current values for next comparison
        previousLatitude  = currentLatitude;
        previousLongitude = currentLongitude;

        // Save data
        data.gps.latitude  = currentLatitude;
        data.gps.longitude = currentLongitude;

        // Print output
        Serial.printf("============================================\n");
        Serial.printf("GPS STATUS\t\t\t= %c\n", data.gps.status);
        Serial.printf("GPS LATITUDE\t\t\t= %f\n", data.gps.latitude);
        Serial.printf("GPS LONGITUDE\t\t\t= %f\n", data.gps.longitude);
        Serial.printf("Analog Input\t\t\t= %.2f V\n", data.voltageSupply);
        Serial.printf("Hour Meter(seconds)\t\t= %ld s\n", data.hourMeter);
        Serial.printf("Hour Meter + offset(seconds)\t= %ld s\n",
                      calculateHMOffsetSeconds(data.hourMeter, setting.offsetHM));
        Serial.printf("Hour Meter(hours)\t\t= %.3f Hrs\n", static_cast<float>(data.hourMeter / 3600.0f));
        Serial.printf("Hour Meter + offset(hours)\t= %.3f Hrs\n", calculateHMOffset(data.hourMeter, setting.offsetHM));
        Serial.printf("============================================\n");
        Serial.printf("[setting] ID\t\t\t: %s\n", setting.ID.c_str());
        Serial.printf("[setting] threshold HM\t\t: %.2f V\n", setting.thresholdHM);
        Serial.printf("[setting] offsetAnalogInput\t: %f\n", setting.offsetAnalogInput);
        Serial.printf("[setting] offsetHM \t\t: %.2f%%\n", setting.offsetHM);
        // Serial.printf("[Err] Glitch Counter\t\t: %d\n", glitchCounter);
        Serial.printf("============================================\n");

        xSemaphoreGive(dataReadySemaphore);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

static void sendBLEData(void *pvParam)
{
    while (1)
    {
        ble->setCustomBeacon(data, setting);

        ble->advertiseBeacon();

        if (currentState == NORMAL)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

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
            // Serial.println("[GPS] GPS module not sending data, check wiring or
            // module power");
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
    TickType_t startTime = xTaskGetTickCount();      // Record the start time
    TickType_t duration  = pdMS_TO_TICKS(60000 / 4); // 15 seconds

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
    int8_t   intervalRaw     = 0;
    time_t   intervalTime    = 0;
    bool     isCounting      = false; // Tracks if counting has started
    float    offsetValue     = 0;
    time_t   currentSeconds  = 0;
    time_t   previousSeconds = 0;

    while (1)
    {
        if (data.voltageSupply >= setting.thresholdHM)
        {
            if (!isCounting)
            {
                // Initialize startTime when counting begins
                startTime = rtc->now();
                Serial.printf("[HM] Voltage above threshold. Counting started at "
                              "%02d:%02d:%02d\n",
                              startTime.hour(), startTime.minute(), startTime.second());
                isCounting = true;

                // NOTE: add 10s upfront to normalize late trigger on HM
                //       in the future, it's better to refactor RTOS task
                intervalTime = 10;
                data.hourMeter += intervalTime;
                hm->saveToStorage(data.hourMeter);

                previousTime = startTime;
            }

            currentTime = rtc->now();
            vTaskDelay(pdMS_TO_TICKS(50));
            currentSeconds = currentTime.secondstime();
            vTaskDelay(pdMS_TO_TICKS(50));
            previousSeconds = previousTime.secondstime();

            intervalRaw = static_cast<int8_t>(currentSeconds - previousSeconds);

            if (intervalRaw > 11 || intervalRaw < 0)
            {
                intervalTime = 10;
            }
            else
            {
                intervalTime = static_cast<time_t>(abs(intervalRaw));
            }

            // NOTE: UNCOMMENT TO DEBUG

            Serial.printf("============================================\n");
            Serial.printf("[DEBUG] current - start = %d - %d = %d \n", currentSeconds, previousSeconds, intervalRaw);
            Serial.printf("============================================\n");

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
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        else
        {
            // Serial.println("[HM] Voltage below threshold, counting paused.");
            isCounting = false; // Reset counting state
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static bool updateConfigFromUART(Setting_t &setting, const String &input)
{
    // Check if input starts with "CONFIG"
    if (input.startsWith("CONFIG"))
    {
        // Remove "CONFIG" from the string
        String tail = input.substring(input.indexOf(',') + 1);

        // Now, tail holds everything after "CONFIG", such as
        // "CD0002,24.8,0.1,98.5,0.2,DONE"

        // Find the positions of the commas
        int firstComma  = tail.indexOf(',');
        int secondComma = tail.indexOf(',', firstComma + 1);
        int thirdComma  = tail.indexOf(',', secondComma + 1);
        int fourthComma = tail.indexOf(',', thirdComma + 1);
        int fifthComma  = tail.indexOf(',', fourthComma + 1);

        // Parse each part of the string after "CONFIG"
        if (firstComma > 0)
        {
            setting.ID = tail.substring(0, firstComma); // Extract ID
        }

        if (secondComma > firstComma + 1)
        {
            String thresholdPart = tail.substring(firstComma + 1, secondComma);
            setting.thresholdHM  = thresholdPart.toFloat(); // Extract threshold value
        }

        if (thirdComma > secondComma + 1)
        {
            String offsetPart         = tail.substring(secondComma + 1, thirdComma);
            setting.offsetAnalogInput = offsetPart.toFloat(); // Extract offset value
            scaleAdjusted             = setting.offsetAnalogInput;
        }

        if (fourthComma > thirdComma + 1)
        {
            String hourMeterPart = tail.substring(thirdComma + 1, fourthComma);
            // TODO: Recheck because when user input .3f or more, it will be containing commas
            float HMcomma = hourMeterPart.toFloat();
            // Serial.printf("float HM : %.f\n", HMcomma);
            time_t tempInput = static_cast<time_t>(hourMeterPart.toFloat() * 3600);
            // Serial.printf("float HM : %d\n", tempInput);
            data.hourMeter = static_cast<time_t>(hourMeterPart.toFloat() * 3600); // Extract hour meter value
        }

        if (fifthComma > fourthComma + 1)
        {
            String offsetHMPart = tail.substring(fourthComma + 1, fifthComma);
            setting.offsetHM    = offsetHMPart.toFloat(); // Extract offsetHM value
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
    // Create buffer for averaging battery monitoring
    std::vector<float> batteryVoltageReadings;

    // Create deep sleep callback
    TimerHandle_t goingDeepSleepTimer;
    goingDeepSleepTimer = xTimerCreate("OneShot5Min", pdMS_TO_TICKS(DEEP_SLEEP_TIMER_CYCLE_ON * 1000), pdFALSE, NULL,
                                       goingDeepSleepTimerCB);

    // Check if the callback is created
    if (goingDeepSleepTimer == NULL)
    {
        Serial.println("[SLEEP] Failed to create timer.");
    }

    while (1)
    {
        // if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE)
        // {
        if (data.voltageSupply > LOWEST_ANALOG_THRESHOLD)
        {
            // Serial.println("Adapter is plugged in. Keeping system running.");
            currentState = NORMAL;

            if (xTimerIsTimerActive(goingDeepSleepTimer) == pdTRUE)
            {
                Serial.println("[SLEEP] Stopping the timer since the analog is HOT");
                xTimerStop(goingDeepSleepTimer, 0);
            }
        }
        else
        {
            // Serial.printf("Analog Input is less than %.2f V. Reading the battery
            // voltage..\n", LOWEST_ANALOG_THRESHOLD);
            currentState = ON_BATTERY;

            // enable pin to read battery voltage
            digitalWrite(PIN_EN_READ_BATT_VOLT, HIGH);
            vTaskDelay(pdMS_TO_TICKS(100));

            // Monitor battery voltage for 15 seconds
            unsigned long startTime     = millis();
            int           voltageSum    = 0;
            int           readingsCount = 0;

            while (millis() - startTime < MONITOR_DURATION)
            {
                int16_t batteryADC     = analogRead(PIN_READ_BATT_VOLT);
                float   batteryVoltage = calculateBatteryVoltage(batteryADC);
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
            Serial.printf("Battery voltage : %.2f\n", averageVoltage);

            // Check if average voltage is below the threshold
            if (averageVoltage < BATTERY_THRESHOLD)
            {
                Serial.println("Battery voltage is critically low. Starting The Timer...");

                if (xTimerIsTimerActive(goingDeepSleepTimer) == pdFALSE)
                {
                    Serial.println("[SLEEP] Timer is currently not active. starting the timer.");
                    xTimerStart(goingDeepSleepTimer, 0);
                }
                else
                {
                    Serial.println("[SLEEP] Timer is active.");
                }

                batteryVoltageReadings.clear();
            }
            else
            {
                batteryVoltageReadings.clear();
                // Serial.println("Battery voltage is sufficient. Continuing
                // operation...");
            }
        }

        xSemaphoreGive(dataReadySemaphore);
        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void OTABLEUpdate(void *pvParam)
{
    TimerHandle_t wifiTimer;
    wifiTimer = xTimerCreate("WiFi Timeout", pdMS_TO_TICKS(OTA_WIFI_DURATION), pdFALSE, NULL, wifiTimeoutCallback);

    ble->startServerOTA();

    BLE::OTAState state        = BLE::IDLE;
    bool          wasConnected = false; // Track previous connection state

    while (1)
    {
        switch (state)
        {
        case BLE::IDLE:
            if (ble->isConnectedToWiFi)
            {
                Serial.println("Switching to OTA ON state...");
                // ble->stopAdvertiseOTA();
                ble->startHTTPServer();

                if (xTimerIsTimerActive(wifiTimer) == pdFALSE)
                {
                    xTimerStart(wifiTimer, 0);
                }

                state = BLE::OTA_ON;
            }
            break;

        case BLE::OTA_ON:
            if (!ble->isConnectedToWiFi || xTimerIsTimerActive(wifiTimer) == pdFALSE)
            {
                Serial.println("Switching to IDLE state...");
                // ble->startAdvertiseOTA();
                ble->stopHTTPServer();

                if (xTimerIsTimerActive(wifiTimer) == pdTRUE)
                {
                    xTimerStop(wifiTimer, 0);
                }

                state = BLE::IDLE;
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(500)); // Prevent busy looping
    }
}

static void printFirmwareVersion()
{
    Serial.printf("\n");
    Serial.printf("########################################\n");
    Serial.printf("##                                    ##\n");
    Serial.printf("##  FIRMWARE VERSION: %-10s  ##\n", FIRMWARE_VERSION);
    Serial.printf("##                                    ##\n");
    Serial.printf("########################################\n");
    Serial.printf("\n");
}

/**
 * @brief calculate seconds of hour meter into hours of hour meter with offset
 * @retval calculatedHM hour meter with offset
 * */
static float calculateHMOffset(time_t seconds, float offset)
{
    float hour         = seconds / 3600.0f;
    float calculatedHM = hour + (hour * (offset / 100.0f));
    return calculatedHM;
}

static time_t calculateHMOffsetSeconds(time_t seconds, float offset)
{
    time_t calculatedHM = static_cast<time_t>(round(seconds + (seconds * (offset / 100.0f))));
    return calculatedHM;
}

static void wifiTimeoutCallback(TimerHandle_t xTimer)
{
    // Access the global instance of BLE
    if (ble->_wifi)
    {
        Serial.println("WiFi is about to shutdown..");
        ble->_wifi->disconnect(true); // disconnect WiFi
        // vTaskDelay(pdMS_TO_TICKS(500));

        ble->_wifi->mode(WIFI_OFF); // Turn off WiFi
        // vTaskDelay(pdMS_TO_TICKS(500));

        Serial.println("WiFi is shutting down..");
        delete ble->_wifi;
        ble->_wifi = nullptr;
    }
}

void goingDeepSleepTimerCB(TimerHandle_t xTimer)
{
    Serial.println("[SLEEP] deep sleep timer triggered!");

    if (data.voltageSupply > LOWEST_ANALOG_THRESHOLD)
    {
        // do nothing
        Serial.println("[SLEEP] deep sleep timer is cancelled");
    }
    else
    {
        // PIN CHG_STS will be low when the adaptor is plugged in again.
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0); // 1 = High, 0 = Low
        digitalWrite(PIN_EN_READ_BATT_VOLT,
                     LOW); // Recontrol pin before sleeping
        // batteryVoltageReadings.clear();

        vTaskDelay(pdMS_TO_TICKS(100));

        // Make GPS to be StandbyMode
        Serial.println("$PMTK161,0*28");
        vTaskDelay(pdMS_TO_TICKS(100));

        vTaskSuspend(dataAcquisitionHandler);
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Serial.flush();

        esp_deep_sleep_start();
    }
}
