#include "hour_meter_manager.h"

HourMeter::HourMeter(uint8_t format_on_fail)
{
    if (LittleFS.begin(format_on_fail))
    {
        Serial.println("Storage has been allocated");
        checkAndCreateFiles();
    }
    else
    {
        Serial.println("Failed to allocate storage");
    }
}

HourMeter::~HourMeter()
{
}

void HourMeter::checkAndCreateFiles()
{
    // Check and create /data.txt
    if (!LittleFS.exists("/data.txt"))
    {
        File file = LittleFS.open("/data.txt", "w"); // Open for writing (creates if not exists)
        if (file)
        {
            file.println("0"); // Write initial value
            file.close();      // Close the file
            Serial.println("Created /data.txt with initial value 0");
        }
        else
        {
            Serial.println("Failed to create /data.txt");
        }
    }
    else
    {
        Serial.println("/data.txt already exists");
    }

    // Check and create /setting.txt
    if (!LittleFS.exists("/setting.txt"))
    {
        File file = LittleFS.open("/setting.txt", "w"); // Open for writing (creates if not exists)
        if (file)
        {
            file.println("0"); // Write initial value
            file.close();      // Close the file
            Serial.println("Created /setting.txt with initial value 0");
        }
        else
        {
            Serial.println("Failed to create /setting.txt");
        }
    }
    else
    {
        Serial.println("/setting.txt already exists");
    }
}

// int32_t HourMeter::getSavedHourMeter()
// {
//     return savedHourMeter;
// }

time_t HourMeter::loadHMFromStorage()
{
    time_t hourMeter = 0;
    Serial.println("Attempting to open data.txt for reading...");

    FILE *file = fopen("/littlefs/data.txt", "r");
    if (file)
    {
        // Attempt to read the time value from the file
        if (fscanf(file, "%ld", &hourMeter) == 1) // Check that exactly one value is read
        {
            Serial.printf("Successfully read hour meter value: %ld\n", hourMeter);
        }
        else
        {
            Serial.println("Data format error: unable to read a valid time value.");
            hourMeter = 0; // Reset to a default or error-indicating value
        }

        fclose(file);
    }
    else
    {
        Serial.println("Error: Could not open data.txt for reading.");
    }

    return hourMeter;
}

Setting_t HourMeter::loadSetting()
{
    Setting_t setting;
    Serial.println("Attempting to open settings.json for reading...");

    File file = LittleFS.open("/settings.json", "r");

    if (file)
    {
        JsonDocument doc;

        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            Serial.print("Data format error: Failed to parse JSON in settings.json: ");
            Serial.println(error.c_str());
            file.close();
            return setting; // Return with default or previously set values
        }

        // Extract values from JSON if successfully parsed
        setting.ID = doc["ID"].as<String>();
        setting.thresholdHM = doc["thresholdHM"].as<uint8_t>();
        setting.offsetAnalogInput = doc["offsetAnalogInput"].as<float>();

        Serial.println("Successfully read settings from settings.json");
        file.close();
    }
    else
    {
        Serial.println("Error: Could not open settings.json for reading.");
    }

    return setting;
}