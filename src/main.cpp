#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Hello World!");
    delay(1000);
}