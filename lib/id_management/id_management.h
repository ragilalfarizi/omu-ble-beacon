#include <Arduino.h>
#include "common.h"

ListID_t stringToEnum(const std::string &id);

// Function to process the input string
bool extractBeaconID(const std::string &input, ListID_t &id, uint16_t &number);