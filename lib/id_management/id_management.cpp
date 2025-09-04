#include <Arduino.h>
#include "common.h"

ListID_t stringToEnum(const std::string &id)
{
    if (id == "AC")
        return AC;
    if (id == "CC")
        return CC;
    if (id == "CD")
        return CD;
    if (id == "CE")
        return CE;
    if (id == "CG")
        return CG;
    if (id == "CO")
        return CO;
    if (id == "CT")
        return CT;
    if (id == "DP")
        return DP;
    if (id == "FL")
        return FL;
    if (id == "LS")
        return LS;
    if (id == "MC")
        return MC;
    if (id == "MS")
        return MS;
    if (id == "PP")
        return PP;
    if (id == "ST")
        return ST;
    if (id == "TH")
        return TH;
    if (id == "TL")
        return TL;
    if (id == "WT")
        return WT;
    if (id == "XCD")
        return XCD;
    if (id == "XCE")
        return XCE;
    if (id == "XCT")
        return XCT;
    if (id == "XDP")
        return XDP;
    if (id == "XMC")
        return XMC;
    if (id == "XST")
        return XST;
    if (id == "XWT")
        return XWT;
    if (id == "XAC")
        return XAC;
    if (id == "XCC")
        return XCC;
    if (id == "XCG")
        return XCG;
    if (id == "XCO")
        return XCO;
    if (id == "XFL")
        return XFL;
    if (id == "XLS")
        return XLS;
    if (id == "XMS")
        return XMS;
    if (id == "XPP")
        return XPP;
    if (id == "XTH")
        return XTH;
    if (id == "XTL")
        return XTL;
    return UNKNOWN; // Fallback for unknown strings
}

// Function to process the input string
bool extractBeaconID(const std::string &input, ListID_t &id, uint16_t &number)
{
    // Split alphabet and numeric parts
    std::string alphabets, digits;

    for (char ch : input)
    {
        if (isalpha(ch))
        {
            alphabets += ch;
        }
        else if (isdigit(ch))
        {
            digits += ch;
        }
    }

    // Validate extracted parts
    if (alphabets.empty() || digits.size() != 4)
    {
        return false; // Invalid format
    }

    // Convert the alphabetic part to enum
    id = stringToEnum(alphabets);

    if (id == UNKNOWN)
    {
        return false; // Unknown ID
    }

    // Convert numeric part to uint16_t
    number = static_cast<uint16_t>(std::stoi(digits));

    return true;
}
