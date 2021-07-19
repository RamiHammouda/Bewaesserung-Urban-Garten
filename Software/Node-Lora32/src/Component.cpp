
#include <map>
#include "Component.h"


/// <summary>
/// All available Pins
/// Set to true, because all available at beginning
/// </summary>
std::map<int, bool> Component::loraPins =
{
    { 34, true },
    { 35, true },
    { 14, true },
    { 12, true },
    { 13, true },
    { 15, true },
    { 2, true },
    { 0, true },
    { 4, true },
    { 25, true },
    { 21, true },
    { 22, true },
    { 23, true },
    { 19, true },
    { 26, true }
};


/// <summary>
/// Check wether Pin is available
/// </summary>
/// <param name="pin">pin number, that should be checked</param>
/// <returns>false, if used or not on Lora32 ; else: true</returns>
bool Component::checkPin(uint8_t pin) {
    
    return loraPins[pin];
}

/// <summary>
/// Set a pin as used --> blocked for other things
/// </summary>
/// <param name="pin">pin number</param>
/// <returns>succesfull or already used?</returns>
bool Component::blockPin(uint8_t pin) {
    if (checkPin(pin))
    {
        loraPins[pin] = false;
        return true;
    }
    else 
    {
        return false;
    }
}