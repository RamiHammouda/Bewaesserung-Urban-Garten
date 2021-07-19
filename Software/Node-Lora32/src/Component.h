// Pins that are available on Lora32
#pragma once
#include <array>
#include <map>
#include <Arduino.h>

using namespace std;

/// <summary>
/// Component can be sensor or actor
/// All components use pins of microcontroller
/// </summary>
class Component
{
protected:
    static std::map<int, bool> loraPins; 


    Component() {};

    bool checkPin(uint8_t pin);
    bool blockPin(uint8_t pin);

    //virtual void init() = 0; // pure virtual --> abstract class; it is not possible to initialise Component

    virtual ~Component() {}; // virtual destructor


}; 
