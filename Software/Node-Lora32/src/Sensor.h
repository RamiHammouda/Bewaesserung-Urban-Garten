#include "Component.h"
#pragma once

class Sensor : public Component
{

protected:
    uint8_t inputPin; // every Sensor needs an input pin on Lora32
    

public:
    Sensor();

    uint8_t get_inputPin() {return inputPin;}
   // virtual void init() = 0; // pure virtual --> abstract class; it is not possible to initialise Component
    bool setInputPin(uint8_t pin);

    float mapWithFloat(float x, float in_min, float in_max, float out_min, float out_max);

    virtual ~Sensor() {} // virtual destructor


};
