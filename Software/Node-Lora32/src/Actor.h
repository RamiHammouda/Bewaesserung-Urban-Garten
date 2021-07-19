#pragma once
#include "Component.h"

class Actor : public Component
{
protected:
    uint8_t outputPin; // every Actor needs an output pin on Lora32
    bool invert;

public:
    Actor(bool inv = true);

    uint8_t get_outputPin() {return outputPin;}

    void turnOff();
    void turnOn();

    bool setOutputPin(uint8_t pin);

    virtual ~Actor() {} // virtual destructor

};
