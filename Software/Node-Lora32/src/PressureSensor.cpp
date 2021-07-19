#include "PressureSensor.h"

float PressureSensor::getPressure_bar()
{
    return mapWithFloat(analogRead(inputPin),0,1023,0,100) / 14.504;
}


