#include "MoistureSensor.h"

float MoistureSensor::getMoisture_percent()
{
    float solidMoistureValue = (float)analogRead(inputPin) / 1023 * 100; //Get Analog Value and transform it to percent

    return solidMoistureValue;
}