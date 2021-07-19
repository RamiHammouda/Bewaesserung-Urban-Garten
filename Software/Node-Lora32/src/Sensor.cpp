#include "Sensor.h"
#include <Arduino.h>

bool Sensor::setInputPin(uint8_t pin) 
{
	inputPin = pin;
	blockPin(inputPin);
	pinMode(inputPin, INPUT); // Sets the inputPin as an INPUT
	return true;
}

Sensor::Sensor()
{
	analogReadResolution(10); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
 	analogSetAttenuation(ADC_6db);
}

float Sensor::mapWithFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 const float dividend = out_max - out_min;
 const float divisor = in_max - in_min;
 const float delta = x - in_min;
 if (divisor == 0)
 {
   log_e("Invalid map input range, min == max");
   return -1; //AVR returns -1, SAM returns 0
 }
 return (delta * dividend + (divisor / 2)) / divisor + out_min;
}