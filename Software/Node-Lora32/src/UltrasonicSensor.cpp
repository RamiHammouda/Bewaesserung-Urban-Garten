#include "UltrasonicSensor.h"

bool UltrasonicSensor::set_trig_echo(uint8_t trig, uint8_t echo) 
{
	set_trig(trig);
	setInputPin(echo);
	return true; 
}

bool UltrasonicSensor::set_trig(uint8_t trig)
{
	triggerPin = trig;
	pinMode(triggerPin, OUTPUT); // Sets the outputPin as an OUTPUT
	return true;
}

int UltrasonicSensor::getDistance_cm()
{
	// Clears the trigPin condition
  	digitalWrite(triggerPin, LOW);
  	delayMicroseconds(2);
  	// Sets the trigPin HIGH (ACTIVE) for 10 microseconds
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);
	// Reads the echoPin, returns the sound wave travel time in microseconds
	double duration = pulseIn(inputPin, HIGH);
	// Calculating the distance
	double distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
	// Displays the distance on the Serial Monitor

	return (int)distance;

}
