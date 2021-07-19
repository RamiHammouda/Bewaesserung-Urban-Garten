#include "Actor.h"

bool Actor::setOutputPin(uint8_t pin)
{
	outputPin = pin;
	blockPin(pin);
	pinMode(outputPin, OUTPUT);
	return true;
}


void Actor::turnOn()
{
    digitalWrite(outputPin, invert? LOW : HIGH); //Relais schaltet auf Low
}

void Actor::turnOff()
{
    digitalWrite(outputPin, invert? HIGH : LOW);
}

Actor::Actor(bool inv)
{
	invert = inv;
}