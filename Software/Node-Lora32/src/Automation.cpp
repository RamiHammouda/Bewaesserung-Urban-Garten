#include "Automation.h"

void Automation::turnActor(Actor &actor, bool &onOff)
{
    if (onOff)
    {
        actor.turnOn();
    }
    else
    {
        actor.turnOff();
    }
}

bool Automation::set(int sensorValue,string comparisonOperator, int valueToReach, Actor &actor, bool onOff)
{

    if(comparisonOperator == ">")
    {
        if (sensorValue > valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == ">=")
    {
        if (sensorValue >= valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "==")
    {
        if (sensorValue == valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "<=")
    {
        if (sensorValue <= valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "<")
    {
        if (sensorValue < valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else
    {
        return false;
    }

    return true;
}

//Überladung
bool Automation::set(float sensorValue,string comparisonOperator, float valueToReach, Actor &actor, bool onOff)
{

    if(comparisonOperator == ">")
    {
        if (sensorValue > valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == ">=")
    {
        if (sensorValue >= valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "==")
    {
        if (sensorValue == valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "<=")
    {
        if (sensorValue <= valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else if(comparisonOperator == "<")
    {
        if (sensorValue < valueToReach)
        {
            turnActor(actor, onOff);
        }
    }
    else
    {
        return false;
    }

    return true;
}
