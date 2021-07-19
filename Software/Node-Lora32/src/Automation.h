#include "Actor.h"


/// <summary>
/// Makes it easy for the user to make an automation and automatically turn on/off an actor when a certain value of a sensor is reached
///(experiment feature)
/// </summary>
class Automation
{
    private:
        void turnActor(Actor &actor, bool &onOff);

    public: 
        Automation(){}
        
        bool set(int sensorValue,string comparisonOperator, int valueToReach, Actor &actor, bool onOff);
        bool set(float sensorValue,string comparisonOperator, float valueToReach, Actor &actor, bool onOff);

        ~Automation(){}  
};