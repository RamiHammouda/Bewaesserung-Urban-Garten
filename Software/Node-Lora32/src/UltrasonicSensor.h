#include "Sensor.h"

class UltrasonicSensor : public Sensor
{
private:
    uint8_t triggerPin;

public:
    UltrasonicSensor() {};
    bool set_trig_echo(uint8_t trig, uint8_t echo);
    bool set_trig(uint8_t trig);
    void init(){}

    int getDistance_cm();

    ~UltrasonicSensor() {};
};
