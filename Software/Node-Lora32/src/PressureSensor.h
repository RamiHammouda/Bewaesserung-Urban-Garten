#include "Sensor.h"

class PressureSensor : public Sensor
{

public:
    PressureSensor() {};
    void init(){}
    float getPressure_bar();

    ~PressureSensor() {};

};
