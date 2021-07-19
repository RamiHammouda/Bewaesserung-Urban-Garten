#include "Sensor.h"

class MoistureSensor : public Sensor
{

public:
    MoistureSensor() {};
    void init(){}
    float getMoisture_percent();

    ~MoistureSensor() {};

};
