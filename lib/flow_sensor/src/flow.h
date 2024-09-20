#include <Arduino.h>

class FlowSensor
{
private:
    /* data */
    int pinFlow;
    int counter;
public:
    FlowSensor(/* args */);
    ~FlowSensor();
    void setup_sensor();
    int get_flow();
    static void flowPulseCounter();
};

FlowSensor::FlowSensor(/* args */)
{
}

FlowSensor::~FlowSensor()
{
}
