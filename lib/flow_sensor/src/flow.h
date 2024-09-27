#include <Arduino.h>

class FlowSensor
{
private:
    /* data */
    long int saved_time;
    int ppr; // Pulse per rotation
    int pinFlow;
    PinName p;
    GPIO_TypeDef *port;
    static int counter; // Warn: It's shared with every object that uses FlowSensor class.
public:
    FlowSensor(/* args */);
    ~FlowSensor();
    void setup_sensor_interrupt();
    float get_flow();
    void disable_interrupt();
    void setup_sensor(int pinFlow);
    static void flowPulseCounter();
};