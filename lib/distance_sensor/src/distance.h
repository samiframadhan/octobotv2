#include <Arduino.h>

class DistanceSensor
{
private:
    /* data */
    Stream *interface;
    unsigned char data_buffer[4];
    unsigned char cs;
public:
    DistanceSensor(/* args */);
    ~DistanceSensor();
    void setup_sensor(Stream &serialPort);
    int get_distance();
};

DistanceSensor::DistanceSensor(void) : 
interface{nullptr}, data_buffer{0}, cs{0}
{
}

DistanceSensor::~DistanceSensor()
{
}
