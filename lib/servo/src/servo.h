#include <Arduino.h>

class ServoControl
{
private:
    /* data */
    int pinPWM;
public:
    ServoControl(/* args */);
    ~ServoControl();
    void setup();
    void set_length(int x);
};

ServoControl::ServoControl(/* args */):
pinPWM{0}
{
}

ServoControl::~ServoControl()
{
}
