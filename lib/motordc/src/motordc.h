#include <Arduino.h>

class MotorDC
{
private:
    /* data */
    int pinPWM;
public:
    MotorDC(/* args */);
    ~MotorDC();
    void setup();
    void set_speed(int pwm);
};

MotorDC::MotorDC(/* args */):
pinPWM{0}
{
}

MotorDC::~MotorDC()
{
}
