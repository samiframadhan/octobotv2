#include <Arduino.h>

class MotorDC
{
private:
    /* data */
    int pinA;
    int pinB;
public:
    MotorDC(/* args */);
    ~MotorDC();
    void setup(int pinA, int pinB);
    void set_direction(int direction);
};

MotorDC::MotorDC(/* args */):
pinA{0}, pinB{0}
{
}

MotorDC::~MotorDC()
{
}
