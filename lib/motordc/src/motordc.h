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