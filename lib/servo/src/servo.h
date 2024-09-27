#include <Arduino.h>

class ServoControl
{
private:
    /* data */
    int pinPWM;
public:
    ServoControl(/* args */);
    ~ServoControl();
    void setup(int pinPWM_);
    void set_length(int x);
};
