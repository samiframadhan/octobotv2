#include <Arduino.h>

class Relay
{
private:
    /* data */
    int pin_1;
    int pin_2;
    int pin_3;
    int pin_4;
    Stream *interface;
public:
    Relay(/* args */);
    ~Relay();
    void setup(int pin1, int pin2, int pin3, int pin4);
    void setup(int pins[4]);
    void set_channels(int x);
};

Relay::Relay(/* args */):
pin_1{0}, pin_2{0}, pin_3{0}, pin_4{0}, interface{nullptr}
{
}

Relay::~Relay()
{
}
