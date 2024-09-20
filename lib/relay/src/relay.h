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
    void setup();
    void set_channels(int x);
};

Relay::Relay(/* args */)
{
}

Relay::~Relay()
{
}
