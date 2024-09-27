#include <relay_control.h>

Relay::Relay(/* args */):
pin_1{0}, pin_2{0}, pin_3{0}, pin_4{0}, interface{nullptr}
{
}

Relay::~Relay(){
    
}

void Relay::setup(int pin1, int pin2, int pin3, int pin4){
    // TODO: Implement HAL_TIM_Encoder_Init manually. Read docs
    pinMode(pin_1, OUTPUT);
    pinMode(pin_2, OUTPUT);
    pinMode(pin_3, OUTPUT);
    pinMode(pin_4, OUTPUT);

    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, LOW);
    digitalWrite(pin_3, LOW);
    digitalWrite(pin_4, LOW);
}

void Relay::setup(int pins[4]){
    for (size_t i = 0; i < 4; i++)
    {
        pinMode(pins[i], OUTPUT);
    }
}

void Relay::set_channels(int cases){
    switch (cases)
    {
        // turn off everything
        case 0:
        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
        digitalWrite(pin_3, LOW);
        digitalWrite(pin_4, LOW);
        break;

        // pin_1
        case 1:
        //   delay(500);
        digitalWrite(pin_1, HIGH - digitalRead(pin_1));
        break;

        // pin_2
        case 2:
        //   delay(500);
        digitalWrite(pin_2, HIGH - digitalRead(pin_2));
        break;

        // pin_3
        case 3:
        //   delay(500);
        digitalWrite(pin_3, HIGH - digitalRead(pin_3));
        break;

        // pin_4
        case 4:
        //   delay(500);
        digitalWrite(pin_4, HIGH - digitalRead(pin_4));
        break;

        default:
        break;
    }

}