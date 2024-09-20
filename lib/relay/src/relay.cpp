#include <Arduino.h>
#include <relay.h>

void Relay::setup(){
    // TODO: Implement HAL_TIM_Encoder_Init manually. Read docs
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