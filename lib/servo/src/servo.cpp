#include <servo.h>

ServoControl::ServoControl(/* args */):
pinPWM{0}
{
}

ServoControl::~ServoControl()
{
}

void ServoControl::setup(int pinPWM_){
    pinPWM = pinPWM_;
    pinMode(pinPWM, OUTPUT);
    analogWrite(pinPWM, LOW);
}

void ServoControl::set_length(int pwm){
    analogWrite(pinPWM, pwm);
}