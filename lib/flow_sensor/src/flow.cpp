#include <flow.h>

FlowSensor::FlowSensor(/* args */):
saved_time{0}, ppr{0}, pinFlow{0}, p{}, port{nullptr}
{
}

FlowSensor::~FlowSensor()
{
}

void FlowSensor::setup_sensor(int pinFlowSet){
    // TODO: Implement HAL_TIM_Encoder_Init manually. Read docs
    p = digitalPinToPinName(pinFlowSet);
    port = set_GPIO_Port_Clock(STM_PORT(p));
    if(!port){
        return;
    }
    counter = 0;
    pinFlow = pinFlowSet;
    saved_time = millis();
    setup_sensor_interrupt();
}

void FlowSensor::setup_sensor_interrupt(){
    // Docs: WInterrupts.cpp
    stm32_interrupt_enable(port, STM_GPIO_PIN(p), FlowSensor::flowPulseCounter, GPIO_MODE_IT_RISING);
}

void FlowSensor::flowPulseCounter(){
    counter ++;
}

void FlowSensor::disable_interrupt(){
    // Docs: WInterrupts.cpp
    stm32_interrupt_disable(port, STM_GPIO_PIN(p));
}

float FlowSensor::get_flow(){
    PinName p = digitalPinToPinName(pinFlow);
    GPIO_TypeDef *port = set_GPIO_Port_Clock(STM_PORT(p));
    if(!port){
        return 0;
    }
    disable_interrupt();
    long int diff_time = millis() - saved_time;
    float spd = (counter/ppr)/diff_time * 60000/*1000 millisecond -> 1 sec*/; // Unit: rpm; SPD = (pulse count/ppr) * 1/time elapsed (in minutes)
    setup_sensor_interrupt();
    return spd;
}