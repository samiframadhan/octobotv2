#include <Arduino.h>

// #ifdef USE_FREERTOS
#include <STM32FreeRTOS.h>
#define sleep(x) vTaskDelay(x / portTICK_PERIOD_MS)
// #else
// #define sleep(x) delay(x)
// #endif

class DistanceSensor
{
private:
    /* data */
    Stream *interface;
    unsigned char data_buffer[4];
    unsigned char cs;
public:
    DistanceSensor(/* args */);
    ~DistanceSensor();
    void setup_sensor(Stream &serialPort);
    int get_distance();
};