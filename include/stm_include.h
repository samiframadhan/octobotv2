/*************************** <include header files> ***************************/
#include <Wire.h>
//#include <SoftwareSerial.h>
#include "ADS1X15.h"

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float32.h>

#include <std_srvs/srv/trigger.h>
#include <stm_interface/srv/relay_control.h>
#include <stm_interface/srv/motor_status.h>
#include <stm_interface/srv/tool_status.h>

#include <relay_control.h>
#include <motordc.h>
#include <servo.h>
#include <distance.h>
#include <flow.h>


/*************************** <UART> ***************************/
/**
  tx1 : pa9, pa15, pb6
  rx1 : pa10, pb3, pb7

  tx2 : pa2
  rx2 : pa3

  tx6 : pa11
  rx6 : pa12
*/

/*************************** <start define> ***************************/

/* Debug */
#define DEBUG 0

#if DEBUG == 1
#define debug(x)      Serial.print(x)
#define debugln(x)    Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif


//generic macros
#define RESET                   (0)
#define SET                     (1)
#define NOT_AVAILABLE           (-1)



// Wire Sensor
#define WIRE_ADDR          (0x48)
#define MAX_LENGTH         (100.00F)
#define FLOAT_ROUND        (4)

#define ADS_GAIN            (4.096F)
#define MV_TO_V             (1000.00F)
#define ADC_VAL_15          (32767.00F)
#define MV_TO_AMP           (10.00F)


// Flow Sensor
#define FLOW_CONSTANT       (1.0F)



//delay
#define WIRE_DELAY        (100.0F) // 0.1 sec
#define FLOW_DELAY        (1000.0F) // 3 sec
#define MOTION_DELAY      (3000.0F) // 3 sec
#define STOP_DELAY        (1000.0F) // 1 sec
#define SERVO_DELAY       (100.0F) // 0.1 sec

// Sensor pins
#define FLOW_SENSOR_PIN         (PB14)
#define LED_PIN                 (PC13)

//DC-motor pins
#define ACT_A1_PIN              (PA8) //PWM-1
#define ACT_A2_PIN              (PB15) //DIR-1

// Linear Servo pin
#define LINEAR_SERVO_PIN        (PB13)

// Switch
#define RELAY_1                 (PB9) //crawler
#define RELAY_2                 (PB6) //light
#define RELAY_3                 (PB5) //bottom light
#define RELAY_4                 (PB4) //pump

//I2C2 pins
#define SCL1_PIN                (PB10)
#define SDA1_PIN                (PB3)

// UART2 pins
#define RX2_PIN                 (PA3)
#define TX2_PIN                 (PA2)

// UART6 pins
#define RX6_PIN                 (PA12)
#define TX6_PIN                 (PA11)


/*************************** <end define> ***************************/

Relay relay_control;
MotorDC motorDC_control;
ServoControl servo_control;
DistanceSensor distance_left;
DistanceSensor distance_right;
// FlowSensor flow_sensor;

HardwareSerial SerialL( RX2_PIN , TX2_PIN);
HardwareSerial SerialR( RX6_PIN , TX6_PIN);

// draw wire
ADS1115 D_WIRE(WIRE_ADDR);
unsigned long prev_wire_millis = RESET;

// linear Servo
int servoPose = SET;
bool servoFlag = true;
unsigned long prev_servo_millis = RESET;

//12V DC motor
bool motorFlag = RESET;

// flow sensor
volatile int16_t pulseCount = RESET;
unsigned long prev_flow_millis = RESET;
float flowRate = RESET;
unsigned int flowRate_ml = RESET;
unsigned long totalVolume = RESET;
unsigned long lastFlowPublishTime = 0;
const unsigned long flowPublishInterval = 3000;

// distance(left)
unsigned char dataL_buffer[4] = {0};
int distanceL  = RESET;
unsigned char csL = RESET; //checksum-left

// distance(right)
unsigned char dataR_buffer[4] = {0};
int distanceR  = RESET;
unsigned char csR = RESET; //checksum-left



//  -- END OF FILE --