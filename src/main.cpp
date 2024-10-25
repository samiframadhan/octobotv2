/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#define USE_FREERTOS
// #undef USE_FREERTOS
#define STM_CODE 0
// STM Only = 0
// Dry STM = 1

#include <Arduino.h>

#include <stm_include.h>

#ifdef USE_FREERTOS
#include <STM32FreeRTOS.h>
#endif
// #include <FreeRTOS.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN PC13
#endif

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#pragma region Micro ROS Objects

std_msgs__msg__Int64 distL;
rcl_publisher_t distL_publisher;

std_msgs__msg__Int64 distR;
rcl_publisher_t distR_publisher;

std_msgs__msg__Float32 wireVal;
rcl_publisher_t wire_publisher;

std_msgs__msg__Int8 motor_dir_msg;
rcl_subscription_t motor_subscriber;

std_msgs__msg__Int32 lac_msg;
rcl_subscription_t lac_subscriber;
int pose = 0;

std_msgs__msg__Float32 flowVal;
rcl_publisher_t flow_publisher;

rcl_service_t motor_status_service;
std_srvs__srv__Trigger_Request motor_status_req;
std_srvs__srv__Trigger_Response motor_status_res;

rcl_service_t relay_service;
stm_interface__srv__RelayControl_Request relay_req;
stm_interface__srv__RelayControl_Response relay_res;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t flow_timer;
rcl_timer_t wire_timer;

#pragma endregion

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

#pragma region STM_Only

void flowPulseCounter(){
  pulseCount ++;
}

void flow_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // RCSOFTCHECK(rcl_publish(&flow_publisher, &flowVal, NULL));
    // flowVal.data++;
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN));

    flowRate = ((FLOW_DELAY / (millis() - prev_flow_millis)) * pulseCount) / FLOW_CONSTANT;
    prev_flow_millis = millis();

    flowRate_ml = (flowRate / 60) * 1000;
    totalVolume  += flowRate_ml;

    debug("flowRate: ");
    debug(flowRate_ml);
    debug(" mL/s | ");


    debug("flowVolume: ");
    debug(totalVolume);
    debugln(" mL");
    pulseCount = 0;

    // float rflow = flow_sensor.get_flow();
    flowVal.data = flowRate_ml;
    RCSOFTCHECK(rcl_publish(&flow_publisher, &flowVal, NULL));
    // // flow_pub.publish(&flowVal);

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseCounter, RISING);
  }
}

void wire_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&wire_publisher, &wireVal, NULL));
    // wireVal.data++;
    // int16_t adcVal = D_WIRE.readADC(0);
    // float mVolt = adcVal * (ADS_GAIN / ADC_VAL_15);
    // float mAmp = mVolt / MV_TO_AMP;

    // debug("ADC: ");
    // debug(adcVal);
    // debug(" | mVolt: ");
    // debug(mVolt);
    // debug(" | mAmp: ");
    // debugln(mAmp);

    // wireVal.data = mAmp;
    // RCSOFTCHECK(rcl_publish(&wire_publisher, &wireVal, NULL));
    // wire_pub.publish(&wireVal);
  }
}

void gpioInit()
{
  /** I2C */
  Wire.setSDA(SDA1_PIN);
  Wire.setSCL(SCL1_PIN);
  Wire.begin();

  // draw wire
  D_WIRE.begin();
  D_WIRE.setGain(0);


  // 12V dc motor cytron
  motorDC_control.setup(ACT_A1_PIN, ACT_A2_PIN);

  // lin-servo (pwm-out)
  servo_control.setup(LINEAR_SERVO_PIN);

  // switch
  relay_control.setup(RELAY_1, RELAY_2, RELAY_3, RELAY_4);

  // flow sensor
  pinMode(FLOW_SENSOR_PIN, INPUT);

  // inbuilt led
  pinMode(LED_PIN, OUTPUT);

  // 12V DC motor (cytron)
  digitalWrite(LED_PIN, HIGH);

  // flow sensor pulse count
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseCounter, RISING);
}

void get_distL(){
  int distance = distance_left.get_distance();
  distL.data = distance;
  RCSOFTCHECK(rcl_publish(&distL_publisher, &distL, NULL));
}

void get_distR(){
  int distance = distance_right.get_distance();
  distR.data = distance;
  RCSOFTCHECK(rcl_publish(&distR_publisher, &distR, NULL));
}

void pose_sub(const void * msgin){
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;

  pose = msg->data;
}

void motor_service_callback(const void * req, void * res){
  motorDC_control.set_direction(motorFlag);
  motorFlag = !motorFlag;

  motor_status_res.success = true;
}

void relay_service_callback(const void * req, void * res){
  relay_control.set_channels(relay_req.data);
  digitalWrite(LED_PIN, HIGH - digitalRead(LED_PIN));
  relay_res.response = true;
}

void motor_sub(const void * msgin){
  motorDC_control.set_direction(motor_dir_msg.data);
}

#pragma endregion

static void ros_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void servo_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    servo_control.set_length(pose);
    vTaskDelay(pdMS_TO_TICKS(107));
  }
}

static void motorDC_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    motorDC_control.set_direction(motor_dir_msg.data);
    vTaskDelay(pdMS_TO_TICKS(101));
  }
}

static void relay_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    relay_control.set_channels(relay_req.data);
    vTaskDelay(pdMS_TO_TICKS(102));
  }
  
}

static void distance_left_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    get_distL();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}

static void distance_right_loop(void* arg){
  UNUSED(arg);
  while (1)
  {
    get_distR();
    vTaskDelay(pdMS_TO_TICKS(104));
  }
  
}

void ros_setup(){
  #pragma region Micro ROS Initialization
  set_microros_serial_transports(Serial1);
  
  allocator = rcl_get_default_allocator();

  // create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  rcl_ret_t is_successfull;
  is_successfull = rclc_support_init(&support, 0, NULL, &allocator);
  if (is_successfull != RCL_RET_OK)
  while (is_successfull != RCL_RET_OK)
  {
    delay(100);
    is_successfull = rclc_support_init(&support, 0, NULL, &allocator);
  }
  

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_node", "", &support));

  #pragma region Publishers
  // create publisher

  RCCHECK(rclc_publisher_init_default(
    &distL_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "distance_left_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &distR_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    "distance_right_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &flow_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "flow_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &wire_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "wire_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
		&lac_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"servo_pose"));

  RCCHECK(rclc_subscription_init_default(
		&motor_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
		"motor_direction"));

  // create services
  RCCHECK(rclc_service_init_default(
    &motor_status_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "motor_trig"
  ));

  RCCHECK(rclc_service_init_default(
    &relay_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(stm_interface, srv, RelayControl),
    "relay_toggle_channel"
  ));
  
  // create timer,
  RCCHECK(rclc_timer_init_default(
    &wire_timer,
    &support,
    RCL_MS_TO_NS(WIRE_DELAY),
    wire_callback));

  RCCHECK(rclc_timer_init_default(
    &flow_timer,
    &support,
    RCL_MS_TO_NS(FLOW_DELAY),
    flow_callback));

  #pragma endregion

  #pragma region Executor Initializations
  
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_dir_msg, &motor_sub, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lac_subscriber, &lac_msg, &pose_sub, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(
    &executor, &motor_status_service, &motor_status_req, &motor_status_res, motor_service_callback));
  RCCHECK(rclc_executor_add_service(
    &executor, &relay_service, &relay_req, &relay_res, relay_service_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &flow_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &wire_timer));
  #pragma endregion

  #pragma endregion
}

void setup()
{
  // initialize LED digital pin as an output.

  #pragma region STM_Only
  // SerialUSB.begin();

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
  
  Serial1.begin(115200);
  SerialL.begin(9600);
  SerialR.begin(9600);
  distance_left.setup_sensor(SerialL);
  distance_right.setup_sensor(SerialR);

  ros_setup();

  if(digitalPinToInterrupt(FLOW_SENSOR_PIN) == NOT_AN_INTERRUPT){
    debug("Pin ");
    debug(FLOW_SENSOR_PIN);
    debugln("can't take interrupt");
  }
  else{
    gpioInit();
  }

  portBASE_TYPE s1 = xTaskCreate(ros_loop, NULL, configMINIMAL_STACK_SIZE + 11000, NULL, 1, NULL);

  portBASE_TYPE s2 = xTaskCreate(distance_left_loop, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

  portBASE_TYPE s3 = xTaskCreate(distance_right_loop, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

  portBASE_TYPE s4 = xTaskCreate(servo_loop, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

  portBASE_TYPE s5 = xTaskCreate(motorDC_loop, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

  portBASE_TYPE s6 = xTaskCreate(relay_loop, NULL, configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

  vTaskStartScheduler();
  #pragma endregion
}

void loop()
{
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  // servo_control.set_length(pose);
  // get_distL();
  // get_distR();
}
