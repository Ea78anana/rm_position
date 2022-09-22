#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>

rcl_subscription_t subscriber;
std_msgs__msg__Int16 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#include <CAN.h>

#include "rm_set.h"
#include "driver.h"
//#include "tri_omni.h"
//#include <ESP32Servo.h>

//using namespace ros2;

#define ELEVATOR 18
#define CSXY 23
#define CSZ 25
#define ELEVATORA 110
#define ELEVATORB 180
#define CSXYA 0
#define CSXYB 180
#define CSZA 0
#define CSZB 110

#define INTAKE 27
#define LEDR 32
#define LEDG 33
#define LEDB 26
//#define XRCEDDS_PORT Serial

int lx;
int ly;
int rx;
int rx_id;
char can_rx[8];
Rm_set carbase_motors(0x200);
Carbase carbase(4, 0.6, 1.57, 3.14);

int carbase_motor_rpm[3] = {0};
int last_time = 0;

/**
Servo elevator;  //(80-125)
Servo csxy;  // (0-180)
Servo csz;  // (80-180)


int elevator_pos;
int csxy_pos;
int csz_pos;

bool intake_temp_on = false;
bool intake_toggled_on = false;
bool intake_prev = false;
bool intake_current = false;

bool catapult_prev = false;


// 0: elevator down
// 1: elevator up
// 2: xy retract
// 3: xy extend
// 4: z down
// 5: z up
//bool servo_changes[6] = {false};

/**
void rm_position(std_msgs::Bool catapult_current, void* arg){
  if (catapult_current.data == !catapult_prev){
    carbase_motors.set_target_pos(0, carbase_motors.target_pos[0] - 157313);
    Serial.println("1");
  }
  catapult_prev = catapult_current.data;
}
**/
void error_loop(){
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  if (msg->data == 1){
    carbase_motors.set_target_pos(0, carbase_motors.target_pos[0] - 157313);
    Serial.println("OK");
  }
}

void send_can_frame(int id){
  CAN.beginPacket(id);
  CAN.write(carbase_motors.can_msg, 8);
  CAN.endPacket();
}

void canCallback(int packetSize){
  if (packetSize) {
    rx_id = CAN.packetId();
    if (0x200 < rx_id && 0x205 > rx_id){
      for (int j = 0; j < 8; j++){
        can_rx[j] = CAN.read();
      }
      carbase_motors.update_motor_status(rx_id - 0x201, micros(),can_rx[0] << 8 | can_rx[1], can_rx[2] << 8 | can_rx[3]); 
    }
  }
}


void setup() {
  set_microros_transports();

  CAN.onReceive(canCallback);
  if(!CAN.begin(1000E3)){
    Serial.println("Starting CAN failed!");
    while(1);
  }

  carbase_motors.reset_gearbox_pos(0);

  carbase_motors.set_target_rpm(0, -300);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rm_position", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "Triangle_cmd"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}


/**
class Controller : public ros2::Node{

  public:
    Controller() : Node("controller_node"){
      this->create_subscription<std_msgs::Bool>("Triangle_cmd", (ros2::CallbackFunc)rm_position, nullptr);
    }
};


void setup() 
{
  XRCEDDS_PORT.begin(115200);
  while(!XRCEDDS_PORT);
  init(&XRCEDDS_PORT);
  Serial.println("done");
 
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  
  for (int i = 0; i < 3; i++){
     carbase_motors.set_pid_rpm(i, 1.5, 0.01, 0); 
  }
     carbase_motors.set_pid_rpm(3, 2, 0.01, 0); 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  csxy.setPeriodHertz(50);
  csz.setPeriodHertz(50);
  elevator.attach(ELEVATOR, 500, 2500); 
  csxy.attach(CSXY, 500, 2500);
  csz.attach(CSZ, 500, 2500); 

  // servo init pos
  elevator_pos = 180;
  elevator.write(elevator_pos);
  csz_pos = 0;
  csz.write(csz_pos);
  delay(500);
  csxy_pos = 180;
  csxy.write(csxy_pos);
  delay(500);
  csz_pos = 110;
  csz.write(csz_pos);
  delay(500);

  pinMode(INTAKE, OUTPUT);
}
**/


void loop() 
{ /** 
  static Controller rm_controller;

  ros2::spin(&rm_controller);
  Serial.println("Test");
  **/
  if(CAN.begin(1000E3)){
    Serial.println("Starting CAN Succeed!");
  }
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  send_can_frame(0x200);
}
