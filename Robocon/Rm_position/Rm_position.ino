#include <due_can.h>
#include <ESP32Servo.h>
#include <ros2arduino.h>
#include "rm_set.h"

using namespace ros2;

void rm_position(std_msgs::Int16 i,std_msgs:: void* arg)
{
 catapult_current = i;
 /** if (PS4.Circle()){
    carbase_motors.reset_gearbox_pos(3);
  }
 **/
  if (catapult_current && !catapult_prev){
    carbase_motors.set_target_pos(3, carbase_motors.target_pos[3] - 157313);
  }
  catapult_prev = catapult_current;
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

void setup() 
{
  Serial.begin(115200);

  CAN.onReceive(canCallback);
  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

class Controller : public ros2::Node {

  public:
    Controller() : Node("functions_arduino_node") {
      this->createSubscriber<std_msgs::Int16>("trianglecross_cmd", (ros2::CallbackFunc)rm_position, nullptr);
    }
};

void loop()
{
  ros2::spin(&controller);
  send_can_frame(0x200);
}
