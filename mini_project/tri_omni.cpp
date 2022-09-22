#include "tri_omni.h"
#include <math.h>
#include <Arduino.h>

Tri_omni::Tri_omni(double wheel_radius, double carbase_radius) {
  this->wheel_radius = wheel_radius;
  this->carbase_radius = carbase_radius;
  double cm[NUM_OF_WHEEL][3] = {{0.5, SQRT3_2, this->carbase_radius}, {0.5, -SQRT3_2, this->carbase_radius}, {-1, 0, this->carbase_radius}};
  for (int i = 0; i < NUM_OF_WHEEL; i++){
    for (int j = 0; j < 3; j++){
      this->carbase_matrix[i][j] = cm[i][j];
    }
  }
  
}

void Tri_omni::getMovement(int *movement, double linear_x, double linear_y, double angular_z) {
  static double move_vel = 0.0;

  for(int i = 0; i < this->NUM_OF_WHEEL; i++) {
    move_vel = (this->carbase_matrix[i][0] * linear_x + this->carbase_matrix[i][1] * linear_y + this->carbase_matrix[i][2] * angular_z) / (this->wheel_radius * M_PI * 2);


    move_vel = move_vel * 1152;
    
    move_vel = constrain(move_vel, -this->MAX_RPM, this->MAX_RPM);
    movement[i] = int(move_vel);
  }
}
