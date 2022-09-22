#include "driver.h"
#include <math.h>

Carbase::Carbase(double max_a, double max_v, double max_w, double max_w_a){
    this->max_a = max_a;
    this->max_v = max_v;
    this->max_w = max_w;
    this->max_w_a = max_w_a;
}

void Carbase::update(double target_v[2], double target_w){
    this->target_v[0] = target_v[0];
    this->target_v[1] = target_v[1];
    this->target_w = target_w;
}

void Carbase::update_from_controller(double x, double y, double w){
    this->target_v[0] = x * this->max_v / 128;
    this->target_v[1] = y * this->max_v / 128;
    this->target_w = w * this->max_w / 128;
} 

void Carbase::tick(int micros){
    this->delta_v[0] = this->target_v[0] - this->current_v[0];
    this->delta_v[1] = this->target_v[1] - this->current_v[1];

    double rx = -this->current_v[1];
    double ry = this->current_v[0];
    this->centri_force[0] = rx * this->current_w * this->current_w;
    this->centri_force[1] = ry * this->current_w * this->current_w;
    if (sqrt(this->delta_v[0] * this->delta_v[0] + this->delta_v[1] * this->delta_v[1]) > 0.01){
        double a = this->delta_v[0] * this->delta_v[0] + this->delta_v[1] * this->delta_v[1];
        double b = -2 * (this->delta_v[0] * this->centri_force[0] + this->delta_v[1] * this->centri_force[1]);
        double c = -(this->max_a * this->max_a - this->centri_force[0] * this->centri_force[0]);
        double n1 = 0;
        if (a != 0 && b * b - 4 * a * c > 0){
            n1 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
        }
        this->accel_force[0] = abs(n1) * this->delta_v[0];
        this->accel_force[1] = abs(n1) * this->delta_v[1];
    } else {
        this->accel_force[0] = 0;
        this->accel_force[1] = 0;
    }
    double dvx = this->accel_force[0] * (micros - this->last_time) * 1E-6;
    double dvy = this->accel_force[1] * (micros - this->last_time) * 1E-6;
    this->current_v[0] = this->current_v[0] + (abs(dvx) > abs(this->delta_v[0]) ? this->delta_v[0] : dvx);
    this->current_v[1] = this->current_v[1] + (abs(dvy) > abs(this->delta_v[1]) ? this->delta_v[1] : dvy);

    if (this->current_w < this->target_w){
        this->current_w += this->max_w_a > this->target_w - this->current_w ? this->target_w - this->current_w : this->max_w_a;
    } else {
        this->current_w -= this->max_w_a > this->current_w - this->target_w ? this->current_w - this->target_w : this->max_w_a;
    }

    this->last_time = micros;
}
