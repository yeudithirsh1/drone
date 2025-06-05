#include "controllerPID.h"
float PID::update(float error, float dt) {
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
    prevError = 0;
    integral = 0;
}
