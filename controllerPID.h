#pragma once
class PID {
private:
    float kp, ki, kd;
    float prevError = 0;
    float integral = 0;

public:
    PID(){}
    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}
    float update(float error, float dt);
    void reset();

};
