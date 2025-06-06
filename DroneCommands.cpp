#include "DroneFeatures.h"
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min
#include "GPS.h"
#include <iostream>
#include <chrono>
#include <thread>
#include "controllerPID.h"

using namespace std;
using namespace std::chrono;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// ������
const float T0 = 288.15; // �������� ��� ��� 
const float P0 = 101325.0; // ��� ��� ���
const float L = 0.0065; // ��� ����� �������� 
const float R = 287.05; // ���� ����� 
const float g = 9.80665; // ����� ����� 

void startMotors(Drone& drone)
{
    drone.setMotor1(Motor(true, 0, -1));
    drone.setMotor2(Motor(true, 0, 1));
    drone.setMotor3(Motor(true, 0, -1));
    drone.setMotor4(Motor(true, 0, 1));
}

void updateThrustFromRPM(Drone& drone)
{
    float rpm = drone.getRpm();

    Motor m = drone.getMotor1(); m.speed = rpm; drone.setMotor1(m);
    m = drone.getMotor2(); m.speed = rpm; drone.setMotor2(m);
    m = drone.getMotor3(); m.speed = rpm; drone.setMotor3(m);
    m = drone.getMotor4(); m.speed = rpm; drone.setMotor4(m);
}

void calculateAirDensity(Drone& drone)
{
    float T = T0 - L * drone.getDronePos().z;
    float P = P0 * pow((1 - (L * drone.getDronePos().z) / T0), (g / (R * L)));
    drone.setRho(P / (R * T));
}

void UpdateFollowingProgress(Drone& drone, float dt)
{
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);
    float weight = drone.getMass() * g;
    float drag = 0.5f * drone.getC_d() * drone.getA() * drone.getRho() * pow(drone.getVelocity(), 2);
    float netForce = thrust - weight;
    if (drone.getSpeedInAxes().vz > 0)
        netForce -= drag;
    else
        netForce += drag;

    float az = netForce / drone.getMass();

    Velocity speed = drone.getSpeedInAxes();
    speed.vz += az * dt;
    drone.setSpeedInAxes(speed);

    Point pos = drone.getDronePos();
    pos.z += speed.vz * dt;
    drone.setDronePos(pos);

    float totalVelocity = sqrt(speed.vx * speed.vx + speed.vy * speed.vy + speed.vz * speed.vz);
    drone.setVelocity(totalVelocity);

    Acceleration acc = drone.getAccelerationInAxes();
    acc.az = az;
    drone.setAccelerationInAxes(acc);

    float totalAcceleration = sqrt(acc.ax * acc.ax + acc.ay * acc.ay + acc.az * acc.az);
    drone.setAcceleration(totalAcceleration);
}

void takeoff(Drone& drone)
{
    PID pid(2.0f, 0.5f, 0.3f); // ������ ������ � ���� ����� ���� ������� �����
    startMotors(drone);

    float baseRpm = drone.getHoverSpeed(); // ����� ����� �����
    drone.setRpm(baseRpm);

    auto previousTime = steady_clock::now();

    while (drone.getDronePos().z < drone.getTargetAltitude())
    {
        auto currentTime = steady_clock::now();
        float dt = duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;

        calculateAirDensity(drone);

        float error = drone.getTargetAltitude() - drone.getDronePos().z;
        float correction = pid.update(error, dt);
        float newRpm = baseRpm + correction;

        // ����� RPM ����� ����
        newRpm = max(0.0f, min(newRpm, 2000.0f));

        drone.setRpm(newRpm);
        updateThrustFromRPM(drone);

        UpdateFollowingProgress(drone, dt);

        this_thread::sleep_for(milliseconds(10)); // ����� ����� �� ����� ��� ����
    }
}

//void decreaseMotorSpeed(Drone drone)
//{
//    // ����� ������ ������� ������
//	Motor motor1 = drone.getMotor1();
//    motor1.speed = max(motor1.speed - 5.0f, 0.0f); // ����� ������� �-5 �� ���
//    drone.setMotor1(motor1);
//    Motor motor2 = drone.getMotor2();
//	motor2.speed = max(motor2.speed - 5.0f, 0.0f); // ����� ������� �-5 �� ���
//	drone.setMotor2(motor2);
//	Motor motor3 = drone.getMotor3();
//    motor3.speed = max(motor3.speed - 5.0f, 0.0f);
//    drone.setMotor3(motor3);
//	Motor motor4 = drone.getMotor4();
//    motor4.speed = max(motor4.speed - 5.0f, 0.0f);
//    drone.setMotor4(motor4);
//}
//
//
//void land(Drone drone)
//{
//    // �������� �� �����
//    while (drone.getDronePos().z > 0.1)
//    {
//        drone.setRpm(drone.getRpm() - 10);// ��� ������ ���� ����, ������� �� RPM ���
//        decreaseMotorSpeed(drone); // ����� ������ �������
//        calculateAirDensity(drone); // ����� ������ ������
//        updateAltitude(drone, 1); // ����� ���� �� 1 ����
//    }
//    startMotorsOrStopMotors(drone, false, 0);
//    drone.setVelocity(0);
//    drone.setRpm(0);
//    Point point = drone.getDronePos();
//    point.z = 0;
//    drone.setDronePos(point);
//}
//
//
//float hover(Drone drone)
//{
//    startMotorsOrStopMotors(drone, true, 0);
//
//    drone.setRpm(sqrt(drone.getMass() * g / drone.getC_t()));
//    float hoverAltitude = drone.getDronePos().z;
//
//    while (abs(drone.getDronePos().z - hoverAltitude) > 0.1 || abs(drone.getSpeedInAxes().vz) > 0.1)
//    {
//        calculateAirDensity(drone);      // ����� ������ ������
//        updateAltitude(drone, 0.1);        // ����� ������� �� ����� ��������
//
//        // ������� RPM ����� ������ �����
//        if (drone.getDronePos().z < hoverAltitude)
//            drone.setRpm(drone.getRpm() + 5);
//        else if (drone.getDronePos().z > hoverAltitude)
//            drone.setRpm(drone.getRpm() - 5);
//
//        // Replace the problematic line with explicit usage of std::max and std::min  
//        drone.setRpm(max(0.0f, min(drone.getRpm(), drone.getMaxRPM())));
//    }
//    return drone.getRpm();
//}
//
//void setSpeed(Drone drone, Velocity newVelocity) {
//    drone.setSpeedInAxes(newVelocity);
//    drone.setVelocity(sqrt(newVelocity.vx * newVelocity.vx +
//        newVelocity.vy * newVelocity.vy +
//        newVelocity.vz * newVelocity.vz));
//}
//
//
//void adjustMotorsToRotate(Drone drone, float currentYaw, float targetYaw) {
//    float yawError = targetYaw - currentYaw;
//
//    float balancingSpeed = hover(drone);
//
//    Motor motor1 = drone.getMotor1();
//    motor1.speed = balancingSpeed; // ����� ������� �-5 �� ���
//    drone.setMotor1(motor1);
//    Motor motor2 = drone.getMotor2();
//    motor2.speed = balancingSpeed; // ����� ������� �-5 �� ���
//    drone.setMotor2(motor2);
//    Motor motor3 = drone.getMotor3();
//    motor3.speed = balancingSpeed;
//    drone.setMotor3(motor3);
//    Motor motor4 = drone.getMotor4();
//    motor4.speed = balancingSpeed;
//    drone.setMotor4(motor4);
//    // ����� ����� ������� ������ ��� ������
//    float adjustment = yawError * 2.0f; // ���� ����, ���� �����
//
//    while (yawError > 180.0f) yawError -= 360.0f;
//    while (yawError < -180.0f) yawError += 360.0f;
//
//    // ����� �����
//    if (adjustment > drone.getMaxRPM() / 2) adjustment = drone.getMaxRPM() / 2;
//    if (adjustment < -drone.getMaxRPM() / 2) adjustment = -drone.getMaxRPM() / 2;
//
//    if (yawError > 0) {
//        // ����� �����
//        motor1.speed -= adjustment;
//        motor2.speed += adjustment;
//        motor3.speed += adjustment;
//        motor4.speed -= adjustment;
//    }
//    else {
//        // ����� �����
//        motor1.speed += adjustment;
//        motor2.speed -= adjustment;
//        motor3.speed -= adjustment;
//        motor4.speed += adjustment;
//    }
//
//    // maxRPM ����� ������ ������ ��� 0 �� 
//    Motor* motors[] = { &motor1, &motor2, &motor3, &motor4 };
//    for (int i = 0; i < 4; i++) {
//        if (motors[i]->speed > drone.getMaxRPM()) motors[i]->speed = drone.getMaxRPM();
//        if (motors[i]->speed < 0) motors[i]->speed = 0;
//    }
//}
//
//
//void updateOrientation(Drone drone, float dt)
//{
//    float clockwiseThrust = drone.getMotor1().speed + drone.getMotor2().speed;
//    float counterClockwiseThrust = drone.getMotor3().speed + drone.getMotor4().speed;
//
//    // ��� �� ������ ������ ��� �����
//    drone.setYawRate(counterClockwiseThrust - clockwiseThrust * drone.getYawRate());
//
//    // ���� �� ������ (����� ��������� ����� �� 0.01 �����)
//    drone.setYaw(drone.getYaw() + drone.getYawRate() * dt);
//
//    // ��� ������ �� ������ ����� [0, 360)
//    if (drone.getYaw() >= 360.0f) drone.setYaw(drone.getYaw() - 360.0f);
//    if (drone.getYaw() < 0.0f) drone.setYaw(drone.getYaw() + 360.0f);
//}
//
//

//
//
//void rotate(Drone drone, float deltaYawDegrees, float dt)
//{
//    // ����� ����� ����
//    float targetYaw = drone.getYaw() + deltaYawDegrees;
//
//    // ����� ������ �� ����� ������ ����
//    while (abs(drone.getYaw() - targetYaw) > 0.5f)
//    {
//        // ���� ��� ����� (�����/�����)
//        adjustMotorsToRotate(drone, deltaYawDegrees, drone.getYaw());
//
//        // ����� ��� �����
//        updateOrientation(drone, dt);
//
//        // ������ ���
//        wait(0.01f);
//    }
//}
//
//void updateForwardMotion(Drone drone, float deltaTime)
//{
//    //  ����� thrust ����
//    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);
//
//    //  ������
//    float pitch = drone.getPitch(); // ��������
//    float yaw = drone.getYaw();     // ��������
//
//    //  ��� thrust ����� ��� pitch
//    float forwardThrust = thrust * sin(pitch);
//
//    //  ����� ��X ��Y ��� yaw (����� �������)
//    float fx = forwardThrust * cos(yaw); // ���� thrust ���� X
//    float fy = forwardThrust * sin(yaw); // ���� thrust ���� Y
//
//    //  ����� ��� ���� X ��Y
//    Velocity speed = drone.getSpeedInAxes();
//    float dragX = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vx * speed.vx;
//    float dragY = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vy * speed.vy;
//
//    // ����� ���� ���� ������ ������
//    dragX *= (speed.vx > 0) ? -1 : 1;
//    dragY *= (speed.vy > 0) ? -1 : 1;
//
//    //  ��� ���
//    float netFx = fx + dragX;
//    float netFy = fy + dragY;
//
//    //  ����� �����
//    Acceleration acceleration = drone.getAccelerationInAxes();
//    acceleration.ax = netFx / drone.getMass();
//    acceleration.ay = netFy / drone.getMass();
//    drone.setAccelerationInAxes(acceleration);
//
//    //  ����� ������
//    speed.vx += acceleration.ax * deltaTime;
//    speed.vy += acceleration.ay * deltaTime;
//    drone.setSpeedInAxes(speed);
//
//    //  ����� �����
//    Point pos = drone.getDronePos();
//    pos.x += speed.vx * deltaTime;
//    pos.y += speed.vy * deltaTime;
//    drone.setDronePos(pos);
//
//    //  ����� ������
//    float vTotal = sqrt(speed.vx * speed.vx + speed.vy * speed.vy + speed.vz * speed.vz);
//    float aTotal = sqrt(acceleration.ax * acceleration.ax + acceleration.ay * acceleration.ay + acceleration.az * acceleration.az);
//    drone.setVelocity(vTotal);
//    drone.setAcceleration(aTotal);



void startForwardMotion(Drone& drone, bool d, float pitch, float engineSpeed)
{
    drone.setMotor1(Motor(true, engineSpeed - pitch, -1));//����� ����
    drone.setMotor2(Motor(true, engineSpeed - pitch, 1));//����� ����
    drone.setMotor3(Motor(true, engineSpeed + pitch, -1));
    drone.setMotor4(Motor(true, engineSpeed + pitch, 1));
}


float computePitchAngle(Drone& drone, float drag_force) {
    float value = ( drone.getAccelerationInAxes().ax + drag_force / drone.getMass()) / g;
    float pitch_radians = atan(value);
    float pitch_degrees = pitch_radians * 180.0 / M_PI;
    return pitch_radians; 
}


float computeThrust(Drone& drone, float motorRPM) {
    float omega = motorRPM * 2 * M_PI / 60.0; // ����� RPM ������� ������� (���/����)
    return drone.getC_t() * drone.getRho() * drone.getA() * omega * omega;
}


void updateAltitude(Drone& drone, float dt)
{
    float motorRPM[4] = {drone.getMotor1().speed, drone.getMotor2().speed, drone.getMotor3().speed, drone.getMotor4().speed};

    float totalThrust = 0.0f;

    for (int i = 0; i < 4; ++i) {
        float thrust = computeThrust(drone, motorRPM[i]);
        totalThrust += thrust;
    }

    float weight = drone.getMass() * g;

    // ����� ����� ���� Z (�����/�����)
    float pitch = drone.getPitch();
    float yaw = drone.getYaw(); // ����� ����� �����

    float netForceZ = totalThrust * cos(pitch) - weight;
    float accelerationZ = netForceZ / drone.getMass();

    // ����� ����� ����� (����� ����)
    float netForceForward = totalThrust * sin(pitch);
    float accelerationForward = netForceForward / drone.getMass();

    // �������� ����� ����
    float newVelocityForward = drone.getSpeedInAxes().vx + accelerationForward * dt;
    float newVelocityZ = drone.getSpeedInAxes().vz + accelerationZ * dt;

    // ����� ������ ����� ����� ����� (X ��Y)
    float velocityX_world = newVelocityForward * cos(yaw);
    float velocityY_world = newVelocityForward * sin(yaw);

    // ����� ����� ��� ���� �����
    Point currentPos = drone.getDronePos();
    currentPos.x += velocityX_world * dt;
    currentPos.y += velocityY_world * dt;
    currentPos.z += newVelocityZ * dt;

    // ����� �����
    Velocity newVelocityInAxes = { newVelocityForward, drone.getSpeedInAxes().vy, newVelocityZ };
    drone.setSpeedInAxes(newVelocityInAxes);
    drone.setDronePos(currentPos);
}

void wait(float seconds) {
    this_thread::sleep_for(chrono::milliseconds(static_cast<int>(seconds * 1000)));
}



void moveForward(Drone& drone, Point targetPosition, float stopThreshold, float dt)
{
    PID pidPitch(0.1, 0.01, 0.05); // ���� ����� �� ������� ��� ������� �����
    pidPitch.reset();

    while (true)
    {
        float distance = sqrt(pow(targetPosition.y - drone.getDronePos().x, 2) + pow(targetPosition.y - drone.getDronePos().y, 2) + pow(targetPosition.z - drone.getDronePos().z, 2));

        if (distance <= stopThreshold) {
            drone.setDronePos(targetPosition);
            break;
        }

        calculateAirDensity(drone); // ����� ������ ����� ��� ����

        // ����� ������ ������
        float currentVelocity = drone.getVelocity();

        // ������ �� ����� �-pitch ��� PID
        float pitchCommand = pidPitch.update(distance, dt);

        // ������� �� �-pitch ������ �����
        pitchCommand = clamp(pitchCommand, -0.3f, 0.3f); // �������

        // ����� pitch
        drone.setPitch(pitchCommand);

        // ����� ������� ������ �����
        startForwardMotion(drone, true, pitchCommand, drone.getHoverSpeed());

        // ����� ������ ��������
        updateAltitude(drone, dt);

        wait(dt);
    }

    // ����� �����
    drone.setPitch(0);
}
