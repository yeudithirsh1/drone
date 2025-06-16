#include "DroneFeatures.h"
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min
#include "GPS.h"
#include <iostream>
#include <chrono>
#include <thread>
#include "controllerPID.h"
#include "KalmanFilter.h"
#include <Eigen/Dense>
#include "Global.h"

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


//����� ������� ������� ������ ������ ������� ����� ��� ������ ����� �������
//���� 1 - CW ����� ����� ���� ���� �����
//���� 2 - CCW ��� ����� ����� ���� ����
//���� 3 - CCW ��� ����� ����� ����� �����
// ���� 4 - CW ����� ����� ����� ����

void initMotorsWithRPM(Drone& drone)
{
    float rpm = drone.getHoverSpeed();

    drone.setMotor1(Motor(true, rpm, -1));
    drone.setMotor2(Motor(true, rpm, 1));
    drone.setMotor3(Motor(true, rpm, -1));
    drone.setMotor4(Motor(true, rpm, 1));
}

// ����� ������ ������ ��� ���� �����
// ���� ����� ���� Z
// �������� ����� �����
// ��� ����� �����
// ����� ������ ������ �� ����� ��� �����

void calculateAirDensity(Drone& drone)
{
	float dronePosZ = drone.getDronePos().z;
	float T = T0 - L * dronePosZ;
	float P = P0 * pow((1 - (L * dronePosZ) / T0), (g / (R * L)));
    drone.setRho(P / (R * T));
}

void updateThrustFromRPM(Drone& drone)
{
    float rpm = drone.getRpm();

    Motor m = drone.getMotor1(); m.speed = rpm; drone.setMotor1(m);
    m = drone.getMotor2(); m.speed = rpm; drone.setMotor2(m);
    m = drone.getMotor3(); m.speed = rpm; drone.setMotor3(m);
    m = drone.getMotor4(); m.speed = rpm; drone.setMotor4(m);
}


void UpdateFollowingProgress(Drone& drone, float dt, KalmanFilter& kalmanFilter)
{
    // ����� ���� ���� ����
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);

    // ���� ��� ������ ����� ����� ����� ����
    float pitch = drone.getPitch();
    float yaw = drone.getYaw();

    // ��� ������ ���� z �� ����
    float gravityZ_body = cos(pitch) * (-drone.getMass() * g);

    float drag = 0.5f * drone.getC_d() * drone.getA() * drone.getRho() * pow(drone.getVelocity(), 2);

    float netForceZ = thrust + gravityZ_body;  // gravityZ_body ����� - ����� ����

    if (drone.getSpeedInAxes().vz > 0)
        netForceZ -= drag;
    else
        netForceZ += drag;

    // ����� ����� ���� z �� ����
    float az = netForceZ / drone.getMass();

    // ����� ����� ������ ����� ����
    Acceleration acceleration = drone.getAccelerationInAxes();
    acceleration.az = az;
    VectorXf vectorControl;
    vectorControl << acceleration.ax, acceleration.ay, acceleration.az, yaw, drone.getYawRate(), pitch, drone.getPitchRate();
    // ����� ���� ���� �� ������ ��������
    kalmanFilter.externalInputUpdate(vectorControl);
}


void takeoff(Drone& drone, KalmanFilter& kalmanFilter)
{
    PID pid(2.0f, 0.5f, 0.3f); 
    initMotorsWithRPM(drone);

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

        // ����� RPM ����� ���� ��� ���� ����� ��� ���� ���� ����
        newRpm = max(0.0f, min(newRpm, 2000.0f));

        drone.setRpm(newRpm);
        updateThrustFromRPM(drone);

        UpdateFollowingProgress(drone, dt, kalmanFilter);

        this_thread::sleep_for(milliseconds(10)); // ����� ����� �� ����� ��� ����
    }
}

void stopMoter(Drone& drone) {
    Motor m(false, 0, 0);
    drone.setMotor1(m);
    drone.setMotor2(m);
    drone.setMotor3(m);
    drone.setMotor4(m);
}

void land(Drone& drone, KalmanFilter& kalmanFilter)
{
    PID pid(1.5f, 0.4f, 0.2f); // ������ ������ ���� � ������ ������ ��� ������� ������
    float baseRpm = drone.getHoverSpeed(); // ������ ��� ����� ��� ����
    drone.setRpm(baseRpm);

    auto previousTime = steady_clock::now();

    while (drone.getDronePos().z > 0.05f || drone.getSpeedInAxes().vz > 0.05f) // ����� ����� ����� ������� ���� ��� �����
    {
        auto currentTime = steady_clock::now();
        float dt = duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;

        calculateAirDensity(drone);

        float error = drone.getTargetAltitude() - drone.getDronePos().z;
        float correction = pid.update(error, dt);
        float newRpm = baseRpm + correction;

        // ����� RPM ����� ���� ���� ��� �����
        newRpm = max(0.0f, min(newRpm, baseRpm));

        drone.setRpm(newRpm);
        updateThrustFromRPM(drone);

        UpdateFollowingProgress(drone, dt, kalmanFilter);

        this_thread::sleep_for(milliseconds(10));
    }

    // ����� ������ ���� �����
    drone.setRpm(0.0f);

}

//float hover(Drone& drone)
//{
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

float deg2rad(float deg) { return deg * M_PI / 180.0f; }
float rad2deg(float rad) { return rad * 180.0f / M_PI; }

float normalizeAngle(float angle) {
    while (angle > M_PI)
        angle -= 2.0f * M_PI;
    while (angle <= -M_PI)
        angle += 2.0f * M_PI;
    return angle;
}


void adjustMotorsToRotate(Drone& drone, float currentYawRad, float targetYawRad, PID& pid, float dt) {
    float yawError = normalizeAngle(targetYawRad - currentYawRad);

    float adjustment = pid.update(yawError, dt);
    float baseSpeed = drone.getHoverSpeed();

    Motor motor1 = drone.getMotor1(); // CW
    Motor motor2 = drone.getMotor2(); // CCW
    Motor motor3 = drone.getMotor3(); // CCW
    Motor motor4 = drone.getMotor4(); // CW

    // ����� ������ ��� �����, ����� ������ �����
    motor1.speed = baseSpeed - adjustment; // CW
    motor2.speed = baseSpeed + adjustment; // CCW
    motor3.speed = baseSpeed + adjustment; // CCW
    motor4.speed = baseSpeed - adjustment; // CW

    // ������
    Motor* motors[] = { &motor1, &motor2, &motor3, &motor4 };
    for (int i = 0; i < 4; ++i) {
        if (motors[i]->speed > drone.getMaxRPM()) motors[i]->speed = drone.getMaxRPM();
        if (motors[i]->speed < 0) motors[i]->speed = 0;
    }

    drone.setMotor1(motor1);
    drone.setMotor2(motor2);
    drone.setMotor3(motor3);
    drone.setMotor4(motor4);
}


void updateOrientation(Drone& drone, float dt, KalmanFilter &kalmanFilter) {
    float cwThrust = drone.getMotor1().speed + drone.getMotor4().speed;
    float ccwThrust = drone.getMotor2().speed + drone.getMotor3().speed;

    float yawRateGain = 0.001f;
    float yawRate = (ccwThrust - cwThrust) * yawRateGain;


    float yaw = deg2rad(drone.getYaw()) + yawRate * dt;
    yaw = normalizeAngle(yaw);
    VectorXf vectorControl;
    vectorControl << drone.getAccelerationInAxes().ax, drone.getAccelerationInAxes().ay, drone.getAccelerationInAxes().az,
        yaw, yawRate, drone.getPitch(), drone.getPitchRate();
    kalmanFilter.externalInputUpdate(vectorControl);
}


void rotate(Drone& drone, Vector3f startPoint, Vector3f endPoint, KalmanFilter& kalmanFilter) {
    Vector3f direction = (startPoint - endPoint).normalized();

    float targetYawRad = atan2(direction.y(), direction.x());

    PID pidYaw(0.1, 0.01, 0.05);
    pidYaw.reset();

    auto lastTime = chrono::high_resolution_clock::now();
    while (true) {
        auto now = chrono::high_resolution_clock::now();
        float dt = chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        float currentYawRad = deg2rad(drone.getYaw());
        float deltaYaw = normalizeAngle(targetYawRad - currentYawRad);

        if (fabs(deltaYaw) < 0.01f) return;

        adjustMotorsToRotate(drone, currentYawRad, targetYawRad, pidYaw, dt);
        updateOrientation(drone, dt, kalmanFilter);

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    pidYaw.reset();
}


void rotateYaw(Drone & drone, float targetYawDeg, KalmanFilter & kalmanFilter) {
    float targetYawRad = deg2rad(targetYawDeg);

    PID pidYaw(0.1, 0.01, 0.05);
    pidYaw.reset();

    auto lastTime = chrono::high_resolution_clock::now();
    while (true) {
        auto now = chrono::high_resolution_clock::now();
        float dt = chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        float currentYawRad = deg2rad(drone.getYaw());
        float deltaYaw = normalizeAngle(targetYawRad - currentYawRad);

        if (fabs(deltaYaw) < 0.01f) return;

        adjustMotorsToRotate(drone, currentYawRad, targetYawRad, pidYaw, dt);
        updateOrientation(drone, dt, kalmanFilter);

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    pidYaw.reset();
}


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


void updateAltitude(Drone& drone, float dt, KalmanFilter& kalmaFilter)
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
    Velocity speedInAxes = drone.getSpeedInAxes(); // ������ ���� Z �� �����

    float netForceZ = totalThrust * cos(pitch) - weight;
    float accelerationZ = netForceZ / drone.getMass();

    // ����� ����� ����� (����� ����)
    float netForceForward = totalThrust * sin(pitch);
    float accelerationForward = netForceForward / drone.getMass();

    // �������� ����� ����
    float newVelocityForward = speedInAxes.vx + accelerationForward * dt;
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
	VectorXf vectorControl;
	vectorControl << accelerationForward, speedInAxes.vy, accelerationZ, yaw, drone.getYawRate(), pitch, drone.getPitchRate();
	kalmaFilter.externalInputUpdate(vectorControl);
}


void moveForward(Drone& drone, Point targetPosition, float stopThreshold, KalmanFilter& kalmaFilter)
{
    PID pidPitch(0.1, 0.01, 0.05); // ���� ����� �� ������� ��� ������� �����
    pidPitch.reset();

    auto lastTime = chrono::high_resolution_clock::now();

    while (true) {
        {
            shared_lock<shared_mutex> lock(mutexObstacleStatus);
            if (obstacleStatuse) {
                // ����� ������ ����� ����� ���
                VectorXf vectorControl;
                vectorControl << drone.getAccelerationInAxes().ax, drone.getAccelerationInAxes().ay, drone.getAccelerationInAxes().az,
                    drone.getYaw(), drone.getYawRate(), 0.0f, drone.getPitchRate();
                kalmaFilter.externalInputUpdate(vectorControl);

                startForwardMotion(drone, false, 0.0f, 0.0f);
                this_thread::sleep_for(chrono::milliseconds(100)); // ��� ��������� ���� �����
                return;
            }
        }
        auto now = chrono::high_resolution_clock::now();
        float dt = chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        {
            float distance = sqrt(pow(targetPosition.x - drone.getDronePos().x, 2) +
                pow(targetPosition.y - drone.getDronePos().y, 2) +
                pow(targetPosition.z - drone.getDronePos().z, 2));

            if (distance <= stopThreshold) {
                return;
            }

            calculateAirDensity(drone); // ����� ������ ����� ��� ����

            float currentVelocity = drone.getVelocity();

            float pitchCommand = pidPitch.update(distance, dt);
            pitchCommand = clamp(pitchCommand, -0.3f, 0.3f); // �������

            drone.setPitch(pitchCommand);
            startForwardMotion(drone, true, pitchCommand, drone.getHoverSpeed());

            updateAltitude(drone, dt, kalmaFilter);
            this_thread::sleep_for(chrono::milliseconds(10));
        }
    }
}
