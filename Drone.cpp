#include "Drone.h"
#include <iostream>
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min

using namespace std;

// ������
const float T0 = 288.15; // �������� ��� ��� 
const float P0 = 101325.0; // ��� ��� ���
const float L = 0.0065; // ��� ����� �������� 
const float R = 287.05; // ���� ����� 
const float g = 9.80665; // ����� ����� 



void Drone::startMotorsOrStopMotors(bool d, float initialSpeed)
{

    motor1.isActive = motor2.isActive = motor3.isActive = motor4.isActive = d;

    motor1.direction = -1;
    motor2.direction = 1;
    motor3.direction = -1;
    motor4.direction = 1;

    motor1.speed = motor2.speed = motor3.speed = motor4.speed = initialSpeed;
}

void Drone::IncreaseEngineSpeed()
{
    // ����� ������ ������� ������ �� ��������
    if (motor1.speed < 100)
    {
        motor1.speed += 5; // ����� ������� �-5 �� ���
        motor2.speed += 5;
        motor3.speed += 5;
        motor4.speed += 5;
    }
}

void Drone::calculateAirDensity()
{
    float T = T0 - L * dronePos.z;//����� ��������� ����� ���� �����
	float P = P0 * pow((1 - (L * dronePos.z) / T0), (g / (R * L))); // ����� ���� ����� ���� �����

    float rho = P / (R * T); // ������ ������ ����� ���� �����

}

void Drone::updateAltitude(float deltaTime)
{
    float thrust = C_t * pow(rpm, 2);//����� ��� ����
    float drag = 0.5 * C_d * A * rho * pow(velocity, 2); // ����� ��� ����
    float weight = mass * g;// ����� ���� �����
    
    float netForce = thrust - weight;// ����� ���� ����

    if (SpeedInAxes.vz > 0)
    {
        netForce -= drag; // ����� ��� ���� �� ����� ����
    }
    else
    {
		netForce += drag; // ����� ��� ���� �� ����� ����
    }
    AccelerationInAxes.wz = netForce / mass;// ����� �����
    SpeedInAxes.vz += AccelerationInAxes.wz * deltaTime;//����� ������ �����
    dronePos.z += velocity * deltaTime;// ����� ���� �����    
    velocity = sqrt(SpeedInAxes.vx * SpeedInAxes.vx + SpeedInAxes.vy * SpeedInAxes.vy + SpeedInAxes.vz * SpeedInAxes.vz);
    acceleration = sqrt(AccelerationInAxes.wx * AccelerationInAxes.wx + AccelerationInAxes.wy * AccelerationInAxes.wy + AccelerationInAxes.wz * AccelerationInAxes.wz);
}

// ������� ������ ���� �� �� ��� ����, ���� ������� ������
void Drone::takeof()
{
    startMotorsOrStopMotors(true, 0);
    rpm = sqrt(mass*g / C_t);
    // �������� �� �����
    while (dronePos.z < target_altitude)
    {
        IncreaseEngineSpeed(); // ����� ������ �������
        calculateAirDensity(); // ����� ������ ������
        updateAltitude(0.1);  // ����� �� 0.1 �����
        rpm += 10; // ��� ������ ���� ����, ������� �� RPM ���
    }
}


void Drone::decreaseMotorSpeed()
{
    // ����� ������ ������� ������
    motor1.speed = max(motor1.speed - 5, 0); // ����� ������� �-5 �� ���
    motor2.speed = max(motor2.speed - 5, 0);
    motor3.speed = max(motor3.speed - 5, 0);
    motor4.speed = max(motor4.speed - 5, 0);
}


void Drone::land()
{
    // �������� �� �����
    while (dronePos.z > 0.1)
    {
        rpm -= 10;
        decreaseMotorSpeed(); // ����� ������ �������
        calculateAirDensity(); // ����� ������ ������
        updateAltitude(1); // ����� ���� �� 1 ����
    }             
    startMotorsOrStopMotors(false, 0);
    velocity = 0;
    rpm = 0;
	dronePos.z = 0;
}



void Drone::hover()
{
    startMotorsOrStopMotors(true, 0);

    rpm = sqrt(mass * g / C_t);

    float hoverAltitude = dronePos.z;

    while (abs(dronePos.z - hoverAltitude) > 0.1 || abs(SpeedInAxes.vz) > 0.1)
    {
        calculateAirDensity();      // ����� ������ ������
        updateAltitude(0.1);        // ����� ������� �� ����� ��������

        // ������� RPM ����� ������ �����
        if (dronePos.z < hoverAltitude)
            rpm += 5;
        else if (dronePos.z > hoverAltitude)
            rpm -= 5;

        // Replace the problematic line with explicit usage of std::max and std::min  
        rpm = std::max(0.0f, std::min(rpm, maxRPM));
    }
}

void Drone::setSpeed(Velocity newVelocity) {
    SpeedInAxes = newVelocity;
    velocity = sqrt(newVelocity.vx * newVelocity.vx +
        newVelocity.vy * newVelocity.vy +
        newVelocity.vz * newVelocity.vz);
}


void adjustMotorsToRotate(float currentYaw, float targetYaw) {
    float yawError = targetYaw - currentYaw;

	motor1.speed = motor2.speed = motor3.speed = motor4.speed = maxRPM; // ����� �������
    // ����� ����� ������� ������ ��� ������
    float adjustment = yawError * 2.0f; // ���� ����, ���� �����

    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // ����� �����
    if (adjustment > maxRPM / 2) adjustment = maxRPM / 2;
    if (adjustment < -maxRPM / 2) adjustment = -maxRPM / 2;

    if (yawError > 0) {
        // ����� �����
        motor1.speed -= adjustment;
        motor2.speed += adjustment;
        motor3.speed += adjustment;
        motor4.speed -= adjustment;
    }
    else {
        // ����� �����
        motor1.speed += adjustment;
        motor2.speed -= adjustment;
        motor3.speed -= adjustment;
        motor4.speed += adjustment;
    }

    // maxRPM ����� ������ ������ ��� 0 �� 
    Motor* motors[] = { &motor1, &motor2, &motor3, &motor4 };
    for (int i = 0; i < 4; i++) {
        if (motors[i]->speed > maxRPM) motors[i]->speed = maxRPM;
        if (motors[i]->speed < 0) motors[i]->speed = 0;
    }
}

void Drone::updateOrientation()
{

    float clockwiseThrust = motorSpeeds[0] + motorSpeeds[3];
    float counterClockwiseThrust = motorSpeeds[1] + motorSpeeds[2];

    // ��� �� ������ ������ ��� �����
    float yawRate = (counterClockwiseThrust - clockwiseThrust) * yawSensitivity;

    // ���� �� ������ (����� ��������� ����� �� 0.01 �����)
    DroneDirection.yaw += yawRate * 0.01f;

    // ��� ������ �� ������ ����� [0, 360)
    if (DroneDirection.yaw >= 360.0f) DroneDirection.yaw -= 360.0f;
    if (DroneDirection.yaw < 0.0f) DroneDirection.yaw += 360.0f;
}


void Drone::rotate(float deltaYawDegrees)
{
    // ����� ����� ����
    float targetYaw = DroneDirection.yaw + deltaYawDegrees;

    // ����� ������ �� ����� ������ ����
    while (abs(DroneDirection.yaw - targetYaw) > 0.5f)
    {
        // ���� ��� ����� (�����/�����)
        adjustMotorsToRotate(targetOrientation.yaw);

        // ����� ��� �����
        updateOrientation();

        // ������ ���
        wait(0.01f);
    }

    // ����� �����
    stopRotation();
}
