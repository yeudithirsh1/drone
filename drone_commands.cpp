#include "drone_commands.h"
#include <iostream>
#include <cmath> // ���� ������� �������
using namespace std;

// ������
const float g = 9.81;  // ����� ����� (�/��)

void startMotorsOrStopMotors(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4,bool d, float initialSpeed)
{

    motor1.isActive = d;
    motor2.isActive = d;
    motor3.isActive = d;
    motor4.isActive = d;

    motor1.direction = -1; 
	motor2.direction = 1;
	motor3.direction = -1;
	motor4.direction = 1;

    motor1.speed = initialSpeed;
    motor2.speed = initialSpeed;
    motor3.speed = initialSpeed;
    motor4.speed = initialSpeed;
}

void increaseMotorSpeed(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4) 
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

float calculateAirDensity(float altitude_m) 
{
    float T0 = 288.15;       // �������� ��� ��� (������)
    float P0 = 101325.0;     // ��� ��� ��� (����)
    float L = 0.0065;        // ��� ����� �������� (K/m)
    float R = 287.05;        // ���� ����� (J/kg*K)
    float g = 9.80665;       // ����� ����� (m/s^2)

    float T = T0 - L * altitude_m;
    float P = P0 * pow((1 - (L * altitude_m) / T0), (g / (R * L)));

    float rho = P / (R * T); // ������ ������

    return rho;
}


// ������� ������ ���� �� �� ��� ����, ���� ������� ������
void updateAltitude(float& altitude, float rpm, float deltaTime, float mass, float& velocity, float rho, float A, float C_d, float C_t)
{
	float thrust = C_t * pow(rpm, 2);//����� ��� ����
	float drag = 0.5 * C_d * A * rho * pow(velocity, 2); // ����� ��� ����
	float weight = mass * g;// ����� ���� �����

    if (thrust > weight) 
    {
		float netForce = thrust - weight - drag;// ����� ���� ����
		float acceleration = netForce / mass;// ����� �����
		velocity += acceleration * deltaTime;//����� ������ �����
		altitude += velocity * deltaTime;// ����� ���� �����
    }
    else 
    {
        std::cout << "The drone cannot take off. Thrust is not enough." << std::endl;
    }
}


void takeof(float mass, float targetAltitude, float A, float C_d, float C_t) {

    Motor motor1;//���� �����
	Motor motor2;//���� ����
	Motor motor3; //����� �����
	Motor motor4; //����� ����
    
    float altitude = 0; // ���� ����� �� �����
    float rpm = 5000;  // ������ ������� ���� (RPM)
    float velocity = 0.0;
    float rho;
    // ����� �������
    startMotorsOrStopMotors(motor1, motor2, motor3, motor4, true, 0);

    // �������� �� �����
    while (altitude < targetAltitude) 
    {
        increaseMotorSpeed(motor1, motor2, motor3, motor4); // ����� ������ �������
        rho = calculateAirDensity(altitude); // ����� ������ ������
        updateAltitude(altitude, mass, rpm, 1, velocity, rho, A, C_d, C_t);  // ����� �� 0.1 �����
        rpm = 5000 + (altitude / 10); // ��� ������ ���� ����, ����� ����� ������ RPM ���
    }
}

void decreaseMotorSpeed(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4)
{
    // ����� ������ ������� ������
    motor1.speed = max(motor1.speed - 5, 0); // ����� ������� �-5 �� ���
    motor2.speed = max(motor2.speed - 5, 0);
    motor3.speed = max(motor3.speed - 5, 0);
    motor4.speed = max(motor4.speed - 5, 0);
    
}


void land(float mass, float targetAltitude, float A, float C_d, float C_t)
{
    Motor motor1; // ���� �����
    Motor motor2; // ���� ����
    Motor motor3; // ����� �����
    Motor motor4; // ����� ����

    float altitude = targetAltitude; // ���� ����� �� ����� (������ ���� ���� ����� �����)
    float rpm = 5000;  // ������ ������� ������ (RPM)
    float velocity = 0.0; // ������ ����
    float rho;

    // ����� �������
    startMotorsOrStopMotors(motor1, motor2, motor3, motor4, true, rpm / 50.0f);

    // �������� �� �����
    while (altitude > 0)
    {
        decreaseMotorSpeed(motor1, motor2, motor3, motor4); // ����� ������ �������
        rho = calculateAirDensity(altitude); // ����� ������ ������
        updateAltitude(altitude, rpm, 1, mass, velocity, rho, A, C_d, C_t); // ����� ���� �� 1 ����

        // ����� RPM ������ ��� ������ ����� �����
        rpm = max(1000.0f, rpm - (altitude / 10)); // �� ����� �-RPM ���� ���� �-1000

        // ����� �� ������ ������ (����)
        if (velocity < 0) 
        {
            velocity = 0;
            startMotorsOrStopMotors(motor1, motor2, motor3, motor4, false, 0);

        }
    }

    cout << "The drone has landed successfully." << std::endl;
}
