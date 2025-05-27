#include "DroneFeatures.h"
#include <iostream>
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min
#include <windows.h>

using namespace std;

// ������
const float T0 = 288.15; // �������� ��� ��� 
const float P0 = 101325.0; // ��� ��� ���
const float L = 0.0065; // ��� ����� �������� 
const float R = 287.05; // ���� ����� 
const float g = 9.80665; // ����� ����� 

void startMotorsOrStopMotors(Drone drone, bool d, float initialSpeed)
{
	drone.setMotor1(Motor(true, initialSpeed, -1));
	drone.setMotor2(Motor(true, initialSpeed, 1));
	drone.setMotor3(Motor(true, initialSpeed, -1));
	drone.setMotor4(Motor(true, initialSpeed, 1));
}

void IncreaseEngineSpeed(Drone drone)
{
    // ����� ������ ������� ������ �� ��������
    if (drone.getMotor1().speed < 100)
    {
        Motor motor1 = drone.getMotor1();
        motor1.speed = drone.getMotor1().speed + 5.0f; // ����� ������ �������
		drone.setMotor1(motor1);
		Motor motor2 = drone.getMotor2();
		motor2.speed = drone.getMotor2().speed + 5.0f; // ����� ������ �������
		drone.setMotor2(motor2);
		Motor motor3 = drone.getMotor3();
		motor3.speed = drone.getMotor3().speed + 5.0f; // ����� ������ �������
		drone.setMotor3(motor3);
		Motor motor4 = drone.getMotor4();
		motor4.speed = drone.getMotor4().speed + 5.0f; // ����� ������ �������
		drone.setMotor4(motor4);
    }
}

void calculateAirDensity(Drone drone)
{
    float T = T0 - L * drone.getDronePos().z;//����� ��������� ����� ���� �����
    float P = P0 * pow((1 - (L * drone.getDronePos().z) / T0), (g / (R * L))); // ����� ���� ����� ���� �����
    drone.setRho(P / (R * T)); // ������ ������ ����� ���� �����

}

void updateAltitude(Drone drone, float deltaTime)
{
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);//����� ��� ����
    float drag = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * pow(drone.getVelocity(), 2); // ����� ��� ����
    float weight = drone.getMass() * g;// ����� ���� �����

    float netForce = thrust - weight;// ����� ���� ����

    if (drone.getSpeedInAxes().vz > 0)
    {
        netForce -= drag; // ����� ��� ���� �� ����� ����
    }
    else
    {
        netForce += drag; // ����� ��� ���� �� ����� ����
    }
	Acceleration Acceleration = drone.getAccelerationInAxes();
    Acceleration.az = netForce / drone.getMass();
    drone.setAccelerationInAxes(Acceleration);// ����� �����
	Velocity SpeedInAxes = drone.getSpeedInAxes();
	SpeedInAxes.vz += drone.getAccelerationInAxes().az * deltaTime; // ����� ������ �����
    drone.setSpeedInAxes(SpeedInAxes);
	Point dronePos = drone.getDronePos();
	dronePos.z += drone.getVelocity() * deltaTime; // ����� ����� ����� ���� z
	drone.setDronePos(dronePos); // ����� ���� �����
    drone.setVelocity(sqrt(SpeedInAxes.vx * SpeedInAxes.vx + SpeedInAxes.vy * SpeedInAxes.vy + SpeedInAxes.vz * SpeedInAxes.vz));
    drone.setAcceleration(sqrt(Acceleration.ax * Acceleration.ax + Acceleration.ay * Acceleration.ay + Acceleration.az * Acceleration.az));
}

// ������� ������ ���� �� �� ��� ����, ���� ������� ������
void takeof(Drone drone)
{
    startMotorsOrStopMotors(drone, true, 0);
    drone.setRpm(sqrt(drone.getMass() * g / drone.getC_t()));
    // �������� �� �����
    while (drone.getDronePos().z < drone.getTargetAltitude())
    {
        IncreaseEngineSpeed(drone); // ����� ������ �������
        calculateAirDensity(drone); // ����� ������ ������
        updateAltitude(drone, 0.1);  // ����� �� 0.1 �����
        drone.setRpm(drone.getRpm() + 10);// ��� ������ ���� ����, ������� �� RPM ���
    }
}


void decreaseMotorSpeed(Drone drone)
{
    // ����� ������ ������� ������
	Motor motor1 = drone.getMotor1();
    motor1.speed = max(motor1.speed - 5.0f, 0.0f); // ����� ������� �-5 �� ���
    drone.setMotor1(motor1);
    Motor motor2 = drone.getMotor2();
	motor2.speed = max(motor2.speed - 5.0f, 0.0f); // ����� ������� �-5 �� ���
	drone.setMotor2(motor2);
	Motor motor3 = drone.getMotor3();
    motor3.speed = max(motor3.speed - 5.0f, 0.0f);
    drone.setMotor3(motor3);
	Motor motor4 = drone.getMotor4();
    motor4.speed = max(motor4.speed - 5.0f, 0.0f);
    drone.setMotor4(motor4);
}


void land(Drone drone)
{
    // �������� �� �����
    while (drone.getDronePos().z > 0.1)
    {
        drone.setRpm(drone.getRpm() - 10);// ��� ������ ���� ����, ������� �� RPM ���
        decreaseMotorSpeed(drone); // ����� ������ �������
        calculateAirDensity(drone); // ����� ������ ������
        updateAltitude(drone, 1); // ����� ���� �� 1 ����
    }
    startMotorsOrStopMotors(drone, false, 0);
    drone.setVelocity(0);
    drone.setRpm(0);
    Point point = drone.getDronePos();
    point.z = 0;
    drone.setDronePos(point);
}


float hover(Drone drone)
{
    startMotorsOrStopMotors(drone, true, 0);

    drone.setRpm(sqrt(drone.getMass() * g / drone.getC_t()));
    float hoverAltitude = drone.getDronePos().z;

    while (abs(drone.getDronePos().z - hoverAltitude) > 0.1 || abs(drone.getSpeedInAxes().vz) > 0.1)
    {
        calculateAirDensity(drone);      // ����� ������ ������
        updateAltitude(drone, 0.1);        // ����� ������� �� ����� ��������

        // ������� RPM ����� ������ �����
        if (drone.getDronePos().z < hoverAltitude)
            drone.setRpm(drone.getRpm() + 5);
        else if (drone.getDronePos().z > hoverAltitude)
            drone.setRpm(drone.getRpm() - 5);

        // Replace the problematic line with explicit usage of std::max and std::min  
        drone.setRpm(max(0.0f, min(drone.getRpm(), drone.getMaxRPM())));
    }
    return drone.getRpm();
}

void setSpeed(Drone drone, Velocity newVelocity) {
    drone.setSpeedInAxes(newVelocity);
    drone.setVelocity(sqrt(newVelocity.vx * newVelocity.vx +
        newVelocity.vy * newVelocity.vy +
        newVelocity.vz * newVelocity.vz));
}


void adjustMotorsToRotate(Drone drone, float currentYaw, float targetYaw) {
    float yawError = targetYaw - currentYaw;

    float balancingSpeed = hover(drone);

    Motor motor1 = drone.getMotor1();
    motor1.speed = balancingSpeed; // ����� ������� �-5 �� ���
    drone.setMotor1(motor1);
    Motor motor2 = drone.getMotor2();
    motor2.speed = balancingSpeed; // ����� ������� �-5 �� ���
    drone.setMotor2(motor2);
    Motor motor3 = drone.getMotor3();
    motor3.speed = balancingSpeed;
    drone.setMotor3(motor3);
    Motor motor4 = drone.getMotor4();
    motor4.speed = balancingSpeed;
    drone.setMotor4(motor4);
    // ����� ����� ������� ������ ��� ������
    float adjustment = yawError * 2.0f; // ���� ����, ���� �����

    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // ����� �����
    if (adjustment > drone.getMaxRPM() / 2) adjustment = drone.getMaxRPM() / 2;
    if (adjustment < -drone.getMaxRPM() / 2) adjustment = -drone.getMaxRPM() / 2;

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
        if (motors[i]->speed > drone.getMaxRPM()) motors[i]->speed = drone.getMaxRPM();
        if (motors[i]->speed < 0) motors[i]->speed = 0;
    }
}


void updateOrientation(Drone drone, float dt)
{
    float clockwiseThrust = drone.getMotor1().speed + drone.getMotor2().speed;
    float counterClockwiseThrust = drone.getMotor3().speed + drone.getMotor4().speed;

    // ��� �� ������ ������ ��� �����
    drone.setYawRate(counterClockwiseThrust - clockwiseThrust * drone.getYawRate());

    // ���� �� ������ (����� ��������� ����� �� 0.01 �����)
    drone.setYaw(drone.getYaw() + drone.getYawRate() * dt);

    // ��� ������ �� ������ ����� [0, 360)
    if (drone.getYaw() >= 360.0f) drone.setYaw(drone.getYaw() - 360.0f);
    if (drone.getYaw() < 0.0f) drone.setYaw(drone.getYaw() + 360.0f);
}


void wait(float seconds) {
    Sleep(static_cast<DWORD>(seconds * 1000)); // ���� ����������
}


void rotate(Drone drone, float deltaYawDegrees, float dt)
{
    // ����� ����� ����
    float targetYaw = drone.getYaw() + deltaYawDegrees;

    // ����� ������ �� ����� ������ ����
    while (abs(drone.getYaw() - targetYaw) > 0.5f)
    {
        // ���� ��� ����� (�����/�����)
        adjustMotorsToRotate(drone, deltaYawDegrees, drone.getYaw());

        // ����� ��� �����
        updateOrientation(drone, dt);

        // ������ ���
        wait(0.01f);
    }
}

void updateForwardMotion(Drone drone, float deltaTime)
{
    //  ����� thrust ����
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);

    //  ������
    float pitch = drone.getPitch(); // ��������
    float yaw = drone.getYaw();     // ��������

    //  ��� thrust ����� ��� pitch
    float forwardThrust = thrust * sin(pitch);

    //  ����� ��X ��Y ��� yaw (����� �������)
    float fx = forwardThrust * cos(yaw); // ���� thrust ���� X
    float fy = forwardThrust * sin(yaw); // ���� thrust ���� Y

    //  ����� ��� ���� X ��Y
    Velocity speed = drone.getSpeedInAxes();
    float dragX = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vx * speed.vx;
    float dragY = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vy * speed.vy;

    // ����� ���� ���� ������ ������
    dragX *= (speed.vx > 0) ? -1 : 1;
    dragY *= (speed.vy > 0) ? -1 : 1;

    //  ��� ���
    float netFx = fx + dragX;
    float netFy = fy + dragY;

    //  ����� �����
    Acceleration acceleration = drone.getAccelerationInAxes();
    acceleration.ax = netFx / drone.getMass();
    acceleration.ay = netFy / drone.getMass();
    drone.setAccelerationInAxes(acceleration);

    //  ����� ������
    speed.vx += acceleration.ax * deltaTime;
    speed.vy += acceleration.ay * deltaTime;
    drone.setSpeedInAxes(speed);

    //  ����� �����
    Point pos = drone.getDronePos();
    pos.x += speed.vx * deltaTime;
    pos.y += speed.vy * deltaTime;
    drone.setDronePos(pos);

    //  ����� ������
    float vTotal = sqrt(speed.vx * speed.vx + speed.vy * speed.vy + speed.vz * speed.vz);
    float aTotal = sqrt(acceleration.ax * acceleration.ax + acceleration.ay * acceleration.ay + acceleration.az * acceleration.az);
    drone.setVelocity(vTotal);
    drone.setAcceleration(aTotal);
}

void moveForward(Drone drone, float durationSeconds, float pitchAngleDegrees, float dt)
{
    // ����� �� ����� �������
    float targetAltitude = drone.getDronePos().z;

    // ����� ����� �����
    drone.setPitch(pitchAngleDegrees);

    float elapsedTime = 0.0f;

    while (elapsedTime < durationSeconds)
    {
        // ����� ������ ����� �� �� ����
        calculateAirDensity(drone);

		updateForwardMotion(drone, dt); 

        wait(dt);
        elapsedTime += dt;
    }

    // ����� pitch �-0 ��� ����� �� ��������
    drone.setPitch(0);
}
