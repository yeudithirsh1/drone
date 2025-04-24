#pragma once

struct Motor {
	bool isActive; // ��� ����� ����
	int speed; // ������ ����� (�- RPM)
	int direction; // ����� ����� (1 �� -1)
	Motor() : isActive(false), speed(0), direction(0) {} // ���� ����� ����
};

class drone_commands
{
private:
	Motor motor;
public:
	// ����
	drone_commands() : motor() {};
	// ��������
	void startMotorsOrStopMotors(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4, bool d, float initialSpeed);
	void increaseMotorSpeed(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4);
	float calculateAirDensity(float altitude_m);
	void updateAltitude(float& altitude, float rpm, float deltaTime, float mass, float& velocity, float rho, float A, float C_d, float C_t);
	void takeof(float mass, float targetAltitude, float A, float C_d, float C_t);
	void land(float mass, float targetAltitude, float A, float C_d, float C_t);
};

