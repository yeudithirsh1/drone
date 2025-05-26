//#pragma once
//#include "pointInSpace.h" // ���� �� ����� pointSpace.h
//
//struct Motor {
//	bool isActive; // ��� ����� ����
//	int speed; // ������ ����� (�- RPM)
//	int direction; // ����� ����� (1 �� -1)
//	Motor(bool isActive, int speed, int direction) : isActive(isActive), speed(speed), direction(direction) {} // ����
//};
//
//struct droneDimension
//{
//	float length;
//	float width;
//	float height;
//	droneDimension(float l, float w, float h) : length(l), width(w), height(h) {} // ����
//};
//
//struct Velocity {
//	float vx;
//	float vy;
//	float vz;
//	Velocity(float x, float y, float z) : vx(x), vy(y), vz(z) {} // ����
//};
//
//struct Acceleration
//{
//	float wx;
//	float wy;
//	float wz;
//	Acceleration(float x, float y, float z) : wx(x), wy(y), wz(z) {} // ����
//};
//
//struct DroneDirection {
//	float yaw; // ����� �-Yaw �� �����
//	float pitch; // ����� �-Pitch �� �����
//	float roll; // ����� �-Roll �� �����
//	DroneDirection(float y, float p, float r) : yaw(y), pitch(p), roll(r) {} // ����
//};
//
//struct MovementDirection
//{
//	float heading_deg;    // �����: ����� ������ ���� (0-360)
//	float elevation_deg;  // ����: ����� ���� ����/��� (-90 �� 90)
//};
//
//
//class Drone
//{
//private:
//	const float maxRPM = 5000.0f;
//	const float target_altitude;
//	const float max_altitude;
//	const float mass;
//	const float A;
//	const float C_d;
//	const float C_t;
//	float rpm;
//	float rho;
//	float velocity;
//	float acceleration;
//	float yawRate; // ������ ����� ���� ��� �����, ������� rad/s
//
//	Motor motor1;//���� ����
//	Motor motor2;//���� ����
//	Motor motor3;//����� ����
//	Motor motor4;//����� ����
//	droneDimension droneDim;
//	Point dronePos;
//	Velocity SpeedInAxes;
//	MovementDirection movementDir;
//	Acceleration AccelerationInAxes;
//public:
//	void startMotorsOrStopMotors(bool d, float initialSpeed);
//	void decreaseMotorSpeed();
//	void calculateAirDensity();
//	void updateAltitude(float deltaTime);
//	void takeof();
//	void land();
//	void IncreaseEngineSpeed();
//	void hover();
//	void setSpeed(Velocity newVelocity);
//	void updateOrientation()
//
//};
//
