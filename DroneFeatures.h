#pragma once  
#include "pointInSpace.h" 
#include <vector> 
#include <Eigen/Dense>  
#include <shared_mutex>
#include "Global.h"

using namespace std;

extern const float g; // �� �����

struct Motor {
	bool isActive;     // ��� ����� ����
	float speed;       // ������ ����� (�-RPM)
	int direction;     // ����� ����� (1 �� -1)

	Motor() : isActive(false), speed(0.0f), direction(1) {}
	Motor(bool active, float spd, int dir) : isActive(active), speed(spd), direction(dir) {}

};

struct droneDimension {
	float length;
	float width;
	float height;

	droneDimension() : length(0.0f), width(0.0f), height(0.0f) {}
	droneDimension(float l, float w, float h) : length(l), width(w), height(h) {}
};

struct Velocity {
	float vx;
	float vy;
	float vz;

	Velocity() : vx(0.0f), vy(0.0f), vz(0.0f) {}
	Velocity(float x, float y, float z) : vx(x), vy(y), vz(z) {}
};

struct Acceleration {
	float ax;
	float ay;
	float az;

	Acceleration() : ax(0.0f), ay(0.0f), az(0.0f) {}
	Acceleration(float x, float y, float z) : ax(x), ay(y), az(z) {}
};

class Drone {  
private:  
  const float maxRPM = 120000.0f;  
  const float target_altitude = targetAltitude;
  const float mass = 1.5f;  
  const float A = 0.05f;  
  const float C_d = 1.0f;  
  const float C_t = 0.1f; //���� ���� �� ������� - ����� ��� ��� ����� ������� ���� ������ ����� ������
  const float hoverSpeed = sqrt(mass * g / C_t);
  float rpm;  
  float rho;  //������ ������ 
  shared_mutex velosityMutex;
  float velocity;  
  shared_mutex accelerationMutex;
  float acceleration;   
  shared_mutex yawRateMutex;
  float yawRate = 0; // ������ ����� ���� ��� �����, ������� rad/s  
  shared_mutex pitchRateMutex;
  float pitchRate = 0; // ������ ����� ���� ��� ������, ������� rad/s
  shared_mutex yawMutex;
  float yaw;
  shared_mutex pitchMutex; 
  float pitch;
  Motor motor1; // ���� ����  
  Motor motor2; // ���� ����  
  Motor motor3; // ����� ����  
  Motor motor4; // ����� ����  
  droneDimension droneDim; 
  shared_mutex dronePosMutex;
  Point dronePos; 
  shared_mutex speedInAxesMutex;
  Velocity speedInAxes; 
  shared_mutex accelerationInAxesMutex;
  Acceleration accelerationInAxes;  
public:  
  Drone(); // ���� ����� ����  
  float getMaxRPM();  
  float getTargetAltitude();  
  float getMass();  
  float getA();  
  float getC_d();  
  float getC_t(); 
  float getHoverSpeed();
  float getRpm();  
  void setRpm(float rpm);  
  float getRho();  
  void setRho(float rho); 
  float getVelocity();  
  void setVelocity(float velocity);  
  float getAcceleration();  
  void setAcceleration(float acceleration); 
  float getYawRate();  
  void setYawRate(float yawRate); 
  float getYaw(); 
  void setYaw(float yow); 
  float getPitch();
  void setPitch(float pitch);
  float getPitchRate();
  void setPitchRate(float pitchRate);
  Motor getMotor1();  
  void setMotor1(Motor motor1);  
  Motor getMotor2();  
  void setMotor2(Motor motor2);  
  Motor getMotor3();  
  void setMotor3(Motor motor3);  
  Motor getMotor4();  
  void setMotor4(Motor motor4);  
  droneDimension getDroneDim();  
  void setDroneDim(droneDimension droneDim);  
  Point getDronePos();  
  void setDronePos(Point dronePos);  
  Velocity getSpeedInAxes();  
  void setSpeedInAxes(Velocity SpeedInAxes);  
  Acceleration getAccelerationInAxes();  
  void setAccelerationInAxes(Acceleration AccelerationInAxes);  
}; 