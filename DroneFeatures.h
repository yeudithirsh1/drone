#pragma once  
#include "pointInSpace.h" 
#include <vector> 
#include <Eigen/Dense>  
#include <shared_mutex>
#include "Global.h"

using namespace std;

extern const float g; // רק הצהרה

struct Motor {
	bool isActive;     // האם המנוע פעיל
	float speed;       // מהירות המנוע (ב-RPM)
	int direction;     // כיוון המנוע (1 או -1)

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
  const float C_t = 0.1f; //מקדם הדחף של המנועים - כלומר כמה דחף מייצר פרופלור עבור מהירות סיבוב מסוימת
  const float hoverSpeed = sqrt(mass * g / C_t);
  float rpm;  
  float rho;  //צפיפות האוויר 
  shared_mutex velosityMutex;
  float velocity;  
  shared_mutex accelerationMutex;
  float acceleration;   
  shared_mutex yawRateMutex;
  float yawRate = 0; // מהירות סיבוב סביב ציר האנכי, ביחידות rad/s  
  shared_mutex pitchRateMutex;
  float pitchRate = 0; // מהירות סיבוב סביב ציר האופקי, ביחידות rad/s
  shared_mutex yawMutex;
  float yaw;
  shared_mutex pitchMutex; 
  float pitch;
  Motor motor1; // קדמי שמאל  
  Motor motor2; // קדמי ימין  
  Motor motor3; // אחורי שמאל  
  Motor motor4; // אחורי ימין  
  droneDimension droneDim; 
  shared_mutex dronePosMutex;
  Point dronePos; 
  shared_mutex speedInAxesMutex;
  Velocity speedInAxes; 
  shared_mutex accelerationInAxesMutex;
  Acceleration accelerationInAxes;  
public:  
  Drone(); // בנאי ברירת מחדל  
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