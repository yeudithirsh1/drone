#pragma once  
#include "pointInSpace.h" // כולל את הקובץ pointSpace.h  
#include <vector> // Include the vector header for std::vector  
#include <Eigen/Dense> // Include Eigen for Vector3  

using std::vector; // Use the std namespace for vector  
using Eigen::Vector3f; // Use Eigen's Vector3f for 3D vector representation  

struct Motor {  
  bool isActive; // האם המנוע פעיל  
  float speed; // מהירות המנוע (ב- RPM)  
  int direction; // כיוון המנוע (1 או -1)  
  Motor(bool isActive, int speed, int direction) : isActive(isActive), speed(speed), direction(direction) {} // בנאי  
};  

struct droneDimension {  
  float length;  
  float width;  
  float height;  
  droneDimension(float l, float w, float h) : length(l), width(w), height(h) {} // בנאי  
};  

struct Velocity {  
  float vx;  
  float vy;  
  float vz;  
  Velocity(float x, float y, float z) : vx(x), vy(y), vz(z) {} // בנאי  
};  

struct Acceleration {  
  float ax;  
  float ay;  
  float az;  
  Acceleration(float x, float y, float z) : ax(x), ay(y), az(z) {} // בנאי  
};  

class Drone {  
private:  
  const int maxRPM = 120000;  
  const float target_altitude = 10.0f;  
  const float max_altitude = 120.0f;  
  const float mass = 1.5f;  
  const float A = 0.05f;  
  const float C_d = 1.0f;  
  const float C_t = 0.1f;  
  float rpm;  
  float rho;  
  float velocity;  
  float acceleration;  
  float yawRate = 0; // מהירות סיבוב סביב ציר האנכי, ביחידות rad/s  
  float yaw;  
  Motor motor1; // קדמי שמאל  
  Motor motor2; // קדמי ימין  
  Motor motor3; // אחורי שמאל  
  Motor motor4; // אחורי ימין  
  droneDimension droneDim;  
  Point dronePos;  
  Velocity SpeedInAxes;  
  Acceleration AccelerationInAxes;  
public:  
  Drone(); // בנאי ברירת מחדל  
  int getMaxRPM();  
  float getTargetAltitude();  
  float getMaxAltitude();  
  float getMass();  
  float getA();  
  float getC_d();  
  float getC_t();  
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
  void computeLinearAcceleration(const vector<Vector3f>& velocities, const vector<double>& times);  
};