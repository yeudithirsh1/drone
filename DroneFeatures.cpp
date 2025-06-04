#include "DroneFeatures.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;
Drone::Drone() {}
int Drone::getMaxRPM()
{
	return maxRPM;
}
float Drone::getTargetAltitude()
{
	return target_altitude;
}
float Drone::getMaxAltitude()
{
	return max_altitude;
}
float Drone::getMass()
{
	return mass;
}
float Drone::getA()
{
	return A;
}
float Drone::getC_d()
{
	return C_d;
}
float Drone::getC_t()
{
	return C_t;
}
float Drone::getHoverSpeed()
{
	return hoverSpeed;
}
float Drone::getRpm()
{
	return rpm;
}
void Drone::setRpm(float rpm)
{
	this->rpm = rpm;
}
float Drone::getRho()
{
	return rho;
}
void Drone::setRho(float rho)
{
	this->rho = rho;
}
float Drone::getVelocity()
{
	return velocity;
}
void Drone::setVelocity(float velocity)
{
	this->velocity = velocity;
}
float Drone::getAcceleration()
{
	return acceleration;
}
void Drone::setAcceleration(float acceleration)
{
	this->acceleration = acceleration;
}
float Drone::getYawRate()
{
	return yawRate;
}
void Drone::setYawRate(float yawRate)
{
	this->yawRate = yawRate;
}
float Drone::getPitchRate()
{
	return pitchRate;
}
void Drone::setPitchRate(float pitchRate)
{
	this->pitchRate = pitchRate;
}
float Drone::getYaw()
{
	return yaw;
}
void Drone::setYaw(float yaw)
{
	this->yaw = yaw;
}
float Drone::getPitch()
{
	return pitch;
}
void Drone::setPitch(float pitch)
{
	this->pitch = pitch;
}
Motor Drone::getMotor1()
{
	return motor1;
}
void Drone::setMotor1(Motor motor1)
{
	this->motor1 = motor1;
}
Motor Drone::getMotor2()
{
	return motor2;
}
void Drone::setMotor2(Motor motor2)
{
	this->motor2 = motor2;
}
Motor Drone::getMotor3()
{
	return motor3;
}
void Drone::setMotor3(Motor motor3)
{
	this->motor3 = motor3;
}
Motor Drone::getMotor4()
{
	return motor4;
}
void Drone::setMotor4(Motor motor4)
{
	this->motor4 = motor4;
}
droneDimension Drone::getDroneDim()
{
	return droneDim;
}
void Drone::setDroneDim(droneDimension droneDim)
{
	this->droneDim = droneDim;
}
Point Drone::getDronePos()
{
	return dronePos;
}
void Drone::setDronePos(Point dronePos)
{
	this->dronePos = dronePos;
}
Velocity Drone::getSpeedInAxes()
{
	return SpeedInAxes;
}
void Drone::setSpeedInAxes(Velocity SpeedInAxes)
{
	this->SpeedInAxes = SpeedInAxes;
}
void Drone::setAccelerationInAxes(Acceleration AccelerationInAxes)
{
	this->AccelerationInAxes = AccelerationInAxes;
}
Acceleration Drone::getAccelerationInAxes()
{
	return AccelerationInAxes;
}


























//struct Vector3 {
//	float x, y, z;
//};
//
//void Drone::computeLinearAcceleration(const vector<Eigen::Vector3f>& velocities, const vector<float>& times) {
//	vector<Eigen::Vector3f> accelerations;
//
//	for (size_t i = 1; i < velocities.size(); ++i) {
//		float dt = times[i] - times[i - 1];
//		if (dt == 0) {
//			// Prevent division by zero
//			accelerations.push_back(Eigen::Vector3f(0, 0, 0));
//			continue;
//		}
//		Eigen::Vector3f a;
//		a.x() = (velocities[i].x() - velocities[i - 1].x()) / dt;
//		a.y() = (velocities[i].y() - velocities[i - 1].y()) / dt;
//		a.z() = (velocities[i].z() - velocities[i - 1].z()) / dt;
//
//		accelerations.push_back(a);
//	}
//
//	for (size_t i = 0; i < accelerations.size(); ++i) {
//		AccelerationInAxes.ax = accelerations[i].x();
//		AccelerationInAxes.ay = accelerations[i].y();
//		AccelerationInAxes.az = accelerations[i].z();
//	}
//}
