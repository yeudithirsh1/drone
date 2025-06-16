#include "DroneFeatures.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <shared_mutex>

using namespace std;
Drone::Drone() {}
float Drone::getMaxRPM()
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
	shared_lock<shared_mutex> lock(velosityMutex);
    return velocity;
}
void Drone::setVelocity(float velocity)
{
	unique_lock<shared_mutex> lock(velosityMutex);
	this->velocity = velocity;
}
float Drone::getAcceleration()
{
	shared_lock<shared_mutex> lock(accelerationMutex);
	return acceleration;
}
void Drone::setAcceleration(float acceleration)
{
	unique_lock<shared_mutex> lock(accelerationMutex);
	this->acceleration = acceleration;
}
float Drone::getYawRate()
{
	shared_lock<shared_mutex> lock(yawRateMutex);
	return yawRate;
}
void Drone::setYawRate(float yawRate)
{
	unique_lock<shared_mutex> lock(yawRateMutex);
	this->yawRate = yawRate;
}
float Drone::getPitchRate()
{
	shared_lock<shared_mutex> lock(pitchRateMutex);
	return pitchRate;
}
void Drone::setPitchRate(float pitchRate)
{
	unique_lock<shared_mutex> lock(pitchRateMutex);
	this->pitchRate = pitchRate;
}
float Drone::getYaw()
{
	shared_lock<shared_mutex> lock(yawMutex);
	return yaw;
}
void Drone::setYaw(float yaw)
{
	unique_lock<shared_mutex> lock(yawMutex);
	this->yaw = yaw;
}
float Drone::getPitch()
{
	shared_lock<shared_mutex> lock(pitchMutex);
	return pitch;
}
void Drone::setPitch(float pitch)
{
	unique_lock<shared_mutex> lock(pitchMutex);
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
	shared_lock<shared_mutex> lock(dronePosMutex);
	return dronePos;
}
void Drone::setDronePos(Point dronePos)
{
	unique_lock<shared_mutex> lock(dronePosMutex);
	this->dronePos = dronePos;
}
Velocity Drone::getSpeedInAxes()
{
	shared_lock<shared_mutex> lock(speedInAxesMutex);
	return speedInAxes;
}
void Drone::setSpeedInAxes(Velocity SpeedInAxes)
{
	unique_lock<shared_mutex> lock(speedInAxesMutex);
	this->speedInAxes = SpeedInAxes;
}
Acceleration Drone::getAccelerationInAxes()
{
	shared_lock<shared_mutex> lock(accelerationInAxesMutex);
	return accelerationInAxes;
}
void Drone::setAccelerationInAxes(Acceleration accelerationInAxes)
{
	unique_lock<shared_mutex> lock(accelerationInAxesMutex);
	this->accelerationInAxes = accelerationInAxes;
}
