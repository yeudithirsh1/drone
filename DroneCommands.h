#pragma once
#include "DroneFeatures.h"
#include "KalmanFilter.h"
#include "controllerPID.h"

void startMotors(Drone& drone);
void updateThrustFromRPM(Drone& drone);
void calculateAirDensity(Drone& drone);
void UpdateFollowingProgress(Drone& drone, float dt, KalmanFilter& kalmanFilter);
void takeoff(Drone& drone, KalmanFilter& kalmanFilter);                      
void startForwardMotion(Drone& drone, bool d, float pitch, float engineSpeed);
float computePitchAngle(Drone& drone, float drag_force);
float computeThrust(Drone& drone, float motorRPM);
void updateAltitude(Drone& drone, float dt, KalmanFilter& kalmanFilter);
void moveForward(Drone& drone, Point targetPosition, float stopThreshold, KalmanFilter& kalmaFilter);
float deg2rad(float deg);
float rad2deg(float rad);
float normalizeAngle(float angle);
void adjustMotorsToRotate(Drone& drone, float currentYawRad, float targetYawRad, PID& pid, float dt);
void updateOrientation(Drone& drone, float dt, KalmanFilter& kalmanFilter);
void rotate(Drone& drone, Vector3f startPoint, Vector3f endPoint, KalmanFilter& kalmanFilter);
void rotateYaw(Drone& drone, float targetYawDeg, KalmanFilter& kalmanFilter);
void stopMoter(Drone& drone);
void land(Drone& drone, KalmanFilter& kalmanFilter);


