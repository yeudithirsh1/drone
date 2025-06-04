#pragma once
#include "DroneFeatures.h"

void startMotorsForTakeoff(Drone& drone, float hoverSpeed);
void AdjustEngineSpeed(Drone& drone);
void calculateAirDensity(Drone& drone);
void UpdateFollowingProgress(Drone& drone, float dt);
void wait(float seconds);
void takeoff(Drone& drone, float dt);
void startForwardMotion(Drone& drone, bool d, float pitch, float engineSpeed);
float computePitchAngle(Drone& drone, float drag_force);
float computeThrust(Drone& drone, float motorRPM);
void updateAltitude(Drone& drone, float dt);
void moveForward(Drone& drone, Point targetPosition, float stopThreshold, float dt);
