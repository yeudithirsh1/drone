#pragma once
#include "PointInSpace.h"
#include <vector>
#include "DroneFeatures.h"
#include "LIDAR.h"
using namespace std;

bool findHighestPointAbove(LIDAR& sensorLidar, const Point dronePos, const droneDimension droneDim, Point& leftmostPointOut);
void processTakingOff(Drone& drone, LIDAR& sensorLidar);

