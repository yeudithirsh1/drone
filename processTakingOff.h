#pragma once
#include "PointInSpace.h"
#include <vector>
#include "DroneFeatures.h"
using namespace std;

vector<Point> loadPointCloudFromFile(const string& filePath);
bool findLeftmostPointAbove(const vector<Point>& cloud, const
	Point dronePos, const droneDimension droneDim, Point& leftmostPointOut);
void processTakingOff(Drone& drone, const string& filePath);

