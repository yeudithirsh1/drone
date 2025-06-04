#include "LIDAR.h"
#include <cmath>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "ICP.h"
#include "DroneFeatures.h"



using namespace std;
using namespace Eigen;

LIDAR::LIDAR() {}

vector<Point> LIDAR::getCurrentScan()
{
    return currentScan;
}

vector<Point> LIDAR::getPreviousScan()
{
    return previousScan;
}

Point LIDAR::getLidarLocation()
{
	return lidarLocation;
}

ICP_OUT LIDAR::getIcpTransformation()
{
    return icpTransformation;
}

void LIDAR::loadPointCloudFromFile(Drone drone, const string& filePath) {
    vector<Point> cloud;

    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filePath << endl;
    }

    string line;
    while (getline(file, line)) {
        istringstream iss(line);
        Point p;
        if (iss >> p.x >> p.y >> p.z) {
            cloud.push_back(p);
        }
        else {
            cerr << "Invalid line in file: " << line << endl;
        }
    }
    mergePointClouds(drone, cloud);
}


void LIDAR::mergePointClouds(Drone drone, vector<Point>& clouds) {

    // עדכון currentScan
    currentScan = clouds;

    // המרה למטריצות ישירות מתוך currentScan
    MatrixXf currentMat(currentScan.size(), 3);
    for (size_t i = 0; i < currentScan.size(); ++i) {
        currentMat(i, 0) = currentScan[i].x;
        currentMat(i, 1) = currentScan[i].y;
        currentMat(i, 2) = currentScan[i].z;
    }

    // המרה גם ל־ previousScan
    MatrixXf previousMat(previousScan.size(), 3);
    for (size_t i = 0; i < previousScan.size(); ++i) {
        previousMat(i, 0) = previousScan[i].x;
        previousMat(i, 1) = previousScan[i].y;
        previousMat(i, 2) = previousScan[i].z;
    }
    icpTransformation = icp(currentMat, previousMat);
	locationFromLidarMeasurement(drone, icpTransformation.trans);
}

void LIDAR::locationFromLidarMeasurement(Drone drone, Matrix4f& icpTransformation)
{
	Vector4f pos = {
		drone.getDronePos().x,
		drone.getDronePos().y,
		drone.getDronePos().z,
		1.0f
	};
	Vector4f transformedPos = icpTransformation * pos;
	lidarLocation = {
		transformedPos[0],
		transformedPos[1],
		transformedPos[2]
	};
}
