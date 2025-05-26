#include "Lidar.h"
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

Lidar::Lidar() {}

vector<Point> Lidar::getCurrentScan()
{
    return currentScan;
}

vector<Point> Lidar::getPreviousScan()
{
    return previousScan;
}

Point Lidar::getLidarLocation()
{
	return lidarLocation;
}

ICP_OUT Lidar::getIcpTransformation()
{
    return icpTransformation;
}


void Lidar::loadMultiplePointCloudsAsVectors(Drone drone, const vector<string>& filePaths) {
    vector<vector<Point>> clouds;

    for (const auto& path : filePaths) {
        ifstream file(path);
        if (!file.is_open()) {
            cerr << "Error opening file: " << path << endl;
            continue;
        }

        vector<Point> cloud;
        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            Point p;
            if (!(iss >> p.x >> p.y >> p.z)) {
                // שורה לא תקינה, מדלגים
                continue;
            }
            cloud.push_back(p);
        }

        clouds.push_back(cloud);
    }
    mergePointClouds(drone, clouds);
}

void Lidar::mergePointClouds(Drone drone, vector<vector<Point>>& clouds) {
    vector<Point> merged;

    // חישוב גודל כולל
    size_t totalSize = 0;
    for (const auto& cloud : clouds) {
        totalSize += cloud.size();
    }
    merged.reserve(totalSize);

    // מיזוג
    for (const auto& cloud : clouds) {
		merged.insert(merged.end(), cloud.begin(), cloud.end());
    }

    // עדכון currentScan
    currentScan = merged;

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

void Lidar::locationFromLidarMeasurement(Drone drone, Matrix4f& icpTransformation)
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
