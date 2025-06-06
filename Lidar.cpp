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
#include "KalmanFilter.h"
#include "Global.h"
#include <shared_mutex>



using namespace std;
using namespace Eigen;

LIDAR::LIDAR() {}

vector<Point> LIDAR::getCurrentScan()
{
    {
        shared_lock<shared_mutex> lock(currentScanMutex);
        return currentScan;
    }
}

vector<Point> LIDAR::getPreviousScan()
{
    return previousScan;
}

VectorXf LIDAR::getLidarLocation()
{
	return lidarLocation;
}

ICP_OUT LIDAR::getIcpTransformation()
{
    return icpTransformation;
}

void LIDAR::updateLidarReadingsFromFile(Drone& drone, KalmanFilter& kalmanFilter)
{
    ifstream file("src/LidarPoints.txt");
    if (!file.is_open()) {
        cerr << "שגיאה: לא ניתן לפתוח את הקובץ src/LidarPoints.txt" << std::endl;
        return;
    }

    string line;

    while (true)
    {
        {
            shared_lock<shared_mutex> lock(mutexReachedDestination);
            if (reachedDestination) {
                break;
            }
        }
        vector<Point> cloud;
        while (getline(file, line))
        {
            istringstream iss(line);
            Point point;
            iss >> point.x >> point.y >> point.z;
            cloud.push_back(point);
        }
        {
            shared_lock<shared_mutex> lock(currentScanMutex);
            previousScan = currentScan;
        }
        {
            unique_lock<shared_mutex> lock(currentScanMutex);
            currentScan = cloud;
        }
        // עיבוד הסריקה הנוכחית
        this_thread::sleep_for(chrono::milliseconds(100)); // עדכון  פעמים בשנייה
        mergePointClouds(drone, cloud);
        kalmanFilter.updateLidar(lidarLocation);
    }

    file.close();
}


void LIDAR::mergePointClouds(Drone& drone, vector<Point>& clouds) {

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
    for (size_t i = 0; i < previousScan.size(); i++) {
        previousMat(i, 0) = previousScan[i].x;
        previousMat(i, 1) = previousScan[i].y;
        previousMat(i, 2) = previousScan[i].z;
    }
    icpTransformation = icp(currentMat, previousMat);
	locationFromLidarMeasurement(drone, icpTransformation.trans);
}

void LIDAR::locationFromLidarMeasurement(Drone& drone, Matrix4f& icpTransformation)
{
	Vector4f pos = {
		drone.getDronePos().x,
		drone.getDronePos().y,
		drone.getDronePos().z,
		1.0f
	};
	Vector4f transformedPos = icpTransformation * pos;
	lidarLocation  << transformedPos[0], transformedPos[1], transformedPos[2];
}
