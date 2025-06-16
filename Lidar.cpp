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
#include <shared_mutex>
#include "Global.h"
#include "Object.h"


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
    
    while (true)
    { 
        {
            shared_lock<shared_mutex> lock(mutexReachedDestination);
            if (reachedDestination) {
                return;
            }
        }
        ifstream file("src/LidarPoints.txt");
        if (!file.is_open()) {
            cerr << "שגיאה: לא ניתן לפתוח את הקובץ src/LidarPoints.txt" << std::endl;
            return;
        }

        string line;
        
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
        mergePointClouds(drone);
        kalmanFilter.updateLidar(lidarLocation);
        isObstacleInFront(drone, drone.getDroneDim()); 
        file.close();   
        this_thread::sleep_for(chrono::milliseconds(100)); // עדכון  פעמים בשנייה
    }
}

void LIDAR::mergePointClouds(Drone& drone) {

    vector<Point> currentClude;
    {
        shared_lock<shared_mutex> lock(currentScanMutex);
        currentClude = currentScan;
    }
    // המרה למטריצות ישירות מתוך currentScan
    MatrixXf currentMat(currentClude.size(), 3);
    for (size_t i = 0; i < currentClude.size(); ++i) {
        currentMat(i, 0) = currentClude[i].x;
        currentMat(i, 1) = currentClude[i].y;
        currentMat(i, 2) = currentClude[i].z;
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

void LIDAR::isObstacleInFront(Drone& drone, droneDimension droneDim) {

    vector<Point> currentClude;
    {
        shared_lock<shared_mutex> lock(currentScanMutex);
        currentClude = currentScan;
    }

    vector<Point> filteredCloud;
    DivisionIntoClusters(filteredCloud, currentClude);
    if(filteredCloud.size())
    {
        unique_lock<shared_mutex> lock(currentScanMutex);
        currentScan = filteredCloud;
    }


    Point dronePos = drone.getDronePos();
    Vector3f dronePosVector(dronePos.x, dronePos.y, dronePos.z);

    float yaw = drone.getYaw(); // כיוון התנועה של הרחפן - זווית ביונים
    float yawRad = yaw * M_PI / 180.0f;

    // כיוון קדימה לפי ה-yaw
    Vector3f forward(cos(yawRad), sin(yawRad), 0);
    Vector3f up(0, 0, 1);
    Vector3f right = forward.cross(up).normalized();

    for (const auto& p : currentClude) {
        Vector3f pVector(p.x, p.y, p.z);
        Vector3f delta = pVector - dronePosVector;

        float x = delta.dot(forward); // מרחק קדימה
        float y = delta.dot(right);   // מרחק צדדי
        float z = delta.dot(up);      // מרחק לגובה

        bool inside = (x >= 0 && x <= drone.getDroneDim().height) &&
            (fabs(y) <= droneDim.width) &&
            (fabs(z) <= droneDim.height);

        if (inside)
        {
            {
                unique_lock<shared_mutex> lock(mutexObstacleStatus);
                obstacleStatuse = true;
            }
        }
    }
}
