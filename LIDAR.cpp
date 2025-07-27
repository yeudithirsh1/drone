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

vector<Point>& LIDAR::getCurrentScan()
{
   shared_lock<shared_mutex> lock(currentScanMutex);
   return currentScan;
}

void LIDAR::setCurrentScan(vector<Point>& newCurrentScan)
{
    unique_lock<shared_mutex> lock(currentScanMutex);
    currentScan = newCurrentScan;
}

vector<Point> LIDAR::getPreviousScan()
{
    shared_lock<shared_mutex> lock(previousScanMutex);
    return previousScan;
}

void LIDAR::setPreviousScan(vector<Point>& newPreviousScan)
{
    unique_lock<shared_mutex> lock(previousScanMutex);
    previousScan = newPreviousScan;

}

vector<Point> LIDAR::getFilteredClouds()
{
    shared_lock<shared_mutex> lock(filteredCloudMutex);
    return filteredCloud;
}

void LIDAR::setFilteredClouds(vector<Point>& newFilteredCloud)
{
    unique_lock<shared_mutex> lock(filteredCloudMutex);
    filteredCloud = newFilteredCloud;
}

VectorXf LIDAR::getLidarLocation()
{
	return lidarLocation;
}

ICP_OUT LIDAR::getIcpTransformation()
{
    return icpTransformation;
}

vector<Point> LIDAR::getQ()  
{  
    shared_lock<shared_mutex> lock(qMutex);  
    if (!q.empty()) {  
        vector<Point> front = q.front();  
        q.pop();  
        return front;  
    }  
    return {};
}


void LIDAR::updateLidarReadingsFromFile(Drone& drone, KalmanFilter& kalmanFilter, const string& filePath)
{
    
    while (!getReachedDestination())
    {
        ifstream inputFile(filePath);
        if (!inputFile.is_open()) {
            cerr << "Error: Cannot open file " << filePath << endl;
            return;
        }

        string line, lastLine;
        while (getline(inputFile, line)) {
            if (!line.empty()) {
                lastLine = line;
            }
        }

        inputFile.close();

        vector<Point> cloud;

        if (!lastLine.empty()) {
            istringstream iss(line);
            Point point;
            iss >> point.x >> point.y >> point.z;
            cloud.push_back(point);
        }
        
        setPreviousScan(getCurrentScan());

		setCurrentScan(cloud);
        
		q.push(cloud);

        mergePointClouds(drone);
        kalmanFilter.updateLidar(lidarLocation);
        isObstacleInFront(drone, drone.getDroneDim()); 
        this_thread::sleep_for(chrono::milliseconds(100)); // עדכון  פעמים בשנייה
    }
}

void LIDAR::mergePointClouds(Drone& drone) {

    vector<Point> currentClude;
    currentClude = getCurrentScan();
    
    // המרה למטריצות ישירות מתוך currentScan
    MatrixXf currentMat(currentClude.size(), 3);
    for (size_t i = 0; i < currentClude.size(); i++) {
        currentMat(i, 0) = currentClude[i].x;
        currentMat(i, 1) = currentClude[i].y;
        currentMat(i, 2) = currentClude[i].z;
    }

    // המרה גם ל־ previousScan
    MatrixXf previousMat(getPreviousScan().size(), 3);
    for (size_t i = 0; i < getPreviousScan().size(); i++) {
        previousMat(i, 0) = getPreviousScan()[i].x;
        previousMat(i, 1) = getPreviousScan()[i].y;
        previousMat(i, 2) = getPreviousScan()[i].z;
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

void LIDAR::isObstacleInFront(Drone& drone, droneDimension droneDim)
{
    vector<Point> filteredCloud;
    filteredCloud = DivisionIntoClusters(getFilteredClouds(), getCurrentScan());
    setFilteredClouds(filteredCloud);

    Point dronePos = drone.getDronePos();
    Vector3f dronePosVector(dronePos.x, dronePos.y, dronePos.z);

    float yaw = drone.getYaw(); // כיוון התנועה של הרחפן - זווית ביונים
    float yawRad = yaw * M_PI / 180.0f;

    // כיוון קדימה לפי ה-yaw
    Vector3f forward(cos(yawRad), sin(yawRad), 0);
    Vector3f up(0, 0, 1);
    Vector3f right = forward.cross(up).normalized();

    for (const auto& p : filteredCloud) {
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
