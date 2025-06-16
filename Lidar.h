#pragma once
#include <vector>
#include <utility> // בשביל std::pair
#include <shared_mutex>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "PointInSpace.h" // כולל את הקובץ PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // כולל את הקובץ DroneFeatures.h
using namespace std;


class LIDAR
{
  private:
	 vector<Point> currentScan;      // סריקה נוכחית
	 vector<Point> previousScan;     // סריקה קודמת
	 VectorXf lidarLocation;            // תזוזה אחרונה שחושבה - מיקום חדש
	 ICP_OUT icpTransformation; // אובייקט ICP לחישוב תזוזות
	 float distance;
	 shared_mutex currentScanMutex;
  public:
	   // Constructor
	   LIDAR();
	   // Getters
	   vector<Point> getCurrentScan();
	   vector<Point> getPreviousScan();
	   VectorXf getLidarLocation();
	   ICP_OUT getIcpTransformation();
	   void updateLidarReadingsFromFile(Drone& drone, KalmanFilter& kalmanFilter);
	   void mergePointClouds(Drone& drone);
	   void locationFromLidarMeasurement(Drone& drone, Matrix4f& icpTransformation);
	   void isObstacleInFront(Drone& drone, droneDimension droneDim);
};

