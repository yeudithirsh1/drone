#pragma once
#include <vector>
#include <utility>
#include <shared_mutex>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "PointInSpace.h" 
#include "ICP.h"
#include "DroneFeatures.h" 
#include <queue> 

using namespace std;


class LIDAR
{
 private:
	 vector<Point> currentScan;      // סריקה נוכחית
	 vector<Point> previousScan;     // סריקה קודמת
	 vector<Point> filteredCloud;
	 queue<vector<Point>> q;
	 shared_mutex qMutex;
	 VectorXf lidarLocation;            // תזוזה אחרונה שחושבה - מיקום חדש
	 ICP_OUT icpTransformation; // אובייקט ICP לחישוב תזוזות
	 float distance;
	 shared_mutex currentScanMutex;
	 shared_mutex previousScanMutex;
	 shared_mutex filteredCloudMutex;
  public:
	   // Constructor
	   LIDAR();
	   // Getters
	   vector<Point>& getCurrentScan();
	   void setCurrentScan(vector<Point>& newCurrentScan);
	   vector<Point> getPreviousScan();
	   void setPreviousScan(vector<Point>& newPreviousScan);
	   vector<Point> getFilteredClouds();
	   void setFilteredClouds(vector<Point>& newFilteredCloud);
	   vector<Point> getQ();
	   VectorXf getLidarLocation();
	   ICP_OUT getIcpTransformation();
	   void updateLidarReadingsFromFile(Drone& drone, KalmanFilter& kalmanFilter, const string& filePath);
	   void mergePointClouds(Drone& drone);
	   void locationFromLidarMeasurement(Drone& drone, Matrix4f& icpTransformation);
	   void isObstacleInFront(Drone& drone, droneDimension droneDim);
};

