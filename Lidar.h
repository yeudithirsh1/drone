#pragma once
#include <vector>
#include <utility> // ����� std::pair
#include <shared_mutex>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "PointInSpace.h" // ���� �� ����� PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // ���� �� ����� DroneFeatures.h
using namespace std;


class LIDAR
{
  private:
	 vector<Point> currentScan;      // ����� ������
	 vector<Point> previousScan;     // ����� �����
	 VectorXf lidarLocation;            // ����� ������ ������ - ����� ���
	 ICP_OUT icpTransformation; // ������� ICP ������ ������
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

