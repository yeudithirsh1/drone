#pragma once
#include <vector>
#include <utility> // ����� std::pair
#include <mutex>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "PointInSpace.h" // ���� �� ����� PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // ���� �� ����� DroneFeatures.h
#include "Sensors.h" // ���� �� ����� Sensors.h
using namespace std;


class LIDAR : Sensors
{
  private:
	 vector<Point> currentScan;      // ����� ������
	 vector<Point> previousScan;     // ����� �����
	 VectorXf lidarLocation;            // ����� ������ ������ - ����� ���
	 ICP_OUT icpTransformation; // ������� ICP ������ ������
	 float distance;
	 mutex currentScanMutex;
  public:
	   // Constructor
	   LIDAR();
	   // Getters
	   vector<Point> getCurrentScan();
	   vector<Point> getPreviousScan();
	   VectorXf getLidarLocation();
	   ICP_OUT getIcpTransformation();
	   void updateLidarReadingsFromFile(Drone& drone, mutex& mutexReachedDestination, bool reachedDestination,  KalmanFilter& kalmanFilter);
	   void mergePointClouds(Drone& drone, vector<Point>& clouds);
	   void locationFromLidarMeasurement(Drone& drone, Matrix4f& icpTransformation);
};

