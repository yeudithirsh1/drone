#pragma once
#include <vector>
#include <utility> // ����� std::pair
#include "PointInSpace.h" // ���� �� ����� PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // ���� �� ����� DroneFeatures.h
#include "Sensors.h" // ���� �� ����� Sensors.h
using namespace std;


class Lidar : Sensors
{
  private:
	 vector<Point> currentScan;      // ����� ������
	 vector<Point> previousScan;     // ����� �����
	 Point lidarLocation;            // ����� ������ ������
	 ICP_OUT icpTransformation; // ������� ICP ������ ������
	 bool isMovementNoisy;            // ��� ������ �����
	 float distance;

  public:
	    // Constructor
	    Lidar();
	
	    // Getters
	    vector<Point> getCurrentScan();
	    vector<Point> getPreviousScan();
	    Point getLidarLocation();
	    ICP_OUT getIcpTransformation();
       // Simple placeholder for ICP-like calculation
	   void loadPointCloudFromFile(Drone drone, const string& filePath);
  	   void mergePointClouds(Drone drone, vector<Point>& clouds);
	   void locationFromLidarMeasurement(Drone drone, Matrix4f& icpTransformation);
};

