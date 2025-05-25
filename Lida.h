#pragma once
#include <vector>
#include <utility> // ����� std::pair
#include "PointInSpace.h" // ���� �� ����� PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // ���� �� ����� DroneFeatures.h
using namespace std;

class Lidar
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
		// Setters
       // Simple placeholder for ICP-like calculation
	   void loadMultiplePointCloudsAsVectors(Drone drone, const vector<string>& filePaths);
	   void mergePointClouds(Drone drone, vector<vector<Point>>& clouds);
	   void locationFromLidarMeasurement(Drone drone, Matrix4f& icpTransformation);
};

