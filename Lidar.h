#pragma once
#include <vector>
#include <utility> // בשביל std::pair
#include "PointInSpace.h" // כולל את הקובץ PointInSpace.h
#include "ICP.h"
#include "DroneFeatures.h" // כולל את הקובץ DroneFeatures.h
#include "Sensors.h" // כולל את הקובץ Sensors.h
using namespace std;


class Lidar : Sensors
{
  private:
	 vector<Point> currentScan;      // סריקה נוכחית
	 vector<Point> previousScan;     // סריקה קודמת
	 Point lidarLocation;            // תזוזה אחרונה שחושבה
	 ICP_OUT icpTransformation; // אובייקט ICP לחישוב תזוזות
	 bool isMovementNoisy;            // האם התזוזה רועשת
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

