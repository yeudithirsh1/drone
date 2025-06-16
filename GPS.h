#include <mutex>
#include "pointInSpace.h" 
#include "KalmanFilter.h"

class GPS
{  
private:  
Point GPSLocation;  
public:

	GPS() : GPSLocation(0, 0, 0) {} 
	Point getGPSLocation() const { return GPSLocation; }
	void setGPSLocation(float x, float y, float z) { GPSLocation = Point(x, y, z); }
	void updateGPSReadingsFromFile(KalmanFilter& kalmanfilter);
};

