#pragma once  
#include "pointInSpace.h" 
#include "Sensors.h"

class GPS : public Sensors
{  
private:  
Point GPSLocation;  
public:

	GPS() : GPSLocation(0, 0, 0) {} 
	Point getGPSLocation() const { return GPSLocation; }
	void setGPSLocation(float x, float y, float z) { GPSLocation = Point(x, y, z); }
	void UpdatePossion();
	double haversine(double lat1, double lon1, double lat2, double lon2);
};
