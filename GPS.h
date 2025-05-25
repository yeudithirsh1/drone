#pragma once  
#include "pointInSpace.h"  

class GPS  
{  
private:  
Point GPSLocation;  
public:
	GPS(float x, float y, float z) : GPSLocation(x, y, z) {}
	Point getGPSLocation() const { return GPSLocation; }
	void setGPSLocation(float x, float y, float z) { GPSLocation = Point(x, y, z); }
	void UpdatePossion();

};
