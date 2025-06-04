#pragma once  
#include "Sensors.h" // Include the header file for the Sensors class  
#include "DroneFeatures.h" 
using namespace std;  

class IMU : public Sensors  
{  
private:  
  Acceleration acceleration;  
  float yawRate; 
  float pitchRate;
public:  
  IMU(); // הכרזה על הבנאי במחלקה  
  void calculateSpeed();  
};
