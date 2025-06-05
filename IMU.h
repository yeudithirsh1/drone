#pragma once  
#include "Sensors.h" // Include the header file for the Sensors class  
#include "DroneFeatures.h" 
#include <mutex>
#include "KalmanFilter.h"
using namespace std;  

class IMU : public Sensors  
{  
private:  
  Acceleration acceleration;  
  float yawRate; 
  float pitchRate;
public:  
  IMU(); // הכרזה על הבנאי במחלקה  
  void updateIMUReadingsFromFile(mutex& mutexReachedDestination, bool reachedDestination, KalmanFilter& kalmanfilter);
};
