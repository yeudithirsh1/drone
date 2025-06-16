#pragma once  
#include "DroneFeatures.h" 
#include "KalmanFilter.h"
using namespace std;  

class IMU  
{  
private:  
  Acceleration acceleration;  
  float yawRate; 
  float pitchRate;
public:  
  IMU(); // ����� �� ����� ������  
  void updateIMUReadingsFromFile(KalmanFilter& kalmanfilter);
};
