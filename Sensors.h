#pragma once  
#include <chrono>  

using time = std::chrono::time_point<std::chrono::high_resolution_clock>;  

class Sensors  
{  
private:  
  float distanceBetweenMeasurements;  
  time timeMeasurements;  
public:  
  Sensors();  
  Sensors(float distanceBetweenMeasurements, time timeMeasurements);  
  void setDistanceBetweenMeasurements(float distanceBetweenMeasurements);  
  float getDistanceBetweenMeasurements();  
  void setTimeMeasurements(time timeMeasurements); 
  time getTimeMeasurements();
};
