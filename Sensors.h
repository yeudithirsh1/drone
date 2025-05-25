#pragma once  
#include <chrono>  

using namespace std;
using time_point = chrono::time_point<chrono::high_resolution_clock>;  

class Sensors  
{  
private:  
  float distanceBetweenMeasurements;  
  time_point timeMeasurements;  
public:  
  Sensors(float distanceBetweenMeasurements, time_point timeMeasurements);  
  void setDistanceBetweenMeasurements(float distanceBetweenMeasurements);  
  float getDistanceBetweenMeasurements();  
  void setTimeMeasurements(time_point timeMeasurements); 
  time_point getTimeMeasurements();
};
