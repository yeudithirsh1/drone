#include "GPS.h"  
#include <fstream>  
#include <sstream>  
#include <iostream>  
#include <thread>  
#include <chrono>
#include <Eigen/Dense>
#include "KalmanFilter.h"
#include "Global.h"
#include <shared_mutex>

using namespace std;
using namespace Eigen;

void GPS::updateGPSReadingsFromFile(KalmanFilter& kalmanfilter)
{  
   ifstream inputFile("src/GPS.txt");
   if (!inputFile.is_open()) {
       cerr << "שגיאה: לא ניתן לפתוח את הקובץ src/GPS.txt" << endl;
       return;
   }
   inputFile.seekg(0, ios::end); // להתחיל מהסוף
   string line;  
   while (true)
   {
       {
           shared_lock<shared_mutex> lock(mutexReachedDestination);
           if (reachedDestination) {
               break;
           }
       }
       getline(inputFile, line);
       istringstream iss(line);  
       iss >> GPSLocation.x >> GPSLocation.y >> GPSLocation.z;
       Vector3f GPSLocationVector;
       GPSLocationVector << GPSLocation.x, GPSLocation.y, GPSLocation.z;
       kalmanfilter.updateGPS(GPSLocationVector);
       this_thread::sleep_for(chrono::seconds(200));// 5 פעמים בשניה
   } 
   inputFile.close();
}
