#include "IMU.h"  
#include <string>  
#include <fstream> // Include this header for std::ifstream  
#include <sstream>  
#include <cmath>  
#include <iostream>  
#include <thread>  
#include <chrono>
#include "DroneFeatures.h"
#include "KalmanFilter.h"
#include "Global.h"
#include <shared_mutex>

using namespace std;

IMU::IMU() {};
void IMU::updateIMUReadingsFromFile(KalmanFilter& kalmanfilter, const string& filePath)
{
   
    while (!getReachedDestination())
    {
        ifstream file(filePath);
        if (!file.is_open()) {
            cerr << "שגיאה: לא ניתן לפתוח את הקובץ src/IMU.txt" << endl;
            return;
        }
        
        string line, lastLine;
        while (getline(file, line)) {
            if (!line.empty()) {
                lastLine = line;
            }
        }

        file.close();

        if (!lastLine.empty()) {
            istringstream iss(lastLine);

            iss >> acceleration.ax >> acceleration.ay >> acceleration.az >> yawRate >> pitchRate;
            Vector3f accelerationVector;
            accelerationVector << acceleration.ax, acceleration.ay, acceleration.az;
            float yaw_rate, pitch_rate;
            yaw_rate = yawRate;
            pitch_rate = pitchRate;
            kalmanfilter.updateIMU(accelerationVector, yaw_rate, pitch_rate);
        }
        this_thread::sleep_for(chrono::milliseconds(50)); // 20 פעמים בשניה
    }
}
