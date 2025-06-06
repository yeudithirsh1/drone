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

IMU::IMU()
    : Sensors(0.0f, chrono::high_resolution_clock::now()), // קריאה לבנאי של Sensors
    acceleration(0.0f, 0.0f, 0.01f){}

void IMU::updateIMUReadingsFromFile(KalmanFilter& kalmanfilter)
{
    ifstream file("src/IMU.txt"); // Open the text file for reading  
    if (!file.is_open()) {
        cerr << "שגיאה: לא ניתן לפתוח את הקובץ src/IMU.txt" << endl;
        return;
    }

    string line;
    file.seekg(0, ios::end); // להתחיל מהסוף

    while (true)
    {
        {
           shared_lock<shared_mutex> lock(mutexReachedDestination);
           if (reachedDestination) {
              break;
           }
        }
        getline(file, line);
        istringstream iss(line);
        iss >> acceleration.ax >> acceleration.ay >> acceleration.az >> yawRate >> pitchRate; // Ensure valid input extraction 
        Vector3f accelerationVector;
        accelerationVector << acceleration.ax, acceleration.ay, acceleration.az;
        float yaw_rate, pitch_rate;
        yaw_rate = yawRate;
        pitch_rate = pitchRate;
        kalmanfilter.updateIMU(accelerationVector, yaw_rate, pitch_rate);
        this_thread::sleep_for(chrono::milliseconds(50)); // 20 פעמים בשניה
    }
    file.close(); // Close the file  
}
