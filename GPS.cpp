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

void GPS::updateGPSReadingsFromFile(KalmanFilter& kalmanfilter, const string& filePath)
{
    while (!getReachedDestination())
    {
        ifstream inputFile(filePath);
        if (!inputFile.is_open()) {
            cerr << "Error: Cannot open file " << filePath << endl;
            return;
        }

        string line, lastLine;
        while (getline(inputFile, line)) {
            if (!line.empty()) {
                lastLine = line;
            }
        }

        inputFile.close();

        if (!lastLine.empty()) {
            istringstream iss(lastLine);
            iss >> GPSLocation.x >> GPSLocation.y >> GPSLocation.z;

            Vector3f GPSLocationVector;
            GPSLocationVector << GPSLocation.x, GPSLocation.y, GPSLocation.z;

            kalmanfilter.updateGPS(GPSLocationVector);
        }

        this_thread::sleep_for(chrono::milliseconds(200)); // 5 פעמים בשניה
    }
}

