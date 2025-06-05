#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include "Graph.h"
#include "DroneFeatures.h"
#include "processTakingOff.h"
#include "GPS.h"
#include "IMU.h"
#include "LIDAR.h"
#include "KalmanFilter.h"
using namespace std;

int main() 
{   
// Point(5, 0, 5, nullptr), Point(10,5,5, nullptr) , Point (5,10, 5, nullptr), Point(0,5, 5, nullptr ) };
    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }

    //בינתיים
    float height;
    cout << "Enter drone flight height: ";
    cin >> height;
    int numPoints;
    cout << "Enter number of (x, y) points: ";
    cin >> numPoints;
    vector<Vertex> Landmarks;
    for (int i = 0; i < numPoints; ++i)
    {
        double x, y;
        cout << "Enter x and y for point " << i + 1 << ": ";
        cin >> x >> y;
        // מוסיפים את הנקודה עם הגובה שקיבלנו
        Landmarks.emplace_back(x, y, height, nullptr);
    }
    //טווח הראייה של הרחפן
    float fieldView = 2;
    //עד כאן
    

    //מפה לסריקה
    vector<vector<Vertex>> graph = graphNavigationPath(Landmarks, fieldView);//הפונקצייה מקבלת נקודות ציון ושדה ראייה של הרחפן
    Point point = { graph[0][0].x, graph[0][0].y, 0 };
    Drone drone;   
    Sensors sensors;
    GPS sensorGPS;
    IMU sensorIMU; 
    LIDAR sensorLidar;
    KalmanFilter kalmanfilter;
    kalmanfilter.init();
    mutex mutexReachedDestination;
    const string filePath = "fbb";
    bool reachedDestination = false;
    thread gpsThread(&GPS::updateGPSReadingsFromFile, &sensorGPS, ref(mutexReachedDestination), reachedDestination, ref(kalmanfilter));
    thread imuThread(&IMU::updateIMUReadingsFromFile, &sensorIMU, ref(mutexReachedDestination), reachedDestination, ref(kalmanfilter));
    thread lidarThread(&LIDAR::updateLidarReadingsFromFile, &sensorLidar, ref(drone), ref(mutexReachedDestination), reachedDestination, ref(kalmanfilter));
    drone.setDronePos(point);//מיקום הרחפן
    processTakingOff(drone, sensorLidar);//תהליך המראה
   return 0;
}