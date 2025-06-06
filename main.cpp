#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include "Graph.h"
#include "DroneFeatures.h"
#include "GPS.h"
#include "IMU.h"
#include "LIDAR.h"
#include "KalmanFilter.h"
#include "Global.h"
#include "DroneCommands.h"
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
    drone.setDronePos(point);//מיקום הרחפן
    Sensors sensors;
    GPS sensorGPS;
    IMU sensorIMU; 
    LIDAR sensorLidar;
    KalmanFilter kalmanfilter;
    kalmanfilter.init(point.x, point.y, point.z);
    const string filePath = "fbb";
    thread gpsThread(&GPS::updateGPSReadingsFromFile, &sensorGPS, ref(kalmanfilter));
    thread imuThread(&IMU::updateIMUReadingsFromFile, &sensorIMU, ref(kalmanfilter));
    thread lidarThread(&LIDAR::updateLidarReadingsFromFile, &sensorLidar, ref(drone), ref(kalmanfilter));
    thread predictThread(&KalmanFilter::predictLoop, &kalmanfilter);
    thread state(&KalmanFilter::getState, &kalmanfilter, ref(drone));
    takeoff(drone);
   return 0;
}



//Vector3f point1 = { firstPoint.x, firstPoint.y, firstPoint.z };
//Vector3f point2 = { secondPoint->x, secondPoint->y, secondPoint->z };
//Vector3f direction = (point1 - point2).normalized();
//float yaw_current = drone.getYaw();
//float yaw_target = atan2(direction.y(), direction.x());
//float delta_yaw = normalizeAngle(yaw_target - yaw_current);
//float pitch_current = drone.getPitch();
//float pitch_target = atan2(direction.z(), sqrt(direction.x() * direction.x() + direction.y() * direction.y()));
//float delta_pitch = normalizeAngle(pitch_target - pitch_current);