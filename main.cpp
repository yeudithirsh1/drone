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
#include "Object.h"
#include "Soldiers.h"
#include "YOLO.h"
using namespace std;


void mainNavigation(vector<vector<Vertex>> graph, Drone& drone, KalmanFilter& kalmanFilter, LIDAR& lidar) 
{
    Vertex* pointer = &graph[0][0];
    while (pointer)
    {
        Vector3f startingPoint = { pointer->x, pointer->y, pointer->z };
        Vector3f endPoint = { pointer->next->x, pointer->next->y, pointer->next->z };

        rotate(drone, startingPoint, endPoint, kalmanFilter);

        Point pointDeparture = { pointer->x, pointer->y, pointer->z };
        Point pointDestination = { pointer->next->x, pointer->next->y, pointer->next->z };

        moveForward(drone, pointDestination, 0.01f, kalmanFilter);
       
        if (getObstacleStatuse())
        {
            // שמירת מיקום ההתחלה לעקיפה
            Point originalPos = drone.getDronePos();  
            bool d = true;

            // התחלת לולאת העקיפה
            while (d)
            {
                // מציאת זווית פנויה
                float Angle = findMostRightFreeGap(lidar, drone, drone.getDroneDim());

                // סיבוב לזווית פנויה
                rotateYaw(drone, Angle, kalmanFilter);

                float new_x = drone.getDronePos().x + drone.getDroneDim().length * cos(drone.getYaw());
                float new_y = drone.getDronePos().x + drone.getDroneDim().length * sin(drone.getYaw());
                Point nextPoint = { new_x, new_y, drone.getDronePos().z };
                // תנועה קצרה קדימה בזווית החדשה
                moveForward(drone, nextPoint, 0.01f, kalmanFilter);


                // בדיקה האם חזרנו למסלול המקורי
                Point currentPos = drone.getDronePos();
                Point pointOnLine, returnPoint;
                bool isOnLine = TheTwoPointsOnLine(pointDeparture, pointDestination, pointOnLine, returnPoint);

                if (isOnLine){ 

                    vector<Point> returnRoute = analyzeDroneTurnsAndHull(pointDeparture, pointDestination, pointOnLine, returnPoint);
                    vector<Point> extendedForm = inflatePolygon(returnRoute, drone.getDroneDim().length + 0.01f);
                    extendedForm.pop_back();
                    
                    // ניווט לפי המסלול המורחב שהורכב
                    for (size_t i = 0; i < extendedForm.size() - 1; ++i)
                    {
                        Point p1 = extendedForm[i];
                        Point p2 = extendedForm[i + 1];

                        Vector3f start = { p1.x, p1.y, p1.z };
                        Vector3f end = { p2.x, p2.y, p2.z };

                        // סיבוב לעבר היעד הבא במסלול
                        rotate(drone, start, end, kalmanFilter);

                        // תנועה לנקודה הבאה
                        moveForward(drone, p2, 0.01f, kalmanFilter);
                    }

                    // יציאה מהעקיפה
                    d = false;
                }
            }
        }
        pointer = pointer->next;
    }
}

int main() 
{   
// Point(5, 0, 5, nullptr), Point(10,5,5, nullptr) , Point (5,10, 5, nullptr), Point(0,5, 5, nullptr ) };
    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }

    //בינתיים
    cout << "Enter drone flight height: ";
    cin >> targetAltitude;
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
        Landmarks.emplace_back(x, y, targetAltitude, nullptr);
    }
    //טווח הראייה של הרחפן
    float fieldView = 2;
    //עד כאן
    

    //מפה לסריקה
    vector<vector<Vertex>> graph = graphNavigationPath(Landmarks, fieldView);//הפונקצייה מקבלת נקודות ציון ושדה ראייה של הרחפן
   Point point = { graph[0][0].x, graph[0][0].y, 0 };
   string path = "C:/Users/This User/Document/project/Soldiers";
   string pathFileGps = "C:/Users/This User/Documents/project/Sensor/GPS.txt";
   string pathFileImu = "C:/Users/This User/Documents/project/Sensor/IMU.txt";
   string pathFileLidar = "C:/Users/This User/Documents/project/Sensor/LIDAR.txt";

   Drone drone;    
   drone.setDronePos(point);//מיקום הרחפן
   GPS sensorGPS;
   IMU sensorIMU; 
   LIDAR sensorLidar;
   KalmanFilter kalmanFilter;
   kalmanFilter.init(point.x, point.y, point.z);
   thread gpsThread(&GPS::updateGPSReadingsFromFile, &sensorGPS, ref(kalmanFilter), pathFileGps);
   thread imuThread(&IMU::updateIMUReadingsFromFile, &sensorIMU, ref(kalmanFilter), pathFileImu);
   thread lidarThread(&LIDAR::updateLidarReadingsFromFile, &sensorLidar, ref(drone), ref(kalmanFilter), pathFileLidar);
   thread predictThread(&KalmanFilter::predictLoop, &kalmanFilter, ref(drone));
   startTrackingAllSoldiers(path);
   processDirectoryAndCopyFilesOnly();
   bool runPythonScript();
   takeoff(drone, kalmanFilter);
   mainNavigation(graph, drone, kalmanFilter, sensorLidar);
   land(drone, kalmanFilter);
    return 0;
}

