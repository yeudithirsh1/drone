#include <iostream>
#include <vector>
#include "nevigation.h"
#include "PointInSpace.h"
using namespace std;


//int main() 
//{   //{ Point(5, 0, 5, nullptr), Point(10,5,5, nullptr) , Point (5,10, 5, nullptr), Point(0,5, 5, nullptr ) };
//    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }
//    float height;
//    cout << "Enter drone flight height: ";
//    cin >> height;
//    int numPoints;
//    cout << "Enter number of (x, y) points: ";
//    cin >> numPoints;
//    vector<Vertex> points;
//    for (int i = 0; i < numPoints; ++i)
//    {
//        double x, y;
//        cout << "Enter x and y for point " << i + 1 << ": ";
//        cin >> x >> y;
//        // מוסיפים את הנקודה עם הגובה שקיבלנו
//        points.emplace_back(x, y, height, nullptr);
//    }
//    //טווח הראייה של הרחפן
//    int fieldView = 2;
//    //מפה לסריקה
//
//    vector<vector<Vertex>> edges = graphNavigationPath(points, fieldView);
//    //מיקום הרחפן
//    Vertex location = edges[0][0];
//    //גובה הרחפן
//    location.z = 0;
//    drone_commands Drone;
//    float mass = 1.5; // Example mass in kg
//    float targetAltitude = height; // Example target altitude in meters
//    float A = 0.5; // Example area in m^2
//    float C_d = 1.2; // Example drag coefficient
//    float C_t = 0.8; // Example thrust coefficient
//    Drone.takeof(mass, targetAltitude, A, C_d, C_t);
//    return 0;
//}