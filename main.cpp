#include <iostream>
#include <vector>
#include <thread>
#include "Graph.h"
#include "DroneFeatures.h"
#include "processTakingOff.h"
#include "GPS.h"
#include "IMU.h"
using namespace std;

int main() 
{   
// Point(5, 0, 5, nullptr), Point(10,5,5, nullptr) , Point (5,10, 5, nullptr), Point(0,5, 5, nullptr ) };
    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }

    //�������
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
        // ������� �� ������ �� ����� �������
        Landmarks.emplace_back(x, y, height, nullptr);
    }
    //���� ������ �� �����
    float fieldView = 2;
    //�� ���
    

    //��� ������
    vector<vector<Vertex>> graph = graphNavigationPath(Landmarks, fieldView);//��������� ����� ������ ���� ���� ����� �� �����
    Point point = { graph[0][0].x, graph[0][0].y, 0 };
    Drone drone;
    GPS sensorGPS;
    IMU sensorIMU;

    drone.setDronePos(point);//����� �����
    string filePath = "fbb";
    thread gpsThread(&GPS::UpdatePossion, &sensorGPS);

    processTakingOff(drone, filePath);    //����� �����


   return 0;
}