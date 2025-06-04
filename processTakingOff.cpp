#include "PointInSpace.h"
#include <vector>
#include "DroneFeatures.h"
#include <fstream>
#include <iostream>
#include "DroneCommands.h"
#include <limits>

using namespace std;

vector<Point> loadPointCloudFromFile(const string& filePath) {
    vector<Point> cloud;

    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filePath << endl;
        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            Point p;
            if (iss >> p.x >> p.y >> p.z) {
                cloud.push_back(p);
            }
            else {
                cerr << "Invalid line in file: " << line << endl;
            }
        }
        file.close(); // סוגרים את הקובץ לקריאה

        // שלב מחיקת תוכן הקובץ
        ofstream clearFile(filePath, ios::trunc);
        if (!clearFile.is_open()) {
            cerr << "Error clearing file: " << filePath << endl;
        }
        clearFile.close();
        return cloud;
    }
}
// פונקציה שבודקת אם יש נקודות מעל הרחפן בתיבה קבועה, ושומרת את הנקודה הכי שמאלית
bool findLeftmostPointAbove(const vector<Point>& cloud, const Point dronePos, const droneDimension droneDim, Point& leftmostPointOut)
{
    bool found = false;
    float minX = (numeric_limits<float>::max)();
    for (const auto& p : cloud) {
        float dx = p.x - dronePos.x;
        float dy = p.y - dronePos.y;
        float dz = p.z - dronePos.z;

        if (fabs(dx) <= droneDim.length / 2 &&
            fabs(dy) <= droneDim.width / 2 &&
            dz >= 0 && dz <= droneDim.height) {

            if (p.x < minX) {
                minX = p.x;
                leftmostPointOut = p;
                found = true;
            }
        }
    }
    return found;
}

void processTakingOff(Drone& drone, const string& filePath)
{
    bool flag= false;
    while (!flag) {
        vector<Point> cloud = loadPointCloudFromFile(filePath);
        Point point;
        flag = findLeftmostPointAbove(cloud, drone.getDronePos(), drone.getDroneDim(), point);
        if (!flag) {
            moveForward(drone, point, 0.1, 0.1);
        }
        else {
            takeoff(drone, 0.1);
        }
    }
}


