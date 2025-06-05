#include "PointInSpace.h"
#include <vector>
#include "DroneFeatures.h"
#include <fstream>
#include <iostream>
#include "DroneCommands.h"
#include <limits>
#include "LIDAR.h"

using namespace std;

// פונקציה שבודקת אם יש נקודות מעל הרחפן בתיבה קבועה, ושומרת את הנקודה הכי שמאלית
bool findHighestPointAbove(LIDAR& sensorLidar, const Point dronePos, const droneDimension droneDim, Point& highestPointOut)
{
    bool found = false;
    float maxZ = -(numeric_limits<float>::max)(); // התחלה ממינימום אפשרי

    for (auto& p : sensorLidar.getCurrentScan()) {
        float dx = p.x - dronePos.x;
        float dy = p.y - dronePos.y;
        float dz = p.z - dronePos.z;

        if (fabs(dx) <= droneDim.length / 2 &&
            fabs(dy) <= droneDim.width / 2 &&
            dz >= 0 && dz <= droneDim.height) {

            if (p.z > maxZ) {
                maxZ = p.z;
                highestPointOut = p;
            }
        }
    }
    return found;
}


void processTakingOff(Drone& drone, LIDAR& sensorLidar)
{
    bool flag;
    Point highestPointOut;
    flag = findHighestPointAbove(ref(sensorLidar), drone.getDronePos(), drone.getDroneDim(), highestPointOut);
    if (!flag) {
        moveForward(drone, highestPointOut, 0.1, 0.1);
    }
    else {
        takeoff(drone, 0.1);
    }
}


