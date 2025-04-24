#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace Eigen;
using namespace std;
using namespace pcl;

// === קבועים ===
const float SAFE_DISTANCE = 2.0f;
const float SCAN_ANGLE_RESOLUTION = 10.0f;
const float AVOIDANCE_ANGLE_RANGE = 90.0f;
const float STEP_DISTANCE = 0.5f;
const float GOAL_THRESHOLD = 1.0f; // מטר

// === פונקציות ===
bool isDirectionFree(const PointCloud<PointXYZ>::Ptr& cloud,
    const Vector3f& pos,
    const Vector3f& dir) {
    for (const auto& pt : cloud->points) {
        Vector3f obstacle(pt.x, pt.y, pt.z);
        Vector3f rel = obstacle - pos;
        float distance = rel.norm();
        if (distance < SAFE_DISTANCE) {
            float angle = acos(rel.normalized().dot(dir.normalized())) * 180.0f / M_PI;
            if (angle < 15.0f) {
                return false;
            }
        }
    }
    return true;
}

Vector3f findBestDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Vector3f& pos,
    const Vector3f& originalDir) {
    Vector3f bestDir = originalDir;
    float minAngle = 180.0f;

    for (float angle = -AVOIDANCE_ANGLE_RANGE; angle <= AVOIDANCE_ANGLE_RANGE; angle += SCAN_ANGLE_RESOLUTION) {
        float radians = angle * M_PI / 180.0f;
        Matrix3f rot = AngleAxisf(radians, Vector3f::UnitZ()).toRotationMatrix();
        Vector3f newDir = rot * originalDir;

        if (isDirectionFree(cloud, pos, newDir)) {
            float deviation = acos(newDir.normalized().dot(originalDir.normalized())) * 180.0f / M_PI;
            if (deviation < minAngle) {
                minAngle = deviation;
                bestDir = newDir;
            }
        }
    }

    return bestDir;
}

Vector3f navigateToGoal(const PointCloud<PointXYZ>::Ptr& cloud,
    const Vector3f& currentPos,
    const Vector3f& goal) {
    Vector3f desiredDir = goal - currentPos;
    if (desiredDir.norm() < GOAL_THRESHOLD) {
        return currentPos; // כבר קרובים מאוד ליעד
    }

    if (isDirectionFree(cloud, currentPos, desiredDir)) {
        return currentPos + desiredDir.normalized() * STEP_DISTANCE;
    }
    else {
        Vector3f avoidanceDir = findBestDirection(cloud, currentPos, desiredDir);
        return currentPos + avoidanceDir.normalized() * STEP_DISTANCE;
    }
}

// === פונקציה ראשית ===
int main() {
    Vector3f startPos(0, 0, 0);
    Vector3f targetPos(20, 0, 0); // לדוגמה, 20 מטר בכיוון X
    Vector3f currentPos = startPos;

    while ((targetPos - currentPos).norm() > GOAL_THRESHOLD) {
        PointCloud<PointXYZ>::Ptr cloud = getLatestPointCloud(); // יש לממש פונקציה זו
        Vector3f newPos = navigateToGoal(cloud, currentPos, targetPos);

        moveDroneTo(newPos); // יש לממש פונקציה זו
        currentPos = newPos;

        cout << "Current position: " << currentPos.transpose() << endl;
    }

    cout << "Arrived at destination!" << endl;
    return 0;
}
