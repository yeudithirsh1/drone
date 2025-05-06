#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
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
const float ANGLE_THRESHOLD = 15.0f; // סף זווית לבדיקה

// === פונקציות ===
bool isDirectionFree(const PointCloud<PointXYZ>::Ptr& cloud,
    const KdTreeFLANN<PointXYZ>& kdtree,
    const Vector3f& pos,
    const Vector3f& dir)
{
    PointXYZ searchPoint(pos.x(), pos.y(), pos.z());
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    // חיפוש נקודות ברדיוס SAFE_DISTANCE
    if (kdtree.radiusSearch(searchPoint, SAFE_DISTANCE, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        // עוברים על כל הנקודות שמצאנו
        for (int idx : pointIdxRadiusSearch) {
            const auto& pt = cloud->points[idx];
            Vector3f obstacle(pt.x, pt.y, pt.z);
            Vector3f rel = obstacle - pos;

            // מחשבים את הזווית בין הכיוון שלנו לנקודת המכשול
            float angle = acos(rel.normalized().dot(dir.normalized())) * 180.0f / M_PI;
            if (angle < ANGLE_THRESHOLD) {
                return false; // יש מכשול שמפריע במסלול
            }
        }
    }

    return true; // אין מכשול קרוב שמפריע
}

Vector3f findBestDirection(const PointCloud<PointXYZ>::Ptr& cloud,
    const KdTreeFLANN<PointXYZ>& kdtree,
    const Vector3f& pos,
    const Vector3f& originalDir)
{
    Vector3f bestDir = originalDir;
    float minAngle = 180.0f;

    for (float angle = -AVOIDANCE_ANGLE_RANGE; angle <= AVOIDANCE_ANGLE_RANGE; angle += SCAN_ANGLE_RESOLUTION) {
        float radians = angle * M_PI / 180.0f;
        Matrix3f rot = AngleAxisf(radians, Vector3f::UnitZ()).toRotationMatrix();
        Vector3f newDir = rot * originalDir;

        if (isDirectionFree(cloud, kdtree, pos, newDir)) {
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
    const KdTreeFLANN<PointXYZ>& kdtree,
    const Vector3f& currentPos,
    const Vector3f& goal)
{
    Vector3f desiredDir = goal - currentPos;
    if (desiredDir.norm() < GOAL_THRESHOLD) {
        return currentPos; // כבר קרובים מאוד ליעד
    }

    if (isDirectionFree(cloud, kdtree, currentPos, desiredDir)) {
        return currentPos + desiredDir.normalized() * STEP_DISTANCE;
    }
    else {
        Vector3f avoidanceDir = findBestDirection(cloud, kdtree, currentPos, desiredDir);
        return currentPos + avoidanceDir.normalized() * STEP_DISTANCE;
    }
}

