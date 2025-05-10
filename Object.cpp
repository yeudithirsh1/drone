#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include "nevigation.h"

using namespace Eigen;
using namespace std;
using namespace pcl;

// === קבועים ===
const float SAFE_DISTANCE = 2.0f;
const float ANGLE_THRESHOLD = 15.0f; // סף זווית לבדיקה

// === פונקציות ===
bool isDirectionFree(const PointCloud<PointXYZ>::Ptr& cloud,
    const Vector3f& pos,
    const Vector3f& dir,
    vector<Vector3f>& obstructingPoints) // הוספת משתנה שיכיל את כל נקודות המכשול
{
    KdTreeFLANN<PointXYZ> kdtree;
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
                obstructingPoints.push_back(obstacle); // מוסיפים את המכשול לרשימה
            }
        }
    }

    return obstructingPoints.empty(); // אם אין מכשול, מחזירים true
}
