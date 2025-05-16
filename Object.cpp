#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include "Reading_sensors.h"


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





#include "nevigation.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "ICP.h"
#include "Eigen/Eigen"
#include <iostream>

using namespace std;

struct Point3D {
    float x, y, z;
};

struct Point2D {
    float x, y;
};

const float Physical_height = 2.0f;
const float Physical_width = 4.5f;
const float angleResDeg = 5.3f;

vector<Point> listPoint = {};


//מסננת ענן נקודות ומוצאת זווית פנויה לטיסה
float findMostRightFreeGap(const vector<Point3D>& cloud,
    Point droneStart,
    Point droneTarget,
    Point dronePos,
    Point &pointOnLine,
    Point &returnPoint,
    float distance) {

    if(point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size()-1]) > 0 && point_side_of_line(droneStart, droneTarget, dronePos) == 0
    || point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size() - 1]) > 0 && point_side_of_line(droneStart, droneTarget, dronePos) < 0)
       pointOnLine = Intersection_points(droneStart, droneTarget, dronePos);
    else
        if(point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size() - 1]) < 0 && point_side_of_line(droneStart, droneTarget, dronePos) == 0
        || point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size() - 1]) < 0 && point_side_of_line(droneStart, droneTarget, dronePos) > 0)
           returnPoint = Intersection_points(droneStart, droneTarget, dronePos);

    vector<Point2D> projected;
    for (const auto& p : cloud) {
        float dz1 = dronePos.z + Physical_height / 2;
        float dz2 = dronePos.z - Physical_height / 2;
        float dx = p.x - dronePos.x;
        float dy = p.y - dronePos.y;
        float dist = sqrt(dx * dx + dy * dy);
        if (p.z <= dz1 && p.z >= dz2 && dist < distance) {
            projected.push_back({ p.x, p.y });
        }
    }

    Vector3f dronPoseVector = { dronePos.x, dronePos.y, dronePos.z };
    bool foundInside = false;
    float angle;
    for (angle = -180; angle <= 180; angle += angleResDeg) {
        for (const auto& p : cloud) {
            Vector3f pVector = { p.x, p.y, p.z };
            float angle_rad = angle * M_PI / 180.0f;

            Vector3f forward(cos(angle_rad), sin(angle_rad), 0);
            Vector3f up(0, 0, 1);
            Vector3f right = forward.cross(up).normalized();

            Vector3f center = dronPoseVector;
            Vector3f delta = pVector - center;

            float y = delta.dot(right);
            float z = delta.dot(up);

            bool inside = fabs(y) <= Physical_width && fabs(z) <= Physical_height;

            if (inside) {
                foundInside = true; 
                break; // אין צורך לבדוק עוד, מצאנו
            }
        }
        if (!foundInside) {
           if(angle != 0)
              listPoint.push_back(dronePos);
           return angle;
        }   
    }
    return angle;
}


//בדיקה האם נקודה נמצאת בימין המכשול או בשמאל
int point_side_of_line(Point A, Point B, Point P)
{
    int cross = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
    return cross;
}

//פונקציה לחישוב שיפוע בין 2 נקודות, משוואת ישר, ונקודת חיתוך בין 2 ישרים
Point Intersection_points(Point startPos, Point goalPos, Point dronePos)
{
    Point currentPoint1 = dronePos;
    Point currentPoint2 = listPoint[listPoint.size() - 1];
	float m1 = (goalPos.y - startPos.y) / (goalPos.x - startPos.x);
    float b1 = startPos.y - m1 * startPos.x;
	float x1 = (startPos.y - b1) / m1;
	float m2 = (currentPoint2.y - currentPoint1.y) / (currentPoint2.x - currentPoint1.x);
	float b2 = currentPoint1.y - m2 * currentPoint1.x;
	float x2 = (currentPoint1.y - b2) / m2;
	float intersectionPointx = (b2 - b1) / (m1 - m2);
    float intersectionPointy = m1 * intersectionPointx + b1;
    Point intersectionPoint = { intersectionPointx, intersectionPointy };
    return intersectionPoint;
}

//פונקציה לחיבור ענני נקודות
vector<Point> mergePointClouds(const vector<vector<Point>>& clouds) {
    vector<Point> merged;

    // מחשבים כמה נקודות בסך הכל כדי להזמין מקום מראש
    size_t totalSize = 0;
    for (const auto& cloud : clouds) {
        totalSize += cloud.size();
    }
    merged.reserve(totalSize);

    // מוסיפים כל ענן בנפרד
    for (const auto& cloud : clouds) {
        merged.insert(merged.end(), cloud.begin(), cloud.end());
    }

    return merged;
}


//פונקציה לסכימת אורך המסלול מימין ומשמאל
float computePolylineLength(const vector<Point2D>& points) {
    float length = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
}

double crossProduct(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת המעטפה הקמורה עם נקודות בכיוון השעון
void convexHullClockwise(vector<Point2D> turnPoints,vector<Point2D>& hull)
{
    if (turnPoints.size() <= 1)
    {
        hull = turnPoints;
        return;
    }
    // מיון הנקודות לפי x ואז y
    sort(turnPoints.begin(), turnPoints.end(), [](const Point2D& a, const Point2D& b)
        {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });

    for (const auto& p : turnPoints)
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), p) >= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }

    // שמירת גודל החלק התחתון
    int lowerSize = hull.size();

    // בניית החלק העליון
    for (auto it = turnPoints.rbegin(); it != turnPoints.rend(); ++it)
    {
        while (hull.size() > lowerSize && crossProduct(hull[hull.size() - 2], hull.back(), *it) >= 0)
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }

    // הסרת נקודת ההתחלה הכפולה
    hull.pop_back();
}

//פונקציה שבודקת איזה צד ארוך יותר ימין או שמאל
void analyzeDroneTurnsAndHull(const Point3D& startPos,
    const Point3D& goalPos,
    float deltaZ,
    float radius,
    float minGapWidth,
    const Point2D& crossingPoint,
    const vector<Point2D>& turnPoints) {

    vector<Point2D> hull;
    convexHullClockwise(turnPoints, hull);

    vector<Point2D> rightOfLine;
    vector<Point2D> leftOfLine;

    for (const auto& p : hull) {
        float dx1 = goalPos.x - startPos.x;
        float dy1 = goalPos.y - startPos.y;
        float dx2 = p.x - crossingPoint.x;
        float dy2 = p.y - crossingPoint.y;

        float cross = dx1 * dy2 - dy1 * dx2;

        if (cross > 0) {
            leftOfLine.push_back(p);
        }
        else if (cross < 0) {
            rightOfLine.push_back(p);
        }
    }

    // הוספת נקודת החצייה לשני הצדדים
    leftOfLine.push_back(crossingPoint);
    rightOfLine.insert(rightOfLine.begin(), crossingPoint);

    // חישוב אורכים והשוואה
    float leftLength = computePolylineLength(leftOfLine);
    float rightLength = computePolylineLength(rightOfLine);

    if (leftLength > rightLength) {
        cout << "LEFT side is longer." << endl;
    }
    else if (rightLength > leftLength) {
        cout << "RIGHT side is longer." << endl;
    }
    else {
        cout << "Both sides are equal in length." << endl;
    }
}


//float pointToLineDistance(const Point2D& p, const Point2D& lineStart, const Point2D& lineEnd) {
//    float A = p.x - lineStart.x;
//    float B = p.y - lineStart.y;
//    float C = lineEnd.x - lineStart.x;
//    float D = lineEnd.y - lineStart.y;
//
//    float dot = A * C + B * D;
//    float len_sq = C * C + D * D;
//    float param = dot / len_sq;
//
//    float xx, yy;
//
//    if (param < 0 || len_sq == 0) {
//        xx = lineStart.x;
//        yy = lineStart.y;
//    }
//    else if (param > 1) {
//        xx = lineEnd.x;
//        yy = lineEnd.y;
//    }
//    else {
//        xx = lineStart.x + param * C;
//        yy = lineStart.y + param * D;
//    }
//
//    float dx = p.x - xx;
//    float dy = p.y - yy;
//    return sqrt(dx * dx + dy * dy);
//}


//vector<Point2D> performBypass(const Point3D& startPos,
//    const Point3D& goalPos,
//    float distance,
//    Point2D crossingPoint,
//    vector<vector<Point>> all_lidar_data) {
//
//    Point3D currentPos = startPos;
//    vector<Point2D> turnPoints;
//    Eigen::Vector2d rejoinPoint;
//    bool hasRejoinedLine = false;
//    bool crossedLine = false;
//
//    while (true) {
//
//        float distFromLine = pointToLineDistance({ currentPos.x, currentPos.y },
//            { startPos.x, startPos.y },
//            { goalPos.x, goalPos.y });
//
//        if (!crossedLine && distFromLine < 0.2) {
//            crossingPoint = { currentPos.x, currentPos.y };
//            crossedLine = true;
//            cout << "Drone crossed the line at point: (" << crossingPoint.x << ", " << crossingPoint.y << ")" << endl;
//        }
//
//        if (crossedLine && !hasRejoinedLine && distFromLine < 0.2) {
//            hasRejoinedLine = true;
//            cout << "Drone rejoined the original path." << endl;
//        }
//
//        if (hasRejoinedLine) {
//            float distToStart = hypot(currentPos.x - startPos.x, currentPos.y - startPos.y);
//            if (distToStart < 0.1) {
//                cout << "Drone has returned to the starting point!" << endl;
//                break;
//            }
//        }
//
//        float angle = findMostRightFreeGap(currentPointCloud, currentPos,
//            distance);
//
//        if (isnan(angle)) {
//            cout << "No free path found, hovering..." << endl;
//            continue;
//        }
//
//        turnPoints.push_back({ currentPos.x, currentPos.y });
//
//        cout << "Moving drone in direction: " << angle << " degrees" << endl;
//        currentPos = simulateDroneStep(currentPos, angle);
//    }
//
//    return turnPoints;
//}