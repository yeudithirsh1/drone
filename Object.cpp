#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "ICP.h"
#include "Eigen/Eigen"
#include "Object.h"
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <iostream>
#include "PointInSpace.h"
#include "KDTree.h"


using namespace Eigen;
using namespace std;
const float Physical_height = 2.0f;
const float Physical_width = 4.5f;
const float Physical_depth = 3.2f;
const float angleResDeg = 5.3f;

vector<Point> listPoint = {};

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
    Point intersectionPoint = { intersectionPointx, intersectionPointy, startPos.z };
    return intersectionPoint;
}

//מסננת ענן נקודות ומוצאת זווית פנויה לטיסה
float findMostRightFreeGap(const vector<Point>& cloud,
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

    vector<Point2f> projected;
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

            float x = delta.dot(forward);
            float y = delta.dot(right);
            float z = delta.dot(up);     

            bool inside = fabs(x) <= Physical_depth && fabs(y) <= Physical_width && fabs(z) <= Physical_height;

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


//פונקציה לסכימת אורך המסלול מימין ומשמאל
float computePolylineLength(const vector<Point2f>& points) {
    float length = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        length += std::sqrt(dx * dx + dy * dy);
    }
    return length;
}

float crossProduct(const Point2f& a, const Point2f& b, const Point2f& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת המעטפה הקמורה עם נקודות בכיוון השעון
void convexHullClockwise(vector<Point2f> turnPoints,vector<Point2f>& hull)
{
    if (turnPoints.size() <= 1)
    {
        hull = turnPoints;
        return;
    }
    // מיון הנקודות לפי x ואז y
    sort(turnPoints.begin(), turnPoints.end(), [](const Point2f& a, const Point2f& b)
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
void analyzeDroneTurnsAndHull(const Point& startPos,
    const Point& goalPos,
    float deltaZ,
    float radius,
    float minGapWidth,
    const Point2f& crossingPoint,
    const vector<Point2f>& turnPoints) {

    vector<Point2f> hull;
    convexHullClockwise(turnPoints, hull);

    vector<Point2f> rightOfLine;
    vector<Point2f> leftOfLine;

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

// Normalize a vector
Point2f normalize(const Point2f& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y);
    if (len == 0) return { 0, 0 };
    return { static_cast<float> (v.x / len),static_cast<float> (v.y / len) };
}

// Rotate 90 degrees counter-clockwise to get normal vector
Point2f getNormal(const Point2f& v) {
    return { -v.y, v.x };
}

// Inflate or deflate polygon
std::vector<Point2f> inflatePolygon(const std::vector<Point2f>& polygon, float distance) {
    int n = polygon.size();
    std::vector<Point2f> inflated;

    for (int i = 0; i < n; ++i) {
        Point2f prev = polygon[(i - 1 + n) % n];
        Point2f curr = polygon[i];
        Point2f next = polygon[(i + 1) % n];

        Point2f edge1 = normalize(curr - prev);
        Point2f edge2 = normalize(next - curr);

        Point2f normal1 = getNormal(edge1);
        Point2f normal2 = getNormal(edge2);

        Point2f avgNormal = normalize(normal1 + normal2);
        Point2f inflatedPoint = curr + avgNormal * distance;

        inflated.push_back(inflatedPoint);
    }

    return inflated;
}



// מוצא את כל השכנים של נקודה אחת ומכניס אותם לאשכול
void proximity(const vector<Eigen::Vector3f>& points, int idx,
    vector<int>& cluster, vector<bool>& processed, KDTree* tree, float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(idx);
    vector<float> neighbors = tree->radiusSearch(points[idx], distanceTol);
    for (int i : neighbors)
    {
        if (!processed[i])
            proximity(points, i, cluster, processed, tree, distanceTol);
    }
}

// בונה את רשימת האשכולות
vector<vector<int>> euclideanCluster(const vector<Eigen::Vector3f>& points,
    KDTree* tree, float distanceTol)
{
    vector<vector<int>> clusters;
    vector<bool> processed(points.size(), false);

    for (int i = 0; i < points.size(); ++i)
    {
        if (!processed[i])
        {
            vector<int> cluster;
            proximity(points, i, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

// מחשב את אורך האלכסון של קופסה תוחמת של אשכול
float computeClusterSize(const vector<Eigen::Vector3f>& points, const vector<int>& cluster)
{
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;

    for (int idx : cluster)
    {
        const auto& point = points[idx];
        minX = std::min(minX, point[0]);
        minY = std::min(minY, point[1]);
        minZ = std::min(minZ, point[2]);
        maxX = std::max(maxX, point[0]);
        maxY = std::max(maxY, point[1]);
        maxZ = std::max(maxZ, point[2]);
    }

    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void DivisionIntoClusters(vector<Eigen::Vector3f> points)
{
    // שלב 1: בניית KDTree
    KDTree tree;
    for (int i = 0; i < points.size(); ++i)
        tree.insert(points[i], i);

    // שלב 2: אשכול לפי מרחק
    float distanceTol = 1.5;
    vector<vector<int>> clusters = euclideanCluster(points, &tree, distanceTol);

    // שלב 3: הפרדת אשכולות קטנים מגדולים
    float minPhysicalSize = 1.0f;

    vector<vector<int>> largeClusters;
    vector<vector<int>> smallClusters;

    for (const auto& cluster : clusters)
    {
        float size = computeClusterSize(points, cluster);

        if (size <= minPhysicalSize)
            smallClusters.push_back(cluster);
    }
}



struct Node {
    int value;
    Node* left;
    Node* right;
    Node* parent;
};

// מציאת הצומת השמאלי ביותר בתת עץ
Node* LeftMost(Node* node) {
    while (node->left != nullptr) {
        node = node->left;
    }
    return node;
}

// מציאת האיבר העוקב
Node* successor(Node* node) {
    if (node->right != nullptr) {
        return LeftMost(node->right);
    }

    while (node->parent != nullptr && node == node->parent->right) {
        node = node->parent;
    }

    return node->parent;
}

bool checkTreeContainsAnotherTree1(Node* rootA, Node* rootB) {
    Node* a = LeftMost(rootA);
    Node* b = LeftMost(rootB);

    while (a != nullptr && b != nullptr) {
        while (b != nullptr && a->value > b->value) {
            b = successor(b);
        }

        if (b == nullptr || b->value > a->value) {
            return false;
        }

        a = successor(a);
        b = successor(b);
    }

    return true;
}
