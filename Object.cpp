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
#include "DroneFeatures.h"
#include "LIDAR.h"
#include "TreeAVL.h"

using namespace Eigen;
using namespace std;

vector<Point> listPoint = {};

//בדיקה האם נקודה נמצאת בימין המכשול או בשמאל
int point_side_of_line(Point A, Point B, Point P)
{
    int cross = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
    return cross;
}

void Intersection_points(Point startPos, Point goalPos, Point dronePos, Point dronePrevPos, Point& point)
{
    Point currentPoint1 = dronePos;
    Point currentPoint2 = dronePrevPos;
    float m1 = (goalPos.y - startPos.y) / (goalPos.x - startPos.x);
    float b1 = startPos.y - m1 * startPos.x;
    float x1 = (startPos.y - b1) / m1;
    float m2 = (currentPoint2.y - currentPoint1.y) / (currentPoint2.x - currentPoint1.x);
    float b2 = currentPoint1.y - m2 * currentPoint1.x;
    float x2 = (currentPoint1.y - b2) / m2;
    float intersectionPointx = (b2 - b1) / (m1 - m2);
    float intersectionPointy = m1 * intersectionPointx + b1;
    Point intersectionPoint = { intersectionPointx, intersectionPointy, startPos.z };
    point = intersectionPoint;
}

bool TheTwoPointsOnLine(Point droneStart,
    Point droneTarget,
    Point& pointOnLine,
    Point& returnPoint)
  {
    // בדיקה שיש לפחות שתי נקודות ברשימה
    if (listPoint.size() < 2)
        return false;

    Point lastPoint = listPoint[listPoint.size() - 1];
    Point prevPoint = listPoint[listPoint.size() - 2];

    int sideLast = point_side_of_line(droneStart, droneTarget, lastPoint);
    int sidePrev = point_side_of_line(droneStart, droneTarget, prevPoint);

    // בדיקת חיתוך בקו בין נקודות משני צידי הקו או כאשר אחת בדיוק על הקו
    if ((sideLast > 0 && sidePrev <= 0) || (sideLast < 0 && sidePrev >= 0)) {
        Intersection_points(droneStart, droneTarget, lastPoint, prevPoint, pointOnLine);
    }
    else if ((sideLast < 0 && sidePrev >= 0) || (sideLast > 0 && sidePrev <= 0)) {
        Intersection_points(droneStart, droneTarget, lastPoint, prevPoint, returnPoint);
  }
    return true;

}


//מסננת ענן נקודות ומוצאת זווית פנויה לטיסה
float findMostRightFreeGap(const vector<Point>& cloud, Drone& Drone, droneDimension droneDim) {

    Point dronePos = Drone.getDronePos();
    Vector3f dronPosVector = { dronePos.x, dronePos.y, dronePos.z};
    float angle = -180; // התחלה מזווית שמאלית קיצונית
    const float maxAngle = 180;
    bool freeGapFound = false;

    while (angle <= maxAngle) {
        float angle_rad = angle * M_PI / 180.0f;
        Vector3f forward(cos(angle_rad), sin(angle_rad), 0);
        Vector3f up(0, 0, 1);
        Vector3f right = forward.cross(up).normalized();

        float minX = numeric_limits<float>::max();
        bool foundInside = false;
        Point mostLeftPoint;

        for (const auto& p : cloud) {
            Vector3f pVector = { p.x, p.y, p.z };
            Vector3f delta = pVector - dronPosVector;

            float x = delta.dot(forward);
            float y = delta.dot(right);
            float z = delta.dot(up);

            bool inside = fabs(x) <= droneDim.length && fabs(y) <= droneDim.width && fabs(z) <= droneDim.height;

            if (inside) {
                foundInside = true;
                if (p.x < minX) {
                    minX = p.x;
                    mostLeftPoint = p;
                }
            }
        }
        if (!foundInside) {
            // מצאנו תיבה פנויה
            if (angle != 0)
                listPoint.push_back(dronePos);
            return angle;
        }
        else {
            // מחשבים את הזווית מחדש לפי הנקודה הכי שמאלית שמצאנו
            Vector3f dir = { mostLeftPoint.x - dronePos.x, mostLeftPoint.y - dronePos.y, 0 };
            float newAngle = atan2(dir.y(), dir.x()) * 180.0f / M_PI;

            float deltaAngle = newAngle - angle;
            angle += deltaAngle;
        }
    }
    return angle;
}

//פונקציה לסכימת אורך המסלול מימין ומשמאל
float computePolylineLength(const vector<Point>& points) {
    float length = 0.0f;
    for (size_t i = 1; i < points.size(); ++i) {
        float dx = points[i].x - points[i - 1].x;
        float dy = points[i].y - points[i - 1].y;
        length += sqrt(dx * dx + dy * dy);
    }
    return length;
}

float crossProduct(const Point& a, const Point& b, const Point& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת המעטפה הקמורה עם נקודות בכיוון השעון
void convexHullClockwise(vector<Point> turnPoints,vector<Point>& hull)
{
    if (turnPoints.size() <= 1)
    {
        hull = turnPoints;
        return;
    }
    // מיון הנקודות לפי x ואז y
    sort(turnPoints.begin(), turnPoints.end(), [](const Point& a, const Point& b)
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

vector<Point> analyzeDroneTurnsAndHull(
    const Point startPos,
    const Point goalPos,
    const Point crossingPoint,
    const Point returnToLinePoint) { 

    vector<Point> hull;
    convexHullClockwise(listPoint, hull);

    vector<Point> rightOfLine;
    vector<Point> leftOfLine;

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

    leftOfLine.push_back(crossingPoint);
    rightOfLine.insert(rightOfLine.begin(), crossingPoint);

    float leftLength = computePolylineLength(leftOfLine);
    float rightLength = computePolylineLength(rightOfLine);

    if (leftLength > rightLength) {
        leftOfLine.push_back(returnToLinePoint); // הוספת הנקודה לקבוצה השמאלית
    }
    else if (rightLength > leftLength) {
        rightOfLine.push_back(returnToLinePoint); // הוספת הנקודה לקבוצה הימנית
    }
    else {
        // במקרה של שוויון, הוספה לשתי הקבוצות
        leftOfLine.push_back(returnToLinePoint);
        rightOfLine.push_back(returnToLinePoint);
    }
    return (leftLength >= rightLength) ? leftOfLine : rightOfLine;
}

// Normalize a vector
Point normalize(const Point& v) {
    float len = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0) return { 0, 0, 0 };
    return { v.x / len, v.y / len, v.z / len };
}

// Get normal (cross product of two vectors)
Point getNormal(const Point& a, const Point& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}


// Inflate or deflate polygon
vector<Point> inflatePolygon(const vector<Point>& polygon, float distance) {
    int n = polygon.size();
    vector<Point> inflated;

    for (int i = 0; i < n; ++i) {
        Point prev = polygon[(i - 1 + n) % n];
        Point curr = polygon[i];
        Point next = polygon[(i + 1) % n];

        Point edge1 = normalize(curr - prev);
        Point edge2 = normalize(next - curr);

        // נורמל למישור הנוצר מהשוליים - לא תמיד מושלם אם הפוליגון לא שטוח
        Point normal = normalize(getNormal(edge1, edge2));

        // להזיז בניצב למישור ולממוצע הכיוונים של הקצוות
        Point avgDir = normalize(edge1 + edge2);
        Point inflateDir = normalize(getNormal(avgDir, normal)); // וקטור ניצב לשניהם

        Point inflatedPoint = curr + inflateDir * distance;
        inflated.push_back(inflatedPoint);
    }

    return inflated;
}



// מוצא את כל השכנים של נקודה אחת ומכניס אותם לאשכול
void proximity(vector<Point>& points, int idx,
    vector<Point>& cluster, vector<bool>& processed,
    KDTree* tree, float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(points[idx]);

    vector<Point> neighbors = tree->radiusSearch(points[idx], distanceTol);

    for (const Point& neighbor : neighbors)
    {
        // חיפוש האינדקס של הנקודה השכנה בוקטור המקורי
        auto it = find(points.begin(), points.end(), neighbor);
        if (it != points.end())
        {
            int neighbor_idx = distance(points.begin(), it);
            if (!processed[neighbor_idx])
                proximity(points, neighbor_idx, cluster, processed, tree, distanceTol);
        }
    }
}


// בונה את רשימת האשכולות
vector<vector<Point>> euclideanCluster(vector<Point>& clude,
    KDTree* tree, float distanceTol)
{
    vector<vector<Point>> clusters;
    vector<bool> processed(clude.size(), false);

    for (int i = 0; i < clude.size(); ++i)
    {
        if (!processed[i])
        {
            vector<Point> cluster;
            proximity(clude, i, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

// מחשב את אורך האלכסון של קופסה תוחמת של אשכול
float computeClusterSize(vector<Point>& cluster)
{
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;

    for (const auto& point : cluster)
    {
        minX = min(minX, point.x);
        minY = min(minY, point.y);
        minZ = min(minZ, point.z);
        maxX = max(maxX, point.x);
        maxY = max(maxY, point.y);
        maxZ = max(maxZ, point.z);
    }

    float dx = maxX - minX;
    float dy = maxY - minY;
    float dz = maxZ - minZ;

    return sqrt(dx * dx + dy * dy + dz * dz);
}

void DivisionIntoClusters(vector<Point>& filteredCloud, vector<Point> clude)
{
    // בונים את ה-KDTree בצורה רקורסיבית ומאוזנת
    KDTree tree(clude, 0);

    // אשכול לפי מרחק
    float distanceTol = 1.5;
    vector<vector<Point>> clusters = euclideanCluster(clude, &tree, distanceTol);

    // הפרדת אשכולות קטנים מגדולים
    float minPhysicalSize = 1.0f;

    Node* rootA = nullptr;
    for (const auto& p : clude)
        insert(rootA, p);

    Node* rootB = nullptr;
    for (auto& cluster : clusters) {
        float size = computeClusterSize(cluster);
        if (size <= minPhysicalSize) {
            for (const auto& p : cluster)
                insert(rootB, p);
        }
    }
    if(rootB != nullptr)
    {
        Node* filteredLidarTree = checkTreeContainsAnotherTree(rootA, rootB);
        collectPoints(filteredLidarTree, filteredCloud); 
    }

}

void collectPoints(Node* root, vector<Point>& points) {
    if (!root) return;
    points.push_back(root->value);
    collectPoints(root->left, points); 
    collectPoints(root->right, points); 
}



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

Node* checkTreeContainsAnotherTree(Node* rootA, Node* rootB) {
    Node* a = LeftMost(rootA);
    Node* b = LeftMost(rootB);

    while (a != nullptr) {
        if (a->value == b->value) {
            Point val = a->value;  // שמירה כי a ייעלם לאחר remove
            a = successor(a);
            b = successor(b);
            rootA = removeNode(rootA, val); // מחיקת הערך מ־ A
        }
        else if (a->value < b->value) {
            a = successor(a);
        }
        else { // b->value < a->value
            b = successor(b);
        }
    }
    return rootA;
}