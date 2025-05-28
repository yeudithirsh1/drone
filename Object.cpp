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
#include "TreeAVL.h"

using namespace Eigen;
using namespace std;
const float Physical_height = 2.0f;
const float Physical_width = 4.5f;
const float Physical_depth = 3.2f;
const float angleResDeg = 5.3f;

vector<Point> listPoint = {};
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;


//בדיקה האם נקודה נמצאת בימין המכשול או בשמאל
int point_side_of_line(Point A, Point B, Point P)
{
    int cross = (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x);
    return cross;
}

void computeGreatCircleIntersection(
    const Point& p1, const Point& p2,
    const Point& p3, const Point& p4,
    double& outLat, double& outLon)
{
    auto toVec = [](double latDeg, double lonDeg) {
        double lat = latDeg * DEG_TO_RAD;
        double lon = lonDeg * DEG_TO_RAD;
        return array<double, 3>{
            cos(lat)* cos(lon),
                cos(lat)* sin(lon),
                sin(lat)};
        };

    auto dot = [](const array<double, 3>& a, const array<double, 3>& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        };

    auto cross = [](const array<double, 3>& a, const array<double, 3>& b) {
        return array<double, 3>{
            a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]};
        };

    auto normalize = [&](const array<double, 3>& v) {
        double norm = sqrt(dot(v, v));
        return array<double, 3>{v[0] / norm, v[1] / norm, v[2] / norm};
        };

    auto isOnArc = [&](const array<double, 3>& A, const array<double, 3>& B, const array<double, 3>& P) {
        double angleAP = acos(dot(A, P));
        double anglePB = acos(dot(P, B));
        double angleAB = acos(dot(A, B));
        return std::abs((angleAP + anglePB) - angleAB) < 1e-8;
        };

    auto A = toVec(p1.x, p1.y);
    auto B = toVec(p2.x, p2.y);
    auto C = toVec(p3.x, p3.y);
    auto D = toVec(p4.x, p4.y);

    auto n1 = normalize(cross(A, B));
    auto n2 = normalize(cross(C, D));

    auto inter1 = normalize(cross(n1, n2));
    auto inter2 = array<double, 3>{ -inter1[0], -inter1[1], -inter1[2] };

    array<double, 3> chosen;

    if (isOnArc(A, B, inter1) && isOnArc(C, D, inter1)) {
        chosen = inter1;
    }
    else if (isOnArc(A, B, inter2) && isOnArc(C, D, inter2)) {
        chosen = inter2;
        outLat = asin(chosen[2]) * RAD_TO_DEG;
        outLon = atan2(chosen[1], chosen[0]) * RAD_TO_DEG;
    }
}


void TheTwoPointsOnLine(Point droneStart,
Point droneTarget,
Point dronePos,
Point dronePrevPos,
Point& pointOnLine,
Point& returnPoint)
{
    double outLat, outLon;
    if (point_side_of_line(droneStart, droneTarget, dronePos) > 0 && point_side_of_line(droneStart, droneTarget, dronePos) == 0
        || point_side_of_line(droneStart, droneTarget, dronePos) > 0 && point_side_of_line(droneStart, droneTarget, dronePos) < 0)
         computeGreatCircleIntersection(droneStart, droneTarget, dronePos, dronePrevPos, outLon, outLat);
    else if (point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size() - 1]) < 0 && point_side_of_line(droneStart, droneTarget, dronePos) == 0
        || point_side_of_line(droneStart, droneTarget, listPoint[listPoint.size() - 1]) < 0 && point_side_of_line(droneStart, droneTarget, dronePos) > 0)
        computeGreatCircleIntersection(droneStart, droneTarget, dronePos, dronePrevPos, outLon, outLat);
}


//מסננת ענן נקודות ומוצאת זווית פנויה לטיסה
float findMostRightFreeGap(const vector<Point>& cloud, Point dronePos) {

    Vector3f dronPoseVector = { dronePos.x, dronePos.y, dronePos.z };
    float angle = -180; // התחלה מזווית שמאלית קיצונית
    const float maxAngle = 180;
    const float maxStepDeg = 10.0f; // גבול מקסימלי לקפיצת זווית
    bool freeGapFound = false;

    while (angle <= maxAngle) {
        float angle_rad = angle * M_PI / 180.0f;
        Vector3f forward(cos(angle_rad), sin(angle_rad), 0);
        Vector3f up(0, 0, 1);
        Vector3f right = forward.cross(up).normalized();

        float minX = std::numeric_limits<float>::max();
        bool foundInside = false;
        Point mostLeftPoint;

        for (const auto& p : cloud) {
            Vector3f pVector = { p.x, p.y, p.z };
            Vector3f delta = pVector - dronPoseVector;

            float x = delta.dot(forward);
            float y = delta.dot(right);
            float z = delta.dot(up);

            bool inside = fabs(x) <= Physical_depth && fabs(y) <= Physical_width && fabs(z) <= Physical_height;

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

            // הבדל בין הזווית החדשה לקודמת – כדי לוודא שלא תהיה קפיצה גדולה מדי
            float deltaAngle = newAngle - angle;
            if (fabs(deltaAngle) > maxStepDeg)
                deltaAngle = (deltaAngle > 0 ? maxStepDeg : -maxStepDeg);

            angle += deltaAngle;
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
        length += sqrt(dx * dx + dy * dy);
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
void proximity(vector<Point>& points, int idx,
    vector<Point>& cluster, vector<bool>& processed, KDTree* tree, float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(points[idx]);
    vector<int> neighbors = tree->radiusSearch(points[idx], distanceTol);
    for (int i : neighbors)
    {
        if (!processed[i])
            proximity(points, i, cluster, processed, tree, distanceTol);
    }
}

// בונה את רשימת האשכולות
vector<vector<Point>> euclideanCluster(vector<Point>& points,
    KDTree* tree, float distanceTol)
{
    vector<vector<Point>> clusters;
    vector<bool> processed(points.size(), false);

    for (int i = 0; i < points.size(); ++i)
    {
        if (!processed[i])
        {
            vector<Point> cluster;
            proximity(points, i, cluster, processed, tree, distanceTol);
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

void DivisionIntoClusters(vector<Point> points)
{
    // בניית KDTree
    KDTree tree;
    for (int i = 0; i < points.size(); ++i)
        tree.insert(points[i], i);

    // אשכול לפי מרחק
    float distanceTol = 1.5;
    vector<vector<Point>> clusters = euclideanCluster(points, &tree, distanceTol);

    // הפרדת אשכולות קטנים מגדולים
    float minPhysicalSize = 1.0f;

    vector<vector<Point>> largeClusters;
    vector<vector<Point>> smallClusters;

    for (auto& cluster : clusters)
    {
        float size = computeClusterSize(cluster);

        if (size <= minPhysicalSize)
            smallClusters.push_back(cluster);
        else
            largeClusters.push_back(cluster);
    }

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

bool checkTreeContainsAnotherTree1(Node*& rootA, Node* rootB) {
    Node* a = LeftMost(rootA);
    Node* b = LeftMost(rootB);

    while (a != nullptr) {
        if (b == nullptr) break;

        if (a->value == b->value) {
            Point val = a->value;  // שמירה כי a ייעלם לאחר remove
            a = successor(a);
            b = successor(b);
            rootA = remove(rootA, val); // מחיקת הערך מ־A
        }
        else if (a->value < b->value) {
            a = successor(a);
        }
        else { // b->value < a->value
            b = successor(b);
        }
    }
    return true;
}




// פונקציה שבודקת אם יש נקודות מעל הרחפן בתיבה קבועה, ושומרת את הנקודה הכי שמאלית
bool findLeftmostPointAbove(const vector<Point>& cloud, const Point& dronePos,
    float width, float length, float height,
    Point& leftmostPointOut) {
    bool found = false;
    float minX = std::numeric_limits<float>::max();

    for (const auto& p : cloud) {
        float dx = p.x - dronePos.x;
        float dy = p.y - dronePos.y;
        float dz = p.z - dronePos.z;

        if (fabs(dx) <= length / 2 &&
            fabs(dy) <= width / 2 &&
            dz >= 0 && dz <= height) {

            if (p.x < minX) {
                minX = p.x;
                leftmostPointOut = p;
                found = true;
            }
        }
    }
    return found;
}
