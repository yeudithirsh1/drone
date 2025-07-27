#pragma once  
#include "PointInSpace.h"
#include "KDTree.h"
#include <vector>
#include "DroneFeatures.h"
#include "TreeAVL.h"
#include "LIDAR.h"


int point_side_of_line(Point A, Point B, Point P);
void Intersection_points(Point startPos, Point goalPos, Point dronePos, Point dronePrevPos, Point& point);
bool TheTwoPointsOnLine(Point droneStart, Point droneTarget, Point& pointOnLine, Point& returnPoint);
float findMostRightFreeGap(LIDAR& lidar, Drone& Drone, droneDimension droneDim);
float computePolylineLength(const vector<Point>& points);
float crossProduct(const Point& a, const Point& b, const Point& c);
vector<Point> analyzeDroneTurnsAndHull(const Point startPos, const Point goalPos, const Point crossingPoint, const Point returnToLinePoint);
Point normalize(const Point& v);
Point getNormal(const Point & a, const Point & b);
vector<Point> inflatePolygon(const vector<Point>&polygon, float distance);
void proximity(vector<Point>& points, int idx, vector<Point>& cluster, vector<bool>& processed, KDTree* tree, float distanceTol);
float computeClusterSize(vector<Point>& cluster);
vector<Point> DivisionIntoClusters(vector<Point> filteredCloud, vector<Point> clude);
Node* LeftMost(Node* node);
Node* successor(Node* node);
Node* checkTreeContainsAnotherTree(Node* rootA, Node* rootB);
void collectPoints(Node* root, vector<Point>& points);




