#ifndef POINT_IN_SPACE_H  
#define POINT_IN_SPACE_H  

struct Point {  
   float x, y, z;  

   // Default constructor to fix the error  
   Point() : x(0), y(0), z(0) {}  

   // Constructor to initialize Point with x, y, z values  
   Point(float x, float y, float z) : x(x), y(y), z(z) {} 
};  

#endif // POINT_IN_SPACE_H

//#pragma once
//#include <vector>
//#include <utility> // בשביל std::pair
//#include "PointInSpace.h" // כולל את הקובץ PointInSpace.h
//#include "ICP.h"
//using namespace std;
//
//
//
//class Lidar {
//private:
//    vector<Point> currentScan;      // סריקה נוכחית
//    vector<Point> previousScan;     // סריקה קודמת
//    Point lastMovement;            // תזוזה אחרונה שחושבה
//    ICP_OUT icpTransformation; // אובייקט ICP לחישוב תזוזות
//    bool isMovementNoisy;            // האם התזוזה רועשת
//    float distance;
//public:
//    // Constructor
//    Lidar();
//
//    // Getters
//    vector<Point> getCurrentScan();
//    vector<Point> getPreviousScan();
//    Point getLastMovement();
//    ICP_OUT getIcpTransformation();
//    bool getIsMovementNoisy();
//    float getDistance();
//
//    // Setters
//    void setCurrentScan(vector<Point>& scan);
//    void setPreviousScan(vector<Point>& scan);
//    void setLastMovement(Point& movement);
//    void setIcpTransformation(ICP_OUT& icpResult);
//    void setIsMovementNoisy(bool noisy);
//    void setDistance(float dist);
//
//    // Simple placeholder for ICP-like calculation
//    Point calculateMovementICP(const vector<Point>& scan1, const vector<Point>& scan2);
//    void mergePointClouds(const vector<vector<Point>>& clouds) {
//        void processNewScan(const vector<Point>&newScan);
//
//    };





//#include "LIDAR.h"
//#include <cmath>
//#include <pcl/point_types.h>
//#include <iostream>
//#include <vector>
//#include <string>
//#include <fstream>
//#include <sstream>
//#include "ICP.h"
//
//using namespace std;
//
//Lidar::Lidar() : lastMovement{ 0.0f, 0.0f, 0.0f } {}
//
//vector<Point> Lidar::getCurrentScan()
//{
//    return currentScan;
//}
//
//vector<Point> Lidar::getPreviousScan()
//{
//    return previousScan;
//}
//
//Point Lidar::getLastMovement()
//{
//    return lastMovement;
//}
//
//ICP_OUT Lidar::getIcpTransformation()
//{
//    return icpTransformation;
//}
//
//bool Lidar::getIsMovementNoisy() {
//    return isMovementNoisy;
//}
//float Lidar::getDistance() {
//    return distance;
//}
//
//void Lidar::setCurrentScan(vector<Point>& scan) {
//    currentScan = scan;
//}
//
//void Lidar::setPreviousScan(vector<Point>& scan) {
//    previousScan = scan;
//}
//
//void Lidar::setLastMovement(Point& movement) {
//    lastMovement = movement;
//}
//
//void Lidar::setIcpTransformation(ICP_OUT& icpResult) {
//    icpTransformation = icpResult;
//}
//void Lidar::setIsMovementNoisy(bool noisy) {
//    isMovementNoisy = noisy;
//}
//void Lidar::setDistance(float distance) {
//    this->distance = distance;
//}
//
//void loadMultiplePointCloudsAsVectors(const vector<string>& filePaths) {
//    vector<vector<Point>> clouds;
//
//    for (const auto& path : filePaths) {
//        ifstream file(path);
//        if (!file.is_open()) {
//            cerr << "Error opening file: " << path << endl;
//            continue;
//        }
//
//        vector<Point> cloud;
//        string line;
//        while (getline(file, line)) {
//            istringstream iss(line);
//            Point p;
//            if (!(iss >> p.x >> p.y >> p.z)) {
//                // שורה לא תקינה, מדלגים
//                continue;
//            }
//            cloud.push_back(p);
//        }
//
//        clouds.push_back(cloud);
//    }
//    mergePointClouds(clouds);
//}
//
//
////פונקציה לחיבור ענני נקודות
//void Lidar::mergePointClouds(const vector<vector<Point>>& clouds) {
//    vector<Point> merged;
//
//    // מחשבים כמה נקודות בסך הכל כדי להזמין מקום מראש
//    size_t totalSize = 0;
//    for (const auto& cloud : clouds) {
//        totalSize += cloud.size();
//    }
//    merged.reserve(totalSize);
//
//    // מוסיפים כל ענן בנפרד
//    for (const auto& cloud : clouds) {
//        merged.insert(merged.end(), cloud.begin(), cloud.end());
//    }
//
//}
//
//void Lidar::c(vector<vector<Point>>& clouds) {
//    vector<Point> merged;
//
//    // חישוב גודל כולל
//    size_t totalSize = 0;
//    for (const auto& cloud : clouds) {
//        totalSize += cloud.size();
//    }
//    merged.reserve(totalSize);
//
//    // מיזוג
//    for (const auto& cloud : clouds) {
//        merged.insert(merged.end(), cloud.begin(), cloud.end());
//    }
//
//    // עדכון currentScan
//    currentScan = merged;
//
//    // המרה למטריצות ישירות מתוך currentScan
//    Eigen::MatrixXd currentMat(currentScan.size(), 3);
//    for (size_t i = 0; i < currentScan.size(); ++i) {
//        currentMat(i, 0) = currentScan[i].x;
//        currentMat(i, 1) = currentScan[i].y;
//        currentMat(i, 2) = currentScan[i].z;
//    }
//
//    // המרה גם ל־ previousScan
//    Eigen::MatrixXd previousMat(previousScan.size(), 3);
//    for (size_t i = 0; i < previousScan.size(); ++i) {
//        previousMat(i, 0) = previousScan[i].x;
//        previousMat(i, 1) = previousScan[i].y;
//        previousMat(i, 2) = previousScan[i].z;
//    }
//    ICP_OUT result = icp(currentMat, previousMat);
//}
//
//
//
//void Lidar::processNewScan(const vector<Point>& newScan) {
//    previousScan = currentScan;
//    currentScan = newScan;
//    distance = sqrt(
//        lastMovement.x * lastMovement.x +
//        lastMovement.y * lastMovement.y +
//        lastMovement.z * lastMovement.z);
//    isMovementNoisy = (distance > 5.0f); // סף לדוגמה
//}
//
//
//
