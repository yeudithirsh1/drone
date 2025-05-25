#include "LIDAR.h"  
#include <cmath>  
#include <pcl/point_types.h>  
#include <iostream>  
#include <vector>  
#include <string>  
#include <fstream>  
#include <sstream>  
#include "ICP.h"  

using namespace std;  
using namespace Eigen;  

// Ensure the LIDAR class and its methods are properly defined in LIDAR.h  
// Add missing includes, forward declarations, or definitions if necessary  

// Placeholder function for ICP-like calculation  
void wefg() {  
   cout << "This is a placeholder for the ICP-like calculation." << endl;  
}  






//using namespace std;
//
//LIDAR::LIDAR() : lastMovement{ 0.0f, 0.0f, 0.0f } {}
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
