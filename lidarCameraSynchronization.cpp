#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "lidaeCameraSynchronization.h"

using namespace std;
using namespace Eigen;

float x_center, y_center, width, height;
float xmin, ymin, xmax, ymax;

// --- פונקציית הקרנה ---
cv::Point2f projectPoint(const Vector3f& point_cam, const CameraIntrinsics& intrinsics) {
    float u = (intrinsics.fx * point_cam.x()) / point_cam.z() + intrinsics.cx;
    float v = (intrinsics.fy * point_cam.y()) / point_cam.z() + intrinsics.cy;
    return cv::Point2f(u, v);
}

// --- בדיקת אם פיקסל בתוך הבוקס ---
bool isInsideBoundingBox(const cv::Point2f& pt, const BoundingBox& box) {
    return (pt.x >= box.x_center && pt.x <= box.y_center &&
        pt.y >= box.width && pt.y <= box.height);
}

// --- טעינת קליברציה מקובץ ---
CameraCalibration loadCalibrationFromFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open calibration file: " << filename << endl;
        exit(1);
    }

    CameraCalibration calib;
    file >> calib.intrinsics.fx >> calib.intrinsics.fy >> calib.intrinsics.cx >> calib.intrinsics.cy;

    calib.extrinsics = Matrix4f::Identity();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            file >> calib.extrinsics(i, j);

    return calib;
}

ObjectLidarData getObjectPointsInLidar(const vector<cv::Point3f>& cloud,
    const Matrix4f& T_lidar_to_cam,
    const CameraIntrinsics& intrinsics,
    const BoundingBox& bbox) {
    ObjectLidarData result;
    float total_distance = 0.0f;

    for (const auto& pt : cloud) {
        Vector4f pt_lidar(pt.x, pt.y, pt.z, 1.0f);
        Vector4f pt_cam_homogeneous = T_lidar_to_cam * pt_lidar;
        Vector3f pt_cam = pt_cam_homogeneous.head<3>();

        if (pt_cam.z() > 0) {
            cv::Point2f pixel = projectPoint(pt_cam, intrinsics);
            if (isInsideBoundingBox(pixel, bbox)) {
                result.cam_points.push_back(pt_cam);
                total_distance += pt_cam.norm();  // מרחק מהמצלמה
            }
        }
    }

    if (!result.cam_points.empty()) {
        result.avg_distance = total_distance / result.cam_points.size();
    }
    else {
        result.avg_distance = -1.0f;
    }

    return result;
}

ObjectLidarData Lidar_to_camera_ratio(const string& calibration_file,
    const vector<cv::Point3f>& cloud,
    const BoundingBox& bbox) {
    CameraCalibration calib = loadCalibrationFromFile(calibration_file);
    return getObjectPointsInLidar(cloud, calib.extrinsics, calib.intrinsics, bbox);
}
