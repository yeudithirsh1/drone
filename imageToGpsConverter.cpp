#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include "DroneFeatures.h"
#include <fstream>
#include <iostream>

using namespace Eigen;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// המרה מנקודה בתמונה (u,v,depth) לנקודה במרחב התלת־ממדי של המצלמה
Vector3f imageToCamera(float u, float v, float depth, const Matrix3f& K) {
    Vector3f pixel(u, v, 1.0f);
    return depth * K.inverse() * pixel;
}

// סיבוב הנקודה לפי yaw ו־pitch
Vector3f rotateVector(const Vector3f& vec, float yaw, float pitch) {
    Matrix3f R_yaw, R_pitch;

    R_yaw << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    R_pitch << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    return R_yaw * R_pitch * vec;
}

// חיתוך קרן עם הקרקע (Z = 0)
Vector3f intersectWithGround(const Vector3f& origin, const Vector3f& dir) {
    if (dir.z() == 0) return Vector3f::Zero();
    float t = -origin.z() / dir.z();
    return origin + t * dir;
}

// המרת קואורדינטות מקומיות ל־GPS
pair<float, float> localToGPS_GeoLib(float lat0, float lon0, float alt0, float dx, float dy, float dz = 0) {
    GeographicLib::LocalCartesian proj(lat0, lon0, alt0);
    double lat, lon, alt;
    proj.Reverse(dx, dy, dz, lat, lon, alt);
    return { lat, lon };
}

Eigen::Matrix3f loadMatrixFromFile(const std::string& filename) {
    Eigen::Matrix3f K;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open calibration file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            file >> K(i, j);

    file.close();
    return K;
}

// פונקציה סופית: YOLO → GPS עם קריאת מטריצת K מתוך הקובץ
pair<float, float> yoloToGPS(float u, float v, float depth, const string& calibFile, Drone& drone) {
    Matrix3f K = loadMatrixFromFile(calibFile);

    Vector3f cam_point = imageToCamera(u, v, depth, K);
    Vector3f dir_world = rotateVector(cam_point.normalized(), drone.getYaw(), drone.getPitch());
    Vector3f drone_pos(0, 0, drone.getTargetAltitude());
    Vector3f ground_point = intersectWithGround(drone_pos, dir_world);

    if (ground_point.isZero()) return { 0.0f, 0.0f };

    return localToGPS_GeoLib(drone.getDronePos().x, drone.getDronePos().y, drone.getTargetAltitude(),
        ground_point.x(), ground_point.y(), 0);
}
