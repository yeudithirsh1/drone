#include <Eigen/Dense>
#include <cmath>
#include <utility>

using namespace std;
using namespace Eigen;
#ifndef M_PI  
#define M_PI 3.14159265358979323846  
#endif

// המרה מ-(u,v) לנקודת 3D במצלמה
Vector3d imageToCamera(double u, double v, double depth, const Matrix3d& K) {
    Vector3d pixel(u, v, 1.0);
    Vector3d cam_point = depth * K.inverse() * pixel;
    return cam_point;
}

// סיבוב וקטור לפי yaw ו-pitch (בהנחה שאין roll)
Vector3d rotateVector(const Vector3d& vec, double yaw_rad, double pitch_rad) {
    Matrix3d R_yaw;
    R_yaw << cos(yaw_rad), -sin(yaw_rad), 0,
        sin(yaw_rad), cos(yaw_rad), 0,
        0, 0, 1;

    Matrix3d R_pitch;
    R_pitch << cos(pitch_rad), 0, sin(pitch_rad),
        0, 1, 0,
        -sin(pitch_rad), 0, cos(pitch_rad);

    return R_yaw * R_pitch * vec;
}

// חיתוך קרן עם מישור הקרקע (Z=0)
Vector3d intersectWithGround(const Vector3d& origin, const Vector3d& direction) {
    if (direction.z() == 0) {
        return Vector3d::Zero();
    }
    double t = -origin.z() / direction.z();
    return origin + t * direction;
}

// המרה מהמטרים ל-GPS (פשטני)
pair<double, double> localToGPS(double lat0, double lon0, double dx, double dy) {
    const double R = 6378137; // רדיוס כדור הארץ במטרים
    double dLat = dy / R;
    double dLon = dx / (R * cos(M_PI * lat0 / 180.0));
    double lat = lat0 + dLat * 180.0 / M_PI;
    double lon = lon0 + dLon * 180.0 / M_PI;
    return { lat, lon };
}

// פונקציה שמשלבת את כל התהליך ומחזירה את ה-GPS
pair<double, double> yoloToGPS(
    double u, double v, double depth,
    const Matrix3d& K,
    double drone_height,
    double pitch_rad,
    double yaw_rad,
    double drone_lat,
    double drone_lon
) {
    Vector3d cam_point = imageToCamera(u, v, depth, K);
    Vector3d world_dir = rotateVector(cam_point.normalized(), yaw_rad, pitch_rad);
    Vector3d drone_pos(0, 0, drone_height);
    Vector3d ground_point = intersectWithGround(drone_pos, world_dir);

    if (ground_point.isZero()) {
        // במידה ולא נמצא חיתוך עם הקרקע - מחזירים ערך מיוחד (כאן 0,0)
        return { 0.0, 0.0 };
    }

    return localToGPS(drone_lat, drone_lon, ground_point.x(), ground_point.y());
}
