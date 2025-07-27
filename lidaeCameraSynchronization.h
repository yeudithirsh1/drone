#include <Eigen/Dense>  
#include <opencv2/core/types.hpp>
#include <vector>
#include <string>

using namespace Eigen;
using namespace std;


struct BoundingBox {
    float x_center, y_center, width, height;
};

struct CameraIntrinsics {
    float fx, fy, cx, cy;
};

struct CameraCalibration {
    CameraIntrinsics intrinsics;
    Matrix4f extrinsics;
};

struct ObjectLidarData {
    vector<Vector3f> cam_points;     // נקודות לאחר המרה למערכת המצלמה
    float avg_distance;              // מרחק ממוצע מהמצלמה
};

cv::Point2f projectPoint(const Vector3f& point_cam, const CameraIntrinsics& intrinsics);
bool isInsideBoundingBox(const cv::Point2f& pt, const BoundingBox& box);
CameraCalibration loadCalibrationFromFile(const string& filename);
ObjectLidarData getObjectPointsInLidar(const vector<cv::Point3f>& cloud,
    const Matrix4f& T_lidar_to_cam,
    const CameraIntrinsics& intrinsics,
    const BoundingBox& bbox);
ObjectLidarData Lidar_to_camera_ratio(const string& calibration_file,
    const vector<cv::Point3f>& cloud,
    const BoundingBox& bbox);





