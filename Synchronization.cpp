#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace Eigen;

struct BoundingBox {
    int xmin, ymin, xmax, ymax;
};

struct CameraIntrinsics {
    float fx, fy, cx, cy;
};

Point2f projectPoint(const Vector3f& point_cam, const CameraIntrinsics& intrinsics) {
    float u = (intrinsics.fx * point_cam.x()) / point_cam.z() + intrinsics.cx;
    float v = (intrinsics.fy * point_cam.y()) / point_cam.z() + intrinsics.cy;
    return cv::Point2f(u, v);
}

bool isInsideBoundingBox(const Point2f& pt, const BoundingBox& box) {
    return (pt.x >= box.xmin && pt.x <= box.xmax && pt.y >= box.ymin && pt.y <= box.ymax);
}

vector<PointXYZ> getObjectPointsInLidar(PointCloud<PointXYZ>::Ptr cloud,
    const Matrix4f& T_lidar_to_cam,
    const CameraIntrinsics& intrinsics,
    const BoundingBox& bbox) {
    vector<PointXYZ> result;

    for (const auto& pt : cloud->points) {
        Vector4f pt_lidar(pt.x, pt.y, pt.z, 1.0f);
        Vector4f pt_cam = T_lidar_to_cam * pt_lidar;

        if (pt_cam.z() <= 0) continue;

        Point2f pixel = projectPoint(pt_cam.head<3>(), intrinsics);
        if (isInsideBoundingBox(pixel, bbox)) {
            result.push_back(pt);
        }
    }
    return result;
}

PointCloud<PointXYZ>::Ptr loadLidarFromFile(const string& filename) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    ifstream file(filename);
    float x, y, z;
    while (file >> x >> y >> z) {
        cloud->push_back(PointXYZ(x, y, z));
    }
    return cloud;
}

CameraIntrinsics loadIntrinsicsFromFile(const string& filename) {
    ifstream file(filename);
    CameraIntrinsics intrinsics;
    file >> intrinsics.fx >> intrinsics.fy >> intrinsics.cx >> intrinsics.cy;
    return intrinsics;
}

vector<PointXYZ> Lidar_to_camera_ratio()
 {
    string lidar_file = "C:/Users/This User/Downloads/0000000371.txt";
    string intrinsics_file = "C:/Users/This User/Downloads/371.txt";
    string image_file = "C:/Users/This User/Downloads/0000000371.png";

    // קריאת תמונה
    Mat image = imread(image_file);
    if (image.empty()) {
        cerr << "Couldn't load image." << endl;
    }

    // קריאת נתוני לידאר ופרמטרים פנימיים של מצלמה
    PointCloud<PointXYZ>::Ptr cloud = loadLidarFromFile(lidar_file);
    CameraIntrinsics intrinsics = loadIntrinsicsFromFile(intrinsics_file);

    // תיבת גבול - נבחרת ידנית לדוגמה
    BoundingBox bbox = { 300, 200, 400, 300 };

    // כאן תכניסי את ערכי R ו-t מהמיפוי שלך, לדוגמה
    Mat R = (Mat_<double>(3, 3) << 1, 0, 0,
        0, 1, 0,
        0, 0, 1);
    Mat tvec = (Mat_<double>(3, 1) << 0, 0, 0);

    Vector3f t_eigen(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    Matrix3f R_eigen;
    R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);

    Matrix4f T_lidar_to_cam = Matrix4f::Identity();
    T_lidar_to_cam.block<3, 3>(0, 0) = R_eigen;
    T_lidar_to_cam.block<3, 1>(0, 3) = t_eigen;

    // קבלת הנקודות שנמצאות בתוך הבוקס
    vector<PointXYZ> object_points_in_lidar = getObjectPointsInLidar(cloud, T_lidar_to_cam, intrinsics, bbox);

    // הדפסת נקודות
    cout << "Points inside bounding box:" << endl;
    for (const auto& pt : object_points_in_lidar) {
        cout << pt.x << ", " << pt.y << ", " << pt.z << endl;
    }
	return object_points_in_lidar;

}
