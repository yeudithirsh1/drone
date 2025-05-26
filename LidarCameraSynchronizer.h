#pragma once  
#include <opencv2/core/types.hpp> // Ensure Point2f is defined  
#include <Eigen/Dense> // Ensure Vector3f is defined  
#include <vector> // Include vector template  
#include <pcl/point_cloud.h> // Include PointCloud  
#include <pcl/point_types.h> // Include PointXYZ  

using namespace Eigen;  
using namespace cv;  
using namespace std;  
using namespace pcl;  

struct BoundingBox {  
  int xmin, ymin, xmax, ymax;  
};  

struct CameraIntrinsics {  
  float fx, fy, cx, cy;  
};  
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
class Synchronization
{  
private:  
  BoundingBox boundingBox;  
  CameraIntrinsics cameraIntrinsics;  

public:  
    Synchronization(int xmin, int ymin, int xmax, int ymax, float fx, float fy, float cx, float cy) :
      boundingBox{ xmin, ymin, xmax, ymax },  
      cameraIntrinsics{ fx, fy, cx, cy } {}  

  Point2f projectPoint(const Vector3f& point_cam, const CameraIntrinsics& intrinsics);  
  bool isInsideBoundingBox(const Point2f& pt, const BoundingBox& box);  
  vector<PointXYZ> getObjectPointsInLidar(PointCloud<PointXYZ>::Ptr cloud, const Matrix4f& T_lidar_to_cam, const CameraIntrinsics& intrinsics, const BoundingBox& bbox);
  PointCloud<PointXYZ>::Ptr loadLidarFromFile(const string& filename);
  CameraIntrinsics loadIntrinsicsFromFile(const string& filename);
  vector<PointXYZ> Lidar_to_camera_ratio();
};
