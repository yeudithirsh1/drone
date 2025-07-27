#include <vector>
#include <Eigen/Dense>
#include "DroneFeatures.h"

using namespace std;
using namespace Eigen;


Vector3f imageToCamera(float u, float v, float depth, const Matrix3f& K);
Vector3f rotateVector(const Vector3f& vec, float yaw_rad, float pitch_rad);
pair<float, float> localToGPS_GeoLib(float lat0, float lon0, float alt0, float dx, float dy, float dz = 0);
pair<float, float> yoloToGPS(float u, float v, float depth, const string& calibFile, Drone& drone);

