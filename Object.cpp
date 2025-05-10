#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include "nevigation.h"

using namespace Eigen;
using namespace std;
using namespace pcl;

// === ������ ===
const float SAFE_DISTANCE = 2.0f;
const float ANGLE_THRESHOLD = 15.0f; // �� ����� ������

// === �������� ===
bool isDirectionFree(const PointCloud<PointXYZ>::Ptr& cloud,
    const Vector3f& pos,
    const Vector3f& dir,
    vector<Vector3f>& obstructingPoints) // ����� ����� ����� �� �� ������ ������
{
    KdTreeFLANN<PointXYZ> kdtree;
    PointXYZ searchPoint(pos.x(), pos.y(), pos.z());
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;

    // ����� ������ ������ SAFE_DISTANCE
    if (kdtree.radiusSearch(searchPoint, SAFE_DISTANCE, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        // ������ �� �� ������� ������
        for (int idx : pointIdxRadiusSearch) {
            const auto& pt = cloud->points[idx];
            Vector3f obstacle(pt.x, pt.y, pt.z);
            Vector3f rel = obstacle - pos;

            // ������ �� ������ ��� ������ ���� ������ ������
            float angle = acos(rel.normalized().dot(dir.normalized())) * 180.0f / M_PI;
            if (angle < ANGLE_THRESHOLD) {
                obstructingPoints.push_back(obstacle); // ������� �� ������ ������
            }
        }
    }

    return obstructingPoints.empty(); // �� ��� �����, ������� true
}
