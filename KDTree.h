#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "PointInSpace.h"
using namespace std;
using namespace Eigen;

class KDTree {
public:
    KDTree(); // עץ ריק
    KDTree(vector<Point>& points, int depth);
    KDTree(MatrixXf& points, vector<int>& indices, int depth);
    bool nearest(const Vector3f& target, int& nearest_index_out, float& dist_sq_out);
    void nearestSearch(const Vector3f& target, int depth, int& best_index, float& best_dist_sq, bool& found);
    vector<Point> radiusSearch(Point& target, float radius);
    void radiusSearchRec(Point& target, float radiusSq, int depth, vector<Point>& result);
private:
    // תכונות של צומת
    Point point;
    int index;
    unique_ptr<KDTree> left;
    unique_ptr<KDTree> right;
    bool valid = false;
};


