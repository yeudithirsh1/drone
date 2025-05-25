#pragma once
#ifndef KDTREE_H
#define KDTREE_H

#include <Eigen/Dense>
#include <memory>
#include <vector>
using namespace std;

class KDTree {
public:
    KDTree(); // עץ ריק
    KDTree(const Eigen::MatrixXd& data); // בניית עץ שלם
    KDTree(const std::vector<std::pair<Eigen::Vector3d, int>>& points, int depth);
    void insert(const Eigen::Vector3d& new_point, int new_index, int depth = 0);
    void nearest(const Eigen::Vector3d& point, int& index, double& dist_sq) const;
    void nearestSearch(const Eigen::Vector3d& target, int depth, int& best_idx, double& best_dist_sq) const;
    vector<float> radiusSearch(const Eigen::Vector3d& target, float radius);
    void radiusSearchRec(const Eigen::Vector3d& target, double radiusSq, int depth, vector<float>& result);

private:
    // תכונות של צומת
    Eigen::Vector3d point;
    int index;
    unique_ptr<KDTree> left;
    unique_ptr<KDTree> right;
    bool valid = false;
};

#endif // KDTREE_H

