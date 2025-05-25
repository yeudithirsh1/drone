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
    KDTree(const Eigen::MatrixXf& data); // בניית עץ שלם
    KDTree(const std::vector<std::pair<Eigen::Vector3f, int>>& points, int depth);
    void insert(const Eigen::Vector3f& new_point, int new_index, int depth = 0);
    void nearest(const Eigen::Vector3f& point, int& index, float& dist_sq) const;
    void nearestSearch(const Eigen::Vector3f& target, int depth, int& best_idx, float& best_dist_sq) const;
    vector<float> radiusSearch(const Eigen::Vector3f& target, float radius);
    void radiusSearchRec(const Eigen::Vector3f& target, float radiusSq, int depth, vector<float>& result);

private:
    // תכונות של צומת
    Eigen::Vector3f point;
    int index;
    unique_ptr<KDTree> left;
    unique_ptr<KDTree> right;
    bool valid = false;
};

#endif // KDTREE_H

