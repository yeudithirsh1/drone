#pragma once
#ifndef KDTREE_H
#define KDTREE_H
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "PointInSpace.h"
using namespace std;
using namespace Eigen;

class KDTree {
public:
    KDTree(); // עץ ריק
	KDTree(vector<Point>& points);
    KDTree(vector<Point>& points, int depth);
    void insert(Point& new_point, int depth = 0);
    bool nearest(Point& target, Point& nearest_point_out, float& dist_sq_out);
    void nearestSearch(Point& target, int depth, Point& best_point, float& best_dist_sq, bool& found);
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

#endif // KDTREE_H

