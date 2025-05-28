#include "KDTree.h"
#include <algorithm>
#include <limits>
#include "PointInSpace.h"
using namespace std;

KDTree::KDTree() : valid(false) {}

KDTree::KDTree(vector<Point>& data) {
    if (data.empty()) {
        valid = false;
        return;
    }
    *this = KDTree(data, 0);
}

KDTree::KDTree(vector<Point>& points, int depth) {
    if (points.empty()) {
        valid = false;
        return;
    }

    int axis = depth % 3;
    auto comp = [axis](const Point& a, const Point& b) {
        if (axis == 0) return a.x < b.x;
        if (axis == 1) return a.y < b.y;
        return a.z < b.z;
        };

    vector<Point> sorted_points = points;
    sort(sorted_points.begin(), sorted_points.end(), comp);

    int mid = sorted_points.size() / 2;
    point = sorted_points[mid];
    valid = true;

    vector<Point> left_pts(sorted_points.begin(), sorted_points.begin() + mid);
    vector<Point> right_pts(sorted_points.begin() + mid + 1, sorted_points.end());

    if (!left_pts.empty())
        left = make_unique<KDTree>(left_pts, depth + 1);
    if (!right_pts.empty())
        right = make_unique<KDTree>(right_pts, depth + 1);
}

void KDTree::insert(Point& new_point, int depth) {
    if (!valid) {
        point = new_point;
        valid = true;
        return;
    }

    int axis = depth % 3;
    float new_val = (axis == 0) ? new_point.x : (axis == 1) ? new_point.y : new_point.z;
    float cur_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;

    if (new_val < cur_val) {
        if (left)
            left->insert(new_point, depth + 1);
        else
            left = make_unique<KDTree>(vector<Point>{new_point}, depth + 1);
    }
    else {
        if (right)
            right->insert(new_point, depth + 1);
        else
            right = make_unique<KDTree>(vector<Point>{new_point}, depth + 1);
    }
}

bool KDTree::nearest(Point& target, Point& nearest_point_out, float& dist_sq_out) {
    if (!valid) return false;

    dist_sq_out = numeric_limits<float>::max();
    bool found = false;
    nearestSearch(target, 0, nearest_point_out, dist_sq_out, found);
    return found;
}

void KDTree::nearestSearch(Point& target, int depth,
    Point& best_point, float& best_dist_sq, bool& found) {
    if (!valid) return;

    float dx = point.x - target.x;
    float dy = point.y - target.y;
    float dz = point.z - target.z;
    float d = dx * dx + dy * dy + dz * dz;

    if (d < best_dist_sq) {
        best_dist_sq = d;
        best_point = point;
        found = true;
    }

    int axis = depth % 3;
    float target_val = (axis == 0) ? target.x : (axis == 1) ? target.y : target.z;
    float point_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;

    KDTree* near = (target_val < point_val) ? left.get() : right.get();
    KDTree* far = (target_val < point_val) ? right.get() : left.get();

    if (near) near->nearestSearch(target, depth + 1, best_point, best_dist_sq, found);

    float diff = target_val - point_val;
    if (diff * diff < best_dist_sq && far)
        far->nearestSearch(target, depth + 1, best_point, best_dist_sq, found);
}

vector<Point> KDTree::radiusSearch(Point& target, float radius){
    vector<Point> result;
    float radiusSq = radius * radius;
    radiusSearchRec(target, radiusSq, 0, result);
    return result;
}

void KDTree::radiusSearchRec(Point& target, float radiusSq, int depth, vector<Point>& result){
    if (!valid) return;

    float dx = point.x - target.x;
    float dy = point.y - target.y;
    float dz = point.z - target.z;
    float d = dx * dx + dy * dy + dz * dz;

    if (d <= radiusSq)
        result.push_back(point);

    int axis = depth % 3;
    float target_val = (axis == 0) ? target.x : (axis == 1) ? target.y : target.z;
    float point_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;

    float diff = target_val - point_val;

    if (diff < 0) {
        if (left) left->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && right)
            right->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
    else {
        if (right) right->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && left)
            left->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
}
