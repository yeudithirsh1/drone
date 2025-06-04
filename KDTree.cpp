#include "KDTree.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include "PointInSpace.h"

using namespace std;


KDTree::KDTree() : valid(false) {}

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


KDTree::KDTree(MatrixXf& points, vector<int>& indices, int depth) {
    if (points.rows() == 0) {
        valid = false;
        return;
    }

    int axis = depth % 3;

    // שלב 1: יצירת סדר מיון לפי הציר
    vector<size_t> order(points.rows());
    iota(order.begin(), order.end(), 0);

    sort(order.begin(), order.end(), [&](size_t i, size_t j) {
        return points(i, axis) < points(j, axis);
        });

    // שלב 2: מיון לפי הסדר
    MatrixXf sorted_points(points.rows(), 3);
    vector<int> sorted_indices(indices.size());
    for (size_t i = 0; i < order.size(); ++i) {
        sorted_points.row(i) = points.row(order[i]);
        sorted_indices[i] = indices[order[i]];
    }

    int mid = sorted_points.rows() / 2;
    point = { sorted_points(mid, 0), sorted_points(mid, 1), sorted_points(mid, 2) };
    index = sorted_indices[mid];
    valid = true;

    if (mid > 0) {
        MatrixXf left_pts = sorted_points.topRows(mid);
        vector<int> left_indices(sorted_indices.begin(), sorted_indices.begin() + mid);
        left = make_unique<KDTree>(left_pts, left_indices, depth + 1);
    }

    if (mid + 1 < sorted_points.rows()) {
        MatrixXf right_pts = sorted_points.bottomRows(sorted_points.rows() - mid - 1);
        vector<int> right_indices(sorted_indices.begin() + mid + 1, sorted_indices.end());
        right = make_unique<KDTree>(right_pts, right_indices, depth + 1);
    }
}


bool KDTree::nearest(const Vector3f& target, int& nearest_index_out, float& dist_sq_out) {
    if (!valid) return false;

    dist_sq_out = std::numeric_limits<float>::max();
    bool found = false;
    nearestSearch(target, 0, nearest_index_out, dist_sq_out, found);
    return found;
}

void KDTree::nearestSearch(const Vector3f& target, int depth, int& best_index, float& best_dist_sq, bool& found) {
    if (!valid) return;

    float dx = point.x - target(0);
    float dy = point.y - target(1);
    float dz = point.z - target(2);
    float d = dx * dx + dy * dy + dz * dz;

    if (d < best_dist_sq) {
        best_dist_sq = d;
        best_index = index;
        found = true;
    }

    int axis = depth % 3;
    float target_val = target(axis);
    float point_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;

    KDTree* near = (target_val < point_val) ? left.get() : right.get();
    KDTree* far = (target_val < point_val) ? right.get() : left.get();

    if (near) near->nearestSearch(target, depth + 1, best_index, best_dist_sq, found);

    float diff = target_val - point_val;
    if (diff * diff < best_dist_sq && far)
        far->nearestSearch(target, depth + 1, best_index, best_dist_sq, found);
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
