#include "KDTree.h"
#include <algorithm>
#include <limits>

KDTree::KDTree() : index(-1), valid(false) {}

// בניית עץ מקובץ מטריצה
KDTree::KDTree(const Eigen::MatrixXd& data) {
    if (data.rows() == 0) {
        valid = false;
        return;
    }

    std::vector<std::pair<Eigen::Vector3d, int>> pts;
    for (int i = 0; i < data.rows(); ++i) {
        pts.emplace_back(data.row(i).transpose(), i);
    }

    // בניית העץ בעזרת הפעולה הרקורסיבית
    *this = KDTree(pts, 0);  // פעולה זו בטוחה כי מוגדרים move constructor ו-assignment operator דיפולטיים
}

// בניית עץ מרשימת נקודות (נקודת רקורסיה)
KDTree::KDTree(const std::vector<std::pair<Eigen::Vector3d, int>>& points, int depth) {
    if (points.empty()) {
        valid = false;
        return;
    }

    int axis = depth % 3;
    auto comp = [axis](const auto& a, const auto& b) {
        return a.first[axis] < b.first[axis];
        };
    std::vector<std::pair<Eigen::Vector3d, int>> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(), comp);

    int mid = sorted_points.size() / 2;
    point = sorted_points[mid].first;
    index = sorted_points[mid].second;
    valid = true;

    std::vector<std::pair<Eigen::Vector3d, int>> left_pts(sorted_points.begin(), sorted_points.begin() + mid);
    std::vector<std::pair<Eigen::Vector3d, int>> right_pts(sorted_points.begin() + mid + 1, sorted_points.end());

    if (!left_pts.empty())
        left = std::make_unique<KDTree>(left_pts, depth + 1);
    if (!right_pts.empty())
        right = std::make_unique<KDTree>(right_pts, depth + 1);
}


void KDTree::nearest(const Eigen::Vector3d& point, int& index, double& dist_sq) const {
    index = -1;
    dist_sq = std::numeric_limits<double>::max();
    nearestSearch(point, 0, index, dist_sq);
}

void KDTree::nearestSearch(const Eigen::Vector3d& target, int depth,
    int& best_idx, double& best_dist_sq) const {
    if (!valid) return;

    double d = (point - target).squaredNorm();
    if (d < best_dist_sq) {
        best_dist_sq = d;
        best_idx = index;
    }

    int axis = depth % 3;
    const KDTree* near = nullptr;
    const KDTree* far = nullptr;

    if (target[axis] < point[axis]) {
        near = left.get();
        far = right.get();
    }
    else {
        near = right.get();
        far = left.get();
    }

    if (near) near->nearestSearch(target, depth + 1, best_idx, best_dist_sq);

    double diff = target[axis] - point[axis];
    if (diff * diff < best_dist_sq && far) {
        far->nearestSearch(target, depth + 1, best_idx, best_dist_sq);
    }
}

vector<float> KDTree::radiusSearch(const Eigen::Vector3d& target, float radius) {
    std::vector<float> result;
    double radiusSq = static_cast<double>(radius) * static_cast<double>(radius);
    radiusSearchRec(target, radiusSq, 0, result);
    return result;
}

void KDTree::radiusSearchRec(const Eigen::Vector3d& target, double radiusSq, int depth, vector<float>& result) {
    if (!valid) return;

    double d = (point - target).squaredNorm();
    if (d <= radiusSq) {
        result.push_back(index);
    }

    int axis = depth % 3;
    double diff = target[axis] - point[axis];

    if (diff < 0) {
        if (left) left->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && right) right->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
    else {
        if (right) right->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && left) left->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
}

void KDTree::insert(const Eigen::Vector3d& new_point, int new_index, int depth) {
    if (!valid) {
        point = new_point;
        index = new_index;
        valid = true;
        return;
    }

    int axis = depth % 3;
    if (new_point[axis] < point[axis]) {
        if (left)
            left->insert(new_point, new_index, depth + 1);
        else {
            std::vector<std::pair<Eigen::Vector3d, int>> single = { {new_point, new_index} };
            left = std::make_unique<KDTree>(single, depth + 1);
        }
    }
    else {
        if (right)
            right->insert(new_point, new_index, depth + 1);
        else {
            std::vector<std::pair<Eigen::Vector3d, int>> single = { {new_point, new_index} };
            right = std::make_unique<KDTree>(single, depth + 1);
        }
    }
}