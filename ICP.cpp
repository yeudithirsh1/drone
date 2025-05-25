#include "ICP.h"
#include <iostream>
#include <numeric>
#include "Eigen/Eigen"
#include "KDTree.h"

using namespace std;
using namespace Eigen;



Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    /*
    Notice:
    1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
    2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
    */
    Matrix4d T = MatrixXd::Identity(4, 4);
    Vector3d centroid_A(0, 0, 0);
    Vector3d centroid_B(0, 0, 0);
    MatrixXd AA = A;
    MatrixXd BB = B;
    int row = A.rows();

    for (int i = 0; i < row; i++) {
        centroid_A += A.block<1, 3>(i, 0).transpose();
        centroid_B += B.block<1, 3>(i, 0).transpose();
    }
    centroid_A /= row;
    centroid_B /= row;
    for (int i = 0; i < row; i++) {
        AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
        BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
    }

    MatrixXd H = AA.transpose() * BB;
    MatrixXd U;
    VectorXd S;
    MatrixXd V;
    MatrixXd Vt;
    Matrix3d R;
    Vector3d t;

    JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    R = Vt.transpose() * U.transpose();

    if (R.determinant() < 0) {
        Vt.block<1, 3>(2, 0) *= -1;
        R = Vt.transpose() * U.transpose();
    }

    t = centroid_B - R * centroid_A;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;

}

ICP_OUT icp(const MatrixXd& A, const MatrixXd& B, int max_iterations, int tolerance) {
    int row = A.rows();
    MatrixXd src = MatrixXd::Ones(3 + 1, row);
    MatrixXd src3d = MatrixXd::Ones(3, row);
    MatrixXd dst = MatrixXd::Ones(3 + 1, row);
    NEIGHBOR neighbor;
    Matrix4d T;
    MatrixXd dst_chorder = MatrixXd::Ones(3, row);
    ICP_OUT result;
    int iter = 0;

    for (int i = 0; i < row; i++) {
        src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();

    }

    double prev_error = 0;
    double mean_error = 0;
    for (int i = 0; i < max_iterations; i++) {
        neighbor = nearest_neighbot(src3d.transpose(), B);

        for (int j = 0; j < row; j++) {
            dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
        }

        T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

        src = T * src;
        for (int j = 0; j < row; j++) {
            src3d.block<3, 1>(0, j) = src.block<3, 1>(0, j);
        }

        mean_error = std::accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / neighbor.distances.size();
        if (abs(prev_error - mean_error) < tolerance) {
            break;
        }
        prev_error = mean_error;
        iter = i + 2;
    }

    T = best_fit_transform(A, src3d.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}



NEIGHBOR nearest_neighbot(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst) {
    KDTree tree(dst);
    NEIGHBOR neigh;

    for (int i = 0; i < src.rows(); ++i) {
        Eigen::Vector3d query = src.row(i).transpose();
        int nearest_idx = -1;
        double best_dist_sq = std::numeric_limits<double>::max();
        tree.nearest(query, nearest_idx, best_dist_sq);
        neigh.indices.push_back(nearest_idx);
        neigh.distances.push_back(std::sqrt(best_dist_sq));
    }

    return neigh;
}

float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb) {
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) + (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}

