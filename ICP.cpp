#include "ICP.h"
#include <iostream>
#include <numeric>
#include "Eigen/Eigen"
#include "KDTree.h"

using namespace std;
using namespace Eigen;



Matrix4f best_fit_transform(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B) {
    /*
    Notice:
    1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
    2/ matrix type 'MatrixXf' or 'MatrixXf' matters.
    */
    Matrix4f T = MatrixXf::Identity(4, 4);
    Vector3f centroid_A(0, 0, 0);
    Vector3f centroid_B(0, 0, 0);
    MatrixXf AA = A;
    MatrixXf BB = B;
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

    MatrixXf H = AA.transpose() * BB;
    MatrixXf U;
    VectorXf S;
    MatrixXf V;
    MatrixXf Vt;
    Matrix3f R;
    Vector3f t;

    JacobiSVD<MatrixXf> svd(H, ComputeFullU | ComputeFullV);
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

ICP_OUT icp(const MatrixXf& A, const MatrixXf& B, int max_iterations, int tolerance) {
    int row = A.rows();
    MatrixXf src = MatrixXf::Ones(3 + 1, row);
    MatrixXf src3d = MatrixXf::Ones(3, row);
    MatrixXf dst = MatrixXf::Ones(3 + 1, row);
    NEIGHBOR neighbor;
    Matrix4f T;
    MatrixXf dst_chorder = MatrixXf::Ones(3, row);
    ICP_OUT result;
    int iter = 0;

    for (int i = 0; i < row; i++) {
        src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        src3d.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();

    }

    float prev_error = 0;
    float mean_error = 0;
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



NEIGHBOR nearest_neighbot(const Eigen::MatrixXf& src, const Eigen::MatrixXf& dst) {
    KDTree tree(dst);
    NEIGHBOR neigh;

    for (int i = 0; i < src.rows(); ++i) {
        Eigen::Vector3f query = src.row(i).transpose();
        int nearest_idx = -1;
        float best_dist_sq = std::numeric_limits<float>::max();
        tree.nearest(query, nearest_idx, best_dist_sq);
        neigh.indices.push_back(nearest_idx);
        neigh.distances.push_back(std::sqrt(best_dist_sq));
    }

    return neigh;
}

float dist(const Eigen::Vector3f& pta, const Eigen::Vector3f& ptb) {
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) + (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}

