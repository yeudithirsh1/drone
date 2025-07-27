#include "ICP.h"
#include <iostream>
#include <numeric>
#include "Eigen/Eigen"
#include "KDTree.h"

using namespace std;
using namespace Eigen;

Matrix4f best_fit_transform(const MatrixXf& A, const MatrixXf& B) {
   
    Matrix4f T = MatrixXf::Identity(4, 4);//����� ������ ������������ ������� ������

	//����� ����� ���� ���� �� A ��� B
    Vector3f centroid_A(0, 0, 0);
	Vector3f centroid_B(0, 0, 0);

	//����� ���� �� A �-B ��� ����� �� ������� ��������
	MatrixXf AA = A;
	MatrixXf BB = B;

    // ����� ���� ���� �� A �-B
    // ����� ������ ���� �� ������� �������� A �-B
	// ���� ���, ������ ����� ������ ��� ���� �� ���� ����
	int row = A.rows();

    for (int i = 0; i < row; i++) {
        centroid_A += A.block<1, 3>(i, 0).transpose(); 
        centroid_B += B.block<1, 3>(i, 0).transpose();
    }
    centroid_A /= row;
    centroid_B /= row;

	//����� �������: ������ ��� ����� �� ���� ���� ��� �� ��� ����� ��� ����� �����  (0,0,0) �� ��� ���� ������ SVD
    for (int i = 0; i < row; i++) {
        AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
        BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
    }

    //�� ������ ������ ����� �� ������� �� ������� �������� �����, ���� ����� H
    MatrixXf H = AA.transpose() * B;

    //����� ������ �����������
    MatrixXf U;
    VectorXf S;
    MatrixXf V; 

	MatrixXf Vt; // ������ �������� �� V
    Matrix3f R; //������ ����� 
    Vector3f t; //������ ����

	// ����� �-SVD �� ������ �������� H
    //���� ������: ����� �� ������ R ������ ������ �� A ��B
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

    //������ ����
    t = centroid_B - R * centroid_A;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;

}

ICP_OUT icp(const MatrixXf& A, const MatrixXf& B, int max_iterations, int tolerance) {
    int row = A.rows(); // ���� ������ ������� A
    MatrixXf src = MatrixXf::Ones(4, row);
    MatrixXf src3f = MatrixXf::Ones(3, row);
    MatrixXf dst = MatrixXf::Ones(4, row);
    NEIGHBOR neighbor;
    Matrix4f T;
    MatrixXf dst_chorder = MatrixXf::Ones(3, row);
    ICP_OUT result;
    int iter = 0;

    for (int i = 0; i < row; i++) {
        src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        src3f.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
    }

    float prev_error = 0;
    float mean_error = 0;
    bool continue_loop = true;
    int i = 0;

    while (continue_loop && i < max_iterations) {
        neighbor = nearest_neighbot(src3f.transpose(), B);

        for (int j = 0; j < row; j++) {
            dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
        }

        T = best_fit_transform(src3f.transpose(), dst_chorder.transpose());

        src = T * src;
        for (int j = 0; j < row; j++) {
            src3f.block<3, 1>(0, j) = src.block<3, 1>(0, j);
        }

        mean_error = accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / neighbor.distances.size();

        if (abs(prev_error - mean_error) < tolerance) {
            continue_loop = false;  // ������� �� ������
        }
        else {
            prev_error = mean_error;
            iter = i + 2;
        }
        i++;
    }

    T = best_fit_transform(A, src3f.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}



//����� �� ���� ����� ����� ���� �� ����� ������� src ���� ���� ������ ������� dst
NEIGHBOR nearest_neighbot(const MatrixXf& src, const MatrixXf& dst) {

    vector<int> dst_indices(dst.rows());    // ����� ����� �������� �-dst
	iota(dst_indices.begin(), dst_indices.end(), 0); // ��� �� ������ ������ �-0 �� ���� �-dst

	MatrixXf dst_copy = dst; // ����� ���� �� ������ �-dst ��� ����� �� ������� ��������
    KDTree tree(dst_copy, dst_indices, 0);    // ����� �� KD


    NEIGHBOR neigh;//����� ���� �� ������ ��������� �� ������ ������� ����� �������� �����

	for (int i = 0; i < src.rows(); ++i) {      //���� �� �� ������� �������-src
		Vector3f query = src.row(i).transpose();    //���� ����� ������� ��� ����
		int nearest_idx = -1;      //����� ������� ����� ����� �- -1
		float best_dist_sq = numeric_limits<float>::max(); //����� ����� ���� ����� ������ ���� �������� ������
        tree.nearest(query, nearest_idx, best_dist_sq); //����� ���� ����� ����� ������ ������� 
		neigh.indices.push_back(nearest_idx);  //����� ������� �� ���� ����� ����� ����� �-NEIGHBOR
		neigh.distances.push_back(sqrt(best_dist_sq)); //����� ����� �� ���� ����� ����� ����� �-NEIGHBOR
    }

    return neigh;
}


float dist(const Vector3f& pta, const Vector3f& ptb) {
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) + (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}

