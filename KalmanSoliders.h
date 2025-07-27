#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class KalmanSoliders {
public:
    KalmanSoliders();

    void init(const Vector2f& initial_position);
    void predict();
    void update(const Vector2f& measured_position);
    void updatePosition();
private:
    Vector2f x_;      // ���: ����� [x, y, z]
    Matrix2f P_;      // ���� �����
    Matrix2f F_;      // ������ ���� ��� (����)
    Matrix2f H_;      // ������ ����� (����)
    Matrix2f Q_;      // ��� �����
    Matrix2f R_;      // ��� �����
    Matrix2f I_;      // ������ �����
};



