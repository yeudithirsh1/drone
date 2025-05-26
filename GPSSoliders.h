#pragma once
#ifndef KALMAN_FILTER_POSITION_ONLY_H
#define KALMAN_FILTER_POSITION_ONLY_H

#include <Eigen/Dense>

class KalmanFilterPositionOnly {
public:
    KalmanFilterPositionOnly(double processNoise, double measurementNoise);

    void init(const Eigen::Vector3d& initial_position);
    void predict();
    void update(const Eigen::Vector3d& measured_position);
    Eigen::Vector3d getPosition() const;

private:
    Eigen::Vector3d x_;      // ���: ����� [x, y, z]
    Eigen::Matrix3d P_;      // ���� �����
    Eigen::Matrix3d F_;      // ������ ���� ��� (����)
    Eigen::Matrix3d H_;      // ������ ����� (����)
    Eigen::Matrix3d Q_;      // ��� �����
    Eigen::Matrix3d R_;      // ��� �����
    Eigen::Matrix3d I_;      // ������ �����
};

#endif // KALMAN_FILTER_POSITION_ONLY_H


