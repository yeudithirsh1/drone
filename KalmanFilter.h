#ifndef EKF_HPP
#define EKF_HPP

#pragma once
#include <Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    // ����� �� ����� ��� 15-����
    // [x, y, z, vx, vy, vz, ax, ay, az, omega_x, omega_y, omega_z, roll, pitch, yaw]
    void initialize(const Eigen::VectorXd& x);

    // ����� �� ����� ����� ���� (x, y, z)
    void update(const Eigen::VectorXd& z);

    // ����� �� ����� ������� ������� ���� (u = [ax, ay, az, omega_x, omega_y, omega_z])
    void propagate(const Eigen::VectorXd& u, double dt);

    // �����
    Eigen::VectorXd x_;  // ����� ��� 15-����
    Eigen::MatrixXd P_;  // ������ ��-������ (15x15)
    Eigen::MatrixXd Q_;  // ��� ����� (3x3)
    Eigen::MatrixXd R_;  // ��� ����� (15x15)
};

#endif // EKF_HPP
