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
    Eigen::Vector3d x_;      // מצב: מיקום [x, y, z]
    Eigen::Matrix3d P_;      // טעות אומדן
    Eigen::Matrix3d F_;      // מטריצת מעבר מצב (זהות)
    Eigen::Matrix3d H_;      // מטריצת מדידה (זהות)
    Eigen::Matrix3d Q_;      // רעש תהליך
    Eigen::Matrix3d R_;      // רעש מדידה
    Eigen::Matrix3d I_;      // מטריצת יחידה
};

#endif // KALMAN_FILTER_POSITION_ONLY_H


