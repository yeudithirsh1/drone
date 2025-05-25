#ifndef EKF_HPP
#define EKF_HPP

#pragma once
#include <Eigen/Dense>

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();

    // אתחול עם וקטור מצב 15-ממדי
    // [x, y, z, vx, vy, vz, ax, ay, az, omega_x, omega_y, omega_z, roll, pitch, yaw]
    void initialize(const Eigen::VectorXd& x);

    // עדכון עם מדידת מיקום בלבד (x, y, z)
    void update(const Eigen::VectorXd& z);

    // חיזוי עם תאוצה ומהירות זוויתית כקלט (u = [ax, ay, az, omega_x, omega_y, omega_z])
    void propagate(const Eigen::VectorXd& u, double dt);

    // מצבים
    Eigen::VectorXd x_;  // וקטור מצב 15-ממדי
    Eigen::MatrixXd P_;  // מטריצת קו-וריאנץ (15x15)
    Eigen::MatrixXd Q_;  // רעש מדידה (3x3)
    Eigen::MatrixXd R_;  // רעש תהליך (15x15)
};

#endif // EKF_HPP
