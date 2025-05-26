// KalmanFilter.cpp
#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(float dt) {
    x = Eigen::Matrix<float, 11, 1>::Zero(); // [x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate]
    P = Eigen::Matrix<float, 11, 11>::Identity() * 1.0f;

    F = Eigen::Matrix<float, 11, 11>::Identity();
    Q = Eigen::Matrix<float, 11, 11>::Identity() * 0.01f;

    H_gps = Eigen::Matrix<float, 3, 11>::Zero();
    H_gps.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // x, y, z
    R_gps = Eigen::Matrix3f::Identity() * 2.0f;

    H_lidar = Eigen::Matrix<float, 3, 11>::Zero();
    H_lidar.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // x, y, z
    R_lidar = Eigen::Matrix3f::Identity() * 1.0f;

    H_imu = Eigen::Matrix<float, 5, 11>::Zero();
    H_imu.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity(); // ax, ay, az
    H_imu(3, 9) = 1.0f;  // yaw
    H_imu(4, 10) = 1.0f; // yaw_rate
    R_imu = Eigen::Matrix<float, 5, 5>::Identity() * 0.5f;

    predict(dt); // שיערוך ראשו
}

void KalmanFilter::predict(float dt) {
    F.setIdentity();
    for (int i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;               // מיקום ← מהירות
        F(i, i + 6) = 0.5f * dt * dt;   // מיקום ← תאוצה
        F(i + 3, i + 6) = dt;           // מהירות ← תאוצה
    }

    // עדכון yaw לפי קצב הסיבוב
    F(9, 10) = dt; // yaw ← yaw_rate

    x = F * x;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::updateGPS(const Eigen::Matrix<float, 3, 1>& position) {
    Eigen::VectorXf z = position;
    update(z, H_gps, R_gps);
}

void KalmanFilter::updateLidar(const Eigen::Matrix<float, 3, 1>& position) {
    Eigen::VectorXf z = position;
    update(z, H_lidar, R_lidar);
}

void KalmanFilter::updateIMU(const Eigen::Matrix<float, 3, 1>& linear_accel, float angular_velocity_z) {
    Eigen::VectorXf z(5);
    z.head<3>() = linear_accel;
    z(3) = x(9);            // שמירת yaw נוכחי
    z(4) = angular_velocity_z; // קצב סיבוב (מדידה ישירה מ־IMU)
    update(z, H_imu, R_imu);
}

void KalmanFilter::update(const Eigen::VectorXf& z,
    const Eigen::MatrixXf& H,
    const Eigen::MatrixXf& R) {
    Eigen::VectorXf y = z - H * x;
    Eigen::MatrixXf S = H * P * H.transpose() + R;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = (Eigen::MatrixXf::Identity(x.size(), x.size()) - K * H) * P;
}

Eigen::Matrix<float, 11, 1> KalmanFilter::getState() const {
    return x;
}
