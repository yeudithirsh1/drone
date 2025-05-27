#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(float dt) {
    x = Eigen::Matrix<float, 13, 1>::Zero();
    P = Eigen::Matrix<float, 13, 13>::Identity() * 1.0f;
    F = Eigen::Matrix<float, 13, 13>::Identity();
    B = Eigen::Matrix<float, 13, 3>::Zero();
    Q = Eigen::Matrix<float, 13, 13>::Identity() * 0.01f;

    H_gps = Eigen::Matrix<float, 3, 13>::Zero();
    H_gps.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // x, y, z
    R_gps = Eigen::Matrix3f::Identity() * 2.0f;

    H_lidar = Eigen::Matrix<float, 3, 13>::Zero();
    H_lidar.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity(); // x, y, z
    R_lidar = Eigen::Matrix3f::Identity() * 1.0f;

    H_imu = Eigen::Matrix<float, 7, 13>::Zero();
    H_imu.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity(); // ax, ay, az
    H_imu(3, 9) = 1.0f;  // yaw
    H_imu(4, 10) = 1.0f; // yaw_rate
    H_imu(5, 11) = 1.0f; // pitch
    H_imu(6, 12) = 1.0f; // pitch_rate
    R_imu = Eigen::Matrix<float, 7, 7>::Identity() * 0.5f;
}

void KalmanFilter::predict(float dt, const Eigen::Vector3f& u) {
    F.setIdentity();
    for (int i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;               // מיקום ← מהירות
        F(i, i + 6) = 0.5f * dt * dt;   // מיקום ← תאוצה
        F(i + 3, i + 6) = dt;           // מהירות ← תאוצה
    }

    B.setZero();
    for (int i = 0; i < 3; ++i) {
        B(i, i) = 0.5f * dt * dt;       // תאוצה משפיעה על מיקום
        B(i + 3, i) = dt;               // תאוצה משפיעה על מהירות
    }

    x = F * x + B * u;
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

void KalmanFilter::updateIMU(const Eigen::Matrix<float, 3, 1>& linear_accel, float angular_velocity_z, float angular_velocity_y) {
    Eigen::VectorXf z(7);
    z.head<3>() = linear_accel;
    z(3) = x(9);          // yaw נוכחי
    z(4) = angular_velocity_z;
    z(5) = x(11);         // pitch נוכחי
    z(6) = angular_velocity_y;
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

Eigen::Matrix<float, 13, 1> KalmanFilter::getState() const {
    return x;
}
