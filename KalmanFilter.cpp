#include "KalmanFilter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init() {
    x = Matrix<float, 13, 1>::Zero();
    P = Matrix<float, 13, 13>::Identity() * 1.0f;
    F = Matrix<float, 13, 13>::Identity();
    B = Matrix<float, 13, 3>::Zero();
    Q = Matrix<float, 13, 13>::Identity() * 0.01f;

    H_gps = Matrix<float, 3, 13>::Zero();
    H_gps.block<3, 3>(0, 0) = Matrix3f::Identity(); // x, y, z
    R_gps = Matrix3f::Identity() * 2.0f;

    H_lidar = Matrix<float, 3, 13>::Zero();
    H_lidar.block<3, 3>(0, 0) = Matrix3f::Identity(); // x, y, z
    R_lidar = Matrix3f::Identity() * 1.0f;

    H_imu = Matrix<float, 7, 13>::Zero();
    H_imu.block<3, 3>(0, 6) = Matrix3f::Identity(); // ax, ay, az
    H_imu(3, 9) = 1.0f;  // yaw
    H_imu(4, 10) = 1.0f; // yaw_rate
    H_imu(5, 11) = 1.0f; // pitch
    H_imu(6, 12) = 1.0f; // pitch_rate
    R_imu = Matrix<float, 7, 7>::Identity() * 0.5f;
}

void KalmanFilter::predict(float dt, const Vector3f& u, float pitch) {
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
    x(11) = pitch;

}

void KalmanFilter::updateGPS(const Vector3f& position) {
    update(position, H_gps, R_gps);
}

void KalmanFilter::updateLidar(const VectorXf& position) {
    update(position, H_lidar, R_lidar);
}

void KalmanFilter::updateIMU(const Vector3f& linear_accel, float yaw_rate, float pitch_rate) {
    VectorXf z(7);
    z.head<3>() = linear_accel;
    z(3) = x(9);
    z(4) = yaw_rate;
    z(5) = x(11);         
    z(6) = pitch_rate;
    update(z, H_imu, R_imu);
}

void KalmanFilter::update(const VectorXf& z, const MatrixXf& H, const MatrixXf& R)
{
    {
        lock_guard<mutex> lock(updateMutex);

        VectorXf y = z - H * x;
        MatrixXf S = H * P * H.transpose() + R;
        MatrixXf K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (MatrixXf::Identity(x.size(), x.size()) - K * H) * P;
    }
}

Matrix<float, 13, 1> KalmanFilter::getState(){
    return x;
}
