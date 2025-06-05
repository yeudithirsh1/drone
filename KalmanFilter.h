#pragma once
#include <Eigen/Dense>
#include <mutex>

using namespace std;
using namespace Eigen;


class KalmanFilter {
public:
    KalmanFilter();
    void init();
    void update(const VectorXf& z, const MatrixXf& H, const MatrixXf& R);
    void predict(float dt, const Vector3f& u, float pitch);
    void updateGPS(const Vector3f& position);
    void updateLidar(const VectorXf& position);
    void updateIMU(const Vector3f& linear_accel, float yaw_rate, float pitch_rate);
    Matrix<float, 13, 1> getState();


private:
    mutex updateMutex;
    // State: [x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate, pitch, pitch_rate]
    Matrix<float, 13, 1> x;
    Matrix<float, 13, 13> P;
    Matrix<float, 13, 13> F;
    Matrix<float, 13, 13> Q;
    Matrix<float, 13, 3> B;


    Matrix<float, 3, 13> H_gps;
    Matrix3f R_gps;

    Matrix<float, 3, 13> H_lidar;
    Matrix3f R_lidar;

    Matrix<float, 7, 13> H_imu; // ax, ay, az, yaw, yaw_rate, pitch, pitch_rate
    Matrix<float, 7, 7> R_imu;
};
