// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();

    void init(float dt);
    void predict(float dt);
    void updateGPS(const Eigen::Matrix<float, 3, 1>& position);
    void updateLidar(const Eigen::Matrix<float, 3, 1>& position);
    void updateIMU(const Eigen::Matrix<float, 3, 1>& linear_accel, float angular_velocity_z);
    Eigen::Matrix<float, 11, 1> getState() const;

private:
    void update(const Eigen::VectorXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R);

    Eigen::Matrix<float, 11, 1> x; // [x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate]
    Eigen::Matrix<float, 11, 11> P;
    Eigen::Matrix<float, 11, 11> F;
    Eigen::Matrix<float, 11, 11> Q;

    Eigen::Matrix<float, 3, 11> H_gps;
    Eigen::Matrix3f R_gps;

    Eigen::Matrix<float, 3, 11> H_lidar;
    Eigen::Matrix3f R_lidar;

    Eigen::Matrix<float, 5, 11> H_imu; // ax, ay, az, yaw, yaw_rate
    Eigen::Matrix<float, 5, 5> R_imu;
};

#endif // KALMANFILTER_H
