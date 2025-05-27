#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();

    void init(float dt);
    void predict(float dt, const Eigen::Vector3f& u);
    void updateGPS(const Eigen::Matrix<float, 3, 1>& position);
    void updateLidar(const Eigen::Matrix<float, 3, 1>& position);
    void updateIMU(const Eigen::Matrix<float, 3, 1>& linear_accel, float angular_velocity_z, float angular_velocity_y);
    Eigen::Matrix<float, 13, 1> getState() const;

private:
    void update(const Eigen::VectorXf& z,
        const Eigen::MatrixXf& H,
        const Eigen::MatrixXf& R);

    // State: [x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate, pitch, pitch_rate]
    Eigen::Matrix<float, 13, 1> x;
    Eigen::Matrix<float, 13, 13> P;
    Eigen::Matrix<float, 13, 13> F;
    Eigen::Matrix<float, 13, 13> Q;
    Eigen::Matrix<float, 13, 3> B;

    Eigen::Matrix<float, 3, 13> H_gps;
    Eigen::Matrix3f R_gps;

    Eigen::Matrix<float, 3, 13> H_lidar;
    Eigen::Matrix3f R_lidar;

    Eigen::Matrix<float, 7, 13> H_imu; // ax, ay, az, yaw, yaw_rate, pitch, pitch_rate
    Eigen::Matrix<float, 7, 7> R_imu;
};

#endif // KALMANFILTER_H
