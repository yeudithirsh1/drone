#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

FusionEKF::FusionEKF()
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    
    R_radar_ = MatrixXd(3, 3);  
    R_gps_ = MatrixXd(3, 3);    
    R_lidar_ = MatrixXd(2, 2);  
    R_imu_ = MatrixXd(6, 6);    
    H_gps_ = MatrixXd(3, 9);    
    H_lidar_ = MatrixXd(2, 9);  
    H_imu_ = MatrixXd(6, 9);    

    // Initialize the matrices with realistic values
 
    R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    R_gps_ << 0.01, 0, 0,
        0, 0.01, 0,
        0, 0, 0.01;

    R_lidar_ << 0.01, 0,
        0, 0.01;

    R_imu_ = MatrixXd::Identity(6, 6) * 0.01;

    H_gps_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0;

    H_lidar_ << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0;

    H_imu_ = MatrixXd::Identity(6, 9);

    ekf_.P_ = MatrixXd(9, 9); // Initialize the state covariance matrix
    ekf_.P_ = MatrixXd::Identity(9, 9) * 1000;
    ekf_.P_(0, 0) = ekf_.P_(1, 1) = ekf_.P_(2, 2) = 1;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    if (!is_initialized_)
    {
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(9); // State vector size is 9
        ekf_.x_ << 1, 1, 1, 0, 0, 0, 0, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double theta = measurement_pack.raw_measurements_[2];
            double rho_dot = measurement_pack.raw_measurements_[3];

            double x = rho * cos(phi) * sin(theta);
            double y = rho * sin(phi) * sin(theta);
            double z = rho * cos(theta);

            double vx = rho_dot * cos(phi) * sin(theta);
            double vy = rho_dot * sin(phi) * sin(theta);
            double vz = rho_dot * cos(theta);

            ekf_.x_ << x, y, z, vx, vy, vz, 0, 0, 0;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::GPS)
        {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2], 0, 0, 0, 0, 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR)
        {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2], 0, 0, 0, 0, 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::IMU)
        {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2], measurement_pack.raw_measurements_[3], measurement_pack.raw_measurements_[4], measurement_pack.raw_measurements_[5], 0, 0, 0;
        }

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.F_ = MatrixXd(9, 9);  // State transition matrix
    ekf_.F_ << 1, 0, 0, dt, 0, 0, 0, 0, 0,
               0, 1, 0, 0, dt, 0, 0, 0, 0,
               0, 0, 1, 0, 0, dt, 0, 0, 0,
               0, 0, 0, 1, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 1, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 1, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 1;

    double noise_ax = 9.0;
    double noise_ay = 9.0;
    double noise_az = 9.0;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;
    double dt_4_4 = dt_4 / 4;
    double dt_3_2 = dt_3 / 2;

    ekf_.Q_ = MatrixXd(9, 9);
    ekf_.Q_ <<
        dt_4_4 * noise_ax, 0, 0, dt_3_2* noise_ax, 0, 0, 0, 0, 0,
        0, dt_4_4* noise_ay, 0, 0, dt_3_2* noise_ay, 0, 0, 0, 0,
        0, 0, dt_4_4* noise_az, 0, 0, dt_3_2* noise_az, 0, 0, 0,
        dt_3_2* noise_ax, 0, 0, dt_2* noise_ax, 0, 0, 0, 0, 0,
        0, dt_3_2* noise_ay, 0, 0, dt_2* noise_ay, 0, 0, 0, 0,
        0, 0, dt_3_2* noise_az, 0, 0, dt_2* noise_az, 0, 0, 0,
        0, 0, 0, 0, 0, 0, noise_ax, 0, 0,
        0, 0, 0, 0, 0, 0, 0, noise_ay, 0,
        0, 0, 0, 0, 0, 0, 0, 0, noise_az;


    ekf_.Predict();

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::GPS)
    {
        ekf_.H_ = H_gps_;
		ekf_.R_ = R_gps_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR)
    {
        ekf_.H_ = H_lidar_;
		ekf_.R_ = R_lidar_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::IMU)
    {
        ekf_.H_ = H_imu_;
        ekf_.R_ = R_imu_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
