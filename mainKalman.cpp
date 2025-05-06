#include <math.h>
#include <iostream>
#include "FusionEKF.h"
#include "tools.h"
#include "measurement_package.h"
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

void processSensorData(const string& sensor_measurement, FusionEKF& fusionEKF, Tools& tools, vector<VectorXd>& estimations, vector<VectorXd>& ground_truth)
{
    MeasurementPackage meas_package;
    istringstream iss(sensor_measurement);

    long long timestamp;
    string sensor_type;
    iss >> sensor_type;

    if (sensor_type.compare("R") == 0)
    {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(4);
        float ro;
        float theta;
        float phi;
        float ro_dot;
        iss >> ro;
        iss >> theta;
        iss >> phi;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro, theta, phi, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }
    else if (sensor_type.compare("G") == 0)
    {
        meas_package.sensor_type_ = MeasurementPackage::GPS;
        meas_package.raw_measurements_ = VectorXd(3);
        float lat;
        float lon;
        float alt;
        iss >> lat;
        iss >> lon;
        iss >> alt;
        meas_package.raw_measurements_ << lat, lon, alt;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }
    else if (sensor_type.compare("LIDAR") == 0)
    {
        meas_package.sensor_type_ = MeasurementPackage::LIDAR;
        meas_package.raw_measurements_ = VectorXd(3);
        float x;
        float y;
        float z;
        iss >> x;
        iss >> y;
        iss >> z;
        meas_package.raw_measurements_ << x, y, z;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }
    else if (sensor_type.compare("I") == 0)
    {
        meas_package.sensor_type_ = MeasurementPackage::IMU;
        meas_package.raw_measurements_ = VectorXd(9);
        float ax, ay, az, gx, gy, gz, wx, wy, wz;
        iss >> ax >> ay >> az >> gx >> gy >> gz >> wx >> wy >> wz;
        meas_package.raw_measurements_ << ax, ay, az, gx, gy, gz, wx, wy, wz;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
    }

    float x_gt;
    float y_gt;
    float z_gt;
    float vx_gt;
    float vy_gt;
    float vz_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> z_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    iss >> vz_gt;

    VectorXd gt_values(6);
    gt_values(0) = x_gt;
    gt_values(1) = y_gt;
    gt_values(2) = z_gt;
    gt_values(3) = vx_gt;
    gt_values(4) = vy_gt;
    gt_values(5) = vz_gt;
    ground_truth.push_back(gt_values);

    fusionEKF.ProcessMeasurement(meas_package);

    VectorXd estimate(6);

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double p_z = fusionEKF.ekf_.x_(2);
    double v1 = fusionEKF.ekf_.x_(3);
    double v2 = fusionEKF.ekf_.x_(4);
    double v3 = fusionEKF.ekf_.x_(5);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = p_z;
    estimate(3) = v1;
    estimate(4) = v2;
    estimate(5) = v3;

    estimations.push_back(estimate);

    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

    cout << "Estimate: [" << p_x << ", " << p_y << ", " << p_z << ", " << v1 << ", " << v2 << ", " << v3 << "]" << std::endl;
    cout << "RMSE: [" << RMSE(0) << ", " << RMSE(1) << ", " << RMSE(2) << ", " << RMSE(3) << ", " << RMSE(4) << ", " << RMSE(5) << "]" << std::endl;
}

//int main()
//{
//    FusionEKF fusionEKF;
//    Tools tools;
//    vector<VectorXd> estimations;
//    vector<VectorXd> ground_truth;
//
//    vector<string> sensor_data = {
//        "R 1.0 0.5 0.1 0.5 1477010443100000 1.1 2.1 0.3 0.5 0.6 0.7",
//        "G 37.7749 -122.4194 30.0 1477010443200000 1.2 2.2 0.4 0.6 0.7 0.8",
//        "I 0.1 0.2 0.3 0.01 0.02 0.03 0.001 0.002 0.003 1477010443300000 1.3 2.3 0.5 0.7 0.8 0.9",
//        "LIDAR 1.0 2.0 3.0 1477010443400000 1.4 2.4 0.6 0.8 0.9 1.0"
//    };
//
//    for (const auto& data : sensor_data)
//    {
//        processSensorData(data, fusionEKF, tools, estimations, ground_truth);
//    }
//
//    return 0;
//}
