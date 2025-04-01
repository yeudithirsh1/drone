#include "Tools.h"
#include <iostream>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
    VectorXd rmse(6);
    rmse << 0, 0, 0, 0, 0, 0;

    // Check the validity of the following inputs:
    // 1. The estimation vector size should not be zero
    // 2. The estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout << "Invalid estimation or ground truth data" << endl;
        return rmse;
    }

    // Accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    // Calculate the mean
    rmse = rmse / estimations.size();

    // Calculate the square root
    rmse = rmse.array().sqrt();

    // Return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj(3, 9); // ����� ���� �������
    Hj.setZero(); // ����� ���� ������� ������

    double px = x_state(0);
    double py = x_state(1);
    double pz = x_state(2); // ����� ��� ������ ����� �����
    double vx = x_state(3);
    double vy = x_state(4);
    double vz = x_state(5);
    double ax = x_state(6); // ����� ���� X
    double ay = x_state(7); // ����� ���� Y
    double az = x_state(8); // ����� ���� Z

    // ������� �������
    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = c1 * c2;

    // ����� ����� ����
    if (fabs(c1) < 0.0001)
    {
        cout << "CalculateJacobian() - Error - Division by Zero" << endl;
        return Hj;
    }

    // ����� ������ ��'��������
    Hj << px / c2, py / c2, 0, 0, 0, 0, 0, 0, 0,
        -py / c1, px / c1, 0, 0, 0, 0, 0, 0, 0,
        py* (vx * py - vy * px) / c3, px* (vy * px - vx * py) / c3, 0, px / c2, py / c2, 0, 0, 0, 0;

    return Hj;
}

