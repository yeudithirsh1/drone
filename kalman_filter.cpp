#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "kalman_filter.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in,
    MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd& z)
{
	VectorXd z_pred = H_ * x_;
	VectorXd y = z.head(z_pred.size()) - z_pred;
    CommonUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
    float px = x_(0);
    float py = x_(1);
    float pz = x_(2);
    float vx = x_(3);
    float vy = x_(4);
    float vz = x_(5);

    float rho = sqrt(px * px + py * py + pz * pz);

    if (rho < 0.00001)
    {
        px += 0.001;
        py += 0.001;
        pz += 0.001;
        rho = sqrt(px * px + py * py + pz * pz);
    }

    float phi = atan2(py, px);
    float theta = acos(pz / rho);
    float rho_dot = (px * vx + py * vy + pz * vz) / rho;

    VectorXd hx(3);
    hx << rho, phi, rho_dot;

    VectorXd y = z.head(3) - hx;

    while (y(1) > M_PI || y(1) < -M_PI)
    {
        if (y(1) > M_PI) y(1) -= 2 * M_PI;
        else if (y(1) < -M_PI) y(1) += 2 * M_PI;
    }

    CommonUpdate(y);
}

void KalmanFilter::CommonUpdate(const VectorXd& y)
{
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
