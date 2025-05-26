#include "GPSSoliders.h"

KalmanFilterPositionOnly::KalmanFilterPositionOnly(double processNoise, double measurementNoise)
{
    F_ = Eigen::Matrix3d::Identity();  // מצב נשאר קבוע בין מדידות
    H_ = Eigen::Matrix3d::Identity();  // מודל מדידה ישיר של מיקום
    I_ = Eigen::Matrix3d::Identity();

    Q_ = Eigen::Matrix3d::Identity() * processNoise;
    R_ = Eigen::Matrix3d::Identity() * measurementNoise;

    P_ = Eigen::Matrix3d::Identity() * 1000; // מתחילים עם חוסר וודאות גבוה
    x_ = Eigen::Vector3d::Zero();
}

void KalmanFilterPositionOnly::init(const Eigen::Vector3d& initial_position)
{
    x_ = initial_position;
}

void KalmanFilterPositionOnly::predict()
{
    // אין שינוי במצב בין מדידות - מודל סטטי
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilterPositionOnly::update(const Eigen::Vector3d& z)
{
    Eigen::Vector3d y = z - H_ * x_;              // Innovation
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;   // Innovation covariance
    Eigen::Matrix3d K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

Eigen::Vector3d KalmanFilterPositionOnly::getPosition() const
{
    return x_;
}
