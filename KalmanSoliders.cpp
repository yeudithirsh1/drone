#include "KalmanSoliders.h"

using namespace Eigen;

KalmanSoliders::KalmanSoliders()
{
    F_ = Matrix2f::Identity();  // מצב נשאר קבוע בין מדידות
    H_ = Matrix2f::Identity();  // מודל מדידה ישיר של מיקום
    I_ = Matrix2f::Identity();
               
    Q_ = Matrix2f::Identity() * 0.01f;
    R_ = Matrix2f::Identity() * 2.0f;

    P_ = Matrix2f::Identity() * 1000; // מתחילים עם חוסר וודאות גבוה
    x_ = Vector2f::Zero();
}

void KalmanSoliders::init(const Vector2f& initial_position)
{
    x_ = initial_position;
}

void KalmanSoliders::predict()
{
    // אין שינוי במצב בין מדידות - מודל סטטי
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanSoliders::update(const Vector2f& z)
{
    Vector2f y = z - H_ * x_;              // Innovation
    Matrix2f S = H_ * P_ * H_.transpose() + R_;   // Innovation covariance
    Matrix2f K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    x_ = x_ + K * y;
    P_ = (I_ - K * H_) * P_;
}

Vector2f KalmanSoliders::getPosition()
{
    return x_;
}
