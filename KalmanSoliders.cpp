#include "KalmanSoliders.h"
#include "Soldiers.h"
#include "Geohash.h"
#include <thread>

using namespace Eigen;

KalmanSoliders::KalmanSoliders()
{
    F_ = Matrix2f::Identity();  
    H_ = Matrix2f::Identity(); 
    I_ = Matrix2f::Identity(); 
               
    Q_ = Matrix2f::Identity() * 0.01f;
    R_ = Matrix2f::Identity() * 2.0f;

    P_ = Matrix2f::Identity() * 1000; // ������� �� ���� ������ ����
    x_ = Vector2f::Zero();
}

void KalmanSoliders::init(const Vector2f& initial_position)
{
    x_ = initial_position;
}

void KalmanSoliders::predict()
{
    // ��� ����� ���� ��� ������ - ���� ����
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanSoliders::update(const Vector2f& z)
{
    Vector2f y = z - H_ * x_;              
    Matrix2f S = H_ * P_ * H_.transpose() + R_;   
    Matrix2f K = P_ * H_.transpose() * S.inverse(); 

    x_ = x_ + K * y;
	P_ = (I_ - K * H_) * P_; // ����� ���� ������� - ����� ��������� ��� ���� ����� ������� �������
}

void KalmanSoliders::updatePosition()
{
    string geohash = encodeGeohash(x_[0], x_[1], 12);
    soldierLocationUpdate(geohash);
    this_thread::sleep_for(chrono::milliseconds(100)); // ����� 10 ����� ������
}
