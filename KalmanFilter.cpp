//#include "KalmanFilter.h"
//#include <cmath>
//
//ExtendedKalmanFilter::ExtendedKalmanFilter() {
//    x_ = Eigen::VectorXd::Zero(15);
//    P_ = Eigen::MatrixXd::Identity(15, 15);
//    Q_ = Eigen::MatrixXd::Zero(15, 15);
//    R_ = Eigen::MatrixXd::Zero(9, 9);
//}
//
//void ExtendedKalmanFilter::initialize(const Eigen::VectorXd& x) {
//    x_ = x;
//    // מטריצת רעש תהליך (Q) – חוסר ודאות במודל
//    Q_ = Eigen::MatrixXd::Zero(15, 15);
//    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-4;   // מיקום
//    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;   // מהירות
//    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-2;   // תאוצה
//    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-2;   // מהירות זוויתית
//    Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-3; // זוויות
//
//    // מטריצת רעש מדידה (R) – חוסר ודאות במדידות מהחיישנים
//    R_ = Eigen::MatrixXd::Zero(9, 9);
//    R_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1.0;    // GPS / LiDAR מיקום
//    R_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-2;   // IMU תאוצה
//    R_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e-2;   // IMU ג'יירו
//}
//
//
//void ExtendedKalmanFilter::update(const Eigen::VectorXd& z) {
//    // z = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, omega_x, omega_y, omega_z]
//    Eigen::Matrix<double, 9, 15> H = Eigen::Matrix<double, 9, 15>::Zero();
//    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // מיקום
//    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(); // תאוצה
//    H.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity(); // מהירות זוויתית
//
//    Eigen::VectorXd z_pred = x_.segment<9>(0); // הנחה: המיקום, תאוצה ומהירות זוויתית בראש הוקטור
//
//    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
//    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
//
//    x_ = x_ + K * (z - z_pred);
//    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(15, 15);
//    P_ = (I - K * H) * P_;
//}
//
//void ExtendedKalmanFilter::propagate(const Eigen::VectorXd& u, double dt) {
//    // u = [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z, pos_x, pos_y, pos_z]
//    double omega_x = u(0);
//    double omega_y = u(1);
//    double omega_z = u(2);
//    double acc_x = u(3);
//    double acc_y = u(4);
//    double acc_z = u(5);
//    double pos_x = u(6);
//    double pos_y = u(7);
//    double pos_z = u(8);
//
//    // עידכון מצב
//    double& x = x_(0);
//    double& y = x_(1);
//    double& z = x_(2);
//    double& vx = x_(3);
//    double& vy = x_(4);
//    double& vz = x_(5);
//    double& ax = x_(6);
//    double& ay = x_(7);
//    double& az = x_(8);
//    double& omegaX = x_(9);
//    double& omegaY = x_(10);
//    double& omegaZ = x_(11);
//    double& roll = x_(12);
//    double& pitch = x_(13);
//    double& yaw = x_(14);
//
//    // עדכון מהירות
//    vx += ax * dt;
//    vy += ay * dt;
//    vz += az * dt;
//
//    // עדכון מיקום
//    x += vx * dt;
//    y += vy * dt;
//    z += vz * dt;
//
//    // עדכון תאוצה ומהירויות זוויתיות מהקלט
//    ax = acc_x;
//    ay = acc_y;
//    az = acc_z;
//    omegaX = omega_x;
//    omegaY = omega_y;
//    omegaZ = omega_z;
//
//    // עדכון זוויות
//    roll += omegaX * dt;
//    pitch += omegaY * dt;
//    yaw += omegaZ * dt;
//
//    // Jacobian
//    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(15, 15);
//
//    // מיקום מושפע ממהירות קווית
//    G(0, 3) = dt;  // ∂x/∂vx
//    G(1, 4) = dt;  // ∂y/∂vy
//    G(2, 5) = dt;  // ∂z/∂vz
//
//    // מהירות מושפעת מתאוצה
//    G(3, 6) = dt;  // ∂vx/∂ax
//    G(4, 7) = dt;  // ∂vy/∂ay
//    G(5, 8) = dt;  // ∂vz/∂az
//
//    // זוויות מושפעות ממהירות זוויתית
//    G(12, 9) = dt;  // ∂roll/∂omegaX
//    G(13, 10) = dt; // ∂pitch/∂omegaY
//    G(14, 11) = dt; // ∂yaw/∂omegaZ
//
//
//    P_ = G * P_ * G.transpose() + R_;
//}
