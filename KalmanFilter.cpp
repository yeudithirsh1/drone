#include "KalmanFilter.h"
#include "Global.h"
#include <shared_mutex>
#include "DroneFeatures.h"

using namespace Eigen;
extern bool reachedDestination;
extern shared_mutex mutexReachedDestination;


KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(float initial_x, float initial_y, float initial_z) {
    x = Matrix<float, 13, 1>::Zero();
    x(0) = initial_x;  // מיקום X
    x(1) = initial_y;  // מיקום Y
    x(2) = initial_z;  // מיקום Z
    P = Matrix<float, 13, 13>::Identity() * 1.0f;
    P(9, 9) = 100.0f; // חוסר ודאות גדול ב־yaw
    F = Matrix<float, 13, 13>::Identity();
    B = Matrix<float, 13, 3>::Zero();
    Q = Matrix<float, 13, 13>::Identity() * 0.01f;

    H_gps = Matrix<float, 3, 13>::Zero();
    H_gps.block<3, 3>(0, 0) = Matrix3f::Identity(); // x, y, z
    R_gps = Matrix3f::Identity() * 2.0f;

    H_lidar = Matrix<float, 3, 13>::Zero();
    H_lidar.block<3, 3>(0, 0) = Matrix3f::Identity(); // x, y, z
    R_lidar = Matrix3f::Identity() * 1.0f;

    H_imu = Matrix<float, 7, 13>::Zero();
    H_imu.block<3, 3>(0, 6) = Matrix3f::Identity(); // ax, ay, az
    H_imu(3, 9) = 1.0f;  // yaw
    H_imu(4, 10) = 1.0f; // yaw_rate
    H_imu(5, 11) = 1.0f; // pitch
    H_imu(6, 12) = 1.0f; // pitch_rate
    R_imu = Matrix<float, 7, 7>::Identity() * 0.5f;
}

void KalmanFilter::predict(float dt, const Vector3f& u, float yaw, float pitch) {
    F.setIdentity();
    for (int i = 0; i < 3; ++i) {
        F(i, i + 3) = dt;               // מיקום ← מהירות
        F(i, i + 6) = 0.5f * dt * dt;   // מיקום ← תאוצה
        F(i + 3, i + 6) = dt;           // מהירות ← תאוצה
    }

    B.setZero();
    for (int i = 0; i < 3; ++i) {
        B(i, i) = 0.5f * dt * dt;       // תאוצה משפיעה על מיקום
        B(i + 3, i) = dt;               // תאוצה משפיעה על מהירות
    }

    x = F * x + B * u;
    P = F * P * F.transpose() + Q;
    float new_yaw = x(9) + yaw;   
    x(10) = (new_yaw - x(9)) / dt;  
    x(9) = new_yaw;
    x(11) = pitch;
}

void KalmanFilter::predictLoop(Drone& drone) 
{
    auto last_time = chrono::steady_clock::now();
    while (true) {
        {
            shared_lock<shared_mutex> lock(mutexReachedDestination);
            if (reachedDestination) {
                break;
            }
        }
        Vector3f u;
        float yaw;
        float pitch;

        {
            shared_lock<shared_mutex> lock(predictMutex);
            u = latest_u;
            yaw = latest_yaw;
            pitch = latest_pitch;
        }

        auto current_time = chrono::steady_clock::now();
        float dt = chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        predict(dt, u, yaw, pitch);
        updatingDroneVariables(x, drone);
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}


// פונקציה שמעדכנת קלט חדש
void KalmanFilter::externalInputUpdate(const Vector3f& new_u, float new_yaw, float new_pitch) 
{
    {
        unique_lock<shared_mutex> lock(predictMutex);
        latest_u = new_u;
        latest_yaw = new_yaw;
        latest_pitch = new_pitch;
    }
}


void KalmanFilter::updateGPS(const Vector3f& position) {
    update(position, H_gps, R_gps);
}

void KalmanFilter::updateLidar(const VectorXf& position) {
    update(position, H_lidar, R_lidar);
}

void KalmanFilter::updateIMU(const Vector3f& linear_accel, float yaw_rate, float pitch_rate) {
    VectorXf z(7);
    z.head<3>() = linear_accel;
    z(3) = x(9);
    z(4) = yaw_rate;
    z(5) = x(11);         
    z(6) = pitch_rate;
    update(z, H_imu, R_imu);
}

void KalmanFilter::update(const VectorXf& z, const MatrixXf& H, const MatrixXf& R)
{
    {
        unique_lock<shared_mutex> lock(updateMutex);
        VectorXf y = z - H * x;
        MatrixXf S = H * P * H.transpose() + R;
        MatrixXf K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (MatrixXf::Identity(x.size(), x.size()) - K * H) * P;
    }
}

void KalmanFilter::updatingDroneVariables(Matrix<float, 13, 1> x, Drone& drone)
{
    Point pos = { x(0), x(1), x(2) };
    drone.setDronePos(pos);
    Velocity speed = { x(3), x(4), x(5) };
    drone.setSpeedInAxes(speed);
    Acceleration acc = { x(6), x(7), x(8) };
    drone.setAccelerationInAxes(acc);
    drone.setYaw(x(9));
    drone.setYawRate(x(10));
    drone.setPitch(x(11));
    drone.setPitchRate(x(12)); 
    float velocity = sqrt(drone.getSpeedInAxes().vx * drone.getSpeedInAxes().vx +
        drone.getSpeedInAxes().vy * drone.getSpeedInAxes().vy +
        drone.getSpeedInAxes().vz * drone.getSpeedInAxes().vz);
    drone.setVelocity()
}
