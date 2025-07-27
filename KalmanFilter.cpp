#include "KalmanFilter.h"
#include "Global.h"
#include <shared_mutex>
#include "DroneFeatures.h"

using namespace Eigen;

KalmanFilter::KalmanFilter() {}

Matrix<float, 13, 1> KalmanFilter::getX() {
    shared_lock<shared_mutex>lock(XMutex);
    return x;
}
void KalmanFilter::setX(const Matrix<float, 13, 1>& newX) {
    unique_lock<shared_mutex> lock(XMutex);
    x = newX;
}
Matrix<float, 13, 13> KalmanFilter::getP() {
    lock_guard<shared_mutex> lock(PMutex);
    return P;
}
void KalmanFilter::setP(const Matrix<float, 13, 13>& newP) {
    unique_lock<shared_mutex> lock(PMutex);
    P = newP;
}

VectorXf KalmanFilter::getLatestVectorControl() {
    lock_guard<shared_mutex> lock(latestVectorControlMutex);
    return latestVectorControl;
}
void KalmanFilter::setLatestVectorControl(VectorXf newLatestVectorControl) {
    unique_lock<shared_mutex> lock(latestVectorControlMutex);
    latestVectorControl = newLatestVectorControl;
}

void KalmanFilter::init(float initial_x, float initial_y, float initial_z) {
    x = Matrix<float, 13, 1>::Zero();
    x(0) = initial_x;  // מיקום X
    x(1) = initial_y;  // מיקום Y
    x(2) = initial_z;  // מיקום Z

	//תפקיד המטריצה P הוא לייצג את חוסר הוודאות של המצב הנוכחי של המערכת.
    P = Matrix<float, 13, 13>::Identity() * 1.0f; 
    P(9, 9) = 100.0f; // חוסר ודאות גדול ב־yaw
	P(11, 11) = 100.0f; // חוסר ודאות גדול ב־pitch

    //תפקיד המטריצה F לתאר איך המצב משתנה עם הזמן ללא מדידות וללא פעולה חיצונית
    F = Matrix<float, 13, 13>::Identity();

	// תפקיד המטריצה B הוא לתאר את השפעת הפעולה החיצונית על המצב
    B = Matrix<float, 13, 3>::Zero();

	// תפקיד המטריצה Q הוא לייצג את חוסר הוודאות במודל המערכת עצמו
    Q = Matrix<float, 13, 13>::Identity() * 0.01f;

	// הגדרת מטריצות המדידה והחוסר ודאות שלהן
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

void KalmanFilter::predict(float dt, const VectorXf& VectorControl) {
    F.setIdentity();
    for (int i = 0; i < 3; i++) {
        F(i, i + 3) = dt;               // מיקום ← מהירות
        F(i, i + 6) = 0.5f * dt * dt;   // מיקום ← תאוצה
        F(i + 3, i + 6) = dt;           // מהירות ← תאוצה
    }

    B.setZero();
    for (int i = 0; i < 3; i++) {
        B(i, i) = 0.5f * dt * dt;     // מיקום ← תאוצה
        B(i + 3, i) = dt;             // מהירות ← תאוצה
    }

    // רכיבי סיבוב:
    B(9, 3) = 0.5f * dt * dt;         // yaw ← תאוצה זוויתית
    B(10, 3) = dt;                    // yawRate ← תאוצה זוויתית

    B(11, 4) = 0.5f * dt * dt;        // pitch ← תאוצה זוויתית
    B(12, 4) = dt;                    // pitchRate ← תאוצה זוויתית

    x = F * x + B * VectorControl;    // חיזוי המצב הבא
    P = F * P * F.transpose() + Q;    // עדכון חוסר הוודאות
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
    drone.setVelocity(velocity);
    float acceleration = sqrt(drone.getAccelerationInAxes().ax * drone.getAccelerationInAxes().ax +
        drone.getAccelerationInAxes().ay * drone.getAccelerationInAxes().ay +
		drone.getAccelerationInAxes().az * drone.getAccelerationInAxes().az);
	drone.setAcceleration(acceleration);
}

void KalmanFilter::predictLoop(Drone& drone)
{
    auto last_time = chrono::steady_clock::now();
    while (!getReachedDestination())
    {        
        VectorXf vectorControl;
		vectorControl = getLatestVectorControl();

        auto current_time = chrono::steady_clock::now();
        float dt = chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        predict(dt, vectorControl);
        updatingDroneVariables(x, drone);
        this_thread::sleep_for(chrono::milliseconds(100));
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
    VectorXf y = z - H * getX();//ההפש בין חיזוי המדידה לבין המדידה עצמה
    MatrixXf S = H * getP() * H.transpose() + R;//חוסר הוודאות בתחזית המדידה מוסיפים גם את חוסר הוודאות של החיישן התוצאה כמה אנחנו בטוחים במדידה 
    MatrixXf K = getP() * H.transpose() * S.inverse();//מטריצת הגיינמן, קובעת כמה להעדיף את המדידה על פני החיזוי

    setX(getX() + K * y);//עדכון המצב הנוכחי של המערכת על סמך המדידה
    setP((MatrixXf::Identity(x.size(), x.size()) - K * H) * getP());//לאחר תיקון לפי מדידה, אנחנו סומכים יותר על המצב ולכן חוסר הוודאות קטן
}


