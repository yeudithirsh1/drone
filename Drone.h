//#pragma once
//#include "pointInSpace.h" // כולל את הקובץ pointSpace.h
//
//struct Motor {
//	bool isActive; // האם המנוע פעיל
//	int speed; // מהירות המנוע (ב- RPM)
//	int direction; // כיוון המנוע (1 או -1)
//	Motor(bool isActive, int speed, int direction) : isActive(isActive), speed(speed), direction(direction) {} // בנאי
//};
//
//struct droneDimension
//{
//	float length;
//	float width;
//	float height;
//	droneDimension(float l, float w, float h) : length(l), width(w), height(h) {} // בנאי
//};
//
//struct Velocity {
//	float vx;
//	float vy;
//	float vz;
//	Velocity(float x, float y, float z) : vx(x), vy(y), vz(z) {} // בנאי
//};
//
//struct Acceleration
//{
//	float wx;
//	float wy;
//	float wz;
//	Acceleration(float x, float y, float z) : wx(x), wy(y), wz(z) {} // בנאי
//};
//
//struct DroneDirection {
//	float yaw; // זווית ה-Yaw של הרחפן
//	float pitch; // זווית ה-Pitch של הרחפן
//	float roll; // זווית ה-Roll של הרחפן
//	DroneDirection(float y, float p, float r) : yaw(y), pitch(p), roll(r) {} // בנאי
//};
//
//struct MovementDirection
//{
//	float heading_deg;    // אופקי: כיוון התנועה במפה (0-360)
//	float elevation_deg;  // אנכי: כיוון כלפי מעלה/מטה (-90 עד 90)
//};
//
//
//class Drone
//{
//private:
//	const float maxRPM = 5000.0f;
//	const float target_altitude;
//	const float max_altitude;
//	const float mass;
//	const float A;
//	const float C_d;
//	const float C_t;
//	float rpm;
//	float rho;
//	float velocity;
//	float acceleration;
//	float yawRate; // מהירות סיבוב סביב ציר האנכי, ביחידות rad/s
//
//	Motor motor1;//קדמי שמאל
//	Motor motor2;//קדמי ימין
//	Motor motor3;//אחורי שמאל
//	Motor motor4;//אחורי ימין
//	droneDimension droneDim;
//	Point dronePos;
//	Velocity SpeedInAxes;
//	MovementDirection movementDir;
//	Acceleration AccelerationInAxes;
//public:
//	void startMotorsOrStopMotors(bool d, float initialSpeed);
//	void decreaseMotorSpeed();
//	void calculateAirDensity();
//	void updateAltitude(float deltaTime);
//	void takeof();
//	void land();
//	void IncreaseEngineSpeed();
//	void hover();
//	void setSpeed(Velocity newVelocity);
//	void updateOrientation()
//
//};
//
