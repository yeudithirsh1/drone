#include "Drone.h"
#include <iostream>
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min

using namespace std;

// קבועים
const float T0 = 288.15; // טמפרטורת פני הים 
const float P0 = 101325.0; // לחץ פני הים
const float L = 0.0065; // קצב ירידת טמפרטורה 
const float R = 287.05; // קבוע הגזים 
const float g = 9.80665; // תאוצת הכובד 



void Drone::startMotorsOrStopMotors(bool d, float initialSpeed)
{

    motor1.isActive = motor2.isActive = motor3.isActive = motor4.isActive = d;

    motor1.direction = -1;
    motor2.direction = 1;
    motor3.direction = -1;
    motor4.direction = 1;

    motor1.speed = motor2.speed = motor3.speed = motor4.speed = initialSpeed;
}

void Drone::IncreaseEngineSpeed()
{
    // הגברת מהירות המנועים בהדרגה עד למקסימום
    if (motor1.speed < 100)
    {
        motor1.speed += 5; // הגברת המהירות ב-5 כל פעם
        motor2.speed += 5;
        motor3.speed += 5;
        motor4.speed += 5;
    }
}

void Drone::calculateAirDensity()
{
    float T = T0 - L * dronePos.z;//חישוב הטמפרטורה בגובה טיסת הרחפן
	float P = P0 * pow((1 - (L * dronePos.z) / T0), (g / (R * L))); // חישוב הלחץ בגובה טיסת הרחפן

    float rho = P / (R * T); // צפיפות האוויר בגובה טיסת הרחפן

}

void Drone::updateAltitude(float deltaTime)
{
    float thrust = C_t * pow(rpm, 2);//חישוב כוח הרמה
    float drag = 0.5 * C_d * A * rho * pow(velocity, 2); // חישוב כוח הגרר
    float weight = mass * g;// חישוב משקל הרחפן
    
    float netForce = thrust - weight;// חישוב הכוח הנקי

    if (SpeedInAxes.vz > 0)
    {
        netForce -= drag; // הפחתת כוח הגרר אם הרחפן עולה
    }
    else
    {
		netForce += drag; // הוספת כוח הגרר אם הרחפן יורד
    }
    AccelerationInAxes.wz = netForce / mass;// חישוב תאוצה
    SpeedInAxes.vz += AccelerationInAxes.wz * deltaTime;//עדכון מהירות הרחפן
    dronePos.z += velocity * deltaTime;// עדכון גובה הרחפן    
    velocity = sqrt(SpeedInAxes.vx * SpeedInAxes.vx + SpeedInAxes.vy * SpeedInAxes.vy + SpeedInAxes.vz * SpeedInAxes.vz);
    acceleration = sqrt(AccelerationInAxes.wx * AccelerationInAxes.wx + AccelerationInAxes.wy * AccelerationInAxes.wy + AccelerationInAxes.wz * AccelerationInAxes.wz);
}

// פונקציה לעדכון גובה על פי כוח הרמה, משקל ותנגודת האוויר
void Drone::takeof()
{
    startMotorsOrStopMotors(true, 0);
    rpm = sqrt(mass*g / C_t);
    // סימולציה של המראה
    while (dronePos.z < target_altitude)
    {
        IncreaseEngineSpeed(); // הגברת מהירות המנועים
        calculateAirDensity(); // חישוב צפיפות האוויר
        updateAltitude(0.1);  // עדכון כל 0.1 שניות
        rpm += 10; // ככל שהרחפן גבוה יותר, מגדילים את RPM מעט
    }
}


void Drone::decreaseMotorSpeed()
{
    // הפחתת מהירות המנועים בהדרגה
    motor1.speed = max(motor1.speed - 5, 0); // הפחתת המהירות ב-5 כל פעם
    motor2.speed = max(motor2.speed - 5, 0);
    motor3.speed = max(motor3.speed - 5, 0);
    motor4.speed = max(motor4.speed - 5, 0);
}


void Drone::land()
{
    // סימולציה של נחיתה
    while (dronePos.z > 0.1)
    {
        rpm -= 10;
        decreaseMotorSpeed(); // הפחתת מהירות המנועים
        calculateAirDensity(); // חישוב צפיפות האוויר
        updateAltitude(1); // עדכון גובה כל 1 שניה
    }             
    startMotorsOrStopMotors(false, 0);
    velocity = 0;
    rpm = 0;
	dronePos.z = 0;
}



void Drone::hover()
{
    startMotorsOrStopMotors(true, 0);

    rpm = sqrt(mass * g / C_t);

    float hoverAltitude = dronePos.z;

    while (abs(dronePos.z - hoverAltitude) > 0.1 || abs(SpeedInAxes.vz) > 0.1)
    {
        calculateAirDensity();      // עדכון צפיפות האוויר
        updateAltitude(0.1);        // עדכון פיזיקלי של הגובה והמהירות

        // כיוונון RPM בהתאם לשינוי בגובה
        if (dronePos.z < hoverAltitude)
            rpm += 5;
        else if (dronePos.z > hoverAltitude)
            rpm -= 5;

        // Replace the problematic line with explicit usage of std::max and std::min  
        rpm = std::max(0.0f, std::min(rpm, maxRPM));
    }
}

void Drone::setSpeed(Velocity newVelocity) {
    SpeedInAxes = newVelocity;
    velocity = sqrt(newVelocity.vx * newVelocity.vx +
        newVelocity.vy * newVelocity.vy +
        newVelocity.vz * newVelocity.vz);
}


void adjustMotorsToRotate(float currentYaw, float targetYaw) {
    float yawError = targetYaw - currentYaw;

	motor1.speed = motor2.speed = motor3.speed = motor4.speed = maxRPM; // עצירת המנועים
    // חישוב התאמה למהירות מנועים לפי הזווית
    float adjustment = yawError * 2.0f; // מקדם פשוט, ניתן לשנות

    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // הגבלת ערכים
    if (adjustment > maxRPM / 2) adjustment = maxRPM / 2;
    if (adjustment < -maxRPM / 2) adjustment = -maxRPM / 2;

    if (yawError > 0) {
        // סיבוב ימינה
        motor1.speed -= adjustment;
        motor2.speed += adjustment;
        motor3.speed += adjustment;
        motor4.speed -= adjustment;
    }
    else {
        // סיבוב שמאלה
        motor1.speed += adjustment;
        motor2.speed -= adjustment;
        motor3.speed -= adjustment;
        motor4.speed += adjustment;
    }

    // maxRPM הגבלת מהירות מנועים בין 0 ל־ 
    Motor* motors[] = { &motor1, &motor2, &motor3, &motor4 };
    for (int i = 0; i < 4; i++) {
        if (motors[i]->speed > maxRPM) motors[i]->speed = maxRPM;
        if (motors[i]->speed < 0) motors[i]->speed = 0;
    }
}

void Drone::updateOrientation()
{

    float clockwiseThrust = motorSpeeds[0] + motorSpeeds[3];
    float counterClockwiseThrust = motorSpeeds[1] + motorSpeeds[2];

    // חשב את מהירות הסיבוב לפי ההפרש
    float yawRate = (counterClockwiseThrust - clockwiseThrust) * yawSensitivity;

    // עדכן את הזווית (בהנחה שהפונקציה קורית כל 0.01 שניות)
    DroneDirection.yaw += yawRate * 0.01f;

    // דאג להשאיר את הזווית בתחום [0, 360)
    if (DroneDirection.yaw >= 360.0f) DroneDirection.yaw -= 360.0f;
    if (DroneDirection.yaw < 0.0f) DroneDirection.yaw += 360.0f;
}


void Drone::rotate(float deltaYawDegrees)
{
    // חישוב זווית היעד
    float targetYaw = DroneDirection.yaw + deltaYawDegrees;

    // לולאה לסיבוב עד להגעה לזווית היעד
    while (abs(DroneDirection.yaw - targetYaw) > 0.5f)
    {
        // סובב לפי כיוון (ימינה/שמאלה)
        adjustMotorsToRotate(targetOrientation.yaw);

        // עדכון מצב הרחפן
        updateOrientation();

        // הדמיית זמן
        wait(0.01f);
    }

    // עצירת סיבוב
    stopRotation();
}
