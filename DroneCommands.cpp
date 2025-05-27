#include "DroneFeatures.h"
#include <iostream>
#include <cmath>
#include <algorithm> // Add this include for std::max and std::min
#include <windows.h>

using namespace std;

// קבועים
const float T0 = 288.15; // טמפרטורת פני הים 
const float P0 = 101325.0; // לחץ פני הים
const float L = 0.0065; // קצב ירידת טמפרטורה 
const float R = 287.05; // קבוע הגזים 
const float g = 9.80665; // תאוצת הכובד 

void startMotorsOrStopMotors(Drone drone, bool d, float initialSpeed)
{
	drone.setMotor1(Motor(true, initialSpeed, -1));
	drone.setMotor2(Motor(true, initialSpeed, 1));
	drone.setMotor3(Motor(true, initialSpeed, -1));
	drone.setMotor4(Motor(true, initialSpeed, 1));
}

void IncreaseEngineSpeed(Drone drone)
{
    // הגברת מהירות המנועים בהדרגה עד למקסימום
    if (drone.getMotor1().speed < 100)
    {
        Motor motor1 = drone.getMotor1();
        motor1.speed = drone.getMotor1().speed + 5.0f; // אתחול מהירות המנועים
		drone.setMotor1(motor1);
		Motor motor2 = drone.getMotor2();
		motor2.speed = drone.getMotor2().speed + 5.0f; // אתחול מהירות המנועים
		drone.setMotor2(motor2);
		Motor motor3 = drone.getMotor3();
		motor3.speed = drone.getMotor3().speed + 5.0f; // אתחול מהירות המנועים
		drone.setMotor3(motor3);
		Motor motor4 = drone.getMotor4();
		motor4.speed = drone.getMotor4().speed + 5.0f; // אתחול מהירות המנועים
		drone.setMotor4(motor4);
    }
}

void calculateAirDensity(Drone drone)
{
    float T = T0 - L * drone.getDronePos().z;//חישוב הטמפרטורה בגובה טיסת הרחפן
    float P = P0 * pow((1 - (L * drone.getDronePos().z) / T0), (g / (R * L))); // חישוב הלחץ בגובה טיסת הרחפן
    drone.setRho(P / (R * T)); // צפיפות האוויר בגובה טיסת הרחפן

}

void updateAltitude(Drone drone, float deltaTime)
{
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);//חישוב כוח הרמה
    float drag = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * pow(drone.getVelocity(), 2); // חישוב כוח הגרר
    float weight = drone.getMass() * g;// חישוב משקל הרחפן

    float netForce = thrust - weight;// חישוב הכוח הנקי

    if (drone.getSpeedInAxes().vz > 0)
    {
        netForce -= drag; // הפחתת כוח הגרר אם הרחפן עולה
    }
    else
    {
        netForce += drag; // הוספת כוח הגרר אם הרחפן יורד
    }
	Acceleration Acceleration = drone.getAccelerationInAxes();
    Acceleration.az = netForce / drone.getMass();
    drone.setAccelerationInAxes(Acceleration);// חישוב תאוצה
	Velocity SpeedInAxes = drone.getSpeedInAxes();
	SpeedInAxes.vz += drone.getAccelerationInAxes().az * deltaTime; // עדכון מהירות הרחפן
    drone.setSpeedInAxes(SpeedInAxes);
	Point dronePos = drone.getDronePos();
	dronePos.z += drone.getVelocity() * deltaTime; // עדכון מיקום הרחפן בציר z
	drone.setDronePos(dronePos); // עדכון גובה הרחפן
    drone.setVelocity(sqrt(SpeedInAxes.vx * SpeedInAxes.vx + SpeedInAxes.vy * SpeedInAxes.vy + SpeedInAxes.vz * SpeedInAxes.vz));
    drone.setAcceleration(sqrt(Acceleration.ax * Acceleration.ax + Acceleration.ay * Acceleration.ay + Acceleration.az * Acceleration.az));
}

// פונקציה לעדכון גובה על פי כוח הרמה, משקל ותנגודת האוויר
void takeof(Drone drone)
{
    startMotorsOrStopMotors(drone, true, 0);
    drone.setRpm(sqrt(drone.getMass() * g / drone.getC_t()));
    // סימולציה של המראה
    while (drone.getDronePos().z < drone.getTargetAltitude())
    {
        IncreaseEngineSpeed(drone); // הגברת מהירות המנועים
        calculateAirDensity(drone); // חישוב צפיפות האוויר
        updateAltitude(drone, 0.1);  // עדכון כל 0.1 שניות
        drone.setRpm(drone.getRpm() + 10);// ככל שהרחפן גבוה יותר, מגדילים את RPM מעט
    }
}


void decreaseMotorSpeed(Drone drone)
{
    // הפחתת מהירות המנועים בהדרגה
	Motor motor1 = drone.getMotor1();
    motor1.speed = max(motor1.speed - 5.0f, 0.0f); // הפחתת המהירות ב-5 כל פעם
    drone.setMotor1(motor1);
    Motor motor2 = drone.getMotor2();
	motor2.speed = max(motor2.speed - 5.0f, 0.0f); // הפחתת המהירות ב-5 כל פעם
	drone.setMotor2(motor2);
	Motor motor3 = drone.getMotor3();
    motor3.speed = max(motor3.speed - 5.0f, 0.0f);
    drone.setMotor3(motor3);
	Motor motor4 = drone.getMotor4();
    motor4.speed = max(motor4.speed - 5.0f, 0.0f);
    drone.setMotor4(motor4);
}


void land(Drone drone)
{
    // סימולציה של נחיתה
    while (drone.getDronePos().z > 0.1)
    {
        drone.setRpm(drone.getRpm() - 10);// ככל שהרחפן גבוה יותר, מפחיתים את RPM מעט
        decreaseMotorSpeed(drone); // הפחתת מהירות המנועים
        calculateAirDensity(drone); // חישוב צפיפות האוויר
        updateAltitude(drone, 1); // עדכון גובה כל 1 שניה
    }
    startMotorsOrStopMotors(drone, false, 0);
    drone.setVelocity(0);
    drone.setRpm(0);
    Point point = drone.getDronePos();
    point.z = 0;
    drone.setDronePos(point);
}


float hover(Drone drone)
{
    startMotorsOrStopMotors(drone, true, 0);

    drone.setRpm(sqrt(drone.getMass() * g / drone.getC_t()));
    float hoverAltitude = drone.getDronePos().z;

    while (abs(drone.getDronePos().z - hoverAltitude) > 0.1 || abs(drone.getSpeedInAxes().vz) > 0.1)
    {
        calculateAirDensity(drone);      // עדכון צפיפות האוויר
        updateAltitude(drone, 0.1);        // עדכון פיזיקלי של הגובה והמהירות

        // כיוונון RPM בהתאם לשינוי בגובה
        if (drone.getDronePos().z < hoverAltitude)
            drone.setRpm(drone.getRpm() + 5);
        else if (drone.getDronePos().z > hoverAltitude)
            drone.setRpm(drone.getRpm() - 5);

        // Replace the problematic line with explicit usage of std::max and std::min  
        drone.setRpm(max(0.0f, min(drone.getRpm(), drone.getMaxRPM())));
    }
    return drone.getRpm();
}

void setSpeed(Drone drone, Velocity newVelocity) {
    drone.setSpeedInAxes(newVelocity);
    drone.setVelocity(sqrt(newVelocity.vx * newVelocity.vx +
        newVelocity.vy * newVelocity.vy +
        newVelocity.vz * newVelocity.vz));
}


void adjustMotorsToRotate(Drone drone, float currentYaw, float targetYaw) {
    float yawError = targetYaw - currentYaw;

    float balancingSpeed = hover(drone);

    Motor motor1 = drone.getMotor1();
    motor1.speed = balancingSpeed; // הפחתת המהירות ב-5 כל פעם
    drone.setMotor1(motor1);
    Motor motor2 = drone.getMotor2();
    motor2.speed = balancingSpeed; // הפחתת המהירות ב-5 כל פעם
    drone.setMotor2(motor2);
    Motor motor3 = drone.getMotor3();
    motor3.speed = balancingSpeed;
    drone.setMotor3(motor3);
    Motor motor4 = drone.getMotor4();
    motor4.speed = balancingSpeed;
    drone.setMotor4(motor4);
    // חישוב התאמה למהירות מנועים לפי הזווית
    float adjustment = yawError * 2.0f; // מקדם פשוט, ניתן לשנות

    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // הגבלת ערכים
    if (adjustment > drone.getMaxRPM() / 2) adjustment = drone.getMaxRPM() / 2;
    if (adjustment < -drone.getMaxRPM() / 2) adjustment = -drone.getMaxRPM() / 2;

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
        if (motors[i]->speed > drone.getMaxRPM()) motors[i]->speed = drone.getMaxRPM();
        if (motors[i]->speed < 0) motors[i]->speed = 0;
    }
}


void updateOrientation(Drone drone, float dt)
{
    float clockwiseThrust = drone.getMotor1().speed + drone.getMotor2().speed;
    float counterClockwiseThrust = drone.getMotor3().speed + drone.getMotor4().speed;

    // חשב את מהירות הסיבוב לפי ההפרש
    drone.setYawRate(counterClockwiseThrust - clockwiseThrust * drone.getYawRate());

    // עדכן את הזווית (בהנחה שהפונקציה קורית כל 0.01 שניות)
    drone.setYaw(drone.getYaw() + drone.getYawRate() * dt);

    // דאג להשאיר את הזווית בתחום [0, 360)
    if (drone.getYaw() >= 360.0f) drone.setYaw(drone.getYaw() - 360.0f);
    if (drone.getYaw() < 0.0f) drone.setYaw(drone.getYaw() + 360.0f);
}


void wait(float seconds) {
    Sleep(static_cast<DWORD>(seconds * 1000)); // המרה למילישניות
}


void rotate(Drone drone, float deltaYawDegrees, float dt)
{
    // חישוב זווית היעד
    float targetYaw = drone.getYaw() + deltaYawDegrees;

    // לולאה לסיבוב עד להגעה לזווית היעד
    while (abs(drone.getYaw() - targetYaw) > 0.5f)
    {
        // סובב לפי כיוון (ימינה/שמאלה)
        adjustMotorsToRotate(drone, deltaYawDegrees, drone.getYaw());

        // עדכון מצב הרחפן
        updateOrientation(drone, dt);

        // הדמיית זמן
        wait(0.01f);
    }
}

void updateForwardMotion(Drone drone, float deltaTime)
{
    //  חישוב thrust כולל
    float thrust = drone.getC_t() * pow(drone.getRpm(), 2);

    //  זוויות
    float pitch = drone.getPitch(); // ברדיאנים
    float yaw = drone.getYaw();     // ברדיאנים

    //  כוח thrust קדימה לפי pitch
    float forwardThrust = thrust * sin(pitch);

    //  תרגום ל־X ו־Y לפי yaw (מערכת גלובלית)
    float fx = forwardThrust * cos(yaw); // רכיב thrust בציר X
    float fy = forwardThrust * sin(yaw); // רכיב thrust בציר Y

    //  חישוב גרר בציר X ו־Y
    Velocity speed = drone.getSpeedInAxes();
    float dragX = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vx * speed.vx;
    float dragY = 0.5 * drone.getC_d() * drone.getA() * drone.getRho() * speed.vy * speed.vy;

    // כיוון הגרר הפוך לכיוון התנועה
    dragX *= (speed.vx > 0) ? -1 : 1;
    dragY *= (speed.vy > 0) ? -1 : 1;

    //  כוח נטו
    float netFx = fx + dragX;
    float netFy = fy + dragY;

    //  חישוב תאוצה
    Acceleration acceleration = drone.getAccelerationInAxes();
    acceleration.ax = netFx / drone.getMass();
    acceleration.ay = netFy / drone.getMass();
    drone.setAccelerationInAxes(acceleration);

    //  עדכון מהירות
    speed.vx += acceleration.ax * deltaTime;
    speed.vy += acceleration.ay * deltaTime;
    drone.setSpeedInAxes(speed);

    //  עדכון מיקום
    Point pos = drone.getDronePos();
    pos.x += speed.vx * deltaTime;
    pos.y += speed.vy * deltaTime;
    drone.setDronePos(pos);

    //  עדכון כוללים
    float vTotal = sqrt(speed.vx * speed.vx + speed.vy * speed.vy + speed.vz * speed.vz);
    float aTotal = sqrt(acceleration.ax * acceleration.ax + acceleration.ay * acceleration.ay + acceleration.az * acceleration.az);
    drone.setVelocity(vTotal);
    drone.setAcceleration(aTotal);
}

void moveForward(Drone drone, float durationSeconds, float pitchAngleDegrees, float dt)
{
    // שמירה על הגובה ההתחלתי
    float targetAltitude = drone.getDronePos().z;

    // הטיית הרחפן קדימה
    drone.setPitch(pitchAngleDegrees);

    float elapsedTime = 0.0f;

    while (elapsedTime < durationSeconds)
    {
        // חישוב צפיפות אוויר אם יש צורך
        calculateAirDensity(drone);

		updateForwardMotion(drone, dt); 

        wait(dt);
        elapsedTime += dt;
    }

    // החזרת pitch ל-0 כדי לעצור את ההתקדמות
    drone.setPitch(0);
}
