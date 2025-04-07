#include "drone_commands.h"
#include <iostream>
#include <cmath> // עבור חישובים מתקדמים
using namespace std;

// קבועים
const float g = 9.81;  // תאוצת הכובד (מ/ש²)

void startMotorsOrStopMotors(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4,bool d, float initialSpeed)
{

    motor1.isActive = d;
    motor2.isActive = d;
    motor3.isActive = d;
    motor4.isActive = d;

    motor1.direction = -1; 
	motor2.direction = 1;
	motor3.direction = -1;
	motor4.direction = 1;

    motor1.speed = initialSpeed;
    motor2.speed = initialSpeed;
    motor3.speed = initialSpeed;
    motor4.speed = initialSpeed;
}

void increaseMotorSpeed(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4) 
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

float calculateAirDensity(float altitude_m) 
{
    float T0 = 288.15;       // טמפרטורת פני הים (קלווין)
    float P0 = 101325.0;     // לחץ פני הים (פסקל)
    float L = 0.0065;        // קצב ירידת טמפרטורה (K/m)
    float R = 287.05;        // קבוע הגזים (J/kg*K)
    float g = 9.80665;       // תאוצת הכובד (m/s^2)

    float T = T0 - L * altitude_m;
    float P = P0 * pow((1 - (L * altitude_m) / T0), (g / (R * L)));

    float rho = P / (R * T); // צפיפות האוויר

    return rho;
}


// פונקציה לעדכון גובה על פי כוח הרמה, משקל ותנגודת האוויר
void updateAltitude(float& altitude, float rpm, float deltaTime, float mass, float& velocity, float rho, float A, float C_d, float C_t)
{
	float thrust = C_t * pow(rpm, 2);//חישוב כוח הרמה
	float drag = 0.5 * C_d * A * rho * pow(velocity, 2); // חישוב כוח הגרר
	float weight = mass * g;// חישוב משקל הרחפן

    if (thrust > weight) 
    {
		float netForce = thrust - weight - drag;// חישוב הכוח הנקי
		float acceleration = netForce / mass;// חישוב תאוצה
		velocity += acceleration * deltaTime;//עדכון מהירות הרחפן
		altitude += velocity * deltaTime;// עדכון גובה הרחפן
    }
    else 
    {
        std::cout << "The drone cannot take off. Thrust is not enough." << std::endl;
    }
}


void takeof(float mass, float targetAltitude, float A, float C_d, float C_t) {

    Motor motor1;//קדמי שמאלי
	Motor motor2;//קדמי ימני
	Motor motor3; //אחורי שמאלי
	Motor motor4; //אחורי ימני
    
    float altitude = 0; // גןבה נוכחי של הרחפן
    float rpm = 5000;  // סיבובי המנועים בדקה (RPM)
    float velocity = 0.0;
    float rho;
    // הפעלת המנועים
    startMotorsOrStopMotors(motor1, motor2, motor3, motor4, true, 0);

    // סימולציה של המראה
    while (altitude < targetAltitude) 
    {
        increaseMotorSpeed(motor1, motor2, motor3, motor4); // הגברת מהירות המנועים
        rho = calculateAirDensity(altitude); // חישוב צפיפות האוויר
        updateAltitude(altitude, mass, rpm, 1, velocity, rho, A, C_d, C_t);  // עדכון כל 0.1 שניות
        rpm = 5000 + (altitude / 10); // ככל שהרחפן גבוה יותר, ייתכן שתרצה להגדיל RPM מעט
    }
}

void decreaseMotorSpeed(Motor& motor1, Motor& motor2, Motor& motor3, Motor& motor4)
{
    // הפחתת מהירות המנועים בהדרגה
    motor1.speed = max(motor1.speed - 5, 0); // הפחתת המהירות ב-5 כל פעם
    motor2.speed = max(motor2.speed - 5, 0);
    motor3.speed = max(motor3.speed - 5, 0);
    motor4.speed = max(motor4.speed - 5, 0);
    
}


void land(float mass, float targetAltitude, float A, float C_d, float C_t)
{
    Motor motor1; // קדמי שמאלי
    Motor motor2; // קדמי ימני
    Motor motor3; // אחורי שמאלי
    Motor motor4; // אחורי ימני

    float altitude = targetAltitude; // גובה נוכחי של הרחפן (בהתחלה יהיה שווה לגובה המטרה)
    float rpm = 5000;  // סיבובי המנועים בהתחלה (RPM)
    float velocity = 0.0; // מהירות רחפן
    float rho;

    // הפעלת המנועים
    startMotorsOrStopMotors(motor1, motor2, motor3, motor4, true, rpm / 50.0f);

    // סימולציה של נחיתה
    while (altitude > 0)
    {
        decreaseMotorSpeed(motor1, motor2, motor3, motor4); // הפחתת מהירות המנועים
        rho = calculateAirDensity(altitude); // חישוב צפיפות האוויר
        updateAltitude(altitude, rpm, 1, mass, velocity, rho, A, C_d, C_t); // עדכון גובה כל 1 שניה

        // הפחתת RPM בהדרגה ככל שהרחפן מתקרב לקרקע
        rpm = max(1000.0f, rpm - (altitude / 10)); // אל תאפשר ל-RPM לרדת מתחת ל-1000

        // שמירה על מהירות שלילית (הנחה)
        if (velocity < 0) 
        {
            velocity = 0;
            startMotorsOrStopMotors(motor1, motor2, motor3, motor4, false, 0);

        }
    }

    cout << "The drone has landed successfully." << std::endl;
}
