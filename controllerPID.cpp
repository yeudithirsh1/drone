#include "controllerPID.h"

//חישוב האינטגרל של השגיאה - סכום מצטבר של השגיאה לאורך זמן
//חישוב הנגזרת של השגיאה - השינוי בשגיאה בין שני זמנים כדי לדעת כמה הטעות משתנה לאורך זמן
//עדכון השגיאה הקודמת
//חישוב הפלט של ה-PID לפי הנוסחה: P + I + D

float PID::update(float error, float dt) {
	integral += error * dt;
	float derivative = (error - prevError) / dt;
	prevError = error;
	return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
    prevError = 0;
    integral = 0;
}
