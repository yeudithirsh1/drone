#include "Sensors.h"

Sensors::Sensors(float distanceBetweenMeasurements, time_point timeMeasurements)
    : distanceBetweenMeasurements(distanceBetweenMeasurements), timeMeasurements(timeMeasurements){}

void Sensors::setDistanceBetweenMeasurements(float distanceBetweenMeasurements)
{
    this->distanceBetweenMeasurements = distanceBetweenMeasurements;
}

// מחזיר את המרחק בין מדידות
float Sensors::getDistanceBetweenMeasurements()
{
    return distanceBetweenMeasurements;
}

// קובע את זמן המדידה
void Sensors::setTimeMeasurements(time_point timeMeasurements)
{
    this->timeMeasurements = timeMeasurements;
}

// מחזיר את זמן המדידה
time_point Sensors::getTimeMeasurements()
{
    return timeMeasurements;
}

