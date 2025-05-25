#include "Sensors.h"

Sensors::Sensors(float distanceBetweenMeasurements, time_point timeMeasurements)
    : distanceBetweenMeasurements(distanceBetweenMeasurements), timeMeasurements(timeMeasurements){}

void Sensors::setDistanceBetweenMeasurements(float distanceBetweenMeasurements)
{
    this->distanceBetweenMeasurements = distanceBetweenMeasurements;
}

// ����� �� ����� ��� ������
float Sensors::getDistanceBetweenMeasurements()
{
    return distanceBetweenMeasurements;
}

// ���� �� ��� ������
void Sensors::setTimeMeasurements(time_point timeMeasurements)
{
    this->timeMeasurements = timeMeasurements;
}

// ����� �� ��� ������
time_point Sensors::getTimeMeasurements()
{
    return timeMeasurements;
}

