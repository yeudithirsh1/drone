#include "Global.h"
#include <shared_mutex>
#include <queue>
#include <unordered_set>

bool reachedDestination = false;
shared_mutex mutexReachedDestination;

bool obstacleStatuse = false;                   
shared_mutex mutexObstacleStatus;  

float targetAltitude = 0.0f;

shared_mutex soldiersPositionHistoryMutex;
queue<shared_ptr<unordered_set<string>>> soldiersPositionHistory;


queue<shared_ptr<unordered_set<string>>> getSoldiersPositionHistory() {
	shared_lock<shared_mutex> lock(soldiersPositionHistoryMutex);
	return soldiersPositionHistory;
}


bool getObstacleStatuse() {
	shared_lock<shared_mutex> lock(mutexObstacleStatus);
	return obstacleStatuse;
}

bool getReachedDestination() {
	shared_lock<shared_mutex> lock(mutexReachedDestination);
	return reachedDestination;
}

