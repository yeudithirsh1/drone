#include "Global.h"
#include <shared_mutex>

bool reachedDestination = false;
shared_mutex mutexReachedDestination;

bool obstacleStatuse = false;                   
shared_mutex mutexObstacleStatus;               

bool getObstacleStatuse() {
	shared_lock<shared_mutex> lock(mutexObstacleStatus);
	return obstacleStatuse;
}
