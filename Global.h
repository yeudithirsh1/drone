#pragma once
#include <shared_mutex>
using namespace std;

extern bool reachedDestination;
extern shared_mutex mutexReachedDestination;

extern bool obstacleStatuse;                    
extern shared_mutex mutexObstacleStatus;        

bool getObstacleStatuse();
