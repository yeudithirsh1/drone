#pragma once
#include <shared_mutex>
#include <unordered_set>
#include <queue>

using namespace std;

extern bool reachedDestination;
extern shared_mutex mutexReachedDestination;

extern bool obstacleStatuse;
extern shared_mutex mutexObstacleStatus;

extern float targetAltitude;

extern shared_mutex soldiersPositionHistoryMutex;
extern queue<shared_ptr<unordered_set<string>>> soldiersPositionHistory;

bool getObstacleStatuse();
bool getReachedDestination(); 
queue<shared_ptr<unordered_set<string>>> getSoldiersPositionHistory();