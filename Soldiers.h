#pragma once  

#include <queue>  
#include <unordered_set>
#include <string>  
#include "KalmanSoliders.h"

using namespace std;

queue<shared_ptr<unordered_set<string>>> getSoldiersPositionHistory();
void soldierLocationUpdate(string& geohash);
int countFilesInDirectory(string& path);
void updateGPSReadingsFromFile(string& path, KalmanSoliders& kalmanSoliders);
void startTrackingAllSoldiers(string& path);
