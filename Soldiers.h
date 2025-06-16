#pragma once

#include "KalmanSoliders.h"

int countFilesInDirectory(string& path);
void updateGPSReadingsFromFile(string& path, KalmanSoliders& kalmanSoliders);
void startTrackingAllSoldiers(string& path);
