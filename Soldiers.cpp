#include <iostream>
#include <filesystem>
#include <string>
#include <sstream>
#include <fstream>
#include <thread>
#include <unordered_set>
#include "Geohash.h"
#include "KalmanSoliders.h"
#include <shared_mutex>
#include "Global.h"
#include <queue>

using namespace std;
namespace fs = filesystem;
using namespace chrono;



void soldierLocationUpdate(string& geohash)
{
    if (getSoldiersPositionHistory().empty()) {
        return;
    }
    auto lastSetPtr = soldiersPositionHistory.back();
    lastSetPtr->insert(geohash);
}

int countFilesInDirectory(string& path) {
    int count = 0;
    try {
        for (const auto& entry : fs::directory_iterator(path)) {
            if (fs::is_regular_file(entry.status())) {
                count++;
            }
        }
    }
    catch (const fs::filesystem_error& e) {
        cerr << "שגיאת מערכת קבצים: " << e.what() << endl;
    }
    return count;
}

void updateGPSReadingsFromFile(string& path, KalmanSoliders& kalmanSoliders)
{
    bool initialized = false;
    while (true)
    {
        {
            shared_lock<shared_mutex> lock(mutexReachedDestination);
            if (reachedDestination) {
                return;
            }
        }

        ifstream inputFile(path);
        vector<string> lines;

        if (inputFile.is_open()) {
            string line;
            while (getline(inputFile, line)) {
                if (!line.empty())
                    lines.push_back(line);
            }
            inputFile.close();
        }

        if (inputFile.is_open() && !lines.empty()) {
            string lastLine = lines.back();

            float lat, lon;
            string id;	

            istringstream iss(lastLine);
            iss >> lat >> lon >> id;


            Vector2f location = { lat, lon };

            if (!initialized) {
                kalmanSoliders.init(location);
                initialized = true;
            }
            else {
                kalmanSoliders.update(location);
            }

        }
        else {
            cerr << "שגיאה: לא ניתן לפתוח את הקובץ או שהוא ריק " << path << endl;
        }

        this_thread::sleep_for(milliseconds(200));
    }
}

void snapshotLoop()
{
    while (!getReachedDestination())
    {
        {
            unique_lock<shared_mutex> lock(soldiersPositionHistoryMutex);
            auto newSetPtr = make_shared<unordered_set<string>>();
            soldiersPositionHistory.push(newSetPtr);
        }

        this_thread::sleep_for(chrono::milliseconds(100)); // או פרק הזמן שלך
    }
}


void startTrackingAllSoldiers(string& path) {
    int count = countFilesInDirectory(path);
    vector<thread> threads;

    threads.emplace_back(snapshotLoop);

    // יצירת עצמים מסוג kalmanSoliders, אחד לכל חייל
    vector<KalmanSoliders> kalmanList(count);

    for (int i = 0; i < count; i++) {
        string filePath = path + "/" + "soldiers_" + to_string(i) + ".txt";

        // שליחת הקובץ והאובייקט המתאים לפונקציה
        threads.emplace_back(updateGPSReadingsFromFile, ref(filePath), ref(kalmanList[i]));
    }
}