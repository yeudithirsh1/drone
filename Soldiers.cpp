#include <iostream>
#include <filesystem>
#include <string>
#include <sstream>
#include <fstream> // Added to fix E0070: incomplete type "std::ifstream" is not allowed
#include <thread>
#include "Geohash.h"
#include "KalmanSoliders.h"
#include <shared_mutex>
#include "Global.h"

using namespace std;
namespace fs = filesystem;

shared_mutex geohashMapMutex; 
unordered_map<string, string> geohashMap;


//shared_lock<shared_mutex> lock();

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
    string oldGeohash;

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
            Vector2f estimatedLocation;

            if (!initialized) {
                kalmanSoliders.init(location);
                estimatedLocation = kalmanSoliders.getPosition();
                initialized = true;
            }
            else {
                kalmanSoliders.update(location);
                estimatedLocation = kalmanSoliders.getPosition();
            }

            string newGeohash = encodeGeohash(estimatedLocation.x(), estimatedLocation.y(), 12);

            {
                unique_lock<shared_mutex> lock(geohashMapMutex);
                if (oldGeohash.size()) {
                    geohashMap.erase(oldGeohash);
                }
                geohashMap[newGeohash] = id;
                oldGeohash = newGeohash;
            }
        }
        else {
            cerr << "שגיאה: לא ניתן לפתוח את הקובץ או שהוא ריק " << path << endl;
        }

        this_thread::sleep_for(chrono::milliseconds(200));
    }
}

void startTrackingAllSoldiers(string& path) {
    int count = countFilesInDirectory(path);
    vector<thread> threads;

    // יצירת עצמים מסוג kalmanSoliders, אחד לכל חייל
    vector<KalmanSoliders> kalmanList(count);

    for (int i = 0; i < count; i++) {
        string filePath = path + "/" + "soldiers_" + to_string(i) + ".txt";

        // שליחת הקובץ והאובייקט המתאים לפונקציה
        threads.emplace_back(updateGPSReadingsFromFile, ref(filePath), ref(kalmanList[i]));
    }
}