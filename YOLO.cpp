#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <filesystem> 
#include <chrono>
#include <thread>
#include <shared_mutex>
#include "Global.h"


using namespace std;
namespace fs = filesystem;

struct YoloDetection {
    int classId;
    float bbox[4];
};

string getBaseFilename(const string& filepath) {
    return fs::path(filepath).stem().string(); // מחזיר את השם בלי הסיומת
}

string getLastCreatedFile(const string& directoryPath) {
    fs::path latestFile;
    fs::file_time_type latestTime;

    try {
        if (!fs::exists(directoryPath) || !fs::is_directory(directoryPath)) {
            std::cerr << "Directory does not exist: " << directoryPath << std::endl;
            return "";
        }

        for (const auto& entry : fs::directory_iterator(directoryPath)) {
            if (fs::is_regular_file(entry.status())) {
                auto ftime = fs::last_write_time(entry);

                if (latestFile.empty() || ftime > latestTime) {
                    latestTime = ftime;
                    latestFile = entry.path();
                }
            }
        }
        return latestFile.string();
    }
    catch (const std::exception& e) {
        cerr << "Error reading directory: " << e.what() << std::endl;
        return "";
    }
}

vector<YoloDetection> runYoloPrediction(const string& FilePath) {
    string pythonExe = "C:\\Users\\This User\\AppData\\Local\\Programs\\Python\\Python313\\python.exe";
    string scriptPath = "C:\\Users\\This User\\Documents\\project\\model\\movingObjects.py";

    string commandToExecute = "\"" + pythonExe + "\" \"" + scriptPath + "\" \"" + FilePath + "\"";

    string batFileName = "run_yolo_script.bat";

    ofstream batFile(batFileName);
    if (!batFile.is_open()) {
        cerr << "Error: Could not create temporary batch file '" << batFileName << "'" << endl;
        return {};
    }

    batFile << "@echo off" << endl;
    batFile << commandToExecute << endl;
    batFile << "exit" << endl;
    batFile.close();

    cout << "Created batch file '" << batFileName << "' with command:" << endl;
    cout << commandToExecute << endl;

    int result = system(batFileName.c_str());

    try {
        fs::remove(batFileName);
        cout << "Removed temporary batch file: " << batFileName << endl;
    }
    catch (const fs::filesystem_error& e) {
        cerr << "Error removing batch file '" << batFileName << "': " << e.what() << endl;
    }

    vector<YoloDetection> detections;

    if (result != 0) {
        cerr << "Error running Python script via batch file! System call returned: " << result << endl;
        return detections;
    }

    string baseName = getBaseFilename(FilePath);
    string outputFile = "C:\\Users\\This User\\Documents\\project\\model\\lable\\yolo_classes_" + baseName + ".txt";

    ifstream infile(outputFile);
    if (!infile) {
        cerr << "Could not open result file: " << outputFile << endl;
        return detections;
    }

    int cls;
    float x, y, w, h;
    while (infile >> cls >> x >> y >> w >> h) {
        YoloDetection det;
        det.classId = cls;
        det.bbox[0] = x;
        det.bbox[1] = y;
        det.bbox[2] = w;
        det.bbox[3] = h;
        detections.push_back(det);
    }
    infile.close();

    return detections;
}



void processDirectoryAndReadFiles() {
    string latest_file_str;

    while (true) {
        {
            shared_lock<shared_mutex> lock(mutexReachedDestination);
            if (reachedDestination) {
                return;
            }
        }
        latest_file_str = getLastCreatedFile("C:\\Users\\This User\\Documents\\project\\model\\source");

        bool valid_file = true;

        // בדיקה אם המחרוזת ריקה
        if (latest_file_str.empty()) {
            std::cerr << "No files found." << std::endl;
            valid_file = false;
        }

        fs::path latest_file_path(latest_file_str);

        if (valid_file) {
            // קוראים את תוכן הקובץ האחרון שנוצר
            vector<YoloDetection> detectedObjects = runYoloPrediction(latest_file_str);
        }
        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
