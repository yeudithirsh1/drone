#include <fstream>
#include <vector>
#include <string>
#include <filesystem> 
#include <chrono>
#include <thread>
#include <shared_mutex>
#include <deque>
#include <iostream>
#include "Global.h"
#include <sstream>
#include <unordered_map>
#include <functional>
#include "lidaeCameraSynchronization.h"
#include "PointInSpace.h"
#include "YOLO.h"
#include "imageToGpsConverter.h"
#include "Geohash.h"
#include "LIDAR.h"
#include <set>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef byte



using namespace std;
namespace fs = filesystem;

deque<string> YOLO_Processing_File;

string getLastCreatedFile(const string& directoryPath) {
    fs::path latestFile;
    filesystem::file_time_type latestTime;

    try {
        if (!fs::exists(directoryPath) || !fs::is_directory(directoryPath)) {
            cerr << "Directory does not exist: " << directoryPath << endl;
            return "";
        }

        for (const auto& entry : fs::directory_iterator(directoryPath)) {
            if (is_regular_file(entry)) {
                auto ftime = fs::last_write_time(entry);
                if (latestFile.empty() || ftime > latestTime) {
                    latestTime = ftime;
                    latestFile = entry.path();
                }
            }
        }

        return latestFile.empty() ? "" : latestFile.string();
    }
    catch (const exception& e) {
        cerr << "Error reading directory: " << e.what() << endl;
        return "";
    }
}       

void processDirectoryAndCopyFilesOnly() {
    string last_file;

    while (!getReachedDestination()) {
        string latest_file_str = getLastCreatedFile("C:\\Users\\This User\\Documents\\project\\model\\source");

        bool shouldSkip = latest_file_str.empty() || latest_file_str == last_file;

        if (!shouldSkip) {
            fs::path src(latest_file_str);

            // שם זמני בתיקיית to_process
            string temp_filename = src.filename().string() + ".tmp";
            fs::path dst_tmp = "C:\\Users\\This User\\Documents\\project\\model\\to_process\\" + temp_filename;

            // שם סופי
            fs::path dst_final = "C:\\Users\\This User\\Documents\\project\\model\\to_process\\" + src.filename().string();

            try {
                fs::copy_file(src, dst_tmp, fs::copy_options::overwrite_existing);
                cout << "Copied to temporary file: " << dst_tmp << endl;

                fs::rename(dst_tmp, dst_final);
                cout << "Renamed " << dst_tmp << " to " << dst_final << endl;

                last_file = latest_file_str;
                YOLO_Processing_File.push_back(dst_final.string());
            }
            catch (const fs::filesystem_error& e) {
                cerr << "Error copying or renaming file: " << e.what() << endl;
            }
        }

        this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}



bool runPythonScript() {
    string pythonExe = "C:\\Users\\This User\\AppData\\Local\\Programs\\Python\\Python313\\python.exe";
    string scriptPath = "C:\\Users\\This User\\Documents\\project\\model\\movingObjects.py";
    string commandToExecute = "\"" + pythonExe + "\" \"" + scriptPath + "\"";
    string batFileName = "run_yolo_script.bat";

    // יצירת קובץ BAT
    ofstream batFile(batFileName);
    if (!batFile.is_open()) {
        cerr << "Error: Could not create batch file '" << batFileName << "'" << endl;
        return false;
    }

    batFile << "@echo off\n";
    batFile << "chcp 65001 >nul\n"; // קידוד UTF-8 למניעת בעיות טקסט
    batFile << commandToExecute << "\n";
    batFile << "exit /B %ERRORLEVEL%\n";
    batFile.close();

    cout << "Running YOLO Python script:\n" << commandToExecute << endl;

    // הרצת הפקודה
    int result = system(batFileName.c_str());

    // מחיקת קובץ ה־BAT
    try {
        fs::remove(batFileName);
    }
    catch (const fs::filesystem_error& e) {
        cerr << "Error deleting batch file: " << e.what() << endl;
    }

    if (result != 0) {
        cerr << "Python script failed with code: " << result << endl;
        return false;
    }

    return true;
}


struct Detection {
    int classId;
    float x1, y1, x2, y2;
};



void handlePerson(const string& calibration_file, BoundingBox boundingBox, LIDAR& lidar, Drone& drone) {

    vector<Point> cloud = lidar.getQ();
    vector<cv::Point3f> points3f;
    points3f.reserve(cloud.size());

    for (const auto& pt : cloud) {
        points3f.emplace_back(cv::Point3f(pt.x, pt.y, pt.z));
    }

    float distance = Lidar_to_camera_ratio(calibration_file, points3f, boundingBox).avg_distance;
    CameraCalibration cameraCalibration;
    cameraCalibration = loadCalibrationFromFile(calibration_file);
    pair<float, float> coords = yoloToGPS(boundingBox.x_center, boundingBox.y_center, distance, calibration_file, drone);
    string str = encodeGeohash(coords.first, coords.second, 12);



}

void printTerroristDetectedMessage() {
    cout << "זוהה מחבל" << endl;
}

bool birdDetected = false;


void handleBird(BoundingBox boundingBox) {
    cout << ">> ציפור זוהתה! טיפול חד פעמי." << endl;
}

void handleDrone() {
    cout << ">> רחפן זוהה - תהליך עתידי." << endl;
}

unordered_map<int, function<void(BoundingBox)>> classHandlers = {
    {0, [](BoundingBox boundingBox) {
        thread(handlePerson, boundingBox);
    }},
    {1, [](BoundingBox boundingBox) {
        if (!birdDetected) {
            birdDetected = true;
            thread(handleBird, boundingBox);
        }
    }},
    {2, [](BoundingBox boundingBox) {
        thread(handleDrone).detach();
    }}
};


void processDetectionFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "שגיאה בפתיחת קובץ: " << filename << endl;
        return;
    }

    vector<Detection> currentDetections;
    string line;
    Detection det;

    while (getline(file, line)) {
        istringstream iss(line);
        if (!(iss >> det.classId >> det.x1 >> det.y1 >> det.x2 >> det.y2)) {
            cerr << "שורה לא תקינה: " << line << endl;
            return;
        }
        currentDetections.push_back(det);
    }

    bool birdFoundThisRound = false;

    for (const auto& det1 : currentDetections) {
        BoundingBox boundingBox = { det1.x1, det1.y1, det1.x2, det1.y2 };

        if (classHandlers.count(det1.classId)) {
            if (det1.classId == 1) birdFoundThisRound = true;
            classHandlers[det1.classId](boundingBox);
        }
    }

    if (!birdFoundThisRound) {
        birdDetected = false;
    }
}


bool convertWCharToChar(const wchar_t* wideStr, char* buffer, size_t bufferSize) {
    if (!wideStr || !buffer || bufferSize == 0) {
        return false;
    }

    size_t convertedChars = 0;
    errno_t err = wcstombs_s(&convertedChars, buffer, bufferSize, wideStr, _TRUNCATE);

    if (err != 0) {
        std::cerr << "שגיאה בהמרת מחרוזת רחבה למחרוזת רגילה" << std::endl;
        return false;
    }

    return true;
}




































//void runYoloScript(const string& imagePath) {
//    string pythonExe = "C:\\Users\\This User\\AppData\\Local\\Programs\\Python\\Python313\\python.exe";
//    string scriptPath = "C:\\Users\\This User\\Documents\\project\\model\\movingObjects.py";
//
//    string command = "\"" + pythonExe + "\" \"" + scriptPath + "\" \"" + imagePath + "\"";
//    string batFileName = "run_yolo_script.bat";
//
//    ofstream batFile(batFileName);
//    if (!batFile.is_open()) {
//        std::cerr << "Error: Could not create batch file." << std::endl;
//        return;
//    }
//
//    batFile << "@echo off" << endl;
//    batFile << command << endl;
//    batFile << "exit" << endl;
//    batFile.close();
//
//    int result = std::system(batFileName.c_str());
//    if (result != 0) {
//        std::cerr << "Error: Script execution failed. Exit code: " << result << std::endl;
//    }
//
//    remove(batFileName.c_str());
//}
//
//int main() {
//    string imagePath = "C:\\Users\\This User\\Documents\\project\\model\\source\\Frame_00005.jpg";
//    runYoloScript(imagePath);
//    return 0;
//}
