//#include <string>  
//#include <filesystem>  
//#include <thread>  
//#include <chrono> 
//#include <fstream> // Include this header to resolve incomplete type "std::ifstream"
//#include <iostream>
//#include "lidarCameraSynchronization.cpp"
//using namespace std;
//namespace fs = std::filesystem;
//using namespace fs;
//
//typedef void (*FuncPtr)(BoundingBox);
//FuncPtr functions[3] = {};
//bool arrState[3]{ false };
//
//
//string getLastCreatedFolder(const string& folderPath) {
//    path latestFolder;
//    file_time_type latestTime;
//
//    for (const auto& entry : directory_iterator(folderPath)) {
//        if (is_directory(entry)) {
//            auto ftime = last_write_time(entry);
//            if (latestFolder.empty() || ftime > latestTime) {
//                latestTime = ftime;
//                latestFolder = entry.path();
//            }
//        }
//    }
//
//    return latestFolder.string();
//}
//
//void UpdateStateFromYolo()  
//{  
//    string folderPath = "C:/Users/USER/Documents/project/final_modal/yolov5/runs/detect";  
//    string lastCreatedFolder = getLastCreatedFolder(folderPath) + "/labels";  
//    onyolo = true;  
//    while (onyolo)  
//    {  
//        for (const auto& entry : filesystem::directory_iterator(lastCreatedFolder))
//        {  
//            if (entry.path().extension() == ".txt")  
//                processFile(entry.path().string());  
//        }  
//        // Wait for 1 second before checking again  
//        this_thread::sleep_for(chrono::seconds(1));  
//    }  
//}
//
//
//void processFile(const string& filePath)
//{
//  ifstream inputFile(filePath);
//  int state, placeinHashFunctionDrivingScenarios;
//  if (!inputFile)
//  {
//      cerr << "Error opening the file: " << filePath << "\n";
//      return;
//  }
//  string word;
//  while (inputFile >> word)
//  {
//    state = stoi(word);
//    if (arrState[state] != true)
//    {
//        functions[state]();
//        thread t(&DrivingScenarios::PlayHashFunctionDrivingScenarios, this, state);
//        t.detach();
//    }
//    // Ignore the rest of the line
//    inputFile.ignore(numeric_limits<streamsize>::max(), '\n');
//  }
//  inputFile.close(); // Close the file after processing
//
//  // Close the file and then delete it
//  remove(filePath.c_str());
//  cout << "File " << filePath << " processed and deleted. \n";
//}