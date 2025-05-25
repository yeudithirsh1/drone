#include "IMU.h"  
#include <string>  
#include <fstream> // Include this header for std::ifstream  
#include <sstream>  
#include <cmath>  
#include <iostream>  
#include <thread>  
#include <chrono>
#include "DroneFeatures.h"


using namespace std;  
IMU::IMU()
    : Sensors(0.0f, chrono::high_resolution_clock::now()), // קריאה לבנאי של Sensors
    acceleration(0.0f, 0.0f, 0.01f)
{
}

void IMU::calculateSpeed()  
{  
   string line;

   while (1)  
   {  
       ifstream inputFile("src/IMUsensor.txt"); // Open the text file for reading  
       if (!inputFile.is_open())  
       {  
           cerr << "Error opening file." << endl;  
       }  
       getline(inputFile, line);
       istringstream iss(line);  
       iss >> acceleration.ax >> acceleration.ay >> acceleration.az >> yawRate; // Ensure valid input extraction  
       inputFile.close(); // Close the file  
       this_thread::sleep_for(chrono::seconds(1)); // Wait for 1 second between iterations  
   }  
} 
