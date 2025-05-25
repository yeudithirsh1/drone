#include "GPS.h"  
#include <fstream>  
#include <sstream>  
#include <iostream>  
#include <thread>  
#include <chrono> 

using namespace std;

void GPS::UpdatePossion()
{  
   ifstream file("src/GPS.txt");  
   double pos1, pos2, pos3;  
   try  
   {  
       string line;  
       while (std::getline(file, line))  
       {  
           istringstream iss(line);  
           iss >> pos1 >> pos2 >> pos3;  
		   GPSLocation.z = pos1;
		   GPSLocation.x = pos2;
		   GPSLocation.y = pos3;
           //מסנן קלמן
           this_thread::sleep_for(std::chrono::seconds(2));  
       }  
       file.close();  
   }  
   catch (const std::exception& error)  
   {  
       std::cout << "Error: " << error.what() << std::endl;  
   }  
}