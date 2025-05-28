#include "GPS.h"  
#include <fstream>  
#include <sstream>  
#include <iostream>  
#include <thread>  
#include <chrono> 

using namespace std;
const double EARTH_RADIUS_KM = 6371.0;
#ifndef M_PI  
#define M_PI 3.14159265358979323846  
#endif  

void GPS::UpdatePossion()
{  
   ifstream file("src/GPS.txt");  
   float pos1, pos2, pos3;  
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
   catch (const exception& error)  
   {  
       cout << "Error: " << error.what() << endl;  
   }  
}


// פונקציה להמרת מעלות לרדיאנים
double deg2rad(double deg) {  
    return deg * M_PI / 180.0;  
}

// פונקציית Haversine
double GPS::haversine(double lat1, double lon1, double lat2, double lon2) {
    // המרת הקואורדינטות לרדיאנים
    lat1 = deg2rad(lat1);
    lon1 = deg2rad(lon1);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);

    // הפרשים
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // נוסחת Haversine
    double a = std::pow(std::sin(dlat / 2), 2) +
        std::cos(lat1) * std::cos(lat2) * std::pow(std::sin(dlon / 2), 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // חישוב המרחק
    return EARTH_RADIUS_KM * c;
}
