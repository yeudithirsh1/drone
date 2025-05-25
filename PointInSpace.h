#ifndef POINT_IN_SPACE_H  
#define POINT_IN_SPACE_H  

struct Point {  
   float x, y, z;  

   // Default constructor to fix the error  
   Point() : x(0), y(0), z(0) {}  

   // Constructor to initialize Point with x, y, z values  
   Point(float x, float y, float z) : x(x), y(y), z(z) {} 
};  

#endif // POINT_IN_SPACE_H


