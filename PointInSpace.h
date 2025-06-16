#pragma once

struct Point {
   float x, y, z;  

   // Default constructor to fix the error  
   Point() : x(0), y(0), z(0) {}  

   // Constructor to initialize Point with x, y, z values  
   Point(float x, float y, float z) : x(x), y(y), z(z) {} 
   
   // השוואה לקסיקוגרפית: קודם לפי x, אחר כך y, אחר כך z
   bool operator<(const Point& other) const {
       if (x != other.x) return x < other.x;
       if (y != other.y) return y < other.y;
       return z < other.z;
   }

   bool operator>(const Point& other) const {
       return other < *this; // שימוש באופרטור < שהוגדר כבר
   }

   bool operator==(const Point& other) const {
       return x == other.x && y == other.y && z == other.z;
   }

   bool operator!=(const Point& other) const {
       return !(*this == other);
   }

   Point operator+(const Point& other) const { return { x + other.x, y + other.y, z + other.z }; }
   Point operator-(const Point& other) const { return { x - other.x, y - other.y, z - other.z}; }
   Point operator*(float scalar) const { return { x * scalar, y * scalar, z * scalar }; }
   Point operator/(float scalar) const { return { x / scalar, y / scalar, z / scalar}; }
};  



	// Add a constructor to initialize Point with x, y, z values  
