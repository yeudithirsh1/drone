#pragma once  

struct Point2D
{
	float x, y;

	// Add a constructor to initialize Point with x, y, z values  
	Point2D(float x, float y) : x(x), y(y) {}
	Point2D operator+(const Point2D& other) const { return { x + other.x, y + other.y }; }
	Point2D operator-(const Point2D& other) const { return { x - other.x, y - other.y }; }
	Point2D operator*(float scalar) const { return { x * scalar, y * scalar }; }
	Point2D operator/(float scalar) const { return { x / scalar, y / scalar }; }


};