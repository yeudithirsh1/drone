#pragma once  

struct Point2f
{
	float x, y;

	// Add a constructor to initialize Point with x, y, z values  
	Point2f(float x, float y) : x(x), y(y) {}
	Point2f operator+(const Point2f& other) const { return { x + other.x, y + other.y }; }
	Point2f operator-(const Point2f& other) const { return { x - other.x, y - other.y }; }
	Point2f operator*(float scalar) const { return { x * scalar, y * scalar }; }
	Point2f operator/(float scalar) const { return { x / scalar, y / scalar }; }


};