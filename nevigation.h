#pragma once
class nevigation
{
public:
	nevigation();
	struct Point
	{
		double x, y, z;
		Point* next;
	};
	double crossProduct(const Point& a, const Point& b, const Point& c);
    
    vector<Point> convexHull(vector<Point>& points);
    
    vector<pair<Point, Point>> createEdges(const vector<Point>& points);
    
    vector<Point> generatePointsOnLine(const Point& p1, const Point& p2, int r, bool b);
    
    vector<vector<Point>> zigzag(vector<vector<Point>> graph);
    
    vector<vector<Point>> processPoints(vector<Point>& points, int r);
    
    bool isOnLine(Point A, Point B, Point P);
    

};

