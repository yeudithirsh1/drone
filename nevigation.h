#pragma once  
using namespace std;  

struct Point  
{  
   double x, y, z;  
   Point* next;  

   Point(double x_val, double y_val, double z_val, Point* next_val = nullptr) :  
       x(x_val), y(y_val), z(z_val), next(next_val) {}  
};  

class nevigation  
{  
 private:  
     Point point;  

 public:  
   nevigation(double x_val, double y_val, double z_val, Point* next_val) :  
       point(x_val, y_val, z_val, next_val) {}  
};


double crossProduct(const Point &a, const Point &b, const Point &c);

vector<Point> convexHull(vector<Point> &points);

vector<pair<Point, Point>> createEdges(vector<Point> &points);

vector<Point> generatePointsOnLine(Point A, Point B, double step, bool flag);

vector<vector<Point>> zigzag(vector<vector<Point>> graph);

vector<vector<Point>> processPoints(vector<Point> &points, double r);

bool isOnLine(Point A, Point B, Point P);


