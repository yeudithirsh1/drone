#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "nevigation.h"
using namespace std;

//מבנה של נקודה על גרף
using Point = nevigation::Point;

// חישוב מכפלה וקטורית
double crossProduct(const Point &a, const Point &b, const Point &c) 
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת נקודות קיצון
vector<Point> convexHull(vector<Point> &points)
{
    sort(points.begin(), points.end(), [](const Point &a, const Point &b) 
    {
       return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    vector<Point> hull;
    for (const auto &p : points) 
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), p) <= 0) 
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    size_t lowerSize = hull.size();
    for (auto it = points.rbegin(); it != points.rend(); ++it) 
    {
        while (hull.size() > lowerSize && crossProduct(hull[hull.size() - 2], hull.back(), *it) <= 0) 
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    hull.pop_back(); // הסרת נקודת הסיום המיותרת
    return hull;
}

//(יצירת קשתות (מחבר את הנקודות בצורה מחזורית
vector<pair<Point, Point>> createEdges(const vector<Point> &points) 
{
    vector<pair<Point, Point>> edges;
    int n = points.size();
    for (int i = 0; i < n; ++i) 
    {
        edges.push_back({points[i], points[(i + 1) % n] });
    }
    for (const auto& edge : edges)
    {
        cout << "Edge: (" << edge.first.x << ", " << edge.first.y << ", " << edge.first.z << ") -> ("
            << edge.second.x << ", " << edge.second.y << ", " << edge.second.z << ")\n";
    }
    return edges;
}

bool isOnLine(Point A, Point B, Point P) {
    if (A.x == B.x) { // בדיקה אם הישר אנכי
        return std::abs(P.x - A.x) < 1e-6; // כל נקודה עם אותו x נמצאת על הישר
    }
    double slope = (B.y - A.y) / (B.x - A.x); // חישוב שיפוע הישר
    double expectedY = slope * (P.x - A.x) + A.y; // חישוב ה-y הצפוי של P על פי משוואת הישר
    return std::abs(P.y - expectedY) < 1e-6; // סובלנות קטנה לשגיאות חישוב
}

//שמירת כל הנקודות שנמצאות על ישר עם הפרש של רדיוס בין כל נקודה
vector<Point> generatePointsOnLine(Point A, Point B, double step) {
    vector<Point> points;
	if (A.y == B.y) {
	    points.push_back({A.x, A.y, A.z, nullptr });
        points.push_back({B.x, B.y, A.z, nullptr });
		return points;
	}
    double distance = sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
    int steps = distance / step;
    double dx = (B.x - A.x) / distance * step;
    double dy = (B.y - A.y) / distance * step;
    for (int i = 0; i <= steps; i++) 
    {
        Point P = { A.x + i * dx, A.y + i * dy };
        if (isOnLine(A, B, P)) 
            points.push_back({ P.x, P.y, A.z, nullptr });
    }

    return points;
}

 vector<vector<Point>> zigzag(vector<vector<Point>> &graph)
 {
    int j, l, t, m;
	bool flag = false;
	for (int i = 0, k = graph.size()-1; i < graph.size(), k >= 0; i++, k--)
	{
        if (i>0)
        {
		   if (flag)
		   	   t = 0;
           else
		       m = min(graph[i].size(), graph[k].size())-1;
		}
        else
        {
			t = 0;
			m = min(graph[i].size(), graph[k].size()) - 1;
        }
        for (j = t; j < min(graph[i].size(), graph[k].size()); j++)
        {
            if (j % 2 == 0)
                graph[i][j].next = &graph[i][j + 1];
            else
                graph[i][j].next = &graph[k][j];
        }
        for (l = m-1; l >= 0; l--)
        {
            if (l % 2 != 0)
                graph[k][l].next = &graph[k][l+1];
            else
                graph[k][l].next = &graph[i][l];
        }
        if (j < graph[i].size() && l== graph[k].size())
        {
            i--;
			t = j;
			flag = false; 
        }
        else
        {
            if (l < graph[k].size() && j== graph[i].size())
            {
                k--;
                m = l;	
				flag = true;
            }

        }
	}
    return graph;
 }

// פונקציה ראשית
vector<vector<Point>> processPoints(vector<Point> &points, int r)
{
    vector<Point> hull = convexHull(points);
    vector<pair<Point, Point>> edges = createEdges(hull);
    vector<vector<Point>> pointsOnLines;
    for (int i = 0; i < edges.size(); ++i)
    { 
        vector<Point> linePoints = generatePointsOnLine(edges[i].first, edges[i].second, r);
        pointsOnLines.push_back(linePoints);
    }
	pointsOnLines = zigzag(pointsOnLines);
    //הדפסת הגרף
    for (const auto& line : pointsOnLines)
    {
        cout << "Points on line:\n";
        for (const auto& p : line)
        {
            cout << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.next << ")\n";
        }
    }
    return pointsOnLines;
}

int main() 
{
    vector<Point> points = {{0, 5, 5, nullptr}, {5, 0, 5, nullptr}, {10, 5, 5, nullptr}, {5, 10, 5, nullptr} };
    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }
    double r;
    cout << "Enter the drone's visual range.";
    cin >> r;   
    vector<vector<Point>> edges = processPoints(points, r);
    return 0;
}
