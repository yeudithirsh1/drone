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
vector<Point> generatePointsOnLine(Point A, Point B, double step, bool flag) {
    vector<Point> points;
    if (A.y == B.y) {
        if (flag)
            points.push_back({ A.x, A.y, A.z, nullptr });
        points.push_back({ B.x, B.y, A.z, nullptr });
        return points;
    }

    double distance = sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
    int steps = distance / step;
    double dx = (B.x - A.x) / distance * step;
    double dy = (B.y - A.y) / distance * step;

    for (int i = 0; i <= steps; i++) {
        Point P = { A.x + i * dx, A.y + i * dy };
        if (isOnLine(A, B, P)) {
            if (i == 0 && !flag) continue; // דילוג על הנקודה הראשונה אם flag שווה false
            points.push_back({ P.x, P.y, A.z, nullptr });
        }
    }

    return points;
}

vector<vector<Point>> zigzag(vector<vector<Point>> graph)
{
    int j = NULL, l, t=0, m = graph[graph.size()-1].size() - 1, n = graph[graph.size() - 1].size() - 1, s = 2, length = min(graph[0].size(), graph[graph.size() - 1].size());
    bool flag = false, d1 = true, d2 =true, b;

    // לולאה עיקרית שמבצעת את הסריקה
    for (int i = 0, k = graph.size() - 1; i < graph.size() && k >= 0; i++, k--)
    {
     
        // חיבור בין נקודות בצלע הנוכחית
        for (j = t; j < length && n >= 0; j++)
        {    
            cout << i << "," << k << "," << j << "," << t << "," << n << "," << d2 << "\n";
            if (d2)
            {
                if (j + 1 < graph[i].size())
                    graph[i][j].next = &graph[i][j + 1];
                else
                    if (i + 1 < graph.size())
                    {
                        graph[i][j].next = &graph[i + 1][0];
                        b = true;
                    }
                d2 = false;
            }
            else
            {
                if (n >= 0)
                    graph[i][j].next = &graph[k][n];
                d2 = true;
                n--;
            }
        }    
        cout << "------------------\n";
        // חיבור בין הנקודות בצלע השנייה בכיוון ההפוך
        for (l = m; l >= 0 && s <= graph[i].size(); l--)
        {
            cout << i << "," << k << "," << l << "," << m << "," << s << "," << d1 << "\n";
            if (d1)
            {
                if (l - 1 >= 0)
                    graph[k][l].next = &graph[k][l - 1];
                else
                    if (k > 0)
                    {
                        graph[k][l].next = &graph[k - 1][graph[k - 1].size() - 1];
						flag = true;
                    }
                d1 = false;
            }
            else
            {
                if (s < graph[i].size())
                    graph[k][l].next = &graph[i][s];
                d1 = true;
                s++;
            }
        }
        cout << "------------------\n";

        // עדכון מצב כדי להחליף צלע
        if (j < graph[i].size() && l <= 0)
        {
            cout << j << "\n";
            i--;
            t = j;
            flag = false;
            length = graph[i].size();
            m = graph[k - 1].size() - 1;
            if (flag)
                n = graph[k - 1].size() - 2;
            else
                n = graph[k - 1].size() - 1;
        }
        else 
        { 
            if (l < graph[k].size() && j == graph[i].size())
            {
                k--;
                m = l;
				length = graph[k].size();
                t = 0;
                if (b)
                    s = 1;
                else
                    s = 0;     
            }
            else 
            { 
                if (j == graph[i].size() && l == graph[k].size())
                 {
                    t = 0;
                    m = graph[k -1].size() - 1;
                    if (b)
                        s = 1;
                    else
                        s = 0;
                    if (flag)
                        n = graph[k - 1].size() - 2;
                    else
                        n = graph[k - 1].size() - 1;
                 }
            }
        }
    }

    return graph; // החזרת הגרף לאחר סיום החיבורים
}

// פונקציה ראשית
vector<vector<Point>> processPoints(vector<Point> &points, int r)
{
    vector<Point> hull = convexHull(points);
    vector<pair<Point, Point>> edges = createEdges(hull);
    vector<vector<Point>> pointsOnLines;
	bool flag = true;
    for (int i = 0; i < edges.size(); ++i)
    { 
        if (i != 0)
		{
			flag = false;
		}
        vector<Point> linePoints = generatePointsOnLine(edges[i].first, edges[i].second, r, flag);
        pointsOnLines.push_back(linePoints);
    }
    if (!pointsOnLines.empty() && !pointsOnLines.back().empty()) 
    {
        pointsOnLines.back().back().next = nullptr;
    }
    pointsOnLines = zigzag(pointsOnLines);
    //הדפסת הגרף
    for (const auto& line : pointsOnLines)
    {
        cout << "Points on line:\n";
        for (const auto& p : line)
        {
            cout << "(" << p.x << ", " << p.y << ", " << p.z << ", next: (";
            if (p.next != nullptr) {
                cout << p.next->x << ", " << p.next->y << ", " << p.next->z << ")\n";
            }
            else {
                cout << "nullptr\n";
            }
        }
    }
    return pointsOnLines;
}

int main() 
{
    vector<Point> points = { {5, 0, 5, nullptr}, {10,5,5, nullptr} , {5,10, 5, nullptr}, {0,5, 5, nullptr } };

    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }
    double r;
    cout << "Enter the drone's visual range.";
    cin >> r;   
    vector<vector<Point>> edges = processPoints(points, r);
    return 0;
}
//pointsOnLines[0][0].next = &pointsOnLines[0][1];
//pointsOnLines[0][1].next = &pointsOnLines[3][2];
//pointsOnLines[3][2].next = &pointsOnLines[3][1];
//pointsOnLines[3][1].next = &pointsOnLines[0][2];
//pointsOnLines[0][2].next = &pointsOnLines[0][3];
//pointsOnLines[0][3].next = &pointsOnLines[3][0];
//pointsOnLines[3][0].next = &pointsOnLines[2][2];
//pointsOnLines[2][2].next = &pointsOnLines[1][0];
//pointsOnLines[1][0].next = &pointsOnLines[1][1];
//pointsOnLines[1][1].next = &pointsOnLines[2][1];
//pointsOnLines[2][1].next = &pointsOnLines[2][0];
//pointsOnLines[2][0].next = &pointsOnLines[1][2];
