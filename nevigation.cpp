#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "nevigation.h"
using namespace std;

// חישוב מכפלה וקטורית
double crossProduct(const Point& a, const Point& b, const Point& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת נקודות קיצון
template <typename T>
vector<Point> convexHull(vector<T>& points)
{
    sort(points.begin(), points.end(), [](Point& a, Point& b)
        {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });
    vector<Point> hull;
    for (const auto& p : points)
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), p) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    int lowerSize = hull.size();
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

// יצירת קשתות (מחבר את הנקודות בצורה מחזורית)
vector<pair<Point, Point>> createEdges(vector<Point>& points)
{
    vector<pair<Point, Point>> edges;
    int n = points.size();
    for (int i = 0; i < n; ++i)
    {
        edges.push_back({ points[i], points[(i + 1) % n] });
    }
    return edges;
}

bool isOnLine(Point A, Point B, Point P) {
    if (A.x == B.x) { // בדיקה אם הישר אנכי
        return std::abs(P.x - A.x) < 1e-6;
    }
    double slope = (B.y - A.y) / (B.x - A.x);
    double expectedY = slope * (P.x - A.x) + A.y;
    return std::abs(P.y - expectedY) < 1e-6;
}

// שמירת כל הנקודות שנמצאות על ישר עם הפרש של רדיוס בין כל נקודה
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
        Point P = { A.x + i * dx, A.y + i * dy, A.z, nullptr };
        if (isOnLine(A, B, P)) {
            if (i == 0 && !flag) continue;
            points.push_back({ P.x, P.y, A.z, nullptr });
        }
    }

    return points;
}

vector<vector<Point>> zigzag(vector<vector<Point>> graph) {
    bool flag = true;
    int k = graph.size() - 1, n = graph[graph.size() - 1].size() - 1, i, j;
    for (i = 0; i < graph.size() && i < k; i++)
    {
        for (j = 0; j < graph[i].size(); j++)
        {
            if (flag)
            {
                if (j + 1 < graph[i].size())
                {
                    graph[i][j].next = &graph[i][j + 1];
                }
                else
                {
                    graph[i][j].next = &graph[i + 1][0];
                }
                flag = false;
            }
            else
            {
                if (n >= 0)
                {
                    graph[i][j].next = &graph[k][n];
                    if (n - 2 >= 0)
                        n -= 2;
                    else
                    {
                        if (n == 0)
                        {
                            n = graph[k - 1].size() - 2;
                            k--;
                        }
                        else
                        {
                            n = graph[k - 1].size() - 1;
                            k--;
                        }
                    }
                }
                flag = true;
            }
        }
    }
    flag = true;
    k = 0, n = 2;
    for (i = graph.size() - 1; i >= 0 && k < i; i--)
    {
        for (j = graph[i].size() - 1; j >= 0; j--)
        {
            if (flag)
            {
                if (j - 1 >= 0)
                {
                    graph[i][j].next = &graph[i][j - 1];
                }
                else
                {
                    graph[i][j].next = &graph[i - 1][graph[i - 1].size() - 1];
                }
                flag = false;
            }
            else
            {
                if (n < graph[i].size())
                {
                    graph[i][j].next = &graph[k][n];
                    if (n + 2 <= graph[i].size() - 1)
                        n += 2;
                    else
                    {
                        if (n == graph[k].size() - 1)
                        {
                            n = 1;
                            k++;
                        }
                        else
                        {
                            n = 0;
                            k++;
                        }
                    }
                }
                flag = true;
            }
        }
    }
    graph[i + 1][j + 1].next = nullptr;
    return graph;
}

// הדפסת הגרף
void printGraph(const vector<vector<Point>>& graph) {
    cout << "Graph structure:\n";
    for (int i = 0; i < graph.size(); ++i) {
        for (int j = 0; j < graph[i].size(); ++j) {
            const Point& p = graph[i][j];
            cout << "Point(" << p.x << ", " << p.y << ", " << p.z << ")";
            if (p.next) {
                cout << " -> Next(" << p.next->x << ", " << p.next->y << ", " << p.next->z << ")";
            }
            else {
                cout << " -> Next(nullptr)";
            }
            cout << endl;
        }
    }
}

// פונקציה ראשית
vector<vector<Point>> processPoints(vector<Point>& points, double r)
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

    // הדפסת הגרף
    printGraph(pointsOnLines);

    return pointsOnLines;
}
