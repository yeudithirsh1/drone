#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "nevigation.h"
using namespace std;

//���� �� ����� �� ���
using Point = nevigation::Point;

// ����� ����� �������
double crossProduct(const Point &a, const Point &b, const Point &c) 
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// ����� ������ �����
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
    hull.pop_back(); // ���� ����� ����� �������
    return hull;
}

//(����� ����� (���� �� ������� ����� �������
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
    if (A.x == B.x) { // ����� �� ���� ����
        return std::abs(P.x - A.x) < 1e-6; // �� ����� �� ���� x ����� �� ����
    }
    double slope = (B.y - A.y) / (B.x - A.x); // ����� ����� ����
    double expectedY = slope * (P.x - A.x) + A.y; // ����� �-y ����� �� P �� �� ������ ����
    return std::abs(P.y - expectedY) < 1e-6; // ������� ���� ������� �����
}

//����� �� ������� ������� �� ��� �� ���� �� ����� ��� �� �����
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

// vector<vector<Point>> zigzag(vector<vector<Point>> &graph)
// {
//    return graph;
// }

vector<vector<Point>> zigzag(vector<vector<Point>> edges)
{
    if (edges.size() < 2) return edges; // ��� ����� ����� ������
    size_t numEdges = edges.size();
    bool goingUp = true; // ����� ������� ������ (����� �� ����)

    for (size_t i = 0; i < numEdges - 1; ++i) { // ���� ��� ����� �����
        vector<Point>& currentEdge = edges[i];
        vector<Point>& nextEdge = edges[i + 1];
        size_t curSize = currentEdge.size();
        size_t nextSize = nextEdge.size();
        size_t curIdx = goingUp ? 0 : curSize - 1; // ���� ����/�����
        size_t nextIdx = goingUp ? 0 : nextSize - 1;
        while (curIdx < curSize && nextIdx < nextSize) {
            currentEdge[curIdx].next = &nextEdge[nextIdx]; // ����� ������ �������

            // ���� ������ ������ (�����/����)
            if (goingUp) {
                if (++curIdx >= curSize) break;
            }
            else {
                if (curIdx == 0) break;
                --curIdx;
            }

            // ����� ������ ���� ����� ���
            nextEdge[nextIdx].next = &currentEdge[curIdx];

            // ���� ���� �������
            if (goingUp) {
                if (++nextIdx >= nextSize) break;
            }
            else {
                if (nextIdx == 0) break;
                --nextIdx;
            }
        }

        goingUp = !goingUp; // ���� ����� ������ �����
    }
    return edges;
}

// ������� �����
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
    //����� ����
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
                cout << "nullptr";
            }
        }
    }
    return pointsOnLines;
}

//int main() 
//{
//    vector<Point> points = {{0, 5, 5, nullptr}, {5, 0, 5, nullptr}, {10, 5, 5, nullptr}, {5, 10, 5, nullptr} };
//    // { {0, 0, 5, nullptr}, {4, 0, 5, nullptr}, {2, 1, 5, nullptr}, {1, 3, 5, nullptr}, {3, 4, 5, nullptr}, {5, 2, 5, nullptr}, {2, 2, 5, nullptr} }
//    double r;
//    cout << "Enter the drone's visual range.";
//    cin >> r;   
//    vector<vector<Point>> edges = processPoints(points, r);
//    return 0;
//}
