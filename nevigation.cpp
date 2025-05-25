#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "nevigation.h"
using namespace std;

// חישוב מכפלה וקטורית
float crossProduct(const Vertex &a, const Vertex &b, const Vertex &c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת נקודות קיצון
void convexHull(vector<Vertex> &Vertexs, vector<Vertex> &hull)
{
    sort(Vertexs.begin(), Vertexs.end(), [](Vertex &a, Vertex &b) 
    {
       return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    for (const auto &p : Vertexs) 
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), p) <= 0) 
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    int lowerSize = hull.size();
    for (auto it = Vertexs.rbegin(); it != Vertexs.rend(); ++it) 
    {
        while (hull.size() > lowerSize && crossProduct(hull[hull.size() - 2], hull.back(), *it) <= 0) 
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    hull.pop_back(); // הסרת נקודת הסיום המיותרת
}

//(יצירת קשתות (מחבר את הנקודות בצורה מחזורית
vector<pair<Vertex, Vertex>> createEdges(vector<Vertex> &Vertexs) 
{
    vector<pair<Vertex, Vertex>> edges;
    int n = Vertexs.size();
    for (int i = 0; i < n; ++i) 
    {
        edges.push_back({Vertexs[i], Vertexs[(i + 1) % n] });
    }
    return edges;
}

bool isOnLine(Vertex A, Vertex B, Vertex P) {
    if (A.x == B.x) { // בדיקה אם הישר אנכי
        return std::abs(P.x - A.x) < 1e-6; // כל נקודה עם אותו x נמצאת על הישר
    }
    float slope = (B.y - A.y) / (B.x - A.x); // חישוב שיפוע הישר
    float expectedY = slope * (P.x - A.x) + A.y; // חישוב ה-y הצפוי של P על פי משוואת הישר
    return std::abs(P.y - expectedY) < 1e-6; // סובלנות קטנה לשגיאות חישוב
}

//שמירת כל הנקודות שנמצאות על ישר עם הפרש של רדיוס בין כל נקודה
vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float step, bool flag) {
    vector<Vertex> Vertexs;
    if (A.y == B.y) {
        if (flag)
            Vertexs.push_back({ A.x, A.y, A.z, nullptr });
        Vertexs.push_back({ B.x, B.y, A.z, nullptr });
        return Vertexs;
    }

    float distance = sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
    int steps = distance / step;
    float dx = (B.x - A.x) / distance * step;
    float dy = (B.y - A.y) / distance * step;

    for (int i = 0; i <= steps; i++) {
        Vertex P = { A.x + i * dx, A.y + i * dy, A.z, nullptr};
        if (isOnLine(A, B, P)) {
            if (i == 0 && !flag) continue; // דילוג על הנקודה הראשונה אם flag שווה false
            Vertexs.push_back({ P.x, P.y, A.z, nullptr });
        }
    }

    return Vertexs;
}

vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph) {
	bool flag = true;
    int k = graph.size() - 1, n = graph[graph.size() - 1].size() - 1,
        i, j;
    for(i = 0; i< graph.size() && i < k; i++)
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
	for (i = graph.size() - 1; i >= 0 && k<i; i--)
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
                            k++ ;
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

// פונקציה ראשית
vector<vector<Vertex>> graphNavigationPath(vector<Vertex> &Vertexs, float fieldView)
{
    vector<Vertex> hull;
	convexHull(Vertexs, hull);
    vector<pair<Vertex, Vertex>> edges = createEdges(hull);
    vector<vector<Vertex>> VertexsOnLines;
	bool flag = true;
    for (int i = 0; i < edges.size(); ++i)
    { 
        if (i != 0)
		{
			flag = false;
		}
        vector<Vertex> lineVertexs = generateVertexsOnLine(edges[i].first, edges[i].second, fieldView, flag);
        VertexsOnLines.push_back(lineVertexs);
    }
    if (!VertexsOnLines.empty() && !VertexsOnLines.back().empty()) 
    {
        VertexsOnLines.back().back().next = nullptr;
    }
    VertexsOnLines = zigzag(VertexsOnLines);
    return VertexsOnLines;
}


