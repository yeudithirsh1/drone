#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "Graph.h"

using namespace std;


float crossProduct(const Vertex& a, const Vertex& b, const Vertex& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// מציאת הנקודות החיצוניות של הצורה
void convexHull(vector<Vertex>& Landmarks, vector<Vertex>& hull)
{
    sort(Landmarks.begin(), Landmarks.end(), [](Vertex& a, Vertex& b)
        {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });
    for (const auto& p : Landmarks)
    {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), p) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    int lowerSize = hull.size();
    for (auto it = Landmarks.rbegin(); it != Landmarks.rend(); ++it)
    {
        while (hull.size() > lowerSize && crossProduct(hull[hull.size() - 2], hull.back(), *it) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    hull.pop_back();
}

vector<pair<Vertex, Vertex>> createEdges(vector<Vertex>& Vertexs)
{
    vector<pair<Vertex, Vertex>> edges;
    int n = Vertexs.size();
    for (int i = 0; i < n; ++i)
    {
        edges.push_back({ Vertexs[i], Vertexs[(i + 1) % n] });
    }
    return edges;
}

bool isOnLine(Vertex A, Vertex B, Vertex P) {
    
    double slope = (B.y - A.y) / (B.x - A.x); // חישוב שיפוע הישר
    double expectedY = slope * (P.x - A.x) + A.y; // חישוב ה-y הצפוי של P על פי משוואת הישר
    return abs(P.y - expectedY) < 1e-6; // סובלנות קטנה לשגיאות חישוב
}


vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float step, bool flag) {
    vector<Vertex> Vertexs;
    if (flag)
        Vertexs.push_back({ A.x, A.y, A.z, nullptr });

    float distance = sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
    int steps = distance / step;

    float dx = (B.x - A.x) / distance * step;
    float dy = (B.y - A.y) / distance * step;

    for (int i = 0; i <= steps; i++) {
        Vertex P = { A.x + i * dx, A.y + i * dy, A.z, nullptr };
        if (isOnLine(A, B, P)) {
            if (i == 0 && flag) continue;
            Vertexs.push_back(P);
        }
    }

    return Vertexs;
}



vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph) {
    bool flag = true;
    int parallelRibs = graph.size() - 1, parallelVertex = graph[graph.size() - 1].size() - 1,
        currentRib, vertexOnRib;
    for (currentRib = 0; currentRib < graph.size() && currentRib < parallelRibs; currentRib++)
    {
        for (vertexOnRib = 0; vertexOnRib < graph[currentRib].size(); vertexOnRib++)
        {
            if (flag)
            {
                if (vertexOnRib + 1 < graph[currentRib].size()) {
                    graph[currentRib][vertexOnRib].next = &graph[currentRib][vertexOnRib + 1];
                }
                else {
                    graph[currentRib][vertexOnRib].next = &graph[currentRib + 1][0];
                }
                flag = false;
            }
            else
            {
                if (parallelVertex >= 0)
                {
                    graph[currentRib][vertexOnRib].next = &graph[parallelRibs][parallelVertex];
                    if (parallelVertex - 2 >= 0)
                        parallelVertex -= 2;
                    else
                    {
                        if (parallelVertex == 0) {
                            parallelVertex = graph[parallelRibs - 1].size() - 2;
                            parallelRibs--;
                        }
                        else {
                            parallelVertex = graph[parallelRibs - 1].size() - 1;
                            parallelRibs--;
                        }
                    }
                }
                flag = true;
            }
        }
    }
    int lastPoint = currentRib - 1;
    flag = true;
    parallelRibs = 0, parallelVertex = 2;
    for (currentRib = graph.size() - 1; currentRib >= 0 && parallelRibs < currentRib; currentRib--)
    {
        for (vertexOnRib = graph[currentRib].size() - 1; vertexOnRib >= 0; vertexOnRib--)
        {
            if (flag)
            {
                if (vertexOnRib - 1 >= 0) {
                    graph[currentRib][vertexOnRib].next = &graph[currentRib][vertexOnRib - 1];
                }
                else {
                    graph[currentRib][vertexOnRib].next = &graph[currentRib - 1][graph[currentRib - 1].size() - 1];
                }
                flag = false;
            }
            else
            {
                if (parallelVertex < graph[currentRib].size())
                {
                    graph[currentRib][vertexOnRib].next = &graph[parallelRibs][parallelVertex];
                    if (parallelVertex + 2 <= graph[currentRib].size() - 1)
                        parallelVertex += 2;
                    else
                    {
                        if (parallelVertex == graph[parallelRibs].size() - 1) {
                            parallelVertex = 1;
                            parallelRibs++;
                        }
                        else {
                            parallelVertex = 0;
                            parallelRibs++;
                        }
                    }
                }
                flag = true;
            }
        }
    }

    for (int i = currentRib; i > lastPoint; i--) {
        for (int j = vertexOnRib; j >= 0; j--)
            if (i - 1== lastPoint && j == 0)
                graph[i][j].next = nullptr;
            else if (j == 0 && i - 1 > lastPoint)
                graph[i - 1][graph[i - 1].size() - 1].next = nullptr;
            else
                graph[i][j].next = &graph[i][j - 1];
    }
    return graph;
}

// פונקציה ראשית
vector<vector<Vertex>> graphNavigationPath(vector<Vertex>& Landmarks, float fieldView)
{
    bool flag;
    vector<Vertex> hull;
    convexHull(Landmarks, hull);//חישוב המעטפה הקמורה 
    vector<pair<Vertex, Vertex>> edges = createEdges(hull);
    vector<vector<Vertex>> VertexsOnLines;//גרף
    for (int i = 0; i < edges.size(); i++)
    {
        if (i == 0) {
            flag = true;}
        else {
            if (VertexsOnLines.back().back() == edges[i].first)
                flag = false;
            else
                flag = true;}  
        vector<Vertex> lineVertexs = generateVertexsOnLine(edges[i].first, edges[i].second, fieldView, flag);
        VertexsOnLines.push_back(lineVertexs);
    }
    VertexsOnLines = zigzag(VertexsOnLines);
    for (int i = 0; i < VertexsOnLines.size(); i++) {
        cout << i << "\n";
        for (int j = 0; j < VertexsOnLines[i].size(); j++) {
            const Vertex& v = VertexsOnLines[i][j];
            cout << "Vertex (" << v.x << ", " << v.y << ", " << v.z << ") -> ";
            if (v.next)
                cout << "(" << v.next->x << ", " << v.next->y << ", " << v.next->z << ")";
            else
                cout << "nullptr";
            cout << endl;
        }
    }
    return VertexsOnLines;
}
