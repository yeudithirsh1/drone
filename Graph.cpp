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

// ����� ������� ��������� �� �����
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


vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float step, bool flag) {
    vector<Vertex> Vertexs;
    if (flag)
        Vertexs.push_back({ A.x, A.y, A.z, nullptr });

    float distance = sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
    int steps = distance / step;

    float dx = (B.x - A.x) / distance * step;
    float dy = (B.y - A.y) / distance * step;

    for (int i = 1; i <= steps; i++) {
        Vertex P = { A.x + i * dx, A.y + i * dy, A.z, nullptr };
        Vertexs.push_back(P);
    }
    return Vertexs;
}



vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph) {
	int count = 0;
    bool flag = true;
    int parallelSide = graph.size() - 1, parallelVertex = graph[graph.size() - 1].size() - 1,
        currentRib, vertexOnRib;
    for (currentRib = 0; currentRib < graph.size() && currentRib < parallelSide; currentRib++)
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
                graph[currentRib][vertexOnRib].next = &graph[parallelSide][parallelVertex];
                if (parallelVertex - 2 >= 0)
                    parallelVertex -= 2;
                else
                {
                    if (parallelVertex == 0) {
                        parallelVertex = graph[parallelSide - 1].size() - 2;
                        parallelSide--;
                    }
                    else {
                        parallelVertex = graph[parallelSide - 1].size() - 1;
                        parallelSide--;
                    }
                }
                flag = true;
            }
			count++;
        }
    }
    int lastRib = currentRib - 1;
    int lastPoint = vertexOnRib - 1;
	int num = 0;
    flag = true;
    parallelSide = 0, parallelVertex = 2;
    for (currentRib = graph.size() - 1; currentRib >= 0 && parallelSide < currentRib; currentRib--)
    {
        for (vertexOnRib = graph[currentRib].size() - 1; vertexOnRib >= 0 && num < count; vertexOnRib--)
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
                graph[currentRib][vertexOnRib].next = &graph[parallelSide][parallelVertex];
                if (parallelVertex + 2 <= graph[currentRib].size() - 1)
                    parallelVertex += 2;
                else
                {
                    if (parallelVertex == graph[parallelSide].size() - 1) {
                        parallelVertex = 1;
                        parallelSide++;
                    }
                    else {
                        parallelVertex = 0;
                        parallelSide++;
                    }
                }
                flag = true;
            }
        }
    }

    if (graph[currentRib + 1][vertexOnRib + 1] == graph[lastRib + 1][0]) {
        graph[currentRib + 1][vertexOnRib + 1].next = nullptr;
        return graph;
    }
    for (int i = vertexOnRib; i >= 0; i--)
    {
        if(i==0)
           graph[currentRib + 1][i].next = nullptr;
		else
		   graph[currentRib + 1][i].next = &graph[currentRib + 1][i - 1];
    }
    return graph;
}

// ������� �����
vector<vector<Vertex>> graphNavigationPath(vector<Vertex>& Landmarks, float fieldView)
{
    bool flag;
    vector<Vertex> hull;
    convexHull(Landmarks, hull);//����� ������ ������ 
    vector<pair<Vertex, Vertex>> edges = createEdges(hull);
    vector<vector<Vertex>> VertexsOnLines;//���
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
