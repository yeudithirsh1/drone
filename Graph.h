#pragma once
#include <vector>

using namespace std;

struct Vertex
{
    float x, y, z;
    Vertex* next;
    Vertex(float x, float y, float z, Vertex* next = nullptr)
        : x(x), y(y), z(z), next(next) {
    }
    bool operator==(const Vertex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

};

float crossProduct(const Vertex& a, const Vertex& b, const Vertex& c);
void convexHull(vector<Vertex>& Landmarks, vector<Vertex>& hull);
vector<pair<Vertex, Vertex>> createEdges(vector<Vertex>& Vertexs);
bool isOnLine(Vertex A, Vertex B, Vertex P);
vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float step, bool flag);
vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph);
vector<vector<Vertex>> graphNavigationPath(vector<Vertex>& Landmarks, float fieldView);