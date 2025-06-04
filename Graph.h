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

float toRadians(float degrees);
float toDegrees(float radians);
float bearingOrAzimuth(float lat1, float lon1, float lat2, float lon2, bool d);
float relativeBearing(Vertex A, Vertex B, Vertex C, bool d);
void convexHull(vector<Vertex>& Vertexs, vector<Vertex>& hull, bool d);
pair<float, float> destinationPoint(float lat, float lon, float azimuthDeg, float distanceMeters);
vector<pair<Vertex, Vertex>> createEdges(vector<Vertex>& Vertexs);
vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float fieldView, bool flag, bool d);
vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph);
vector<vector<Vertex>> graphNavigationPath(vector<Vertex>& Landmarks, float fieldView);