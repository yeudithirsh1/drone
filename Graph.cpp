#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "GPS.h"  

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vertex
{
    float x, y, z;
    Vertex* next;
    Vertex(float x, float y, float z, Vertex* next = nullptr)
        : x(x), y(y), z(z), next(next) {
    }

};
//המרה ממעלות לרדיינים
float toRadians(float degrees) {
    return degrees * M_PI / 180.0;
}
// ממירה מרדיינים למעלות
float toDegrees(float radians) {
    return radians * 180.0 / M_PI;
}

// חישוב כיוון (bearing) בין שתי נקודות על פני כדור - סיבוב בין 2 נקודות ביחס לצפון
float bearingOrAzimuth(float lat1, float lon1, float lat2, float lon2, bool d) {
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    float dLon = lon2 - lon1;
    float x = sin(dLon) * cos(lat2);
    float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float brng, azimuth;
    if (d) {
        brng = atan2(x, y);
        return fmod(toDegrees(brng) + 2 * M_PI, 2 * M_PI); // להחזיר ערך בין 0 ל-360
    }
    azimuth = atan2(y, x);
    return toDegrees(fmod((azimuth + 2 * M_PI), 2 * M_PI)); // תוצאה בין 0 ל־360
}

float relativeBearing(Vertex A, Vertex B, Vertex C, bool d) {
    float brngAB = bearingOrAzimuth(A.x, A.y, B.x, B.y, d);
    float brngAC = bearingOrAzimuth(A.x, A.y, C.x, C.y, d);
    float diff = fmod(brngAC - brngAB + 540.0, 360.0) - 180.0;
    return diff;
}

// מציאת הנקודות החיצוניות של הצורה
void convexHull(vector<Vertex> &Vertexs, vector<Vertex> &hull, bool d)
{
    sort(Vertexs.begin(), Vertexs.end(), [](Vertex &a, Vertex &b) 
    {
       return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    for (const auto &p : Vertexs) 
    {

        while (hull.size() >= 2 && relativeBearing(hull[hull.size() - 2], hull.back(), p, d) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    int lowerSize = hull.size();
    for (auto it = Vertexs.rbegin(); it != Vertexs.rend(); ++it) 
    {
        while (hull.size() > lowerSize && relativeBearing(hull[hull.size() - 2], hull.back(), *it, d) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(*it);
    }
    hull.pop_back(); // הסרת נקודת הסיום המיותרת
}

pair<float, float> destinationPoint(float lat, float lon, float azimuthDeg, float distanceMeters) {
    float R = 6371000.0; // רדיוס כדור הארץ במטרים
    float bearing = toRadians(azimuthDeg);
    float delta = distanceMeters / R;

    lat = toRadians(lat);
    lon = toRadians(lon);

    float newLat = asin(sin(lat) * cos(delta) + cos(lat) * sin(delta) * cos(bearing));
    float newLon = lon + atan2(sin(bearing) * sin(delta) * cos(lat),
        cos(delta) - sin(lat) * sin(newLat));

    return { toDegrees(newLat), toDegrees(newLon) };
}



//שמירת כל הנקודות שנמצאות על ישר עם הפרש של רדיוס בין כל נקודה
vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float fieldView, bool flag, bool d) {
    vector<Vertex> Vertexs;
    if (flag)
        Vertexs.push_back({ A.x, A.y, A.z, nullptr });
    float distance = haversine(A.x, A.y, B.x, B.y);
    int steps = distance / fieldView;

    float azimuth = bearingOrAzimuth(A.x, A.y, B.x, B.y, d); // מעלות

    for (int i = 1; i <= steps; i++) {
        float r = i * fieldView;
        auto point = destinationPoint(A.x, A.y, azimuth, r);
        Vertex P = { point.first, point.second, A.z, nullptr };
        Vertexs.push_back(P);
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
                if (j + 1 < graph[i].size()){
                    graph[i][j].next = &graph[i][j + 1];}
                else{
                    graph[i][j].next = &graph[i + 1][0];}
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
                        if (n == 0){
                            n = graph[k - 1].size() - 2;
                            k--;}
                        else{
                            n = graph[k - 1].size() - 1;
                            k--;}
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
				if (j - 1 >= 0){
					graph[i][j].next = &graph[i][j - 1];}
				else{
					graph[i][j].next = &graph[i - 1][graph[i - 1].size() - 1];}
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
						if (n == graph[k].size() - 1){
							n = 1;
                            k++;}
						else{
							n = 0;
                            k++ ;}
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
vector<vector<Vertex>> graphNavigationPath(vector<Vertex> &Landmarks, float fieldView)
{
	bool flag = true, d = true;
    vector<Vertex> hull;
	convexHull(Landmarks, hull, d);//חישוב המעטפה הקמורה 
    vector<vector<Vertex>> VertexsOnLines;//גרף
    d = false;
    for (int i = 1; i < hull.size(); ++i)
    { 
        if (i != 1)
		{
			flag = false;
            vector<Vertex> lineVertexs = generateVertexsOnLine(hull[i-1], hull[i], fieldView, flag, d);
		}
        vector<Vertex> lineVertexs = generateVertexsOnLine(VertexsOnLines[i-1][VertexsOnLines.size()-1], hull[i], fieldView, flag, d);
        VertexsOnLines.push_back(lineVertexs);
    }
    if (!VertexsOnLines.empty() && !VertexsOnLines.back().empty()) 
    {
        VertexsOnLines.back().back().next = nullptr;
    }
    VertexsOnLines = zigzag(VertexsOnLines);
    return VertexsOnLines;
}



