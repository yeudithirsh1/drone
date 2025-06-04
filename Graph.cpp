#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include "GPS.h"  
#include "Graph.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
const float EARTH_RADIUS_KM = 6371.0;


////המרה ממעלות לרדיינים
float toRadians(float degrees) {
    return degrees * M_PI / 180.0;
}
// ממירה מרדיינים למעלות
float toDegrees(float radians) {
    return radians * 180.0 / M_PI;
}

//// חישוב כיוון (bearing) בין שתי נקודות על פני כדור - סיבוב בין 2 נקודות ביחס לצפון
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
void convexHull(vector<Vertex>& Landmarks, vector<Vertex>& hull, bool d)
{
    sort(Landmarks.begin(), Landmarks.end(), [](Vertex& a, Vertex& b)
        {
            return a.x < b.x || (a.x == b.x && a.y < b.y);
        });
    for (const auto& p : Landmarks)
    {
        while (hull.size() >= 2 && relativeBearing(hull[hull.size() - 2], hull.back(), p, d) <= 0)
        {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    int lowerSize = hull.size();
    for (auto it = Landmarks.rbegin(); it != Landmarks.rend(); ++it)
    {
        while (hull.size() > lowerSize && relativeBearing(hull[hull.size() - 2], hull.back(), *it, d) <= 0)
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
    cout << distance << "\n";
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
    int parallelRibs = graph.size() - 1, parallelVertex = graph[graph.size() - 1].size() - 1,
        currentRib, vertexOnRib;
    for(currentRib = 0; currentRib < graph.size() && currentRib < parallelRibs; currentRib++)
    {
		for (vertexOnRib = 0; vertexOnRib < graph[currentRib].size(); vertexOnRib++)
		{
            if (flag)
            {
                if (vertexOnRib + 1 < graph[currentRib].size()){
                    graph[currentRib][vertexOnRib].next = &graph[currentRib][vertexOnRib + 1];}
                else{
                    graph[currentRib][vertexOnRib].next = &graph[currentRib + 1][0];}
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
                        if (parallelVertex == 0){
                            parallelVertex  = graph[parallelRibs - 1].size() - 2;
                            parallelRibs--;}
                        else{
                            parallelVertex = graph[parallelRibs - 1].size() - 1;
                            parallelRibs--;}
					}
				}
                flag = true;
            }
		}
    }
    int lastPoint = currentRib-1;
    flag = true;
    parallelRibs = 0, parallelVertex = 2;
	for (currentRib = graph.size() - 1; currentRib >= 0 && parallelRibs < currentRib; currentRib--)
	{
		for (vertexOnRib = graph[currentRib].size() - 1; vertexOnRib >= 0; vertexOnRib--)
		{
            if (flag)
            {
				if (vertexOnRib - 1 >= 0){
					graph[currentRib][vertexOnRib].next = &graph[currentRib][vertexOnRib - 1];}
				else{
					graph[currentRib][vertexOnRib].next = &graph[currentRib - 1][graph[currentRib - 1].size() - 1];}
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
						if (parallelVertex == graph[parallelRibs].size() - 1){
                            parallelVertex = 1;
                            parallelRibs++;}
						else{
                            parallelVertex = 0;
                            parallelRibs++ ;}
					}
				}
				flag = true;
			}
		}
	}

    //for (int i = currentRib; i > lastPoint; i--) {
    //    for (int j = vertexOnRib; j >= 0; j--)
    //        if (i - 1== lastPoint && j == 0)
    //            graph[i][j].next = nullptr;
    //        else if (j == 0 && i - 1 > lastPoint)
    //            graph[i - 1][graph[i - 1].size() - 1].next = nullptr;
    //        else
    //            graph[i][j].next = &graph[i][j - 1];
    //}
    return graph;
}

//vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph) {
//    bool flag = true;
//    int k = graph.size() - 1, n = graph[graph.size() - 1].size() - 1,
//        i, j;
//    for (i = 0; i < graph.size() && i < k; i++)
//    {
//        for (j = 0; j < graph[i].size(); j++)
//        {
//            if (flag)
//            {
//                if (j + 1 < graph[i].size()) {
//                    graph[i][j].next = &graph[i][j + 1];
//                }
//                else {
//                    graph[i][j].next = &graph[i + 1][0];
//                }
//                flag = false;
//            }
//            else
//            {
//                if (n >= 0)
//                {
//                    graph[i][j].next = &graph[k][n];
//                    if (n - 2 >= 0)
//                        n -= 2;
//                    else
//                    {
//                        if (n == 0) {
//                            n = graph[k - 1].size() - 2;
//                            k--;
//                        }
//                        else {
//                            n = graph[k - 1].size() - 1;
//                            k--;
//                        }
//                    }
//                }
//                flag = true;
//            }
//        }
//    }
//    flag = true;
//    k = 0, n = 2;
//    for (i = graph.size() - 1; i >= 0 && k < i; i--)
//    {
//        for (j = graph[i].size() - 1; j >= 0; j--)
//        {
//            if (flag)
//            {
//                if (j - 1 >= 0) {
//                    graph[i][j].next = &graph[i][j - 1];
//                }
//                else {
//                    graph[i][j].next = &graph[i - 1][graph[i - 1].size() - 1];
//                }
//                flag = false;
//            }
//            else
//            {
//                if (n < graph[i].size())
//                {
//                    graph[i][j].next = &graph[k][n];
//                    if (n + 2 <= graph[i].size() - 1)
//                        n += 2;
//                    else
//                    {
//                        if (n == graph[k].size() - 1) {
//                            n = 1;
//                            k++;
//                        }
//                        else {
//                            n = 0;
//                            k++;
//                        }
//                    }
//                }
//                flag = true;
//            }
//        }
//    }
//    graph[i + 1][j + 1].next = nullptr;
//    return graph;
//}




// פונקציה ראשית
vector<vector<Vertex>> graphNavigationPath(vector<Vertex> &Landmarks, float fieldView)
{
	bool flag = true, d = true;
    vector<Vertex> hull;
	convexHull(Landmarks, hull, d);//חישוב המעטפה הקמורה 
    vector<pair<Vertex, Vertex>> edges = createEdges(hull);
    vector<vector<Vertex>> VertexsOnLines;//גרף
    d = false;
    for (int i = 0; i < edges.size(); i++)
    { 
        if (i == 0){
            vector<Vertex> lineVertexs = generateVertexsOnLine(edges[i].first, edges[i].second, fieldView, flag, d);
            VertexsOnLines.push_back(lineVertexs);

	 	}
        else {
            if (VertexsOnLines.back().back() == edges[i].first)
                flag = false;
            else
                flag = true;
            vector<Vertex> lineVertexs = generateVertexsOnLine(edges[i].first, edges[i].second, fieldView, flag, d);
            VertexsOnLines.push_back(lineVertexs);
        }
    }        

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
    /*if (!VertexsOnLines.empty() && !VertexsOnLines.back().empty()) 
    {
        VertexsOnLines.back().back().next = nullptr;
    }*/
    VertexsOnLines = zigzag(VertexsOnLines);   

    for (int i = 0; i < VertexsOnLines.size(); i++) {
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





