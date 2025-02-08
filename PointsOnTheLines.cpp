//vector<Point> generatePointsOnLine(Point p1, Point p2, double r) {
//    vector<Point> points;
//    double dx = p2.x - p1.x;
//    double dy = p2.y - p1.y;
//    double distance = sqrt(dx * dx + dy * dy);
//    int n = ceil(distance / r);
//    double x = p1.x, y = p1.y, m, A, B, C, b;
//    int nill = p2.x - p1.x;
//    m = (p1.y - p2.y) / (p1.x - p2.x);
//    b = p1.y - m * p1.x;
//    for (int d = 0; d < n; d++)
//    {
//        Point P = { x, y, p1.z, nullptr };
//        if (isOnLine(p1, p2, P))
//            points.push_back({ x, y, p1.z, nullptr });
//        A = atan(m);
//        B = (r / sin(90)) * sin(A);
//        C = sqrt(r * r - B * B);
//        if (m < 0 && nill != 0)
//        {
//            x += B;
//            y += C;
//        }
//        else
//        {
//            if (m > 0 && nill != 0)
//            {
//                x -= B;
//                y -= C;
//            }
//            else
//            {
//                if (p1.x == p2.x && p1.y != p2.y)
//                {
//                    if (p1.y < p2.y)
//                        y += r;
//                    else
//                        y -= r;
//                }
//                else
//                {
//                    if (p1.x < p2.x)
//                        x += r;
//                    else
//                        x -= r;
//                }
//            }
//        }
//    }
//    return points;
//}
  //graph[1][1].next = &graph[1][3];
    //cout << graph[1][1].next->x << ", " << graph[1][1].next->y << ", " << graph[1][1].next->z;

    //int j, l, t, m;
    //bool flag = false;
    //for (int i = 0, k = graph.size()-1; i < graph.size(), k >= 0; i++, k--)
    //{
    //    if (i>0)
    //    {
    //       if (flag)
    //           t = 1;
    //       else
    //           m = min(graph[i].size(), graph[k].size());
    //    }
    //    else
    //    {
    //        t = 1;
    //        m = min(graph[i].size(), graph[k].size());
    //    }
    //    for (j = t; j <= min(graph[i].size(), graph[k].size()); j++)
    //    {
    //        if (j % 2 == 0)
    //            graph[i][j].next = &graph[i][j + 1];
    //        else
    //            graph[i][j].next = &graph[k][j];
    //    }
    //    for (l = m-1; l > 0; l--)
    //    {
    //        if (l % 2 != 0)
    //            graph[k][l].next = &graph[k][l+1];
 /*           else
                graph[k][l].next = &graph[i][l];
        }
        if (j < graph[i].size() && l== graph[k].size())
        {
            i--;
            t = j;
            flag = false;
        }
        else
        {
            if (l < graph[k].size() && j== graph[i].size())
            {
                k--;
                m = l;
                flag = true;
            }

        }
    }*/
//vector<vector<Point>> zigzag(vector<vector<Point>> edges)
//{
//    if (edges.size() < 2) return edges; // אין מספיק צלעות לזיגזג
//    size_t numEdges = edges.size();
//    bool goingUp = true; // נתחיל מהכיוון הראשון (למעלה או למטה)
//
//    for (size_t i = 0; i < numEdges - 1; ++i) { // מעב
//        vector<Point>& currentEdge = edges[i];
//        vector<Point>& nextEdge = edges[i + 1];
//        size_t curSize = currentEdge.size();
//        size_t nextSize = nextEdge.size();
//
//        size_t curIdx = goingUp ? 0 : curSize - 1; // ה
//        size_t nextIdx = goingUp ? 0 : nextSize - 1;
//
//        while (curIdx < curSize && nextIdx < nextSize) {
//            currentEdge[curIdx].next = &nextEdge[nextIdx]; // חיבור לנקודה ה
//
//            // זזים בכיוון הנוכחי (למעלה/למטה)
//            if (goingUp) {
//                if (++curIdx >= curSize) break;
//            }
//            else {
//                if (curIdx == 0) break;
//                --curIdx;
//            }
//
//            // חיבור לנקודה הבאה באותה צלע
//            nextEdge[nextIdx].next = &currentEdge[curIdx];
//
//            // זזים בצלע המקבילה
//            if (goingUp) {
//                if (++nextIdx >= nextSize) break;
//            }
//            else {
//                if (nextIdx == 0) break;
//                --nextIdx;
//            }
//        }
//
//        goingUp = !goingUp; // הפוך כיוון לצלעות הבאות
//    }
//    return edges;
//}
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
using namespace std;

// הגדרה של קבועים עבור הרחפן
const float TAKEOFF_SPEED = 5.0; // מהירות ההמראה של הרחפן במטרים לשנייה
const float LANDING_SPEED = -2.0; //(מהירות הנחיתה של הרחפן (שלילית כי הרחפן יורד
const float MAX_ALTITUDE = 100.0; // גובה טיסה מקסימלי

class Drone {
private:
	float altitude; // גובה נוכחי של הרחפן
	bool is_flying; // האם הרחפן נמצא במצב טיסה

public:
    Drone()  
    { 
		altitude = 0.0f; // אתחול הגובה לגובה פני הקרקע
		is_flying = false; // הרחפן לא במצב טיסה
    }

    // הדמיית המראה על ידי הגדלת גובה בהדרגה עד הגעה למקסימום
    void takeOff() {
        is_flying = true;
        while (altitude < MAX_ALTITUDE) {
            altitude += TAKEOFF_SPEED; // הגדל גובה
            cout << "Ascending.Current altitude : " << altitude << "m";
                this_thread::sleep_for(chrono::seconds(1)); // Wait for a second.
        }
        cout << "Drone reached maximum altitude of"  << MAX_ALTITUDE << "m.";
    }

    // Simulate landing by decreasing altitude until the drone reaches the ground.
    void land() {
        while (altitude > 0) {
            altitude += LANDING_SPEED; // Decrease altitude.
            cout << "Descending.Current altitude : " << altitude << "m";
                this_thread::sleep_for(chrono::seconds(1)); // Wait for a second.
        }
        is_flying = false;
        cout << "Drone has landed.";
    }

    // Function to fly the drone to a specified altitude.
    void flyToAltitude(float target_altitude) {
        if (!is_flying) {
            cout << "Drone is not flying.Can't change altitude.";
                return;
        }

        if (target_altitude > MAX_ALTITUDE) {
            cout << "Target altitude exceeds maximum safe operating altitude.";
                return;
        }

        float direction = target_altitude > altitude ? 1 : -1;
        while (altitude != target_altitude) {
            altitude += direction; // Ascend or descend.
            cout << "Flying.Current altitude : " << altitude << "m";
                this_thread::sleep_for(chrono::seconds(1)); // Wait for a second.
        }
        cout << "Reached target altitude of " << target_altitude << "m.";
    }
};

int main() {
    Drone drone;
    drone.takeOff();
    drone.flyToAltitude(50.0f);
    drone.land();
    return 0;
}