#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "PointInSpace.h"
#include "lidaeCameraSynchronization.h"
#include "LIDAR.h"
#include <opencv2/core/types.hpp>
#include "imageToGpsConverter.h"
#include "Geohash.h"

using namespace std;

//אוביקט שזוהה ע"י לידאר
struct DetectedObject {
    vector<Point> lidar_points;
};

//מייצג פריים בזמן מסוים
struct Frame {
    vector<DetectedObject> objects;
    double timestamp;
};


struct ObjectTrack {
	int id; // מזהה ייחודי של האובייקט
	Point last_position; // המיקום האחרון שבו האובייקט נראה
	Point initial_position; // המיקום בו האובייקט זוהה לראשונה
    double last_timestamp; //הזמן שבו האובייקט זוהה בפעם האחרונה
    int missed_frames = 0; //כמה פריימים רצופים האובייקט לא זוהה
    bool is_valid = true; //האם האובייקט עדיין במעקב

    double first_visible_time = -1; //הזמן בו האוביקט הופיע לראשונה
	bool was_alerted = false; // האם נשלחה התראה על האובייקט הזה
};

int next_id = 0; //מונה המספק מזהה ייחודי לכל אובייקט חדש שמתחלים לעקוב אחריו
int max_missed_frames = 5; //מספר המקסימלי של פריימים רצופים שבהם מותר לא לזהות אובייקט

//המרחק המקסימלי (במטרים למשל) שמותר בין מיקום קודם של 
// אובייקט למיקום חדש כדי שנחשיב את זה כאותו אובייקט
float max_tracking_distance = 3.0f;


 
// פונקציה לחישוב מרחק בין שני נקודות
float distance(const Point& a, const Point& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// חישוב מרכז הכובד של נקודות מסוג Point
Point computeCentroid(const vector<Point>& points) {
    Point sum{ 0, 0, 0 };
    for (const auto& p : points) {
        sum.x += p.x;
        sum.y += p.y;
        sum.z += p.z;
    }
    float count = (float)points.size();
    sum.x /= count;
    sum.y /= count;
    sum.z /= count;
    return sum;
}

// מימוש פשוט של Hungarian Algorithm
vector<int> hungarianAlgorithm(const vector<vector<float>>& cost_matrix) {
    int n = (int)cost_matrix.size();
    int m = (int)cost_matrix[0].size();

    vector<int> u(n + 1, 0), v(m + 1, 0);
    vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
		p[0] = i; //הווקטור P אומר לאיזה שורה שייכנו את העמודה j
        int j0 = 0;
        vector<float> minv(m + 1, numeric_limits<float>::max());
        vector<char> used(m + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            float delta = numeric_limits<float>::max();
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    float cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    vector<int> assignment(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            assignment[p[j] - 1] = j - 1;
        }
    }
    return assignment;
}

void trackObjects(const Frame& frame, vector<ObjectTrack>& tracked_objects, vector<Point> detected_centroids) {
    
    int n = tracked_objects.size(); // מספר האובייקטים שנמצאים כבר במעקב
	int m = detected_centroids.size(); //מספר האובייקטים שזוהו בפריים הנוכחי


    // יצירת מטריצה בגודל n*m כל ערך מיצג מרחק התאמה ביו אובייקט במעקב לנקודת זיהוי חדשה
    vector<vector<float>> cost_matrix(n, vector<float>(m, max_tracking_distance + 1.0f));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            float dist = distance(tracked_objects[i].last_position, detected_centroids[j]);
            if (dist <= max_tracking_distance) {
                cost_matrix[i][j] = dist;
            }
        }
    }

    vector<int> assignment = hungarianAlgorithm(cost_matrix);
    vector<bool> matched(m, false);

    for (int i = 0; i < n; ++i) {
        int j = assignment[i];
        if (j != -1 && cost_matrix[i][j] <= max_tracking_distance) {
            tracked_objects[i].last_position = detected_centroids[j];
            tracked_objects[i].last_timestamp = frame.timestamp;
            tracked_objects[i].missed_frames = 0;
            matched[j] = true;

            if (tracked_objects[i].first_visible_time < 0) {
                tracked_objects[i].first_visible_time = frame.timestamp;
            }

            double duration = frame.timestamp - tracked_objects[i].first_visible_time;
            if (duration >= 10.0 && !tracked_objects[i].was_alerted) {
                cout << "" << endl;
                tracked_objects[i].was_alerted = true;
                return; // עצירת הפונקציה ברגע שיש התראה
            }
        }
        else {
            tracked_objects[i].missed_frames++;
            // אם לא זוהה כבר 10 שניות מאז הפעם האחרונה שנראה
            if (frame.timestamp - tracked_objects[i].last_timestamp >= 10.0) {
                tracked_objects[i].is_valid = false; // סימון למחיקה
            }
            tracked_objects[i].first_visible_time = -1;
            if (tracked_objects[i].missed_frames > max_missed_frames) {
                tracked_objects[i].is_valid = false;
            }
        }
    }

    for (int j = 0; j < m; ++j) {
        if (!matched[j]) {
            ObjectTrack new_track;
            new_track.id = next_id++;
            new_track.last_position = detected_centroids[j];
            new_track.initial_position = detected_centroids[j];
            new_track.last_timestamp = frame.timestamp;
            new_track.missed_frames = 0;
            new_track.is_valid = true;
            new_track.first_visible_time = frame.timestamp;
            new_track.was_alerted = false;

            tracked_objects.push_back(new_track);
        }
    }

    // מחיקה של אובייקטים לא תקפים
    tracked_objects.erase(
        remove_if(tracked_objects.begin(), tracked_objects.end(),
            [](const ObjectTrack& t) { return !t.is_valid; }),
        tracked_objects.end());
}



void handlePerson( const string& calibration_file, BoundingBox boundingBox, LIDAR& lidar, Drone& drone) {
     
    vector<Point> cloud = lidar.getQ();
    vector<cv::Point3f> points3f;
    points3f.reserve(cloud.size());

    for (const auto& pt : cloud) {
        points3f.emplace_back(cv::Point3f(pt.x, pt.y, pt.z));
    }

    float distance =  Lidar_to_camera_ratio(calibration_file, points3f, boundingBox).avg_distance;
    CameraCalibration cameraCalibration;
    cameraCalibration = loadCalibrationFromFile(calibration_file);
    pair<float, float> coords = yoloToGPS(boundingBox.x_center, boundingBox.y_center, distance, calibration_file, drone);
    string str = encodeGeohash(coords.first, coords.second, 12);



}

void printTerroristDetectedMessage() {
    cout << "זוהה מחבל" << endl;
}
