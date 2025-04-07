#pragma once
#include "Eigen/Eigen"
#include <vector>

#ifndef ICP_H
#define ICP_H

#define N_pt 30    // מערך נתונים של 30 נקודות
#define N_tests 100    // # 100 איטרציות לבדיקה - ניסויים
#define noise_sigma 0.01    // הוספת שגיאה של סטיית תקן
#define translation 0.1     // תרגום מקסימלי של הסט הבדיקה
#define rotation 0.1        // סיבוב מקסימלי (ברדיאנים) של הסט הבדיקה
using namespace std;


typedef struct {
    Eigen::Matrix4d trans;
    vector<float> distances;
    int iter;
}  ICP_OUT;

typedef struct {
    vector<float> distances;
    vector<int> indices;
} NEIGHBOR;

Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

ICP_OUT icp(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, int max_iterations = 20, int tolerance = 0.001);

// throughout method
NEIGHBOR nearest_neighbot(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
float dist(const Eigen::Vector3d& pta, const Eigen::Vector3d& ptb);

#endif

