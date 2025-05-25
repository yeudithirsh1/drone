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
using namespace Eigen;

typedef struct {
    Eigen::Matrix4f trans;
    vector<float> distances;
    int iter;
}  ICP_OUT;

typedef struct {
    vector<float> distances;
    vector<int> indices;
} NEIGHBOR;

Matrix4f best_fit_transform(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B);

ICP_OUT icp(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, int max_iterations = 20, int tolerance = 0.001);

// throughout method
NEIGHBOR nearest_neighbot(const Eigen::MatrixXf& src, const Eigen::MatrixXf& dst);
float dist(const Eigen::Vector3f& pta, const Eigen::Vector3f& ptb);

#endif

