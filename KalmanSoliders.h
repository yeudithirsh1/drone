#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class KalmanSoliders {
public:
    KalmanSoliders();

    void init(const Vector2f& initial_position);
    void predict();
    void update(const Vector2f& measured_position);
    void updatePosition();
private:
    Vector2f x_;      // מצב: מיקום [x, y, z]
    Matrix2f P_;      // טעות אומדן
    Matrix2f F_;      // מטריצת מעבר מצב (זהות)
    Matrix2f H_;      // מטריצת מדידה (זהות)
    Matrix2f Q_;      // רעש תהליך
    Matrix2f R_;      // רעש מדידה
    Matrix2f I_;      // מטריצת יחידה
};



