#include "ICP.h"
#include <iostream>
#include <numeric>
#include "Eigen/Eigen"
#include "KDTree.h"

using namespace std;
using namespace Eigen;

Matrix4f best_fit_transform(const MatrixXf& A, const MatrixXf& B) {
   
    Matrix4f T = MatrixXf::Identity(4, 4);//אתחול מטריצת הטרנספורמציה כמטריצת היחידה

	//אתחול וקטור מרכז המסה של A ושל B
    Vector3f centroid_A(0, 0, 0);
	Vector3f centroid_B(0, 0, 0);

	//יצירת העתק של A ו-B כדי לשמור על הנתונים המקוריים
	MatrixXf AA = A;
	MatrixXf BB = B;

    // חישוב מרכז המסה של A ו-B
    // לולאה לחישוב סכום כל הנקודות במטריצות A ו-B
	// לאחר מכן, מחלקים במספר השורות כדי לקבל את מרכז המסה
	int row = A.rows();

    for (int i = 0; i < row; i++) {
        centroid_A += A.block<1, 3>(i, 0).transpose(); 
        centroid_B += B.block<1, 3>(i, 0).transpose();
    }
    centroid_A /= row;
    centroid_B /= row;

	//מרכוז הנקודות: מחסרים מכל נקודה את מרכז המסה שלה כך שכל קבוצה נעה למרכז המקור  (0,0,0) זה שלב חשוב בתהליך SVD
    for (int i = 0; i < row; i++) {
        AA.block<1, 3>(i, 0) = A.block<1, 3>(i, 0) - centroid_A.transpose();
        BB.block<1, 3>(i, 0) = B.block<1, 3>(i, 0) - centroid_B.transpose();
    }

    //זו מטריצה שמכילה סיכום של המכפלות של הנקודות המקוריות והיעד, אחרי מרכוז H
    MatrixXf H = AA.transpose() * B;

    //רכיבי הפירוק הסינגולריים
    MatrixXf U;
    VectorXf S;
    MatrixXf V; 

	MatrixXf Vt; // מטריצת הטרנספוז של V
    Matrix3f R; //מטריצת סיבוב 
    Vector3f t; //ווקטור הזזה

	// חישוב ה-SVD של מטריצת הקורלציה H
    //מטרת הפירוק: למצוא את הסיבוב R המיטבי שמעביר את A ל־B
    JacobiSVD<MatrixXf> svd(H, ComputeFullU | ComputeFullV);
	U = svd.matrixU();
	S = svd.singularValues();
	V = svd.matrixV();
	Vt = V.transpose();
    R = Vt.transpose() * U.transpose();

    if (R.determinant() < 0) {
        Vt.block<1, 3>(2, 0) *= -1;
        R = Vt.transpose() * U.transpose();
    }

    //ווקטור הזזה
    t = centroid_B - R * centroid_A;

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;

}

ICP_OUT icp(const MatrixXf& A, const MatrixXf& B, int max_iterations, int tolerance) {
    int row = A.rows(); // מספר השורות במטריצה A
    MatrixXf src = MatrixXf::Ones(4, row);
    MatrixXf src3f = MatrixXf::Ones(3, row);
    MatrixXf dst = MatrixXf::Ones(4, row);
    NEIGHBOR neighbor;
    Matrix4f T;
    MatrixXf dst_chorder = MatrixXf::Ones(3, row);
    ICP_OUT result;
    int iter = 0;

    for (int i = 0; i < row; i++) {
        src.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        src3f.block<3, 1>(0, i) = A.block<1, 3>(i, 0).transpose();
        dst.block<3, 1>(0, i) = B.block<1, 3>(i, 0).transpose();
    }

    float prev_error = 0;
    float mean_error = 0;
    bool continue_loop = true;
    int i = 0;

    while (continue_loop && i < max_iterations) {
        neighbor = nearest_neighbot(src3f.transpose(), B);

        for (int j = 0; j < row; j++) {
            dst_chorder.block<3, 1>(0, j) = dst.block<3, 1>(0, neighbor.indices[j]);
        }

        T = best_fit_transform(src3f.transpose(), dst_chorder.transpose());

        src = T * src;
        for (int j = 0; j < row; j++) {
            src3f.block<3, 1>(0, j) = src.block<3, 1>(0, j);
        }

        mean_error = accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / neighbor.distances.size();

        if (abs(prev_error - mean_error) < tolerance) {
            continue_loop = false;  // מפסיקים את הלולאה
        }
        else {
            prev_error = mean_error;
            iter = i + 2;
        }
        i++;
    }

    T = best_fit_transform(A, src3f.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}



//חיפוש של השכן הקרוב ביותר עבור כל נקודה במטריצת src מתוך אוסף נקודות במטריצת dst
NEIGHBOR nearest_neighbot(const MatrixXf& src, const MatrixXf& dst) {

    vector<int> dst_indices(dst.rows());    // יצירת וקטור אינדקסים ל-dst
	iota(dst_indices.begin(), dst_indices.end(), 0); // רצף של מספרים עוקבים מ-0 עד גודל ה-dst

	MatrixXf dst_copy = dst; // יצירת העתק של מטריצת ה-dst כדי לשמור על הנתונים המקוריים
    KDTree tree(dst_copy, dst_indices, 0);    // יצירת עץ KD


    NEIGHBOR neigh;//יצירת מבנה בו יישמרו האינדקסים של השכנים הקרובים ביותר והמרחקים אליהם

	for (int i = 0; i < src.rows(); ++i) {      //מעבר על כל הנקודות במטריצה-src
		Vector3f query = src.row(i).transpose();    //המרת נקודה לווקטור תלת ממדי
		int nearest_idx = -1;      //אתחול האינדקס הקרוב ביותר ל- -1
		float best_dist_sq = numeric_limits<float>::max(); //אתחול המרחק הטוב ביותר בריבוע לערך המקסימלי האפשרי
        tree.nearest(query, nearest_idx, best_dist_sq); //מציאת השכן הקרוב ביותר לנקודה הנוכחית 
		neigh.indices.push_back(nearest_idx);  //שמירת האינדקס של השכן הקרוב ביותר במבנה ה-NEIGHBOR
		neigh.distances.push_back(sqrt(best_dist_sq)); //שמירת המרחק של השכן הקרוב ביותר במבנה ה-NEIGHBOR
    }

    return neigh;
}


float dist(const Vector3f& pta, const Vector3f& ptb) {
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]) + (pta[2] - ptb[2]) * (pta[2] - ptb[2]));
}

