#include "KDTree.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include "PointInSpace.h"

using namespace std;


KDTree::KDTree() : valid(false) {}

KDTree::KDTree(vector<Point>& cloud, int depth) {

    //אם אין נקודות בכלל
    if (cloud.empty()) {
        valid = false;
        return;
    }

    int axis = depth % 3; // משמש כדי לקבוע באיזה ציר תתבצע החלוקה הנוכחית של הנקודות בעץ KD.

	// פונקציית השוואה המשמשת למיון הנקודות לפי הציר הנוכחי
    auto comp = [axis](const Point& a, const Point& b) {
        if (axis == 0) return a.x < b.x;
        if (axis == 1) return a.y < b.y;
        return a.z < b.z;
        };

    
    vector<Point> sorted_points = cloud;

	// מיון הנקודות לפי הציר הנוכחי
    sort(sorted_points.begin(), sorted_points.end(), comp);

    // בוחרים את הנקודה האמצעית אחרי המיון – היא תהיה הצומת הנוכחי
    int mid = sorted_points.size() / 2;
    point = sorted_points[mid];
    valid = true;

	vector<Point> left_pts(sorted_points.begin(), sorted_points.begin() + mid);// כל הנקודות שקטנות מהציר הנוכחי עד לאמצע
	vector<Point> right_pts(sorted_points.begin() + mid + 1, sorted_points.end()); // כל הנקודות שגדולות מהציר הנוכחי אחרי האמצע

    //אם קיימות נקודות בצד שמאל בונים תת עץ שמאלי רקורסיבי
    if (!left_pts.empty())
        left = make_unique<KDTree>(left_pts, depth + 1);
	//אם קיימות נקודות בצד ימין בונים תת עץ ימני רקורסיבי
    if (!right_pts.empty())
        right = make_unique<KDTree>(right_pts, depth + 1);
}


KDTree::KDTree(MatrixXf& points, vector<int>& indices, int depth) {

	//אם אין נקודות בכלל
    if (points.rows() == 0) {
        valid = false;
        return;
    }

    int axis = depth % 3;// משמש כדי לקבוע באיזה ציר תתבצע החלוקה הנוכחית של הנקודות בעץ KD.

	// מיון האינדקסים של הנקודות לפי הציר הנוכחי
    vector<size_t> order(points.rows());
    iota(order.begin(), order.end(), 0);

    sort(order.begin(), order.end(), [&](size_t i, size_t j) {
        return points(i, axis) < points(j, axis);
        });

    MatrixXf sorted_points(points.rows(), 3);
    vector<int> sorted_indices(indices.size());
    for (size_t i = 0; i < order.size(); ++i) {
        sorted_points.row(i) = points.row(order[i]);  //הנקודות ממוינות לפי הציר הנוכחי
        sorted_indices[i] = indices[order[i]];		//האינדקסים המקוריים של הנקודות לפני המיון כאשר האינדקסים ממוינים לפי הסדר

    }

    int mid = sorted_points.rows() / 2;	//בחירת הנקודה האמצעית שתהיה הצומת הנוכחי
    point = { sorted_points(mid, 0), sorted_points(mid, 1), sorted_points(mid, 2) };	//הצבת הנקודה האמצעית כצומת הנוכחי
    index = sorted_indices[mid];	//הצבת האינדקס המקורי של הנקודה האמצעית כצומת הנוכחי
    valid = true;

	//אם יש נקודות בתת העץ השמאלי
    if (mid > 0) {
        MatrixXf left_pts = sorted_points.topRows(mid);		//הצבת תת העץ השמאלי עם הנקודות שממוקמות לפני האמצע
        vector<int> left_indices(sorted_indices.begin(), sorted_indices.begin() + mid);		//הצבת האינדקסים המקוריים של הנקודות בתת העץ השמאלי
        left = make_unique<KDTree>(left_pts, left_indices, depth + 1);		//יצירת תת העץ השמאלי רקורסיבי

    }

    //אם יש נקודות בתת העץ הימני
    if (mid + 1 < sorted_points.rows()) {
        MatrixXf right_pts = sorted_points.bottomRows(sorted_points.rows() - mid - 1);	//הצבת תת העץ הימני עם הנקודות שממוקמות לפני האמצע
        vector<int> right_indices(sorted_indices.begin() + mid + 1, sorted_indices.end()); //הצבת האינדקסים המקוריים של הנקודות בתת העץ הימני
        right = make_unique<KDTree>(right_pts, right_indices, depth + 1); //יצירת תת העץ הימני רקורסיבי
    }
}


bool KDTree::nearest(const Vector3f& target, int& nearest_index_out, float& dist_sq_out) {
	if (!valid) return false; // אם העץ לא תקין, אין אפשרות לבצע חיפוש

	dist_sq_out = std::numeric_limits<float>::max(); // אתחול המרחק המרבי האפשרי
	bool found = false; // משתנה כדי לבדוק אם נמצאה נקודה קרובה
	nearestSearch(target, 0, nearest_index_out, dist_sq_out, found); // קריאה לפונקציית החיפוש הקרוב ביותר
	return found; // מחזיר true אם נמצאה נקודה קרובה, אחרת false
}

void KDTree::nearestSearch(const Vector3f& target, int depth, int& best_index, float& best_dist_sq, bool& found) {
    if (!valid) return;	//אם בצומת אין נקודה מפסיקים את החיפוש


	// חישוב המרחק בין הנקודה הנוכחית לנקודת היעד
    float dx = point.x - target(0);
    float dy = point.y - target(1);
    float dz = point.z - target(2);
	float d = dx * dx + dy * dy + dz * dz;// חישוב המרחק המרובע

	if (d < best_dist_sq) { //אם המרחק הנוכחי קטן מהמרחק הטוב ביותר שנמצא עד כה
		best_dist_sq = d; //עדכון המרחק הטוב ביותר
		best_index = index; //עדכון האינדקס של הנקודה הקרובה ביותר
		found = true; //עדכון המשתנה שמצביע על כך שנמצאה נקודה קרובה
    }

	int axis = depth % 3; // קביעת הציר הנוכחי לחיפוש
	float target_val = target(axis); // קבלת הערך של נקודת היעד בציר הנוכחי
	float point_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z; // קבלת הערך של הנקודה הנוכחית בציר הנוכחי

	KDTree* near = (target_val < point_val) ? left.get() : right.get();// קביעת תת העץ הקרוב ביותר לחיפוש
	KDTree* far = (target_val < point_val) ? right.get() : left.get(); // קביעת תת העץ הרחוק ביותר לחיפוש

	if (near) near->nearestSearch(target, depth + 1, best_index, best_dist_sq, found);//קריאה רקורסיבית לתת העץ הקרוב ביותר

	float diff = target_val - point_val; // חישוב ההפרש בין הערך של נקודת היעד לערך של הנקודה הנוכחית בציר הנוכחי
	if (diff * diff < best_dist_sq && far) // אם ההפרש בריבוע קטן מהמרחק הטוב ביותר שנמצא עד כה, נמשיך לחפש בתת העץ הרחוק ביותר
		far->nearestSearch(target, depth + 1, best_index, best_dist_sq, found); // קריאה רקורסיבית לתת העץ הרחוק ביותר
}

// פונקציה לחיפוש נקודות בטווח מסוים סביב נקודת יעד
vector<Point> KDTree::radiusSearch(Point& target, float radius){
    vector<Point> result;
	float radiusSq = radius * radius; // חישוב ריבוע הרדיוס כדי להשוות עם מרחקים מרובעים
	radiusSearchRec(target, radiusSq, 0, result); // קריאה לפונקציה רקורסיבית לחיפוש נקודות בטווח
    return result;
}

void KDTree::radiusSearchRec(Point& target, float radiusSq, int depth, vector<Point>& result){
    if (!valid) return;//אם בצומת אין נקודה מפסיקים את החיפוש

    // חישוב המרחק בין הנקודה הנוכחית לנקודת היעד
    float dx = point.x - target.x;
    float dy = point.y - target.y;
    float dz = point.z - target.z;
    float d = dx * dx + dy * dy + dz * dz;// חישוב המרחק המרובע

	if (d <= radiusSq) // אם המרחק המרובע קטן או שווה לרדיוס בריבוע, מוסיפים את הנקודה לתוצאה
        result.push_back(point);

	int axis = depth % 3; // קביעת הציר הנוכחי לחיפוש


    //קובעים את הציר לפי העומק ומוצאים את הערכים של נקודת היעד והנקודה בצומת הנוכחית
    float target_val = (axis == 0) ? target.x : (axis == 1) ? target.y : target.z;
    float point_val = (axis == 0) ? point.x : (axis == 1) ? point.y : point.z;

    //מחשבים את ההפרש בין הערכים בציר הנוכחי
    float diff = target_val - point_val;

	if (diff < 0) { // אם ההפרש של נקודת היעד מהציר הנוכחי הוא שלילי, מחפשים קודם בתת העץ השמאלי
        if (left) left->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && right)
            right->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
    else {
        if (right) right->radiusSearchRec(target, radiusSq, depth + 1, result);
        if (diff * diff <= radiusSq && left)
            left->radiusSearchRec(target, radiusSq, depth + 1, result);
    }
}
