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

//{

























//void IMUSensor::calculateSpeed(DrivingScenarios& carpoint)
//{
//    double prevSpeed = 0.0, acceleration;
//    string line;
//    bool isRunning = true;
//
//    while (isRunning)
//    {
//        ifstream inputFile("src/IMUsensor.txt");
//        if (!inputFile.is_open())
//        {
//            cerr << "Error opening file." << endl;
//            return;
//        }
//
//        getline(inputFile, line);
//        stringstream iss(line);
//        double ax, ay, az, gx, gy, gz;
//        iss >> ax >> ay >> az >> gx >> gy >> gz; // קריאת המהירויות מכל הצירים
//
//        double dt = GettimeSensor(); // הפרש הזמן בין דגימות
//        speedX += ax * dt;
//        speedY += ay * dt;
//        speedZ += az * dt;
//        double speed = sqrt(speedX * speedX + speedY * speedY + speedZ * speedZ); 
//
//        acceleration = sqrt(ax * ax + ay * ay + az * az); // חישוב התאוצה
//        carpoint.SetaccelerationSpeed(acceleration);
//        carpoint.SetcurrentSpeed(speed);
//
//        double distance = (speed * 1000 / 3600 * dt); // חישוב המרחק שעבר הרחפן
//        carpoint.Setdistance(distance);
//
//        cout << "Distance covered in current iteration: " << distance << " meters" << endl;
//
//        prevSpeed = speed; // עדכון מהירות קודמת
//        inputFile.close();
//
//        this_thread::sleep_for(chrono::seconds(1)); // השהייה של שנייה בין מחזורים
//    }
//}
//
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <string>

void openCamera() {
    cv::VideoCapture cap(0); // פותח את המצלמה (0 היא המצלמה הראשונה המחוברת)

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera!" << std::endl;
    }

    // הגדרת הנתיב לתיקייה לשמירת התמונות
    std::string folderPath = "C:/Users/This User/Documents/project/תמונות/"; // הנתיב לתיקייה
    cv::Mat frame;
    while (true) {
        cap >> frame; // קורא פריים מהמצלמה
        if (frame.empty()) {
            std::cerr << "Error: Empty frame captured!" << std::endl;
            break;
        }

        // קבלת הזמן הנוכחי בפורמט של תאריך ושעה
        std::time_t now = std::time(0);
        std::tm ltm;
        localtime_s(&ltm, &now); // שימוש ב-localtime_s במקום localtime
        std::ostringstream filename;
        filename << folderPath << "frame_" << 1900 + ltm.tm_year << std::setw(2) << std::setfill('0') << 1 + ltm.tm_mon
            << std::setw(2) << std::setfill('0') << ltm.tm_mday << "_"
            << std::setw(2) << std::setfill('0') << ltm.tm_hour
            << std::setw(2) << std::setfill('0') << ltm.tm_min
            << std::setw(2) << std::setfill('0') << ltm.tm_sec << ".jpg"; // שמירת התמונה עם שם לפי הזמן

        // שומר את התמונה בתיקייה שצוינה
        cv::imwrite(filename.str(), frame);

        // מציג את הווידאו בחלון
        cv::imshow("Camera Feed", frame);

        // לחיצה על ESC תסגור את החלון
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
}




