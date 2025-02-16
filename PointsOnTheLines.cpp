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

