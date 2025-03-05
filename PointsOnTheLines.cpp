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
//    int i, k, j = NULL, l, t = 0, m = graph[graph.size() - 1].size() - 1, n = graph[graph.size() - 1].size() - 1, s = 2, length = min(graph[0].size(), graph[graph.size() - 1].size()), length2 = 0;
//    bool flag = false, d1 = true, d2 = true, b= false;
//
//    // לולאה עיקרית שמבצעת את הסריקה
//    for (i = 0, k = graph.size() - 1; i < graph.size() && k >= 0; i++, k--)
//    {
//        // חיבור בין נקודות בצלע הנוכחית
//        for (j = t; j < length && n >= 0; j++)
//        {
//            cout << i << "," << k << "," << j << "," << t << "," << n << "," << d2 << "," << length << "\n";
//            if (d2)
//            {
//                if (j + 1 < graph[i].size())
//                    graph[i][j].next = &graph[i][j + 1];
//                else
//                {
//                    if (i + 1 < graph.size())
//                    {
//                        graph[i][j].next = &graph[i + 1][0];
//                        b = true;
//                    }
//                }
//                d2 = false;
//            }
//            else
//            {
//                if (k < graph.size() ) 
//                {
//                    graph[i][j].next = &graph[k][n];
//                }
//                d2 = true;
//				if (n - 2 >= 0)
//                    n-=2;
//                else
//                {
//                    if (n = 0)
//                        n = graph[k - 1].size() - 2;
//                    else
//                        n = graph[k - 1].size() - 1;
//                }
//            }
//        }
//        cout << "------------------\n";
//        // חיבור בין הנקודות בצלע השנייה בכיוון ההפוך
//        for (l = m; l >= length2 && s <= graph[i].size(); l--)
//        {
//            cout << i << "," << k << "," << l << "," << m << "," << s << "," << d1 << "\n";
//            if (d1)
//            {
//                if (l - 1 > 0)
//                    graph[k][l].next = &graph[k][l - 1];
//                else
//                {
//                    if (k > 0)
//                    {
//                        graph[k][l].next = &graph[k - 1][graph[k - 1].size() - 1];
//                        flag = true;
//                    }
//                }
//                d1 = false;
//            }
//            else
//            {
//                if (s < graph[i].size() && k < graph.size() && s < graph[i].size())
//                    graph[k][l].next = &graph[i][s];
//                d1 = true;
//                if (s + 2 <= graph[i].size() - 1)
//                    s++;
//                else
//                {
//                    if (s == graph[i].size() - 1)
//                        s = 1;
//                    else
//                        s = 0;
//                }
//            }
//        }
//        cout << "------------------\n";
//
//        // עדכון מצב כדי להחליף צלע
//        if (j < graph[i].size() && ++l == 0)
//        {
//            /*cout << i << "," << k << "," << j << "," << t << "," << n << "," << d2 << "," << length << "\n";
//            cout << i << "," << k << "," << l << "," << m << "," << s << "," << d1 << "," << length << "\n";
//            cout << "------------------\n";*/
//            t = j;
//            if ((graph[i].size() - j) <= graph[k - 1].size())
//                length = j + (graph[i].size() - j);
//            else
//                length = graph[k - 1].size();
//            i--;
//            m = graph[k - 1].size() - 1;
//            length2 = (graph[i].size() - j);
//            //cout << i << "," << k << "," << j << "," << t << "," << n << "," << d2 << "," << length << "\n";
//            //cout << i << "," << k << "," << l << "," << m << "," << s << "," << d1 << "," << length << "\n";
//            //cout << "------------------\n";
//        }
//        else
//        {
//            if (l > 0 && j == graph[i].size())
//            {
//                k--;
//                m = l;
//                if ((graph[k].size() - l) <= graph[i + 1].size())
//                    length = graph[k].size() - l;
//                else
//                    length = graph[i + 1].size();
//                t = 0;
//            }
//            else
//            {
//                if (j == graph[i].size() && l == graph[k].size())
//                {
//                    t = 0;
//                    m = graph[k - 1].size() - 1;
//
//                    if (b)
//                        s = 1;
//                    else
//                        s = 0;
//                    if (flag)
//                        n = graph[k - 1].size() - 2;
//                    else
//                        n = graph[k - 1].size() - 1;
//                }
//            }
//        }
//    }
//
//    return graph; // החזרת הגרף לאחר סיום החיבורים
//}
