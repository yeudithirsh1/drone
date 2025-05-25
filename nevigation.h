#pragma once  
#include <vector>

using namespace std;  

struct Vertex  
{  
   double x, y, z;  
   Vertex* next;  

   Vertex(double x_val, double y_val, double z_val, Vertex* next_val = nullptr) :  
       x(x_val), y(y_val), z(z_val), next(next_val) {}  
};  

class nevigation  
{  
 private:  
     Vertex vertex;  

 public:  
   nevigation(double x_val, double y_val, double z_val, Vertex* next_val) :  
       vertex(x_val, y_val, z_val, next_val) {}  
};


double crossProduct(const Vertex &a, const Vertex &b, const Vertex &c);

void convexHull(vector<Vertex>& Vertexs, vector<Vertex>& hall);

vector<pair<Vertex, Vertex>> createEdges(vector<Vertex> &Vertexs);

vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, double step, bool flag);

vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph);

vector<vector<Vertex>> graphNavigationPath(vector<Vertex> &Vertexs, double fieldView);

bool isOnLine(Vertex A, Vertex B, Vertex P);


