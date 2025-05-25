#pragma once  
#include <vector>

using namespace std;  

struct Vertex  
{  
   float x, y, z;  
   Vertex* next;  

   Vertex(float x_val, float y_val, float z_val, Vertex* next_val = nullptr) :  
       x(x_val), y(y_val), z(z_val), next(next_val) {}  
};  

class nevigation  
{  
 private:  
     Vertex vertex;  

 public:  
   nevigation(float x_val, float y_val, float z_val, Vertex* next_val) :  
       vertex(x_val, y_val, z_val, next_val) {}  
};


float crossProduct(const Vertex &a, const Vertex &b, const Vertex &c);

void convexHull(vector<Vertex>& Vertexs, vector<Vertex>& hall);

vector<pair<Vertex, Vertex>> createEdges(vector<Vertex> &Vertexs);

vector<Vertex> generateVertexsOnLine(Vertex A, Vertex B, float step, bool flag);

vector<vector<Vertex>> zigzag(vector<vector<Vertex>> graph);

vector<vector<Vertex>> graphNavigationPath(vector<Vertex> &Vertexs, float fieldView);

bool isOnLine(Vertex A, Vertex B, Vertex P);


