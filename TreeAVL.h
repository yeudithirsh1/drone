#pragma once

#include "PointInSpace.h"

struct Node {
    Point value;
    Node* left;
    Node* right;
    Node* parent;
    int height;
};

int getHeight(Node* n);
Node* createNode(Point value);
int max(int a, int b);
int getBalanceFactor(Node* n);
Node* rightRotate(Node* y);
Node* leftRotate(Node* x);
Node* insertWithParent(Node* node, Point value, Node* parent);
Node* insert(Node* tree, Point value);
Node* minValueNode(Node* node);
Node* removeNode(Node* root, Point value);


