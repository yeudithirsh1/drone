#include <iostream>
#include <algorithm>
#include "PointInSpace.h"
using namespace std;


struct Node {
    Point value;
    Node* left;
    Node* right;
    Node* parent; 
    int height;
};

// פונקציה להחזרת גובה של צומת
int getHeight(Node* n) {
    return n ? n->height : 0;
}

// פונקציה יוצרת צומת חדש
Node* createNode(Point value) {
    Node* node = new Node();
    node->value = value;
    node->left = node->right = nullptr;
    node->parent= nullptr;
    node->height = 1;
    return node;
}

// מקסימום בין שני ערכים
int max(int a, int b) {
    return (a > b) ? a : b;
}

// מקדם איזון
int getBalanceFactor(Node* n) {
    if (!n) return 0;
    return getHeight(n->left) - getHeight(n->right);
}

// סיבוב ימני
Node* rightRotate(Node* y) {
    Node* x = y->left;
    Node* T2 = x->right;

    x->right = y;
    y->left = T2;

    y->height = max(getHeight(y->left), getHeight(y->right)) + 1;
    x->height = max(getHeight(x->left), getHeight(x->right)) + 1;

    return x;
}

// סיבוב שמאלי
Node* leftRotate(Node* x) {
    Node* y = x->right;
    Node* T2 = y->left;

    y->left = x;
    x->right = T2;

    x->height = max(getHeight(x->left), getHeight(x->right)) + 1;
    y->height = max(getHeight(y->left), getHeight(y->right)) + 1;

    return y;
}

// הוספה לעץ AVL
Node* insert(Node* tree, Point value) {
    return insertWithParent(tree, value, nullptr);
}

Node* insertWithParent(Node* node, Point value, Node* parent) {
    if (!node) {
        Node* newNode = createNode(value);
        newNode->parent = parent;
        return newNode;
    }

    if (value < node->value)
        node->left = insertWithParent(node->left, value, node);
    else if (value > node->value)
        node->right = insertWithParent(node->right, value, node);
    else
        return node; // אין כפילויות

    node->height = 1 + max(getHeight(node->left), getHeight(node->right));

    int balance = getBalanceFactor(node);

    // איזונים
    if (balance > 1 && value < node->left->value)
        return rightRotate(node);

    if (balance < -1 && value > node->right->value)
        return leftRotate(node);

    if (balance > 1 && value > node->left->value) {
        node->left = leftRotate(node->left);
        return rightRotate(node);
    }

    if (balance < -1 && value < node->right->value) {
        node->right = rightRotate(node->right);
        return leftRotate(node);
    }

    return node;
}


// פונקציה למציאת מינימום תת־עץ
Node* minValueNode(Node* node) {
    Node* current = node;
    while (current->left != nullptr)
        current = current->left;
    return current;
}

// מחיקת צומת מהעץ
Node* remove(Node* root, Point value) {
    if (!root) return root;

    if (value < root->value)
        root->left = remove(root->left, value);
    else if (value > root->value)
        root->right = remove(root->right, value);
    else {
        // מקרה עם ילד אחד או ללא ילדים
        if (!root->left || !root->right) {
            Node* temp = root->left ? root->left : root->right;
            if (!temp) {
                temp = root;
                root = nullptr;
            }
            else
                *root = *temp;
            delete temp;
        }
        else {
            Node* temp = minValueNode(root->right);
            root->value = temp->value;
            root->right = remove(root->right, temp->value);
        }
    }

    if (!root) return root;

    root->height = 1 + max(getHeight(root->left), getHeight(root->right));

    int balance = getBalanceFactor(root);

    if (balance > 1 && getBalanceFactor(root->left) >= 0)
        return rightRotate(root);

    if (balance > 1 && getBalanceFactor(root->left) < 0) {
        root->left = leftRotate(root->left);
        return rightRotate(root);
    }

    if (balance < -1 && getBalanceFactor(root->right) <= 0)
        return leftRotate(root);

    if (balance < -1 && getBalanceFactor(root->right) > 0) {
        root->right = rightRotate(root->right);
        return leftRotate(root);
    }

    return root;
}

