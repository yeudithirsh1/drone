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
Node* insert(Node* node, int key) {
    if (!node) return createNode(key);

    if (key < node->key)
        node->left = insert(node->left, key);
    else if (key > node->key)
        node->right = insert(node->right, key);
    else
        return node; // אין כפילויות

    node->height = 1 + max(getHeight(node->left), getHeight(node->right));

    int balance = getBalanceFactor(node);

    // איזונים
    if (balance > 1 && key < node->left->key)
        return rightRotate(node);

    if (balance < -1 && key > node->right->key)
        return leftRotate(node);

    if (balance > 1 && key > node->left->key) {
        node->left = leftRotate(node->left);
        return rightRotate(node);
    }

    if (balance < -1 && key < node->right->key) {
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
Node* remove(Node* root, int key) {
    if (!root) return root;

    if (key < root->key)
        root->left = remove(root->left, key);
    else if (key > root->key)
        root->right = remove(root->right, key);
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
            root->key = temp->key;
            root->right = remove(root->right, temp->key);
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

// הדפסה לפי סדר ביניים


//int main() {
//    Node* root = nullptr;
//
//    root = insert(root, 1);
//    root = insert(root, 2);
//    root = insert(root, 4);
//    root = insert(root, 5);
//    root = insert(root, 6);
//    root = insert(root, 3);
//
//    cout << "Inorder traversal: ";
//    inOrder(root);
//    cout << endl;
//
//    root = remove(root, 4);
//    cout << "After deleting 4: ";
//    inOrder(root);
//    cout << endl;
//
//    return 0;
//}
