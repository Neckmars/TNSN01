#include <iostream>
#include <vector>
#include <bits/stdc++.h>
//#include "math.h"

#define NORTH 0
#define WEST 1
#define SOUTH 2
#define EAST 3

using namespace std;

// int currentOrientation = WEST;

const int mazeX = 7;
const int mazeY = 7;

enum direction {
    RIGHT = 1, LEFT = -1, FORWARD = 0, UTURN = 2
};

class Node {
public:
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;

    // Order: North, South, West, East
    int northLimit;
    int southLimit;
    int westLimit;
    int eastLimit;


    Node *parent = nullptr;

    Node() {};

    void SetParams(int x, int y, int northLimit, int southLimit, int westLimit, int eastLimit) {
        this->x = x;
        this->y = y;

        this->northLimit = northLimit;
        this->southLimit = southLimit;
        this->westLimit = westLimit;
        this->eastLimit = eastLimit;
    }

    void resetParams(){
        this->h = 0;
        this->g = 0;
        this->parent = nullptr;
    }

    int f() {
        return g + h;
    }
};

struct importantVectors {
        vector<Node*> v1;
        vector<int> v2;
    }; 

struct dobleInt {
    int value1;
    int value2;
};

struct ordersAndIndex {
    vector<int> order;
    int index;
};

Node MapNodes[mazeY][mazeX];

Node* intersectionNodes[14]= {&MapNodes[1][2], &MapNodes[1][1],
                                &MapNodes[1][1], &MapNodes[6][3],
                                &MapNodes[6][3], &MapNodes[4][4],
                                &MapNodes[3][4], &MapNodes[1][1],
                                &MapNodes[1][2], &MapNodes[2][3],
                                &MapNodes[2][4], &MapNodes[2][4],
                                &MapNodes[2][3], &MapNodes[1][2]};
Node* finalCylinderNode = &MapNodes[2][6];

void createMap();
void showConections();
void resetMap();
vector<Node*> GetNeighbors(Node*) ;
vector<Node*> AStar_Algorithm(Node*, Node*);
int ManhattanDistance(int, int, int, int);

importantVectors translateNodes2Orders(vector<Node*>, int);
dobleInt getNextOrder(Node* , Node*, int);

ordersAndIndex Astar_robot(int currentIntersection, int distanceFromIntersection, int orientation);

/*int main() {
    createMap();
    showConections();

    int currentOr = SOUTH;
    Node* start = &MapNodes[3][0];
    Node* goal = &MapNodes[2][6];

    vector<Node*> path = AStar_Algorithm(start, goal);

    auto [finalNodes, finalOrders] = translateNodes2Orders(path, currentOr);

    cout << "Intersaction Nodes \n";
    
    for (int i = 0; i < finalNodes.size(); i++) {
        Node* p = finalNodes[i];
        int order = finalOrders[i];

        cout << "(x=" << p->x << ", y=" << p->y << ")  " << "\n";
        cout << "Order: " << order << "\n";
    }
    
    return 0;
}*/

void createMap() {
    // Order: North, South, West, East
    // Generate first line
    MapNodes[0][0].SetParams(0, 0, 0, 0, 0, 1);
    MapNodes[0][1].SetParams(1, 0, 1, 0, 1, 0);
    MapNodes[0][2].SetParams(2, 0, 1, 0, 0, 1);
    MapNodes[0][3].SetParams(3, 0, 0, 0, 1, 0);
    MapNodes[0][4].SetParams(4, 0, 1, 0, 0, 1);
    MapNodes[0][5].SetParams(5, 0, 0, 0, 1, 1);
    MapNodes[0][6].SetParams(6, 0, 1, 0, 1, 0);

    // Generate second line
    MapNodes[1][0].SetParams(0, 1, 1, 0, 0, 1);
    MapNodes[1][1].SetParams(1, 1, 1, 1, 1, 1);
    MapNodes[1][2].SetParams(2, 1, 0, 1, 1, 1);
    MapNodes[1][3].SetParams(3, 1, 1, 0, 1, 0);
    MapNodes[1][4].SetParams(4, 1, 0, 1, 0, 1);
    MapNodes[1][5].SetParams(5, 1, 1, 0, 1, 0);
    MapNodes[1][6].SetParams(6, 1, 1, 1, 0, 0);

    // Generate third line
    MapNodes[2][0].SetParams(0, 2, 1, 1, 0, 0);
    MapNodes[2][1].SetParams(1, 2, 1, 1, 0, 0);
    MapNodes[2][2].SetParams(2, 2, 0, 0, 0, 1);
    MapNodes[2][3].SetParams(3, 2, 0, 1, 1, 1);
    MapNodes[2][4].SetParams(4, 2, 1, 0, 1, 1);
    MapNodes[2][5].SetParams(5, 2, 0, 1, 1, 0);
    MapNodes[2][6].SetParams(6, 2, 1, 1, 0, 0);
    
    // Generate fourth line
    MapNodes[3][0].SetParams(0, 3, 1, 1, 0, 0);
    MapNodes[3][1].SetParams(1, 3, 1, 1, 0, 0);
    MapNodes[3][2].SetParams(2, 3, 1, 0, 0, 1);
    MapNodes[3][3].SetParams(3, 3, 0, 0, 1, 1);
    MapNodes[3][4].SetParams(4, 3, 1, 1, 1, 0);
    MapNodes[3][5].SetParams(5, 3, 1, 0, 0, 1);
    MapNodes[3][6].SetParams(6, 3, 0, 1, 1, 0);

    // Generate fifth line
    MapNodes[4][0].SetParams(0, 4, 1, 1, 0, 0);
    MapNodes[4][1].SetParams(1, 4, 0, 1, 0, 1);
    MapNodes[4][2].SetParams(2, 4, 0, 1, 1, 0);
    MapNodes[4][3].SetParams(3, 4, 1, 0, 0, 1);
    MapNodes[4][4].SetParams(4, 4, 1, 1, 1, 0);
    MapNodes[4][5].SetParams(5, 4, 0, 1, 0, 1);
    MapNodes[4][6].SetParams(6, 4, 0, 0, 1, 0);

    // Generate sixth line
    MapNodes[5][0].SetParams(0, 5, 1, 1, 0, 0);
    MapNodes[5][1].SetParams(1, 5, 1, 0, 0, 1);
    MapNodes[5][2].SetParams(2, 5, 1, 0, 1, 0);
    MapNodes[5][3].SetParams(3, 5, 1, 1, 0, 0);
    MapNodes[5][4].SetParams(4, 5, 0, 1, 0, 1);
    MapNodes[5][5].SetParams(5, 5, 0, 0, 1, 1);
    MapNodes[5][6].SetParams(6, 5, 1, 0, 1, 0);

    // Generate seventh line
    MapNodes[6][0].SetParams(0, 6, 0, 1, 0, 1);
    MapNodes[6][1].SetParams(1, 6, 0, 1, 1, 0);
    MapNodes[6][2].SetParams(2, 6, 0, 1, 0, 1);
    MapNodes[6][3].SetParams(3, 6, 0, 1, 1, 1);
    MapNodes[6][4].SetParams(4, 6, 0, 0, 1, 0);
    MapNodes[6][5].SetParams(5, 6, 0, 0, 0, 1);
    MapNodes[6][6].SetParams(6, 6, 0, 1, 1, 0);

    return;
}

void showConections(){
    Node CurrentNode;
    for(int j = mazeY-1; j >= 0; j--){

        // Verify west and east paths
        for (int i = 0; i < mazeX; i++) {
            CurrentNode = MapNodes[j][i];
            if(CurrentNode.westLimit == 1) cout << "-";
            else cout << " ";
            cout << " o ";
            if(CurrentNode.eastLimit == 1) cout << "-";
            else cout << " ";
        }
        cout << "\n";

        // Verify south path
        for (int i = 0; i < mazeX; i++) {
            CurrentNode = MapNodes[j][i];
            if(CurrentNode.southLimit == 1) cout << "  |  ";
            else cout << "     ";
        }
        cout << "\n";
    } 
    return;
}

void resetMap() {
    for(int j = 0; j < mazeY; j++){
        for (int i = 0; i < mazeX; i++) {
            MapNodes[j][i].resetParams();
        }
    }
}

int ManhattanDistance(int x1, int y1, int x2, int y2) {
    return (abs(x1 - x2) + abs(y1 - y2));
}

vector<Node*> GetNeighbors(Node* thisNode) {
    vector<Node*> vectorOut;
    if (thisNode->northLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y+1][thisNode->x]);
    if (thisNode->southLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y-1][thisNode->x]);
    if (thisNode->westLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y][thisNode->x-1]);
    if (thisNode->eastLimit == 1) vectorOut.push_back(&MapNodes[thisNode->y][thisNode->x+1]);

    return vectorOut;
}

vector<Node*> AStar_Algorithm(Node* StartNode, Node* GoalNode) {
    vector<Node*> open;
    vector<Node*> closed;

    Node* start = StartNode;
    resetMap();
    start->g = 0;
    start->h = ManhattanDistance(StartNode->x, StartNode->y, GoalNode->x, GoalNode->y);
    start->parent = nullptr;
    open.push_back(start);

    while(!open.empty()) {

        // Verify and choose the lowest f
        Node* current = open[0];
        for (Node* n: open) {
            if (n->f() < current->f()) current = n;
        }

        // In case we are in the goal, find the parents of the nodes until get to the start node
        if(current->x == GoalNode->x && current->y == GoalNode->y) {
            vector<Node*> path;
            while(current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            // Reverse the path to obtain the right order
            reverse(path.begin(), path.end());
            // Return final path
            return path;
        }

        // Move the current node to closed
        open.erase(remove(open.begin(), open.end(), current), open.end());
        closed.push_back(current);

        // Identify neighbors and explore them
        vector<Node*> neighbors = GetNeighbors(current);
        for (Node* neighbor : neighbors) {
            int neighbor_g = current->g + 1;
            bool skip = false;
            bool inOpen = false;
            
            // Verify if the node is in the open vector
            for (Node* o : open) {
                // If the current cost is greater or equal than the cost that Node already has,
                // move to the next neighbor 
                if (o == neighbor) {
                    inOpen = true;
                    // Update neighbor only if it finds a better path
                    if (neighbor_g < o->g) {
                        o->g = neighbor_g;
                        o->parent = current;
                    }
                    break;
                }
            }

            // Verify if the node is in the closed vector
            for (Node* c : closed) {
                // If the current cost is greater or equal than the cost that Node already has,
                // move to the next neighbor 
                if (c == neighbor) {
                    skip = true;
                    break;
                }
            }
            if (skip) continue;

            // In case the neighbor is neither in the open nor the closed vector,
            // add it to the open vector
            if (!inOpen) {
                neighbor->g = neighbor_g;
                neighbor->h = ManhattanDistance(neighbor->x, neighbor->y, GoalNode->x, GoalNode->y);
                neighbor->parent = current;
                open.push_back(neighbor);
            }

        }
    }
    return {};
}

// Functions to translate the Node vector into positions and orders
importantVectors translateNodes2Orders(vector<Node*> Path, int originalOrientation) {
    vector<Node*> final_nodes;
    vector<int> final_orders;
    vector<int> tempOrders;
    int n;
    int currentOrientation = originalOrientation;
    int nextOrientation;
    for (int i = 0; i < Path.size()-1; i++) {
        Node* p = Path[i];
        auto [n, nextOrientation] = getNextOrder(Path[i], Path[i+1], currentOrientation);
        tempOrders.push_back(n);
        cout << nextOrientation << " \n";
        currentOrientation = nextOrientation;
    }

    for (int i = 0; i < Path.size()-1; i++) {
        Node* p = Path[i];
        int ord = tempOrders[i];
        if (p->northLimit+p->southLimit+p->eastLimit+p->westLimit > 2) {
            final_nodes.push_back(p);
            final_orders.push_back(ord);
        }
    }

    return importantVectors { final_nodes, final_orders };
}

dobleInt getNextOrder(Node* currentNode, Node* nextNode, int originalOrientation) {
    int change_y = nextNode->y - currentNode->y;
    int change_x = nextNode->x - currentNode->x;

    int currentOrientation = originalOrientation;
    int targetOrientation;
    int order;

    if (change_y == 1)          targetOrientation = NORTH;
    else if (change_y == -1)    targetOrientation = SOUTH;
    else if (change_x == 1)     targetOrientation = EAST;
    else if (change_x == -1)    targetOrientation = WEST;

    int diff = (targetOrientation - currentOrientation + 4) % 4;

    switch (diff) {
    case 0:
        order = FORWARD;
        break;
    case 1:
        order = RIGHT;
        break;

    case 3:
        order = LEFT;
        break;
    
    case 2:
        order = UTURN;
        break;

    default:
        break;
    }
    currentOrientation = targetOrientation;
    return dobleInt {order, currentOrientation};
}

ordersAndIndex Astar_robot(int currentIntersection, int distanceFromIntersection, int orientation){
    // define 2 paths
    // path 1: from the last node to the goal node (distance = path + distanceFromIntersaction)
    // path 2: from the following node, to the (distance = path + (distanceBetweenIntersactions - distanceFromIntersaction))
    // choose the path with the lowest distance
    
    // specialNode will tell us if the robot will cross the node without dark line. If it doesn't cross that node, or if it does it and the order is to go forward, the number will stay at -1
    // Otherwise the number will corespond to the number of the order.

    int specialNode = -1;
    vector<Node*> path1 = AStar_Algorithm(intersectionNodes[currentIntersection], finalCylinderNode);
    vector<Node*> path2 = AStar_Algorithm(intersectionNodes[currentIntersection + 1], finalCylinderNode);

    importantVectors myVectors;

    if (path1.size() > path2.size()) {
        myVectors = translateNodes2Orders(path2, orientation);
    }
    else {
        orientation += 2;
        myVectors = translateNodes2Orders(path1, orientation);       
    }

    auto& [finalNodes, finalOrders] = myVectors;

    for(int i = 0; i < finalNodes.size(); i++) {
        Node* c_node = finalNodes[i];
        int c_order = finalOrders[i];
        if (c_node->x == 4 && c_node->y == 4) {
            if (c_order == 0) finalOrders.erase(finalOrders.begin() + i);
            else specialNode = i;
            break;
        }  
    }

    if (path1.size() <= path2.size()) {
        finalOrders.insert(finalOrders.begin(), 2);
    }

    finalOrders.insert(finalOrders.end(), 0);
    finalOrders.insert(finalOrders.end(), -1);
    finalOrders.insert(finalOrders.end(), -1);


    return ordersAndIndex {finalOrders, specialNode};
}