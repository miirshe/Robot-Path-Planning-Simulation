#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <stack>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <iomanip>
#include <fstream>
using namespace std;

#define ROW 12
#define COL 24

// ANSI color codes
#define RESET   "\033[0m"
#define GREEN   "\033[32m"
#define RED     "\033[31m"
#define BLUE    "\033[34m"
#define YELLOW  "\033[33m"
#define WHITE   "\033[37m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

// Direction vectors
int dx[] = {-1, 1, 0, 0};
int dy[] = {0, 0, -1, 1};

// Performance metrics structure
struct PerformanceMetrics {
    int nodesExplored;
    int pathLength;
    double executionTime;
    vector<pair<int, int>> path;
    int obstacles;
    int freeSpaces;
};

// Cell structure
struct Node {
    int x, y;
    int g, h;
    Node* parent;

    Node(int x, int y, int g, int h, Node* p = nullptr)
        : x(x), y(y), g(g), h(h), parent(p) {}

    int f() const { return g + h; }
};

// Comparator for priority queue
struct Compare {
    bool operator()(const Node* a, const Node* b) {
        return a->f() > b->f();
    }
};

// Heuristic: Manhattan distance
int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// Check if cell is valid
bool isValid(int x, int y, vector<vector<int>>& maze) {
    return x >= 0 && x < ROW && y >= 0 && y < COL && maze[x][y] != 1;
}

// Clear screen
void clearScreen() {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

// Print statistics panel
void printStats(const PerformanceMetrics& metrics) {
    cout << CYAN << "\n=== Performance Metrics ===\n" << RESET;
    cout << "Nodes Explored: " << metrics.nodesExplored << endl;
    cout << "Path Length: " << metrics.pathLength << endl;
    cout << "Execution Time: " << fixed << setprecision(3) << metrics.executionTime << " ms" << endl;
    cout << "Obstacles: " << metrics.obstacles << endl;
    cout << "Free Spaces: " << metrics.freeSpaces << endl;
    cout << CYAN << "========================\n" << RESET;
}

// Print the maze with colors and a title and legend
void printMaze(const vector<vector<int>>& maze, const PerformanceMetrics& metrics) {
    cout << GREEN << "\n\u25CF Optimal Path Planning for Robot in a Dynamic Environment\n" << RESET;
    cout << GREEN << "\u25A0" << RESET << " Wall (1)  ";
    cout << WHITE << "\u25A1" << RESET << " Space (0)  ";
    cout << RED << "\u25CF" << RESET << " Start (-1)  ";
    cout << BOLD << BLUE << "\u25B2" << RESET << " Target (9)  ";
    cout << YELLOW << "\u25CF" << RESET << " Path (2)\n\n";

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == 1)
                cout << GREEN << "██" << RESET;
            else if (maze[i][j] == -1)
                cout << RED << " R" << RESET;
            else if (maze[i][j] == 9)
                cout << BOLD << BLUE << "▲" << RESET;
            else if (maze[i][j] == 2)
                cout << YELLOW << " ." << RESET;
            else
                cout << "  ";
        }
        cout << endl;
    }
    
    printStats(metrics);
}

// Calculate environment statistics
void calculateEnvironmentStats(const vector<vector<int>>& maze, PerformanceMetrics& metrics) {
    metrics.obstacles = 0;
    metrics.freeSpaces = 0;
    
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == 1) metrics.obstacles++;
            else if (maze[i][j] == 0) metrics.freeSpaces++;
        }
    }
}

// A* Pathfinding
bool aStar(vector<vector<int>>& maze, int sx, int sy, int tx, int ty, PerformanceMetrics& metrics) {
    auto start_time = chrono::high_resolution_clock::now();
    priority_queue<Node*, vector<Node*>, Compare> openSet;
    vector<vector<bool>> closed(ROW, vector<bool>(COL, false));
    metrics.nodesExplored = 0;

    Node* start = new Node(sx, sy, 0, heuristic(sx, sy, tx, ty));
    openSet.push(start);

    while (!openSet.empty()) {
        Node* current = openSet.top(); openSet.pop();
        int x = current->x, y = current->y;
        metrics.nodesExplored++;

        if (closed[x][y]) continue;
        closed[x][y] = true;

        if (x == tx && y == ty) {
            Node* temp = current;
            metrics.pathLength = 0;
            metrics.path.clear();
            
            while (temp->parent != nullptr) {
                if (maze[temp->x][temp->y] == 0) {
                    maze[temp->x][temp->y] = 2;
                    metrics.path.push_back({temp->x, temp->y});
                }
                metrics.pathLength++;
                temp = temp->parent;
            }
            
            auto end_time = chrono::high_resolution_clock::now();
            metrics.executionTime = chrono::duration_cast<chrono::milliseconds>
                                  (end_time - start_time).count();
            return true;
        }

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            if (isValid(nx, ny, maze) && !closed[nx][ny]) {
                Node* neighbor = new Node(nx, ny, current->g + 1, 
                                       heuristic(nx, ny, tx, ty), current);
                openSet.push(neighbor);
            }
        }

        clearScreen();
        printMaze(maze, metrics);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}

// Save performance report to file
void saveReport(const PerformanceMetrics& metrics, int sx, int sy, int tx, int ty) {
    ofstream report("single_target_report.txt");
    report << "=== Single-Target A* Path Planning Performance Report ===\n\n";
    report << "Algorithm: A* Search\n";
    report << "Environment Size: " << ROW << "x" << COL << "\n\n";
    
    report << "Performance Metrics:\n";
    report << "-------------------\n";
    report << "Nodes Explored: " << metrics.nodesExplored << "\n";
    report << "Path Length: " << metrics.pathLength << "\n";
    report << "Execution Time: " << fixed << setprecision(3) << metrics.executionTime << " ms\n";
    report << "Obstacles: " << metrics.obstacles << "\n";
    report << "Free Spaces: " << metrics.freeSpaces << "\n\n";
    
    report << "Start Position: (" << sx << ", " << sy << ")\n";
    report << "Target Position: (" << tx << ", " << ty << ")\n\n";
    
    report << "Path Coordinates:\n";
    for (const auto& point : metrics.path) {
        report << "(" << point.first << ", " << point.second << ")\n";
    }
    
    report.close();
}

int main() {
    vector<vector<int>> maze = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,-1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,1,1,1,1,0,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,1,1,0,0,9,1,0,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,0,0,1,0,0,0,1,0,1},
        {1,0,1,1,1,1,1,1,1,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,1,0,0,0,1,1,1,0,1,0,0,0,1,0,1},
        {1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,1},
        {1,0,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
    };

    int sx, sy, tx, ty;
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == -1) { sx = i; sy = j; }
            else if (maze[i][j] == 9) { tx = i; ty = j; }
        }
    }

    PerformanceMetrics metrics = {0, 0, 0.0, {}, 0, 0};
    calculateEnvironmentStats(maze, metrics);

    cout << "\nStarting Single-Target Path Planning...\n";
    cout << "Press Enter to begin...";
    cin.get();

    if (!aStar(maze, sx, sy, tx, ty, metrics)) {
        cout << "No path found." << endl;
        return 1;
    }

    clearScreen();
    printMaze(maze, metrics);
    cout << "\nPath visualization complete." << endl;
    
    // Save performance report
    saveReport(metrics, sx, sy, tx, ty);
    cout << "\nPerformance report has been saved to 'single_target_report.txt'" << endl;
    
    return 0;
}