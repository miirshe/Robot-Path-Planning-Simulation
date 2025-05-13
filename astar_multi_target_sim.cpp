#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <stack>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <algorithm>
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
#define MAGENTA "\033[35m"
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
    int targetsReached;
    vector<pair<int, int>> path;
};

// Cell structure
struct Node {
    int x, y;
    int g, h;
    Node* parent;
    int target_index;

    Node(int x, int y, int g, int h, int target_idx, Node* p = nullptr)
        : x(x), y(y), g(g), h(h), target_index(target_idx), parent(p) {}

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
void printStats(const PerformanceMetrics& metrics, int currentTarget, int totalTargets) {
    cout << CYAN << "\n=== Performance Metrics ===\n" << RESET;
    cout << "Nodes Explored: " << metrics.nodesExplored << endl;
    cout << "Path Length: " << metrics.pathLength << endl;
    cout << "Execution Time: " << fixed << setprecision(3) << metrics.executionTime << " ms" << endl;
    cout << "Targets Reached: " << metrics.targetsReached << "/" << totalTargets << endl;
    cout << "Current Target: " << currentTarget + 1 << endl;
    cout << CYAN << "========================\n" << RESET;
}

// Print the maze with colors and a title and legend
void printMaze(const vector<vector<int>>& maze, const PerformanceMetrics& metrics, int currentTarget, int totalTargets) {
    cout << GREEN << "\n\u25CF Multi-Target Path Planning for Robot in a Dynamic Environment\n" << RESET;
    cout << GREEN << "\u25A0" << RESET << " Wall (1)  ";
    cout << WHITE << "\u25A1" << RESET << " Space (0)  ";
    cout << RED << "\u25CF" << RESET << " Start (-1)  ";
    cout << BOLD << BLUE << "\u25B2" << RESET << " Target 1 (9)  ";
    cout << BOLD << MAGENTA << "\u25B2" << RESET << " Target 2 (8)  ";
    cout << BOLD << YELLOW << "\u25B2" << RESET << " Target 3 (7)  ";
    cout << BOLD << CYAN << "\u25B2" << RESET << " Target 4 (6)  ";
    cout << YELLOW << "\u25CF" << RESET << " Path (2)\n\n";

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == 1)
                cout << GREEN << "██" << RESET;
            else if (maze[i][j] == -1)
                cout << RED << " R" << RESET;
            else if (maze[i][j] == 9)
                cout << BOLD << BLUE << "▲" << RESET;
            else if (maze[i][j] == 8)
                cout << BOLD << MAGENTA << "▲" << RESET;
            else if (maze[i][j] == 7)
                cout << BOLD << YELLOW << "▲" << RESET;
            else if (maze[i][j] == 6)
                cout << BOLD << CYAN << "▲" << RESET;
            else if (maze[i][j] == 2)
                cout << YELLOW << " ." << RESET;
            else
                cout << "  ";
        }
        cout << endl;
    }
    
    printStats(metrics, currentTarget, totalTargets);
}

// A* Pathfinding for a single target
bool aStar(vector<vector<int>>& maze, int sx, int sy, int tx, int ty, int target_index, 
          PerformanceMetrics& metrics) {
    auto start_time = chrono::high_resolution_clock::now();
    priority_queue<Node*, vector<Node*>, Compare> openSet;
    vector<vector<bool>> closed(ROW, vector<bool>(COL, false));
    metrics.nodesExplored = 0;

    Node* start = new Node(sx, sy, 0, heuristic(sx, sy, tx, ty), target_index);
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
                                       heuristic(nx, ny, tx, ty), 
                                       target_index, current);
                openSet.push(neighbor);
            }
        }

        clearScreen();
        printMaze(maze, metrics, target_index, 4); // Assuming 4 targets
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}

// Find all targets in the maze
vector<pair<int, int>> findTargets(const vector<vector<int>>& maze) {
    vector<pair<int, int>> targets;
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == 9 || maze[i][j] == 8 || maze[i][j] == 7 || maze[i][j] == 6) {
                targets.push_back({i, j});
            }
        }
    }
    return targets;
}

// Find the starting position
pair<int, int> findStart(const vector<vector<int>>& maze) {
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (maze[i][j] == -1) {
                return {i, j};
            }
        }
    }
    return {-1, -1};
}

// Save performance report to file
void saveReport(const PerformanceMetrics& metrics, const vector<pair<int, int>>& targets) {
    ofstream report("performance_report.txt");
    report << "=== Multi-Target A* Path Planning Performance Report ===\n\n";
    report << "Algorithm: A* Search\n";
    report << "Environment Size: " << ROW << "x" << COL << "\n";
    report << "Total Targets: " << targets.size() << "\n\n";
    
    report << "Performance Metrics:\n";
    report << "-------------------\n";
    report << "Total Nodes Explored: " << metrics.nodesExplored << "\n";
    report << "Total Path Length: " << metrics.pathLength << "\n";
    report << "Total Execution Time: " << fixed << setprecision(3) << metrics.executionTime << " ms\n";
    report << "Targets Successfully Reached: " << metrics.targetsReached << "\n\n";
    
    report << "Target Positions:\n";
    for (size_t i = 0; i < targets.size(); i++) {
        report << "Target " << i + 1 << ": (" << targets[i].first << ", " << targets[i].second << ")\n";
    }
    
    report << "\nPath Coordinates:\n";
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

    // Add more targets to the maze with different values
    maze[3][20] = 9;  // Target 1 (Blue)
    maze[7][15] = 8;  // Target 2 (Magenta)
    maze[5][5] = 7;   // Target 3 (Yellow)
    maze[9][18] = 6;  // Target 4 (Cyan)

    auto start_pos = findStart(maze);
    auto targets = findTargets(maze);
    
    if (start_pos.first == -1 || targets.empty()) {
        cout << "Error: No start position or targets found!" << endl;
        return 1;
    }

    PerformanceMetrics metrics = {0, 0, 0.0, 0, {}};
    int current_x = start_pos.first;
    int current_y = start_pos.second;

    cout << "\nStarting Multi-Target Path Planning...\n";
    cout << "Press Enter to begin...";
    cin.get();

    // Visit each target in sequence
    for (size_t i = 0; i < targets.size(); i++) {
        // Find path to current target
        if (!aStar(maze, current_x, current_y, targets[i].first, targets[i].second, i, metrics)) {
            cout << "No path found to target " << i + 1 << endl;
            continue;
        }

        // Update current position
        current_x = targets[i].first;
        current_y = targets[i].second;
        
        // Mark target as visited
        maze[targets[i].first][targets[i].second] = 2;
        metrics.targetsReached++;
    }

    clearScreen();
    printMaze(maze, metrics, targets.size() - 1, targets.size());
    cout << "\nMulti-target path planning complete." << endl;
    
    // Save performance report
    saveReport(metrics, targets);
    cout << "\nPerformance report has been saved to 'performance_report.txt'" << endl;
    
    return 0;
}
