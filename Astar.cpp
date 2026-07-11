#include "raylib.h"
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>

const int CELL_SIZE = 20; 
const int COLS = 40;      
const int ROWS = 30;      

struct Point {
    int r, c;
    bool operator==(const Point& o) const { return r == o.r && c == o.c; }
};

struct AStarNode {
    double f_score;
    int index;
    bool operator>(const AStarNode& other) const {
        return f_score > other.f_score;
    }
};

// Manhattan distance heuristic
inline double get_heuristic(int r, int c, int tr, int tc) {
    return std::abs(r - tr) + std::abs(c - tc);
}

// A* Implementation
std::vector<Point> find_a_star_path(const std::vector<char>& grid, Point start, Point end) {
    int start_idx = start.r * COLS + start.c;
    int end_idx = end.r * COLS + end.c;

    std::vector<double> g_score(ROWS * COLS, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(ROWS * COLS, -1);
    std::vector<bool> closed_set(ROWS * COLS, false);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

    g_score[start_idx] = 0.0;
    open_set.push({get_heuristic(start.r, start.c, end.r, end.c), start_idx});

    // 4-Way Movement: Up, Down, Left, Right
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = {0, 0, -1, 1};

    while (!open_set.empty()) {
        AStarNode current = open_set.top();
        open_set.pop();

        int curr_idx = current.index;
        if (curr_idx == end_idx) {
            std::vector<Point> path;
            int step = end_idx;
            while (step != -1) {
                path.push_back({step / COLS, step % COLS});
                step = came_from[step];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed_set[curr_idx]) continue;
        closed_set[curr_idx] = true;

        int r = curr_idx / COLS;
        int c = curr_idx % COLS;

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int n_idx = nr * COLS + nc;

            // Strict bounds checking
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) {
                if (grid[n_idx] == '*') continue; 

                double tentative_g = g_score[curr_idx] + 1.0;

                if (tentative_g < g_score[n_idx]) {
                    came_from[n_idx] = curr_idx;
                    g_score[n_idx] = tentative_g;
                    double f = tentative_g + 1.001 * get_heuristic(nr, nc, end.r, end.c);
                    open_set.push({f, n_idx});
                }
            }
        }
    }
    return {}; 
}

int main() {
    // 1. Initialize grid layout
    std::vector<char> grid(ROWS * COLS, '-');

    // 2. Clear open coordinates for Start & End nodes
    Point startPos = {2, 2};
    Point endPos = {25, 35};

    // 3. Generate structured obstacles that leave simple corridors open
    // We create alternating horizontal pillars with open lanes at the ends
    for (int c = 0; c < 30; ++c)  grid[8 * COLS + c] = '*';   // Row 8 wall (gap on right)
    for (int c = 10; c < 40; ++c) grid[16 * COLS + c] = '*';  // Row 16 wall (gap on left)
    for (int c = 0; c < 30; ++c)  grid[22 * COLS + c] = '*';  // Row 22 wall (gap on right)

    // Make absolutely sure start and end nodes are never overwritten by walls
    grid[startPos.r * COLS + startPos.c] = '-';
    grid[endPos.r * COLS + endPos.c] = '-';

    // 4. Compute path via A* and print immediately to verify
    std::vector<Point> computedPath = find_a_star_path(grid, startPos, endPos);
    
    std::cout << "\n==========================================" << std::endl;
    if (computedPath.empty()) {
        std::cout << "[CRITICAL ERROR] A* could not find any path." << std::endl;
    } else {
        std::cout << "[SUCCESS] A* path found! Total Nodes: " << computedPath.size() << std::endl;
        std::cout << "Path steps mapped out: ";
        for (const auto& pt : computedPath) {
            std::cout << "(" << pt.r << "," << pt.c << ") -> ";
        }
        std::cout << "END" << std::endl;
    }
    std::cout << "==========================================\n" << std::endl;

    // Set up window
    InitWindow(COLS * CELL_SIZE, ROWS * CELL_SIZE, "A* Maze Pathfinder Simulation");
    SetTargetFPS(60);

    // Initial agent center position mapping
    Vector2 agentPos = {
        (float)startPos.c * CELL_SIZE + CELL_SIZE / 2.0f, 
        (float)startPos.r * CELL_SIZE + CELL_SIZE / 2.0f
    };
    size_t targetPathIndex = 0;
    float agentSpeed = 250.0f; // Pixels per second

    // Vector to store historical line tracks
    std::vector<Vector2> traceHistory;
    traceHistory.push_back(agentPos);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // 5. Move Agent along path array nodes sequentially
        if (!computedPath.empty() && targetPathIndex < computedPath.size()) {
            Point targetNode = computedPath[targetPathIndex];
            Vector2 targetPixelPos = {
                (float)targetNode.c * CELL_SIZE + CELL_SIZE / 2.0f,
                (float)targetNode.r * CELL_SIZE + CELL_SIZE / 2.0f
            };

            float dx = targetPixelPos.x - agentPos.x;
            float dy = targetPixelPos.y - agentPos.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance > 1.5f) {
                agentPos.x += (dx / distance) * agentSpeed * dt;
                agentPos.y += (dy / distance) * agentSpeed * dt;
                traceHistory.push_back(agentPos); 
            } else {
                targetPathIndex++; 
            }
        }

        // 6. Drawing Layout
        BeginDrawing();
        ClearBackground(RAYWHITE); 

        // Render Maze Squares
        for (int r = 0; r < ROWS; ++r) {
            for (int c = 0; c < COLS; ++c) {
                int idx = r * COLS + c;
                if (grid[idx] == '*') {
                    DrawRectangle(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE - 1, CELL_SIZE - 1, DARKGRAY);
                } else {
                    DrawRectangle(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE - 1, CELL_SIZE - 1, WHITE);
                    DrawRectangleLines(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE, LIGHTGRAY);
                }
            }
        }

        // Highlight Checkpoints (Ensure they paint cleanly on top of backgrounds)
        DrawRectangle(startPos.c * CELL_SIZE, startPos.r * CELL_SIZE, CELL_SIZE, CELL_SIZE, GREEN);
        DrawRectangle(endPos.c * CELL_SIZE, endPos.r * CELL_SIZE, CELL_SIZE, CELL_SIZE, RED);

        // Render Traced Line path historical vectors
        if (traceHistory.size() > 1) {
            for (size_t i = 1; i < traceHistory.size(); ++i) {
                DrawLineEx(traceHistory[i - 1], traceHistory[i], 3.0f, BLACK);
            }
        }

        // Render Moving Agent 
        DrawCircleV(agentPos, CELL_SIZE / 3.0f, BLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}