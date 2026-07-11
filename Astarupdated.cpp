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

// Euclidean distance heuristic (essential when 8-way diagonal movement is allowed)
inline double get_heuristic(int r, int c, int tr, int tc) {
    double dr = r - tr;
    double dc = c - tc;
    return std::sqrt(dr * dr + dc * dc);
}

// 1. Raycast Check: Evaluates if a straight line hits a wall cell
bool has_line_of_sight(Point p1, Point p2, const std::vector<char>& grid) {
    float steps = std::max(std::abs(p2.r - p1.r), std::abs(p2.c - p1.c)) * 3.0f;
    if (steps == 0) return true;

    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / steps;
        int check_r = std::round(p1.r + t * (p2.r - p1.r));
        int check_c = std::round(p1.c + t * (p2.c - p1.c));
        
        if (check_r >= 0 && check_r < ROWS && check_c >= 0 && check_c < COLS) {
            if (grid[check_r * COLS + check_c] == '*') {
                return false; 
            }
        }
    }
    return true;
}

// 2. Post-Processing Path Smoother (String Puller)
std::vector<Point> smooth_path(const std::vector<Point>& original_path, const std::vector<char>& grid) {
    if (original_path.size() <= 2) return original_path;

    std::vector<Point> smoothed;
    smoothed.push_back(original_path[0]); // Anchor the start point

    size_t current = 0;
    while (current < original_path.size() - 1) {
        size_t furthest_visible = current + 1;
        
        // Skip ahead as far as possible if a clear diagonal line of sight exists
        for (size_t next = current + 2; next < original_path.size(); ++next) {
            if (has_line_of_sight(original_path[current], original_path[next], grid)) {
                furthest_visible = next;
            } else {
                break; 
            }
        }
        
        smoothed.push_back(original_path[furthest_visible]);
        current = furthest_visible;
    }
    return smoothed;
}

// 3. 8-Way A* Pathfinder using 1.414 Diagonal Cost values
std::vector<Point> find_a_star_path(const std::vector<char>& grid, Point start, Point end) {
    int start_idx = start.r * COLS + start.c;
    int end_idx = end.r * COLS + end.c;

    std::vector<double> g_score(ROWS * COLS, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(ROWS * COLS, -1);
    std::vector<bool> closed_set(ROWS * COLS, false);

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

    g_score[start_idx] = 0.0;
    open_set.push({get_heuristic(start.r, start.c, end.r, end.c), start_idx});

    // 8-Way Movement: Cardinals followed by Diagonals
    const int dr[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dc[] = {0, 0, -1, 1, -1, 1, -1, 1};
    const double move_cost[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

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

        for (int i = 0; i < 8; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];
            int n_idx = nr * COLS + nc;

            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS) {
                if (grid[n_idx] == '*') continue; 

                // Corner cutting safeguard: prevent clipping past diagonal walls
                if (i >= 4) {
                    if (grid[r * COLS + nc] == '*' && grid[nr * COLS + c] == '*') continue;
                }

                double tentative_g = g_score[curr_idx] + move_cost[i];

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
    std::vector<char> grid(ROWS * COLS, '-');
    Point startPos = {2, 2};
    Point endPos = {25, 35};

    // Construct a classic wall obstacle chunk down the middle
    for (int c = 12; c < 28; ++c) grid[14 * COLS + c] = '*';

    // Compute original path and optimize it immediately
    std::vector<Point> rawPath = find_a_star_path(grid, startPos, endPos);
    std::vector<Point> computedPath = smooth_path(rawPath, grid);

    InitWindow(COLS * CELL_SIZE, ROWS * CELL_SIZE, "A* Any-Angle Smoothed Pathfinder");
    SetTargetFPS(60);

    Vector2 agentPos = {
        (float)startPos.c * CELL_SIZE + CELL_SIZE / 2.0f, 
        (float)startPos.r * CELL_SIZE + CELL_SIZE / 2.0f
    };
    size_t targetPathIndex = 0;
    float agentSpeed = 250.0f; 

    std::vector<Vector2> traceHistory;
    traceHistory.push_back(agentPos);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Path following logic
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

        BeginDrawing();
        ClearBackground(RAYWHITE); 

        // Render Grid and Walls
        for (int r = 0; r < ROWS; ++r) {
            for (int c = 0; c < COLS; ++c) {
                if (grid[r * COLS + c] == '*') {
                    DrawRectangle(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE - 1, CELL_SIZE - 1, DARKGRAY);
                } else {
                    DrawRectangle(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE - 1, CELL_SIZE - 1, WHITE);
                    DrawRectangleLines(c * CELL_SIZE, r * CELL_SIZE, CELL_SIZE, CELL_SIZE, LIGHTGRAY);
                }
            }
        }

        DrawRectangle(startPos.c * CELL_SIZE, startPos.r * CELL_SIZE, CELL_SIZE, CELL_SIZE, GREEN);
        DrawRectangle(endPos.c * CELL_SIZE, endPos.r * CELL_SIZE, CELL_SIZE, CELL_SIZE, RED);

        // Draw the trace trail
        if (traceHistory.size() > 1) {
            for (size_t i = 1; i < traceHistory.size(); ++i) {
                DrawLineEx(traceHistory[i - 1], traceHistory[i], 3.0f, BLACK);
            }
        }

        DrawCircleV(agentPos, CELL_SIZE / 3.0f, BLUE);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}