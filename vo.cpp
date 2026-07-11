#include "raylib.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

const float SCALE = 60.0f; 

struct Vector2D {
    double x, y;
    Vector2D operator+(const Vector2D& o) const { return {x + o.x, y + o.y}; }
    Vector2D operator-(const Vector2D& o) const { return {x - o.x, y - o.y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
    double dot(const Vector2D& o) const { return x * o.x + y * o.y; }
    double length() const { return std::sqrt(x * x + y * y); }
};

double normalize_angle(double angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

bool is_in_velocity_obstacle(Vector2D posA, double radA, Vector2D vA, Vector2D posB, double radB, Vector2D vB) {
    Vector2D p_rel = posB - posA;
    double dist = p_rel.length();
    double combined_radius = radA + radB;

    if (dist <= combined_radius) return true;
    Vector2D v_rel = vA - vB;
    if (v_rel.dot(p_rel) <= 0) return false;

    double alpha = std::atan2(p_rel.y, p_rel.x);
    double theta = std::asin(combined_radius / dist);
    double beta = std::atan2(v_rel.y, v_rel.x);

    return std::abs(normalize_angle(beta - alpha)) <= theta;
}

Vector2D compute_safe_velocity(Vector2D posA, double radA, Vector2D v_pref, Vector2D posB, double radB, Vector2D vB) {
    if (!is_in_velocity_obstacle(posA, radA, v_pref, posB, radB, vB)) return v_pref;

    double speed = v_pref.length();
    double pref_angle = std::atan2(v_pref.y, v_pref.x);

    for (double angle_offset = 0.02; angle_offset < PI; angle_offset += 0.02) {
        double a_left = pref_angle + angle_offset;
        Vector2D v_left = { speed * std::cos(a_left), speed * std::sin(a_left) };
        if (!is_in_velocity_obstacle(posA, radA, v_left, posB, radB, vB)) return v_left;

        double a_right = pref_angle - angle_offset;
        Vector2D v_right = { speed * std::cos(a_right), speed * std::sin(a_right) };
        if (!is_in_velocity_obstacle(posA, radA, v_right, posB, radB, vB)) return v_right;
    }
    return {0.0, 0.0};
}

int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Velocity Obstacle Path Tracer");
    SetTargetFPS(60);

    // Initial Positions
    Vector2D posA = { 1.0, 5.0 };
    double radA = 0.4;
    Vector2D v_prefA = { 3.5, 0.0 }; 

    Vector2D posB = { 8.0, 8.0 };
    double radB = 0.4;
    Vector2D vB = { -1.5, -2.0 }; 

    // Store history directly as Raylib Vector2 screen points
    std::vector<Vector2> screenPathHistory;
    int frameCounter = 0;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime(); 

        // 1. Physics & Position Updates
        Vector2D safe_vA = compute_safe_velocity(posA, radA, v_prefA, posB, radB, vB);
        posA = posA + (safe_vA * dt);
        posB = posB + (vB * dt);

        // Convert current position to screen pixels right now and save it
        Vector2 currentScreenPosA = { (float)posA.x * SCALE, (float)posA.y * SCALE };
        screenPathHistory.push_back(currentScreenPosA);

        // Optional Diagnostic: Print coordinates to terminal every 60 frames to verify movement
        frameCounter++;
        if (frameCounter % 60 == 0) {
            std::cout << "Agent A Screen Position: X=" << currentScreenPosA.x << ", Y=" << currentScreenPosA.y << std::endl;
        }

        // 2. Rendering
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw Background Grid
        for (int i = 0; i < screenWidth; i += (int)SCALE) DrawLine(i, 0, i, screenHeight, LIGHTGRAY);
        for (int i = 0; i < screenHeight; i += (int)SCALE) DrawLine(0, i, screenWidth, i, LIGHTGRAY);

        // --- RENDER TRACED PATH ---
        if (screenPathHistory.size() > 1) {
            for (size_t i = 1; i < screenPathHistory.size(); ++i) {
                // Using DrawLineV for clean vector-to-vector lines
                DrawLineV(screenPathHistory[i - 1], screenPathHistory[i], BLACK);
            }
        }

        // Draw Current Positions
        Vector2 screenPosB = { (float)posB.x * SCALE, (float)posB.y * SCALE };
        DrawCircleV(currentScreenPosA, radA * SCALE, BLUE);
        DrawCircleV(screenPosB, radB * SCALE, RED);

        // Draw Velocity Vectors
        Vector2 vectorEndA = { currentScreenPosA.x + (float)safe_vA.x * SCALE, currentScreenPosA.y + (float)safe_vA.y * SCALE };
        Vector2 vectorEndB = { screenPosB.x + (float)vB.x * SCALE, screenPosB.y + (float)vB.y * SCALE };
        DrawLineEx(currentScreenPosA, vectorEndA, 3.0f, DARKBLUE);
        DrawLineEx(screenPosB, vectorEndB, 3.0f, MAROON);

        DrawText("Blue: Controlled Agent | Red: Moving Obstacle", 20, 20, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}