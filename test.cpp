#include "raylib.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>


const float SCALE = 60.0f; // 1 unit in math = 60 pixels on screen

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
    // Initialize Raylib window
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Velocity Obstacle Simulation");
    SetTargetFPS(60);

    // Positions & sizes in "Simulation Space" (meters)
    Vector2D posA = { 1.0, 5.0 };
    double radA = 0.4;
    Vector2D v_prefA = { 3.5, 0.0 }; // Wants to move straight right

    Vector2D posB = { 8.0, 8.0 };
    double radB = 0.4;
    Vector2D vB = { -1.5, -2.0 }; // Moving up and left, crossing A's path

    // Main game loop
    while (!WindowShouldClose()) {
        // Delta time: time elapsed since last frame (around 0.016s at 60fps)
        float dt = GetFrameTime(); 

        // 1. Calculate the evasion velocity for this frame
        Vector2D safe_vA = compute_safe_velocity(posA, radA, v_prefA, posB, radB, vB);

        // 2. Update positions using physics: position = position + (velocity * time)
        posA = posA + (safe_vA * dt);
        posB = posB + (vB * dt);

        // 3. Render
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw Grid Lines for visual reference
        for (int i = 0; i < screenWidth; i += (int)SCALE) DrawLine(i, 0, i, screenHeight, LIGHTGRAY);
        for (int i = 0; i < screenHeight; i += (int)SCALE) DrawLine(0, i, screenWidth, i, LIGHTGRAY);

        // Convert math coordinates to screen pixels for drawing
        Vector2 screenPosA = { (float)posA.x * SCALE, (float)posA.y * SCALE };
        Vector2 screenPosB = { (float)posB.x * SCALE, (float)posB.y * SCALE };

        // Draw Agents
        DrawCircleV(screenPosA, radA * SCALE, BLUE);
        DrawCircleV(screenPosB, radB * SCALE, RED);

        // Draw Velocity Vectors (lines pointing where they are going)
        Vector2 vectorEndA = { screenPosA.x + (float)safe_vA.x * SCALE, screenPosA.y + (float)safe_vA.y * SCALE };
        Vector2 vectorEndB = { screenPosB.x + (float)vB.x * SCALE, screenPosB.y + (float)vB.y * SCALE };
        
        DrawLineEx(screenPosA, vectorEndA, 3.0f, DARKBLUE);
        DrawLineEx(screenPosB, vectorEndB, 3.0f, MAROON);

        DrawText("Blue: Controlled Agent | Red: Moving Obstacle", 20, 20, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}