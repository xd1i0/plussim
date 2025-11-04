#include "raylib.h"
#include "double_pendulum.h"
#include <string>
#include "string"
#include "stellaris/models/planetes.h"
#include "rlImGui.h"

int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Double Pendulum Simulation");

    SetTargetFPS(60);

    DoublePendulum pendulum(100.0f, 100.0f, 10.0f, 10.0f, M_PI / 2.0f, M_PI / 2.0f, 0.05f);

    Vector2 origin = {screenWidth / 2.0f, 200.0f};

    while (!WindowShouldClose()) {
        pendulum.update();

        // Get positions
        Vector2 pos1 = pendulum.getPos1(origin);
        Vector2 pos2 = pendulum.getPos2(origin);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawLineEx(origin, pos1, 2, DARKGRAY);
        DrawLineEx(pos1, pos2, 2, DARKGRAY);

        DrawCircleV(pos1, pendulum.getMass1(), RED);
        DrawCircleV(pos2, pendulum.getMass2(), BLUE);

        DrawCircleV(origin, 5, BLACK);

        int fps = GetFPS();
        float frameTime = GetFrameTime();
        std::string frameTimeStr = "FPS: " + std::to_string(fps) + " FrameTime: " + std::to_string(frameTime);
        DrawText(frameTimeStr.c_str() , 10, 10, 20, BLACK);

        DrawText("Double Pendulum Simulation", 10, 40, 20, BLACK);
        DrawText("Press ESC to exit", 10, 60, 16, BLACK);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
