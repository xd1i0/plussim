#include "raylib.h"
#include "double_pendulum.h"
#include <string>
#include <cmath>
#include <algorithm>
#include "rlImGui.h"
#include "imgui.h"

#include "cube.h"

struct CameraControl {
    float angle;
    float height;
    float zoom;
    float zoom_speed;
    float min_zoom;
    float max_zoom;
    float rotation_speed;
    float height_speed;
};

void handleCamera(Camera3D& camera, CameraControl& camControl) {
    // Handle arrow key input
    if (IsKeyDown(KEY_LEFT)) {
        camControl.angle += camControl.rotation_speed * GetFrameTime();
    }
    if (IsKeyDown(KEY_RIGHT)) {
        camControl.angle -= camControl.rotation_speed * GetFrameTime();
    }
    if (IsKeyDown(KEY_UP)) {
        camControl.height += camControl.height_speed * GetFrameTime();
    }
    if (IsKeyDown(KEY_DOWN)) {
        camControl.height -= camControl.height_speed * GetFrameTime();
    }

    // Handle mouse wheel zoom
    float wheelMovement = GetMouseWheelMove();
    camControl.zoom += wheelMovement * camControl.zoom_speed;
    camControl.zoom = std::clamp(camControl.zoom, camControl.min_zoom, camControl.max_zoom);

    // Clamp camera height to reasonable values
    camControl.height = std::clamp(camControl.height, 1.0f, 30.0f);

    // Update camera position based on angle and height
    float angleRad = camControl.angle * DEG2RAD;
    camera.position = (Vector3){
        std::cos(angleRad) * camControl.zoom,
        camControl.height,
        std::sin(angleRad) * camControl.zoom
    };
}

int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Spring-Mass Simulation");

    // Get monitor dimensions and center window (similar to Rust version)
    int monitor = GetCurrentMonitor();
    int monitorWidth = GetMonitorWidth(monitor);
    int monitorHeight = GetMonitorHeight(monitor);
    SetWindowSize(monitorWidth / 2, monitorHeight / 2);
    SetWindowPosition(monitorWidth / 4, monitorHeight / 4);

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 0.0f, 5.0f, 10.0f };   // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type

    CameraControl cameraControl = {
        45.0f,      // angle
        10.0f,      // height
        15.0f,      // zoom
        5.0f,       // zoom_speed
        1.0f,       // min_zoom
        100.0f,     // max_zoom
        100.0f,     // rotation_speed
        20.0f       // height_speed
    };

    SetTargetFPS(61);

    rlImGuiSetup(true);

    // Initialize cube at (0, 10, 0) where Y=10 is the height - hangs below anchor
    Cube cube(1.0, 0.0, 10.0, 0.0, 2.0);

    // Set up spring: anchor at (0, 15, 0) - 15 units high (above cube)
    // spring_k=50 (moderate stiffness), damping=3.0, rest_length=4.0
    // The cube will hang down from the anchor point
    cube.setSpring(0.0, 15.0, 0.0, 50.0, 3.0, 4.0);

    // Configuration variables for ImGui
    float mass = 1.0f;
    float spring_k = 50.0f;
    float damping = 3.0f;
    float rest_length = 4.0f;
    float anchor_x = 0.0f;
    float anchor_y = 15.0f;
    float anchor_z = 0.0f;
    float cube_size = 2.0f;
    int solver_type = 0; // 0 = Euler, 1 = RK4
    bool show_config = true;

    //DoublePendulum pendulum(2.0f, 2.0f, 0.2f, 0.2f, M_PI / 2.0f, M_PI / 2.0f, 0.05f);

    while (!WindowShouldClose()) {
        // Reset cube with R key
        if (IsKeyPressed(KEY_R)) {
            cube.reset(0.0, 10.0, 0.0);
        }

        // Toggle config window with C key
        if (IsKeyPressed(KEY_C)) {
            show_config = !show_config;
        }

        handleCamera(camera, cameraControl);

        double dt = GetFrameTime();
        cube.update(dt);

        double x, y, z;
        cube.getPosition(x, y, z);

        BeginDrawing();
        ClearBackground(RAYWHITE);
        {
            BeginMode3D(camera);
            DrawGrid(50, 1.0f);

            // Draw spring if enabled
            if (cube.hasSpring()) {
                double anchor_x, anchor_y, anchor_z;
                cube.getSpringAnchor(anchor_x, anchor_y, anchor_z);

                Vector3 anchorPos = {(float)anchor_x, (float)anchor_y, (float)anchor_z};
                Vector3 cubePos = {(float)x, (float)y, (float)z};
                DrawSphere(anchorPos, 0.3f, DARKGRAY);
                DrawLine3D(anchorPos, cubePos, BLUE);
            }

            // Draw cube
            DrawCube((Vector3){(float)x, (float)y, (float)z},
                     (float)cube.getSize(),
                     (float)cube.getSize(),
                     (float)cube.getSize(), RED);
            DrawCubeWires((Vector3){(float)x, (float)y, (float)z},
                          (float)cube.getSize(),
                          (float)cube.getSize(),
                          (float)cube.getSize(), BLACK);
            EndMode3D();
        }

        // ImGui Configuration Panel
        rlImGuiBegin();

        if (show_config) {
            ImGui::Begin("Spring-Mass Configuration", &show_config);

            ImGui::Text("Solver Settings");
            ImGui::Separator();
            const char* solvers[] = { "Euler", "RK4" };
            if (ImGui::Combo("Solver Type", &solver_type, solvers, IM_ARRAYSIZE(solvers))) {
                cube.setSolverType(solver_type);
            }

            ImGui::Spacing();
            ImGui::Text("Cube Properties");
            ImGui::Separator();
            if (ImGui::SliderFloat("Mass (kg)", &mass, 0.1f, 10.0f)) {
                cube.setMass(mass);
            }
            if (ImGui::SliderFloat("Size", &cube_size, 0.5f, 5.0f)) {
                cube.setSize(cube_size);
            }

            ImGui::Spacing();
            ImGui::Text("Spring Properties");
            ImGui::Separator();
            bool spring_changed = false;
            spring_changed |= ImGui::SliderFloat("Spring Constant (k)", &spring_k, 1.0f, 500.0f);
            spring_changed |= ImGui::SliderFloat("Damping", &damping, 0.0f, 20.0f);
            spring_changed |= ImGui::SliderFloat("Rest Length", &rest_length, 0.5f, 10.0f);

            ImGui::Spacing();
            ImGui::Text("Anchor Position");
            ImGui::Separator();
            spring_changed |= ImGui::SliderFloat("Anchor X", &anchor_x, -10.0f, 10.0f);
            spring_changed |= ImGui::SliderFloat("Anchor Y", &anchor_y, 5.0f, 30.0f);
            spring_changed |= ImGui::SliderFloat("Anchor Z", &anchor_z, -10.0f, 10.0f);

            if (spring_changed) {
                cube.setSpring(anchor_x, anchor_y, anchor_z, spring_k, damping, rest_length);
            }

            ImGui::Spacing();
            if (ImGui::Button("Reset Cube Position")) {
                cube.reset(0.0, 10.0, 0.0);
            }

            ImGui::Spacing();
            ImGui::Text("Position: (%.2f, %.2f, %.2f)", x, y, z);

            ImGui::End();
        }

        rlImGuiEnd();

        int fps = GetFPS();
        float frameTime = GetFrameTime();
        std::string frameTimeStr = "FPS: " + std::to_string(fps) + " FrameTime: " + std::to_string(frameTime);
        DrawText(frameTimeStr.c_str() , 10, 10, 20, BLACK);

        DrawText("3D Spring-Mass Simulation", 10, 40, 20, BLACK);
        DrawText("Press ESC to exit | Press R to reset | Press C to toggle config", 10, 60, 16, BLACK);

        EndDrawing();
    }

    rlImGuiShutdown();

    CloseWindow();
    return 0;
}
