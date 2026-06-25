#include "raylib.h"
#include "raygui.h"

int main() {
    SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    InitWindow(900, 600, "Trajectory Planner");
    SetTargetFPS(60);
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(DARKGRAY);
        DrawText("It builds!", 20, 20, 20, RAYWHITE);
        GuiButton((Rectangle){20, 60, 140, 32}, "raygui works");
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
