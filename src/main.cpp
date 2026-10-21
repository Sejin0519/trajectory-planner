#include "raylib.h"
#include "rlImGui.h"
#include "imgui.h"
#include "implot.h"
#include "trajectory.hpp"
#include "tinyfiledialogs.h"
#include "pathManager.hpp"



int main() {
    //Initialize window and ImGui
    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(900, 600, "Trajectory Planner");
    SetTargetFPS(60);
    rlImGuiBeginInitImGui();
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();                                 
    io.Fonts->AddFontFromFileTTF("/System/Library/Fonts/Supplemental/Arial.ttf", 18.0f);
    ImGui::StyleColorsDark();
    rlImGuiEndInitImGui();
    ImPlot::CreateContext();
    ImPlot::GetInputMap().Fit = ImGuiMouseButton_Middle;
    PathManager document;
    document.addPath();
    Camera2D camera = {0};
    camera.zoom = 1.0f;
    SetGesturesEnabled(GESTURE_PINCH_IN | GESTURE_PINCH_OUT);    


    const float aspectRatio = 16.0f / 10.0f; //matching my macbook size 

    Vector2 fieldSize = { GetScreenHeight() * 0.8f, GetScreenHeight() * 0.8f };
    Vector2 fieldPos = {(GetScreenWidth() - fieldSize.x)/2, GetScreenHeight() * 0.01f};

    RenderTexture2D field = LoadRenderTexture(fieldSize.x, fieldSize.y);

    const float FIELD = 144.0f;
    camera.offset = {fieldSize.x / 2, fieldSize.y / 2};
    camera.zoom = fieldSize.x / FIELD * 0.95f;
    camera.target = {0, 0};

    bool dragging = false;


    while (!WindowShouldClose()) {
        // Handle window resizing
        if (IsWindowResized()) { 
            int w = GetScreenWidth();
            SetWindowSize(w, (int)(w / aspectRatio));
            UnloadRenderTexture(field);
            float maxFieldSize = GetScreenHeight() * 0.8f;
            fieldSize = {maxFieldSize, maxFieldSize};
            fieldPos = {(GetScreenWidth() - fieldSize.x)/2, GetScreenHeight() * 0.01f};
            field = LoadRenderTexture((int)fieldSize.x, (int)fieldSize.y);
            camera.offset = {fieldSize.x / 2, fieldSize.y / 2};
            camera.zoom = fieldSize.x / FIELD * 0.95f;
        }

        float panelX = fieldPos.x + fieldSize.x + GetScreenWidth() * 0.005f;
        float panelW = GetScreenWidth() - panelX;


        //Field interactions

        if(IsKeyPressed(KEY_BACKSPACE) && !ImGui::IsAnyItemActive() && !document.isDragging()) {
            document.selected().deleteSelected();
        }

        Vector2 mousePos = GetMousePosition();
        Vector2 mouseFieldPos = mousePos - fieldPos;
        bool inField = mouseFieldPos.x >= 0 && mouseFieldPos.y >= 0 && mouseFieldPos.x < fieldSize.x && mouseFieldPos.y < fieldSize.y;

        if(!ImGui::GetIO().WantCaptureMouse && inField) {
            Vector2 slide = GetMouseWheelMoveV();

            if (IsKeyDown(KEY_LEFT_SUPER) || IsKeyDown(KEY_RIGHT_SUPER)) {  
                if (slide.y != 0) {
                    Vector2 worldBefore = GetScreenToWorld2D(mouseFieldPos, camera);
                    camera.offset = mouseFieldPos;
                    camera.target = worldBefore;
                    camera.zoom = Clamp(camera.zoom * expf(0.1f * slide.y), 0.1f, 50.0f);
                }
            } else {                                                         
                if (slide.x != 0 || slide.y != 0) {
                    camera.target.x -= slide.x / camera.zoom;
                    camera.target.y -= slide.y / camera.zoom;
                }
            }

            Vector2 worldMousePos = GetScreenToWorld2D(mouseFieldPos, camera);
            if(IsMouseButtonPressed(0)) {
                document.handleClick(worldMousePos);
            }
            if(IsMouseButtonDown(0) && document.isDragging()) {
                document.moveSelected(worldMousePos);
            }

        }
        if(IsMouseButtonReleased(0)) {
            document.endDrag();
        }

        // Undo/Redo with Cmd+Z / Cmd+Shift+Z
        if(!ImGui::IsAnyItemActive() && (IsKeyDown(KEY_LEFT_SUPER) || IsKeyDown(KEY_RIGHT_SUPER)) && IsKeyPressed(KEY_Z) && !IsMouseButtonDown(0)) {
            if(IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                document.redo();
            } else {
                document.undo();
            }
        }

        document.saveState();
        document.rebuildAll();

        // Render field to texture
        BeginTextureMode(field);
            ClearBackground(GRAY);
            BeginMode2D(camera);
                document.draw();
                DrawRectangleLines(-FIELD/2, -FIELD/2, FIELD, FIELD, WHITE); //field boundary
            EndMode2D();
        EndTextureMode();

        //ImGui Panels
        BeginDrawing();
            ClearBackground(DARKGRAY);  
            DrawTextureRec(field.texture,
                { 0, 0, (float)field.texture.width, -(float)field.texture.height },
                fieldPos, WHITE); 
            rlImGuiBegin();
                // Inspector panel
                ImGui::SetNextWindowPos({panelX, 0});
                ImGui::SetNextWindowSize({ panelW, (float)GetScreenHeight()});
                ImGui::Begin("Inspector", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);
                    ImGui::Text("Control");
                    if(document.selected().isSelected()){
                        Vector2 position = document.selected().selectedPos();
                        position.y = -position.y; //invert y for display

                        if (ImGui::InputFloat2("Position", &position.x)){
                            position.y = -position.y; //invert y back
                            document.selected().setSelectedPos(position);
                        }
                        if (document.selected().isAnchorSelected()) {
                            float heading = document.selected().selectedHeading();
                            if (ImGui::InputFloat("Heading", &heading)){
                                document.selected().setSelectedHeading(wrapAngle(heading));
                            }
                        }
                    } else {
                        ImGui::TextDisabled("No point selected");
                    }
                    static char exportPath[256] = "paths.hpp"; //default export file name
                    static int  exportStatus = 0;   // 0 none, 1 ok, -1 fail


                    ImGui::Separator();
                    ImGui::Text("Settings");
                    float maxVel = document.getMaxSpeed();
                    if(ImGui::InputFloat("Max Speed", &maxVel)) {
                        if(maxVel <= 0) maxVel = 0.1f;
                        document.setMaxSpeed(maxVel);
                    }
                    float density = document.getDensity();
                    if(ImGui::InputFloat("Density", &density)) {
                        if(density <= 0) density = 0.1f;
                        document.setDensity(density);
                    }

                    if (exportStatus == 1)  ImGui::TextColored({0.4f,1,0.4f,1}, "Exported");
                    if (exportStatus == -1) ImGui::TextColored({1,0.4f,0.4f,1}, "Export failed");
                ImGui::End();


                // Paths panel
                ImGui::SetNextWindowPos({0, 0});
                ImGui::SetNextWindowSize({ panelW, (float)GetScreenHeight()});
                ImGui::Begin("Paths", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);
                    ImGui::Text("Paths");
                    ImGui::SameLine();
                    if(ImGui::Button("+")) {
                        document.addPath();
                    }

                    static int editingIndex = -1;
                    static char nameBuffer[64];

                    for(int i = 0; i<document.count(); i++) {
                        ImGui::PushID(i);
                        bool isActive = (i == document.activeIndex());
                        std::string label = document.at(i).getName();

                        if(editingIndex == i) { //editing mode
                            strcpy(nameBuffer, label.c_str());
                            if(ImGui::InputText("##edit", nameBuffer, sizeof(nameBuffer), ImGuiInputTextFlags_EnterReturnsTrue)) {
                                document.at(i).setName(std::string(nameBuffer));
                                editingIndex = -1;
                            }
                        } else {
                            if(ImGui::Selectable(label.c_str(), isActive)) { //select path on click
                                document.setActive(i);
                            }
                            if(ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) { //editing mode on double click
                                editingIndex = i;
                                strcpy(nameBuffer, label.c_str());
                            }
                        }
                        ImGui::PopID();
                    }
                                        ImGui::Separator();
                    ImGui::Text("Export");
                    ImGui::InputText("File", exportPath, sizeof(exportPath));
                    if (ImGui::Button("Export all…")) {
                        const char* filter[] = { "*.txt" };
                        const char* dest = tinyfd_saveFileDialog("Export paths", "paths.hpp", 1, filter, "header files");
                        if (dest != NULL) {
                            exportStatus = document.exportPaths(dest) ? 1 : -1;
                        }
                    }
                ImGui::End();


                // Velocity profile panel
                float totalLength = document.selected().getTotalLength();
                float graphY = fieldPos.y + fieldSize.y + GetScreenHeight() * 0.005f;
                ImGui::SetNextWindowPos({fieldPos.x, graphY});
                ImGui::SetNextWindowSize({ fieldSize.x, GetScreenHeight() - graphY});
                ImGui::Begin("Velocity Profile", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);
                if (ImPlot::BeginPlot("##velprofile", ImVec2(-1, -1))) { //(-1, -1) = fill space
                    ImPlot::SetupAxesLimits(0, totalLength, 0, document.getMaxSpeed(), ImPlotCond_Always);

                    static std::vector<double> x, y;
                    x.clear();
                    y.clear();
                    std::vector<VelPoint>& velNodes = document.selected().getVelNodes();
                    std::vector<VelPoint>& velocityProfile = document.selected().getVelocityProfile();

                    for(int i = 0; i < (int)velocityProfile.size(); i++) {
                        VelPoint& p = velocityProfile[i];
                        x.push_back(p.x);
                        y.push_back(p.speed);
                    }
                    if(!x.empty()) {
                        ImPlot::PlotScatter("##dots", x.data(), y.data(), (int)x.size(), {
                            ImPlotProp_Marker,     ImPlotMarker_Circle,
                            ImPlotProp_MarkerSize, 2.0f
                        });
                    }
                    int deleteIndex = -1;
                    for(int i = 0; i < (int)velNodes.size(); i++) {
                        VelPoint& p = velNodes[i];

                        double dx = p.x;
                        double dy = p.speed;
                        bool held = false;
                        bool hovered = false;
                        if(ImPlot::DragPoint(i, &dx, &dy, ImVec4(0.35f, 0.65f, 1.0f, 1.0f), 6, 0, nullptr, &hovered, &held)) {
                            float lowerBound = (i > 0) ? velNodes[i - 1].x : 0.0f;
                            float upperBound = (i < (int)velNodes.size() - 1) ? velNodes[i + 1].x : totalLength;
                            p.x = Clamp((float)dx, lowerBound, upperBound); //clamp node x position between neighbors
                            p.speed = Clamp((float)dy, 0.0f, document.getMaxSpeed());
                        }
                        if(hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) { //delete node on right click
                            deleteIndex = i;
                        }
                        if(held) {
                            document.setDragging(true);
                        }
                    }
                    if(deleteIndex != -1) {
                        velNodes.erase(velNodes.begin() + deleteIndex);
                    }


                    if(ImPlot::IsPlotHovered() && ImGui::IsMouseDoubleClicked(0)) { //add new node on double click
                        ImPlotPoint point = ImPlot::GetPlotMousePos();
                        VelPoint newNode;
                        newNode.x = Clamp((float)point.x, 0.0f, totalLength);
                        newNode.speed = Clamp((float)point.y, 0.0f, document.getMaxSpeed());
                        auto index = std::lower_bound(velNodes.begin(), velNodes.end(), newNode);
                        velNodes.insert(index, newNode);                       
                    }
                    ImPlot::EndPlot();
                }
                ImGui::End();

            rlImGuiEnd();
        EndDrawing();

    }
    UnloadRenderTexture(field);
    ImPlot::DestroyContext();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}