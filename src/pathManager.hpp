#pragma once
#include "trajectory.hpp"
#include <vector>
#include <algorithm>

struct ManagerState {
    std::vector<PathState> paths;
    int active;
    float density;
    float maxSpeed;
};

class PathManager {
    std::vector<Trajectory> paths;
    int active = 0;
    bool dragging = false;
    std::vector<ManagerState> history;
    ManagerState lastSavedState;
    float density = 2.0f;
    float maxSpeed = 4.0f;
    std::vector<ManagerState> redoStack;
public:
    int activeIndex() {return active;}
    Trajectory& selected() {return paths[active];}
    Trajectory& at(int i) {return paths[i];}
    void addPath();
    void deletePath(int i);
    void handleClick(Vector2 mousePos);
    void endDrag() {dragging = false;}
    bool isDragging() const {return dragging;}
    void setActive(int i) {active = i;}
    void moveSelected(Vector2 mousePos);
    void draw();
    void setDragging(bool d) {dragging = d;}
    int count() const {return paths.size();}
    void saveState();
    void undo();
    void redo();
    bool exportPaths(const std::string& fileName) const;
    float getDensity() const {return density;}
    void setDensity(float d) {density = d;}
    float getMaxSpeed() const {return maxSpeed;}
    void setMaxSpeed(float s) {maxSpeed = s;}
    void rebuildAll();
};
