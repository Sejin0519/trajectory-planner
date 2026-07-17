#include "pathManager.hpp"

void PathManager::addPath() {
    paths.emplace_back();
    paths.back().initializePoints();
    active = paths.size() - 1;
}

void PathManager::deletePath(int i) {
    paths.erase(paths.begin() + i);

    if(paths.empty()) {
        active = -1;
    } else if (active == i) {
        active = std::min(i, (int)paths.size() - 1);
    } else {
        active--;
    }
}

void PathManager::draw() {
    for(Trajectory& path : paths) {
        path.draw(maxSpeed);
    }
}

void PathManager::moveSelected(Vector2 mousePos) {
    selected().moveSelected(mousePos);
}



void PathManager::handleClick(Vector2 mousePos){
    if(selected().selectAt(mousePos)) {
        dragging = true; 
        return;
    }
    for(int i = 0; i < (int)paths.size(); i++) {
        if(i != active && paths[i].selectAt(mousePos)) {
            setActive(i);
            dragging = true;
            return;
        }
    }
    selected().addPoints(mousePos);
    dragging = true;
}

void PathManager::saveState() {
    if(dragging) return;
    ManagerState now;
    for(Trajectory& path : paths) {
        now.paths.push_back(path.getState());
    }
    now.active = active;
    now.density = density;
    now.maxSpeed = maxSpeed;
    bool sameState = true;
    if(now.active == lastSavedState.active && now.paths.size() == lastSavedState.paths.size()) {
        for(int i = 0; i < (int)now.paths.size(); i++) {
            if(now.paths[i].controlPoints != lastSavedState.paths[i].controlPoints
            || now.paths[i].controlHeading != lastSavedState.paths[i].controlHeading
            || now.paths[i].velNodes != lastSavedState.paths[i].velNodes
            || now.density != lastSavedState.density
            || (now.maxSpeed != lastSavedState.maxSpeed)) {
                sameState = false;
                break;
            }
        }
    } else {sameState = false;}
    if(sameState) return;
    history.push_back(now);
    redoStack.clear();
    lastSavedState = now;
}

void PathManager::redo() {
    if(redoStack.empty()) return;
    ManagerState nextState = redoStack.back();
    redoStack.pop_back();
    history.push_back(nextState);
    paths.resize(nextState.paths.size());
    for(int i = 0; i < (int)paths.size(); i++) {
        paths[i].setState(nextState.paths[i]);
    }
    active = nextState.active;
    density = nextState.density;
    maxSpeed = nextState.maxSpeed;
    lastSavedState = nextState;
}

void PathManager::undo() {
    if(history.size() < 2) return;
    redoStack.push_back(history.back());
    history.pop_back();
    const ManagerState& lastState = history.back();
    paths.resize(lastState.paths.size());
    for(int i = 0; i < (int)paths.size(); i++) {
        paths[i].setState(lastState.paths[i]);
    }
    active = lastState.active;
    density  = lastState.density;
    maxSpeed = lastState.maxSpeed;
    lastSavedState = lastState;
}

bool PathManager::exportPaths(const std::string& fileName) const {
    std::ofstream file(fileName);
    for(size_t i = 0; i < paths.size(); i++) {
        file << "#" << paths[i].getName() << "\n";
        paths[i].exportStates(file, maxSpeed);
    }
    return true;
}

void PathManager::rebuildAll() {
    for(Trajectory& path : paths) {
        path.rebuildPoints(density, maxSpeed);
    }
}