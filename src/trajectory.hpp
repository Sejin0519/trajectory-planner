#pragma once
#include <vector>
#include "raylib.h"
#include "raymath.h"
#include "cubicBezier.hpp"
#include "util.hpp"
#include <fstream>


struct VelPoint {
    float x; //distance or time
    float speed;
    bool operator==(const VelPoint& other) const {
        return(x == other.x && speed == other.speed);
    }
    bool operator<(const VelPoint& other) const {
        return x < other.x;
    }
};


struct PathState {
    std::vector<Vector2> controlPoints;
    std::vector<float>   controlHeading;
    std::vector<VelPoint>  velNodes;
};

class Trajectory {
    std::vector<Vector2> controlPoints;
    std::vector<float> controlHeading; //only at anchors - [3*k]
    std::vector<Vector2> prevControlPoints;
    std::vector<float> prevControlHeading;
    std::vector<Pose> points;
    std::vector<PathState> history;
    std::vector<Vector2> lastSavedPoints;
    std::vector<float> lastSavedHeadings;
    std::string name = "Path";
    std::vector<VelPoint> velNodes;
    std::vector<VelPoint> prevVelNodes;
    std::vector<VelPoint> velocityProfile;
    float prevMaxVel = 0.0f;
    float prevDensity = 0.0f;

    float totalLength = 0.0f;


    int selected = -1;
public:
    const std::string& getName() const {return name;}
    float getTotalLength() const {return totalLength;}
    std::vector<VelPoint>& getVelNodes() {return velNodes;}
    std::vector<VelPoint>& getVelocityProfile() {return velocityProfile;}
    void setName(const std::string& newName) {name = newName;}
    void initializePoints();
    void addPoints(Vector2 mouse);
    void draw(float maxVel) const;

    void deleteSelected();
    bool selectAt(Vector2 mouse);
    bool isSelected() { return selected != -1; }
    void unSelect() { selected = -1; }
    void moveSelected(Vector2 mouse);
    void rebuildPoints(float density, float maxVel);

    bool isAnchorSelected() const {return selected != -1 && isAnchorPoint(selected);}
    Vector2 selectedPos() const {return controlPoints[selected];}
    float selectedHeading() const {return controlHeading[selected / 3];}
    float speedAt(float s, float maxVel) const;
    void  setSelectedPos(Vector2 position)  { controlPoints[selected] = position; }
    void  setSelectedHeading(float heading) { controlHeading[selected / 3] = heading; }

    void exportStates(std::ofstream& file, float maxVel) const;


    void saveState(bool dragging);
    void undo();

    PathState getState() const {return {controlPoints, controlHeading, velNodes};}
    void setState(const PathState& s) {
        controlPoints = s.controlPoints;
        controlHeading = s.controlHeading;
        velNodes = s.velNodes;
        
        prevControlPoints.clear();
        prevControlHeading.clear();
        prevVelNodes.clear();
        lastSavedPoints = controlPoints;
        lastSavedHeadings = controlHeading;

    }
    void constructVelocityProfile(float maxVel, float density);
private:
    bool isAnchorPoint(int index) const { return index % 3 == 0; }
};