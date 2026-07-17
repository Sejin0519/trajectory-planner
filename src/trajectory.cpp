#include "trajectory.hpp"
#include "util.hpp"

void Trajectory::initializePoints() {
    if(!controlPoints.empty()) return;
    controlPoints.push_back({0, 0}); //TODO: Change this
    controlHeading.push_back(0.0f);
    controlPoints.push_back({0, 0});
    controlPoints.push_back({0, 0});
    controlPoints.push_back({0, 0});
    controlHeading.push_back(0.0f);
}

void Trajectory::addPoints(Vector2 mouse) {
    if(controlPoints.empty()) {
        controlPoints.push_back({0, 0});
        controlPoints.push_back({0, 25});
        controlHeading.push_back(0.0f);
    } else {
        Vector2 lastPoint = controlPoints.back();
        Vector2 lastControlPoint = controlPoints[controlPoints.size() - 2];
        controlPoints.push_back(lastPoint * 2 - lastControlPoint);
    }
    controlPoints.push_back({mouse.x, mouse.y - 25});
    controlPoints.push_back(mouse);
    selected = controlPoints.size() - 1;
    controlHeading.push_back(0.0f);
}

void Trajectory::rebuildPoints(float density, float maxVel) {
    if((controlPoints == prevControlPoints && controlHeading == prevControlHeading && velNodes == prevVelNodes && density == prevDensity && maxVel == prevMaxVel) || controlPoints.empty()) return;
    prevControlPoints = controlPoints;
    prevControlHeading = controlHeading;
    prevDensity = density;
    prevMaxVel = maxVel;

    prevVelNodes = velNodes;
    points.clear();
    int segments = (controlPoints.size() - 1) / 3;
    CubicBezier bezier;
    float carry = 0.0f; //distance carried over segments
    totalLength = 0.0f;
    for(int i = 0; i < segments; i++) {
        bezier.p0 = controlPoints[i * 3];
        bezier.p1 = controlPoints[i * 3 + 1];
        bezier.p2 = controlPoints[i * 3 + 2];
        bezier.p3 = controlPoints[i * 3 + 3];
        bezier.precomputeArcLength();
        totalLength += bezier.totalLength;
        float s = carry;
        while(s < bezier.totalLength) {
            Vector2 point = bezier.getPointAtArcLength(s);
            float heading = lerpAngle(controlHeading[i], controlHeading[i+1], s/bezier.totalLength);
            points.push_back({point, heading});
            s += density;
        }
        carry = s - bezier.totalLength;
    }
    points.push_back({controlPoints.back(), controlHeading.back()});
    constructVelocityProfile(maxVel, density);

}

void Trajectory::deleteSelected() {
    if(!isAnchorSelected()) return;
    if(controlPoints.size() <= 4) {
        controlPoints.clear();
        controlHeading.clear();
        selected = -1;
        return;
    }
    int deleteIndex = selected;
    if(selected == (int)controlPoints.size() - 1) deleteIndex-=2;
    else if(selected != 0) deleteIndex--;
    controlPoints.erase(controlPoints.begin() + deleteIndex, controlPoints.begin() + deleteIndex + 3);
    controlHeading.erase(controlHeading.begin() + selected / 3);
    selected = -1;
}

void Trajectory::draw(float maxVel) const {
    if(controlPoints.empty()) return;
    for(int i = 0; i < points.size(); i++) {
        float ratio = velocityProfile[i].speed / maxVel;
        float hue = ratio * 120.0f; 
        Color color = ColorFromHSV(hue, 0.6f, 0.7f);
        DrawCircleV({points[i].position.x, points[i].position.y}, 2.5*0.25f, color);
    }
    // DrawSplineBezierCubic(controlPoints.data(), controlPoints.size(), 3*0.25f, BLUE);

    for(int i = 0; i < controlPoints.size(); i++){
        int index = i % 3;
        bool isSelected = (i == selected);
        float selectedOpacity = isSelected ? 0.3f : 0.0f;
        float selectedSize = isSelected ? 1.5f * 0.25f : 0.0f;
        if(isAnchorPoint(i)){
            DrawCircleV(controlPoints[i], (10*0.25f + selectedSize), Fade(BLUE, 0.5f + selectedOpacity));
            Vector2 endPointHeading = controlPoints[i] + Vector2Rotate({0, -10*0.25f}, degToRad(controlHeading[i/3]));
            DrawLineV(controlPoints[i], endPointHeading, BLACK);
        } else if(index == 1){
            DrawCircleV(controlPoints[i], (6*0.25f + selectedSize), Fade(BLUE, 0.5f + selectedOpacity));
            DrawLineV(controlPoints[i - 1], controlPoints[i], Fade(BLACK, 0.3f));
        } else {
            DrawCircleV(controlPoints[i], (6*0.25f + selectedSize), Fade(BLUE, 0.5f + selectedOpacity));
            DrawLineV(controlPoints[i], controlPoints[i + 1], Fade(BLACK, 0.3f));
        }
    }
}

bool Trajectory::selectAt(Vector2 mouse) {
    for(int i = 0; i < controlPoints.size(); i++){
        int radius = isAnchorPoint(i) ? 10*0.25f : 6*0.25f;
        if(CheckCollisionPointCircle(mouse, controlPoints[i], radius)) {
            selected = i;
            return true;
        }
    }
    return false;
}

void Trajectory::moveSelected(Vector2 mouse) {
    if(selected != -1) {
        controlPoints[selected] = mouse;
    }
}


void Trajectory::saveState(bool dragging) { //not in use, moved to PathManager
    if((controlPoints == lastSavedPoints && controlHeading == lastSavedHeadings) || dragging) return;

    history.push_back({controlPoints, controlHeading});
    lastSavedPoints  = controlPoints;                   
    lastSavedHeadings = controlHeading;
}

void Trajectory::undo() {
    if(history.size () < 2) {
        controlPoints.clear();
        controlHeading.clear();
        history.clear();
    } else {
        history.pop_back();
        controlPoints = history.back().controlPoints;
        controlHeading = history.back().controlHeading;
    }
    lastSavedHeadings = controlHeading;
    lastSavedPoints = controlPoints;
}

void Trajectory::constructVelocityProfile(float maxVel, float density) {
    velocityProfile.clear();
    for (float s = 0; s <= totalLength; s += density) {
        velocityProfile.push_back({s, speedAt(s, maxVel)});
    }
    velocityProfile.push_back({totalLength, speedAt(totalLength, maxVel)});
}



float Trajectory::speedAt(float s, float maxVel) const {
    if (velNodes.empty()) return maxVel;
    if (s <= velNodes.front().x) return velNodes.front().speed;
    if (s >= velNodes.back().x)  return velNodes.back().speed;
    for (int i = 0; i < (int)velNodes.size() - 1; i++) {
        if (s <= velNodes[i + 1].x) {
            float t = (s - velNodes[i].x) / (velNodes[i + 1].x - velNodes[i].x);
            return Lerp(velNodes[i].speed, velNodes[i + 1].speed, t);
        }
    }
    return velNodes.back().speed;
}

void Trajectory::exportStates(std::ofstream& file, float maxVel) const {
    size_t numStates = points.size();
    if(numStates <= 1) return;

    std::vector<float> t(numStates); //time at each state
    t[0] = 0.0f;
    for(size_t i = 1; i < numStates; i++) {
        float distance = Vector2Distance(points[i].position, points[i-1].position);
        float averageSpeed = (velocityProfile[i].speed + velocityProfile[i-1].speed) / 2.0f;
        float time = averageSpeed > 0.0f ? distance / averageSpeed : 0.0f;
        t[i] = t[i-1] + time;
    }

    file << "t, x, y, theta, xdot, ydot, thetadot\n";
    for(size_t i = 0; i < numStates; i++) {
        size_t IndexBehind = (i == 0) ? 0 : i - 1;
        size_t IndexAhead = (i == numStates - 1) ? numStates - 1 : i + 1;
        Vector2 displacement = Vector2Subtract(points[IndexAhead].position, points[IndexBehind].position);
        Vector2 unitTangent = (Vector2Length(displacement) > 0.0f) ? Vector2Normalize(displacement) : Vector2{0.0f, 0.0f};

        float speed = velocityProfile[i].speed;
        float xdot = unitTangent.x * speed;
        float ydot = -unitTangent.y * speed; //invert y-axis for output
        float theta = degToRad(-points[i].heading);
        float dtheta = (float)(degToRad(wrapAngle(points[IndexAhead].heading - points[IndexBehind].heading)));
        float dt = t[IndexAhead] - t[IndexBehind];
        float thetadot = (dt > 0.0f) ? dtheta / dt : 0.0f;
        file << t[i] << ", " << points[i].position.x << ", " << -points[i].position.y << ", " << theta << ", " << xdot << ", " << ydot << ", " << thetadot << "\n";
        
    }

}