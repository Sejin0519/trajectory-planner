#include "trajectory.hpp"
#include "util.hpp"
#include <algorithm>
#include <cmath>

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
        //add a control handle opposite to the previous one
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
    constructVelocityProfile(maxVel, density); //update the velocity graph

}

void Trajectory::deleteSelected() {
    if(!isAnchorSelected()) return;
    if(controlPoints.size() <= 4) { //delete everything if there are only 2 anchors
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
        Color color = ColorFromHSV(hue, 0.6f, 0.7f); //from green to red depending on speed
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

namespace {
    //Cubic Hermite interpolation on [0,len], matching value v0/slope m0 at x=0 and value v1/slope
    //m1 at x=len.
    float hermite(float v0, float m0, float v1, float m1, float len, float x) {
        float u = x / len;
        float u2 = u * u, u3 = u2 * u;
        float b0 = 2.0f*u3 - 3.0f*u2 + 1.0f;
        float b1 = -2.0f*u3 + 3.0f*u2;
        float bm0 = u3 - 2.0f*u2 + u;
        float bm1 = u3 - u2;
        return v0*b0 + v1*b1 + (m0*len)*bm0 + (m1*len)*bm1;
    }
}

float Trajectory::speedAt(float s, float maxVel) const {
    if (velNodes.empty()) return maxVel;
    if (s <= velNodes.front().x) return velNodes.front().speed;
    if (s >= velNodes.back().x)  return velNodes.back().speed;

    int n = (int)velNodes.size();
    const float cornerRadius = 6.0f; //how far the rounding extends into each adjoining segment

    for (int i = 0; i < n - 1; i++) {
        if (s > velNodes[i + 1].x) continue;
        float segStart = velNodes[i].x,     segEnd = velNodes[i + 1].x;
        float v0 = velNodes[i].speed,       v1 = velNodes[i + 1].speed;
        float slope = (v1 - v0) / (segEnd - segStart);

        //round the corner at this segment's left node (i), if it's an interior node with a
        //segment before it. Outside this small window, the line is untouched (pure linear).
        if (i > 0) {
            float prevSlope = (v0 - velNodes[i - 1].speed) / (segStart - velNodes[i - 1].x);
            float R = std::min({cornerRadius, 0.4f * (segStart - velNodes[i - 1].x), 0.4f * (segEnd - segStart)});
            if (R > 0.0f && s < segStart + R) {
                float mid = 0.5f * (prevSlope + slope); //shared meeting slope with the other half
                float endVal = v0 + slope * R; //where this rejoins the pure-linear line
                return Clamp(hermite(v0, mid, endVal, slope, R, s - segStart), 0.0f, maxVel);
            }
        }
        //round the corner at this segment's right node (i+1), if it's interior with a segment after.
        if (i + 1 < n - 1) {
            float nextSlope = (velNodes[i + 2].speed - v1) / (velNodes[i + 2].x - segEnd);
            float R = std::min({cornerRadius, 0.4f * (segEnd - segStart), 0.4f * (velNodes[i + 2].x - segEnd)});
            if (R > 0.0f && s > segEnd - R) {
                float mid = 0.5f * (slope + nextSlope);
                float startVal = v1 - slope * R; //where the pure-linear line would be here
                return Clamp(hermite(startVal, slope, v1, mid, R, s - (segEnd - R)), 0.0f, maxVel);
            }
        }
        //plain linear interpolation for the rest of the segment
        return Lerp(v0, v1, (s - segStart) / (segEnd - segStart));
    }
    return velNodes.back().speed;
}

std::vector<ExportState> Trajectory::buildStates(float maxVel) const {
    size_t numStates = points.size();
    if(numStates <= 1) return {};
    std::vector<ExportState> exportStates;

    std::vector<float> t(numStates); //time at each state
    t[0] = 0.0f;
    for(size_t i = 1; i < numStates; i++) { 
        float distance = Vector2Distance(points[i].position, points[i-1].position);
        float averageSpeed = (velocityProfile[i].speed + velocityProfile[i-1].speed) / 2.0f;
        float time = averageSpeed > 0.0f ? distance / averageSpeed : 0.0f;
        t[i] = t[i-1] + time;
    }

    for(size_t i = 0; i < numStates; i++) {
        size_t indexBehind = (i == 0) ? 0 : i - 1;
        size_t indexAhead = (i == numStates - 1) ? numStates - 1 : i + 1;

        Vector2 displacement = Vector2Subtract(points[indexAhead].position, points[indexBehind].position); //displacement = tangent vector
        Vector2 unitTangent = (Vector2Length(displacement) > 0.0f) ? Vector2Normalize(displacement) : Vector2{0.0f, 0.0f};

        float speed = velocityProfile[i].speed;
        ExportState currentState;
        currentState.time = t[i];

        currentState.x = points[i].position.x;
        currentState.y = -points[i].position.y;

        currentState.xDot = unitTangent.x * speed;
        currentState.yDot = -unitTangent.y * speed; //invert y-axis for output

        currentState.theta = degToRad(-points[i].heading);
        float dtheta = (float)(degToRad(wrapAngle(points[indexAhead].heading - points[indexBehind].heading)));
        float dt = t[indexAhead] - t[indexBehind];
        currentState.thetaDot = (dt > 0.0f) ? -dtheta / dt : 0.0f;

        exportStates.push_back(currentState);
    }

    //accleration
    for (size_t i = 0; i < numStates; i++) {
        size_t indexBehind = (i == 0) ? 0 : i - 1;
        size_t indexAhead = (i == numStates - 1) ? numStates - 1 : i + 1;
        float dt = t[indexAhead] - t[indexBehind];
        if (dt > 0.0f) {
            exportStates[i].xDDot = (exportStates[indexAhead].xDot - exportStates[indexBehind].xDot) / dt;
            exportStates[i].yDDot = (exportStates[indexAhead].yDot - exportStates[indexBehind].yDot) / dt;
            exportStates[i].thetaDDot = (exportStates[indexAhead].thetaDot - exportStates[indexBehind].thetaDot) / dt;
        } else {
            exportStates[i].xDDot = 0.0f;
            exportStates[i].yDDot = 0.0f;
            exportStates[i].thetaDDot = 0.0f;
        }
    }

    return exportStates;
}