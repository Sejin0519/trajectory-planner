#pragma once
#include <cmath>

inline double degToRad(double degrees) {
    return degrees * (M_PI / 180);
}

inline double radToDeg(double radians) {
    return radians * (180 / M_PI);
}

inline double wrapAngle(double angle, bool rad = false) { //-180 to 180
    if(rad) {
        double wrapped = std::fmod(angle + M_PI, 2 * M_PI);
        if(wrapped < 0) {
            wrapped += 2 * M_PI;
        }
        return wrapped - M_PI;
    }
    double wrapped = std::fmod(angle + 180, 360);
    if(wrapped < 0) {
        wrapped += 360;
    }
    return wrapped - 180;
}

inline double lerpAngle(double start, double end, double t, bool rad = false) {
    double difference = wrapAngle(end - start, rad);
    return wrapAngle(start + difference * t, rad); 
}

inline double lerp(double start, double end, double t) {
    return start + (end - start) * t;
}


struct Pose {
    Vector2 position;
    float heading;
    Pose(Vector2 p, float h) : position(p), heading(h) {}
    Pose(float x, float y, float h) : position{x, y}, heading(h) {}

};