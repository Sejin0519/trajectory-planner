#pragma once
#include "raylib.h"
#include "raymath.h"
#include <vector>

#define LUT_SIZE 256

struct CubicBezier {
    Vector2 p0, p1, p2, p3;
    float totalLength;
    std::vector<float> lut; //stores accumulated distance at each t

    Vector2 Evaluate(float t) const;
    void precomputeArcLength();
    float getTFromArcLength(float s) const;
    Vector2 getPointAtArcLength(float s) const;
};
