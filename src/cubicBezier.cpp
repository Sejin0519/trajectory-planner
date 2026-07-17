#include "cubicBezier.hpp"



    Vector2 CubicBezier::Evaluate(float t) const {
        Vector2 point;
        point.x = powf(1 - t, 3) * p0.x + 3 * powf(1 - t, 2) * t * p1.x + 3 * (1 - t) * powf(t, 2) * p2.x + powf(t, 3) * p3.x;
        point.y = powf(1 - t, 3) * p0.y + 3 * powf(1 - t, 2) * t * p1.y + 3 * (1 - t) * powf(t, 2) * p2.y + powf(t, 3) * p3.y;
        return point;
    }
    void CubicBezier::precomputeArcLength() {
        lut.clear();
        lut.resize(LUT_SIZE + 1);
        lut[0] = 0.0f;
        Vector2 prevPoint = p0;
        float accumulatedDistance = 0.0f;
        for (int i = 1; i <= LUT_SIZE; i++) {
            float t = (float) i/LUT_SIZE;
            Vector2 currentPoint = Evaluate(t);
            accumulatedDistance += Vector2Distance(prevPoint, currentPoint);
            lut[i] = accumulatedDistance;
            prevPoint = currentPoint;
        }
        totalLength = accumulatedDistance;
    }
    float CubicBezier::getTFromArcLength(float s) const {
        if (s <= 0) return 0.0f;
        if (s >= totalLength) return 1.0f;

        float targetDistance = s;

        //binary Search to find the closest t in the LUT
        int low = 0;
        int high = LUT_SIZE;
        while (low < high) {
            int mid = (low + high) / 2;
            if (lut[mid] < targetDistance) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        //Linear interpolation between the two closest points
        if (low < 1) low = 1;
        float dLow = lut[low - 1];
        float dHigh = lut[low];
        if (dHigh - dLow <= 0) return (float) low / LUT_SIZE;
        float t = ((float)(low-1) + (targetDistance - dLow) / (dHigh - dLow)) / LUT_SIZE;
        return t;
    } 
    Vector2 CubicBezier::getPointAtArcLength(float s) const {
        float t = getTFromArcLength(s);
        return Evaluate(t);
    }
