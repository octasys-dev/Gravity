#include <cstdlib>
#include <utility>
#include <cmath>
#include "GravityMath.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GravityMath::GravityMath()
{
}

// Calulates the relative position [deltax deltay] of two positions
Vector GravityMath::calcRelativePositionVector(float x1, float y1, float x2, float y2) const
{
    float dx = x2 - x1;
    float dy = y2 - y1;

    Vector v;
    v.x = dx;
    v.y = dy;
    return v;
}

// Calculates the Euclidean distance of deltax, deltay
float GravityMath::calcEuclideanDistance(float dx, float dy) const
{
    return std::sqrt(dx * dx + dy * dy);
}

// Calculates the Euclidean distance of two points
float GravityMath::calcEuclideanDistance(float x1, float y1, float x2, float y2) const
{
    Vector v = calcRelativePositionVector(x1, y1, x2, y2);
    return calcEuclideanDistance(v.x, v.y);
}

// Computes position damping factor (based on distance from origin)
float GravityMath::calcPositionDamping(float x, float y, float baseDamping) const
{
    float dist = std::sqrt(x * x + y * y);
    return baseDamping * (1.0f + dist);
}

// Computes velocity magnitude (speed)
float GravityMath::calcSpeed(float vx, float vy) const
{
    return std::sqrt(vx * vx + vy * vy);
}

// Computes the acceleration
float GravityMath::calcAcceleration(float ax, float ay) const
{
    return std::sqrt(ax * ax + ay * ay);
}

// Generates random angle in [0, 2π)
float GravityMath::randomAngle() const
{
    return static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
}

// Generates random float in [min, max)
float GravityMath::randomRange(float min, float max) const
{
    return min + static_cast<float>(rand()) / RAND_MAX * (max - min);
}

// Returns a random impulse vector (vx, vy) with random direction and strength
Vector GravityMath::randomImpulse(float minStrength, float maxStrength) const
{
    float angle = randomAngle();
    float strength = randomRange(minStrength, maxStrength);

    Vector v;
    v.x = strength * std::cos(angle);
    v.y = strength * std::sin(angle);
    return v;
}

// Clamps velocity to a range [vmin, vmax]
Vector GravityMath::clampSpeed(float vx, float vy, float vmin, float vmax) const
{
    float speed = calcSpeed(vx, vy);

    if (speed == 0.0f)
        return {vx, vy};

    if (speed < vmin)
    {
        float scale = vmin / speed;
        return {vx * scale, vy * scale};
    }

    if (speed > vmax)
    {
        float scale = vmax / speed;
        return {vx * scale, vy * scale};
    }

    Vector v;
    v.x = vx;
    v.y = vy;
    return v;
}
