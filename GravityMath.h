#ifndef GRAVITYMATH_H
#define GRAVITYMATH_H

#include <utility>

// Holds delta
struct Vector
{
    float x;
    float y;
};

class GravityMath
{
public:
    GravityMath();

    // Calulates the relative position [deltax deltay] of two positions
    Vector calcRelativePositionVector(float x1, float y1, float x2, float y2) const;
    // Calculates the Euclidean distance based on deltax, deltay
    float calcEuclideanDistance(float dx, float dy) const;
    // Calculates the Euclidean distance from coordinates
    float calcEuclideanDistance(float x1, float y1, float x2, float y2) const;
    // Computes position-based damping factor (grows with distance from origin)
    float calcPositionDamping(float x, float y, float baseDamping) const;
    // Computes velocity magnitude (speed) from vx/vy
    float calcSpeed(float vx, float vy) const;
    // Computes acceleration from vx/vy
    float calcAcceleration(float ax, float ay) const;
    // Generates a random angle between 0 and 2π
    float randomAngle() const;
    // Generates a random float between min and max
    float randomRange(float min, float max) const;
    // Returns a velocity vector with random direction and magnitude
    Vector randomImpulse(float minStrength, float maxStrength) const;
    // Clamps the given velocity to a min/max speed, returns scaled pair
    Vector clampSpeed(float vx, float vy, float vmin, float vmax) const;

protected:
private:
};

#endif // GRAVITYMATH_H