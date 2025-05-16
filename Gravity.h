// Gravity.h – Simulation core for N-body gravitational interaction
// This header defines the Body struct and the Gravity class which implements the physics simulation

#ifndef GRAVITY_H
#define GRAVITY_H

#include <utility>
#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include "GravityMath.h"

// Represents a single body in 2D space
struct Body
{
    float x, y;   // Position
    float vx, vy; // Velocity
    float ax, ay; // Acceleration
    float mass;   // Mass
};

// Encapsulates the physics simulation for up to 10 bodies
class Gravity
{
public:
    Gravity();  // Constructor 
    ~Gravity(); // Destructor
    static const int BodyCount = 10; // Number of bodies in the system

    void loadPreset(int presetIndex); // Load a predefined body configuration (0–13)

    void setG(float g);                              // Set gravitational constant
    void setDt(float d);                             // Set simulation time step
    void setPosDamping(float damp);                  // Set position-based damping coefficient
    void setVelDamping(float vd);                    // Set velocity-based damping coefficient
    void setSoftening(float s);                      // Set base softening value to prevent singularities
    void setVmin(float v);                           // Set the minimum velocity
    void setVmax(float v);                           // Set the maximum velocity
    void setBodyCount(int count);                    // Set how many bodies are active (2–10)
    void setBodyMass(int index, float mass);         // Sets a bodies mass at simulation time
    void setBlackHole(float x, float y, float mass); // Sets position and mass for the black hole

    void nudge(); // Nudges the Bodies when they got stuck

    float getG() const { return G; }                    // Get gravitational constant
    float getDt() const { return dt; }                  // Get simulation time step
    float getVmin() const { return vmin; }              // Gets the minimum velocity
    float getVmax() const { return vmax; }              // Gets the maximumn velocity
    float getPosDamping() const { return pos_damping; } // Get position damping coefficient
    float getVelDamping() const { return vel_damping; } // Get velocity damping coefficient
    float getSoftening() const { return softening; }    // Get base softening value
    int getBodyCount() const { return body_count; }     // Get current number of active bodies

    const Body &getBlackHole() const;         // Gets the black hole
    const Body &getBody(int index) const;     // Get body by index (current state)
    std::vector<Body> getBodies() const;      // Returns a copy of all current body states for thread safety
    const Body &getInitBody(int index) const; // Get initial body state by index

    void setBody(int index, float x, float y, float vx, float vy, float mass); // Set initial values for a body

    void reset();    // Reset all bodies to initial state and reinitialize
    void simulate(); // Perform one simulation step

private:
    GravityMath *math;              // Physics calculations
    void initParams();               // Initialize default simulation parameters
    void resetBodies();              // Resets every body value to 0
    void initBody(int index);        // Initialize a single body’s acceleration
    float computeAdaptiveDt() const; // Adaptive timestep depending on proximity
    void applyMinSpeed();            // Minimal velocity calculation

    // This helps prevent them from sticking together by applying a distance-based counter-force.
    void applyCloseBodyRepulsion(int index, float vmin, float amin, float repel_zone, float repel_max);

    Vector computeAcceleration(int targetIndex) const; // Calculate acceleration on one body

    Body initBodies[BodyCount]; // Array holding initial body states
    Body bodies[BodyCount];     // Array holding current body states
    Body blackHole;             // The black hole

    float G;           // Gravitational constant
    float dt;          // Timestep
    float pos_damping; // Damping based on distance from origin
    float vel_damping; // Damping based on body speed
    float softening;   // Base value to prevent singularities
    float vmin;        // Minimum vewlociy
    float vmax;        // Maximum velocity
    int body_count;    // Number of active bodies
    bool nudge_mode;   // nudge indicator for simulation
    int nudge_step;    // Current simulation step in nudging mode
};

#endif // GRAVITY_H
