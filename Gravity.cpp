// g.cpp – Implementation of N-body gravitational simulation core
// This file implements the g class defined in g.h

#include "Gravity.h"
#include "GravityMath.h"
#include "m_pd.h" // For pd_error and post (Pure Data logging)

extern t_class *grav_class; // External reference for error logging

Gravity::Gravity()
{
    math = new GravityMath();
    body_count = 3;
    nudge_mode = false;
    nudge_step = 0;

    initParams();
    loadPreset(0);
    post("");
    post("      .     o");
    post("        \\  /|\\        .");
    post("      o---> * <---o");
    post("        /  \\|/        o");
    post("           o     .");
    post("");
    post("Chaos engaged – gravitational system online");
}

// Destructor
Gravity::~Gravity()
{
    delete math;
}

// Initializes simulation parameters
void Gravity::initParams()
{
    G = 1.0f;
    dt = 0.01f;
    pos_damping = 0.0005f;
    vel_damping = 0.005f;
    softening = 0.1f;
    vmin = 1;
    vmax = 5;
}

// Zeroes all body values
void Gravity::resetBodies()
{
    for (int i = 0; i < BodyCount; ++i)
    {
        bodies[i] = Body{};
        initBodies[i] = Body{};
    }
}

// Sets the gravity constant
void Gravity::setG(float g)
{
    if (g < 0.0f || g > 10.0f)
    {
        pd_error(grav_class, "[grav] G must be in (0.1, 10], got %f", g);
        return;
    }

    G = (g < 0.1f) ? 0.1 : g;
}

// Sets the delta time between two simulations steps
void Gravity::setDt(float dt)
{
    if (dt <= 0.0f || dt > 0.1f)
    {
        pd_error(grav_class, "[grav] dt must be in (0.001, 0.1], got %f", dt);
        return;
    }
    this->dt = dt;
}

// Sets the position damping factor
void Gravity::setPosDamping(float damp)
{
    if (damp < 0.0f || damp > 0.1f)
    {
        pd_error(grav_class, "[grav] posdamp must be in [0.0, 0.1], got %f", damp);
        return;
    }

    pos_damping = damp;
}

// Sets the velocity damping factor
void Gravity::setVelDamping(float damp)
{
    if (damp < 0.0f || damp > 0.5f)
    {
        pd_error(grav_class, "[grav] veldamp must be in [0.0, 0.5], got %f", damp);
        return;
    }

    vel_damping = damp;
}

// Set base softening value to prevent singularities
void Gravity::setSoftening(float s)
{
    if (s < 0.0f || s > 5.0f)
    {
        pd_error(grav_class, "[grav] softening must be in (0.0, 5.0], got %f", s);
        return;
    }

    softening = s;
}

// Set the minimal velocity
void Gravity::setVmin(float v)
{
    if (v < 0.1f || v > 1.0f)
    {
        pd_error(grav_class, "[grav] vmin must be in (0.1, 1.0], got %f", v);
        return;
    }

    vmin = v;
    if (vmax < v)
        vmax = v;
}

// Set the maximal velocity
void Gravity::setVmax(float v)
{
    if (v < 1.0f || v > 5.0f)
    {
        pd_error(grav_class, "[grav] vmax must be in (1.0, 5.0], got %f", v);
        return;
    }

    vmax = v;

    if (vmin > v)
        vmin = 0.0001;
}

// Sets the number of active bodies
void Gravity::setBodyCount(int count)
{
    if (count < 2 || count > BodyCount)
    {
        pd_error(grav_class, "[grav] count must be between 2 and %d, got %d", BodyCount, count);
        return;
    }

    body_count = count;
}

// Sets a bodies mass at simulation time
void Gravity::setBodyMass(int index, float mass)
{
    if (index < 0 || index > BodyCount - 1)
    {
        pd_error(grav_class, "[grav] index must be between 0 and %d, got %d", BodyCount - 1, index);
        return;
    }

    if (mass < 0.1 || mass > 30)
    {
        pd_error(grav_class, "[grav] mass must be between 0.1 and 30, got %f", mass);
        return;
    }

    Body &body = bodies[index];
    body.mass = mass;
}

// Sets position and mass for the black hole
void Gravity::setBlackHole(float x, float y, float mass)
{
    if (x < -100 || x > 100)
    {
        pd_error(grav_class, "[grav] black hole x must be between -50 and 50, got %f", x);
        return;
    }

    if (y < -100 || y > 100)
    {
        pd_error(grav_class, "[grav] black hole y must be between -50 and 50, got %f", y);
        return;
    }

    if (mass < 0 || mass > 10000)
    {
        pd_error(grav_class, "[grav] black hole mass must be between 0 and 10000, got %f", mass);
        return;
    }

    blackHole.x = x;
    blackHole.y = y;
    blackHole.vx = 0;
    blackHole.vy = 0;
    blackHole.ax = 0;
    blackHole.ay = 0;

    blackHole.mass = mass;
}

// Gets the body with a given index
const Body &Gravity::getBody(int index) const
{
    if (index < 0 || index > BodyCount - 1)
    {
        pd_error(grav_class, "[grav] index must be between 0 and %d, got %d => 1. body returned", BodyCount - 1, index);
        return bodies[0];
    }

    return bodies[index];
}

// Returns a copy of all current body states for thread safety
std::vector<Body> Gravity::getBodies() const
{
    return std::vector<Body>(bodies, bodies + body_count); // std::vector<Body>
}

// Gets the body with a given index
const Body &Gravity::getInitBody(int index) const
{
    if (index < 0 || index > BodyCount - 1)
    {
        pd_error(grav_class, "[grav] nr must be between 0 and %d, got %d => 1. body returned", BodyCount - 1, index);
        return initBodies[0];
    }

    return initBodies[index];
}

// initializes a bodies starting values
void Gravity::initBody(int index)
{
    Body &body = bodies[index];

    // Compute initial acceleration from all other bodies
    Vector v = computeAcceleration(index);

    // Calculate distance to origin to appy position damping
    float pdamp = math->calcPositionDamping(body.x, body.y, pos_damping);
    body.ax = v.x - body.x * pdamp;
    body.ay = v.y - body.y * pdamp;
}

void Gravity::setBody(int index, float x, float y, float vx, float vy, float mass)
{
    // Validate index range to avoid out-of-bounds access
    if (index < 0 || index > BodyCount - 1)
        return;

    Body &body = bodies[index];
    Body &iBody = initBodies[index];

    body.x = x;
    body.y = y;
    body.vx = vx;
    body.vy = vy;
    body.mass = mass;

    iBody.x = x;
    iBody.y = y;
    iBody.vx = vx;
    iBody.vy = vy;
    iBody.mass = mass;
    iBody.ax = 0;
    iBody.ay = 0;

    initBody(index);
}

// resets the bodies to init values
void Gravity::reset()
{
    for (int i = 0; i < 10; ++i)
    {
        Body &body = bodies[i];
        Body &iBody = initBodies[i];

        body.x = iBody.x;
        body.y = iBody.y;
        body.vx = iBody.vx;
        body.vy = iBody.vy;
        body.ax = iBody.ax;
        body.ay = iBody.ay;

        initBody(i);
    }
}

// Gets the black hole
const Body &Gravity::getBlackHole() const
{
    return blackHole;
}

// Computes a reduced time step when bodies get very close to each other.
// Ensures more simulation detail during close encounters.
float Gravity::computeAdaptiveDt() const
{
    float minDist = std::numeric_limits<float>::max();

    for (int i = 0; i < body_count; ++i)
    {
        for (int j = i + 1; j < body_count; ++j)
        {
            float dist = math->calcEuclideanDistance(bodies[i].x, bodies[i].y, bodies[j].x, bodies[j].y);
            if (dist < minDist)
                minDist = dist;
        }
    }

    float scale = 0.8f + 0.8f * std::tanh(minDist * 0.8f);

    float dampDt = dt * scale;

    // -nan prevention
    if (dampDt <= 0.0f || std::isnan(dampDt))
    {
        return 0.01f;
    }

    return dampDt;
    // return dt;
}

const float amin = 0.01f; // Minimal acceleration
// Minimal velocity calculation
void Gravity::applyMinSpeed()
{
    for (int i = 0; i < body_count; ++i)
    {
        Body &body = bodies[i];

        float v = math->calcSpeed(body.vx, body.vy);
        float a = math->calcAcceleration(body.ax, body.ay);

        if (v < vmin && a < amin)
        {
            // Random angle in [0, 2π)
            Vector v = math->randomImpulse(0.02f, 0.07f);
            body.vx += v.x;
            body.vy += v.y;
        }
    }
}

// Nudges the Bodies when they got stuck
void Gravity::nudge()
{
    nudge_mode = true;
}
// Implementation of the ThreeBodySystem methods
// Performs one simulation step using the Leapfrog integration method.
// Updates positions, calculates new accelerations, and updates velocities with damping.
void Gravity::simulate()
{
    float currentDt = computeAdaptiveDt();

    for (int i = 0; i < body_count; ++i)
    {
        Body &body = bodies[i];
        body.x += body.vx * currentDt + 0.5f * body.ax * currentDt * currentDt;
        body.y += body.vy * currentDt + 0.5f * body.ay * currentDt * currentDt;
    }

    // Store current accelerations to be used in velocity update
    float oldAx[body_count], oldAy[body_count];
    for (int i = 0; i < body_count; ++i)
    {
        oldAx[i] = bodies[i].ax;
        oldAy[i] = bodies[i].ay;
    }

    for (int i = 0; i < body_count; ++i)
    {
        Body &body = bodies[i];

        // Compute new acceleration including gravitational and position damping
        Vector v = computeAcceleration(i);

        // Damping increases with distance to prevent runaway trajectories
        float pdamp = math->calcPositionDamping(body.x, body.y, pos_damping);
        body.ax = v.x - body.x * pdamp;
        body.ay = v.y - body.y * pdamp;

        applyCloseBodyRepulsion(i, 0.02f, 0.001f, 1.0f, 0.1f);
    }

    // new positions
    for (int i = 0; i < body_count; ++i)
    {
        Body &body = bodies[i];

        // Velocity update using averaged acceleration (Leapfrog step 2)
        body.vx += 0.5f * (oldAx[i] + body.ax) * currentDt;
        body.vy += 0.5f * (oldAy[i] + body.ay) * currentDt;

        if (nudge_mode)
        {
            if (nudge_step == 20)
            {
                nudge_mode = false;
                nudge_step = 0;
            }

            float nudge_factor = 10 * (5.0f + pos_damping);
            Vector v = math->randomImpulse(-nudge_factor / 2, nudge_factor / 2);
            body.vx = v.x;
            body.vy = v.y;

            nudge_step++;
        }

        // Compute velocity magnitude for dynamic velocity damping
        float speed = math->calcSpeed(body.vx, body.vy);

        // Velocity damping increases with speed to limit energy escalation
        float vdamp = vel_damping * (1.f + speed);
        body.vx *= 1.f - vdamp;
        body.vy *= 1.f - vdamp;

        // Clamp velocity to minimum and maximum thresholds
        Vector v = math->clampSpeed(body.vx, body.vy, vmin, vmax);
        body.vx = v.x;
        body.vy = v.y;
    }

    applyMinSpeed();
}

// This helps prevent them from sticking together by applying a distance-based counter-force.
void Gravity::applyCloseBodyRepulsion(int index, float vmin, float amin, float repel_zone, float repel_max)
{
    if (index < 0 || index >= body_count)
        return;

    Body &body = bodies[index];

    // Check if body is stagnating
    float v = math->calcSpeed(body.vx, body.vy);
    float acc = math->calcAcceleration(body.ax, body.ay);

    bool isStagnating = (v < vmin && acc < amin);
    if (!isStagnating)
        return;

    // Strong random impulse to break deadlocks
    float angle = ((float)rand() / RAND_MAX) * 2.0f * M_PI;
    float impulse = 0.02f + ((float)rand() / RAND_MAX) * 0.05f;

    body.vx += impulse * std::cos(angle);
    body.vy += impulse * std::sin(angle);

    // Repulsion from nearby bodies
    for (int j = 0; j < body_count + 1; ++j)
    {
        if (j == index)
            continue;

        Body &nbbody = (j < body_count) ? bodies[j] : blackHole;

        if (j == body_count && blackHole.mass == 0)
            continue;

        Vector v = math->calcRelativePositionVector(nbbody.x, nbbody.y, body.x, body.y);

        if (std::abs(v.x) > repel_zone || std::abs(v.y) > repel_zone)
            continue;

        float dist_sqr = v.x * v.x + v.y * v.y;
        if (dist_sqr >= repel_zone * repel_zone)
            continue;

        float dist = std::sqrt(dist_sqr) + 1e-6f;
        float norm = 1.0f / dist;

        float factor = (repel_zone - dist) / repel_zone;
        float base_strength = repel_max * factor * factor;

        // Stronger jitter: ±100% of base strength
        float jitter = ((float)rand() / RAND_MAX - 0.5f) * base_strength * 2.0f;

        float fx = (base_strength + jitter) * v.x * norm;
        float fy = (base_strength + jitter) * v.y * norm;

        body.ax -= fx;
        body.ay -= fy;
    }
}

// Computes the gravitational acceleration on the body at targetIndex
// from all other bodies, including softening to avoid singularities.
Vector Gravity::computeAcceleration(int targetIndex) const
{
    const Body &target = bodies[targetIndex];
    float ax = 0, ay = 0;

    for (int i = 0; i < body_count + 1; ++i)
    {
        // Skip self-interaction
        if (i == targetIndex)
            continue;

        const Body &other = (i < body_count) ? bodies[i] : blackHole;

        if (i == body_count && blackHole.mass == 0)
            continue;

        // Compute relative position vector
        Vector v = math->calcRelativePositionVector(target.x, target.y, other.x, other.y);

        // Compute Euclidean distance between target and other
        float distance = math->calcEuclideanDistance(v.x, v.y);

        // Apply softening to reduce numerical instability at short ranges
        float currentSoftening = std::max(softening, distance * softening);

        // Compute softened squared distance for force calculation
        float distSqr = v.x * v.x + v.y * v.y + currentSoftening * currentSoftening;

        if (distSqr <= 0.0f || distSqr < 0.0001f)
        {
            continue;
        }

        // Compute inverse distance and its cube
        float invDist = 1.0f / std::sqrt(distSqr);
        float invDist3 = invDist * invDist * invDist;

        // Accumulate gravitational acceleration components
        ax += G * other.mass * v.x * invDist3;
        ay += G * other.mass * v.y * invDist3;
    }

    // Return the total acceleration vector acting on the target body
    Vector v;
    v.x = ax;
    v.y = ay;
    return v;
}

// Loads one of ten predefined body configurations and sets active body count.
void Gravity::loadPreset(int presetIndex)
{
    // Clamp preset index to valid range
    int p = presetIndex < 1 ? 1 : (presetIndex > 14 ? 14 : presetIndex);

    p--;

    resetBodies();

    switch (p)
    {
    case 0:
        // Simple rotating ring around massive center
        setG(0.1f);
        setDt(0.01f);
        setSoftening(0.3f);
        setPosDamping(0.001f);
        setVelDamping(0.005f);
        setBodyCount(10);

        setBody(0, 50, 0, 0, 0.4, 1);
        setBody(1, 40, 40, -0.3, 0.3, 1);
        setBody(2, 0, 50, -0.4, 0.0, 1);
        setBody(3, -40, 40, -0.3, -0.3, 1);
        setBody(4, -50, 0, 0, -0.4, 1);
        setBody(5, -40, -40, 0.3, -0.3, 1);
        setBody(6, 0, -50, 0.4, 0.0, 1);
        setBody(7, 40, -8, 0.3, 0.3, 1);
        setBody(8, 0, 0, 0.0, 0.0, 3);
        setBody(9, 0, 30, 0.0, 0.0, 0.5);
        break;
    case 1:
        // Asymmetric cluster with slow drift
        setG(0.1f);
        setDt(0.01f);
        setSoftening(0.3f);
        setPosDamping(0.001f);
        setVelDamping(0.01f);
        setBodyCount(10);

        setBody(0, 50, 20, 0.1, 0.05, 1);
        setBody(1, -40, -10, -0.1, 0.1, 1);
        setBody(2, -60, 70, 0.1, -0.05, 1);
        setBody(3, 30, -60, -0.1, -0.1, 1);
        setBody(4, 10, 10, 0.0, 0.0, 2);
        setBody(5, -10, -40, 0.05, 0.0, 0.8);
        setBody(6, 0, -70, 0.0, 0.1, 0.8);
        setBody(7, -80, 20, 0.1, 0.0, 0.8);
        setBody(8, 70, -20, -0.05, 0.1, 0.8);
        setBody(9, 0, 0, 0.0, 0.0, 3);
        break;
    case 2:
        // Spiral start with mild rotation
        setG(0.1f);
        setDt(0.01f);
        setSoftening(0.3f);
        setPosDamping(0.001f);
        setVelDamping(0.01f);
        setBodyCount(10);

        for (int i = 0; i < 10; ++i)
        {
            float angle = i * 0.6f;
            float radius = 20.0f * i;
            float x = std::cos(angle) * radius;
            float y = std::sin(angle) * radius;
            float vx = -std::sin(angle) * 0.2f;
            float vy = std::cos(angle) * 0.2f;
            setBody(i, x, y, vx, vy, 1.0f);
        }
        break;
    case 3:
        // Symmetrical cross
        setG(0.3f);
        setDt(0.008f);
        setSoftening(0.3f);
        setPosDamping(0.002f);
        setVelDamping(0.01f);
        setBodyCount(10);

        for (int i = 0; i < 5; ++i)
        {
            setBody(i, 0, i * 50.0f - 100.0f, 0.2f, 0, 1.0f);
            setBody(i + 5, i * 50.0f - 100.0f, 0, 0, -0.2f, 1.0f);
        }
        break;
    case 4:
        // Circular orbit with center mass
        setG(0.1f);
        setDt(0.02f);
        setSoftening(0.5f);
        setPosDamping(0.001f);
        setVelDamping(0.002f);
        setBodyCount(10);

        for (int i = 0; i < 10; ++i)
        {
            float angle = 2 * M_PI * i / 9.0f;
            float x = 100.0f * std::cos(angle);
            float y = 100.0f * std::sin(angle);
            float vx = -std::sin(angle) * 0.5f;
            float vy = std::cos(angle) * 0.5f;
            setBody(i, x, y, vx, vy, 1.0f);
        }
        setBody(9, 0, 0, 0, 0, 5.0f);
        break;
    case 5:
        // Random cluster
        setG(0.5f);
        setDt(0.01f);
        setSoftening(0.4f);
        setPosDamping(0.003f);
        setVelDamping(0.01f);
        setBodyCount(BodyCount);

        for (int i = 0; i < BodyCount; ++i)
        {
            float x = (rand() % 200) - 100;
            float y = (rand() % 200) - 100;
            float vx = ((rand() % 200) - 100) * 0.005f;
            float vy = ((rand() % 200) - 100) * 0.005f;
            setBody(i, x, y, vx, vy, 0.5f + (rand() % 100) * 0.01f);
        }
        break;
    case 6:
        // Two binary systems plus orbiters
        setG(0.2f);
        setDt(0.01f);
        setSoftening(0.3f);
        setPosDamping(0.002f);
        setVelDamping(0.005f);
        setBodyCount(10);

        setBody(0, -50, 0, 0, 0.3, 1);
        setBody(1, -30, 0, 0, -0.3, 1);
        setBody(2, 50, 0, 0, -0.3, 1);
        setBody(3, 30, 0, 0, 0.3, 1);
        setBody(4, 0, 80, -0.3, 0, 1);
        setBody(5, 0, 60, 0.3, 0, 1);
        setBody(6, 0, -60, 0.3, 0, 1);
        setBody(7, 0, -80, -0.3, 0, 1);
        setBody(8, 0, 0, 0, 0, 2);
        setBody(9, 0, 20, 0, 0, 0.5);
        break;
    case 7:
        // Figure-eight approximation
        setG(1.0f);
        setDt(0.005f);
        setSoftening(0.01f);
        setPosDamping(0.0f);
        setVelDamping(0.0f);
        setBodyCount(3);

        setBody(0, 0, 0, 0.347111, 0.532728, 1);
        setBody(1, 0.970004, -0.243087, -0.347111, 0.532728, 1);
        setBody(2, -0.970004, 0.243087, 0, -1.065456, 1);
        for (int i = 3; i < BodyCount; ++i)
        {
            setBody(i, 0, 0, 0, 0, 0);
        }
        break;
    case 8:
        // Line of increasing mass and spacing
        setG(0.15f);
        setDt(0.01f);
        setSoftening(0.2f);
        setPosDamping(0.001f);
        setVelDamping(0.005f);
        setBodyCount(BodyCount);

        for (int i = 0; i < BodyCount; ++i)
        {
            float x = i * 50.0f;
            float y = 0;
            float vx = 0;
            float vy = (i - 5) * 0.1f;
            float mass = 0.5f + 0.5f * i;
            setBody(i, x, y, vx, vy, mass);
        }
        break;
    case 9:
        // Radial outburst from center
        setG(0.2f);
        setDt(0.008f);
        setSoftening(0.3f);
        setPosDamping(0.002f);
        setVelDamping(0.01f);
        setBodyCount(BodyCount);

        for (int i = 0; i < BodyCount; ++i)
        {
            float angle = 2 * M_PI * i / 10.0f;
            float vx = std::cos(angle) * 0.3f;
            float vy = std::sin(angle) * 0.3f;
            setBody(i, 0, 0, vx, vy, 1.0f);
        }
        break;
    case 10:
        // Asymetric chaos at high speed
        setG(0.15f);
        setDt(0.01f);
        setSoftening(0.05f);
        setPosDamping(0.0001f);
        setVelDamping(0.0005f);
        setBodyCount(5);

        setBody(0, -120, 80, 0.9, -0.4, 1.5f);
        setBody(1, 100, 60, -0.5, 0.6, 2.0f);
        setBody(2, 0, -100, 0.4, 0.8, 1.2f);
        setBody(3, 50, 50, -0.9, -0.2, 0.8f);
        setBody(4, -70, -80, 0.6, 0.3, 1.0f);
        break;
    case 11:
        // Chaos cluster drift
        setG(0.2f);
        setDt(0.008f);
        setSoftening(0.05f);
        setPosDamping(0.0002f);
        setVelDamping(0.001f);
        setBodyCount(6);

        setBody(0, -40, 20, 0.5, 0.4, 1.0f);
        setBody(1, 30, -10, -0.6, 0.3, 1.8f);
        setBody(2, 0, 0, 0.1, -0.5, 0.6f);
        setBody(3, -30, -30, 0.3, 0.6, 1.2f);
        setBody(4, 60, 10, -0.4, -0.3, 1.5f);
        setBody(5, -50, 40, 0.7, -0.1, 0.9f);
        break;
    case 12:
        // Chaos extreme
        setG(0.25f);
        setDt(0.007f);
        setSoftening(0.07f);
        setPosDamping(0.0001f);
        setVelDamping(0.0002f);
        setBodyCount(7);

        setBody(0, -200, 100, 1.0, -0.3, 1.2f);
        setBody(1, 180, 80, -0.8, 0.6, 2.1f);
        setBody(2, 0, -90, 0.5, 0.9, 0.7f);
        setBody(3, 60, 200, -1.1, -0.2, 1.4f);
        setBody(4, -160, -150, 0.9, 0.4, 1.0f);
        setBody(5, 30, -70, -0.3, -0.8, 0.8f);
        setBody(6, 90, 0, -0.5, 0.5, 1.6f);
        break;
    case 13: // Chaos  – scattered triangle with tangential velocity
        setG(0.15f);
        setDt(0.009f);
        setSoftening(0.05f);
        setPosDamping(0.0002f);
        setVelDamping(0.0005f);
        setBodyCount(3);

        // triangle
        setBody(0, -100.0f, -50.0f, 0.65f, 0.3f, 1.2f);
        setBody(1, 100.0f, -50.0f, -0.6f, 0.35f, 1.8f);
        setBody(2, 0.0f, 120.0f, -0.05f, -0.7f, 2.0f);
        break;
    default:
        // Default: all bodies at origin, mass 0
        break;
    }
}
