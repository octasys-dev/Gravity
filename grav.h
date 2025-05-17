// g.h – Interface declarations for the Pure Data wrapper
// This header defines all PD-specific interaction methods and the wrapper struct

#ifndef GRAV_H
#define GRAV_H

extern "C"
{
#include "m_pd.h"
}

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include "Gravity.h"

// Internal data structure for the Pure Data object
struct t_grav
{
    t_object x_obj;                                    // Required for PD objects
    t_float x_f;                                       // DSP inlet dummy
    std::chrono::steady_clock::time_point last_time;   // Time tracking
    std::chrono::steady_clock::time_point last_output; // Timestamp of the last data output to ensure output rate matches simulation timing
    int timestep_ms;                                   // Time step interval in milliseconds
    int internal_steps;                                // Simulation steps per PD tick
    float expand_scale;                                // Scale für value ranges
    float limit_max;                                   // maximum for scaled value
    bool running;                                      // Is simulation active?
    bool limits;                                       // Limits -100 100 on/off

    t_outlet *out_bang;       // Bang for step finished
    t_outlet *out_pos;        // Position outlet
    t_outlet *out_vel;        // Velocity outlet
    t_outlet *out_acc;        // Acceleration outlet
    t_outlet *out_hole;       // Black hole outlet
    t_outlet *out_params;     // Simulation paramters
    t_outlet *out_initvalues; // Body initialization values

    std::thread worker;               // Background thread that runs the simulation loop independently of the DSP thread
    std::atomic<bool> running_thread; // Thread control flag: true while the background simulation thread should continue running
    t_clock *clock;
    std::vector<Body> buffer_read;
    std::vector<Body> buffer_write;
    Body hole_read;
    Body hole_write;
    std::mutex swap_mutex;

    Gravity *system; // Pointer to the simulation system
};

#endif // GRAF_H
