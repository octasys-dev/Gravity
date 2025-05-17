// g.cpp - gravity simulation as a Pure Data External (C++) with up to 10 bodies
// This file has been split into simulation and PD interface parts.
// You are currently viewing: Pure Data wrapper implementation

#include "grav.h"

t_class *grav_class = nullptr;

// Project output values (optional transformation)
static float project(t_grav *x, float val)
{
    return val * x->expand_scale;
}

// Sends data for each body and black hole
// outlets
// 1: bang on finished
// 2: Body Nr, x, y
// 3: Body Nr, Vx, Vy
// 4: Body Nr, Ax, Ay
// 5: Black hole
static void grav_out(t_grav *x)
{
    Body hole;
    std::vector<Body> copy;

    // Safe data access using try_lock to avoid blocking Pd thread
    hole = x->hole_read;
    copy = x->buffer_read;

    // outlet 5
    t_atom holelist[2];
    SETFLOAT(&holelist[0], project(x, hole.x));     // x
    SETFLOAT(&holelist[1], project(x, hole.y));     // y
    outlet_list(x->out_hole, &s_list, 2, holelist); // black hole => outlet 5

    for (int i = 0; i < static_cast<int>(copy.size()); ++i)
    {
        const Body &body = copy[i];

        float bx = project(x, body.x);
        float by = project(x, body.y);

        if (x->limits)
        {
            if (bx < -x->limit_max || bx > x->limit_max || by < -x->limit_max || by > x->limit_max)
            {
                continue;
            }
        }

        // outlet 2
        t_atom poslist[3];
        SETFLOAT(&poslist[0], i);  // Body nr
        SETFLOAT(&poslist[1], bx); // X
        SETFLOAT(&poslist[2], by); // Y

        // outlet 3
        t_atom vellist[3];
        SETFLOAT(&vellist[0], i);                   // Body nr
        SETFLOAT(&vellist[1], project(x, body.vx)); // Vx
        SETFLOAT(&vellist[2], project(x, body.vy)); // Vy

        // outlet 4
        t_atom acclist[3];
        SETFLOAT(&acclist[0], i);                   // Body nr
        SETFLOAT(&acclist[1], project(x, body.ax)); // Ax
        SETFLOAT(&acclist[2], project(x, body.ay)); // Ay

        outlet_list(x->out_acc, &s_list, 3, acclist); // outlet 4 accelerations
        outlet_list(x->out_vel, &s_list, 3, vellist); // outlet 3 velocities
        outlet_list(x->out_pos, &s_list, 3, poslist); // outlet 2 positions
    }

    outlet_bang(x->out_bang); // outlet 1 Finished
}

// Output of body initialization values on outlet 4
void grav_outinit(t_grav *x)
{
    t_atom output[6];

    for (int i = 0; i < x->system->BodyCount; ++i)
    {
        const Body &b = x->system->getInitBody(i);

        SETFLOAT(&output[0], i); // Numeric body index
        SETFLOAT(&output[1], b.x);
        SETFLOAT(&output[2], b.y);
        SETFLOAT(&output[3], b.vx);
        SETFLOAT(&output[4], b.vy);
        SETFLOAT(&output[5], b.mass);

        // Send to params outlet 4
        outlet_list(x->out_initvalues, &s_list, 6, output);
    }
}

// Output of body initialization values on outlet 5
void grav_outparams(t_grav *x)
{
    t_atom output;

    // Gravitational constant (G)
    SETFLOAT(&output, x->system->getG());
    outlet_anything(x->out_params, gensym("G"), 1, &output);

    // Time step (dt)
    SETFLOAT(&output, x->system->getDt());
    outlet_anything(x->out_params, gensym("dt"), 1, &output);

    // Position damping
    SETFLOAT(&output, x->system->getPosDamping());
    outlet_anything(x->out_params, gensym("posdamp"), 1, &output);

    // Velocity damping
    SETFLOAT(&output, x->system->getVelDamping());
    outlet_anything(x->out_params, gensym("veldamp"), 1, &output);

    // Softening
    SETFLOAT(&output, x->system->getSoftening());
    outlet_anything(x->out_params, gensym("softening"), 1, &output);

    // Vmin
    SETFLOAT(&output, x->system->getVmin());
    outlet_anything(x->out_params, gensym("vmin"), 1, &output);

    // Vmax
    SETFLOAT(&output, x->system->getVmax());
    outlet_anything(x->out_params, gensym("vmax"), 1, &output);

    // Active bodies
    SETFLOAT(&output, x->system->getBodyCount());
    outlet_anything(x->out_params, gensym("count"), 1, &output);

    // Speed
    SETFLOAT(&output, static_cast<float>(x->internal_steps) / 50.0f);
    outlet_anything(x->out_params, gensym("speed"), 1, &output);

    // Scale
    SETFLOAT(&output, x->expand_scale);
    outlet_anything(x->out_params, gensym("scale"), 1, &output);

    // Limits
    SETFLOAT(&output, x->limits ? 1 : 0);
    outlet_anything(x->out_params, gensym("limits"), 1, &output);

    Body blackHole = x->system->getBlackHole();
    t_atom args[3]; // x, y, m
    SETFLOAT(&args[0], blackHole.x);
    SETFLOAT(&args[1], blackHole.y);
    SETFLOAT(&args[2], blackHole.mass);
    outlet_anything(x->out_params, gensym("hole"), 3, args);
}

// Bang message: triggers one simulation step and sends output
void grav_bang(t_grav *x)
{
    x->system->simulate();
    grav_out(x);
}

// Sets speed of simulation as a float in range [0.0, 1.0]
void grav_speed(t_grav *x, t_floatarg val)
{
    if (val < 0.0f || val > 1.0f)
    {
        pd_error(x, "[g] speed must be in range [0.0, 1.0], got %.3f", val);
        return;
    }

    x->internal_steps = (val > 0) ? val * 50 : 1;
}

// Scaling of value ranges
void grav_scale(t_grav *x, t_floatarg val)
{
    if (val < 1.0f || val > 100.0f)
    {
        pd_error(x, "[g] scale must be in range [1.0, 100.0], got %.3f", val);
        return;
    }

    x->expand_scale = val;
}

// Limits -100 100 on/off
void grav_limits(t_grav *x, t_floatarg val)
{
    if (val < 0.0f || val > 1.0f)
    {
        pd_error(x, "[g] limits must be in range [0.0, 1.0], got %.3f", val);
        return;
    }

    x->limits = val != 0;
}

// Nudges the bodies
void grav_nudge(t_grav *x)
{
    x->system->nudge();
}

// Parameter setters delegate to simulation
void grav_dt(t_grav *x, t_floatarg val) { x->system->setDt(val); }
void grav_G(t_grav *x, t_floatarg val) { x->system->setG(val); }
void grav_posdamp(t_grav *x, t_floatarg val) { x->system->setPosDamping(val); }
void grav_veldamp(t_grav *x, t_floatarg val) { x->system->setVelDamping(val); }
void grav_softening(t_grav *x, t_floatarg val) { x->system->setSoftening(val); }
void grav_vmin(t_grav *x, t_floatarg val) { x->system->setVmin(val); }
void grav_vmax(t_grav *x, t_floatarg val) { x->system->setVmax(val); }
void grav_preset(t_grav *x, t_floatarg val) { x->system->loadPreset(static_cast<int>(val)); }
void grav_reset(t_grav *x, t_floatarg) { x->system->reset(); }
void grav_count(t_grav *x, t_floatarg val) { x->system->setBodyCount(static_cast<int>(val)); }

// Prints simulation parameters and states to PD console
void grav_dump(t_grav *x)
{
    post("[g] --- Parameters ---");
    post("dt = %f", x->system->getDt());
    post("G = %f", x->system->getG());
    post("pos_damping = %f", x->system->getPosDamping());
    post("vel_damping = %f", x->system->getVelDamping());
    post("softening = %f", x->system->getSoftening());
    post("scale = %.3f", x->expand_scale);
    post("speed = %.3f", static_cast<float>(x->internal_steps) / 50.0f);
    post("limits = %.3f", static_cast<float>(x->limits == 0 ? 0 : 1));

    post("[g] --- Initial body values ---");
    for (int i = 0; i < x->system->BodyCount; ++i)
    {
        const Body &b = x->system->getInitBody(i);
        post("[g] >>> body[%d]: x:%.3f y:%.3f vx:%.3f vy:%.3f m:%.3f", i, b.x, b.y, b.vx, b.vy, b.mass);
    }

    post("[g] --- Current body values ---");
    for (int i = 0; i < x->system->BodyCount; ++i)
    {
        const Body &b = x->system->getBody(i);
        post("[g] >>> body[%d]: x:%.3f y:%.3f vx:%.3f vy:%.3f ax:%.3f ay:%.3f m:%.3f", i, b.x, b.y, b.vx, b.vy, b.ax, b.ay, b.mass);
    }
    const Body &hole = x->system->getBlackHole();
    post("[g] --- Current black hole values ---");
    post("[g] >>> x:%f y:%f mass:%f", hole.x, hole.y, hole.mass);
}

// Set individual body parameters from message: [body index x y vx vy mass(
void grav_body(t_grav *x, t_symbol *, int argc, t_atom *argv)
{
    if (argc != 6 || argv[0].a_type != A_FLOAT)
        return;

    int index = static_cast<int>(atom_getfloat(argv));

    if (index < 0 || index >= 10)
        return;

    float px = atom_getfloat(argv + 1);
    float py = atom_getfloat(argv + 2);
    float vx = atom_getfloat(argv + 3);
    float vy = atom_getfloat(argv + 4);
    float mass = atom_getfloat(argv + 5);

    x->system->setBody(index, px, py, vx, vy, mass);
}

// Changes a bodies mass at simulation time [mass index m(
void grav_mass(t_grav *x, t_symbol *, int argc, t_atom *argv)
{
    if (argc != 2 || argv[0].a_type != A_FLOAT)
        return;

    int index = static_cast<int>(atom_getfloat(argv));
    float mass = atom_getfloat(argv + 1);

    if (index < 0 || index >= 10)
        return;

    x->system->setBodyMass(index, mass);
}

void grav_hole(t_grav *x, t_symbol *, int argc, t_atom *argv)
{
    if (argc != 3 || argv[0].a_type != A_FLOAT)
        return;

    float posx = atom_getfloat(argv);
    float posy = atom_getfloat(argv + 1);
    float mass = atom_getfloat(argv + 2);

    x->system->setBlackHole(posx, posy, mass);
}

// Internal DSP loop: advances simulation if time has passed
static void grav_tick(t_grav *x)
{
    if (!x->running_thread.load())
        return;

    grav_out(x);

    clock_delay(x->clock, x->timestep_ms);
}

// Worker thread executes simulation
void simulate_thread(t_grav *x)
{
    using clock = std::chrono::steady_clock;
    auto next_time = clock::now();

    while (x->running_thread.load())
    {
        for (int i = 0; i < x->internal_steps; ++i)
            x->system->simulate();

        // Fill write buffer with simulation results
        x->buffer_write = x->system->getBodies();
        x->hole_write = x->system->getBlackHole();

        // Swap write and read buffers
        {
            std::lock_guard<std::mutex> lock(x->swap_mutex);
            std::swap(x->buffer_read, x->buffer_write);
            std::swap(x->hole_read, x->hole_write);
        }

        next_time += std::chrono::milliseconds(static_cast<int>(x->timestep_ms));

        while (clock::now() < next_time)
        {
            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// Start simulation loop
void grav_start(t_grav *x)
{
    if (x->running_thread.load())
        return;

    x->running_thread = true;
    x->last_output = std::chrono::steady_clock::now();
    x->worker = std::thread(simulate_thread, x);

    x->clock = clock_new(x, (t_method)grav_tick);
    clock_delay(x->clock, x->timestep_ms);
}

// Stop simulation loop
void grav_stop(t_grav *x)
{
    x->running_thread = false;
    if (x->worker.joinable())
        x->worker.join();

    if (x->clock)
        clock_free(x->clock);
}

// Creates new instance of the PD object
void *grav_new()
{
    t_grav *x = reinterpret_cast<t_grav *>(pd_new(grav_class));

    x->out_bang = outlet_new(&x->x_obj, &s_bang);       // bang on finish
    x->out_pos = outlet_new(&x->x_obj, &s_list);        // outlet 2 for positions
    x->out_vel = outlet_new(&x->x_obj, &s_list);        // outlet 3 for velocities
    x->out_acc = outlet_new(&x->x_obj, &s_list);        // outlet 4 for accelerations
    x->out_hole = outlet_new(&x->x_obj, &s_list);       // outlet 5 for black hole
    x->out_params = outlet_new(&x->x_obj, &s_list);     // outlet 6 for current simulations parameters
    x->out_initvalues = outlet_new(&x->x_obj, &s_list); // outlet 7 for body current initialization values

    x->timestep_ms = 10.0;
    x->internal_steps = 1;
    x->expand_scale = 1;
    x->limit_max = 100;
    x->running = false;
    x->limits = false;
    x->system = new Gravity();
    return x;
}

// Destructor: frees simulation object
void grav_free(t_grav *x)
{
    // Ensure thread is stopped before deleting the system
    x->running_thread = false;
    if (x->worker.joinable())
        x->worker.join();

    delete x->system;
    x->system = nullptr;
}

// Setup function: called when external is loaded by PD
extern "C" void grav_setup(void)
{
    grav_class = class_new(gensym("grav"),
                        reinterpret_cast<t_newmethod>(grav_new),
                        reinterpret_cast<t_method>(grav_free),
                        sizeof(t_grav),
                        CLASS_DEFAULT, A_NULL);

    CLASS_MAINSIGNALIN(grav_class, t_grav, x_f);
    class_addbang(grav_class, reinterpret_cast<t_method>(grav_bang));
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_body), gensym("body"), A_GIMME, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_mass), gensym("mass"), A_GIMME, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_hole), gensym("hole"), A_GIMME, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_dt), gensym("dt"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_G), gensym("G"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_posdamp), gensym("posdamp"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_veldamp), gensym("veldamp"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_softening), gensym("softening"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_vmin), gensym("vmin"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_vmax), gensym("vmax"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_preset), gensym("preset"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_dump), gensym("dump"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_count), gensym("count"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_reset), gensym("reset"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_start), gensym("start"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_outparams), gensym("params"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_outinit), gensym("init"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_stop), gensym("stop"), A_NULL);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_speed), gensym("speed"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_scale), gensym("scale"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_limits), gensym("limits"), A_FLOAT, 0);
    class_addmethod(grav_class, reinterpret_cast<t_method>(grav_nudge), gensym("nudge"), A_NULL);
}
